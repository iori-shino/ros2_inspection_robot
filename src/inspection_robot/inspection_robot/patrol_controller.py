"""Visit inspection points in order and collect their detection results."""

import math
import time
from uuid import uuid4

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import ParameterException
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose

from inspection_robot.message_utils import decode_message, encode_message
from inspection_robot.patrol_logic import compute_target_error


class PatrolController(Node):
    """Coordinate a sequential patrol and handle inspection results."""

    def __init__(self):
        super().__init__("patrol_controller")
        self._load_parameters()
        self._point_index = 0
        self._abnormal_points = 0
        self._point_id, self._target_x, self._target_y = self._waypoints[0]
        self._run_id = uuid4().hex
        self._state = "WAITING"
        self._detail = ""
        self._pose = None
        self._last_pose_time = None
        self._movement_start_time = None
        self._inspection_start_time = None
        self._velocity_publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )
        self._status_publisher = self.create_publisher(
            String, "/inspection/status", 10
        )
        self._pose_subscription = self.create_subscription(
            Pose, "/turtle1/pose", self._on_pose, 10
        )
        self._result_subscription = self.create_subscription(
            String, "/inspection/result", self._on_result, 10
        )
        self._control_timer = self.create_timer(0.1, self._control_loop)
        self._status_timer = self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"Controller ready; points={len(self._waypoints)}; "
            f"max_linear_speed={self._max_linear_speed:g}; "
            f"movement_timeout={self._movement_timeout_sec:g} s."
        )
        self._set_state(
            "WAITING", f"run={self._run_id}; waiting for /turtle1/pose."
        )

    def _load_parameters(self):
        """Read and validate startup settings before creating ROS callbacks."""
        defaults = {
            "point_ids": ["A", "B", "C"],
            "target_xs": [8.0, 2.0, 2.0],
            "target_ys": [8.0, 8.0, 2.0],
            "max_linear_speed": 1.0,
            "max_angular_speed": 2.0,
            "linear_gain": 0.8,
            "angular_gain": 2.0,
            "distance_tolerance": 0.1,
            "heading_tolerance": 0.1,
            "pose_timeout_sec": 1.0,
            "movement_timeout_sec": 60.0,
            "inspection_timeout_sec": 5.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(
                name, default, descriptor=ParameterDescriptor(read_only=True)
            )
        values = {name: self.get_parameter(name).value for name in defaults}

        point_ids = values["point_ids"]
        target_xs = values["target_xs"]
        target_ys = values["target_ys"]
        if not point_ids or any(not point.strip() for point in point_ids):
            raise ValueError("point_ids must contain non-empty point names.")
        if len(set(point_ids)) != len(point_ids):
            raise ValueError("point_ids must not contain duplicates.")
        if not (len(point_ids) == len(target_xs) == len(target_ys)):
            raise ValueError(
                "point_ids, target_xs and target_ys must have the same length."
            )
        for name in ("target_xs", "target_ys"):
            if not all(math.isfinite(value) for value in values[name]):
                raise ValueError(f"{name} must contain finite numbers.")
        for name, default in defaults.items():
            if isinstance(default, float):
                value = values[name]
                if not math.isfinite(value) or value <= 0.0:
                    raise ValueError(f"{name} must be finite and greater than 0.")
        if values["heading_tolerance"] > math.pi:
            raise ValueError("heading_tolerance must not exceed pi radians.")

        self._waypoints = list(zip(point_ids, target_xs, target_ys))
        self._max_linear_speed = values["max_linear_speed"]
        self._max_angular_speed = values["max_angular_speed"]
        self._linear_gain = values["linear_gain"]
        self._angular_gain = values["angular_gain"]
        self._distance_tolerance = values["distance_tolerance"]
        self._heading_tolerance = values["heading_tolerance"]
        self._pose_timeout_sec = values["pose_timeout_sec"]
        self._movement_timeout_sec = values["movement_timeout_sec"]
        self._inspection_timeout_sec = values["inspection_timeout_sec"]

    def _on_pose(self, msg):
        self._pose = msg
        self._last_pose_time = time.monotonic()

    def _on_result(self, msg):
        try:
            result = decode_message(msg.data, "result")
        except ValueError as error:
            self.get_logger().warning(f"Ignored invalid result: {error}")
            return
        if self._state != "INSPECTING":
            return
        if (
            result["run_id"] != self._run_id
            or result["point_id"] != self._point_id
        ):
            return
        if (
            time.monotonic() - self._inspection_start_time
            > self._inspection_timeout_sec
        ):
            self._fail(
                "No inspection result within "
                f"{self._inspection_timeout_sec:g} seconds."
            )
            return

        self.stop()
        summary = (
            f"Point {self._point_id} inspected: "
            f"temperature={result['temperature_c']:.1f} C, "
            f"abnormal={result['is_abnormal']}"
        )
        if result["is_abnormal"]:
            self._abnormal_points += 1
            self.get_logger().warning(summary)
        else:
            self.get_logger().info(summary)

        if self._point_index + 1 == len(self._waypoints):
            self._set_state(
                "COMPLETED",
                f"All {len(self._waypoints)} points inspected; "
                f"abnormal points={self._abnormal_points}.",
            )
        else:
            self._point_index += 1
            self._start_current_point()

    def _start_current_point(self):
        point_id, target_x, target_y = self._waypoints[self._point_index]
        self._point_id = point_id
        self._target_x = target_x
        self._target_y = target_y
        self._movement_start_time = time.monotonic()
        self._inspection_start_time = None
        self._set_state(
            "MOVING", f"Point {point_id}: ({target_x}, {target_y})"
        )

    def _control_loop(self):
        if self._state in ("COMPLETED", "ERROR") or self._pose is None:
            self.stop()
            return

        now = time.monotonic()
        if now - self._last_pose_time > self._pose_timeout_sec:
            self._fail(
                f"No fresh pose for more than {self._pose_timeout_sec:g} seconds."
            )
            return

        if self._state == "INSPECTING":
            self.stop()
            if now - self._inspection_start_time > self._inspection_timeout_sec:
                self._fail(
                    "No inspection result within "
                    f"{self._inspection_timeout_sec:g} seconds."
                )
            return

        if self._state == "WAITING":
            self._start_current_point()

        distance, heading_error = compute_target_error(
            self._pose.x, self._pose.y, self._pose.theta,
            self._target_x, self._target_y,
        )
        if distance <= self._distance_tolerance:
            self.stop()
            self._inspection_start_time = now
            self._set_state(
                "INSPECTING",
                f"Point {self._point_id} reached; waiting for detection.",
            )
            return

        if now - self._movement_start_time > self._movement_timeout_sec:
            self._fail(
                "Target was not reached within "
                f"{self._movement_timeout_sec:g} seconds."
            )
            return

        command = Twist()
        command.angular.z = max(
            -self._max_angular_speed,
            min(self._max_angular_speed, self._angular_gain * heading_error),
        )
        if abs(heading_error) <= self._heading_tolerance:
            command.linear.x = min(
                self._max_linear_speed, self._linear_gain * distance
            )
        self._velocity_publisher.publish(command)

    def _publish_status(self):
        pose = self._pose
        status = {
            "run_id": self._run_id,
            "point_id": self._point_id,
            "timestamp": time.time(),
            "state": self._state,
            "x": pose.x if pose is not None else None,
            "y": pose.y if pose is not None else None,
            "theta": pose.theta if pose is not None else None,
            "detail": self._detail,
        }
        self._status_publisher.publish(
            String(data=encode_message(status, "status"))
        )

    def _set_state(self, state, detail):
        self._state = state
        self._detail = detail
        if state == "ERROR":
            self.get_logger().error(f"[{state}] {detail}")
        else:
            self.get_logger().info(f"[{state}] {detail}")
        self._publish_status()

    def stop(self):
        self._velocity_publisher.publish(Twist())

    def _fail(self, reason):
        self.stop()
        self._set_state("ERROR", reason)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PatrolController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ValueError, ParameterException) as error:
        get_logger("patrol_controller").error(f"Configuration error: {error}")
        return 1
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
