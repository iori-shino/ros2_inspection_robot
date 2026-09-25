"""Generate repeatable temperature readings when a patrol point is reached."""

import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import ParameterException
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import String

from inspection_robot.message_utils import decode_message, encode_message


class InspectionDetector(Node):
    """Simulate one inspection per run and point, caching results in memory."""

    def __init__(self):
        super().__init__("inspection_detector")
        self._temperatures_c, self._threshold_c = self._load_parameters()
        self._results = {}
        self._result_publisher = self.create_publisher(
            String, "/inspection/result", 10
        )
        self._status_subscription = self.create_subscription(
            String, "/inspection/status", self._on_status, 10
        )
        self.get_logger().info(
            f"Detector ready; threshold={self._threshold_c:.1f} C; "
            "waiting for INSPECTING status."
        )

    def _load_parameters(self):
        defaults = {
            "point_ids": ["A", "B", "C"],
            "temperatures_c": [35.0, 85.0, 42.0],
            "temperature_threshold_c": 80.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(
                name, default, descriptor=ParameterDescriptor(read_only=True)
            )

        point_ids = self.get_parameter("point_ids").value
        temperatures = self.get_parameter("temperatures_c").value
        threshold = self.get_parameter("temperature_threshold_c").value
        if not point_ids or any(not point.strip() for point in point_ids):
            raise ValueError("point_ids must contain non-empty point names.")
        if len(set(point_ids)) != len(point_ids):
            raise ValueError("point_ids must not contain duplicates.")
        if len(point_ids) != len(temperatures):
            raise ValueError("point_ids and temperatures_c must have the same length.")
        if not all(math.isfinite(value) for value in temperatures):
            raise ValueError("temperatures_c must contain finite numbers.")
        if not math.isfinite(threshold):
            raise ValueError("temperature_threshold_c must be finite.")
        return dict(zip(point_ids, temperatures)), threshold

    def _on_status(self, msg):
        try:
            status = decode_message(msg.data, "status")
        except ValueError as error:
            self.get_logger().warning(f"Ignored invalid status: {error}")
            return

        if status["state"] != "INSPECTING":
            return
        point_id = status["point_id"]
        if point_id not in self._temperatures_c:
            self.get_logger().warning(f"Unknown inspection point: {point_id}")
            return

        key = (status["run_id"], point_id)
        if key not in self._results:
            temperature = self._temperatures_c[point_id]
            result = {
                "run_id": status["run_id"],
                "point_id": point_id,
                "timestamp": time.time(),
                "temperature_c": temperature,
                "threshold_c": self._threshold_c,
                "is_abnormal": temperature > self._threshold_c,
            }
            self._results[key] = encode_message(result, "result")
            summary = (
                f"run={status['run_id']}, point={point_id}, "
                f"temperature={temperature:.1f} C, "
                f"abnormal={result['is_abnormal']}"
            )
            if result["is_abnormal"]:
                self.get_logger().warning(summary)
            else:
                self.get_logger().info(summary)

        self._result_publisher.publish(String(data=self._results[key]))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = InspectionDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ValueError, ParameterException) as error:
        get_logger("inspection_detector").error(f"Configuration error: {error}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
