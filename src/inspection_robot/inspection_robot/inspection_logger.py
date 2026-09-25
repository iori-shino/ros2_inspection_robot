"""Save validated patrol events as JSON Lines, deduplicating within a session."""

import json
from pathlib import Path
import time
from uuid import uuid4

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import ParameterException
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import String

from inspection_robot.message_utils import decode_message


class InspectionLogger(Node):
    """Record the first status per state and first result per run and point."""

    def __init__(self):
        super().__init__("inspection_logger")
        self.declare_parameter(
            "output_dir",
            "~/ros2_inspection_robot/logs",
            descriptor=ParameterDescriptor(read_only=True),
        )
        output_dir = self.get_parameter("output_dir").value
        if not output_dir.strip():
            raise ValueError("output_dir must be a non-empty directory path.")
        directory = Path(output_dir).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        filename = (
            f"inspection_{time.strftime('%Y%m%d_%H%M%S')}_"
            f"{uuid4().hex[:8]}.jsonl"
        )
        self._log_path = directory / filename
        # Fail rather than overwrite an existing session's records.
        with self._log_path.open("x", encoding="utf-8"):
            pass
        self._seen_keys = set()
        self._status_subscription = self.create_subscription(
            String, "/inspection/status", self._on_status, 10
        )
        self._result_subscription = self.create_subscription(
            String, "/inspection/result", self._on_result, 10
        )
        self.get_logger().info(f"Logger ready; file={self._log_path}")

    def _on_status(self, msg):
        self._record_message(msg, "status")

    def _on_result(self, msg):
        self._record_message(msg, "result")

    def _record_message(self, msg, kind):
        try:
            data = decode_message(msg.data, kind)
        except ValueError as error:
            self.get_logger().warning(f"Ignored invalid {kind}: {error}")
            return

        key = (kind, data["run_id"], data["point_id"])
        if kind == "status":
            # Heartbeats change timestamp/pose; retain only the first event.
            key += (data["state"],)
        if key in self._seen_keys:
            return

        record = {"kind": kind, "received_at": time.time(), "data": data}
        line = json.dumps(record, ensure_ascii=False, allow_nan=False)
        # Closing after each event flushes Python's buffer. Disk errors
        # propagate to main and stop the logger with a nonzero exit code.
        with self._log_path.open("a", encoding="utf-8") as stream:
            stream.write(line + "\n")
        self._seen_keys.add(key)

        if kind == "result":
            self.get_logger().info(
                f"Saved result: run={data['run_id']}, "
                f"point={data['point_id']}, abnormal={data['is_abnormal']}"
            )
        elif data["state"] in ("COMPLETED", "ERROR"):
            self.get_logger().info(
                f"Saved {data['state']}: run={data['run_id']}; "
                f"file={self._log_path}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = InspectionLogger()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ValueError, ParameterException) as error:
        get_logger("inspection_logger").error(f"Configuration error: {error}")
        return 1
    except OSError as error:
        get_logger("inspection_logger").error(f"Log file error: {error}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
