#!/usr/bin/env python3
"""Bridge the computer TCP NDJSON protocol to typed ROS status/control APIs."""

import json
import time
import uuid

import rclpy
from eli_cs_robot_serial.srv import ChassisControl
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


STATE_NAMES = {
    0: "idle",
    1: "running",
    2: "completed",
    3: "error",
    4: "offline",
}
STATUS_TYPES = {"chassis_status", "robot_status"}
VALID_TYPES = STATUS_TYPES | {"camera_info", "heartbeat", "ack"}
CONTROL_NAMES = {
    ChassisControl.Request.CONTINUE: "continue",
    ChassisControl.Request.STOP: "stop",
    ChassisControl.Request.EXECUTE_TASK: "execute_task",
}


class ComputerPiStatus(Node):
    def __init__(self):
        super().__init__("computer_pi_status_node")
        rx_topic = self.declare_parameter("rx_topic", "/computer_pi/rx").value
        tx_topic = self.declare_parameter("tx_topic", "/computer_pi/tx").value
        status_topic = self.declare_parameter(
            "status_topic", "/computer_pi/status"
        ).value
        online_topic = self.declare_parameter(
            "online_topic", "/computer_pi/online"
        ).value
        camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/computer_pi/camera_info"
        ).value
        chassis_status_topic = self.declare_parameter(
            "chassis_status_topic", "/computer_pi/chassis_status"
        ).value
        robot_status_topic = self.declare_parameter(
            "robot_status_topic", "/computer_pi/robot_status"
        ).value
        chassis_arrived_topic = self.declare_parameter(
            "chassis_arrived_topic", "/computer_pi/chassis_arrived"
        ).value
        chassis_control_service = self.declare_parameter(
            "chassis_control_service", "/computer_pi/chassis/control"
        ).value
        self.offline_timeout = float(
            self.declare_parameter("offline_timeout", 3.0).value
        )
        heartbeat_period = float(
            self.declare_parameter("heartbeat_period", 1.0).value
        )
        self.source = self.declare_parameter("source", "supervisor").value
        if self.offline_timeout <= 0 or heartbeat_period <= 0:
            raise ValueError("offline_timeout and heartbeat_period must be positive")

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.status_publisher = self.create_publisher(
            String, status_topic, latched_qos
        )
        self.online_publisher = self.create_publisher(
            Bool, online_topic, latched_qos
        )
        self.chassis_arrived_publisher = self.create_publisher(
            Bool, chassis_arrived_topic, latched_qos
        )
        self.camera_info_publisher = self.create_publisher(
            String, camera_info_topic, 10
        )
        self.chassis_status_publisher = self.create_publisher(
            String, chassis_status_topic, 10
        )
        self.robot_status_publisher = self.create_publisher(
            String, robot_status_topic, 10
        )
        self.tx_publisher = self.create_publisher(String, tx_topic, 10)
        self.rx_subscription = self.create_subscription(
            String, rx_topic, self.on_message, 50
        )
        self.chassis_control_server = self.create_service(
            ChassisControl, chassis_control_service, self.send_chassis_control
        )

        self.sequence = 0
        self.started_at = time.monotonic()
        self.last_valid_message = None
        self.is_offline = True
        self.pending_control_commands = {}
        self.publish_connection_state(False, "waiting_for_status_connection")
        self.publish_chassis_arrived(False)
        self.watchdog_timer = self.create_timer(0.2, self.check_offline)
        self.heartbeat_timer = self.create_timer(
            heartbeat_period, self.send_heartbeat
        )
        self.control_retry_timer = self.create_timer(
            0.5, self.retry_control_commands
        )
        self.get_logger().info(
            f"Status/control bridge ready; offline timeout={self.offline_timeout:.1f}s"
        )

    def next_sequence(self):
        self.sequence += 1
        return self.sequence

    @staticmethod
    def compact_json(data):
        return json.dumps(data, ensure_ascii=False, separators=(",", ":"))

    def publish_json(self, publisher, data):
        message = String()
        message.data = self.compact_json(data)
        publisher.publish(message)

    def send_protocol_message(self, data):
        data["sequence"] = self.next_sequence()
        data["timestamp_ns"] = time.time_ns()
        self.publish_json(self.tx_publisher, data)

    def publish_chassis_arrived(self, arrived):
        message = Bool()
        message.data = arrived
        self.chassis_arrived_publisher.publish(message)

    def publish_connection_state(self, online, reason):
        message = {
            "version": 1,
            "type": "connection_status",
            "sequence": self.next_sequence(),
            "timestamp_ns": time.time_ns(),
            "source": self.source,
            "online": online,
            "message": reason,
            "state": 0 if online else 4,
            "state_name": "idle" if online else "offline",
        }
        self.publish_json(self.status_publisher, message)
        online_message = Bool()
        online_message.data = online
        self.online_publisher.publish(online_message)
        if online:
            self.get_logger().info(f"Computer Pi is online: {reason}")
        else:
            self.get_logger().warn(f"Computer Pi is offline: {reason}")

    def validate_message(self, data):
        if not isinstance(data, dict):
            raise ValueError("message is not a JSON object")
        if data.get("version") != 1:
            raise ValueError(f"unsupported version: {data.get('version')}")
        message_type = data.get("type")
        if message_type not in VALID_TYPES:
            raise ValueError(f"unsupported message type: {message_type}")
        for field in ("sequence", "timestamp_ns", "source"):
            if field not in data:
                raise ValueError(f"missing field: {field}")
        if type(data["sequence"]) is not int or type(data["timestamp_ns"]) is not int:
            raise ValueError("sequence and timestamp_ns must be integers")
        if not isinstance(data["source"], str):
            raise ValueError("source must be a string")
        if message_type in STATUS_TYPES:
            state = data.get("state")
            if type(state) is not int or state not in STATE_NAMES:
                raise ValueError(f"invalid state: {state}")
            if data.get("state_name") != STATE_NAMES[state]:
                raise ValueError("state and state_name do not match")
            if not isinstance(data.get("event_id"), str):
                raise ValueError("status event_id must be a string")
            if not isinstance(data.get("task"), str):
                raise ValueError("status task must be a string")
        if message_type == "ack" and not isinstance(data.get("ack_event_id"), str):
            raise ValueError("ack_event_id must be a string")
        return message_type

    def on_message(self, message):
        try:
            data = json.loads(message.data)
            message_type = self.validate_message(data)
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().warn(
                f"Ignoring invalid status message: {exc}; received={message.data[:1000]}",
                throttle_duration_sec=5.0,
            )
            return

        self.last_valid_message = time.monotonic()
        if self.is_offline:
            self.is_offline = False
            self.publish_connection_state(True, "status_connection_restored")

        if message_type == "camera_info":
            self.publish_json(self.camera_info_publisher, data)
        elif message_type == "chassis_status":
            self.publish_json(self.chassis_status_publisher, data)
            self.publish_json(self.status_publisher, data)
            self.publish_chassis_arrived(data["state"] == 2)
        elif message_type == "robot_status":
            self.publish_json(self.robot_status_publisher, data)
            self.publish_json(self.status_publisher, data)
        elif message_type == "ack":
            event_id = data["ack_event_id"]
            if self.pending_control_commands.pop(event_id, None) is not None:
                self.get_logger().info(f"Chassis control acknowledged: {event_id}")
        if message_type in STATUS_TYPES and data["state"] in (2, 3):
            self.send_ack(data["event_id"])

    def send_ack(self, event_id):
        self.send_protocol_message({
            "version": 1,
            "type": "ack",
            "source": self.source,
            "ack_event_id": event_id,
        })

    def send_heartbeat(self):
        self.send_protocol_message({
            "version": 1,
            "type": "heartbeat",
            "source": self.source,
            "uptime_ms": int((time.monotonic() - self.started_at) * 1000),
            "status": "ok",
        })

    def send_chassis_control(self, request, response):
        if self.is_offline:
            response.success = False
            response.message = "status connection is offline"
            return response
        command = CONTROL_NAMES.get(int(request.command))
        if command is None:
            response.success = False
            response.message = "command must be CONTINUE(0), STOP(1), or EXECUTE_TASK(2)"
            return response
        if command == "execute_task" and not request.task:
            response.success = False
            response.message = "task is required for EXECUTE_TASK"
            return response

        command_id = f"control-{uuid.uuid4().hex[:12]}"
        data = {
            "version": 1,
            "type": "control_command",
            "source": self.source,
            "event_id": command_id,
            "command": command,
        }
        if request.task:
            data["task"] = request.task
        self.pending_control_commands[command_id] = data
        self.send_protocol_message(data)
        self.get_logger().info(
            f"Chassis control sent: event_id={command_id} command={command} task={request.task!r}"
        )
        response.success = True
        response.command_id = command_id
        response.message = f"{command} command queued for TCP transmission"
        return response

    def retry_control_commands(self):
        if self.is_offline:
            return
        for data in list(self.pending_control_commands.values()):
            self.send_protocol_message(data)

    def check_offline(self):
        if self.is_offline or self.last_valid_message is None:
            return
        elapsed = time.monotonic() - self.last_valid_message
        if elapsed >= self.offline_timeout:
            self.is_offline = True
            self.publish_chassis_arrived(False)
            self.publish_connection_state(
                False, f"no_valid_status_for_{elapsed:.1f}_seconds"
            )


def main():
    rclpy.init()
    node = ComputerPiStatus()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
