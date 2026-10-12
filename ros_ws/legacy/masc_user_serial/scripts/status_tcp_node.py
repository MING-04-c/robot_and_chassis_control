#!/usr/bin/env python3
"""MASC protocol v1 server using one ``task`` field for chassis commands."""

import ipaddress
import json
import socket
import socketserver
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from masc_chassis_msgs.msg import ChassisStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, String


PROTOCOL_VERSION = 1
STATE_NAMES = {0: "idle", 1: "running", 2: "completed", 3: "error"}
TERMINAL_STATES = {2, 3}
CONTROL_COMMANDS = {"execute_task", "stop"}


class StatusTcpServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, node, allowed_client_ip):
        self.node = node
        self.allowed_client_ip = allowed_client_ip
        super().__init__(address, StatusClientHandler)

    def verify_request(self, request, client_address):
        allowed = client_address[0] == self.allowed_client_ip
        if not allowed:
            self.node.get_logger().warning(
                "Rejected status TCP client %s" % client_address[0]
            )
        return allowed


class StatusClientHandler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(2.0)
        self.server.node.add_client(self.request, self.client_address[0])
        buffer = bytearray()
        try:
            while rclpy.ok():
                try:
                    chunk = self.request.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buffer.extend(chunk)
                if len(buffer) > 65536 and b"\n" not in buffer:
                    raise ValueError("status message exceeds 65536 bytes")
                while b"\n" in buffer:
                    line, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    if line.strip():
                        self.server.node.handle_status_line(bytes(line))
        except (OSError, ValueError) as exc:
            self.server.node.get_logger().warning("Status client closed: %s" % exc)
        finally:
            self.server.node.remove_client(self.request)


class StatusTcpNode(Node):
    def __init__(self):
        super().__init__("status_tcp_node")
        listen_ip = self.declare_parameter("listen_ip", "169.254.199.100").value
        allowed_ip = self.declare_parameter(
            "allowed_client_ip", "169.254.199.250"
        ).value
        listen_port = self.declare_parameter("listen_port", 9001).value
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/image"
        ).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/camera_info"
        ).value
        self.source_encoding = self.declare_parameter(
            "source_encoding", "rgb8"
        ).value
        self.image_port = self.declare_parameter("image_port", 9002).value
        self.image_format = self.declare_parameter("image_format", "jpeg").value
        self.jpeg_quality = self.declare_parameter("jpeg_quality", 80).value
        self.image_max_fps = self.declare_parameter("image_max_fps", 60.0).value
        self.camera_id = self.declare_parameter("camera_id", "hik_camera_0").value
        self.chassis_topic = self.declare_parameter(
            "chassis_topic", "/masc_chassis/chassis_status"
        ).value
        self.control_topic = self.declare_parameter(
            "control_topic", "/chassis/control_command"
        ).value
        self.cmd_vel_topic = self.declare_parameter(
            "cmd_vel_topic", "/cmd_vel"
        ).value
        self.test_start_topic = self.declare_parameter(
            "test_start_topic", "/chassis/task_start"
        ).value
        self.target_reached_topic = self.declare_parameter(
            "target_reached_topic", "/target_reached"
        ).value
        self.task_names = list(self.declare_parameter(
            "task_names", ["forward_10cm", "backward_10cm"]).value)
        self.task_delta_x_m = list(self.declare_parameter(
            "task_delta_x_m", [0.10, -0.10]).value)
        self.task_delta_y_m = list(self.declare_parameter(
            "task_delta_y_m", [0.0, 0.0]).value)
        self.task_delta_yaw_rad = list(self.declare_parameter(
            "task_delta_yaw_rad", [0.0, 0.0]).value)
        count = len(self.task_names)
        if (count == 0 or len(set(self.task_names)) != count or
                len(self.task_delta_x_m) != count or
                len(self.task_delta_y_m) != count or
                len(self.task_delta_yaw_rad) != count):
            raise ValueError("task table arrays must have equal non-zero lengths")
        self.task_index = {name: index for index, name in enumerate(self.task_names)}
        self.peer_timeout_sec = float(
            self.declare_parameter("peer_timeout_sec", 3.0).value
        )

        for name, value in (("listen_ip", listen_ip), ("allowed_client_ip", allowed_ip)):
            try:
                address = ipaddress.IPv4Address(value)
            except ipaddress.AddressValueError as exc:
                raise ValueError("%s must be an IPv4 address" % name) from exc
            if address.is_unspecified or address.is_multicast:
                raise ValueError("%s must identify one host" % name)
        if not 1 <= listen_port <= 65535:
            raise ValueError("listen_port must be between 1 and 65535")
        if self.peer_timeout_sec <= 0.0:
            raise ValueError("peer_timeout_sec must be positive")

        self.lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.tcp_clients = set()
        self.sequence = 0
        self.pending_terminal = {}
        self.seen_control_events = {}
        self.task_running = False
        self.active_task = None
        self.current_terminal_event_id = None
        self.last_peer_message = None
        self.supervisor_online = False
        self.last_camera_frame_monotonic = None
        self.camera_window_start = time.monotonic()
        self.camera_window_frames = 0
        self.measured_camera_fps = 0.0
        self.camera_info = {
            "width": 0,
            "height": 0,
            "source_encoding": "",
            "frame_id": "",
        }
        self.chassis_details = {
            "chassis_state": 0,
            "motion_mode": 0,
            "fault_code": 0,
        }
        self.current_status = None
        self.started_monotonic = time.monotonic()

        self.rx_publisher = self.create_publisher(String, "/tcp_rx", 10)
        self.control_publisher = self.create_publisher(String, self.control_topic, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.test_start_publisher = self.create_publisher(
            String, self.test_start_topic, 10
        )
        self.supervisor_online_publisher = self.create_publisher(
            Bool, "/tcp/supervisor_online", 10
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ChassisStatus, self.chassis_topic, self.on_chassis_feedback, 10
        )
        self.create_subscription(
            String, "/chassis_arrival_status", self.on_chassis_task_status, 10
        )
        self.create_subscription(
            Bool, self.target_reached_topic, self.on_target_reached, 10
        )

        self.server = StatusTcpServer((listen_ip, listen_port), self, allowed_ip)
        self.server_thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.server_thread.start()
        self.create_timer(1.0, self.publish_heartbeat)
        self.create_timer(1.0, self.publish_camera_info)
        self.create_timer(0.5, self.publish_status_snapshot)
        self.create_timer(0.5, self.retry_terminal_messages)
        self.create_timer(0.5, self.check_peer_timeout)
        self.get_logger().info(
            "Status TCP listening on %s:%d, allowing %s"
            % (listen_ip, listen_port, allowed_ip)
        )

    def add_client(self, connection, peer_ip):
        with self.lock:
            self.tcp_clients.add(connection)
            self.last_peer_message = time.monotonic()
        self.get_logger().info("Status client connected: %s" % peer_ip)
        self.publish_camera_info()
        self.publish_status_snapshot()
        self.retry_terminal_messages()

    def remove_client(self, connection):
        with self.lock:
            self.tcp_clients.discard(connection)
            no_clients = not self.tcp_clients
        try:
            connection.close()
        except OSError:
            pass
        if no_clients:
            self.set_supervisor_online(False)

    def set_supervisor_online(self, online):
        with self.lock:
            changed = self.supervisor_online != online
            self.supervisor_online = online
        if changed:
            self.supervisor_online_publisher.publish(Bool(data=online))
            self.get_logger().info(
                "Supervisor is %s" % ("online" if online else "offline")
            )

    def mark_peer_seen(self):
        with self.lock:
            self.last_peer_message = time.monotonic()
        self.set_supervisor_online(True)

    def check_peer_timeout(self):
        with self.lock:
            last_message = self.last_peer_message
            online = self.supervisor_online
        if (
            online
            and last_message is not None
            and time.monotonic() - last_message >= self.peer_timeout_sec
        ):
            self.set_supervisor_online(False)

    def next_sequence(self):
        with self.lock:
            self.sequence += 1
            return self.sequence

    def send_message(self, message):
        envelope = {
            "version": PROTOCOL_VERSION,
            "type": message["type"],
            "sequence": self.next_sequence(),
            "timestamp_ns": time.time_ns(),
            "source": "robot",
        }
        envelope.update({key: value for key, value in message.items() if key != "type"})
        packet = (json.dumps(envelope, separators=(",", ":")) + "\n").encode("utf-8")
        with self.lock:
            clients = list(self.tcp_clients)
        dead = []
        with self.send_lock:
            for connection in clients:
                try:
                    connection.sendall(packet)
                except OSError:
                    dead.append(connection)
        for connection in dead:
            self.remove_client(connection)

    def handle_status_line(self, line):
        try:
            message = json.loads(line.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.get_logger().warning("Invalid status JSON: %s" % exc)
            return
        if not isinstance(message, dict):
            self.get_logger().warning("Status message must be a JSON object")
            return
        if message.get("version") != PROTOCOL_VERSION:
            self.get_logger().warning("Unsupported status protocol version")
            return
        message_type = message.get("type")
        if message_type == "ack":
            event_id = message.get("ack_event_id")
            if not isinstance(event_id, str) or not event_id:
                self.get_logger().warning("Rejected invalid terminal ACK")
                return
            self.mark_peer_seen()
            self.handle_terminal_ack(event_id)
            return
        if message_type == "control_command":
            self.handle_control_command(message)
            return
        if message_type != "heartbeat":
            self.get_logger().warning("Unsupported incoming message type: %s" % message_type)
            return
        self.mark_peer_seen()
        ros_message = String()
        ros_message.data = json.dumps(message, ensure_ascii=False)
        self.rx_publisher.publish(ros_message)

    def handle_terminal_ack(self, event_id):
        with self.lock:
            acknowledged = self.pending_terminal.pop(event_id, None)
            finishes_current_task = event_id == self.current_terminal_event_id
            if finishes_current_task:
                self.current_terminal_event_id = None
                self.task_running = False
                self.active_task = None
        if acknowledged is None:
            return
        self.get_logger().info("Terminal chassis status acknowledged: %s" % event_id)
        if finishes_current_task:
            self.publish_chassis_state(
                state=0,
                event_id="status-idle-%d" % time.time_ns(),
                task="",
                progress=0.0,
                message="waiting_for_task",
            )

    def send_ack(self, event_id):
        self.send_message({"type": "ack", "ack_event_id": event_id})

    def handle_control_command(self, message):
        event_id = message.get("event_id")
        command = message.get("command")
        if not isinstance(event_id, str) or not event_id:
            self.get_logger().warning("Rejected control command without event_id")
            return
        if command not in CONTROL_COMMANDS:
            self.get_logger().warning("Rejected unsupported control command: %s" % command)
            return

        task = message.get("task") if command == "execute_task" else None
        if command == "execute_task" and task not in self.task_index:
            self.get_logger().warning("Rejected unknown task: %s" % task)
            return

        fingerprint = (command, task)
        with self.lock:
            previous = self.seen_control_events.get(event_id)
        if previous is not None:
            if previous != fingerprint:
                self.get_logger().warning("Rejected reused event_id with different content")
                return
            self.mark_peer_seen()
            self.send_ack(event_id)
            return
        with self.lock:
            if event_id in self.seen_control_events:
                # Another client thread accepted it after the first lookup.
                previous = self.seen_control_events[event_id]
            else:
                previous = None
                self.seen_control_events[event_id] = fingerprint
                if len(self.seen_control_events) > 4096:
                    oldest = next(iter(self.seen_control_events))
                    self.seen_control_events.pop(oldest, None)
        if previous is not None:
            if previous == fingerprint:
                self.mark_peer_seen()
                self.send_ack(event_id)
            else:
                self.get_logger().warning("Rejected reused event_id with different content")
            return
        self.mark_peer_seen()

        # ACK first, as required by the wire protocol, then trigger ROS actions.
        self.send_ack(event_id)
        normalized = {"event_id": event_id, "command": command}
        if command == "execute_task":
            index = self.task_index[task]
            normalized.update({
                "task": task,
                "delta_x_m": self.task_delta_x_m[index],
                "delta_y_m": self.task_delta_y_m[index],
                "delta_yaw_rad": self.task_delta_yaw_rad[index],
            })
        ros_message = String()
        ros_message.data = json.dumps(normalized, separators=(",", ":"))
        self.control_publisher.publish(ros_message)

        if command == "execute_task":
            with self.lock:
                self.task_running = True
                self.active_task = task
            self.test_start_publisher.publish(String(data=task))
            self.publish_chassis_state(
                state=1,
                event_id="status-%s-running" % task,
                task=task,
                progress=0.0,
                message="task_started",
            )
        elif command == "stop":
            self.cmd_vel_publisher.publish(Twist())
            with self.lock:
                active_task = self.active_task
                self.task_running = False
            self.publish_chassis_state(
                state=3,
                event_id="status-%s-error-%d" % (
                    active_task or "idle", time.time_ns()),
                task=active_task or "",
                message="stopped_by_supervisor",
            )
        self.get_logger().info(
            "Accepted chassis control %s, event_id=%s" % (command, event_id)
        )

    def on_camera_info(self, message):
        now = time.monotonic()
        self.last_camera_frame_monotonic = now
        self.camera_window_frames += 1
        elapsed = now - self.camera_window_start
        if elapsed >= 1.0:
            self.measured_camera_fps = self.camera_window_frames / elapsed
            self.camera_window_frames = 0
            self.camera_window_start = now
        self.camera_info = {
            "width": int(message.width),
            "height": int(message.height),
            "source_encoding": self.source_encoding,
            "frame_id": message.header.frame_id,
        }

    def publish_camera_info(self):
        online = (
            self.last_camera_frame_monotonic is not None
            and time.monotonic() - self.last_camera_frame_monotonic < 2.0
        )
        quality = self.jpeg_quality if self.image_format == "jpeg" else 0
        self.send_message({
            "type": "camera_info",
            "camera_id": self.camera_id,
            "online": online,
            "image_topic": self.image_topic,
            "camera_info_topic": self.camera_info_topic,
            "width": self.camera_info["width"],
            "height": self.camera_info["height"],
            "source_encoding": self.camera_info["source_encoding"],
            "transport_encoding": self.image_format,
            "jpeg_quality": int(quality),
            "configured_max_fps": float(self.image_max_fps),
            "measured_fps": round(self.measured_camera_fps, 2),
            "image_port": int(self.image_port),
        })

    def on_chassis_feedback(self, message):
        self.chassis_details = {
            "chassis_state": int(message.chassis_state),
            "motion_mode": int(message.current_motion_mode),
            "fault_code": int(message.chassis_fault),
            "brake_active": bool(message.brake_active),
            "low_battery": bool(message.low_battery),
            "obstacle_status_flags": int(message.obstacle_status_flags),
        }

    def parse_task_status(self, text):
        try:
            value = json.loads(text)
        except json.JSONDecodeError:
            value = int(text.strip()) if text.strip().isdigit() else None
        if isinstance(value, int):
            value = {"state": value}
        if not isinstance(value, dict):
            raise ValueError("status must be a state number or JSON object")
        state = value.get("state")
        if isinstance(state, bool) or not isinstance(state, int) or state not in STATE_NAMES:
            raise ValueError("state must be 0, 1, 2 or 3")
        task = value.get("task", "")
        if not isinstance(task, str):
            raise ValueError("task must be a string")
        event_id = value.get(
            "event_id", "status-%s-%d" % (task or "idle", time.time_ns())
        )
        if not isinstance(event_id, str) or not event_id:
            raise ValueError("event_id must be a non-empty string")
        result = {
            "event_id": event_id,
            "task": task,
            "state": state,
            "message": value.get("message", ""),
        }
        for key in ("progress", "error_code", "details"):
            if key in value:
                result[key] = value[key]
        if "details" in result and not isinstance(result["details"], dict):
            raise ValueError("details must be a JSON object")
        return result

    def on_chassis_task_status(self, ros_message):
        try:
            status = self.parse_task_status(ros_message.data)
        except (ValueError, TypeError) as exc:
            self.get_logger().error("Invalid chassis_status input: %s" % exc)
            return
        state = status.pop("state")
        task = status.get("task", "")
        with self.lock:
            running = self.task_running
            active_task = self.active_task
        if state == 0:
            if task:
                self.get_logger().error("idle chassis status must use an empty task")
                return
        elif task not in self.task_index or (active_task and task != active_task):
            self.get_logger().error("status task does not match the active task")
            return
        elif state in TERMINAL_STATES and not running:
            self.get_logger().error("Rejected terminal status before a task is running")
            return
        if state == 1:
            with self.lock:
                self.task_running = True
        elif state in TERMINAL_STATES:
            with self.lock:
                self.task_running = False
        self.publish_chassis_state(state=state, **status)

    def on_target_reached(self, message):
        if not message.data:
            return
        with self.lock:
            running = self.task_running
            active_task = self.active_task
        if not running:
            self.get_logger().warning("Ignored target_reached because no task is running")
            return
        self.publish_chassis_state(
            state=2,
            event_id="status-%s-completed-%d" % (active_task, time.time_ns()),
            task=active_task,
            progress=1.0,
            message="target_reached",
        )
        with self.lock:
            self.task_running = False

    def publish_chassis_state(
        self,
        state,
        event_id,
        task,
        message="",
        progress=None,
        error_code=None,
        details=None,
    ):
        status = {
            "type": "chassis_status",
            "event_id": event_id,
            "task": task,
            "state": state,
            "state_name": STATE_NAMES[state],
            "message": message,
            "details": {**self.chassis_details, **(details or {})},
        }
        if progress is not None:
            status["progress"] = progress
        if error_code is not None:
            status["error_code"] = error_code
        with self.lock:
            self.current_status = status
            if state in TERMINAL_STATES:
                if self.current_terminal_event_id is not None:
                    self.pending_terminal.pop(self.current_terminal_event_id, None)
                self.current_terminal_event_id = event_id
                self.pending_terminal[event_id] = status
        self.send_message(status)

    def publish_status_snapshot(self):
        with self.lock:
            message = self.current_status
        if message is not None and message["state"] == 1:
            self.send_message(message)

    def retry_terminal_messages(self):
        with self.lock:
            messages = list(self.pending_terminal.values())
        for message in messages:
            self.send_message(message)

    def publish_heartbeat(self):
        self.send_message({
            "type": "heartbeat",
            "uptime_ms": int((time.monotonic() - self.started_monotonic) * 1000),
            "status": "ok",
        })

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join()
        with self.lock:
            clients = list(self.tcp_clients)
            self.tcp_clients.clear()
        for connection in clients:
            try:
                connection.close()
            except OSError:
                pass


def main():
    rclpy.init()
    node = None
    try:
        node = StatusTcpNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
