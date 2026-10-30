#!/usr/bin/env python3
"""独立海康拍照 TCP 通道；兼容原 MIMG 主机接收器。"""

from collections import deque
import ipaddress
import hashlib
import json
import signal
import socketserver
import struct
import threading
import time
from uuid import uuid4

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

MAGIC = b"MIMG"
JPEG_PROTOCOL_VERSION = 1
RAW_PROTOCOL_VERSION = 2
DEFAULT_BIND_IP = "0.0.0.0"
DEFAULT_ALLOWED_CLIENT_IP = "169.254.199.120"


class LatestFrame:
    """Keep only the newest frame so slow clients cannot build a queue."""

    def __init__(self):
        self.condition = threading.Condition()
        self.sequence = 0
        self.packet = None
        self.closed = False

    def publish(self, sequence, packet):
        with self.condition:
            self.sequence = sequence
            self.packet = packet
            self.condition.notify_all()

    def wait_after(self, sequence):
        with self.condition:
            while not self.closed and self.sequence <= sequence:
                self.condition.wait()
            if self.closed:
                return None, None
            return self.sequence, self.packet

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


class PhotoQueue(LatestFrame):
    """Retain triggered photos in order until the TCP sender writes each packet."""

    def __init__(self, capacity=8):
        super().__init__()
        self.pending = deque()
        self.capacity = capacity

    def publish(self, sequence, packet):
        with self.condition:
            if len(self.pending) >= self.capacity:
                return False
            self.pending.append((sequence, packet))
            self.condition.notify_all()
            return True

    def wait_after(self, sequence):
        del sequence
        with self.condition:
            while not self.closed and not self.pending:
                self.condition.wait()
            if self.closed:
                return None, None
            return self.pending[0]

    def delivered(self, sequence):
        with self.condition:
            if self.pending and self.pending[0][0] == sequence:
                self.pending.popleft()

    def has_space(self):
        with self.condition:
            return len(self.pending) < self.capacity


class ImageTcpServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, frames, allowed_client_ip):
        self.frames = frames
        self.allowed_client_ip = allowed_client_ip
        self.client_count = 0
        self.clients_lock = threading.Lock()
        super().__init__(address, ImageClientHandler)

    def verify_request(self, request, client_address):
        return client_address[0] == self.allowed_client_ip

    def claim_client(self):
        with self.clients_lock:
            if isinstance(self.frames, PhotoQueue) and self.client_count:
                return False
            self.client_count += 1
            return True

    def change_client_count(self, delta):
        with self.clients_lock:
            self.client_count += delta

    def has_clients(self):
        with self.clients_lock:
            return self.client_count > 0


class ImageClientHandler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(5.0)
        if not self.server.claim_client():
            return
        last_sequence = 0
        try:
            while True:
                sequence, packet = self.server.frames.wait_after(last_sequence)
                if packet is None:
                    break
                try:
                    self.request.sendall(packet)
                except OSError:
                    break
                if isinstance(self.server.frames, PhotoQueue):
                    self.server.frames.delivered(sequence)
                last_sequence = sequence
        finally:
            self.server.change_client_count(-1)


class CameraImageTcpNode(Node):
    def __init__(self):
        super().__init__("hk_image_transfer")
        self.delivery_mode = self.declare_parameter("delivery_mode", "photos").value
        if self.delivery_mode not in ("photos", "stream"):
            raise ValueError("delivery_mode must be photos or stream")
        self.image_topic = self.declare_parameter(
            "image_topic", "/dataset_capture/captured_image"
        ).value
        listen_ip = self.declare_parameter("listen_ip", DEFAULT_BIND_IP).value
        allowed_client_ip = self.declare_parameter(
            "allowed_client_ip", DEFAULT_ALLOWED_CLIENT_IP
        ).value
        listen_port = self.declare_parameter("listen_port", 9002).value
        self.max_fps = self.declare_parameter("max_fps", 60.0).value
        self.image_format = self.declare_parameter("image_format", "jpeg").value
        self.jpeg_quality = self.declare_parameter("jpeg_quality", 80).value
        for parameter_name, address in (
            ("listen_ip", listen_ip),
            ("allowed_client_ip", allowed_client_ip),
        ):
            try:
                parsed = ipaddress.IPv4Address(address)
            except ipaddress.AddressValueError as exc:
                raise ValueError("%s must be an IPv4 address" % parameter_name) from exc
            if (
                (parameter_name == "allowed_client_ip" and parsed.is_unspecified)
                or parsed.is_multicast
                or parsed == ipaddress.IPv4Address("255.255.255.255")
            ):
                raise ValueError(
                    "%s must be a single host IPv4 address" % parameter_name
                )
        if not 1 <= listen_port <= 65535:
            raise ValueError("listen_port must be between 1 and 65535")
        if self.image_format not in ("jpeg", "raw"):
            raise ValueError("image_format must be jpeg or raw")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        if self.max_fps <= 0:
            raise ValueError("max_fps must be positive")

        self.bridge = CvBridge()
        self.frames = PhotoQueue() if self.delivery_mode == "photos" else LatestFrame()
        self.sequence = 0
        self.last_publish_time = 0.0
        self.transfer_session_id = uuid4().hex
        self.server = ImageTcpServer(
            (listen_ip, listen_port), self.frames, allowed_client_ip
        )
        self.server_thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.server_thread.start()
        self.create_service(Trigger, "~/ready", self.on_ready)
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            10 if self.delivery_mode == "photos" else qos_profile_sensor_data,
        )
        self.get_logger().info(
            "%s TCP %s listening on %s:%d, allowing %s, topic %s"
            % (
                self.image_format,
                self.delivery_mode,
                listen_ip,
                listen_port,
                allowed_client_ip,
                self.image_topic,
            )
        )

    def on_ready(self, request, response):
        del request
        response.success = self.server.has_clients() and (
            not isinstance(self.frames, PhotoQueue) or self.frames.has_space()
        )
        response.message = (
            "receiver connected" if response.success else "no image receiver connected"
        )
        return response

    def on_image(self, msg):
        if not self.server.has_clients():
            return
        now = time.monotonic()
        if (
            self.delivery_mode == "stream"
            and now - self.last_publish_time < 1.0 / self.max_fps
        ):
            return
        if self.image_format == "raw":
            # Preserve Image.data exactly, including row padding described by step.
            payload = bytes(msg.data)
            if (
                msg.width <= 0
                or msg.height <= 0
                or msg.step <= 0
                or len(payload) != msg.step * msg.height
            ):
                self.get_logger().error("Invalid ROS Image dimensions or data length")
                return
            header = {
                "version": RAW_PROTOCOL_VERSION,
                "type": "image",
                "sequence": self.sequence + 1,
                "stamp_ns": msg.header.stamp.sec * 1_000_000_000
                + msg.header.stamp.nanosec,
                "frame_id": msg.header.frame_id,
                "topic": self.image_topic,
                "width": int(msg.width),
                "height": int(msg.height),
                "step": int(msg.step),
                "is_bigendian": int(msg.is_bigendian),
                "encoding": msg.encoding,
                "payload_length": len(payload),
            }
        else:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                ok, encoded = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
                )
            except (CvBridgeError, cv2.error) as exc:
                self.get_logger().error("Cannot encode camera frame: %s" % exc)
                return
            if not ok:
                self.get_logger().error("JPEG encoding failed")
                return
            payload = encoded.tobytes()
            header = {
                "version": JPEG_PROTOCOL_VERSION,
                "type": "image",
                "sequence": self.sequence + 1,
                "stamp_ns": msg.header.stamp.sec * 1_000_000_000
                + msg.header.stamp.nanosec,
                "frame_id": msg.header.frame_id,
                "topic": self.image_topic,
                "width": int(frame.shape[1]),
                "height": int(frame.shape[0]),
                "source_encoding": msg.encoding,
                "encoding": "jpeg",
                "payload_length": len(payload),
            }
        header["payload_sha256"] = hashlib.sha256(payload).hexdigest()
        self.sequence += 1
        if self.delivery_mode == "photos":
            # 分组含义明确为“本发送进程内连续两张”，不冒充工作流检测点编号。
            header.update({"transfer_session_id": self.transfer_session_id,
                           "capture_set_id": f"{self.transfer_session_id}_{(self.sequence-1)//2+1:06d}",
                           "capture_index": (self.sequence-1) % 2+1,
                           "grouping": "consecutive_pair"})
        header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
        packet = MAGIC + struct.pack("!I", len(header_bytes)) + header_bytes + payload
        if self.frames.publish(self.sequence, packet) is False:
            self.get_logger().error(
                "Photo send queue is full; photo remains saved locally"
            )
        self.last_publish_time = now

    def close(self):
        self.frames.close()
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join()


def main():
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    previous = {number: signal.signal(number, request_stop)
                for number in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = CameraImageTcpNode()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for number, handler in previous.items():
            signal.signal(number, handler)


if __name__ == "__main__":
    main()
