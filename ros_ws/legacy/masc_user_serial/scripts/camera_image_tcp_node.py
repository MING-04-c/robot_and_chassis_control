#!/usr/bin/env python3
"""Stream ROS images as JPEG or uncompressed pixels over direct TCP."""

import ipaddress
import json
import socketserver
import struct
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

MAGIC = b"MIMG"
JPEG_PROTOCOL_VERSION = 1
RAW_PROTOCOL_VERSION = 2
DEFAULT_BIND_IP = "169.254.199.100"
DEFAULT_ALLOWED_CLIENT_IP = "169.254.199.250"


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

    def change_client_count(self, delta):
        with self.clients_lock:
            self.client_count += delta

    def has_clients(self):
        with self.clients_lock:
            return self.client_count > 0


class ImageClientHandler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(5.0)
        self.server.change_client_count(1)
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
                last_sequence = sequence
        finally:
            self.server.change_client_count(-1)


class CameraImageTcpNode(Node):
    def __init__(self):
        super().__init__("camera_image_tcp_node")
        self.image_topic = self.declare_parameter("image_topic", "/camera/image").value
        listen_ip = self.declare_parameter("listen_ip", DEFAULT_BIND_IP).value
        allowed_client_ip = self.declare_parameter(
            "allowed_client_ip", DEFAULT_ALLOWED_CLIENT_IP
        ).value
        listen_port = self.declare_parameter("listen_port", 9002).value
        self.max_fps = self.declare_parameter("max_fps", 60.0).value
        self.image_format = self.declare_parameter("image_format", "jpeg").value
        self.jpeg_quality = self.declare_parameter("jpeg_quality", 80).value
        for parameter_name, address in (
            ("listen_ip", listen_ip), ("allowed_client_ip", allowed_client_ip)
        ):
            try:
                parsed = ipaddress.IPv4Address(address)
            except ipaddress.AddressValueError as exc:
                raise ValueError("%s must be an IPv4 address" % parameter_name) from exc
            if parsed.is_unspecified or parsed.is_multicast or parsed == ipaddress.IPv4Address("255.255.255.255"):
                raise ValueError("%s must be a single host IPv4 address" % parameter_name)
        if not 1 <= listen_port <= 65535:
            raise ValueError("listen_port must be between 1 and 65535")
        if self.image_format not in ("jpeg", "raw"):
            raise ValueError("image_format must be jpeg or raw")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        if self.max_fps <= 0:
            raise ValueError("max_fps must be positive")

        self.bridge = CvBridge()
        self.frames = LatestFrame()
        self.sequence = 0
        self.last_publish_time = 0.0
        self.server = ImageTcpServer(
            (listen_ip, listen_port), self.frames, allowed_client_ip
        )
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            "%s image TCP stream listening on %s:%d, allowing %s, topic %s, max %.1f fps"
            % (self.image_format, listen_ip, listen_port, allowed_client_ip, self.image_topic, self.max_fps)
        )

    def on_image(self, msg):
        if not self.server.has_clients():
            return
        now = time.monotonic()
        if now - self.last_publish_time < 1.0 / self.max_fps:
            return
        if self.image_format == "raw":
            # Preserve Image.data exactly, including row padding described by step.
            payload = bytes(msg.data)
            if msg.width <= 0 or msg.height <= 0 or msg.step <= 0 or len(payload) != msg.step * msg.height:
                self.get_logger().error("Invalid ROS Image dimensions or data length")
                return
            header = {
                "version": RAW_PROTOCOL_VERSION,
                "type": "image",
                "sequence": self.sequence + 1,
                "stamp_ns": msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
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
                "stamp_ns": msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                "frame_id": msg.header.frame_id,
                "topic": self.image_topic,
                "width": int(frame.shape[1]),
                "height": int(frame.shape[0]),
                "source_encoding": msg.encoding,
                "encoding": "jpeg",
                "payload_length": len(payload),
            }
        self.sequence += 1
        header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
        packet = MAGIC + struct.pack("!I", len(header_bytes)) + header_bytes + payload
        self.frames.publish(self.sequence, packet)
        self.last_publish_time = now

    def close(self):
        self.frames.close()
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join()


def main():
    rclpy.init()
    node = None
    try:
        node = CameraImageTcpNode()
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
