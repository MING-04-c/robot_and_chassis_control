#!/usr/bin/env python3
"""Publish, display, and capture JPEG frames received from the camera TCP node."""

import os
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_srvs.srv import Trigger


WINDOW_TITLE = "Computer Pi Camera"


def decode_image(data):
    encoded = np.frombuffer(data, dtype=np.uint8)
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


class CameraViewer(Node):
    def __init__(self):
        super().__init__("computer_pi_camera_viewer")
        compressed_topic = self.declare_parameter(
            "compressed_topic", "/computer_pi/camera/image/compressed"
        ).value
        image_topic = self.declare_parameter(
            "image_topic", "/computer_pi/camera/image"
        ).value
        capture_service = self.declare_parameter(
            "capture_service", "/computer_pi/camera/capture"
        ).value
        self.snapshot_directory = Path(
            self.declare_parameter(
                "snapshot_directory",
                str(Path.home() / "Pictures" / "computer_pi_camera"),
            ).value
        ).expanduser()
        self.show_image = self.declare_parameter("show_image", True).value

        self.latest_image = None
        self.latest_jpeg = None
        self.first_frame_logged = False
        self.frame_count = 0
        self.frame_times = deque(maxlen=30)
        self.source_encoding = "unknown"

        self.image_publisher = self.create_publisher(
            Image, image_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            compressed_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.capture_server = self.create_service(
            Trigger, capture_service, self.capture
        )
        self.get_logger().info(
            f"Image topic: {image_topic}; capture service: {capture_service}"
        )

    def on_image(self, message):
        image = decode_image(message.data)
        if image is None:
            self.get_logger().warn("Received a JPEG frame that could not be decoded")
            return

        self.latest_image = image
        self.latest_jpeg = bytes(message.data)
        self.frame_count += 1
        self.frame_times.append(time.monotonic())

        marker = "source="
        if marker in message.format:
            self.source_encoding = message.format.split(marker, 1)[1].split(";", 1)[0]

        if self.image_publisher.get_subscription_count() > 0:
            height, width = image.shape[:2]
            raw_message = Image()
            raw_message.header.stamp = message.header.stamp
            raw_message.header.frame_id = message.header.frame_id
            raw_message.height = height
            raw_message.width = width
            raw_message.encoding = "bgr8"
            raw_message.is_bigendian = 0
            raw_message.step = width * 3
            raw_message.data = image.tobytes()
            self.image_publisher.publish(raw_message)

        if not self.first_frame_logged:
            self.get_logger().info(
                f"Receiving camera frames: {image.shape[1]}x{image.shape[0]}"
            )
            self.first_frame_logged = True

    def capture(self, request, response):
        del request
        if self.latest_jpeg is None:
            response.success = False
            response.message = "No camera frame has been received yet"
            return response

        try:
            self.snapshot_directory.mkdir(parents=True, exist_ok=True)
            now = datetime.now()
            filename = (
                f"camera_{now.strftime('%Y%m%d_%H%M%S')}_"
                f"{now.microsecond // 1000:03d}.jpg"
            )
            output_path = self.snapshot_directory / filename
            temporary_path = output_path.with_name(output_path.name + ".tmp")
            temporary_path.write_bytes(self.latest_jpeg)
            os.replace(temporary_path, output_path)
            response.success = True
            response.message = str(output_path)
            self.get_logger().info(f"Captured camera frame: {output_path}")
        except OSError as exc:
            response.success = False
            response.message = f"Failed to save snapshot: {exc}"
            self.get_logger().error(response.message)
        return response

    def display_image(self, waiting):
        if self.latest_image is None:
            return waiting
        display = self.latest_image.copy()
        fps = 0.0
        if len(self.frame_times) > 1:
            elapsed = self.frame_times[-1] - self.frame_times[0]
            if elapsed > 0:
                fps = (len(self.frame_times) - 1) / elapsed
        height, width = display.shape[:2]
        label = (
            f"FPS {fps:5.1f}  Frames {self.frame_count}  "
            f"{width}x{height}  {self.source_encoding}"
        )
        scale = max(0.65, min(width, height) / 1000.0)
        thickness = max(1, round(scale * 2))
        (text_width, text_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness
        )
        cv2.rectangle(
            display,
            (8, 8),
            (min(width - 1, text_width + 28), text_height + baseline + 24),
            (0, 0, 0),
            cv2.FILLED,
        )
        cv2.putText(
            display,
            label,
            (18, text_height + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (0, 255, 0),
            thickness,
            cv2.LINE_AA,
        )
        return display


def main():
    rclpy.init()
    node = CameraViewer()
    show_window = node.show_image
    if show_window and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        node.get_logger().warn(
            "No graphical display available; continuing without the camera window"
        )
        show_window = False

    waiting = None
    if show_window:
        cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(WINDOW_TITLE, 960, 720)
        waiting = np.zeros((720, 960, 3), dtype=np.uint8)
        cv2.putText(
            waiting,
            "Waiting for camera...",
            (90, 360),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (230, 230, 230),
            2,
        )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01 if show_window else 0.1)
            if show_window:
                cv2.imshow(WINDOW_TITLE, node.display_image(waiting))
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    show_window = False
                    cv2.destroyAllWindows()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
