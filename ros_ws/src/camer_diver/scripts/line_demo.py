#!/usr/bin/env python3
"""Synthetic test source, never a substitute for real-camera validation."""
import math
import signal

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Demo(Node):
    def __init__(self):
        super().__init__("line_demo")
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, "image_raw", qos_profile_sensor_data)
        self.counter = 0
        self.create_timer(1 / 15, self.publish)
        self.get_logger().warn("SYNTHETIC DEMO: generated test frames, not a connected camera")

    def publish(self):
        t = self.counter / 15
        image = np.full((480, 640, 3), (200, 214, 224), dtype=np.uint8)
        phase = int(t / 6) % 3
        if phase != 2:
            points = []
            for y in range(479, 90, -3):
                x = 320 + 45 * math.sin(t / 3) + 90 * ((479 - y) / 389) ** 2
                points.append((int(x), y))
            cv2.polylines(image, [np.array(points, dtype=np.int32)], False, (220, 80, 20), 30)
            if phase == 1:
                image[280:324] = (200, 214, 224)  # deliberate long gap
        cv2.putText(image, "SYNTHETIC DEMO", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 180), 2)
        message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "synthetic_demo"
        self.publisher.publish(message)
        self.counter += 1


def main():
    rclpy.init()
    node = Demo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A launch process can relay another SIGINT while the node is cleaning up.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
