#!/usr/bin/env python3
"""Display images published by the Hik camera driver."""

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraViewNode(Node):
    def __init__(self):
        super().__init__("hik_camera_view")
        topic = self.declare_parameter("image_topic", "/camera/image").value
        self.bridge = CvBridge()
        self.window_name = "Hik camera: " + topic
        self.window_width = self.declare_parameter("window_width", 960).value
        self.window_height = self.declare_parameter("window_height", 720).value
        if self.window_width <= 0 or self.window_height <= 0:
            raise ValueError("Window width and height must be positive")
        self.window_created = False
        self.subscription = self.create_subscription(
            Image, topic, self.show_image, qos_profile_sensor_data
        )
        self.get_logger().info("Displaying images from %s" % topic)

    def show_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error("Image conversion failed: %s" % exc)
            return
        if not self.window_created:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
            self.window_created = True
        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = CameraViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
