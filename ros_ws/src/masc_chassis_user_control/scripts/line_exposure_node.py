#!/usr/bin/env python3
"""Adaptive UVC exposure from blue-line confidence and image evidence."""

import json
import math
import time
from dataclasses import replace

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from exposure_backend import V4L2Exposure
from exposure_control import ExposureConfig, ExposurePolicy, image_metrics, quality_from_status


class LineExposureNode(Node):
    def __init__(self, backend=None):
        super().__init__('line_exposure_controller')
        fixed = ParameterDescriptor(read_only=True)
        defaults = ExposureConfig()
        self.config = ExposureConfig(**{
            name: self.declare_parameter(name, value, fixed).value
            for name, value in vars(defaults).items()})
        self.config.validate()
        self.policy = ExposurePolicy(self.config)
        self.enabled = self.declare_parameter('adaptive_enabled', True).value
        device = self.declare_parameter('video_device', '/dev/video0', fixed).value
        override = self.declare_parameter('device_override', '', fixed).value
        self.backend = backend if backend is not None else V4L2Exposure(override or device)
        image_topic = self.declare_parameter('image_topic', '/camera/image_raw', fixed).value
        line_topic = self.declare_parameter('line_status_topic', '/camera/line/status', fixed).value
        self.timeout = self.declare_parameter('input_timeout_sec', 1.0, fixed).value
        if not 0.2 <= self.timeout <= 5:
            raise ValueError('input_timeout_sec must be 0.2..5')
        self.bridge = CvBridge()
        self.metrics = self.quality = None
        self.image_receipt = self.line_receipt = None
        self.last_image_sample = -float('inf')
        self.image_stamp_ns = None
        self.ready = False
        self.failed = False
        self.reason = 'waiting_for_camera'
        self.status_pub = self.create_publisher(String, '/line_follow/exposure_status', 10)
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(String, line_topic, self.on_line, 10)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(0.5, self.tick)
        self.get_logger().info('Adaptive exposure waits for fresh camera/line messages; '
                               f'configured bounds {self.config.exposure_min}..{self.config.exposure_max}')

    def on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'adaptive_enabled':
                self.enabled = parameter.value
                self.policy.bad_checks = self.policy.good_checks = 0
        return SetParametersResult(successful=True)

    def on_image(self, message):
        now = time.monotonic()
        if now - self.last_image_sample < 0.2:
            return
        self.last_image_sample = now
        try:
            self.metrics = image_metrics(self.bridge.imgmsg_to_cv2(message, 'bgr8'), self.config)
            self.image_receipt = now
            self.image_stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        except (CvBridgeError, ValueError) as error:
            self.metrics = None
            self.get_logger().warning(f'Exposure image rejected: {error}')

    def on_line(self, message):
        try:
            self.quality = quality_from_status(json.loads(message.data), self.config)
        except (ValueError, TypeError):
            self.quality = None
        self.line_receipt = time.monotonic()

    def fresh(self, now):
        if any(t is None or now - t > self.timeout for t in (self.image_receipt, self.line_receipt)):
            return False
        if self.image_stamp_ns is None:
            return False
        age = (self.get_clock().now().nanoseconds - self.image_stamp_ns) / 1e9
        return -0.1 <= age <= self.timeout

    def tick(self):
        now = time.monotonic()
        if not self.enabled:
            self.reason = 'disabled'
        elif self.failed:
            self.reason = 'hardware_error'
        elif not self.fresh(now):
            self.reason = 'stale_observation'
            self.policy.bad_checks = 0
        elif self.metrics is None or self.quality is None:
            self.reason = 'waiting_for_observation'
        else:
            try:
                if not self.ready:
                    self.backend.start()
                    lower = max(self.config.exposure_min, self.backend.minimum)
                    upper = min(self.config.exposure_max, self.backend.maximum)
                    step = self.backend.step
                    lower = self.backend.minimum + math.ceil((lower - self.backend.minimum) / step) * step
                    upper = self.backend.minimum + math.floor((upper - self.backend.minimum) / step) * step
                    if lower >= upper:
                        raise RuntimeError('configured exposure interval does not overlap device range')
                    # Config bounds intersect hardware range; camera step is enforced by backend.
                    self.policy = ExposurePolicy(replace(self.config, exposure_min=lower, exposure_max=upper))
                    clamped = max(lower, min(upper, self.backend.current))
                    if clamped != self.backend.current:
                        self.backend.set(clamped)
                    self.ready = True
                    self.policy.last_change = now
                    self.get_logger().info(f'Exposure takeover: manual {self.backend.current}, bounds {lower}..{upper}')
                target, self.reason = self.policy.decide(self.backend.current, self.metrics, self.quality, now)
                if target is not None:
                    previous = self.backend.current
                    actual = self.backend.set(target)
                    self.policy.applied(now)
                    self.get_logger().info(f'Exposure {previous} -> {actual}: {self.reason}')
            except Exception as error:
                self.failed = True
                self.reason = 'hardware_error'
                self.get_logger().error(f'Adaptive exposure disabled after control error: {error}')
        payload = {
            'enabled': self.enabled, 'ready': self.ready and not self.failed,
            'reason': self.reason, 'exposure': self.backend.current,
            'adjustments': self.policy.adjustments,
            'metrics': self.metrics, 'line_quality': self.quality,
        }
        self.status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LineExposureNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
