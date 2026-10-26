#!/usr/bin/env python3
"""Fake UVC backend and isolated ROS messages; no camera/robot control calls."""

import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from line_exposure_node import LineExposureNode  # noqa: E402


class FakeCamera:
    current = 60
    minimum = 1
    maximum = 5000
    step = 1

    def __init__(self):
        self.writes = []
        self.starts = 0

    def start(self):
        self.starts += 1
        return self.current

    def set(self, value):
        self.current = value
        self.writes.append(value)
        return value


def main():
    rclpy.init()
    backend = FakeCamera()
    node = LineExposureNode(backend=backend)
    harness = Node('fake_exposure_camera')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(harness)
    images = harness.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
    lines = harness.create_publisher(String, '/camera/line/status', 10)
    frame = np.full((240, 320, 3), 67, dtype=np.uint8)
    frame[50:230, 150:165] = (5, 1, 0)
    bridge = CvBridge()
    state = {'lost': True, 'valid_row_fraction': 0.0, 'processing_over_budget': False,
             'target_estimated': False, 'target_fallback': False, 'inferred_rows': 0}

    def pump(seconds, publish=True, old=False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if publish:
                image = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                image.header.stamp = harness.get_clock().now().to_msg()
                if old:
                    image.header.stamp.sec -= 5
                images.publish(image)
                lines.publish(String(data=json.dumps(state)))
            for _ in range(4):
                executor.spin_once(timeout_sec=0.01)

    try:
        pump(3.0)
        assert backend.starts == 1 and backend.writes, 'dark unreliable line did not adjust exposure'
        assert backend.writes[0] > 60
        assert all(20 <= value <= 200 for value in backend.writes)
        state.update(lost=False, valid_row_fraction=0.8)
        previous = len(backend.writes)
        pump(2.0)
        assert len(backend.writes) == previous and node.reason == 'stable_hold'
        state.update(lost=True, valid_row_fraction=0.0)
        pump(1.5, publish=False)
        assert len(backend.writes) == previous and node.reason == 'stale_observation'
        pump(1.0, old=True)
        assert len(backend.writes) == previous
        node.set_parameters([Parameter('adaptive_enabled', value=False)])
        pump(1.0)
        assert len(backend.writes) == previous and node.reason == 'disabled'
        print('PASS: fake exposure increases for dark unreliable line; stable/stale/disabled hold; no hardware calls')
    finally:
        executor.shutdown()
        node.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
