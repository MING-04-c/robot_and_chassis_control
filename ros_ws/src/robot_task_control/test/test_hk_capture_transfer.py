#!/usr/bin/env python3
"""Verify HK capture service and compatibility with the original Ethernet receiver."""
import importlib.util
import os
from pathlib import Path
import socket
import tempfile
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '188'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/hk_capture_transfer_test_logs'

import rclpy
from ament_index_python.packages import get_package_prefix
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from robot_task_control.hk_image_transfer import CameraImageTcpNode, PhotoQueue


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


capture_module = load_file('capture_module', Path(get_package_prefix('hik_camera_ros2_driver')) /
                           'lib/hik_camera_ros2_driver/dataset_capture_node.py')
receiver = load_file('receiver', Path(__file__).parents[1] / 'tools/receive_camera_tcp.py')


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        rclpy.init(args=['--ros-args', '-p', 'interactive_input:=false',
                        '-p', f'output_dir:={self.directory.name}',
                        '-p', 'exposure_times:=[1000]', '-p', 'settle_frames:=2',
                        '-p', 'settle_time_sec:=0.01'])
        self.camera = Node('fake_hk_camera')
        self.capture = capture_module.DatasetCaptureNode()
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.camera)
        self.executor.add_node(self.capture)
        self.captured_messages = []
        self.camera.create_subscription(Image, "/dataset_capture/captured_image", self.captured_messages.append, 10)
        self.accept_parameters = True
        self.publish_frames = True
        self.publisher = self.camera.create_publisher(Image, '/camera/image', 10)
        self.camera.create_timer(0.02, self.publish)
        self.camera.create_service(SetParameters, '/hik_camera_ros2_driver/set_parameters', self.set_params)
        self.client = self.camera.create_client(Trigger, '/dataset_capture/capture')
        self.wait_for(self.client.service_is_ready)

    def tearDown(self):
        self.executor.shutdown()
        self.capture.destroy_node()
        self.camera.destroy_node()
        rclpy.shutdown()
        self.directory.cleanup()

    def publish(self):
        if not self.publish_frames:
            return
        image = Image()
        image.header.stamp = self.camera.get_clock().now().to_msg()
        image.height, image.width, image.step = 2, 2, 6
        image.encoding = 'bgr8'
        image.data = bytes([0, 50, 200] * 4)
        self.publisher.publish(image)

    def set_params(self, request, response):
        response.results = [SetParametersResult(successful=self.accept_parameters, reason='mock')
                            for _ in request.parameters]
        return response

    def wait_for(self, predicate, seconds=6):
        end = time.monotonic() + seconds
        while not predicate() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)
        self.assertTrue(predicate(), 'condition timed out')

    def call(self):
        future = self.client.call_async(Trigger.Request())
        self.wait_for(future.done)
        return future.result()

    def test_saved_photo_acknowledgement(self):
        trigger_ns = self.camera.get_clock().now().nanoseconds
        response = self.call()
        self.assertTrue(response.success, response.message)
        self.wait_for(lambda: len(self.captured_messages) == 1)
        stamp = self.captured_messages[0].header.stamp
        self.assertGreaterEqual(stamp.sec * 1_000_000_000 + stamp.nanosec, trigger_ns)
        self.assertEqual(len(list(Path(self.directory.name).glob('*.jpg'))), 1)
        self.assertIn('saved 1/1', response.message)
        self.assertEqual(len((Path(self.directory.name) / 'manifest.csv').read_text().splitlines()), 2)

    def test_parameter_failure_is_not_reported_as_success(self):
        self.accept_parameters = False
        response = self.call()
        self.assertFalse(response.success)
        self.assertEqual(self.captured_messages, [])
        self.assertEqual(list(Path(self.directory.name).glob('*.jpg')), [])

    def test_missing_fresh_frames_fails_capture(self):
        self.publish_frames = False
        response = self.call()
        self.assertFalse(response.success)
        self.assertEqual(self.captured_messages, [])
        self.assertEqual(list(Path(self.directory.name).glob('*.jpg')), [])


class PhotoQueueTests(unittest.TestCase):
    def test_photo_order_and_retry_without_overwriting(self):
        queue = PhotoQueue(capacity=2)
        self.assertTrue(queue.publish(1, b'first'))
        self.assertTrue(queue.publish(2, b'second'))
        self.assertFalse(queue.publish(3, b'overflow'))
        self.assertFalse(queue.has_space())
        self.assertEqual(queue.wait_after(0), (1, b'first'))
        # A failed socket write does not remove the photo; reconnect retries the same packet.
        self.assertEqual(queue.wait_after(0), (1, b'first'))
        queue.delivered(1)
        self.assertEqual(queue.wait_after(0), (2, b'second'))
        self.assertTrue(queue.has_space())
        queue.close()
        self.assertEqual(queue.wait_after(0), (None, None))


class TransferTests(unittest.TestCase):
    def test_original_receiver_accepts_raw_and_jpeg(self):
        with socket.socket() as port_probe:
            port_probe.bind(('127.0.0.1', 0))
            port = port_probe.getsockname()[1]
        rclpy.init(args=['--ros-args', '-p', 'listen_ip:=127.0.0.1',
                        '-p', 'allowed_client_ip:=127.0.0.1', '-p', f'listen_port:={port}'])
        node = CameraImageTcpNode()
        try:
            self.assertFalse(node.on_ready(Trigger.Request(), Trigger.Response()).success)
            with socket.create_connection(('127.0.0.1', port), timeout=3.0) as peer:
                end = time.monotonic() + 2.0
                while not node.server.has_clients() and time.monotonic() < end:
                    time.sleep(0.01)
                self.assertTrue(node.server.has_clients())
                self.assertTrue(node.on_ready(Trigger.Request(), Trigger.Response()).success)
                message = Image(height=2, width=2, step=6, encoding='bgr8',
                                data=bytes([0, 50, 200] * 4))
                node.image_format = 'raw'
                node.on_image(message)
                header, payload = receiver.receive_frame(peer)
                self.assertEqual(header['version'], 2)
                self.assertEqual(payload, bytes(message.data))
                # Rapid photo triggers must bypass video throttling and keep their order.
                node.max_fps = 0.01
                for stamp in (100, 101, 102):
                    message.header.stamp.nanosec = stamp
                    node.on_image(message)
                received_stamps = [receiver.receive_frame(peer)[0]['stamp_ns'] for _ in range(3)]
                self.assertEqual(received_stamps, [100, 101, 102])
                node.image_format = 'jpeg'
                node.on_image(message)
                header, payload = receiver.receive_frame(peer)
                self.assertEqual(header['version'], 1)
                self.assertTrue(payload.startswith(b'\xff\xd8'))
        finally:
            node.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
