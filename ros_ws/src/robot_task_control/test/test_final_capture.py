#!/usr/bin/env python3
"""不打开设备：验证稳定采集、相机串行释放和任务取消的进程边界。"""
from contextlib import contextmanager
from pathlib import Path
import signal
import os
import hashlib
import json
import socket
import struct
import threading
import time
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import yaml

from robot_task_control import final_capture as capture
from robot_task_control.workflow_controller import WorkflowController

CONFIG = Path(__file__).resolve().parents[1]/'config/capture/final_capture.yaml'


class FinalCaptureTest(unittest.TestCase):
    def test_warmup_and_consecutive_stability_reject_moving_frames(self):
        config = capture.load_config(CONFIG)
        config.update(camera_warmup_sec=1., stable_frames=3)
        monitor = capture.StableFrames(config)
        dark = np.zeros((48, 64, 3), np.uint8)
        light = np.full_like(dark, 100)
        self.assertFalse(monitor.observe(dark, 0.))
        self.assertFalse(monitor.observe(dark, .5))
        self.assertFalse(monitor.observe(light, 1.))
        self.assertFalse(monitor.observe(dark, 1.1))
        self.assertFalse(monitor.observe(dark, 1.2))
        self.assertFalse(monitor.observe(dark, 1.3))
        self.assertTrue(monitor.observe(dark, 1.4))
        self.assertFalse(monitor.observe(light, 1.5))
        self.assertFalse(monitor.observe(light, 3.))  # 长时间丢帧后不能沿用旧稳定计数。

    def test_cameras_are_sequential_and_closed_before_transfer(self):
        events = []
        @contextmanager
        def fake_camera(command):
            kind = 'hk' if 'hik_camera' in command[0] else 'depth'
            self.assertFalse(any(item.endswith('_open') for item in events[-1:]))
            events.append(kind+'_open')
            try:
                yield MagicMock()
            finally:
                events.append(kind+'_closed')
        node = MagicMock()
        node.config = capture.load_config(CONFIG)
        node.get_node_names_and_namespaces.return_value = []
        node.bridge.imgmsg_to_cv2.return_value = np.zeros((48, 64, 3), np.uint8)
        node.stable_image.return_value = SimpleNamespace(width=64, height=48,
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=2)))
        future = MagicMock()
        future.done.return_value = True
        node.create_client.return_value.call_async.return_value = future
        saver = MagicMock()
        with tempfile.TemporaryDirectory() as directory:
            node.output = Path(directory)
            node.report = {}
            saved = node.output/'rgbd'/('camera_v5_4_'+node.output.name)/'attempt_01'
            def save(*_):
                saved.mkdir(parents=True)
                return saved
            saver.save_capture.side_effect = save
            with patch.object(capture, 'camera_process', fake_camera), \
                    patch.object(capture, 'executable', lambda package, name: '/'+package+'/'+name), \
                    patch.object(capture, 'depth_module', return_value=saver), \
                    patch.object(capture.rclpy, 'spin_until_future_complete'):
                capture.CaptureSession.capture(node)
            self.assertTrue((node.output/'hk.jpg').is_file())
            self.assertEqual(events, ['hk_open', 'hk_closed', 'depth_open', 'depth_closed'])
            node.pause_line_camera.assert_called_once()
            node.wait.assert_called_once_with(3.)
            self.assertEqual(node.report['rgbd'], str(saved.relative_to(node.output)))
            self.assertEqual(node.write_state.call_args.args[0], 'all_cameras_closed')

    def test_standalone_capture_allows_absent_usb_camera(self):
        node = MagicMock()
        node.config = capture.load_config(CONFIG)
        node.config['allow_no_line_camera'] = True
        client = node.create_client.return_value
        client.wait_for_service.return_value = False
        capture.CaptureSession.pause_line_camera(node)
        client.call_async.assert_not_called()
        node.destroy_client.assert_called_once_with(client)
        node.config['allow_no_line_camera'] = False
        with self.assertRaisesRegex(RuntimeError, '未找到循迹相机'):
            capture.CaptureSession.pause_line_camera(node)

    def test_camera_cleanup_on_capture_failure(self):
        process = MagicMock()
        process.poll.return_value = None
        process.returncode = 0
        with patch.object(capture.subprocess, 'Popen', return_value=process):
            with self.assertRaisesRegex(RuntimeError, 'no stable image'):
                with capture.camera_process(['mock_camera']):
                    raise RuntimeError('no stable image')
        process.send_signal.assert_called_once_with(signal.SIGINT)
        process.wait.assert_called_once_with(timeout=8)

    def test_task_waits_for_capture_and_propagates_error(self):
        node = MagicMock()
        node.phase = 'final_capture_running'
        process = node.capture_process
        process.poll.return_value = None
        WorkflowController.advance_final_capture(node, {})
        node.complete_step.assert_not_called()
        process.poll.return_value = 1
        WorkflowController.advance_final_capture(node, {})
        node.fail_workflow.assert_called_once()
        node.complete_step.assert_not_called()
        node.capture_process = process
        process.poll.return_value = 0
        WorkflowController.advance_final_capture(node, {})
        node.complete_step.assert_called_once()

    def test_stop_targets_only_capture_process_group(self):
        node = MagicMock()
        node.capture_process.pid = 12345
        node.capture_process.poll.return_value = None
        node.capture_process.wait.side_effect = [subprocess.TimeoutExpired('mock', 10), 0]
        with patch('robot_task_control.workflow_controller.os.killpg') as kill:
            WorkflowController.stop_capture_process(node, wait=True)
        self.assertEqual([call.args for call in kill.call_args_list],
                         [(12345, signal.SIGINT), (12345, signal.SIGKILL)])

    def test_existing_hk_and_rgbd_tcp_protocols_receive_one_final_set(self):
        # 合成文件只验证传输/校验协议，不冒充相机标定或实测数据。
        receiver_path = Path(capture.executable('eli_cs_robot_serial', 'industrial_pc_hk_receiver.py'))
        spec = capture.importlib.util.spec_from_file_location('test_hk_receiver', receiver_path)
        receiver = capture.importlib.util.module_from_spec(spec)
        spec.loader.exec_module(receiver)
        receive_frame = receiver.receive_frame
        transfer = capture.depth_module('capture_transfer')
        with tempfile.TemporaryDirectory() as directory, socket.socket() as depth_server:
            output = Path(directory)/'.pending'/'abc123'
            folder = output/'rgbd/camera_v5_4_abc123/attempt_01'
            folder.mkdir(parents=True)
            core = {name: ('fixture-'+name).encode() for name in transfer.CORE}
            core['camera_parameters_same_session.json'] = b'{"rgb_intrinsic":{}}'
            for name, data in core.items():
                (folder/name).write_bytes(data)
            manifest = {'schema': 'paint_rgbd_v5_legacy_capture_v1',
                        'robot_motion_authorized': False,
                        'source_files_sha256': {name: hashlib.sha256(data).hexdigest() for name, data in core.items()}}
            (folder/'capture_manifest.json').write_text(json.dumps(manifest))
            (folder/'CAPTURE_COMPLETE.json').write_text(json.dumps({
                'robot_motion_authorized': False,
                'sha256_capture_manifest': transfer.sha256(folder/'capture_manifest.json')}))
            for name in transfer.GROUP_FILES - transfer.CORE - {'capture_manifest.json', 'CAPTURE_COMPLETE.json'}:
                (folder/name).write_bytes(b'auxiliary-fixture')
            (output/'hk.jpg').write_bytes(b'jpeg-test-payload')
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                hk_port = probe.getsockname()[1]
            depth_server.bind(('127.0.0.1', 0))
            depth_server.listen()
            depth_server.settimeout(5)
            received, errors = {}, []
            def receive_hk():
                try:
                    deadline = time.monotonic()+5
                    while True:
                        try:
                            connection = socket.create_connection(('127.0.0.1', hk_port), timeout=2)
                            break
                        except OSError:
                            if time.monotonic() > deadline:
                                raise
                            time.sleep(.02)
                    with connection:
                        received['hk'] = receive_frame(connection)
                except Exception as error:
                    errors.append(error)
            def receive_depth():
                try:
                    connection, _ = depth_server.accept()
                    with connection:
                        connection.settimeout(5)
                        self.assertEqual(transfer.read_exact(connection, 4), transfer.MAGIC)
                        header = transfer.read_json(connection)
                        body = transfer.read_exact(connection, header['size'])
                        self.assertEqual(hashlib.sha256(body).hexdigest(), header['sha256'])
                        archive = output/'received.zip'
                        archive.write_bytes(body)
                        self.assertEqual(transfer.verify_archive(archive), 1)
                        transfer.write_json(connection, {'ok': True, 'sha256': header['sha256']})
                        received['rgbd'] = True
                except Exception as error:
                    errors.append(error)
            workers = [threading.Thread(target=fn, daemon=True) for fn in (receive_hk, receive_depth)]
            for worker in workers:
                worker.start()
            node = MagicMock()
            node.output, node.config = output, capture.load_config(CONFIG)
            node.config.update(hk_bind_ip='127.0.0.1', receiver_ip='127.0.0.1',
                               hk_port=hk_port, rgbd_port=depth_server.getsockname()[1], transfer_timeout_sec=5.)
            node.report = {'hk': {'width': 64, 'height': 48, 'stamp_ns': 123}}
            node.storage_root = Path(directory)/'visible_pi'
            capture.CaptureSession.persist_hk(node)
            capture.CaptureSession.persist_rgbd(node, folder)
            import capture_storage
            with capture_storage.index(node.storage_root) as metadata:
                record = metadata['sessions']['abc123']
            photos = node.storage_root/record['directory']
            self.assertEqual(sorted(p.name for p in photos.iterdir()), ['RGB', 'hk'])
            self.assertEqual(capture_storage.file_path(node.storage_root, record, 'depth.png').read_bytes(),
                             (folder/'d2c_depth_raw_uint16.png').read_bytes())
            node.wait.side_effect = time.sleep
            capture.CaptureSession.transfer(node)
            for worker in workers:
                worker.join(5)
                self.assertFalse(worker.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual(received['hk'][1], b'jpeg-test-payload')
            self.assertEqual(received['hk'][0]['grouping'], 'task_final_one_hk')
            self.assertEqual(received['hk'][0]['capture_set_id'], 'abc123')
            self.assertTrue(received['rgbd'])
            node.write_state.assert_called_with('complete')

    def test_ros_fresh_frames_and_line_camera_pause_service(self):
        # 隔离域内模拟新图像与USB暂停服务，无真实设备节点。
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from std_srvs.srv import SetBool
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '190',
                                    'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
                                    'ROS_LOG_DIR': '/tmp/final_capture_ros_test'}), tempfile.TemporaryDirectory() as directory:
            capture.rclpy.init()
            config = capture.load_config(CONFIG)
            config.update(camera_warmup_sec=.1, stable_frames=3, stability_timeout_sec=5.,
                          line_streaming_service='/final_capture_mock/streaming')
            node = capture.CaptureSession(config, Path(directory))
            harness = Node('final_capture_mock')
            from rclpy.executors import SingleThreadedExecutor
            harness_executor = SingleThreadedExecutor()
            harness_executor.add_node(harness)
            paused = []
            def pause(request, response):
                paused.append(request.data)
                response.success = True
                return response
            service = harness.create_service(SetBool, '/final_capture_mock/streaming', pause)
            publisher = harness.create_publisher(Image, '/final_capture_mock/image', qos_profile_sensor_data)
            stop = threading.Event()
            sent = []
            def publish():
                bridge = CvBridge()
                while not stop.is_set():
                    harness_executor.spin_once(timeout_sec=.02)
                    brightness = 50 if len(sent) % 2 == 0 and len(sent) < 8 else 100
                    image = bridge.cv2_to_imgmsg(np.full((48, 64, 3), brightness, np.uint8), encoding='bgr8')
                    image.header.stamp = harness.get_clock().now().to_msg()
                    publisher.publish(image)
                    sent.append(image.header.stamp)
                    stop.wait(.03)
            worker = threading.Thread(target=publish, daemon=True)
            worker.start()
            process = MagicMock()
            process.poll.return_value = None
            try:
                node.pause_line_camera()
                self.assertEqual(paused, [False])
                image = node.stable_image('/final_capture_mock/image', process)
                self.assertGreaterEqual(len(sent), 8)
                self.assertEqual(image.width, 64)
                self.assertEqual(image.height, 48)
                self.assertGreater(image.header.stamp.sec, 0)
            finally:
                stop.set()
                worker.join(2)
                harness_executor.shutdown()
                harness.destroy_service(service)
                harness.destroy_node()
                node.destroy_node()
                capture.rclpy.shutdown()

    def test_capture_failure_stops_only_capture_process(self):
        node = MagicMock()
        node.capture_only = True
        node.generation = 0
        WorkflowController.fail_workflow(node, 'capture cancelled')
        node.stop_capture_process.assert_called_once()
        self.assertFalse(node.active)
        self.assertTrue(node.failed)
        node.control_client.call_async.assert_not_called()
        node.admittance_enable_client.call_async.assert_not_called()
        node.mux_enable_client.call_async.assert_not_called()
        node.follow_enable_client.call_async.assert_not_called()

    def test_invalid_config_rejected(self):
        for changes in ({'stable_frames': 1}, {'hk_port': 0}, {'camera_warmup_sec': float('nan')},
                        {'pause_line_camera': 'false'}, {'receiver_ip': 'not-an-ip'}):
            config = yaml.safe_load(CONFIG.read_text())
            config.update(changes)
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/'config.yaml'
                path.write_text(yaml.safe_dump(config))
                with self.assertRaises(ValueError):
                    capture.load_config(path)


if __name__ == '__main__':
    unittest.main()
