#!/usr/bin/env python3
"""隔离ROS域：TCP启动只执行一次，状态查询、恢复消息、旧坐标协议仍可用。"""
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

os.environ['ROS_DOMAIN_ID'] = '186'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOG_DIR'] = '/tmp/industrial-pc-ros-test'
DEVICE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(DEVICE/'src'), str(DEVICE/'host_tools')]

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from industrial_pc_node import IndustrialPC
from robot_tcp_common import request, status


class IndustrialPCTest(unittest.TestCase):
    def test_tcp_detection_points_reach_shared_contact_flow(self):
        """真实TCP与ROS回环；采集、位姿与检测执行器为替身，无硬件或运动驱动。"""
        import numpy as np
        import yaml
        from geometry_msgs.msg import PoseArray
        from robot_task_control import camera_detection_task as module
        from robot_task_control.final_capture import load_config
        from ament_index_python.packages import get_package_share_directory
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {
                'ROS_DOMAIN_ID': '192', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}):
            root = Path(directory)
            config_file = root/'capture.yaml'
            config = load_config(Path(get_package_share_directory('robot_task_control'))/'config/capture/final_capture.yaml')
            config['output_dir'] = str(root/'images')
            config_file.write_text(yaml.safe_dump(config))
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            rclpy.init(args=['--ros-args', '-p', 'bind_host:=127.0.0.1', '-p', 'port:='+str(port),
                            '-p', 'use_shared_key:=false', '-p', 'target_source:=tcp',
                            '-p', 'capture_config_file:='+str(config_file),
                            '-p', 'report_directory:='+str(root/'reports'),
                            '-p', 'output_dir:='+str(root/'results')])
            starts, targets, began = [], [], [None]
            def pose(node, frame):
                value = np.eye(4)
                if frame == node.camera:
                    value[0, 3] = .2
                return value
            def captured(command, **kwargs):
                identity = command[command.index('--run-id')+1]
                images = root/'images'
                images.mkdir(exist_ok=True)
                (images/'.capture_index.json').write_text(json.dumps({'sessions': {identity: {
                    'directory': 'fixture', 'rgbd': {'capture_manifest_sha256': 'a'*64}}}}))
                process = MagicMock()
                process.poll.return_value = process.returncode = 0
                return process
            with patch.object(module.CameraDetectionTask, 'check_hardware', lambda _: None), \
                    patch.object(module.CameraDetectionTask, 'connected', lambda _: True), \
                    patch.object(module.CameraDetectionTask, 'ready', lambda _: True), \
                    patch.object(module.CameraDetectionTask, 'pose', pose), \
                    patch.object(module.subprocess, 'Popen', side_effect=captured):
                task = module.CameraDetectionTask()
                bridge = IndustrialPC()
                detector = Node('contact_test_double')
                def success(req, response):
                    response.success, response.message = True, 'test double'
                    return response
                def start(req, response):
                    starts.append(True)
                    began[0] = time.monotonic()
                    return success(req, response)
                detector.create_service(Trigger, '/detect_controller/clear_targets', success)
                detector.create_service(Trigger, '/detect_controller/record_home', success)
                detector.create_service(Trigger, '/detect_controller/start', start)
                detector.create_service(Trigger, '/detect_controller/cancel', success)
                detector.create_subscription(PoseArray, '/geminipro_calibration/targets_base', targets.append, 10)
                publisher = detector.create_publisher(String, '/detect_controller/status', 10)
                def broadcast():
                    elapsed = time.monotonic()-began[0] if began[0] is not None else -1.
                    state = 'IDLE' if elapsed < 0 else ('HOLD' if elapsed < .8 else 'DONE')
                    completed = (0 if elapsed < .4 else 1) if state == 'HOLD' else (2 if state == 'DONE' else 0)
                    publisher.publish(String(data=json.dumps({'state': state, 'motion_ready': True,
                        'queued_points': len(targets[-1].poses) if targets else 0,
                        'completed_points': completed})))
                detector.create_timer(.05, broadcast)
                executor = SingleThreadedExecutor()
                for node in (task, bridge, detector):
                    executor.add_node(node)
                worker = threading.Thread(target=executor.spin, daemon=True)
                worker.start()
                client = {'robot_host': '127.0.0.1', 'control_port': port, 'timeout': 3,
                          'use_shared_key': False}
                try:
                    deadline = time.monotonic()+8.
                    while task.phase != 'WAIT_RESULT' and time.monotonic() < deadline:
                        time.sleep(.02)
                    self.assertEqual(task.phase, 'WAIT_RESULT', task.detail)
                    payload = {'schema': 'paint_v54_pi_remote_result_v1', 'message_id': 'wrong-photo',
                        'status': 'OK_3D_POINTS', 'capture_manifest_sha256': 'b'*64,
                        'coordinate_frame': 'CAMERA_PINHOLE_PRELIMINARY', 'depth_source': 'MEASURED',
                        'robot_motion_authorized': False, 'robot_base_coordinate_calibrated': False,
                        'robot_command': None, 'points_xyz_camera_mm': [
                            {'point_id': 'p1', 'xyz_camera_mm': [10., 20., 300.]},
                            {'point_id': 'p2', 'xyz_camera_mm': [20., 30., 350.]}]}
                    self.assertTrue(request(client, payload)['ok'])
                    time.sleep(.2)
                    self.assertFalse(starts)
                    payload.update(message_id='current-photo', capture_manifest_sha256='a'*64)
                    self.assertTrue(request(client, payload)['ok'])
                    deadline = time.monotonic()+5.
                    while task.phase not in ('DONE', 'FAULT') and time.monotonic() < deadline:
                        time.sleep(.02)
                    self.assertEqual(task.phase, 'DONE', task.detail)
                    self.assertEqual(len(starts), 1)
                    self.assertEqual(len(targets[-1].poses), 2)
                    first = targets[-1].poses[0].position
                    for actual, expected in zip([first.x, first.y, first.z], [.21, .02, .3]):
                        self.assertAlmostEqual(actual, expected)
                    self.assertEqual(request(client, payload)['disposition'], 'duplicate')
                    self.assertEqual(len(starts), 1)
                finally:
                    executor.shutdown(timeout_sec=1.)
                    worker.join(1.)
                    bridge.close()
                    task.live.close()
                    for node in (bridge, task, detector):
                        node.destroy_node()
                    rclpy.shutdown()

    def test_remote_commands_status_and_old_coordinates(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            key = root/'key'
            key.write_text('test-only-shared-key-more-than-32-bytes')
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            rclpy.init(args=['--ros-args', '-p', 'bind_host:=127.0.0.1', '-p', 'port:='+str(port),
                            '-p', 'use_shared_key:=true', '-p', 'key_file:='+str(key), '-p', 'output_dir:='+str(root/'results')])
            bridge = IndustrialPC()
            mock = Node('mock_task_control')
            counts = {'start': 0, 'stop': 0, 'resume': 0, 'points': 0}
            def command(action):
                def call(req, response):
                    counts[action] += 1
                    response.success = True
                    response.message = 'mock '+action
                    return response
                return call
            mock.create_service(Trigger, '/task_controller/start', command('start'))
            mock.create_service(Trigger, '/task_controller/stop', command('stop'))
            def resumed(message):
                counts['resume'] += 1
            mock.create_subscription(Empty, '/line_follow/resume', resumed, 10)
            retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            task_pub = mock.create_publisher(String, '/task_controller/status', retained)
            follow_pub = mock.create_publisher(String, '/line_follow/status', 10)
            def broadcast():
                task_pub.publish(String(data=json.dumps({'workflow': 'mock', 'active': True})))
                follow_pub.publish(String(data=json.dumps({'state': 'stop_marker_intermediate'})))
            mock.create_timer(.05, broadcast)
            executor = SingleThreadedExecutor()
            executor.add_node(bridge)
            executor.add_node(mock)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            config = {'robot_host': '127.0.0.1', 'control_port': port, 'timeout': 5,
                      'key_file': str(key), 'use_shared_key': True}
            try:
                deadline = time.monotonic()+5
                while time.monotonic() < deadline:
                    snapshot = status(config)['robot_status']
                    if snapshot['start_service_ready'] and snapshot['line_follow']['fresh']:
                        break
                    time.sleep(.05)
                self.assertTrue(snapshot['start_service_ready'])
                payload = {'schema': 'paint_robot_task_request_v1', 'action': 'start', 'message_id': 'start-once'}
                first = request(config, payload)
                self.assertTrue(first['ok'])
                self.assertEqual(request(config, payload)['disposition'], 'duplicate')
                self.assertEqual(counts['start'], 1)
                conflicting = dict(payload, action='stop')
                self.assertFalse(request(config, conflicting)['ok'])
                self.assertEqual(counts['stop'], 0)
                self.assertTrue(request(config, dict(payload, action='stop', message_id='stop-once'))['ok'])
                self.assertEqual(counts['stop'], 1)
                self.assertTrue(request(config, dict(payload, action='resume', message_id='resume-once'))['ok'])
                deadline = time.monotonic()+1
                while counts['resume'] == 0 and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertEqual(counts['resume'], 1)
                request(config, dict(payload, action='resume', message_id='resume-once'))
                self.assertEqual(counts['resume'], 1)
                demo = json.loads((DEVICE/'host_tools/DEMO_XYZ_800mm.json').read_text())
                demo['message_id'] = 'demo-coordinate-test'
                ack = request(config, demo)
                self.assertTrue(ack['ok'])
                self.assertEqual(ack['kind'], 'demo')
                self.assertTrue(status(config)['robot_status']['task']['fresh'])
                # 去重记录是磁盘文件，不仅是当前连接中的内存标记。
                self.assertEqual(len(list((root/'results/commands').glob('*.json'))), 3)
                executor.remove_node(bridge)
                bridge.close()
                bridge.destroy_node()
                bridge = IndustrialPC()
                executor.add_node(bridge)
                self.assertEqual(request(config, payload)['disposition'], 'duplicate')
                self.assertEqual(counts['start'], 1)  # 通讯节点重启也不能再次执行同ID。
            finally:
                executor.shutdown(timeout_sec=2)
                thread.join(2)
                bridge.close()
                bridge.destroy_node()
                mock.destroy_node()
                rclpy.shutdown()


    def test_packaged_start_command_runs_real_waiting_workflow(self):
        # 真工作流只含delay，不加载任何运动驱动，验证封装工具到ROS任务的完整链路。
        import subprocess
        from robot_task_control.workflow_controller import WorkflowController
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            key = root/'industrial_pc.key'
            key.write_text('test-only-shared-key-more-than-32-bytes')
            workflow = root/'workflow.yaml'
            workflow.write_text('workflow:\n  name: remote_start_smoke\n  auto_start: false\n  steps:\n    - action: delay\n      seconds: 1.0\n')
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            rclpy.init(args=['--ros-args', '-p', 'bind_host:=127.0.0.1', '-p', 'port:='+str(port),
                            '-p', 'use_shared_key:=true', '-p', 'key_file:='+str(key), '-p', 'output_dir:='+str(root/'results'),
                            '-p', 'workflow_file:='+str(workflow), '-p', 'auto_start:=false'])
            bridge, task = IndustrialPC(), WorkflowController()
            executor = SingleThreadedExecutor()
            executor.add_node(bridge)
            executor.add_node(task)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            config = {'robot_host': '127.0.0.1', 'control_port': port, 'timeout': 5,
                      'hk_port': 9002, 'rgbd_port': 9030, 'output_dir': str(root/'received'),
                      'key_file': str(key), 'use_shared_key': True}
            config_file = root/'config.json'
            config_file.write_text(json.dumps(config))
            try:
                deadline = time.monotonic()+5
                while time.monotonic() < deadline:
                    snapshot = status(config)['robot_status']
                    if snapshot['start_service_ready'] and snapshot['task']['fresh']:
                        break
                    time.sleep(.05)
                self.assertFalse(task.active)
                # 使用实际交付目录中的可执行脚本，而非只调用协议辅助函数。
                tool = DEVICE.parents[4]/'industrial_pc_kylin_tcp/start_task.py'
                self.assertTrue(tool.is_file(), tool)
                result = subprocess.run([sys.executable, str(tool), '--config', str(config_file),
                                         '--message-id', 'real-workflow-start'],
                                        capture_output=True, text=True, timeout=8)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                deadline = time.monotonic()+5
                while time.monotonic() < deadline and task.step_index < 1:
                    time.sleep(.05)
                self.assertEqual(task.step_index, 1)
                self.assertFalse(task.failed)
                snapshot = status(config)['robot_status']
                self.assertEqual(snapshot['task']['data']['workflow'], 'remote_start_smoke')
                previous_id = task.run_id
                result = subprocess.run([sys.executable, str(tool), '--config', str(config_file),
                                         '--message-id', 'real-workflow-start'],
                                        capture_output=True, text=True, timeout=8)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                self.assertIn('duplicate', result.stdout)
                self.assertEqual(task.run_id, previous_id)
            finally:
                executor.shutdown(timeout_sec=2)
                thread.join(2)
                bridge.close()
                bridge.destroy_node()
                task.destroy_node()
                rclpy.shutdown()


    def test_packaged_capture_with_real_controller_and_motion_interlock(self):
        import subprocess
        from robot_task_control.workflow_controller import WorkflowController
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            key = root/'industrial_pc.key'
            key.write_text('test-only-shared-key-more-than-32-bytes')
            workflow = root/'workflow.yaml'
            workflow.write_text('workflow:\n  name: movement_placeholder\n  auto_start: false\n  steps:\n    - action: delay\n      seconds: 2.0\n')
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            rclpy.init(args=['--ros-args', '-p', 'bind_host:=127.0.0.1', '-p', 'port:='+str(port),
                            '-p', 'use_shared_key:=false', '-p', 'key_file:='+str(root/'not-existing.key'), '-p', 'output_dir:='+str(root/'results'),
                            '-p', 'workflow_file:='+str(workflow), '-p', 'auto_start:=false'])
            bridge, task = IndustrialPC(), WorkflowController()
            launched = []
            def fake_camera_process(command):
                # 只替换相机worker为短暂等待的子进程，仍跑真实服务、轮询和完成逻辑。
                launched.append(command)
                return subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(1)'],
                                        start_new_session=True)
            task.launch_capture_process = fake_camera_process
            executor = SingleThreadedExecutor()
            executor.add_node(bridge)
            executor.add_node(task)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            config = {'robot_host': '127.0.0.1', 'control_port': port, 'timeout': 5,
                      'hk_port': 9002, 'rgbd_port': 9030, 'output_dir': str(root/'received'),
                      'use_shared_key': False}
            config_file = root/'config.json'
            config_file.write_text(json.dumps(config))
            try:
                deadline = time.monotonic()+5
                while time.monotonic() < deadline:
                    if status(config)['robot_status']['capture_service_ready']:
                        break
                    time.sleep(.05)
                tool = DEVICE.parents[4]/'industrial_pc_kylin_tcp/start_capture.py'
                command = [sys.executable, str(tool), '--config', str(config_file),
                           '--message-id', 'capture-once']
                result = subprocess.run(command, capture_output=True, text=True, timeout=8)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                self.assertTrue(task.capture_only)
                self.assertIn('"robot_motion_authorized": false', result.stdout)
                rejected = request(config, {'schema': 'paint_robot_task_request_v1',
                    'action': 'start', 'message_id': 'start-during-capture'})
                self.assertFalse(rejected['ok'])
                deadline = time.monotonic()+5
                while task.active and time.monotonic() < deadline:
                    time.sleep(.05)
                self.assertEqual(task.phase, 'capture_complete')
                self.assertFalse(task.failed)
                self.assertEqual(len(launched), 1)
                self.assertIn('--allow-no-line-camera', launched[0])
                self.assertEqual(task.step_index, 0)  # 没有运行加载的运动工作流。
                result = subprocess.run(command, capture_output=True, text=True, timeout=8)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                self.assertEqual(len(launched), 1)
                # 完成后仍能启动原任务；任务运行中反过来禁止单独采集。
                started = request(config, {'schema': 'paint_robot_task_request_v1',
                    'action': 'start', 'message_id': 'start-after-capture'})
                self.assertTrue(started['ok'])
                self.assertFalse(task.capture_only)
                rejected = request(config, {'schema': 'paint_robot_task_request_v1',
                    'action': 'capture', 'message_id': 'capture-during-task'})
                self.assertFalse(rejected['ok'])
                self.assertEqual(len(launched), 1)
            finally:
                executor.shutdown(timeout_sec=2)
                thread.join(2)
                bridge.close()
                bridge.destroy_node()
                task.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
