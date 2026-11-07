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
sys.path[:0] = [str(DEVICE/'src'), str(DEVICE/'host_tools'),
               str(DEVICE.parents[2]/'geminipro_camer_calibration/scripts')]

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from industrial_pc_node import IndustrialPC
from robot_tcp_common import request, status


class IndustrialPCTest(unittest.TestCase):
    def test_tcp_remains_available_with_status_monitor_disabled(self):
        """本机回环与隔离ROS域；任务服务为替身，不加载任何运动驱动。"""
        with tempfile.TemporaryDirectory() as directory:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            rclpy.init(args=['--ros-args', '-p', 'bind_host:=127.0.0.1', '-p', 'port:='+str(port),
                            '-p', 'enable_status_monitor:=false', '-p', 'output_dir:='+directory])
            with patch('industrial_pc_node.RuntimeMonitor', side_effect=AssertionError('监视器不应启动')):
                bridge = IndustrialPC()
            fake = Node('tcp_only_test_services')
            calls = []
            def handler(name):
                def call(request, response):
                    calls.append(name)
                    response.success, response.message = True, 'test-only service'
                    return response
                return call
            for name in ('start', 'stop'):
                fake.create_service(Trigger, '/task_controller/'+name, handler(name))
            executor = SingleThreadedExecutor()
            executor.add_node(bridge)
            executor.add_node(fake)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            config = dict(robot_host='127.0.0.1', control_port=port, timeout=5., use_shared_key=False)
            try:
                self.assertIsNone(bridge.monitor)
                topics = {sub.topic_name for sub in bridge.subscriptions}
                self.assertNotIn('/rosout', topics)
                self.assertNotIn('/protective_stop_guard/status', topics)
                self.assertNotIn('/arm_joint_postures/status', topics)
                self.assertFalse(any(timer.timer_period_ns == 500_000_000 for timer in bridge.timers))
                bridge.cache_status('task', String(data=json.dumps(dict(active=False, phase='ready'))))
                packet = status(config)
                self.assertTrue(packet['ok'])
                self.assertFalse(packet['robot_status']['status_monitor_enabled'])
                self.assertIsNone(packet['robot_status']['normal_status'])
                self.assertEqual(packet['robot_status']['task']['data']['phase'], 'ready')
                for channel in ('normal', 'abnormal'):
                    packet = request(config, dict(schema='paint_robot_status_request_v1',
                                     message_id='disabled-'+channel, channel=channel))
                    self.assertFalse(packet['ok'])
                    self.assertFalse(packet['status_monitor_enabled'])
                deadline = time.monotonic()+3.
                while not bridge.start_task.service_is_ready() and time.monotonic() < deadline:
                    time.sleep(.02)
                for name in ('start', 'stop'):
                    packet = request(config, dict(schema='paint_robot_task_request_v1',
                                     message_id='tcp-only-'+name, action=name))
                    self.assertTrue(packet['ok'], packet)
                self.assertEqual(calls, ['start', 'stop'])
            finally:
                executor.shutdown()
                thread.join(timeout=3)
                bridge.close()
                bridge.destroy_node()
                fake.destroy_node()
                rclpy.shutdown()

    def test_monitor_codes_unknown_state_dedup_and_explicit_recovery(self):
        from runtime_monitor import RuntimeMonitor
        monitor = RuntimeMonitor()
        normal, abnormal = monitor.snapshot(now=0.)
        self.assertEqual(normal['state'], 'unknown')
        self.assertFalse(normal['healthy'])
        task = dict(active=True, phase='follow_tracking', action='line_follow', run_id='r1')
        monitor.receive('task', task, now=1.)
        monitor.receive('line_follow', dict(state='reverse_visual_finish_tracking'), now=1.)
        normal, abnormal = monitor.snapshot(now=1.)
        self.assertTrue(normal['healthy'])
        monitor.receive('task', dict(task, failed=True, failure_reason='test failed'), now=2.)
        monitor.receive('line_follow', dict(state='charging_dock_search_limit'), now=2.)
        monitor.receive('arm', dict(state='FAILED', error='planning failed', native_error_code=-1), now=2.)
        monitor.receive('safety', dict(latched=True, safety_mode=3, detail='protective stop'), now=2.)
        monitor.receive('chassis', dict(fault=0x100), now=2.)
        monitor.receive('comm', dict(state=4, fail_reason=0), now=2.)
        normal, abnormal = monitor.snapshot(now=2.)
        codes = {item['error_code'] for item in abnormal['active_faults']}
        self.assertTrue({'E1001', 'E2001', 'E2101', 'E3101', 'E4001'} <= codes)
        self.assertEqual(next(e for e in abnormal['active_faults'] if e['error_code'] == 'E2101')['native_error_code'], -1)
        count = len(abnormal['events'])
        self.assertEqual(len(monitor.snapshot(now=2.1)[1]['events']), count, '相同故障重复刷事件')
        normal, abnormal = monitor.snapshot(now=8.)
        codes = {item['error_code'] for item in abnormal['active_faults']}
        self.assertTrue({'E1002', 'E2001', 'E4001', 'E4003'} <= codes, '失联不能抹掉已知故障')
        monitor.receive('task', dict(active=False, phase='ready', step_index=10, step_count=10), now=9.)
        monitor.receive('arm', dict(state='DONE'), now=9.)
        monitor.receive('safety', dict(latched=False, safety_mode=1), now=9.)
        monitor.receive('chassis', dict(fault=0), now=9.)
        monitor.receive('comm', dict(state=4, fail_reason=0), now=9.)
        normal, abnormal = monitor.snapshot(now=9.)
        self.assertEqual(normal['state'], 'completed')
        self.assertFalse(abnormal['has_fault'])
        self.assertTrue(any(event['event'] == 'cleared' for event in abnormal['events']))
        monitor.receive('task', dict(active=False, failed=True, failure_reason='stopped by user'), now=9.1)
        self.assertEqual(monitor.snapshot(now=9.1)[0]['state'], 'stopped')
        self.assertFalse(monitor.snapshot(now=9.1)[1]['has_fault'])
        for i in range(150):
            monitor.log_error('driver', 'error '+str(i)+'x'*2000, 40, now=float(i))
        _, abnormal = monitor.snapshot(now=150.)
        self.assertLessEqual(len(abnormal['events']), 50)
        self.assertLess(len(json.dumps(abnormal).encode()), 200000)

    def test_tcp_monitor_sends_normal_and_abnormal_separately_without_motion(self):
        from monitor_status import read_channel, run
        from std_msgs.msg import Bool
        from rcl_interfaces.msg import Log
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            rclpy.init(args=['--ros-args', '-p', 'bind_host:=127.0.0.1', '-p', 'port:='+str(port),
                            '-p', 'use_shared_key:=false', '-p', 'output_dir:='+str(root/'bridge')])
            node, fake = IndustrialPC(), Node('monitor_test_publisher')
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            executor.add_node(fake)
            pub = fake.create_publisher(String, '/task_controller/status',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            arm_pub = fake.create_publisher(String, '/arm_joint_postures/status',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            log_pub = fake.create_publisher(Log, '/rosout', 10)
            failed = [False]
            def publish():
                pub.publish(String(data=json.dumps(dict(workflow='test', active=True, phase='waiting_delay',
                    action='delay', step_index=4, step_count=10, wait_seconds_remaining=5.,
                    failed=failed[0], failure_reason='test failure' if failed[0] else ''))))
                if failed[0]:
                    arm_pub.publish(String(data=json.dumps(dict(state='FAILED', error='IK failed', native_error_code=-31))))
                    log_pub.publish(Log(level=Log.ERROR, name='simulated_driver', msg='simulated native error',
                                        file='driver.cpp', function='read', line=123))
            fake.create_timer(.05, publish)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            config = dict(robot_host='127.0.0.1', control_port=port, timeout=5., use_shared_key=False,
                          key_file='', output_dir=str(root/'host'), status_interval=.1)
            try:
                deadline = time.monotonic()+4.
                while time.monotonic() < deadline:
                    normal = read_channel(config, 'normal')
                    if normal.get('workflow') == 'test':
                        break
                    time.sleep(.05)
                self.assertEqual(normal['schema'], 'robot_normal_status_v1')
                self.assertEqual(normal['state'], 'waiting')
                abnormal = read_channel(config, 'abnormal')
                self.assertFalse(abnormal['has_fault'])
                failed[0] = True
                deadline = time.monotonic()+4.
                while time.monotonic() < deadline:
                    abnormal = read_channel(config, 'abnormal')
                    codes = {item['error_code'] for item in abnormal['active_faults']}
                    if {'E1001', 'E2101'} <= codes and abnormal['recent_error_logs']:
                        break
                    time.sleep(.05)
                self.assertEqual(abnormal['active_faults'][0]['error_code'], 'E1001')
                self.assertEqual(next(f for f in abnormal['active_faults'] if f['error_code'] == 'E2101')['native_error_code'], -31)
                self.assertEqual(abnormal['recent_error_logs'][0]['source'], 'simulated_driver')
                self.assertEqual(run(config, once=True), 0)
                output = root/'host/status'
                self.assertTrue((output/'normal.json').is_file())
                self.assertTrue((output/'abnormal.json').is_file())
                size = (output/'events.jsonl').stat().st_size
                self.assertEqual(run(config, once=True), 0)
                self.assertEqual((output/'events.jsonl').stat().st_size, size, '历史事件重复保存')
                bad = request(config, dict(schema='paint_robot_status_request_v1', message_id='bad-channel', channel='bad'))
                self.assertFalse(bad['ok'])
                self.assertFalse(list((root/'bridge/commands').glob('*.json')), '只读查询产生了运动命令记录')
            finally:
                executor.shutdown()
                thread.join(timeout=3)
                node.close()
                node.destroy_node()
                fake.destroy_node()
                rclpy.shutdown()

    def test_tcp_battery_charging_layout_and_stale_feedback(self):
        """只发布模拟底盘状态，经真实本机TCP查询；不启动CAN或运动驱动。"""
        from masc_chassis_msgs.msg import BmsStatus, ChassisStatus, CommStatus
        with tempfile.TemporaryDirectory() as directory:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            rclpy.init(args=['--ros-args', '-p', 'bind_host:=127.0.0.1', '-p', 'port:='+str(port),
                            '-p', 'use_shared_key:=false', '-p', 'output_dir:='+directory])
            bridge, mock = IndustrialPC(), Node('mock_chassis_power')
            executor = SingleThreadedExecutor()
            executor.add_node(bridge)
            executor.add_node(mock)
            values = {'raw': 8, 'soc': 73, 'connected': True, 'old_stamp': False}
            pubs = {kind: mock.create_publisher(cls, '/masc_chassis/'+topic, 10)
                    for kind, cls, topic in [('bms', BmsStatus, 'bms_status'),
                        ('system', ChassisStatus, 'chassis_status'), ('comm', CommStatus, 'comm_status')]}
            def publish():
                bms = BmsStatus(bat_soc=values['soc'], bat_voltage=48.2, bat_current=-1.5,
                                bms_health=BmsStatus.BMS_HEALTH_GOOD)
                system = ChassisStatus(system_status_flags=values['raw'], manual_charging=True)
                comm = CommStatus(comm_state=(CommStatus.COMM_STATE_CONNECTED if values['connected']
                                             else CommStatus.COMM_STATE_DISCONNECTED))
                for kind, message in [('bms', bms), ('system', system), ('comm', comm)]:
                    message.header.stamp = mock.get_clock().now().to_msg()
                    if values['old_stamp'] and kind == 'system':
                        message.header.stamp.sec -= 5
                    pubs[kind].publish(message)
            timer = mock.create_timer(.05, publish)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
            config = dict(robot_host='127.0.0.1', control_port=port, timeout=5,
                          use_shared_key=False, key_file='')
            def wait_for(predicate):
                deadline = time.monotonic()+4.
                last = None
                while time.monotonic() < deadline:
                    last = status(config)['robot_status']['chassis_power']
                    if predicate(last):
                        return last
                    time.sleep(.03)
                self.fail('TCP电量状态未更新：'+str(last))
            try:
                power = wait_for(lambda p: p['available'])
                self.assertEqual(power['battery_percent'], 73)
                self.assertTrue(power['charging'])
                self.assertEqual(power['voltage_v'], 48.2)
                self.assertEqual(power['current_a'], -1.5)
                values['raw'] = 4
                power = wait_for(lambda p: p['charging'] is False and p['low_battery'] is True)
                # driver.manual_charging=True，但截图布局必须仅按整个uint32的bit3。
                self.assertEqual(power['charging_status_layout'], 'bitfield32')
                bridge.charging_layout = 'byte_flags'
                self.assertTrue(wait_for(lambda p: p['charging_status_layout'] == 'byte_flags')['charging'])
                bridge.charging_layout = 'bitfield32'
                values['soc'] = 0
                self.assertEqual(wait_for(lambda p: p['battery_percent'] == 0)['battery_percent'], 0)
                values['soc'] = 101
                self.assertIsNone(wait_for(lambda p: p['battery_percent'] is None)['battery_percent'])
                values.update(soc=73, old_stamp=True)
                power = wait_for(lambda p: p['battery_percent'] == 73 and p['charging'] is None)
                self.assertFalse(power['available'])
                values.update(connected=False, old_stamp=False)
                power = wait_for(lambda p: not p['connected'])
                self.assertIsNone(power['battery_percent'])
                self.assertIsNone(power['charging'])
                values['connected'] = True
                wait_for(lambda p: p['available'])
                timer.cancel()
                time.sleep(3.1)
                power = status(config)['robot_status']['chassis_power']
                self.assertIsNone(power['battery_percent'])
                self.assertEqual(power['charging_state'], 'unknown')
            finally:
                executor.shutdown()
                thread.join(timeout=3)
                bridge.close()
                bridge.destroy_node()
                mock.destroy_node()
                rclpy.shutdown()

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
            captures, home_records = [], []
            def pose(node, frame):
                value = np.eye(4)
                if frame == node.camera:
                    value[0, 3] = .2
                return value
            def captured(command, **kwargs):
                identity = command[command.index('--run-id')+1]
                captures.append(identity)
                digest = ('a' if len(captures) == 1 else 'c')*64
                images = root/'images'
                images.mkdir(exist_ok=True)
                (images/'.capture_index.json').write_text(json.dumps({'sessions': {identity: {
                    'directory': 'fixture', 'rgbd': {'capture_manifest_sha256': digest}}}}))
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
                def record_home(req, response):
                    home_records.append(True)
                    return success(req, response)
                detector.create_service(Trigger, '/detect_controller/record_home', record_home)
                detector.create_service(Trigger, '/detect_controller/start', start)
                detector.create_service(Trigger, '/detect_controller/cancel', success)
                detector.create_subscription(PoseArray, '/geminipro_calibration/targets_base', targets.append, 10)
                publisher = detector.create_publisher(String, '/detect_controller/status', 10)
                point_publisher = detector.create_publisher(String, '/detect_controller/point_result',
                    QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                sent_points = set()
                def broadcast():
                    elapsed = time.monotonic()-began[0] if began[0] is not None else -1.
                    state = 'IDLE' if elapsed < 0 else ('HOLD' if elapsed < .8 else 'DONE')
                    if len(starts) == 1 and elapsed >= .3:
                        state = 'RETRY_REQUIRED'
                    if len(starts) == 1:
                        completed = 0
                    else:
                        completed = (0 if elapsed < .4 else 1) if state == 'HOLD' else (2 if state == 'DONE' else 0)
                    publisher.publish(String(data=json.dumps({'state': state, 'batch_id': 'mock-batch', 'motion_ready': True,
                        'returned_home': state in ('DONE', 'RETRY_REQUIRED'),
                        'retry_context': {'code': 'travel_exhausted', 'message': 'mock extra travel used'} if state == 'RETRY_REQUIRED' else None,
                        'queued_points': len(targets[-1].poses) if targets else 0,
                        'completed_points': completed})))
                    if state == 'DONE':
                        for index in (1, 2):
                            if index in sent_points:
                                continue
                            reading = {'schema': 'paint_thickness_measurement_v1', 'value': 100.+index,
                                       'unit': 'unknown', 'sequence': index,
                                       'received_at_ros': {'sec': 100, 'nanosec': index}}
                            point_publisher.publish(String(data=json.dumps({
                                'schema': 'paint_detection_point_result_v1',
                                'result_id': 'mock-batch_'+str(index), 'batch_id': 'mock-batch',
                                'point_index': index, 'success': True, 'returned_home': True,
                                'measurement': reading})))
                            sent_points.add(index)
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
                    deadline = time.monotonic()+8.
                    while (len(captures) < 2 or task.phase != 'WAIT_RESULT') and time.monotonic() < deadline:
                        time.sleep(.02)
                    self.assertEqual(task.phase, 'WAIT_RESULT', task.detail)
                    self.assertEqual(task.retry_count, 1)
                    self.assertEqual(len(home_records), 1, '重拍不能重写本轮初始关节原位')
                    self.assertNotEqual(captures[0], captures[1])
                    # 重拍必须拒绝旧哈希，即使使用一个新的网络消息ID。
                    payload.update(message_id='stale-photo-on-retry')
                    self.assertTrue(request(client, payload)['ok'])
                    time.sleep(.2)
                    self.assertEqual(len(starts), 1)
                    payload.update(message_id='recaptured-photo', capture_manifest_sha256='c'*64)
                    self.assertTrue(request(client, payload)['ok'])
                    deadline = time.monotonic()+5.
                    while task.phase not in ('DONE', 'FAULT') and time.monotonic() < deadline:
                        time.sleep(.02)
                    self.assertEqual(task.phase, 'DONE', task.detail)
                    self.assertEqual(len(starts), 2)
                    self.assertEqual(len(home_records), 1)
                    deadline = time.monotonic()+3.
                    while len(bridge.detection_results) < 2 and time.monotonic() < deadline:
                        time.sleep(.02)
                    returned = status(client)['robot_status']['detection_results']
                    self.assertEqual([x['point_id'] for x in returned], ['p1', 'p2'])
                    self.assertEqual([x['measurement']['value'] for x in returned], [101., 102.])
                    from listen_robot import RobotListener
                    listener = RobotListener({'output_dir': str(root/'host'),
                                              'rgbd_bind': '127.0.0.1', 'rgbd_port': 0})
                    try:
                        listener.save_detection_results({'detection_results': returned})
                        listener.save_detection_results({'detection_results': returned})
                        self.assertEqual(len(list((root/'host/detection_results').glob('*.json'))), 2)
                    finally:
                        listener.server.close()
                    self.assertEqual(len(targets[-1].poses), 2)
                    first = targets[-1].poses[0].position
                    for actual, expected in zip([first.x, first.y, first.z], [.21, .02, .3]):
                        self.assertAlmostEqual(actual, expected)
                    self.assertEqual(request(client, payload)['disposition'], 'duplicate')
                    self.assertEqual(len(starts), 2)
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
