#!/usr/bin/env python3
"""Check device readiness, automatic admittance and chassis startup order."""
import os
from pathlib import Path
import time
import tempfile

import yaml
import unittest

os.environ["ROS_DOMAIN_ID"] = "188"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["ROS_LOG_DIR"] = "/tmp/auto-admittance-test-logs"

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, Twist, WrenchStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
from robot_task_control.workflow_controller import WorkflowController
from robot_task_control.velocity_mux import VelocityMux


class DeviceStartupTest(unittest.TestCase):
    def test_standard_route_versions_and_invalid_record_preserve_previous(self):
        import json
        from robot_task_control.standard_route import publish_standard
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            raw, target=folder/'raw.json',folder/'route_01/standard_route.json'
            document=dict(version=1,valid=True,finished=True,frame_id='odom',child_frame_id='base_link',
                          points=[[i*.01,0.,0.] for i in range(21)],station_events=[],
                          recording_config=dict(speed=.03))
            raw.write_text(json.dumps(document))
            first=publish_standard(raw,target,'first')
            first_bytes=target.read_bytes()
            self.assertTrue(first['quality']['accepted'])
            self.assertTrue((Path(first['version_dir'])/'preview.svg').exists())
            self.assertTrue((Path(first['version_dir'])/'points.csv').exists())
            raw.write_text(json.dumps(dict(document,finished=False)))
            with self.assertRaises(ValueError):
                publish_standard(raw,target,'bad')
            self.assertEqual(target.read_bytes(),first_bytes)
            raw.write_text(json.dumps(document))
            second=publish_standard(raw,target,'second')
            self.assertNotEqual(first['version_dir'],second['version_dir'])
            self.assertEqual((Path(first['version_dir'])/'standard_route.json').read_bytes(),first_bytes)
            with self.assertRaises(ValueError):
                publish_standard(raw,raw,'same-file')

    def test_pure_moveit_step_does_not_wait_for_admittance_service(self):
        from unittest.mock import MagicMock
        node = MagicMock()
        node.manage_admittance = False
        node.phase = 'ready'
        node.arm_ready.return_value = True
        node.execute_client.service_is_ready.return_value = True
        node.admittance_enable_client.service_is_ready.return_value = False
        node.current_step.return_value = {'control_mode': 'moveit'}
        node.get_clock.return_value.now.return_value.nanoseconds = 1000
        node.make_arm_target.return_value = PoseArray()
        node.make_arm_target.return_value.poses = [Pose()]
        node.arm_mode_ready.side_effect = lambda: WorkflowController.arm_mode_ready(node)
        WorkflowController.advance_arm_move(node, node.current_step(), 1000)
        self.assertEqual(node.phase, 'arm_delivery')
        self.assertEqual(node.arm_control_mode, 'moveit')
        node.target_publisher.publish.assert_called_once()
        node.admittance_enable_client.call_async.assert_not_called()

    def test_mux_latest_command_timeout_and_recovery(self):
        # 不启动ROS：确认减少队列后仍保留来源隔离、100ms超时和立即停车。
        from types import SimpleNamespace
        from unittest.mock import Mock, patch
        from robot_task_control import velocity_mux
        published = []
        logger = Mock()
        mux = SimpleNamespace(timeout=.1, enabled=True, source='line_follow', command=None,
            received=None, input_timed_out=False, last_tick=10., last_delay_log=float('-inf'),
            publisher=SimpleNamespace(publish=published.append), get_logger=lambda: logger)
        command = Twist()
        command.linear.x = .075
        with patch.object(velocity_mux, 'monotonic', return_value=10.):
            VelocityMux.on_command(mux, 'line_follow', command)
            VelocityMux.on_command(mux, 'pid', Twist())
        with patch.object(velocity_mux, 'monotonic', return_value=10.02):
            VelocityMux.tick(mux)
        self.assertEqual(published[-1].linear.x, .075)
        with patch.object(velocity_mux, 'monotonic', return_value=10.12):
            VelocityMux.tick(mux)
        self.assertEqual(published[-1].linear.x, 0.)
        self.assertTrue(mux.input_timed_out)
        command.linear.x = .03
        with patch.object(velocity_mux, 'monotonic', return_value=10.13):
            VelocityMux.on_command(mux, 'line_follow', command)
            VelocityMux.tick(mux)
        self.assertEqual(published[-1].linear.x, .03)
        self.assertFalse(mux.input_timed_out)
        mux.enabled = False
        VelocityMux.reset(mux)
        self.assertEqual(published[-1].linear.x, 0.)
        self.assertIsNone(mux.command)

    def test_arm_charge_recipe_first_marker_and_five_second_wait(self):
        flow = WorkflowController.load_workflow(Path(__file__).resolve().parents[1]/
                                                'config/line_follow_arm_charge_test.yaml')
        steps = flow['steps']
        self.assertEqual([step['action'] for step in steps],
            ['wait_chassis_stopped', 'arm_posture', 'line_follow', 'wait_chassis_stopped',
             'save_line_route', 'arm_posture', 'delay', 'arm_posture', 'line_follow', 'wait_chassis_stopped'])
        self.assertEqual([step['posture'] for step in steps if step['action'] == 'arm_posture'],
                         ['travel', 'detection_ready', 'travel'])
        self.assertTrue(steps[2]['finish_at_first_marker'])
        self.assertEqual(steps[6]['seconds'], 5.)
        self.assertEqual(steps[8]['drive_direction'], 'reverse')
        self.assertTrue(steps[8]['return_to_start'])

    def test_arm_charge_automatic_round_trip_keeps_route_and_waits_full_five_seconds(self):
        self.check_charge_round_trip(with_arm=True)

    def test_chassis_charge_automatic_round_trip_without_arm_services(self):
        self.check_charge_round_trip(with_arm=False)

    def test_teach_route_automatic_publish_and_return(self):
        self.check_charge_round_trip(with_arm=False, teach=True)

    def check_charge_round_trip(self, with_arm, teach=False):
        """真实本机ROS工作流，反馈为替身；纯底盘分支没有机械臂服务。"""
        import json
        from uuid import uuid4
        from nav_msgs.msg import Odometry
        config = Path(__file__).resolve().parents[1]/'config'/('line_follow_arm_charge_test.yaml'
            if with_arm else 'line_follow_charge_test.yaml')
        with tempfile.TemporaryDirectory() as directory:
            rclpy.init(args=['--ros-args', '-p', 'workflow_file:='+str(config),
                            '-p', 'auto_start:=true', '-p', 'use_velocity_mux:=true',
                            '-p', 'stop_pid_before_follow:=false', '-p', 'joint_posture_only:='+str(with_arm).lower(), '-p', 'manage_line_stream:='+str(with_arm).lower(),
                            '-p', 'check_devices_on_startup:=true', '-p', 'device_ready_stable_sec:=0.1'] +
                            (['-p','standard_route_output:='+str(Path(directory)/'standard_route.json')] if teach else []))
            node, fake = WorkflowController(), Node('line_follow_controller')
            node.arm_ready = lambda: True
            node.tf_buffer.lookup_transform = lambda *args: object()
            fake.declare_parameter('drive_direction', 'forward')
            fake.declare_parameter('reverse_return_to_start', False)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            executor.add_node(fake)
            retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            arm_status = fake.create_publisher(String, '/arm_joint_postures/status', retained)
            follow_status = fake.create_publisher(String, '/line_follow/status', 10)
            odom_pub = fake.create_publisher(Odometry, '/wheel_odom', 10)
            events, pending = [], []
            enabled, mux, pose = [False], [False], [None]
            ticks, ready_at, route_saved = [0], [None], [False]
            snapshot = Path(directory)/'sealed-route.json'
            snapshot_payload = (json.dumps(dict(version=1,valid=True,finished=True,frame_id='odom',
                child_frame_id='base_link',points=[[i*.01,0.,0.] for i in range(21)],
                recording_config=dict(control=dict(forward_speed=.03)))) if teach else 'sealed-forward-route')
            camera, camera_ready_at, stream_events = [True], [0.], []
            def stream(req, response):
                camera[0] = req.data
                camera_ready_at[0] = time.monotonic()+.2
                stream_events.append(req.data)
                response.success = True
                return response
            def vision_ready(req, response):
                response.success = camera[0] and time.monotonic() >= camera_ready_at[0]
                return response
            fake.create_service(SetBool, '/line_camera/camera_node/streaming', stream)
            fake.create_service(Trigger, '/line_follow_controller/ready', vision_ready)
            def arm(name):
                def callback(req, response):
                    self.assertFalse(enabled[0] and mux[0], '机械臂运动时底盘仍放行')
                    identity = uuid4().hex
                    response.success, response.message = True, json.dumps({'request_id': identity})
                    arm_status.publish(String(data=json.dumps(dict(request_id=identity, state='QUEUED', posture=name))))
                    pending.append((time.monotonic()+.15, identity, name))
                    events.append('arm:'+name)
                    return response
                return callback
            if with_arm:
                for name in ('travel', 'detection_ready'):
                    fake.create_service(Trigger, '/arm_joint_postures/'+name, arm(name))
            def mux_enable(req, response):
                mux[0], response.success = req.data, True
                return response
            def okay(req, response):
                response.success = True
                return response
            def follow_enable(req, response):
                direction = fake.get_parameter('drive_direction').value
                if req.data:
                    self.assertTrue(mux[0], "速度选择器尚未放行就启用循迹")
                    if with_arm:
                        self.assertEqual(pose[0], 'travel', '行驶位尚未到位就出发')
                    ticks[0] = 0
                    events.append('drive:'+direction)
                    self.assertTrue(camera[0] and time.monotonic() >= camera_ready_at[0])
                    if direction == 'reverse':
                        self.assertTrue(route_saved[0])
                        self.assertEqual(snapshot.read_text(), snapshot_payload)
                        self.assertGreaterEqual(time.monotonic()-ready_at[0], 5.)
                        self.assertTrue(fake.get_parameter('reverse_return_to_start').value)
                enabled[0], response.success = req.data, True
                return response
            def save_route(req, response):
                self.assertFalse(enabled[0])
                self.assertFalse(mux[0], "到站等待前未锁定底盘输出")
                snapshot.write_text(snapshot_payload)
                route_saved[0] = True
                if not with_arm:
                    ready_at[0] = time.monotonic()
                events.append('sealed')
                response.success, response.message = True, json.dumps({'snapshot_file': str(snapshot)})
                return response
            fake.create_service(SetBool, '/chassis_velocity_mux/enable', mux_enable)
            fake.create_service(SetBool, '/chassis_velocity_mux/line_follow_mode', okay)
            fake.create_service(SetBool, '/line_follow_controller/enable', follow_enable)
            fake.create_service(Trigger, '/line_follow_controller/save_route', save_route)
            def tick():
                odom = Odometry()
                odom.header.stamp = fake.get_clock().now().to_msg()
                odom_pub.publish(odom)
                if pending and time.monotonic() >= pending[0][0]:
                    _, identity, posture = pending.pop(0)
                    pose[0] = posture
                    if posture == 'detection_ready':
                        ready_at[0] = time.monotonic()
                    arm_status.publish(String(data=json.dumps(dict(request_id=identity, state='DONE', posture=posture))))
                if enabled[0]:
                    ticks[0] += 1
                    direction = fake.get_parameter('drive_direction').value
                    if direction == 'forward':
                        state = 'tracking' if ticks[0] < 5 else 'stop_marker_intermediate'
                    else:
                        state = ('reverse_visual_finish_tracking' if ticks[0] < 5 else
                                 'charging_dock_confirming' if ticks[0] < 8 else 'reverse_route_complete')
                    follow_status.publish(String(data=json.dumps({'state': state})))
            fake.create_timer(.04, tick)
            try:
                deadline = time.monotonic()+23.
                while node.active and not node.failed and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                self.assertFalse(node.failed, node.last_failure)
                self.assertFalse(node.active, '自动往返测试未结束')
                expected = (['arm:travel', 'drive:forward', 'sealed', 'arm:detection_ready',
                             'arm:travel', 'drive:reverse'] if with_arm else ['drive:forward', 'sealed', 'drive:reverse'])
                self.assertEqual(events, expected)
                self.assertFalse(mux[0])
                self.assertEqual(node.route_checkpoint_file, str(snapshot))
                self.assertEqual(stream_events, [False, True, False, True] if with_arm else [])
                if teach:
                    standard=json.loads((Path(directory)/'standard_route.json').read_text())
                    self.assertTrue(standard['standard_route']['quality']['accepted'])
                    self.assertEqual(standard['recording_config']['control']['forward_speed'],.03)
                    self.assertTrue((Path(standard['standard_route']['version_dir'])/'points.csv').exists())
            finally:
                executor.shutdown()
                node.destroy_node()
                fake.destroy_node()
                rclpy.shutdown()

    def test_station_recipe_gates_each_chassis_move_and_reuses_station_points(self):
        path = Path(__file__).resolve().parents[1]/'config/line_follow_arm_test.yaml'
        flow = WorkflowController.load_workflow(path)
        actions = [step['action'] for step in flow['steps']]
        self.assertEqual(actions[:3], ['wait_chassis_stopped', 'arm_posture', 'line_follow'])
        self.assertEqual(flow['steps'][1]['posture'], 'travel')
        self.assertTrue(flow['steps'][2]['service_intermediate_stations'])
        station = flow['station_steps']
        self.assertEqual(station[0]['action'], 'wait_chassis_stopped')
        self.assertEqual(station[1]['posture'], 'detection_ready')
        self.assertEqual(station[-1]['posture'], 'detection_ready')
        self.assertEqual(len([s for s in station if s['action'] == 'arm_move']), 4)
        self.assertEqual(flow['steps'][3:], station)
        self.assertNotIn('final_capture', actions)

    def test_posture_gate_rejects_old_ids_wrong_postures_and_stale_status(self):
        from unittest.mock import Mock
        node = Mock()
        node.phase, node.posture_request_id = 'posture_wait', 'current'
        node.posture_status_time = time.monotonic()
        for state, identity, posture in [('DONE', 'old', 'travel'),
                                          ('DONE', 'current', 'detection_ready'),
                                          ('QUEUED', 'current', 'travel'),
                                          ('FAILED', 'old', 'travel')]:
            node.posture_status = dict(state=state, request_id=identity, posture=posture)
            WorkflowController.advance_arm_posture(node, 'travel')
            node.complete_step.assert_not_called()
            node.fail_workflow.assert_not_called()
        node.posture_status = dict(state='DONE', request_id='current', posture='travel')
        WorkflowController.advance_arm_posture(node, 'travel')
        node.complete_step.assert_called_once()
        node.complete_step.reset_mock()
        node.posture_status_time = time.monotonic()-4.
        WorkflowController.advance_arm_posture(node, 'travel')
        node.complete_step.assert_not_called()
        node.fail_workflow.assert_called_once()

    def test_joint_posture_service_queues_once_and_skips_an_arrived_target(self):
        from unittest.mock import Mock
        from robot_task_control.joint_posture_control import JointPostureControl
        node = Mock()
        node.busy, node.abort_reason, node.report = False, None, {}
        response = JointPostureControl.request_posture(node, 'travel', Trigger.Response())
        self.assertTrue(response.success)
        self.assertTrue(node.busy)
        self.assertEqual(node.requested_posture, 'travel')
        self.assertFalse(JointPostureControl.request_posture(
            node, 'detection_ready', Trigger.Response()).success)
        node.postures = dict(travel=[0.]*6, detection_ready=[1.]*6)
        node.current.return_value = [0.]*6
        JointPostureControl.move_posture(node, 'travel')
        node.write.assert_called_with('DONE')
        node.plan.assert_not_called()
        node.prepare_control_mode.assert_called_once()
        node.wait_ready.assert_called_once()
        node.current.return_value = [2.]*6
        JointPostureControl.move_posture(node, 'travel')
        self.assertEqual([call.args[:2] for call in node.plan.call_args_list],
                         [([2.]*6, [1.]*6), ([1.]*6, [0.]*6)])
        node.execute_segments.assert_called_once()

    def test_station_wait_keeps_route_and_requires_actual_posture_completion(self):
        """真实ROS服务/话题的本机模拟：两途中站+最终站；无物理驱动。"""
        import json
        from std_msgs.msg import Empty
        from nav_msgs.msg import Odometry
        from uuid import uuid4
        config = Path(__file__).resolve().parents[1]/'config/line_follow_arm_test.yaml'
        flow = WorkflowController.load_workflow(config)
        # 这里只测站点调度，机械臂轨迹由独立GenericSystem测试验证。
        station = [dict(action='delay', seconds=.15),
                   dict(action='arm_posture', posture='detection_ready', step_timeout_sec=5.)]
        flow['station_steps'] = station
        flow['steps'] = [dict(action='arm_posture', posture='travel', step_timeout_sec=5.),
                         dict(action='line_follow', until_untrusted=True, require_final_marker=True,
                              service_intermediate_stations=True, progress_timeout_sec=3.,
                              step_timeout_sec=15.), *station]
        # load_workflow会自动给每个line_follow再加一次行驶位，验证跳过已到位仍须确认。
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'workflow.yaml'
            path.write_text(yaml.safe_dump({'workflow': flow}))
            rclpy.init(args=['--ros-args', '-p', 'workflow_file:='+str(path),
                            '-p', 'auto_start:=true', '-p', 'use_velocity_mux:=true',
                            '-p', 'stop_pid_before_follow:=false'])
            fake, node = Node('station_mock'), WorkflowController()
            executor = SingleThreadedExecutor()
            executor.add_node(fake)
            executor.add_node(node)
            events, pending = [], []
            mux_enabled, following, stage = [False], [False], [0]
            follow_enable_calls, ready_count, start_count = [], [0], [0]
            status = fake.create_publisher(String, '/arm_joint_postures/status',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            follow_status = fake.create_publisher(String, '/line_follow/status', 10)
            resume = fake.create_publisher(Empty, '/line_follow/resume', 10)
            def posture(name):
                def callback(req, response):
                    self.assertFalse(mux_enabled[0] and following[0], '机械臂运动时底盘仍放行')
                    identity = uuid4().hex
                    response.success, response.message = True, json.dumps({'request_id': identity})
                    status.publish(String(data=json.dumps(dict(request_id=identity, state='QUEUED', posture=name))))
                    pending.append((time.monotonic()+.15, identity, name))
                    events.append('request:'+name)
                    return response
                return callback
            for name in ('travel', 'detection_ready'):
                fake.create_service(Trigger, '/arm_joint_postures/'+name, posture(name))
            def set_mux(req, response):
                mux_enabled[0] = req.data
                events.append('mux:'+str(req.data))
                response.success = True
                return response
            def set_follow(req, response):
                following[0] = req.data
                follow_enable_calls.append(req.data)
                response.success = True
                return response
            def okay(req, response):
                response.success = True
                return response
            fake.create_service(SetBool, '/chassis_velocity_mux/enable', set_mux)
            fake.create_service(SetBool, '/chassis_velocity_mux/line_follow_mode', okay)
            fake.create_service(SetBool, '/line_follow_controller/enable', set_follow)
            def resumed(msg):
                if following[0] and mux_enabled[0] and node.station_context is None:
                    stage[0] += 1
                    events.append('resume')
            fake.create_subscription(Empty, '/line_follow/resume', resumed, 10)
            tracking_ticks = [0]
            def tick():
                if pending and time.monotonic() >= pending[0][0]:
                    _, identity, name = pending.pop(0)
                    status.publish(String(data=json.dumps(dict(request_id=identity, state='DONE', posture=name))))
                    events.append('done:'+name)
                    if name == 'detection_ready':
                        ready_count[0] += 1
                if following[0]:
                    tracking_ticks[0] += 1
                    # 两个途中站，然后终点。每次继续后先发布几帧tracking以解除站点重触发抑制。
                    label = ('tracking' if tracking_ticks[0] % 12 < 5 else
                             ('stop_marker_final' if stage[0] >= 2 else 'stop_marker_intermediate'))
                    if node.station_context is not None:
                        label = 'stop_marker_intermediate'
                    follow_status.publish(String(data=json.dumps({'state': label})))
                if node.phase == 'station_wait_start' and start_count[0] < ready_count[0]:
                    self.assertFalse(mux_enabled[0])
                    response = node.on_start_request(Trigger.Request(), Trigger.Response())
                    self.assertTrue(response.success)
                    start_count[0] += 1
                if node.phase == 'station_resume_mux':
                    self.assertFalse(mux_enabled[0], '行驶位确认以前放行')
            fake.create_timer(.03, tick)
            try:
                deadline = time.monotonic()+14.
                while time.monotonic() < deadline and node.active and not node.failed:
                    executor.spin_once(timeout_sec=.01)
                self.assertFalse(node.failed, node.last_failure)
                self.assertFalse(node.active, '站点调度没有完成')
                self.assertEqual(start_count[0], 2)
                self.assertEqual(ready_count[0], 3)
                self.assertEqual(follow_enable_calls, [True, False], '途中站重启了循迹，路线会丢失')
                self.assertFalse(mux_enabled[0], '最终站未锁住底盘')
                self.assertEqual(events.count('resume'), 2)
                for i, event in enumerate(events):
                    if event == 'resume':
                        self.assertEqual(events[i-1], 'mux:True')
                        self.assertIn('done:travel', events[:i])
            finally:
                executor.shutdown()
                node.destroy_node()
                fake.destroy_node()
                rclpy.shutdown()

    def test_dashboard_automatic_power_brake_sequence(self):
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'eli_cs_robot_driver/scripts'))
        from auto_power_on import DashboardStartup
        from eli_common_interface.srv import GetRobotMode, GetSafetyMode
        from eli_common_interface.msg import RobotMode, SafetyMode
        for initial, safety, power_ok, brake_ok, expected, result in (
                (RobotMode.POWER_OFF, SafetyMode.NORMAL, True, True, ['power', 'brake'], 0),
                (RobotMode.IDLE, SafetyMode.NORMAL, True, True, ['brake'], 0),
                (RobotMode.RUNNING, SafetyMode.NORMAL, True, True, [], 0),
                (RobotMode.POWER_OFF, SafetyMode.NORMAL, False, True, ['power'], 1),
                (RobotMode.POWER_OFF, SafetyMode.PROTECTIVE_STOP, True, True, [], 1),
                (RobotMode.IDLE, SafetyMode.NORMAL, True, False, ['brake'], 1)):
            with self.subTest(initial=initial, safety=safety, power_ok=power_ok):
                rclpy.init(args=['--ros-args', '-p', 'startup_timeout_sec:=4.0',
                                '-p', 'request_timeout_sec:=1.0', '-p', 'poll_interval_sec:=0.01'])
                dashboard, startup = Node('mock_dashboard'), DashboardStartup()
                executor = SingleThreadedExecutor()
                executor.add_node(dashboard)
                executor.add_node(startup)
                current, calls = [initial], []
                deferred = [None, 0.]
                def get_mode(request, response):
                    if deferred[0] is not None and time.monotonic() >= deferred[1]:
                        current[0], deferred[0] = deferred[0], None
                    response.success = True
                    response.mode.mode = current[0]
                    return response
                def get_safety(request, response):
                    response.success = True
                    response.mode.mode = safety
                    return response
                def power(request, response):
                    calls.append('power')
                    response.success = power_ok
                    if power_ok:
                        current[0] = RobotMode.POWER_ON
                        deferred[:] = [RobotMode.IDLE, time.monotonic()+.15]
                    return response
                def brake(request, response):
                    self.assertEqual(current[0], RobotMode.IDLE)
                    calls.append('brake')
                    if brake_ok:
                        deferred[:] = [RobotMode.RUNNING, time.monotonic()+.15]
                    response.success = brake_ok
                    return response
                dashboard.create_service(GetRobotMode, '/dashboard_client/robot_mode', get_mode)
                dashboard.create_service(GetSafetyMode, '/dashboard_client/get_safety_mode', get_safety)
                dashboard.create_service(Trigger, '/dashboard_client/power_on', power)
                dashboard.create_service(Trigger, '/dashboard_client/brake_release', brake)
                try:
                    deadline = time.monotonic()+5.
                    while startup.result is None and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=.01)
                    self.assertEqual(startup.result, result)
                    self.assertEqual(calls, expected)
                finally:
                    executor.shutdown()
                    startup.destroy_node()
                    dashboard.destroy_node()
                    rclpy.shutdown()

    def test_dashboard_startup_timeouts_are_bounded(self):
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'eli_cs_robot_driver/scripts'))
        from auto_power_on import DashboardStartup
        from rclpy.task import Future
        rclpy.init(args=['--ros-args', '-p', 'startup_timeout_sec:=0.2'])
        startup = DashboardStartup()
        try:
            end = time.monotonic()+1.
            while startup.result is None and time.monotonic() < end:
                rclpy.spin_once(startup, timeout_sec=.01)
            self.assertEqual(startup.result, 1, 'missing dashboard services waited forever')
            # 上电/抱闸服务超过原来的10秒仍应等待；普通状态查询仍受短超时约束。
            from unittest.mock import Mock, patch
            for kind in ('power', 'brake'):
                startup.result = None
                startup.pending = None
                startup.phase = kind
                startup.next_check = 0.
                startup.deadline = 1000.
                startup.dashboard_clients[kind] = Mock()
                startup.dashboard_clients[kind].call_async.return_value = Future()
                with patch('auto_power_on.time.monotonic', return_value=100.):
                    startup.tick()
                self.assertEqual(startup.request_deadline, 220.)
                with patch('auto_power_on.time.monotonic', return_value=111.):
                    startup.tick()
                self.assertIsNone(startup.result)
                self.assertFalse(startup.pending.done())
                with patch('auto_power_on.time.monotonic', return_value=221.):
                    startup.tick()
                self.assertEqual(startup.result, 1)
            startup.result = None
            startup.deadline = time.monotonic()+1.
            startup.pending, startup.pending_kind = Future(), 'mode'
            startup.request_deadline = time.monotonic()-1.
            startup.tick()
            self.assertEqual(startup.result, 1)
            self.assertFalse(startup.power_sent)
            self.assertFalse(startup.brake_sent)
        finally:
            startup.destroy_node()
            rclpy.shutdown()

    def test_protective_stop_requires_confirmation_and_retains_lock_on_failure(self):
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'eli_cs_robot_driver/scripts'))
        from protective_stop_guard import ProtectiveStopGuard
        from eli_common_interface.srv import GetSafetyMode, GetRobotMode
        from sensor_msgs.msg import JointState
        from controller_manager_msgs.srv import SwitchController
        for unlock_ok in (True, False):
            with self.subTest(unlock_ok=unlock_ok), tempfile.TemporaryDirectory() as directory:
                rclpy.init(args=['--ros-args', '-p', f'state_file:={directory}/guard.json'])
                guard, mock = ProtectiveStopGuard(), Node('mock_recovery_robot')
                executor = SingleThreadedExecutor()
                executor.add_node(guard)
                executor.add_node(mock)
                calls, safety, active = [], [3], [False]
                mock.create_subscription(Bool, '/robot_control/safety_inhibit', lambda msg: None,
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                def trigger(name, callback=None):
                    def handle(req, response):
                        calls.append(name)
                        response.success = True
                        if callback:
                            callback(response)
                        return response
                    return handle
                def unlock(response):
                    response.success = unlock_ok
                    if unlock_ok:
                        safety[0] = 1
                def mode(req, response):
                    response.success, response.mode.mode = True, 7
                    return response
                def get_safety(req, response):
                    response.success, response.mode.mode = True, safety[0]
                    return response
                def controllers(req, response):
                    response.controller = [ControllerState(name='scaled_joint_trajectory_controller',
                        state='active' if active[0] else 'inactive')]
                    return response
                def release(response):
                    self.assertTrue(active[0])
                def switch(req, response):
                    if 'scaled_joint_trajectory_controller' in req.activate_controllers:
                        active[0] = True
                    response.ok = True
                    return response
                mock.create_service(SwitchController, '/controller_manager/switch_controller', switch)
                def disable(req, response):
                    calls.append('disable')
                    self.assertFalse(req.data)
                    response.success = True
                    return response
                mock.create_service(SetBool, '/admittance_controller/enable', disable)
                mock.create_service(GetSafetyMode, '/dashboard_client/get_safety_mode', get_safety)
                mock.create_service(GetRobotMode, '/dashboard_client/robot_mode', mode)
                mock.create_service(ListControllers, '/controller_manager/list_controllers', controllers)
                mock.create_service(Trigger, '/dashboard_client/stop', trigger('stop', lambda r: setattr(guard, 'running', False)))
                mock.create_service(Trigger, '/dashboard_client/unlock_protective_stop', trigger('unlock', unlock))
                mock.create_service(Trigger, '/io_and_status_controller/resend_external_script',
                    trigger('resend', lambda r: setattr(guard, 'running', True)))
                mock.create_service(Trigger, '/controller_stopper/release_safety_latch', trigger('release', release))
                mock.create_service(Trigger, '/task_controller/safety_pause', trigger('pause'))
                mock.create_service(Trigger, '/task_controller/safety_resume', trigger('resume'))
                client = mock.create_client(Trigger, '/protective_stop_guard/confirm_recovery')
                def pump(seconds, fresh=True):
                    end = time.monotonic()+seconds
                    while time.monotonic() < end:
                        if fresh:
                            guard.on_joints(JointState(name=[f'j{i}' for i in range(6)], position=[0.]*6))
                            guard.on_force(WrenchStamped())
                        executor.spin_once(timeout_sec=.01)
                try:
                    pump(1.8)
                    guard.busy = True
                    duplicate = client.call_async(Trigger.Request())
                    pump(.2)
                    self.assertTrue(duplicate.done())
                    self.assertFalse(duplicate.result().success)
                    guard.busy = False
                    self.assertTrue(guard.latched)
                    self.assertNotIn('unlock', calls, 'an alarm must never unlock automatically')
                    guard.force = 10.
                    self.assertFalse(guard.force_ready())
                    guard.force = 0.
                    self.assertTrue(guard.force_ready())
                    guard.joints_at = time.monotonic()-1.
                    stale = client.call_async(Trigger.Request())
                    pump(.2, False)
                    self.assertTrue(stale.done())
                    self.assertFalse(stale.result().success)
                    self.assertNotIn('unlock', calls)
                    pump(1.2)
                    result = client.call_async(Trigger.Request())
                    end = time.monotonic()+8.
                    while not result.done() and time.monotonic() < end:
                        pump(.02)
                    self.assertTrue(result.done(), guard.detail)
                    self.assertEqual(result.result().success, unlock_ok, guard.detail)
                    self.assertEqual(calls.count('unlock'), 1)
                    self.assertEqual(guard.latched, not unlock_ok)
                    self.assertEqual(('resume' in calls), unlock_ok)
                    if unlock_ok:
                        self.assertLess(calls.index('pause'), calls.index('unlock'))
                        self.assertLess(calls.index('release'), calls.index('resume'))
                    else:
                        pump(.5)
                        self.assertEqual(calls.count('unlock'), 1, 'failed unlock was retried')
                    saved = __import__('json').loads(Path(directory, 'guard.json').read_text())
                    self.assertEqual(saved['latched'], not unlock_ok)
                finally:
                    executor.shutdown()
                    guard.destroy_node()
                    mock.destroy_node()
                    rclpy.shutdown()

    def test_safety_pause_preserves_absolute_arm_goal_and_restarts_hold_at_current_pose(self):
        from unittest.mock import patch
        from geometry_msgs.msg import Pose, PoseArray
        rclpy.init(args=['--ros-args', '-p', 'workflow_file:='+str(
            Path(__file__).resolve().parents[1]/'config/arm_admittance_only.yaml')])
        node = WorkflowController()
        try:
            with tempfile.TemporaryDirectory() as directory, patch('pathlib.Path.home', return_value=Path(directory)):
                node.active, node.phase, node.arm_control_mode = True, 'arm_tracking', 'admittance'
                node.pending_arm_target = PoseArray()
                node.pending_arm_target.header.frame_id = 'base_link'
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = .31, .24, .56
                pose.orientation.w = 1.
                node.pending_arm_target.poses = [pose]
                generation = node.generation
                node.on_safety_inhibit(Bool(data=True))
                self.assertFalse(node.active)
                self.assertGreater(node.generation, generation)
                self.assertEqual(node.safety_checkpoint['target'].poses[0].position.x, .31)
                rejected = node.on_start_request(Trigger.Request(), Trigger.Response())
                self.assertFalse(rejected.success)
                node.on_safety_inhibit(Bool(data=False))
                resumed = node.on_safety_resume(Trigger.Request(), Trigger.Response())
                self.assertTrue(resumed.success)
                self.assertEqual(node.recovery_arm_target.poses[0].position.x, .31)
                self.assertEqual(node.phase, 'ready')
                transform = TransformStamped()
                transform.transform.translation.x = .305
                transform.transform.translation.y = .24
                transform.transform.translation.z = .56
                transform.transform.rotation.w = 1.
                with patch.object(node.tf_buffer, 'lookup_transform', return_value=transform):
                    target = node.make_arm_target(node.current_step())
                self.assertIsNotNone(target)
                self.assertAlmostEqual(target.poses[0].position.x, .31)
                # 柔顺任务完成后发生报警，恢复只能重新捕获当前姿态。
                node.recovery_arm_target = None
                node.active = False
                node.step_index = len(node.steps)
                node.on_safety_inhibit(Bool(data=True))
                node.on_safety_inhibit(Bool(data=False))
                resumed = node.on_safety_resume(Trigger.Request(), Trigger.Response())
                self.assertTrue(resumed.success)
                self.assertEqual(node.current_step()['target_pose']['position'], [0., 0., 0.])
                self.assertIsNone(node.recovery_arm_target)
                node.complete_step()
                self.assertFalse(node.active)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_controller_stopper_does_not_reactivate_after_safety_alarm(self):
        import subprocess
        from ament_index_python.packages import get_package_prefix
        from controller_manager_msgs.srv import SwitchController
        from eli_common_interface.srv import GetRobotMode
        from eli_common_interface.msg import SafetyMode
        rclpy.init()
        mock = Node('mock_stopper_services')
        active, changes = [True], []
        def mode(req, response):
            response.success, response.mode.mode = True, 7
            return response
        def controllers(req, response):
            response.controller = [ControllerState(name='scaled_joint_trajectory_controller',
                state='active' if active[0] else 'inactive')]
            return response
        def switch(req, response):
            changes.extend(('activate', c) for c in req.activate_controllers)
            changes.extend(('deactivate', c) for c in req.deactivate_controllers)
            if 'scaled_joint_trajectory_controller' in req.activate_controllers:
                active[0] = True
            if 'scaled_joint_trajectory_controller' in req.deactivate_controllers:
                active[0] = False
            response.ok = True
            return response
        mock.create_service(GetRobotMode, '/dashboard_client/robot_mode', mode)
        mock.create_service(ListControllers, '/controller_manager/list_controllers', controllers)
        mock.create_service(SwitchController, '/controller_manager/switch_controller', switch)
        safety_pub = mock.create_publisher(SafetyMode, '/io_and_status_controller/safety_mode', 10)
        running_pub = mock.create_publisher(Bool, '/io_and_status_controller/robot_task_running', 10)
        release = mock.create_client(Trigger, '/controller_stopper/release_safety_latch')
        executable = Path(get_package_prefix('eli_cs_robot_driver'))/'lib/eli_cs_robot_driver/controller_stopper_node'
        with tempfile.TemporaryFile(mode='w+') as output:
            process = subprocess.Popen([str(executable), '--ros-args', '-r', '__node:=controller_stopper',
                '-p', 'headless_mode:=true', '-p', 'consistent_controllers:=[joint_state_broadcaster]'],
                stdout=output, stderr=output)
            def pump(seconds, safety=1, running=True):
                end = time.monotonic()+seconds
                while time.monotonic() < end:
                    safety_pub.publish(SafetyMode(mode=safety))
                    running_pub.publish(Bool(data=running))
                    rclpy.spin_once(mock, timeout_sec=.01)
            try:
                pump(1.5)
                self.assertIsNone(process.poll())
                pump(.8, 3, False)
                if active[0]:
                    output.seek(0)
                    self.fail('stopper did not deactivate: '+output.read()[-3000:])
                pump(.8, 1, True)
                self.assertFalse(active[0], 'external control resumed the old trajectory controller')
                self.assertFalse(any(kind == 'activate' for kind, _ in changes))
                self.assertTrue(release.service_is_ready())
                future = release.call_async(Trigger.Request())
                pump(.3)
                self.assertTrue(future.done())
                self.assertTrue(future.result().success)
                self.assertFalse(active[0], 'releasing latch must not activate unverified controllers')
            finally:
                process.terminate()
                try:
                    process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3.)
                mock.destroy_node()
                rclpy.shutdown()

    def test_all_devices_before_any_task(self):
        config = (
            Path(__file__).resolve().parents[1]
            / "test/fixtures/arm_admittance_line_follow.yaml"
        )
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                f"workflow_file:={config}",
                "-p",
                "auto_start:=true",
                "-p",
                "require_arm_ready:=true",
                "-p",
                "check_devices_on_startup:=true",
                "-p",
                "use_velocity_mux:=true",
                "-p",
                "stop_pid_before_follow:=false",
                "-p",
                "enabled_on_startup:=false",
            ]
        )
        executor = SingleThreadedExecutor()
        fake = Node("fake_arm_endpoints")
        controller = WorkflowController()
        executor.add_node(fake)
        executor.add_node(controller)
        mux = VelocityMux()
        executor.add_node(mux)
        flags = {
            "controllers": False,
            "task": False,
            "wrench": False,
            "line_ready": False,
            "following": False,
        }
        calls, targets, commands, follow_calls = [], [], [], []
        fake.create_subscription(Twist, "/cmd_vel", commands.append, 10)
        follow_command = fake.create_publisher(
            Twist, "/robot_control/line_follow/cmd_vel", 10
        )
        follow_status = fake.create_publisher(String, "/line_follow/status", 10)
        tf = TransformBroadcaster(fake)
        wrench = fake.create_publisher(
            WrenchStamped, "/admittance_controller/measured_wrench", 10
        )
        task = fake.create_publisher(
            Bool,
            "/io_and_status_controller/robot_task_running",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        def list_controllers(request, response):
            for name in (
                "scaled_joint_trajectory_controller",
                "joint_state_broadcaster",
                "speed_scaling_state_broadcaster",
                "tcp_pose_broadcaster",
            ):
                response.controller.append(
                    ControllerState(
                        name=name,
                        state="active" if flags["controllers"] else "inactive",
                    )
                )
            return response

        def enable(request, response):
            calls.append(request.data)
            response.success = True
            return response

        def ready(request, response):
            response.success = flags["line_ready"]
            response.message = "ready" if response.success else "camera_timeout"
            return response

        def follow_enable(request, response):
            if request.data:
                self.assertEqual(calls, [True])
                self.assertTrue(targets)
            follow_calls.append(request.data)
            flags["following"] = request.data
            response.success = True
            return response

        fake.create_service(Trigger, "/line_follow_controller/ready", ready)
        fake.create_service(SetBool, "/line_follow_controller/enable", follow_enable)

        def feedback():
            transform = TransformStamped()
            transform.header.stamp = fake.get_clock().now().to_msg()
            transform.header.frame_id, transform.child_frame_id = "base_link", "tool0"
            transform.transform.translation.x = 0.4
            transform.transform.translation.z = 0.3
            transform.transform.rotation.w = 1.0
            tf.sendTransform(transform)
            task.publish(Bool(data=flags["task"]))
            command = Twist()
            command.linear.x = 0.01 if flags["following"] else 0.0
            follow_command.publish(command)
            follow_status.publish(
                String(
                    data=(
                        '{"state":"tracking"}'
                        if flags["following"]
                        else '{"state":"disabled"}'
                    )
                )
            )
            if flags["wrench"]:
                wrench.publish(WrenchStamped(header=transform.header))

        fake.create_service(
            ListControllers, "/controller_manager/list_controllers", list_controllers
        )
        fake.create_service(SetBool, "/admittance_controller/enable", enable)
        fake.create_subscription(
            PoseArray, "/admittance_controller/target_poses", targets.append, 10
        )
        fake.create_timer(0.02, feedback)

        def pump(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)

        try:
            flags["wrench"] = True
            pump(1.3)
            self.assertEqual(calls, [])
            flags["controllers"] = True
            pump(1.3)
            self.assertEqual(calls, [])  # External robot task still stopped.
            flags["wrench"] = False
            pump(0.7)
            flags["task"] = True
            pump(0.7)
            self.assertEqual(calls, [])  # No fresh post-bias wrench.
            flags["wrench"] = True
            pump(1.0)
            self.assertEqual(
                calls, []
            )  # Arm ready, camera/chassis check still failing.
            self.assertFalse(mux.enabled)
            self.assertFalse(any(msg.linear.x for msg in commands))
            flags["line_ready"] = True
            pump(0.5)
            self.assertEqual(calls, [])  # Not stable for one second yet.
            flags["line_ready"] = False
            pump(0.4)
            self.assertEqual(calls, [])
            flags["line_ready"] = True
            deadline = time.monotonic() + 8
            while controller.active and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
            self.assertFalse(controller.active)
            self.assertFalse(controller.failed)
            self.assertEqual(calls, [True])
            self.assertEqual(len(targets), 1)
            self.assertAlmostEqual(targets[0].poses[0].position.x, 0.4)
            self.assertAlmostEqual(targets[0].poses[0].position.z, 0.3)
            self.assertTrue(controller.devices_checked)
            self.assertEqual(follow_calls, [True, False])
            self.assertTrue(any(msg.linear.x > 0 for msg in commands))
            self.assertEqual(commands[-1].linear.x, 0.0)
            controller.on_stop_request(Trigger.Request(), Trigger.Response())
            pump(0.3)
            self.assertEqual(calls, [True, False])
            self.assertFalse(controller.active)
        finally:
            executor.shutdown()
            controller.destroy_node()
            mux.destroy_node()
            fake.destroy_node()
            rclpy.shutdown()

    def test_force_readiness_depends_on_workflow_control_modes(self):
        """位置规划不等待力数据，含导纳的流程仍必须等待；接口缺失不能放行。"""
        for modes in (("moveit",), ("admittance",), ("moveit", "admittance")):
            with self.subTest(modes=modes), tempfile.TemporaryDirectory() as directory:
                config = Path(directory) / "workflow.yaml"
                config.write_text(yaml.safe_dump({"workflow": {
                    "steps": [{
                        "action": "arm_move", "control_mode": mode,
                        "target_pose": {"position": [0.0, 0.0, 0.0], "orientation": "current"},
                    } for mode in modes],
                }}))
                rclpy.init(args=["--ros-args", "-p", f"workflow_file:={config}",
                                 "-p", "device_ready_stable_sec:=0.1"])
                fake = Node("fake_mode_readiness")
                controller = WorkflowController()
                executor = SingleThreadedExecutor()
                executor.add_node(fake)
                executor.add_node(controller)
                broadcaster = TransformBroadcaster(fake)
                controller.arm_ready = lambda: True
                force = fake.create_publisher(
                    WrenchStamped, "/admittance_controller/measured_wrench", 10)
                send_force = False

                def feedback():
                    transform = TransformStamped()
                    transform.header.stamp = fake.get_clock().now().to_msg()
                    transform.header.frame_id = "base_link"
                    transform.child_frame_id = "tool0"
                    transform.transform.rotation.w = 1.0
                    broadcaster.sendTransform(transform)
                    if send_force:
                        force.publish(WrenchStamped())

                def enable(request, response):
                    response.success = True
                    return response

                fake.create_service(SetBool, "/admittance_controller/enable", enable)
                fake.create_subscription(PoseArray, controller.compliant_target_topic, lambda _: None, 10)
                fake.create_subscription(PoseArray, controller.target_topic, lambda _: None, 10)
                fake.create_timer(0.02, feedback)

                def pump(seconds):
                    deadline = time.monotonic() + seconds
                    ready = False
                    while time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=0.02)
                        ready = controller.advance_device_checks()
                    return ready

                try:
                    self.assertFalse(pump(0.5))  # 无执行接口/无力数据时先不放行。
                    fake.create_service(Trigger, controller.execute_service, enable)
                    if modes == ("moveit",):
                        self.assertTrue(pump(0.6))
                        self.assertIsNone(controller.last_compliant_wrench_ns)
                    else:
                        self.assertFalse(pump(0.6))
                        send_force = True
                        self.assertTrue(pump(0.6))
                finally:
                    executor.shutdown()
                    controller.destroy_node()
                    fake.destroy_node()
                    rclpy.shutdown()

    def test_device_timeout_does_not_enter_task(self):
        config = (
            Path(__file__).resolve().parents[1]
            / "test/fixtures/arm_admittance_line_follow.yaml"
        )
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                f"workflow_file:={config}",
                "-p",
                "auto_start:=true",
                "-p",
                "require_arm_ready:=true",
                "-p",
                "check_devices_on_startup:=true",
                "-p",
                "device_check_timeout_sec:=0.1",
            ]
        )
        controller = WorkflowController()
        try:
            controller.devices_check_started = time.monotonic() - 1.0
            controller.advance()
            self.assertTrue(controller.failed)
            self.assertFalse(controller.active)
            self.assertFalse(controller.devices_checked)
            self.assertIsNone(controller.arm_control_mode)
            self.assertEqual(controller.step_index, 0)
        finally:
            controller.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
