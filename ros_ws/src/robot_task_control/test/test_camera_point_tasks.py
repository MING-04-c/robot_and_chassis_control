#!/usr/bin/env python3
"""离线检查相机毫米坐标、探头补偿及模型数值可达性；不初始化ROS或运动设备。"""
import ast
from collections import deque
from copy import deepcopy
import inspect
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
from time import monotonic
import unittest
from unittest.mock import MagicMock, patch
from builtin_interfaces.msg import Time as TimeMessage
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import xacro
from rclpy.node import Node

from robot_task_control.testing.pixel_point_simulation import JOINTS, read_pixel_point, PixelPointSimulation
from robot_task_control.camera_detection_task import CameraDetectionTask, received_points
from std_msgs.msg import String
from geminipro_camer_calibration.capture_geometry import frozen_probe_targets
from geminipro_camer_calibration.live_tf import LiveRobotTransforms
from geminipro_camer_calibration.geometry import rpy_degrees_rotation, tool_goal

ROOT = Path(__file__).resolve().parents[2]
SAMPLE = ROOT/'robot_task_control/config/tests/pixel_target_sample.json'


class CameraPointTasksTest(unittest.TestCase):
    @staticmethod
    def tcp_result(digest='a'*64):
        return {'schema': 'paint_v54_pi_remote_result_v1', 'message_id': 'tcp-result-test',
                'status': 'OK_3D_POINTS', 'capture_manifest_sha256': digest,
                'coordinate_frame': 'CAMERA_PINHOLE_PRELIMINARY', 'depth_source': 'MEASURED',
                'robot_motion_authorized': False, 'robot_base_coordinate_calibrated': False,
                'robot_command': None, 'points_xyz_camera_mm': [
                    {'point_id': 'p1', 'xyz_camera_mm': [10., 20., 300.]},
                    {'point_id': 'p2', 'xyz_camera_mm': [20., 30., 350.]}]}

    def test_tcp_results_bind_to_current_capture_and_reject_demo_hold(self):
        payload = self.tcp_result()
        np.testing.assert_allclose(received_points(payload, 'a'*64), [[10., 20., 300.], [20., 30., 350.]])
        self.assertIsNone(received_points(payload, 'b'*64))
        demo = dict(payload, schema='paint_v54_xyz_demo_v1', demo_only=True,
                    status='DEMO_ONLY_NOT_VALID_MEASUREMENT')
        self.assertIsNone(received_points(demo, 'a'*64))
        hold = dict(payload, status='HOLD_NO_VALID_3D_POINTS', points_xyz_camera_mm=[])
        with self.assertRaisesRegex(ValueError, '无法触碰'):
            received_points(hold, 'a'*64)

    def test_modes_choose_capture_or_direct_points(self):
        for source in ('tcp', 'local'):
            node = MagicMock()
            node.phase, node.since = 'WAIT_READY', monotonic()
            node.timeouts = {'startup': 120.}
            node.ready.return_value = True
            node.ready_since = monotonic()-3.
            node.pose.return_value = np.eye(4)
            node.target_source = source
            CameraDetectionTask.advance(node)
            self.assertEqual(node.begin_capture.call_count, int(source == 'tcp'))
            self.assertEqual(node.prepare_targets.call_count, int(source == 'local'))

    def test_capture_result_becomes_frozen_probe_targets_without_local_fallback(self):
        node = MagicMock()
        node.run_id, node.target_source = 'current-capture', 'tcp'
        node.snapshot = {'base_T_camera': np.eye(4).tolist(), 'base_T_probe': np.eye(4).tolist(),
                         'frozen_at_ros_ns': 123}
        node.snapshot['base_T_camera'][0][3] = .2
        node.ready.return_value = True
        node.capture_process.poll.return_value = node.capture_process.returncode = 0
        node.results = deque()
        node.completed_point_ids = set()
        node.expected_point_ids = set()
        node.points_mm = np.array([[999., 999., 999.]])
        with tempfile.TemporaryDirectory() as directory:
            node.storage = Path(directory)
            (node.storage/'.capture_index.json').write_text(json.dumps({'sessions': {node.run_id: {
                'directory': '2026.10.30.20.29', 'capture_name': '2026.10.30.20.29.12_123',
                'rgbd': {'capture_manifest_sha256': 'a'*64}}}}))
            CameraDetectionTask.finish_capture(node)
        self.assertEqual(node.snapshot['capture_manifest_sha256'], 'a'*64)
        node.phase = 'WAIT_RESULT'
        CameraDetectionTask.on_result(node, String(data=json.dumps(self.tcp_result('b'*64))))
        CameraDetectionTask.consume_result(node)
        node.prepare_targets.assert_not_called()
        np.testing.assert_allclose(node.points_mm, [[999., 999., 999.]])
        CameraDetectionTask.on_result(node, String(data=json.dumps(self.tcp_result())))
        node.prepare_targets.side_effect = lambda: CameraDetectionTask.prepare_targets(node)
        CameraDetectionTask.consume_result(node)
        np.testing.assert_allclose(node.targets[0][:3, 3], [.21, .02, .3])
        np.testing.assert_allclose(node.targets[1][:3, 3], [.22, .03, .35])
        self.assertEqual(node.snapshot['frozen_at_ros_ns'], 123)
        node.clear_client.call_async.assert_called_once()
        node.start_client.call_async.assert_not_called()
        self.assertEqual(node.snapshot['result_message_id'], 'tcp-result-test')
        node.phase = 'RUNNING'
        CameraDetectionTask.on_result(node, String(data=json.dumps(self.tcp_result())))
        self.assertEqual(len(node.results), 0)

    def test_capture_cancel_targets_only_its_own_process_group(self):
        from robot_task_control import camera_detection_task as task
        node = MagicMock()
        node.capture_process.pid = 12345
        node.capture_process.poll.return_value = None
        node.capture_process.wait.side_effect = [subprocess.TimeoutExpired('capture', 1.), 0]
        with patch.object(task.os, 'killpg') as killpg:
            CameraDetectionTask.stop_capture(node)
        self.assertEqual(killpg.call_args_list[0].args, (12345, signal.SIGTERM))
        self.assertEqual(killpg.call_args_list[1].args, (12345, signal.SIGKILL))

    def test_tf_thread_exits_before_executor_and_node_are_destroyed(self):
        live = LiveRobotTransforms.__new__(LiveRobotTransforms)
        live.closed = False
        events = []
        live.executor = MagicMock()
        live.thread, live.listener, live.node = MagicMock(), MagicMock(), MagicMock()
        live.thread.is_alive.return_value = False
        live.thread.join.side_effect = lambda **_: events.append('join')
        live.executor.shutdown.side_effect = lambda **_: events.append('executor')
        live.node.destroy_node.side_effect = lambda: events.append('node')
        live.close()
        self.assertEqual(events, ['join', 'executor', 'node'])
        live.thread.join.assert_called_once_with(timeout=1.0)
        live.close()  # 重复清理不再销毁节点。
        live.node.destroy_node.assert_called_once()

    def test_sigint_and_sigterm_exit_without_ros_or_robot(self):
        """在独立进程发送真实信号；仅运行替身执行器，不创建 DDS 或硬件节点。"""
        script = '''
import importlib.util
import sys
import time
from unittest.mock import MagicMock
if sys.argv[1] == 'task':
    import robot_task_control.camera_detection_task as module
    constructor = 'CameraDetectionTask'
else:
    spec = importlib.util.spec_from_file_location('detect_shutdown', sys.argv[2])
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    constructor = 'DetectController'
module.rclpy.init = lambda **_: None
module.rclpy.ok = lambda: True
module.rclpy.try_shutdown = lambda: print('CONTEXT_CLOSED', flush=True)
class FakeExecutor:
    def add_node(self, node):
        print('READY', flush=True)
    def spin_once(self, **kwargs):
        time.sleep(.02)
    def remove_node(self, node):
        pass
    def shutdown(self, **kwargs):
        print('EXECUTOR_CLOSED', flush=True)
def make_node():
    node = MagicMock()
    node.cancel_future.done.return_value = False
    node.stop_future.done.return_value = False
    node.active = True
    node.fail.side_effect = lambda *_: print('CANCEL_REQUESTED', flush=True)
    node.live.close.side_effect = lambda: print('TF_CLOSED', flush=True)
    node.destroy_node.side_effect = lambda: print('NODE_CLOSED', flush=True)
    return node
setattr(module, constructor, make_node)
module.SingleThreadedExecutor = FakeExecutor
module.main()
'''
        for mode in ('task', 'detector'):
            for signum in (signal.SIGINT, signal.SIGTERM):
                with self.subTest(mode=mode, signum=signum):
                    process = subprocess.Popen([
                        sys.executable, '-u', '-c', script, mode,
                        str(ROOT/'detect_control/scripts/detect_controller.py')],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                        start_new_session=True, env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
                    try:
                        self.assertTrue(select.select([process.stdout], [], [], 15.)[0], '未进入信号等待')
                        self.assertEqual(process.stdout.readline().strip(), 'READY')
                        process.send_signal(signum)
                        output, _ = process.communicate(timeout=4.)
                        self.assertEqual(process.returncode, 0, output)
                        self.assertIn('CANCEL_REQUESTED', output)
                        self.assertIn('NODE_CLOSED', output)
                        self.assertIn('CONTEXT_CLOSED', output)
                        self.assertLess(output.index('CANCEL_REQUESTED'), output.index('NODE_CLOSED'))
                    finally:
                        if process.poll() is None:
                            process.kill()
                            process.communicate()

    def test_node_properties_are_not_used_for_clients(self):
        """检查成员赋值，避免客户端覆盖 Node.executor 等 ROS 节点属性。"""
        properties = {name for base in Node.__mro__ for name, value in vars(base).items()
                      if isinstance(value, property)}
        for node_class in (PixelPointSimulation, CameraDetectionTask):
            tree = ast.parse(inspect.getsource(node_class))
            assigned = {item.attr for item in ast.walk(tree)
                        if isinstance(item, ast.Attribute) and isinstance(item.ctx, ast.Store)
                        and isinstance(item.value, ast.Name) and item.value.id == 'self'}
            self.assertFalse(assigned & properties,
                             '自定义成员覆盖 ROS 节点属性: '+str(assigned & properties))

    def test_real_targets_use_frozen_camera_pose_and_probe_orientation(self):
        camera, probe = np.eye(4), np.eye(4)
        camera[:3, :3] = rpy_degrees_rotation([180., 0., 90.])
        camera[:3, 3] = [.2, -.1, .5]
        probe[:3, :3] = rpy_degrees_rotation([-160., 0., -77.])
        original = camera.copy()
        goals = frozen_probe_targets(camera, probe, [[10., 20., 200.], [0., 0., 100.]])
        np.testing.assert_allclose(goals[0][:3, 3], [ .22, -.09, .3])
        np.testing.assert_allclose(goals[1][:3, 3], [ .2, -.1, .4])
        np.testing.assert_allclose(goals[0][:3, :3], probe[:3, :3])
        np.testing.assert_allclose(camera, original)  # 转换不能修改拍照快照。
        with self.assertRaises(ValueError):
            frozen_probe_targets(camera, probe, [[0., 0., -1.]])
        invalid = camera.copy()
        invalid[0, 0] = 2.
        with self.assertRaises(ValueError):
            frozen_probe_targets(invalid, probe, [[0., 0., 100.]])

    def test_real_task_invalidates_moved_capture_pose(self):
        node = MagicMock()
        node.camera, node.probe = 'camera', 'probe'
        node.snapshot = {'base_T_camera': np.eye(4).tolist(), 'base_T_probe': np.eye(4).tolist()}
        node.position_tolerance, node.angle_tolerance = .001, np.deg2rad(.5)
        node.pose.return_value = np.eye(4)
        CameraDetectionTask.check_stationary(node)
        moved = np.eye(4)
        moved[0, 3] = .002
        node.pose.return_value = moved
        with self.assertRaisesRegex(ValueError, '位姿已变化'):
            CameraDetectionTask.check_stationary(node)
        moved = np.eye(4)
        moved[:3, :3] = rpy_degrees_rotation([0., 0., 1.])
        node.pose.return_value = moved
        with self.assertRaisesRegex(ValueError, '位姿已变化'):
            CameraDetectionTask.check_stationary(node)

    def test_real_task_clears_records_and_starts_only_after_new_targets(self):
        node = MagicMock()
        node.phase, node.since = 'WAIT_READY', monotonic()
        node.timeouts = {'startup': 120., 'contact': 600.}
        node.snapshot = {'base_T_camera': np.eye(4).tolist(), 'base_T_probe': np.eye(4).tolist()}
        node.points_mm = np.array([[1., 2., 300.]])
        node.run_id, node.camera, node.probe = 'local-test', 'camera', 'probe'
        node.ready.return_value = node.connected.return_value = True
        node.detect_at, node.detect = monotonic(), {'state': 'IDLE', 'queued_points': 0}
        node.base, node.holds = 'base_link', 0
        node.home_recorded = False
        node.point_ids = []
        node.completed_point_ids = {'point_1'}
        node.publish_targets.side_effect = lambda: CameraDetectionTask.publish_targets(node)
        node.get_clock.return_value.now.return_value.to_msg.return_value = TimeMessage(sec=1)
        for client in (node.clear_client, node.record_client, node.start_client):
            client.call_async.return_value.done.return_value = True
            client.call_async.return_value.result.return_value.success = True
        def transition(phase, detail):
            node.phase, node.since = phase, monotonic()
        node.transition.side_effect = transition
        node.operation_complete.side_effect = lambda: CameraDetectionTask.operation_complete(node)
        CameraDetectionTask.prepare_targets(node)
        self.assertEqual(node.phase, 'CLEAR_TARGETS')
        node.record_client.call_async.assert_not_called()
        node.start_client.call_async.assert_not_called()
        CameraDetectionTask.advance(node)
        self.assertEqual(node.phase, 'RECORD_HOME')
        CameraDetectionTask.advance(node)
        self.assertEqual(node.phase, 'WAIT_TARGETS')
        node.start_client.call_async.assert_not_called()
        CameraDetectionTask.advance(node)
        node.start_client.call_async.assert_not_called()  # 旧状态不能作为确认。
        node.detect_at, node.detect = monotonic()+.01, {'state': 'IDLE', 'queued_points': 1}
        CameraDetectionTask.advance(node)
        self.assertEqual(node.phase, 'STARTING')
        CameraDetectionTask.advance(node)
        self.assertEqual(node.phase, 'RUNNING')
        node.detect = {'state': 'DONE', 'completed_points': 1}
        with self.assertRaisesRegex(ValueError, '保持完成证据'):
            CameraDetectionTask.advance(node)
        node.holds = 1
        CameraDetectionTask.advance(node)
        self.assertEqual(node.phase, 'DONE')
        node.target_pub.publish.assert_called_once()
        target = node.target_pub.publish.call_args.args[0].poses[0]
        self.assertAlmostEqual(target.position.z, .3)

    @classmethod
    def setUpClass(cls):
        robot = xacro.process_file(str(ROOT/'eli_cs_robot_description/urdf/cs.urdf.xacro'),
            mappings={'name': 'cs', 'cs_type': 'ls65', 'use_fake_hardware': 'true', 'robot_ip': '127.0.0.1'})
        cls.xml = ET.fromstring(robot.toxml())
        cls.joints = {j.find('child').get('link'): j for j in cls.xml.findall('joint')}

    def fk(self, link, q):
        if link == 'base_link':
            return np.eye(4)
        joint = self.joints[link]
        origin = joint.find('origin')
        transform, rotation = np.eye(4), np.eye(4)
        if origin is not None:
            transform[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
            transform[:3, :3] = rpy_degrees_rotation(np.rad2deg(
                [float(v) for v in origin.get('rpy', '0 0 0').split()]))
        if joint.get('type') in ('revolute', 'continuous'):
            axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
            rotation[:3, :3] = Rotation.from_rotvec(axis*q[JOINTS.index(joint.get('name'))]).as_matrix()
        return self.fk(joint.find('parent').get('link'), q) @ transform @ rotation

    def test_pixel_point_units_and_pinhole_consistency(self):
        data, point = read_pixel_point(SAMPLE)
        np.testing.assert_allclose(point, [-.005370267016871055, -.003820746759094509, .275])
        k = data['intrinsic_used']
        u, v = data['test_pixel_point']['pixel_uv']
        z = data['test_pixel_point']['depth_mm_used_median7x7']/1000.
        np.testing.assert_allclose(point, [(u-k['cx'])*z/k['fx'], (v-k['cy'])*z/k['fy'], z])
        self.assertFalse(data['robot_motion_authorized'])

    def test_invalid_camera_point_is_rejected(self):
        original = json.loads(SAMPLE.read_text())
        for point in ([0, 0, -1], [0, 0, float('nan')], [0, 0], [0, 0, 5000]):
            original['test_pixel_point']['xyz_mm_pinhole_preliminary'] = point
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/'invalid.json'
                path.write_text(json.dumps(original))
                with self.assertRaises(ValueError):
                    read_pixel_point(path)

    def test_simulated_force_is_guarded_and_contact_event_is_latched(self):
        node = MagicMock()
        node.mock_verified = False
        PixelPointSimulation.publish_simulated_force(node)
        node.force_pub.publish.assert_not_called()
        node.mock_verified = True
        node.detect_status = {'state': 'DESCEND'}
        node.target = np.array([0., 0., .2])
        node.contact_latched = False
        node.simulated_force = 21.
        node.get_clock.return_value.now.return_value.to_msg.return_value = TimeMessage(sec=1)
        pose = np.eye(4)
        pose[2, 3] = .21
        node.tf.return_value = pose
        PixelPointSimulation.publish_simulated_force(node)
        self.assertEqual(node.force_pub.publish.call_args.args[0].wrench.force.z, 0.)
        pose[2, 3] = .2
        PixelPointSimulation.publish_simulated_force(node)
        self.assertEqual(node.force_pub.publish.call_args.args[0].wrench.force.z, 21.)
        pose[2, 3] = .21
        node.detect_status = {'state': 'HOLD'}
        PixelPointSimulation.publish_simulated_force(node)
        self.assertEqual(node.force_pub.publish.call_args.args[0].wrench.force.z, 21.)
        node.detect_status = {'state': 'RETRACT'}
        PixelPointSimulation.publish_simulated_force(node)
        self.assertEqual(node.force_pub.publish.call_args.args[0].wrench.force.z, 0.)

    def test_camera_probe_touch_16cm_preserves_tool_orientation_and_compensates_probe(self):
        import yaml
        profile = yaml.safe_load((ROOT/'robot_task_control/config/camera_probe_touch.yaml').read_text())
        configured = profile['camera_detection_task']['ros__parameters']['camera_points_mm']
        self.assertTrue(configured and len(configured) % 3 == 0)
        self.assertTrue(all(type(value) is float for value in configured))
        # 使用ROS原生参数解析器检查配置，不创建节点或连接设备。
        from rclpy.utilities import remove_ros_args
        config_path = ROOT/'robot_task_control/config/camera_probe_touch.yaml'
        self.assertEqual(remove_ros_args(args=['check', '--ros-args', '--params-file', str(config_path)]), ['check'])
        # 几何用固定中心点验证，不约束用户可编辑的现场检测点。
        points = [0., 0., 160.]
        self.assertFalse(profile['detect_controller']['ros__parameters']['lock_tool_orientation'])
        joints = np.array([.3, -1.2, 2.4, -3., -1.57, 0.])
        camera, probe, tool = [self.fk(name, joints) for name in
                               ('terminal_link_1', 'terminal_link_2', 'tool0')]
        probe_target = frozen_probe_targets(camera, probe, np.asarray(points).reshape(-1, 3))[0]
        expected = camera[:3, 3]+.16*camera[:3, 2]
        np.testing.assert_allclose(probe_target[:3, 3], expected, atol=1e-12)
        mounting = np.linalg.inv(tool) @ probe
        tool_target = tool_goal(probe_target, mounting)
        np.testing.assert_allclose((tool_target @ mounting)[:3, 3], expected, atol=1e-12)
        np.testing.assert_allclose(tool_target[:3, :3], tool[:3, :3], atol=1e-12)
        self.assertGreater(np.linalg.norm(tool_target[:3, 3]-expected), .1)

    def test_locked_orientation_is_a_path_constraint_not_only_a_goal(self):
        import importlib.util
        import yaml
        from sensor_msgs.msg import JointState
        spec = importlib.util.spec_from_file_location('detect_camera_touch_test',
            ROOT/'detect_control/scripts/detect_controller.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        node = MagicMock()
        node.cfg = dict(base_frame='base_link', tool_frame='tool0', lock_tool_orientation=True,
                        planning_group='cs_manipulator', velocity_scaling=.1, acceleration_scaling=.1,
                        orientation_tolerance=np.deg2rad(1.))
        node.initial_pose = np.eye(4)
        node.initial_pose[:3, :3] = rpy_degrees_rotation([-160., 0., -77.])
        node.joints = JointState()
        node.orientation_path_constraints.side_effect = lambda: module.DetectController.orientation_path_constraints(node)
        goal = module.DetectController.motion_request(node)
        constraints = goal.request.path_constraints.orientation_constraints
        self.assertEqual(len(constraints), 1)
        self.assertEqual(constraints[0].link_name, 'tool0')
        self.assertAlmostEqual(constraints[0].absolute_z_axis_tolerance, np.deg2rad(1.))
        node.cfg['lock_tool_orientation'] = False
        goal = module.DetectController.motion_request(node)
        self.assertFalse(goal.request.path_constraints.orientation_constraints)

    def test_new_terminal_model_uses_exported_joints_and_original_mount_rotation(self):
        """核对新版相机/探针映射，防止恢复额外180度修正或漏掉装配旋转。"""
        expected = {
            'terminal_assembly_link': ('terminal_assembly_fixed', [0., 0., .0027], [0., 0., 0.], 'Link_1.stl'),
            'terminal_link_1': ('terminal_joint_1', [.02920351, -.05748778, .0582], [0., 0., -2.35619449], 'Link_2.stl'),
            'terminal_link_2': ('terminal_joint_2', [0., 0., .1487], [0., 0., 0.], 'Link_4.stl'),
        }
        for link, (name, xyz, rpy, mesh) in expected.items():
            joint = self.joints[link]
            self.assertEqual(joint.get('name'), name)
            self.assertEqual(joint.get('type'), 'fixed')
            self.assertEqual(joint.find('parent').get('link'), 'terminal_base_link')
            np.testing.assert_allclose([float(v) for v in joint.find('origin').get('xyz').split()], xyz)
            np.testing.assert_allclose([float(v) for v in joint.find('origin').get('rpy').split()], rpy)
            for kind in ('visual', 'collision'):
                filename = self.xml.find(f"link[@name='{link}']/{kind}/geometry/mesh").get('filename')
                self.assertTrue(filename.endswith('/meshes/terminal/'+mesh), filename)
        q = np.array([.3, -1.2, 2.4, -3., -1.57, 0.])
        tool = self.fk('tool0', q)
        np.testing.assert_allclose(self.fk('flange', q), tool, atol=1e-12)
        mount = np.linalg.inv(tool) @ self.fk('terminal_base_link', q)
        np.testing.assert_allclose(mount[:3, :3], rpy_degrees_rotation([0., 0., -90.]), atol=1e-12)
        np.testing.assert_allclose(mount[:3, 3], [0., 0., -.0027], atol=1e-12)
        assembly = np.linalg.inv(tool) @ self.fk('terminal_assembly_link', q)
        np.testing.assert_allclose(assembly[:3, 3], [0., 0., 0.], atol=1e-12)
        camera = np.linalg.inv(tool) @ self.fk('terminal_link_1', q)
        np.testing.assert_allclose(camera[:3, 3], [-.05748778, -.02920351, .0555], atol=1e-12)
        probe = np.linalg.inv(tool) @ self.fk('terminal_link_2', q)
        np.testing.assert_allclose(probe[:3, 3], [0., 0., .1460], atol=1e-12)
        self.assertNotIn('terminal_camera_optical_frame', self.joints)

    @staticmethod
    def probe_controller():
        import importlib.util
        spec = importlib.util.spec_from_file_location('probe_axis_test',
            ROOT/'detect_control/scripts/detect_controller.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.DetectController

    def probe_node(self):
        """构造离线接口替身，使用倾斜向下轴暴露把局部Z误写成基座Z的问题。"""
        from sensor_msgs.msg import JointState
        controller = self.probe_controller()
        node = MagicMock()
        node.cfg = dict(approach_axis='tool_positive_z', continue_until_contact=True,
                        pose_tolerance=.003, lateral_tolerance=.002, max_tracking_error=.005,
                        approach_speed=.002, rate=20., contact_force=20., overload_force=40.,
                        base_frame='base_link', tool_frame='tool0', data_timeout=.3,
                        planning_group='cs_manipulator', approach_clearance=.02,
                        max_penetration=.02, phase_timeout=120., velocity_scaling=.1,
                        acceleration_scaling=.1, initial_force_limit=5., home_joint_tolerance=.01)
        node.retry_context = None
        node.pre = np.eye(4)
        node.pre[:3, :3] = rpy_degrees_rotation([140., 20., 0.])
        node.pre[:3, 3] = [.2, .1, .4]
        node.axis = controller.direction_for(node, node.pre)
        node.final = node.pre.copy()
        node.final[:3, 3] += node.axis*.02
        node.mounting = np.eye(4)
        node.mounting[2, 3] = .15
        node.current_tool.return_value = node.pre.copy()
        node.direction_for.side_effect = lambda pose: controller.direction_for(node, pose)
        node.axial_force.side_effect = lambda force: controller.axial_force(node, force)
        node.handle_force_sample.side_effect = lambda force: controller.handle_force_sample(node, force)
        node.rotation_ok.return_value = True
        node.in_workspace.return_value = True
        node.stamp_fresh.return_value = True
        node.joints = JointState(name=list(JOINTS), position=[0.]*6)
        controller.begin_descend(node)
        node.phase = 'DESCEND'
        return controller, node

    def test_local_z_advances_beyond_nominal_but_stops_at_extra_travel_limit(self):
        from moveit_msgs.msg import MoveItErrorCodes
        controller, node = self.probe_node()
        np.testing.assert_allclose(node.axis, node.pre[:3, 2])
        self.assertLess(node.axis[2], 0.)
        self.assertGreater(abs(node.axis[1]), .1)
        self.assertAlmostEqual(node.travel_limit, .04)
        node.command[:3, 3] = node.line_start[:3, 3]+node.axis*.025
        node.current_tool.return_value = node.command.copy()
        controller.descend_step(node)
        request = node.ik.call_async.call_args.args[0]
        pose = request.ik_request.pose_stamped.pose.position
        np.testing.assert_allclose([pose.x, pose.y, pose.z],
                                  node.line_start[:3, 3]+node.axis*.0251)
        self.assertTrue(request.ik_request.avoid_collisions)
        callback = node.wait.call_args.args[1]
        result = MagicMock()
        result.error_code.val = MoveItErrorCodes.SUCCESS
        callback(result)
        np.testing.assert_allclose(node.command[:3, 3], node.line_start[:3, 3]+node.axis*.0251)
        node.command[:3, 3] = node.line_start[:3, 3]+node.axis*.04
        node.current_tool.return_value = node.command.copy()
        with self.assertRaisesRegex(RuntimeError, '额外接近行程已用完'):
            controller.descend_step(node)
        node.cfg['continue_until_contact'] = False
        node.command[:3, 3] = node.line_start[:3, 3]+node.axis*.02
        node.current_tool.return_value = node.command.copy()
        controller.descend_step(node)
        self.assertEqual(node.state.call_args.args[0], 'COMPLIANT_HOLD')

    def test_contact_uses_axial_force_and_keeps_overload_guard(self):
        from geometry_msgs.msg import WrenchStamped
        controller, node = self.probe_node()
        message = WrenchStamped()
        message.header.frame_id = 'base_link'
        # 横向25N不会误当轴向接触；倾斜轴向21N即使基座Fz不足20仍要停止。
        for force, expected in ((np.cross(node.axis, [1., 0., 0.])*25., False),
                                (node.axis*21., True)):
            node.set_enabled.reset_mock()
            message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = map(float, force)
            controller.on_force(node, message)
            self.assertEqual(node.set_enabled.called, expected)
        node.set_enabled.assert_called_once_with(False, node.begin_hold)
        self.assertEqual(node.state.call_args.args[0], 'STOP_CONTACT')
        force = node.axis*41.
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = map(float, force)
        controller.on_force(node, message)
        node.fail.assert_called_once_with('合力超过保护阈值')

    def test_moveit_descent_rejects_partial_collision_checked_path(self):
        from moveit_msgs.srv import GetCartesianPath
        from moveit_msgs.msg import MoveItErrorCodes
        controller, node = self.probe_node()
        node.cfg['moveit_cartesian_step'] = .005
        node.force = np.zeros(3)
        controller.plan_moveit_descent(node)
        request = node.cartesian.call_async.call_args.args[0]
        self.assertTrue(request.avoid_collisions)
        self.assertEqual(request.max_cartesian_speed, .002)
        self.assertEqual(request.link_name, 'tool0')
        self.assertEqual(len(request.waypoints), 1)
        callback = node.wait.call_args.args[1]
        result = GetCartesianPath.Response()
        result.error_code.val = MoveItErrorCodes.SUCCESS
        result.fraction = .5
        with self.assertRaisesRegex(RuntimeError, '完整无碰撞'):
            callback(result)
        node.action.assert_not_called()
        result.fraction = 1.
        callback(result)
        node.action.assert_called_once()
        node.enable.call_async.assert_not_called()

    def test_moveit_force_stop_requires_post_cancel_feedback_and_has_timeout(self):
        from sensor_msgs.msg import JointState
        controller = self.probe_controller()
        node = MagicMock()
        node.cfg = dict(contact_force=20., service_timeout=5., lock_tool_orientation=False,
                        phase_timeout=120., joint_postures_enabled=False)
        node.phase = 'MOVEIT_DESCEND'
        node.contact_stop_requested = False
        node.generation = 0
        node.joints = JointState(name=list(JOINTS), position=[0.]*6, velocity=[0.]*6)
        node.joints.header.stamp.sec = 10
        node.state.side_effect = lambda phase, detail: setattr(node, 'phase', phase)
        controller.request_moveit_force_stop(node)
        node.goal_handle = None
        self.assertEqual(node.phase, 'STOP_CONTACT')
        self.assertFalse(controller.moveit_stop_ready(node))
        node.joints.header.stamp.sec = 11
        self.assertFalse(controller.moveit_stop_ready(node))
        node.moveit_stop_since = monotonic()-.3
        self.assertTrue(controller.moveit_stop_ready(node))
        # 动作已经返回，但机器人持续运动，同样必须超时退出，不能永远等停稳。
        node.since = monotonic()-6.
        node.last_status = monotonic()
        node.stop_future = None
        node.operation = None
        node.touch_control_mode = 'moveit_force_guard'
        node.outstanding_actions = set()
        controller.tick(node)
        self.assertIn('停稳确认超时', node.fail.call_args.args[0])
        node.begin_hold.assert_not_called()

    def test_moveit_raw_overload_cannot_be_hidden_by_low_pass(self):
        from geometry_msgs.msg import WrenchStamped
        controller = self.probe_controller()
        node = MagicMock()
        node.touch_control_mode = 'moveit_force_guard'
        node.cfg = dict(data_timeout=.3, overload_force=40., force_filter_alpha=.25)
        node.force_bias = np.zeros(3)
        node.force_filter = np.zeros(3)
        node.stamp_fresh.return_value = True
        node.raw_force_in_base.return_value = np.array([0., 0., 50.])
        controller.on_raw(node, WrenchStamped())
        self.assertIn('原始去偏合力', node.fail.call_args.args[0])
        node.handle_force_sample.assert_not_called()

    def test_moveit_live_descent_checks_travel_while_action_pending(self):
        controller = self.probe_controller()
        node = MagicMock()
        node.cfg = dict(lock_tool_orientation=False, phase_timeout=120.,
                        joint_postures_enabled=False, continue_until_contact=True,
                        lateral_tolerance=.002, pose_tolerance=.003)
        node.last_status = node.since = monotonic()
        node.stop_future = None
        node.phase = 'MOVEIT_DESCEND'
        node.line_start = np.eye(4)
        node.axis = np.array([0., 0., -1.])
        node.travel_limit = .07
        pose = np.eye(4)
        pose[2, 3] = -.08
        node.current_tool.return_value = pose
        node.rotation_ok.return_value = True
        controller.tick(node)
        self.assertIn('实际探入行程超过限制', node.fail.call_args.args[0])

    def test_reverse_retract_then_return_to_same_recorded_home_for_each_point(self):
        from moveit_msgs.msg import Constraints, MoveItErrorCodes
        controller, node = self.probe_node()
        node.orientation_path_constraints.return_value = Constraints()
        current = node.pre.copy()
        current[:3, 3] += node.axis*.03
        node.current_tool.return_value = current
        controller.retreat(node)
        request = node.cartesian.call_async.call_args.args[0]
        point = request.waypoints[0].position
        np.testing.assert_allclose([point.x, point.y, point.z], node.pre[:3, 3])
        result = MagicMock()
        result.error_code.val, result.fraction = MoveItErrorCodes.SUCCESS, 1.
        node.wait.call_args.args[1](result)
        self.assertIs(node.action.call_args.args[2], node.return_home)
        node.initial_joints = node.joints
        initial = list(node.initial_joints.position)
        node.completed_points, node.point_index = 0, 0
        node.batch = [(node.pre.copy(), node.final.copy())]*2
        node.force = np.zeros(3)
        node.home_reached.return_value = False
        with self.assertRaisesRegex(RuntimeError, '实际关节'):
            controller.point_returned(node)
        self.assertEqual(node.completed_points, 0)
        node.home_reached.return_value = True
        controller.point_returned(node)
        self.assertEqual(node.completed_points, 1)
        node.plan_pose.assert_called_once()
        controller.point_returned(node)
        self.assertEqual(node.state.call_args.args[0], 'DONE')
        self.assertEqual(list(node.initial_joints.position), initial)

    def test_five_cm_limit_and_recovery_require_stopped_safe_robot(self):
        controller, node = self.probe_node()
        node.cfg.update(max_penetration=.05, recover_approach_failure=True,
                        service_timeout=5., recovery_settle_seconds=.5)
        controller.begin_descend(node)
        node.phase = 'DESCEND'
        self.assertAlmostEqual(node.travel_limit, .07)
        node.command[:3, 3] = node.line_start[:3, 3]+node.axis*.07
        node.current_tool.return_value = node.command.copy()
        with self.assertRaisesRegex(RuntimeError, '行程已用完') as context:
            controller.descend_step(node)
        error = context.exception
        self.assertEqual(error.code, 'travel_exhausted')
        node.safety_inhibited = False
        node.generation = 1
        node.goal_handle = None
        controller.begin_recovery(node, error)
        self.assertFalse(node.stop_confirmed)
        node.set_enabled.assert_called_once_with(False, node.recovery_stopped)
        node.return_home.assert_not_called()
        node.retreat.assert_not_called()
        controller.recovery_stopped(node)
        node.since = monotonic()
        node.outstanding_actions = set()
        node.force = np.zeros(3)
        controller.recovery_step(node)
        node.retreat.assert_not_called()  # 需要连续静止证据。
        node.recovery_quiet_since = monotonic()-1.
        controller.recovery_step(node)
        node.retreat.assert_called_once()
        node.retry_context = {'code': 'travel_exhausted'}
        node.home_reached.return_value = True
        controller.point_returned(node)
        self.assertEqual(node.state.call_args.args[0], 'RETRY_REQUIRED')
        node.publish_point_result.assert_not_called()  # 失败未测得厚度不能冒充完成。
        node.safety_inhibited = True
        node.set_enabled.reset_mock()
        controller.begin_recovery(node, error)
        node.fail.assert_called_once()
        node.set_enabled.assert_not_called()

    def test_returned_point_freezes_contact_value_and_waits_for_home(self):
        controller, node = self.probe_node()
        node.batch_id, node.point_index, node.completed_points = 'batch-test', 0, 0
        node.batch = [(node.pre, node.final)]
        node.thickness_result = {'value': 101.3, 'sequence': 2}
        node.cfg['require_thickness_measurement'] = True
        node.publish_point_result.side_effect = lambda: controller.publish_point_result(node)
        node.get_clock.return_value.now.return_value.to_msg.return_value = TimeMessage(sec=10)
        with tempfile.TemporaryDirectory() as directory:
            node.cfg['point_results_file'] = directory+'/points.jsonl'
            node.home_reached.return_value = False
            with self.assertRaises(RuntimeError):
                controller.point_returned(node)
            node.point_result_pub.publish.assert_not_called()
            node.home_reached.return_value = True
            controller.point_returned(node)
            result = json.loads(node.point_result_pub.publish.call_args.args[0].data)
            self.assertEqual(result['measurement']['value'], 101.3)
            self.assertTrue(result['returned_home'])
            self.assertTrue(result['success'])
            self.assertEqual(node.state.call_args.args[0], 'DONE')

    def test_recapture_preserves_home_and_rejects_new_ids_for_finished_points(self):
        node = MagicMock()
        node.target_source, node.retry_count, node.max_reacquire_retries = 'tcp', 0, 3
        node.home_recorded = True
        node.completed_point_ids, node.expected_point_ids = {'p1'}, {'p1', 'p2'}
        node.results = deque()
        node.run_id = 'first'
        CameraDetectionTask.schedule_reacquire(node, 'no contact')
        self.assertTrue(node.home_recorded)
        self.assertEqual(node.retry_count, 1)
        self.assertNotEqual(node.run_id, 'first')
        node.snapshot = {'capture_manifest_sha256': 'a'*64}
        node.ready.return_value = True
        node.results.append(self.tcp_result())
        CameraDetectionTask.consume_result(node)
        self.assertEqual(node.point_ids, ['p2'])
        np.testing.assert_allclose(node.points_mm, [[20., 30., 350.]])
        invalid = self.tcp_result()
        invalid['points_xyz_camera_mm'][0]['point_id'] = 'new-p1'
        node.results.append(invalid)
        with self.assertRaisesRegex(ValueError, 'point_id必须'):
            CameraDetectionTask.consume_result(node)
        node.retry_count = 3
        with self.assertRaisesRegex(ValueError, '次数已用完'):
            CameraDetectionTask.schedule_reacquire(node, 'no contact')
        node.target_source = 'local'
        with self.assertRaisesRegex(ValueError, '本地固定点'):
            CameraDetectionTask.schedule_reacquire(node, 'no contact')

    def test_joint_test_real_and_fake_hardware_are_strictly_separated(self):
        from robot_task_control.testing.joint_posture_test import JointPostureTest
        node = MagicMock()
        node.ground = True
        node.real_ready.return_value = True
        for simulation, plugin in ((True, 'mock_components/GenericSystem'),
                (False, 'ELITE_CS_ROBOT_ROS_DRIVER/EliteCSPositionHardwareInterface')):
            node.simulation = simulation
            component = MagicMock(plugin_name=plugin)
            node.future.return_value.component = [component]
            JointPostureTest.verify_hardware(node)
            component.plugin_name = 'unexpected_plugin'
            with self.assertRaisesRegex(RuntimeError, '硬件插件'):
                JointPostureTest.verify_hardware(node)
        node.simulation, node.robot_running = False, True
        node.safety_known, node.safety_inhibited = True, False
        node.external_sub.get_publisher_count.return_value = 1
        node.safety_sub.get_publisher_count.return_value = 1
        self.assertTrue(JointPostureTest.real_ready(node))
        node.safety_inhibited = True
        self.assertFalse(JointPostureTest.real_ready(node))
        node.safety_inhibited = False
        node.external_sub.get_publisher_count.return_value = 0
        self.assertFalse(JointPostureTest.real_ready(node))

    def test_joint_launch_real_mode_uses_real_driver_without_forcing_simulation_domain(self):
        import importlib.util
        from launch import LaunchContext
        from launch.actions import OpaqueFunction, SetEnvironmentVariable
        spec = importlib.util.spec_from_file_location('joint_test_launch',
            ROOT/'robot_task_control/launch/tests/joint_posture_test.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for simulation in ('true', 'false'):
            context = LaunchContext()
            context.launch_configurations.update(simulation=simulation,
                postures_file=str(ROOT/'eli_cs_robot_moveit_config/config/arm_joint_postures.yaml'),
                robot_ip='169.254.199.150', local_ip='0.0.0.0')
            includes, nodes = [], []
            def include(*args, **kwargs):
                includes.append(dict(kwargs['launch_arguments']))
                return OpaqueFunction(function=lambda _: [])
            def node(**kwargs):
                nodes.append(kwargs)
                return OpaqueFunction(function=lambda _: [])
            with patch.object(module, 'IncludeLaunchDescription', side_effect=include), patch.object(module, 'Node', side_effect=node):
                actions = module.setup(context)
            driver = includes[0]
            self.assertEqual(driver['use_fake_hardware'], simulation)
            real = simulation == 'false'
            self.assertEqual(driver['robot_ip'], '169.254.199.150' if real else '127.0.0.1')
            self.assertEqual(driver['initial_joint_controller'],
                'scaled_joint_trajectory_controller' if real else 'joint_trajectory_controller')
            self.assertEqual(sum(isinstance(a, SetEnvironmentVariable) for a in actions), 0 if real else 2)
            self.assertEqual(nodes[-1]['parameters'][0]['simulation'], not real)
            if not real:
                Path(driver['initial_positions_file']).unlink(missing_ok=True)

    def test_joint_postures_degrees_and_gateway_return_order(self):
        from eli_cs_robot_moveit_config.joint_postures import load_postures, joint_constraints
        postures = load_postures(ROOT/'eli_cs_robot_moveit_config/config/arm_joint_postures.yaml')
        np.testing.assert_allclose(np.rad2deg(postures['travel']), [-45., 15., -71., 4.4, 148., 75.])
        np.testing.assert_allclose(np.rad2deg(postures['detection_ready']), [-45., 0., 0., 0., 90., 0.])
        constraints = joint_constraints(postures['travel'])
        self.assertEqual([j.joint_name for j in constraints.joint_constraints], list(JOINTS))
        self.assertFalse(constraints.orientation_constraints)
        controller, node = self.probe_node()
        node.cfg['joint_postures_enabled'] = True
        node.postures = postures
        controller.begin_batch(node)
        np.testing.assert_allclose(node.plan_joints.call_args.args[0], postures['detection_ready'])
        node.plan_pose.assert_not_called()
        node.plan_joints.call_args.args[1]()
        node.plan_pose.assert_called_once_with(node.pre)
        node.plan_joints.reset_mock()
        controller.return_home(node)
        np.testing.assert_allclose(node.plan_joints.call_args.args[0], postures['detection_ready'])
        node.return_home_joints.assert_not_called()
        node.plan_joints.call_args.args[1]()
        node.return_home_joints.assert_called_once_with(False)
        node.initial_joints = node.joints
        node.initial_joints.position = postures['travel']
        controller.return_home_joints(node)
        goal = node.action.call_args.args[1]
        np.testing.assert_allclose([j.position for j in goal.request.goal_constraints[0].joint_constraints],
                                   postures['travel'])
        self.assertFalse(goal.request.path_constraints.orientation_constraints)

    def test_record_camera_pose_keeps_actual_joints_and_returns_to_configured_travel(self):
        from eli_cs_robot_moveit_config.joint_postures import load_postures
        from std_srvs.srv import Trigger
        controller, node = self.probe_node()
        node.cfg['joint_postures_enabled'] = True
        node.postures = load_postures(ROOT/'eli_cs_robot_moveit_config/config/arm_joint_postures.yaml')
        node.active, node.outstanding_actions = False, set()
        node.joints_at = monotonic()
        original = list(node.joints.position)
        response = controller.record_home(node, None, Trigger.Response())
        self.assertTrue(response.success)
        np.testing.assert_allclose(node.capture_joints.position, original)
        np.testing.assert_allclose(node.joints.position, original)
        np.testing.assert_allclose(node.recorded_home.position, node.postures['travel'])
        node.motion_readiness.return_value = (True, '')
        node.generation = 1
        controller.prepare_posture(node, 'detection_ready', Trigger.Response())
        node.set_enabled.call_args.args[1]()
        np.testing.assert_allclose(node.plan_joints.call_args.args[0], node.postures['detection_ready'])
        node.plan_joints.call_args.args[1]()
        np.testing.assert_allclose(node.plan_joints.call_args.args[0], original)
        self.assertEqual(node.plan_joints.call_args.args[2], 'CAPTURE_POSTURE')

    def test_ground_top_and_only_fixed_mount_exemption(self):
        import importlib.util
        from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
        spec = importlib.util.spec_from_file_location('ground_scene_test',
            ROOT/'eli_cs_robot_moveit_config/scripts/ground_scene.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        obj = module.ground_object('base_link', .08)
        self.assertAlmostEqual(obj.primitive_poses[0].position.z+obj.primitives[0].dimensions[2]/2., .08)
        original = AllowedCollisionMatrix(entry_names=['shoulder_link', 'upper_arm_link'],
            entry_values=[AllowedCollisionEntry(enabled=[False, True]), AllowedCollisionEntry(enabled=[True, False])])
        matrix = module.allow_fixed_mount(original, ['base_link', 'base_link_inertia'])
        index = matrix.entry_names.index(module.OBJECT_ID)
        for row, name in enumerate(matrix.entry_names):
            self.assertEqual(matrix.entry_values[index].enabled[row], name in ['base_link', 'base_link_inertia'])
        self.assertTrue(matrix.entry_values[0].enabled[1])
        self.assertEqual(len(original.entry_names), 2)
        checker = MagicMock()
        checker.frame, checker.top = 'base_link', .08
        self.assertTrue(module.GroundScene.matches(checker, obj))
        world = deepcopy(obj)
        world.header.frame_id = 'world'
        from geometry_msgs.msg import TransformStamped
        transform = TransformStamped()
        transform.transform.rotation.w = 1.
        checker.buffer.lookup_transform.return_value = transform
        self.assertTrue(module.GroundScene.matches(checker, world))
        world.pose.position.z += .01
        self.assertFalse(module.GroundScene.matches(checker, world))

    def test_target_outside_workspace_recaptures_only_its_own_rejection(self):
        node = MagicMock()
        node.phase, node.since = 'WAIT_TARGETS', monotonic()
        node.detect_at, node.published_at = monotonic(), monotonic()-1.
        node.target_stamp = {'sec': 10, 'nanosec': 1}
        node.detect = {'queued_points': 0, 'target_rejection': {
            'stamp': {'sec': 10, 'nanosec': 0}, 'message': 'TARGET_UNREACHABLE: outside'}}
        node.targets = [np.eye(4)]
        CameraDetectionTask.advance(node)
        node.schedule_reacquire.assert_not_called()
        node.detect['target_rejection']['stamp'] = dict(node.target_stamp)
        CameraDetectionTask.advance(node)
        node.check_stationary.assert_called()
        node.schedule_reacquire.assert_called_once()
        node.start_client.call_async.assert_not_called()

    def test_return_events_wait_independently_of_periodic_status_timestamp(self):
        node = MagicMock()
        node.detect = {'batch_id': 'b', 'completed_points': 1}
        node.return_events, node.completion_wait_started = set(), None
        self.assertFalse(CameraDetectionTask.return_results_ready(node))
        node.detect_at = monotonic()  # 周期状态不能反复延长结果等待期限。
        node.completion_wait_started = monotonic()-3.
        with self.assertRaisesRegex(ValueError, '回位结果话题缺失'):
            CameraDetectionTask.return_results_ready(node)
        node.return_events.add(('b', 1))
        self.assertTrue(CameraDetectionTask.return_results_ready(node))

    def test_model_mount_transform_probe_compensation_and_basic_ik(self):
        reference = np.array([0.3, -1.2, 2.4, -3.0, -1.57, 0.0])
        camera = self.fk('terminal_link_1', reference)
        probe = self.fk('terminal_link_2', reference)
        tool = self.fk('tool0', reference)
        point = read_pixel_point(SAMPLE)[1]
        target = probe.copy()
        target[:3, 3] = (camera @ np.r_[point, 1.])[:3]
        mounting = np.linalg.inv(tool) @ probe
        goal = tool_goal(target, mounting)
        np.testing.assert_allclose(goal @ mounting, target, atol=1e-12)
        self.assertGreater(target[2, 3], 0.)
        lower, upper = [], []
        for name in JOINTS:
            limit = self.xml.find("joint[@name='"+name+"']/limit")
            lower.append(float(limit.get('lower'))+.15)
            upper.append(float(limit.get('upper'))-.15)
        for offset in (.02, 0.):
            requested = goal.copy()
            requested[2, 3] += offset
            def residual(q):
                actual = self.fk('tool0', q)
                return np.r_[actual[:3, 3]-requested[:3, 3],
                    Rotation.from_matrix(requested[:3, :3].T @ actual[:3, :3]).as_rotvec()]
            solved = least_squares(residual, reference, bounds=(lower, upper), max_nfev=1000)
            self.assertLess(np.linalg.norm(solved.fun), 1e-5, solved.message)
        print('离线模型基座探头目标m：', target[:3, 3].tolist(), '（数值可达，未检查碰撞）')


if __name__ == '__main__':
    unittest.main()
