#!/usr/bin/env python3
"""离线检查相机毫米坐标、探头补偿及模型数值可达性；不初始化ROS或运动设备。"""
import ast
from collections import deque
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
