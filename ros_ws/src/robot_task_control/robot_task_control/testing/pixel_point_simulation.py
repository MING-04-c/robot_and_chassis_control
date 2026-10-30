"""只在GenericSystem仿真中回放相机像素测试点，规划探头到点并返回。"""
import json
import os
import signal
from pathlib import Path
from time import monotonic

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListHardwareComponents
from geometry_msgs.msg import Point, PoseArray, WrenchStamped
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from geminipro_camer_calibration.geometry import (
    rpy_degrees_rotation, transform_matrix, to_pose,
)

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def read_pixel_point(path):
    """读取诊断JSON，毫米转米；不把未验证测试点升级为真实检测结果。"""
    data = json.loads(Path(path).expanduser().read_text())
    if data.get('schema') != 'v54_depth_pixel_xyz_diagnostic_v1':
        raise ValueError('需要v54_depth_pixel_xyz_diagnostic_v1像素测试JSON')
    if data.get('coordinate_frame') != 'CAMERA_PINHOLE_PRELIMINARY':
        raise ValueError('文件不是相机光学坐标')
    point = np.asarray(data['test_pixel_point']['xyz_mm_pinhole_preliminary'], dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all() or not 0 < point[2] < 3000:
        raise ValueError('相机XYZ必须有限且Z在0～3000毫米内')
    return data, point/1000.


class PixelPointSimulation(Node):
    def __init__(self):
        super().__init__('pixel_point_simulation')
        def param(name, value):
            return self.declare_parameter(name, value).value
        domain = int(param('simulation_domain', 191))
        if not 128 <= domain <= 232 or os.environ.get('ROS_DOMAIN_ID') != str(domain):
            raise ValueError('必须在配置的独立仿真ROS域中启动')
        self.source = str(param('points_file', ''))
        self.data, self.camera_point = read_pixel_point(self.source)
        self.report_path = Path(param('report_file', '/tmp/pixel_point_simulation_report.json')).expanduser()
        self.mount_rotation = rpy_degrees_rotation(param('mount_to_optical_rpy_deg', [0., 0., 0.]))
        self.reference = list(param('reference_joints', [0.3, -1.2, 2.4, -3.0, -1.57, 0.0]))
        self.clearance = float(param('approach_clearance', .03))
        self.repeat = bool(param('repeat', False))
        self.contact_force = float(param('contact_force', 20.))
        self.simulated_force = float(param('simulated_contact_force', 21.))
        self.detect_status, self.detect_status_at = {}, -1.
        self.mock_verified = False
        self.contact_latched = False
        if not 0 < self.contact_force < self.simulated_force < 40.:
            raise ValueError('仿真要求接触阈值 < 模拟接触力 < 40N保护值')
        if len(self.reference) != 6 or not np.isfinite(self.reference).all() or not .005 <= self.clearance <= .1:
            raise ValueError('仿真初始关节和接近余量无效')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.joints = None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.hardware = self.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        self.planner = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.record_home = self.create_client(Trigger, '/detect_controller/record_home')
        self.start_contact = self.create_client(Trigger, '/detect_controller/start')
        self.cancel_contact = self.create_client(Trigger, '/detect_controller/cancel')
        self.target_pub = self.create_publisher(PoseArray, '/geminipro_calibration/targets_base', 10)
        self.force_pub = self.create_publisher(WrenchStamped, '/admittance_controller/external_wrench', 10)
        self.raw_force_pub = self.create_publisher(WrenchStamped, '/sensor/wrench', 10)
        self.create_subscription(String, '/detect_controller/status', self.on_detect_status, 10)
        self.trajectory_client = ActionClient(self, FollowJointTrajectory,
                                     '/joint_trajectory_controller/follow_joint_trajectory')
        self.markers = self.create_publisher(MarkerArray, '/pixel_point_simulation/markers',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.target = self.above = None
        self.trail = []
        self.report = {'simulation_only': True, 'source_file': self.source,
                       'camera_xyz_mm': (self.camera_point*1000).tolist(),
                       'optical_extrinsic_verified': False,
                       'assumption': 'terminal_link_1安装坐标+配置的临时光学旋转；不是真实手眼外参',
                       'robot_motion_authorized': False, 'segments': [], 'state': 'WAIT_READY',
                       'contact_force_N': self.contact_force, 'force_source': 'SIMULATED_CONTACT_EVENT',
                       'contact_phases': []}
        self.create_timer(.1, self.visualize)
        self.create_timer(.02, self.publish_simulated_force)
        self.write('WAIT_READY')

    def on_joints(self, message):
        if all(name in message.name for name in JOINTS):
            self.joints = message

    def write(self, state):
        self.report['state'] = state
        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.report_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.report, ensure_ascii=False, indent=2))
        temporary.replace(self.report_path)
        self.get_logger().info('像素点仿真：'+state)

    def wait(self, seconds):
        end = monotonic()+seconds
        while rclpy.ok() and monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)

    def future(self, future, timeout=30.):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not rclpy.ok() or not future.done() or future.result() is None:
            raise RuntimeError('仿真服务或轨迹执行超时')
        return future.result()

    def verify_mock(self):
        if not self.hardware.wait_for_service(timeout_sec=5):
            raise RuntimeError('仿真硬件管理器未启动')
        components = self.future(self.hardware.call_async(ListHardwareComponents.Request())).component
        if not components or any(x.plugin_name != 'mock_components/GenericSystem' for x in components):
            raise RuntimeError('不是纯GenericSystem仿真，拒绝发送轨迹')
        self.mock_verified = True

    def tf(self, child, parent='base_link'):
        return transform_matrix(self.buffer.lookup_transform(parent, child, Time()).transform)

    def robot_state(self):
        if self.joints is None:
            raise RuntimeError('没有仿真关节反馈')
        message = RobotState()
        message.joint_state = self.joints
        return message

    def positions(self):
        current = dict(zip(self.joints.name, self.joints.position))
        return [current[name] for name in JOINTS]

    def move_joints(self, goal, label):
        self.verify_mock()
        self.write(label)
        constraints = Constraints()
        for name, value in zip(JOINTS, goal):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(value), tolerance_above=.002,
                tolerance_below=.002, weight=1.))
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name, plan.pipeline_id, plan.planner_id = 'cs_manipulator', 'ompl', 'RRTConnectkConfigDefault'
        plan.start_state = self.robot_state()
        plan.goal_constraints = [constraints]
        plan.allowed_planning_time, plan.num_planning_attempts = 8., 4
        plan.max_velocity_scaling_factor = .12
        plan.max_acceleration_scaling_factor = .12
        response = self.future(self.planner.call_async(request), 20.).motion_plan_response
        if response.error_code.val != 1:
            raise RuntimeError(f'{label} 碰撞检查规划失败，MoveIt code={response.error_code.val}')
        trajectory = response.trajectory.joint_trajectory
        if not trajectory.points:
            raise RuntimeError(label+'规划轨迹为空')
        initial = np.asarray(self.positions())
        self.verify_mock()
        goal_message = FollowJointTrajectory.Goal()
        goal_message.trajectory = trajectory
        goal_message.goal_time_tolerance.sec = 3
        handle = self.future(self.trajectory_client.send_goal_async(goal_message))
        if not handle.accepted:
            raise RuntimeError(label+'仿真轨迹被拒绝')
        duration = trajectory.points[-1].time_from_start
        result = self.future(handle.get_result_async(), duration.sec+duration.nanosec/1e9+15.).result
        if result.error_code != 0:
            raise RuntimeError(label+'轨迹执行失败：'+result.error_string)
        self.wait(.4)
        error = float(np.max(np.abs(np.asarray(self.positions())-np.asarray(goal))))
        self.report['segments'].append({'name': label, 'trajectory_points': len(trajectory.points),
            'max_joint_change_rad': float(np.max(np.abs(np.asarray(self.positions())-initial))),
            'final_joint_error_rad': error})
        if error > .01:
            raise RuntimeError(label+'实际仿真关节没有到位')

    def on_detect_status(self, message):
        self.detect_status = json.loads(message.data)
        self.detect_status_at = monotonic()
        phase = self.detect_status.get('state')
        if not self.report['contact_phases'] or self.report['contact_phases'][-1]['phase'] != phase:
            self.report['contact_phases'].append({
                'phase': phase, 'time_monotonic': self.detect_status_at,
                'force_z_N': self.detect_status.get('force_z_N'),
            })

    def publish_simulated_force(self):
        if not self.mock_verified:
            return
        phase = self.detect_status.get('state')
        contact_phases = ('DESCEND', 'COMPLIANT_HOLD', 'STOP_CONTACT', 'HOLD')
        if phase not in contact_phases:
            self.contact_latched = False
        elif self.target is not None and not self.contact_latched:
            try:
                self.contact_latched = self.tf('terminal_link_2')[2, 3] <= self.target[2]+.0002
            except Exception:
                pass
        force = self.simulated_force if self.contact_latched else 0.
        message = WrenchStamped()
        message.header.frame_id = 'base_link'
        message.header.stamp = self.get_clock().now().to_msg()
        message.wrench.force.z = force
        self.force_pub.publish(message)
        self.raw_force_pub.publish(message)

    def contact_cycle(self, probe_goal):
        # 复用detect_control：MoveIt预接近、导纳-Z下探、力阈值停止、保持、撤离和回原位。
        deadline = monotonic()+30.
        while not self.detect_status.get('motion_ready', False) or monotonic()-self.detect_status_at > 1.:
            if monotonic() > deadline:
                raise RuntimeError('检测控制器未就绪：'+self.detect_status.get('readiness_message', '无状态'))
            self.wait(.1)
        result = self.future(self.record_home.call_async(Trigger.Request()))
        if not result.success:
            raise RuntimeError(result.message)
        message = PoseArray()
        message.header.frame_id = 'base_link'
        message.header.stamp = self.get_clock().now().to_msg()
        message.poses = [to_pose(probe_goal)]
        published = monotonic()
        self.target_pub.publish(message)
        while self.detect_status_at <= published or self.detect_status.get('queued_points') != 1:
            if monotonic()-published > 5.:
                raise RuntimeError('检测控制器没有确认相机点入队')
            self.wait(.1)
        self.verify_mock()
        self.contact_latched = False
        result = self.future(self.start_contact.call_async(Trigger.Request()))
        if not result.success:
            raise RuntimeError(result.message)
        phase_start = len(self.report['contact_phases'])
        self.write('FORCE_LIMITED_CONTACT')
        started, last_phase = monotonic(), None
        while monotonic()-started < 180.:
            self.wait(.1)
            phase = self.detect_status.get('state')
            if phase != last_phase:
                self.get_logger().info('接触阶段：'+str(phase))
                last_phase = phase
            if monotonic()-self.detect_status_at > 2.:
                raise RuntimeError('检测状态断流')
            if phase == 'FAULT':
                raise RuntimeError(self.detect_status.get('message', '接触流程失败'))
            if phase == 'DONE':
                holds = [item for item in self.report['contact_phases'][phase_start:] if item['phase'] == 'HOLD']
                if (not holds or self.detect_status.get('completed_points') != 1
                        or float(holds[-1]['force_z_N'] or 0.) < self.contact_force):
                    raise RuntimeError('缺少接触保持和检测完成证据')
                self.report['contact_event_observed'] = True
                self.report['contact_filtered_force_N'] = holds[-1]['force_z_N']
                self.report['returned_home'] = True
                return
        raise RuntimeError('相机点接触测试180秒未完成')

    def visualize(self):
        array = MarkerArray()
        if self.target is not None:
            for index, (position, color) in enumerate(((self.target, (1., .1, .1)),
                                                      (self.above, (1., .6, .1)))):
                marker = Marker()
                marker.header.frame_id, marker.ns, marker.id = 'base_link', 'target', index
                marker.type, marker.action = Marker.SPHERE, Marker.ADD
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position.tolist()
                marker.pose.orientation.w = 1.
                marker.scale.x = marker.scale.y = marker.scale.z = .015
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.
                array.markers.append(marker)
        try:
            p = self.tf('terminal_link_2')[:3, 3]
            if not self.trail or np.linalg.norm(p-self.trail[-1]) > .002:
                self.trail.append(p.copy())
            trail = Marker()
            trail.header.frame_id, trail.ns, trail.id = 'base_link', 'probe_trail', 0
            trail.type, trail.action, trail.pose.orientation.w = Marker.LINE_STRIP, Marker.ADD, 1.
            trail.scale.x, trail.color.b, trail.color.a = .003, 1., 1.
            trail.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in self.trail[-2000:]]
            array.markers.append(trail)
        except Exception:
            pass
        self.markers.publish(array)

    def run(self):
        deadline = monotonic()+60
        while monotonic() < deadline:
            self.wait(.2)
            if (self.joints is not None and self.planner.service_is_ready() and self.start_contact.service_is_ready()
                    and self.record_home.service_is_ready() and self.trajectory_client.server_is_ready()):
                break
        else:
            raise RuntimeError('仿真规划/控制器60秒未就绪')
        self.verify_mock()
        self.move_joints(self.reference, 'REFERENCE_POSE')
        while rclpy.ok():
            home = self.positions()
            camera = self.tf('terminal_link_1')
            point = np.ones(4)
            point[:3] = self.mount_rotation @ self.camera_point
            self.target = (camera @ point)[:3]
            self.above = self.target+np.array([0., 0., self.clearance])
            probe = self.tf('terminal_link_2')
            contact = probe.copy()
            contact[:3, 3] = self.target
            self.report.update(target_base_m=self.target.tolist(), approach_base_m=self.above.tolist(),
                               reference_joint_positions=home, camera_mount_base=camera.tolist())
            self.contact_cycle(contact)
            self.write('COMPLETE')
            if not self.repeat:
                break
            self.wait(3.)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = PixelPointSimulation()
    try:
        try:
            node.run()
        except Exception as error:
            node.report['error'] = str(error)
            node.write('FAILED')
            node.get_logger().error(str(error))
            if node.mock_verified and node.cancel_contact.service_is_ready():
                node.future(node.cancel_contact.call_async(Trigger.Request()), 3.)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok() and node.mock_verified and node.cancel_contact.service_is_ready():
            try:
                node.future(node.cancel_contact.call_async(Trigger.Request()), 3.)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
