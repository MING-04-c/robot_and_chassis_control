"""双关节姿态测试：核查硬件模式后自动执行往返，支持显式真机模式。"""
import json
import os
from pathlib import Path
from time import monotonic
import numpy as np
import signal
import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from uuid import uuid4
from controller_manager_msgs.srv import ListHardwareComponents, ListControllers
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState, DisplayTrajectory, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from ament_index_python.packages import get_package_share_directory
from eli_cs_robot_moveit_config.joint_postures import JOINTS, load_postures, joint_constraints


class JointPostureControl(Node):
    def __init__(self, node_name='joint_posture_test'):
        super().__init__(node_name)
        self.simulation = bool(self.declare_parameter('simulation', True).value)
        self.startup_timeout = float(self.declare_parameter('startup_timeout', 480.).value)
        self.abort_reason = None
        self.motion_started = False
        self.initialized = False
        self.manage_admittance = bool(self.declare_parameter('manage_admittance', False).value)
        self.disable_client = self.create_client(SetBool, '/admittance_controller/enable')
        self.request_id = ''
        self.requested_posture = None
        self.busy = False
        self.status_pub = self.create_publisher(String, '~/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.robot_running = False
        self.safety_known = self.simulation
        self.safety_inhibited = not self.simulation
        domain = int(self.declare_parameter('simulation_domain', 190).value)
        if self.simulation and (not 128 <= domain <= 232 or os.environ.get('ROS_DOMAIN_ID') != str(domain)):
            raise ValueError('本测试只允许在指定的独立ROS仿真域运行')
        self.postures = load_postures(self.declare_parameter('postures_file', str(
            Path(get_package_share_directory('eli_cs_robot_moveit_config'))/'config/arm_joint_postures.yaml')).value)
        self.path = Path(self.declare_parameter('report_file', '/tmp/joint_posture_test_report.json').value).expanduser()
        self.scale = float(self.declare_parameter('velocity_scaling', .15).value)
        if not 0 < self.scale <= .3:
            raise ValueError('速度缩放必须在(0,0.3]内')
        self.joints, self.joints_at = None, -1.
        self.ground = False
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Bool, '/robot_control/ground_ready',
            self.on_ground, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.external_sub = self.create_subscription(Bool, '/io_and_status_controller/robot_task_running',
                                                     self.on_external, retained)
        self.safety_sub = self.create_subscription(Bool, '/robot_control/safety_inhibit', self.on_safety, retained)
        self.controller_name = 'joint_trajectory_controller' if self.simulation else 'scaled_joint_trajectory_controller'
        self.hardware = self.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.controller_action = ActionClient(self, FollowJointTrajectory,
            '/'+self.controller_name+'/follow_joint_trajectory')
        self.execution_ready = False
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.planner = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.display = self.create_publisher(DisplayTrajectory, '/display_planned_path', 10)
        self.report = {'simulation_only': self.simulation, 'state': 'WAIT_READY',
                       'ground_top_base_m': .08, 'joint_order': JOINTS,
                       'postures_deg': {k: np.rad2deg(v).tolist() for k, v in self.postures.items()},
                       'validity': {}, 'segments': []}
        self.goal_handle = None
        self.sending = None
        self.create_timer(.5, self.publish_status)

    def on_joints(self, message):
        if (len(message.position) == len(message.name) and all(name in message.name for name in JOINTS)
                and np.isfinite(message.position).all()):
            self.joints, self.joints_at = message, monotonic()

    def write(self, state):
        self.report['state'] = state
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.report, ensure_ascii=False, indent=2)+'\n')
        temporary.replace(self.path)
        self.get_logger().info('关节姿态：'+state)
        self.publish_status()

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'request_id': self.request_id, 'state': self.report['state'],
            'active': self.busy, 'error': self.report.get('error'),
            'posture': self.report.get('posture'), 'simulation': self.simulation,
            'native_error_code': self.report.get('native_error_code'),
        }, ensure_ascii=False)))

    def request_posture(self, posture, response):
        if self.busy or self.abort_reason:
            response.success, response.message = False, '关节任务忙或已中止，不能重叠执行'
            return response
        self.busy = True
        self.request_id = uuid4().hex
        self.requested_posture = posture
        self.report.update(request_id=self.request_id, posture=posture, segments=[])
        self.report.pop('error', None)
        self.report.pop('native_error_code', None)
        self.write('QUEUED')
        response.success, response.message = True, json.dumps({'request_id': self.request_id})
        return response

    def stop_request(self, request, response):
        del request
        self.abort('联合任务取消关节运动')
        response.success, response.message = True, '已请求取消本节点的运动'
        return response

    def prepare_control_mode(self):
        if not self.manage_admittance:
            return
        deadline = monotonic()+self.startup_timeout
        while rclpy.ok() and monotonic() < deadline:
            if self.abort_reason:
                raise RuntimeError(self.abort_reason)
            if self.disable_client.service_is_ready() and self.real_ready() and self.ground:
                response = self.future(self.disable_client.call_async(SetBool.Request(data=False)), 10.)
                if not response.success:
                    raise RuntimeError('关闭导纳并切换轨迹控制失败：'+response.message)
                return
            rclpy.spin_once(self, timeout_sec=.1)
        raise RuntimeError('等待导纳模式切换接口超时')

    def abort(self, reason):
        self.abort_reason = self.abort_reason or reason
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def on_ground(self, message):
        self.ground = message.data
        if not self.ground and self.motion_started:
            self.abort('地面场景确认丢失')

    def on_external(self, message):
        self.robot_running = message.data
        if not self.simulation and not self.robot_running and self.motion_started:
            self.abort('机器人外控连接断开')

    def on_safety(self, message):
        self.safety_known, self.safety_inhibited = True, message.data
        if message.data and self.motion_started:
            self.abort('机器人安全锁定，取消本次往返')

    def future(self, future, timeout=30.):
        deadline = monotonic()+timeout
        while rclpy.ok() and not future.done() and monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if self.abort_reason:
                raise RuntimeError(self.abort_reason)
            if self.motion_started and self.joints is not None and monotonic()-self.joints_at > 1.:
                self.abort('实际关节状态断流')
                raise RuntimeError(self.abort_reason)
        if not rclpy.ok() or not future.done() or future.result() is None:
            raise RuntimeError('服务或执行响应超时')
        if self.abort_reason:
            raise RuntimeError(self.abort_reason)
        return future.result()

    def real_ready(self):
        return self.simulation or (self.robot_running and self.external_sub.get_publisher_count() > 0
            and self.safety_known and not self.safety_inhibited and self.safety_sub.get_publisher_count() > 0)

    def verify_hardware(self):
        """模式必须与硬件插件严格一致，防止混用假硬件/真机驱动。"""
        components = self.future(self.hardware.call_async(ListHardwareComponents.Request())).component
        expected = ('mock_components/GenericSystem' if self.simulation else
                    'ELITE_CS_ROBOT_ROS_DRIVER/EliteCSPositionHardwareInterface')
        if not components or any(x.plugin_name != expected for x in components):
            raise RuntimeError('硬件插件与选择的仿真/真机模式不一致，拒绝执行')
        if not self.ground or not self.real_ready():
            raise RuntimeError('地面、机器人外控或安全状态尚未就绪')

    @staticmethod
    def state(positions):
        return RobotState(joint_state=JointState(name=JOINTS, position=list(positions)), is_diff=False)

    def current(self):
        if self.joints is None or monotonic()-self.joints_at > 1.:
            raise RuntimeError('缺少新鲜关节状态')
        return [self.joints.position[self.joints.name.index(j)] for j in JOINTS]

    def validate(self, name, positions):
        req = GetStateValidity.Request(robot_state=self.state(positions), group_name='cs_manipulator')
        result = self.future(self.validity.call_async(req))
        self.report['validity'][name] = {'valid': result.valid, 'contacts': [
            {'body1': c.contact_body_1, 'body2': c.contact_body_2,
             'position': [c.position.x, c.position.y, c.position.z], 'depth_m': c.depth}
            for c in result.contacts]}
        self.write('CHECK_'+name)
        if not result.valid:
            raise RuntimeError(name+'姿态无效/碰撞，请查看报告中的接触物体；不删除碰撞检查强行运动')

    def check_ground_block(self):
        """仅查询一个伸入地面的虚拟关节状态，绝不把它作为执行目标。"""
        unsafe = list(self.postures['detection_ready'])
        unsafe[1] = float(np.pi/2.)
        result = self.future(self.validity.call_async(GetStateValidity.Request(
            robot_state=self.state(unsafe), group_name='cs_manipulator')))
        ground_contacts = [(c.contact_body_1, c.contact_body_2) for c in result.contacts
                           if 'robot_base_ground' in (c.contact_body_1, c.contact_body_2)]
        self.report['ground_block_check'] = {'query_only_not_executed': True,
            'valid': result.valid, 'ground_contacts': ground_contacts}
        if result.valid or not ground_contacts:
            raise RuntimeError('虚拟穿地状态没有被地面碰撞检查阻止')
        self.write('GROUND_BLOCK_CONFIRMED')

    def plan(self, start, target, label):
        req = GetMotionPlan.Request()
        motion = req.motion_plan_request
        motion.group_name, motion.pipeline_id = 'cs_manipulator', 'ompl'
        motion.allowed_planning_time = 15.
        motion.num_planning_attempts = 5
        motion.start_state = self.state(start)
        motion.goal_constraints = [joint_constraints(target)]
        motion.max_velocity_scaling_factor = self.scale
        motion.max_acceleration_scaling_factor = .1
        result = self.future(self.planner.call_async(req), 90.).motion_plan_response
        segment = {'name': label, 'start_rad': start, 'target_rad': target,
                   'planning_error_code': result.error_code.val,
                   'trajectory_points': len(result.trajectory.joint_trajectory.points)}
        self.report['segments'].append(segment)
        self.write('PLAN_'+label)
        if result.error_code.val != MoveItErrorCodes.SUCCESS or not result.trajectory.joint_trajectory.points:
            self.report['native_error_code'] = int(result.error_code.val)
            raise RuntimeError(label+'双向预规划失败：'+str(result.error_code.val))
        return result.trajectory

    def wait_ready(self):
        self.execution_ready = False
        self.write('WAIT_READY')
        deadline = monotonic()+self.startup_timeout
        last_wait_log = -1.
        while rclpy.ok() and monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
            if self.abort_reason:
                raise RuntimeError(self.abort_reason)
            if monotonic()-last_wait_log >= 5.:
                last_wait_log = monotonic()
                self.get_logger().info(f'等待启动：ground={self.ground}, '
                    f'external={self.robot_running}, safety_known={self.safety_known}, '
                    f'inhibited={self.safety_inhibited}, controller={self.controller_name}')
            if (self.ground and self.real_ready() and self.joints is not None and self.hardware.service_is_ready()
                    and self.validity.service_is_ready() and self.planner.service_is_ready()
                    and self.execute.server_is_ready() and self.controller_action.server_is_ready()
                    and self.controllers.service_is_ready()):
                states = {c.name: c.state for c in self.future(
                    self.controllers.call_async(ListControllers.Request()), 5.).controller}
                if all(states.get(name) == 'active' for name in (
                        'joint_state_broadcaster', self.controller_name)):
                    self.execution_ready = True
                    break
        self.verify_hardware()
        if not self.execution_ready:
            raise RuntimeError('轨迹控制器未激活，不提前发送轨迹')
        # 初始姿态保留2秒，等待MoveIt内部轨迹客户端完成DDS发现，也便于RViz观察。
        deadline = monotonic()+((2. if self.simulation else 5.) if not self.initialized else 0.)
        while rclpy.ok() and monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
        self.initialized = True

    def run(self):
        self.wait_ready()
        travel, ready = self.postures['travel'], self.postures['detection_ready']
        self.validate('travel', travel)
        self.validate('detection_ready', ready)
        self.check_ground_block()
        initial = self.current()
        bootstrap = None
        if max(abs(a-b) for a, b in zip(initial, travel)) > .01:
            if self.simulation:
                raise RuntimeError('假硬件未按第一姿态初始化')
            self.validate('actual_start', initial)
            bootstrap = self.plan(initial, travel, 'actual_to_travel')
        # 所有段先规划成功；真机若不在第一姿态，先按关节目标到第一姿态。
        outward = self.plan(travel, ready, 'travel_to_detection_ready')
        backward = self.plan(ready, travel, 'detection_ready_to_travel')
        segments = ([] if bootstrap is None else [('actual_to_travel', initial, travel, bootstrap)])
        segments += [('travel_to_detection_ready', travel, ready, outward),
                     ('detection_ready_to_travel', ready, travel, backward)]
        self.execute_segments(segments)

    def move_posture(self, posture):
        """联合任务用：中转只到中转；行驶位先经中转，再返回行驶关节角。"""
        self.prepare_control_mode()
        self.wait_ready()
        current = self.current()
        self.validate('actual_start', current)
        names = ['detection_ready', 'travel'] if posture == 'travel' else ['detection_ready']
        if max(abs(a-b) for a, b in zip(current, self.postures[posture])) <= .01:
            self.write('DONE')
            return
        segments = []
        for name in names:
            target = self.postures[name]
            self.validate(name, target)
            if max(abs(a-b) for a, b in zip(current, target)) > .01:
                label = 'to_'+name
                segments.append((label, current, target, self.plan(current, target, label)))
            current = target
        self.execute_segments(segments)

    def execute_segments(self, segments):
        for segment_index, (label, start, target, trajectory) in enumerate(segments):
            self.verify_hardware()
            if max(abs(a-b) for a, b in zip(self.current(), start)) > .01:
                raise RuntimeError('实际关节已偏离本段规划起点')
            self.display.publish(DisplayTrajectory(trajectory_start=self.state(start), trajectory=[trajectory]))
            self.write('EXECUTE_'+label)
            self.motion_started = True
            sending = self.execute.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory))
            def cancel_late(future):
                if self.abort_reason and future.result() is not None and future.result().accepted:
                    future.result().cancel_goal_async()
            self.sending = sending
            sending.add_done_callback(cancel_late)
            self.goal_handle = self.future(sending)
            if not self.goal_handle.accepted:
                raise RuntimeError('轨迹被拒绝')
            duration = trajectory.joint_trajectory.points[-1].time_from_start
            result = self.future(self.goal_handle.get_result_async(), max(300., duration.sec+30.) if not self.simulation else duration.sec+30.)
            self.goal_handle = None
            if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
                self.report['native_error_code'] = int(result.result.error_code.val)
                raise RuntimeError('轨迹执行失败：'+str(result.result.error_code.val))
            deadline = monotonic()+3.
            while monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.05)
                if max(abs(a-b) for a, b in zip(self.current(), target)) <= .01:
                    break
            error = max(abs(a-b) for a, b in zip(self.current(), target))
            if error > .01:
                raise RuntimeError('执行返回成功，但实际关节尚未到位')
            self.report['segments'][segment_index]['final_error_rad'] = error
            deadline = monotonic()+2.
            while rclpy.ok() and monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.05)
        self.motion_started = False
        self.write('DONE')


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = JointPostureControl()
    signal.signal(signal.SIGINT, lambda *_: setattr(node, 'abort_reason', '收到Ctrl+C，取消本次任务'))
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, 'abort_reason', '收到退出信号，取消本次任务'))
    try:
        try:
            node.run()
        except RuntimeError as error:
            if node.goal_handle is not None:
                node.goal_handle.cancel_goal_async()
            node.report['error'] = str(error)
            node.write('FAILED')
            node.get_logger().error(str(error))
        while rclpy.ok() and not node.abort_reason:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node.sending is not None and not node.sending.done() and rclpy.ok():
            rclpy.spin_until_future_complete(node, node.sending, timeout_sec=3.)
            if not node.sending.done():
                node.get_logger().error('运动请求结果尚未确认，请检查机器人是否停止')
            elif node.sending.result() is not None and node.sending.result().accepted:
                node.goal_handle = node.sending.result()
        if node.goal_handle is not None and rclpy.ok():
            cancelled = node.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancelled, timeout_sec=3.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def service_main():
    """常驻关节任务服务；服务回调仅排队，规划/执行在主循环中处理。"""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = JointPostureControl('arm_joint_postures')
    signal.signal(signal.SIGINT, lambda *_: setattr(node, 'abort_reason', '收到退出信号'))
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, 'abort_reason', '收到退出信号'))
    node.create_service(Trigger, '~/travel', lambda req, response: node.request_posture('travel', response))
    node.create_service(Trigger, '~/detection_ready', lambda req, response: node.request_posture('detection_ready', response))
    node.create_service(Trigger, '~/cancel', node.stop_request)
    node.write('IDLE')
    try:
        while rclpy.ok() and not node.abort_reason:
            rclpy.spin_once(node, timeout_sec=.1)
            if node.requested_posture is None:
                continue
            posture, node.requested_posture = node.requested_posture, None
            try:
                node.move_posture(posture)
            except Exception as error:
                node.abort(str(error))
                node.report['error'] = str(error)
                node.write('FAILED')
            finally:
                node.busy = False
                node.publish_status()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.abort_reason = node.abort_reason or '退出'
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.abort_reason = node.abort_reason or '服务退出'
        if node.sending is not None and not node.sending.done() and rclpy.ok():
            rclpy.spin_until_future_complete(node, node.sending, timeout_sec=3.)
            if node.sending.done() and node.sending.result() is not None and node.sending.result().accepted:
                node.goal_handle = node.sending.result()
            elif not node.sending.done():
                node.get_logger().error('运动请求结果尚未确认，请检查机器人是否停止')
        if node.goal_handle is not None and rclpy.ok():
            rclpy.spin_until_future_complete(node, node.goal_handle.cancel_goal_async(), timeout_sec=3.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
