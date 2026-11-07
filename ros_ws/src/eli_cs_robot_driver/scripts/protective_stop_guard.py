#!/usr/bin/env python3
"""公共保护停止恢复器：自动锁住输出，人工确认后只执行一轮恢复。"""
import json
import math
from pathlib import Path
from time import monotonic

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.task import Future
from action_msgs.srv import CancelGoal
from controller_manager_msgs.srv import ListControllers, SwitchController
from eli_common_interface.msg import SafetyMode
from eli_common_interface.srv import GetSafetyMode, GetRobotMode
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool


class ProtectiveStopGuard(Node):
    """服务均异步等待并有截止时间；没有报警自动解锁、自动重试或安全重启逻辑。"""

    def __init__(self):
        super().__init__('protective_stop_guard')
        self.file = Path(str(self.declare_parameter('state_file',
                         '~/.ros/robot_control/protective_stop_guard.json').value)).expanduser()
        self.latched = False
        try:
            self.latched = bool(json.loads(self.file.read_text())['latched'])
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError):
            self.latched = True  # 状态文件损坏也不能默认为已获准恢复。
        self.force_limit = float(self.declare_parameter('recovery_force_limit_n', 5.).value)
        if not math.isfinite(self.force_limit) or self.force_limit <= 0:
            raise ValueError('recovery_force_limit_n must be finite and positive')
        self.force, self.force_at = None, 0.
        self.abort_requested = False
        self.busy = False
        self.phase = 'fault_latched' if self.latched else 'monitoring'
        self.detail = '等待机器人安全状态；确认恢复服务不会自动调用'
        self.safety, self.safety_at = -2, 0.
        self.running = None
        self.joints, self.joints_at, self.stable_since = {}, 0., None
        self.pending = self.waiter = self.poll_future = None
        self.poll_at = 0.
        self.next_status_at = 0.
        self.last_inhibit = None
        self.service_clients = {}
        self.consistent = {'io_and_status_controller', 'force_torque_sensor_broadcaster',
                           'joint_state_broadcaster', 'speed_scaling_state_broadcaster',
                           'tcp_pose_broadcaster'}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.inhibit_pub = self.create_publisher(Bool, '/robot_control/safety_inhibit', qos)
        self.status_pub = self.create_publisher(String, '~/status', qos)
        self.create_subscription(SafetyMode, '/io_and_status_controller/safety_mode',
                                 lambda msg: self.observe_safety(msg.mode), 10)
        self.create_subscription(Bool, '/io_and_status_controller/robot_task_running',
                                 self.on_running, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, '/admittance_controller/measured_wrench',
                                 self.on_force, qos_profile_sensor_data)
        self.create_service(Trigger, '~/confirm_recovery', self.confirm_recovery,
                            callback_group=ReentrantCallbackGroup())
        self.create_service(Trigger, '~/cancel_recovery', self.cancel_recovery,
                            callback_group=ReentrantCallbackGroup())
        # 提前建立 DDS 匹配，避免第一次人工确认时把“尚在发现服务”误认为设备故障。
        for name, kind in {
            '/dashboard_client/get_safety_mode': GetSafetyMode,
            '/dashboard_client/robot_mode': GetRobotMode,
            '/dashboard_client/stop': Trigger,
            '/dashboard_client/unlock_protective_stop': Trigger,
            '/dashboard_client/brake_release': Trigger,
            '/controller_manager/list_controllers': ListControllers,
            '/controller_manager/switch_controller': SwitchController,
            '/controller_stopper/release_safety_latch': Trigger,
            '/io_and_status_controller/resend_external_script': Trigger,
            '/task_controller/safety_pause': Trigger,
            '/task_controller/safety_resume': Trigger,
            '/detect_controller/cancel': Trigger,
            '/admittance_controller/enable': SetBool,
        }.items():
            self.client(name, kind)
        self.create_timer(.1, self.tick)
        self.publish()

    def publish(self):
        if self.last_inhibit != self.latched:
            self.inhibit_pub.publish(Bool(data=self.latched))
            self.last_inhibit = self.latched
        self.status_pub.publish(String(data=json.dumps(dict(
            phase=self.phase, detail=self.detail, latched=self.latched, recovering=self.busy,
            safety_mode=self.safety, robot_task_running=self.running), ensure_ascii=False)))

    def state(self, phase, detail):
        self.phase, self.detail = phase, detail
        self.get_logger().info(f'{phase}: {detail}')
        self.publish()

    def persist(self):
        self.file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.file.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(latched=self.latched, detail=self.detail), ensure_ascii=False))
        temporary.replace(self.file)

    def observe_safety(self, mode):
        self.safety, self.safety_at = int(mode), monotonic()
        if mode in range(3, 15) and not self.latched:
            self.latched = True
            self.state('fault_latched', f'安全模式={mode}；旧任务输出已锁定，等待人工确认')
            try:
                self.persist()
            except OSError as error:
                self.get_logger().error(f'恢复锁状态保存失败：{error}')

    def on_force(self, message):
        force = message.wrench.force
        values = (force.x, force.y, force.z)
        self.force = math.sqrt(sum(x*x for x in values)) if all(math.isfinite(x) for x in values) else None
        self.force_at = monotonic()

    def force_ready(self):
        if '/admittance_controller/enable' not in dict(self.get_service_names_and_types()):
            return True
        return self.force is not None and monotonic()-self.force_at < .5 and self.force < self.force_limit

    def on_running(self, message):
        self.running = message.data

    def on_joints(self, message):
        now = monotonic()
        values = dict(zip(message.name, message.position))
        if len(values) < 6 or not all(math.isfinite(x) for x in values.values()):
            self.stable_since = None
            return
        stopped = (values.keys() == self.joints.keys() and now-self.joints_at < .5
                   and all(abs(values[k]-self.joints[k]) < .001 for k in values))
        if message.velocity:
            stopped = stopped and all(math.isfinite(v) and abs(v) < .01 for v in message.velocity)
        self.stable_since = (self.stable_since or now) if stopped else None
        self.joints, self.joints_at = values, now

    def client(self, name, service):
        if name not in self.service_clients:
            self.service_clients[name] = self.create_client(service, name)
        return self.service_clients[name]

    def cancel_recovery(self, request, response):
        if not self.busy:
            response.success, response.message = False, '没有正在执行的恢复'
            return response
        self.abort_requested = True
        self.latched = True
        self.state('fault_latched', '操作者取消恢复，保持输出锁定')
        response.success, response.message = True, self.detail
        return response

    async def call(self, name, service=Trigger, request=None, timeout=10.):
        if self.abort_requested:
            raise RuntimeError('恢复已被操作者取消')
        client = self.client(name, service)
        if not client.service_is_ready():
            await self.wait_for(client.service_is_ready, f'等待服务就绪：{name}', 3.)
        future = client.call_async(request if request is not None else service.Request())
        self.pending = (client, future, monotonic()+timeout)
        try:
            result = await future
            if result is None:
                raise RuntimeError(f'服务无有效回复：{name}')
            if hasattr(result, 'success') and not result.success:
                raise RuntimeError(f'{name}: {result.message}')
            if hasattr(result, 'ok') and not result.ok:
                raise RuntimeError(f'{name}: 控制器切换被拒绝')
            return result
        finally:
            self.pending = None

    async def wait_for(self, predicate, description, timeout=10.):
        self.state('waiting', description)
        future = Future()
        self.waiter = (future, predicate, monotonic()+timeout)
        try:
            await future
        finally:
            self.waiter = None

    async def settle(self, seconds=.3):
        end = monotonic()+seconds
        await self.wait_for(lambda: monotonic() >= end, '等待软件恢复锁传递', seconds+1.)

    def tick(self):
        now = monotonic()
        if self.pending is not None:
            client, future, deadline = self.pending
            if not future.done() and (now >= deadline or self.abort_requested):
                client.remove_pending_request(future)
                error = RuntimeError('恢复已被操作者取消') if self.abort_requested else TimeoutError('服务回复超时；停止本轮恢复，不重发解锁命令')
                future.set_exception(error)
        if self.waiter is not None:
            future, predicate, deadline = self.waiter
            if not future.done():
                if self.abort_requested:
                    future.set_exception(RuntimeError('恢复已被操作者取消'))
                elif predicate():
                    future.set_result(True)
                elif now >= deadline:
                    future.set_exception(TimeoutError(self.detail+'：等待超时'))
        if not self.busy:
            self.poll_safety(now)
        if now >= self.next_status_at:
            self.publish()
            self.next_status_at = now+1.

    def poll_safety(self, now):
        client = self.client('/dashboard_client/get_safety_mode', GetSafetyMode)
        if self.poll_future is not None:
            if self.poll_future.done():
                try:
                    response = self.poll_future.result()
                    if response is not None and response.success:
                        self.observe_safety(response.mode.mode)
                except Exception as error:
                    self.get_logger().warn(f'安全状态查询失败：{error}')
                self.poll_future = None
            elif now-self.poll_at > 3.:
                client.remove_pending_request(self.poll_future)
                self.poll_future.cancel()
                self.poll_future = None
            return
        if now-self.poll_at >= 1. and client.service_is_ready():
            self.poll_at = now
            self.poll_future = client.call_async(GetSafetyMode.Request())
            self.publish()

    async def confirm_recovery(self, request, response):
        """调用即表示操作者已确认现场安全；回复表示本轮恢复的最终结果。"""
        if self.busy or not self.latched:
            response.success = False
            response.message = '正在恢复，禁止重复请求' if self.busy else '没有待确认的安全暂停'
            return response
        now = monotonic()
        if (now-self.safety_at > 3. or self.safety not in (1, 2, 3)
                or now-self.joints_at > .5 or self.stable_since is None
                or now-self.stable_since < 1.):
            response.success = False
            response.message = '需新鲜安全状态、连续停稳1秒及新鲜关节数据；急停/故障等不在此服务恢复范围'
            return response
        if not self.force_ready():
            response.success, response.message = False, f'需新鲜去零偏力数据且合力小于 {self.force_limit:g}N；请松开末端'
            return response
        if len(self.get_publishers_info_by_topic('/robot_control/safety_inhibit')) != 1:
            response.success, response.message = False, '检测到重复恢复器或未知恢复锁发布者，拒绝恢复'
            return response
        if self.inhibit_pub.get_subscription_count() == 0:
            response.success, response.message = False, '没有软件恢复锁订阅者，拒绝解除保护停止'
            return response
        self.abort_requested = False
        self.busy = True
        try:
            self.state('quiescing', '保留任务步骤，停止程序并清除旧目标')
            for name, service, req in [
                ('/task_controller/safety_pause', Trigger, None),
                ('/detect_controller/cancel', Trigger, None),
                ('/admittance_controller/enable', SetBool, SetBool.Request(data=False)),
            ]:
                if self.client(name, service).service_is_ready():
                    await self.call(name, service, req)
            await self.call('/dashboard_client/stop')
            await self.wait_for(lambda: self.running is False, '等待外控程序停止')
            controllers = await self.call('/controller_manager/list_controllers', ListControllers)
            names = [c.name for c in controllers.controller
                     if c.state == 'active' and c.name not in self.consistent]
            if names:
                req = SwitchController.Request(deactivate_controllers=names, strictness=2)
                await self.call('/controller_manager/switch_controller', SwitchController, req)
            for name in ('/scaled_joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal',
                         '/execute_trajectory/_action/cancel_goal', '/move_action/_action/cancel_goal'):
                if self.client(name, CancelGoal).service_is_ready():
                    await self.call(name, CancelGoal)
            safety = await self.call('/dashboard_client/get_safety_mode', GetSafetyMode)
            if safety.mode.mode == SafetyMode.PROTECTIVE_STOP:
                self.state('unlocking', '人工已确认；本轮只发送一次保护停止解锁')
                await self.call('/dashboard_client/unlock_protective_stop')
            elif safety.mode.mode not in (SafetyMode.NORMAL, SafetyMode.REDUCED):
                raise RuntimeError('安全状态不允许恢复，不执行安全重启/故障清除')
            deadline = monotonic()+15.
            while True:
                safety = await self.call('/dashboard_client/get_safety_mode', GetSafetyMode)
                self.observe_safety(safety.mode.mode)
                if self.safety in (1, 2):
                    break
                if self.safety != 3 or monotonic() >= deadline:
                    raise RuntimeError('解锁后安全状态未恢复正常，结束本轮恢复')
                await self.settle(.5)
            mode = await self.call('/dashboard_client/robot_mode', GetRobotMode)
            if mode.mode.mode == 5:  # IDLE：抱闸未释放；不处理 POWER_OFF 或其他故障模式。
                await self.call('/dashboard_client/brake_release', timeout=120.)
                mode = await self.call('/dashboard_client/robot_mode', GetRobotMode)
            if mode.mode.mode != 7:
                raise RuntimeError('机器人未处于 RUNNING，请先检查上电及抱闸状态')
            self.state('external_control', '恢复外控；运动输出仍锁定')
            await self.call('/io_and_status_controller/resend_external_script')
            await self.wait_for(lambda: self.running is True, '等待外控程序重新就绪', 20.)
            current = await self.call('/controller_manager/list_controllers', ListControllers)
            if not any(c.name == 'scaled_joint_trajectory_controller' and c.state == 'active'
                       for c in current.controller):
                await self.call('/controller_manager/switch_controller', SwitchController,
                    SwitchController.Request(activate_controllers=['scaled_joint_trajectory_controller'], strictness=2))
            current = await self.call('/controller_manager/list_controllers', ListControllers)
            if not any(c.name == 'scaled_joint_trajectory_controller' and c.state == 'active'
                       for c in current.controller):
                raise RuntimeError('轨迹控制器未激活；保持任务暂停')
            safety = await self.call('/dashboard_client/get_safety_mode', GetSafetyMode)
            self.observe_safety(safety.mode.mode)
            if self.safety not in (1, 2):
                raise RuntimeError('恢复期间再次发生安全报警，停止本轮恢复')
            if (monotonic()-self.joints_at > .5 or self.stable_since is None
                    or monotonic()-self.stable_since < 1.):
                raise RuntimeError('恢复期间关节数据失效或尚未停稳，保持输出锁定')
            if not self.force_ready():
                raise RuntimeError('恢复期间力数据失效或末端仍受力，保持输出锁定')
            self.latched = False
            self.publish()
            await self.settle()
            await self.call('/controller_stopper/release_safety_latch')
            if self.latched or self.safety not in (1, 2):
                raise RuntimeError('输出恢复前再次出现安全暂停')
            if self.client('/task_controller/safety_resume', Trigger).service_is_ready():
                await self.call('/task_controller/safety_resume')
            self.state('recovered', '恢复完成；支持的机械臂任务从当前位置重新准备，检测接触任务需重新启动')
            self.persist()
            response.success, response.message = True, self.detail
        except Exception as error:
            self.latched = True
            self.state('recovery_failed', str(error)+'；保持暂停，需重新检查后人工确认')
            try:
                self.persist()
            except OSError:
                pass
            response.success, response.message = False, self.detail
        finally:
            self.busy = False
            self.publish()
        return response


def main():
    rclpy.init()
    node = ProtectiveStopGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
