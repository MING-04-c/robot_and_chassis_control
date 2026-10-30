"""相机点真机检测：默认采图等待 TCP 结果，也可直接使用 YAML 点位。"""

from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
from time import monotonic
from uuid import uuid4
from ament_index_python.packages import get_package_share_directory

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from controller_manager_msgs.srv import ListHardwareComponents
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from geminipro_camer_calibration.capture_geometry import frozen_probe_targets
from geminipro_camer_calibration.geometry import to_pose, transform_matrix
from geminipro_camer_calibration.live_tf import LiveRobotTransforms


REAL_PLUGIN = 'ELITE_CS_ROBOT_ROS_DRIVER/EliteCSPositionHardwareInterface'


def received_points(payload, manifest_sha256):
    """返回本次照片的实测毫米点；其他批次/演示忽略，本次 HOLD 明确失败。"""
    from industrial_pc_protocol import validate_result
    kind, points, _ = validate_result(payload)
    if kind == 'demo' or payload.get('capture_manifest_sha256') != manifest_sha256:
        return None
    if kind == 'hold':
        raise ValueError('主机本次结果无法触碰：'+str(payload.get('hold_reason', payload['status'])))
    return np.asarray(points, dtype=float)


class CameraDetectionTask(Node):
    def __init__(self):
        super().__init__('camera_detection_task')

        def param(name, value):
            return self.declare_parameter(name, value).value

        self.base = param('base_frame', 'base_link')
        self.camera = param('camera_frame', 'terminal_link_1')
        self.probe = param('probe_frame', 'terminal_link_2')
        self.target_source = str(param('target_source', 'tcp'))
        if self.target_source not in ('tcp', 'local'):
            raise ValueError('target_source 必须为 tcp 或 local')
        values = np.asarray(param('camera_points_mm', [-5.370267016871055, -3.820746759094509, 275.]), dtype=float)
        if values.ndim != 1 or values.size == 0 or values.size % 3:
            raise ValueError('camera_points_mm 必须按 x,y,z 顺序，每三个数字为一个点，单位毫米')
        self.points_mm = values.reshape(-1, 3)
        if self.target_source == 'local':
            frozen_probe_targets(np.eye(4), np.eye(4), self.points_mm)
        self.capture_config_file = str(param('capture_config_file', str(
            Path(get_package_share_directory('robot_task_control'))/'config/capture/final_capture.yaml')))
        self.storage = None
        if self.target_source == 'tcp':
            from robot_task_control.final_capture import load_config
            self.storage = Path(load_config(self.capture_config_file)['output_dir']).expanduser()
        self.capture_process = None
        self.results = deque(maxlen=16)
        self.report_dir = Path(param('report_directory', '~/.ros/camera_detection')).expanduser()
        self.position_tolerance = float(param('hold_position_tolerance', .001))
        self.angle_tolerance = math.radians(float(param('hold_angle_tolerance_deg', .5)))
        self.timeouts = {name: float(param(name+'_timeout', value)) for name, value in (
            ('startup', 120.), ('capture', 240.), ('result', 300.), ('contact', 600.))}
        if (not all(math.isfinite(x) and x > 0 for x in self.timeouts.values())
                or not 0 < self.position_tolerance <= .005
                or not 0 < self.angle_tolerance <= math.radians(2.)):
            raise ValueError('超时和静止容差参数无效')
        self.live = LiveRobotTransforms()
        self.phase, self.detail = 'WAIT_READY', '等待真机、真实力数据及检测控制器'
        self.since = monotonic()
        self.detect, self.detect_at = {}, -math.inf
        self.robot_running = False
        self.hardware_ok, self.hardware_at = False, -math.inf
        self.audit_future = None
        self.audit_started = -math.inf
        self.operation = None
        self.cancel_future = None
        self.started_detection = False
        self.run_id = uuid4().hex
        self.snapshot = None
        self.targets = []
        self.holds = 0
        self.ready_since = None
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/task_controller/status', retained)
        self.target_pub = self.create_publisher(PoseArray, '/geminipro_calibration/targets_base', 10)
        self.create_subscription(String, '/detect_controller/status', self.on_detect, 10)
        if self.target_source == 'tcp':
            self.create_subscription(String, '/industrial_pc/result', self.on_result, 10)
        self.external_sub = self.create_subscription(Bool, '/io_and_status_controller/robot_task_running',
                                                     self.on_external, retained)
        self.hardware_client = self.create_client(ListHardwareComponents,
                                                  '/controller_manager/list_hardware_components')
        self.record_client = self.create_client(Trigger, '/detect_controller/record_home')
        self.clear_client = self.create_client(Trigger, '/detect_controller/clear_targets')
        self.start_client = self.create_client(Trigger, '/detect_controller/start')
        self.cancel_client = self.create_client(Trigger, '/detect_controller/cancel')
        self.create_service(Trigger, '/task_controller/stop', self.stop_request)
        self.tick_timer = self.create_timer(.1, self.tick)
        self.get_logger().info(
            f'真机相机点检测：target_source={self.target_source}；'
            + ('稳定采图后等待本次TCP检测点，收到即自动限力触碰'
               if self.target_source == 'tcp' else '直接使用YAML毫米点，就绪后自动限力触碰'))

    def stop_request(self, request, response):
        del request
        self.fail('收到停止请求')
        response.success, response.message = True, '已取消本轮检测；停止确认以检测器状态为准'
        return response

    def on_result(self, message):
        """只缓存本轮采集/等结果阶段收到的观测，不执行主机携带的运动命令。"""
        if self.phase not in ('CAPTURE', 'WAIT_RESULT'):
            return
        try:
            from industrial_pc_protocol import validate_result
            payload = json.loads(message.data)
            validate_result(payload)
            self.results.append(payload)
        except (ValueError, TypeError, AttributeError) as error:
            self.get_logger().warning('忽略无效主机结果：'+str(error))

    def on_external(self, message):
        self.robot_running = message.data

    def on_detect(self, message):
        try:
            self.detect = json.loads(message.data)
            self.detect_at = monotonic()
            if self.phase == 'RUNNING' and self.detect.get('state') == 'HOLD':
                self.holds = max(self.holds, int(self.detect.get('completed_points', 0))+1)
        except (ValueError, TypeError):
            self.detect = {}

    def pose(self, frame):
        """读取新鲜的真实 TF；输出基座下的米制刚体矩阵。"""
        message = self.live.buffer.lookup_transform(self.base, frame, Time())
        age = (self.get_clock().now()-Time.from_msg(message.header.stamp)).nanoseconds/1e9
        if not -.1 <= age <= .3:
            raise ValueError(frame+' TF 过期，不能用旧位姿继续任务')
        return transform_matrix(message.transform)

    def check_hardware(self):
        now = monotonic()
        if self.audit_future is not None:
            if self.audit_future.done():
                components = self.audit_future.result().component
                self.hardware_ok = bool(components) and all(x.plugin_name == REAL_PLUGIN for x in components)
                self.hardware_at = now
                self.audit_future = None
                if not self.hardware_ok:
                    raise ValueError('当前不是 Elite 真机硬件，拒绝执行真机任务')
            elif now-self.audit_started > 3.:
                raise ValueError('真机硬件状态查询超时')
        if self.audit_future is None and now-self.hardware_at > 1. and self.hardware_client.service_is_ready():
            self.audit_future = self.hardware_client.call_async(ListHardwareComponents.Request())
            self.audit_started = now

    def connected(self):
        return (self.hardware_ok and monotonic()-self.hardware_at < 4.
                and self.robot_running and self.external_sub.get_publisher_count() > 0)

    def ready(self):
        return (self.connected() and monotonic()-self.detect_at < 1.
                and self.detect.get('motion_ready') is True
                and self.detect.get('state') in ('IDLE', 'DONE')
                and all(client.service_is_ready() for client in (
                    self.record_client, self.clear_client, self.start_client, self.cancel_client)))

    def transition(self, phase, detail):
        self.phase, self.detail, self.since = phase, detail, monotonic()
        self.get_logger().info(phase+'：'+detail)
        self.save_report()

    def save_report(self):
        self.report_dir.mkdir(parents=True, exist_ok=True)
        path = self.report_dir/(self.run_id+'.json')
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps({
            'run_id': self.run_id, 'state': self.phase, 'message': self.detail,
            'snapshot': self.snapshot, 'targets_base': [x.tolist() for x in self.targets],
            'detect_status': self.detect, 'hold_events': self.holds,
        }, ensure_ascii=False, indent=2))
        temporary.replace(path)

    def check_stationary(self):
        """冻结测试坐标到开始运动前保持原位，避免目标和记录的原位不一致。"""
        for frame, name in ((self.camera, 'base_T_camera'), (self.probe, 'base_T_probe')):
            current, original = self.pose(frame), np.asarray(self.snapshot[name])
            delta = original[:3, :3].T @ current[:3, :3]
            angle = math.acos(float(np.clip((np.trace(delta)-1.)/2., -1., 1.)))
            if (np.linalg.norm(current[:3, 3]-original[:3, 3]) > self.position_tolerance
                    or angle > self.angle_tolerance):
                raise ValueError('测试初始位姿已变化，取消本轮；请重新启动')

    def prepare_targets(self):
        self.snapshot.setdefault('frozen_at_ros_ns', self.get_clock().now().nanoseconds)
        self.snapshot.update(
            run_id=self.run_id, base_frame=self.base, camera_frame=self.camera,
            probe_frame=self.probe, source=str(self.target_source),
            camera_points_mm=self.points_mm.tolist())
        self.targets = frozen_probe_targets(self.snapshot['base_T_camera'],
                                            self.snapshot['base_T_probe'], self.points_mm)
        self.get_logger().info('基座下探针目标（米）：'+str([x[:3, 3].tolist() for x in self.targets]))
        self.operation = self.clear_client.call_async(Trigger.Request())
        self.transition('CLEAR_TARGETS', f'{self.target_source}模式 {len(self.targets)} 个点转换完成；清空旧目标')

    def begin_capture(self):
        self.snapshot.update(run_id=self.run_id, source='tcp',
                             frozen_at_ros_ns=self.get_clock().now().nanoseconds)
        self.transition('CAPTURE', '已冻结拍照位姿，顺序采集并发送 HK、RGB 和深度图')
        self.capture_process = subprocess.Popen([
            sys.executable, '-m', 'robot_task_control.final_capture',
            '--config', self.capture_config_file, '--run-id', self.run_id, '--allow-no-line-camera',
        ], start_new_session=True)

    def finish_capture(self):
        if self.capture_process.poll() is None:
            return
        if self.capture_process.returncode != 0:
            raise ValueError('拍照或发送失败，取消本轮；退出码='+str(self.capture_process.returncode))
        data = json.loads((self.storage/'.capture_index.json').read_text())
        record = data['sessions'][self.run_id]
        self.snapshot.update(capture_manifest_sha256=record['rgbd']['capture_manifest_sha256'],
                             image_directory=str(self.storage/record['directory']),
                             capture_name=record.get('capture_name', record['directory']))
        self.transition('WAIT_RESULT', '照片已发送；等待主机返回本次清单哈希对应的相机毫米点')

    def consume_result(self):
        while self.results:
            payload = self.results.popleft()
            points = received_points(payload, self.snapshot['capture_manifest_sha256'])
            if points is None:
                self.get_logger().warning('忽略其他照片结果或DEMO点，不启动触碰')
                continue
            if not self.ready():
                raise ValueError('收到检测点，但真实力或运动接口未就绪，取消本轮')
            self.points_mm = points
            self.snapshot['result_message_id'] = payload['message_id']
            self.prepare_targets()
            return

    def stop_capture(self):
        """只终止本任务创建的采集进程组，不碰机器人或底盘驱动。"""
        process = self.capture_process
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=1.)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=.5)

    def operation_complete(self):
        if not self.operation.done():
            if monotonic()-self.since > 5.:
                raise ValueError('检测服务回复超时')
            return False
        response = self.operation.result()
        if not response.success:
            raise ValueError(response.message)
        return True

    def advance(self):
        self.check_hardware()
        if self.phase == 'WAIT_READY':
            if monotonic()-self.since > self.timeouts['startup']:
                raise ValueError('启动超时：'+self.detect.get('readiness_message', '等待真机反馈'))
            if not self.ready():
                self.ready_since = None
                return
            try:
                camera, probe = self.pose(self.camera), self.pose(self.probe)
            except (TransformException, ValueError):
                self.ready_since = None
                self.detail = '等待相机和探针的新鲜 TF'
                return
            if self.ready_since is None:
                self.ready_since = monotonic()
                self.snapshot = {'base_T_camera': camera.tolist(), 'base_T_probe': probe.tolist()}
            self.check_stationary()
            if monotonic()-self.ready_since >= 2.:
                if self.target_source == 'tcp':
                    self.begin_capture()
                else:
                    self.prepare_targets()
            return
        if not self.connected():
            raise ValueError('真机外控已断开，取消本轮')
        if self.phase in ('CAPTURE', 'WAIT_RESULT', 'CLEAR_TARGETS', 'RECORD_HOME', 'WAIT_TARGETS'):
            self.check_stationary()
        if self.phase == 'CAPTURE':
            if monotonic()-self.since > self.timeouts['capture']:
                raise ValueError('采集或发送超时，取消本轮')
            self.finish_capture()
        elif self.phase == 'WAIT_RESULT':
            if monotonic()-self.since > self.timeouts['result']:
                raise ValueError('等待主机本次结果超时，不使用旧点继续运动')
            self.consume_result()
        elif self.phase == 'CLEAR_TARGETS' and self.operation_complete():
            self.operation = self.record_client.call_async(Trigger.Request())
            self.transition('RECORD_HOME', '旧队列已清空，记录当前六轴关节原位')
        elif self.phase == 'RECORD_HOME' and self.operation_complete():
            message = PoseArray()
            message.header.frame_id = self.base
            # 本地点位在启动位置冻结后，发布为当前基座目标。
            message.header.stamp = self.get_clock().now().to_msg()
            message.poses = [to_pose(goal) for goal in self.targets]
            self.target_pub.publish(message)
            self.published_at = monotonic()
            self.transition('WAIT_TARGETS', '等待检测器确认整组探针目标')
        elif self.phase == 'WAIT_TARGETS':
            if monotonic()-self.since > 5.:
                raise ValueError('检测器没有确认新目标，检查工作空间和坐标')
            if self.detect_at > self.published_at and self.detect.get('queued_points') == len(self.targets):
                if not self.ready():
                    raise ValueError('开始前真实力或控制系统失去就绪')
                self.started_detection = True
                self.operation = self.start_client.call_async(Trigger.Request())
                self.transition('STARTING', '启动真实限力检测')
        elif self.phase == 'STARTING' and self.operation_complete():
            self.transition('RUNNING', '规划预接近、20 N 停止推进、保持 5 秒、逐点回原位')
        elif self.phase == 'RUNNING':
            if monotonic()-self.detect_at > 2.:
                raise ValueError('检测状态断流')
            if self.detect.get('state') == 'FAULT':
                raise ValueError(self.detect.get('message', '检测失败'))
            if monotonic()-self.since > self.timeouts['contact']:
                raise ValueError('整组检测超时')
            if self.detect.get('state') == 'DONE':
                if self.detect.get('completed_points') != len(self.targets) or self.holds != len(self.targets):
                    raise ValueError('缺少整组触碰保持完成证据')
                self.started_detection = False
                self.transition('DONE', '本轮所有检测点完成并返回测试初始位置')

    def fail(self, message):
        if self.phase not in ('FAULT', 'DONE'):
            if self.started_detection and self.cancel_client.service_is_ready():
                self.cancel_future = self.cancel_client.call_async(Trigger.Request())
            self.transition('FAULT', message)
        self.stop_capture()

    def tick(self):
        try:
            if self.phase not in ('FAULT', 'DONE'):
                self.advance()
        except Exception as error:
            self.fail(str(error))
        self.status_pub.publish(String(data=json.dumps({
            'workflow': 'camera_detection', 'state': self.phase, 'phase': self.phase,
            'target_source': self.target_source,
            'active': self.phase not in ('FAULT', 'DONE'),
            'message': self.detail, 'run_id': self.run_id,
            'detect_state': self.detect.get('state'),
            'completed_points': self.detect.get('completed_points', 0),
            'capture_manifest_sha256': (self.snapshot or {}).get('capture_manifest_sha256'),
        }, ensure_ascii=False)))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping, expedite = False, False

    def interrupt(signum, _):
        nonlocal stopping, expedite
        expedite = expedite or stopping or signum == signal.SIGTERM
        stopping = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = CameraDetectionTask()
        executor.add_node(node)
        while rclpy.ok() and not stopping:
            executor.spin_once(timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        stopping = True
    finally:
        try:
            if node is not None:
                node.tick_timer.cancel()  # 清理时不再调度任务或发送新目标。
                try:
                    if rclpy.ok():
                        node.get_logger().info('收到退出请求，取消本轮任务并清理节点')
                        node.fail('用户退出本轮任务')
                        deadline = monotonic()+1.
                        while (rclpy.ok() and not expedite and monotonic() < deadline
                               and node.cancel_future is not None and not node.cancel_future.done()):
                            executor.spin_once(timeout_sec=.05)
                        if node.cancel_future is not None and not node.cancel_future.done():
                            node.get_logger().warning('退出取消请求未确认；进程退出不代表机器人已停止')
                finally:
                    executor.remove_node(node)
                    try:
                        node.stop_capture()
                    finally:
                        try:
                            node.live.close()
                        finally:
                            node.destroy_node()
        finally:
            executor.shutdown(timeout_sec=.5)
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
