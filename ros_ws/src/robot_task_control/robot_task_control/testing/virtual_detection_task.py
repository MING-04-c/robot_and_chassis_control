"""生成当前探头下方的虚拟点，自动调用检测流程；模拟力仅用于 mock 硬件。"""

import json
import math
import signal
from time import monotonic
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from geometry_msgs.msg import PoseArray, WrenchStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from geminipro_camer_calibration.live_tf import LiveRobotTransforms
from geminipro_camer_calibration.geometry import matrix, transform_matrix, to_pose


class VirtualDetectionTask(Node):
    def __init__(self):
        super().__init__("virtual_detection_task")

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.simulation = param("simulation", False)
        self.base = param("base_frame", "base_link")
        self.probe = param("probe_frame", "terminal_link_2")
        self.tool = param("tool_frame", "tool0")
        self.real_target_frame = param("real_target_frame", "tool0")
        if self.real_target_frame not in (self.tool, self.probe):
            raise ValueError("real_target_frame 必须为 tool0 或所配置的 probe_frame")
        self.fixed_real_target = param("fixed_real_target", True)
        self.fixed_target_position = np.asarray(
            param("real_target_position", [-0.3, 0.1, 0.177]), dtype=float
        )
        if (
            self.fixed_target_position.shape != (3,)
            or not np.isfinite(self.fixed_target_position).all()
        ):
            raise ValueError("real_target_position 必须为三个有限米制坐标")
        self.contact_rotation = None
        real_offset = param("point_offset", [0.0, 0.0, -0.005])
        simulated_offset = param("simulation_point_offset", [0.0, 0.0, -0.03])
        self.offset = np.asarray(
            simulated_offset if self.simulation else real_offset, dtype=float
        )
        self.settle = float(param("readiness_seconds", 2.0))
        self.startup_timeout = float(param("startup_timeout", 90.0))
        self.task_timeout = float(param("task_timeout", 90.0))
        self.contact_force = float(param("simulated_contact_force", 21.0))
        if (
            self.offset.shape != (3,)
            or not np.isfinite(self.offset).all()
            or not 0 < np.linalg.norm(self.offset) <= 0.05
            or self.offset[2] >= 0
        ):
            raise ValueError(
                "point_offset 必须为有限、长度不超过 5 cm 且 Z 向下的基座偏移"
            )
        if not all(
            math.isfinite(x) and x > 0
            for x in (
                self.settle,
                self.startup_timeout,
                self.task_timeout,
                self.contact_force,
            )
        ):
            raise ValueError("等待时间和模拟力必须为有限正数")
        self.live_tf = LiveRobotTransforms()
        self.buffer = self.live_tf.buffer
        self.stage = "WAIT_READY"
        self.detail = ""
        self.started = monotonic()
        self.ready_since = None
        self.status = {}
        self.status_at = -math.inf
        self.hardware_verified = False
        self.controllers_active = False
        self.robot_running = False
        self.checked_at = -math.inf
        self.last_check = -math.inf
        self.hardware_future = None
        self.controllers_future = None
        self.operation = None
        self.target = None
        self.origin = None
        self.trace = []
        self.max_displacement = 0.0
        self.last_visual = -math.inf
        self.last_ready_log = -math.inf
        self.prepare = self.create_client(Trigger, "/detect_controller/prepare_initial")
        self.record = self.create_client(Trigger, "/detect_controller/record_home")
        self.start = self.create_client(Trigger, "/detect_controller/start")
        self.cancel = self.create_client(Trigger, "/detect_controller/cancel")
        self.controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.hardware = self.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )
        self.publisher = self.create_publisher(
            PoseArray, "/geminipro_calibration/targets_base", 10
        )
        self.create_subscription(
            String, "/detect_controller/status", self.on_status, 10
        )
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if not self.simulation:
            self.robot_task_sub = self.create_subscription(
                Bool,
                "/io_and_status_controller/robot_task_running",
                self.on_robot_task,
                latched,
            )
        self.status_pub = self.create_publisher(String, "~/status", latched)
        self.marker_pub = self.create_publisher(MarkerArray, "~/markers", latched)
        # 真机模式根本不创建模拟力发布器。
        if self.simulation:
            self.force_pub = self.create_publisher(
                WrenchStamped, "/admittance_controller/external_wrench", 10
            )
            self.raw_pub = self.create_publisher(WrenchStamped, "/sensor/wrench", 10)
        self.create_timer(0.02, self.tick)
        self.get_logger().info(
            f"检测测试等待控制系统就绪，simulation={self.simulation}"
        )

    def on_robot_task(self, message):
        self.robot_running = bool(message.data)
        if not self.robot_running:
            self.ready_since = None
            if self.stage not in ("WAIT_READY", "DONE", "FAILED"):
                self.fail("机械臂外控程序已停止或连接断开，取消检测任务")

    def external_control_ready(self):
        return self.simulation or (
            self.robot_running and self.robot_task_sub.get_publisher_count() > 0
        )

    def report(self, stage, detail):
        self.stage, self.detail = stage, detail
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "state": stage,
                        "message": detail,
                        "simulation": self.simulation,
                        "target_offset_mm": float(np.linalg.norm(self.offset) * 1000.0),
                        "max_probe_displacement_mm": self.max_displacement,
                    },
                    ensure_ascii=False,
                )
            )
        )
        self.get_logger().info(f"{stage}: {detail}")

    def destroy_node(self):
        self.live_tf.close()
        return super().destroy_node()

    def on_status(self, message):
        try:
            self.status = json.loads(message.data)
            self.status_at = monotonic()
        except (ValueError, TypeError):
            self.status = {}

    def pose(self):
        value = self.buffer.lookup_transform(self.base, self.probe, Time())
        age = (
            self.get_clock().now() - Time.from_msg(value.header.stamp)
        ).nanoseconds / 1e9
        if not -0.1 <= age <= 0.3:
            raise ValueError("探头 TF 已过期")
        return transform_matrix(value.transform)

    def check_hardware(self, future):
        try:
            components = future.result().component
            if not components:
                return
            mock = all(
                x.plugin_name == "mock_components/GenericSystem" for x in components
            )
            if self.simulation != mock:
                self.fail("实际硬件类型与 simulation 参数不一致，拒绝测试")
                return
            self.hardware_verified = True
        except Exception as error:
            self.fail(str(error))

    def check_controllers(self, future):
        try:
            states = {x.name: x.state for x in future.result().controller}
            name = (
                "joint_trajectory_controller"
                if self.simulation
                else "scaled_joint_trajectory_controller"
            )
            active = [name, "joint_state_broadcaster"]
            inactive = []
            if self.simulation:
                # 等待所有启动 spawner 完成，避免运动期间再加载插件挤占 TF 更新。
                active += [
                    "io_and_status_controller",
                    "speed_scaling_state_broadcaster",
                    "force_torque_sensor_broadcaster",
                    "tcp_pose_broadcaster",
                ]
                inactive = ["forward_position_controller", "freedrive_controller"]
            self.controllers_active = all(
                states.get(x) == "active" for x in active
            ) and all(states.get(x) == "inactive" for x in inactive)
            self.checked_at = monotonic()
        except Exception as error:
            self.fail(str(error))

    def request(self, client, callback):
        future = client.call_async(Trigger.Request())
        self.operation = (future, callback, monotonic())

    def publish_target(self):
        self.origin = self.pose()
        self.target = self.origin.copy()
        if not self.simulation and self.fixed_real_target:
            self.target[:3, 3] = self.fixed_target_position
            self.target[:3, :3] = self.contact_rotation
            if self.real_target_frame == self.tool:
                # 检测器内部仍统一使用探头位姿。把用户的 tool0 测试目标先转换，
                # 后续求逆补偿后，实际 tool0 目标就严格等于用户填写的 XYZ。
                tool_probe = transform_matrix(
                    self.buffer.lookup_transform(
                        self.tool, self.probe, Time()
                    ).transform
                )
                # 使用检测控制器为初始点锁存的同一 tool0 方向，避免两阶段采样不一致。
                orientation = self.status.get("initial_tool_orientation_xyzw")
                if orientation is None:
                    raise RuntimeError("缺少初始 tool0 方向；请编译并重启 detect_control")
                self.target[:3, :3] = matrix([0.0, 0.0, 0.0], orientation)[:3, :3]
                self.target = self.target @ tool_probe
                self.get_logger().info(
                    f"固定 tool0 测试目标：{self.fixed_target_position.tolist()} m"
                )
            self.offset = self.target[:3, 3] - self.origin[:3, 3]
        else:
            self.target[:3, 3] += self.offset
        message = PoseArray()
        message.header.frame_id = self.base
        message.header.stamp = self.get_clock().now().to_msg()
        message.poses = [to_pose(self.target)]
        self.publisher.publish(message)
        self.published_at = monotonic()
        self.report(
            "WAIT_POINT",
            f"已发送探头检测点 {self.target[:3, 3].tolist()} m（基座坐标）",
        )

    def publish_visual(self):
        if self.origin is None or monotonic() - self.last_visual < 0.2:
            return
        self.last_visual = monotonic()
        try:
            current = self.pose()[:3, 3]
        except Exception:
            # 显示不使用旧位置补造轨迹；控制流程仍独立检查 TF 时效。
            return
        if not self.trace or np.linalg.norm(current - self.trace[-1]) > 0.0002:
            self.trace.append(current.copy())
            self.trace = self.trace[-1000:]

        def point(xyz):
            return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))

        def marker(index, kind, color):
            item = Marker()
            item.header.frame_id = self.base
            item.header.stamp = self.get_clock().now().to_msg()
            item.ns = "detection_test"
            item.id = index
            item.type = kind
            item.action = Marker.ADD
            item.pose.orientation.w = 1.0
            item.color.r, item.color.g, item.color.b, item.color.a = color
            return item

        items = []
        for index, xyz, color, size in [
            (0, self.origin[:3, 3], (0.0, 1.0, 0.0, 1.0), 0.012),
            (1, self.target[:3, 3], (1.0, 0.1, 0.1, 1.0), 0.012),
            (2, current, (1.0, 1.0, 0.0, 1.0), 0.008),
        ]:
            item = marker(index, Marker.SPHERE, color)
            item.pose.position = point(xyz)
            item.scale.x = item.scale.y = item.scale.z = size
            items.append(item)
        arrow = marker(3, Marker.ARROW, (0.2, 0.5, 1.0, 1.0))
        arrow.points = [point(self.origin[:3, 3]), point(self.target[:3, 3])]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.002, 0.007, 0.009
        items.append(arrow)
        trail = marker(4, Marker.LINE_STRIP, (0.0, 1.0, 1.0, 1.0))
        trail.scale.x = 0.002
        trail.points = (
            [point(p) for p in self.trace]
            if len(self.trace) > 1
            else [point(current), point(current)]
        )
        items.append(trail)
        text = marker(5, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
        text.pose.position = point(self.origin[:3, 3] + np.array([0.0, 0.0, 0.08]))
        text.scale.z = 0.018
        distance = np.linalg.norm(current - self.origin[:3, 3]) * 1000.0
        self.max_displacement = max(self.max_displacement, float(distance))
        text.text = (
            f"DETECTION {'SIM' if self.simulation else 'REAL'} / {self.status.get('state', self.stage)}"
            f"\nTarget offset: {np.linalg.norm(self.offset)*1000.:.0f} mm"
            f"\nProbe displacement: {distance:.1f} mm / max {self.max_displacement:.1f} mm"
        )
        items.append(text)
        self.marker_pub.publish(MarkerArray(markers=items))

    def fake_force(self):
        if not self.simulation or not self.hardware_verified or self.stage == "FAILED":
            return
        force = 0.0
        if self.target is not None and self.status.get("state") in (
            "DESCEND",
            "STOP_CONTACT",
            "HOLD",
        ):
            if (
                self.status.get("state") == "HOLD"
                or self.pose()[2, 3] <= self.target[2, 3] + 0.0002
            ):
                force = self.contact_force
        message = WrenchStamped()
        message.header.frame_id = self.base
        message.header.stamp = self.get_clock().now().to_msg()
        message.wrench.force.z = force
        self.force_pub.publish(message)
        self.raw_pub.publish(message)

    def fail(self, detail):
        if self.stage == "FAILED":
            return
        self.hardware_verified = False
        self.report("FAILED", detail)
        if self.cancel.service_is_ready() and self.status.get("state") not in (
            "IDLE",
            "DONE",
            "FAULT",
            None,
        ):
            self.cancel.call_async(Trigger.Request())

    def tick(self):
        try:
            self.publish_visual()
            self.fake_force()
            if self.stage in ("DONE", "FAILED"):
                return
            now = monotonic()
            if self.stage != "WAIT_READY" and not self.external_control_ready():
                raise RuntimeError("机械臂外控程序未运行，停止检测任务")
            if self.stage == "RUNNING":
                elapsed = max(0.0, now - self.last_task_tick)
                self.last_task_tick = now
                if self.status.get("state") != "COMPLIANT_HOLD":
                    self.task_elapsed += elapsed
                if now - self.status_at > 2.0:
                    raise RuntimeError("检测状态断流")
                if self.status.get("state") == "DONE":
                    self.report(
                        "DONE",
                        "DETECTION TEST PASS：检测点接触、保持 5 秒和回原位已完成",
                    )
                elif self.status.get("state") == "FAULT":
                    raise RuntimeError(self.status.get("message", "检测失败"))
                elif self.task_elapsed > self.task_timeout:
                    raise RuntimeError("检测测试执行超时")
                return
            if now - self.started > self.startup_timeout:
                raise RuntimeError("检测测试启动超时")
            if self.operation is not None:
                future, callback, since = self.operation
                if future.done():
                    self.operation = None
                    result = future.result()
                    if not result.success:
                        raise RuntimeError(result.message)
                    callback()
                elif now - since > 5.0:
                    raise RuntimeError("检测服务响应超时")
                return
            if self.stage == "WAIT_INITIAL":
                if self.status.get("state") == "FAULT":
                    raise RuntimeError(self.status.get("message", "移动到初始位置失败"))
                if (
                    self.status.get("state") == "IDLE"
                    and self.status.get("initial_position_ready")
                    and self.status_at > self.prepare_requested_at
                ):
                    self.publish_target()
                return
            if self.stage == "WAIT_POINT":
                if (
                    self.status_at > self.published_at
                    and self.status.get("queued_points") == 1
                ):
                    self.report("STARTING", "检测点已保存，自动开始到达测试")
                    self.request(self.start, self.running)
                elif now - self.published_at > 5.0:
                    raise RuntimeError("检测控制器未确认虚拟点入队")
                return
            if now - self.last_check > 1.0:
                if self.hardware.service_is_ready() and (
                    self.hardware_future is None or self.hardware_future.done()
                ):
                    self.hardware_future = self.hardware.call_async(
                        ListHardwareComponents.Request()
                    )
                    self.hardware_future.add_done_callback(self.check_hardware)
                if self.controllers.service_is_ready() and (
                    self.controllers_future is None or self.controllers_future.done()
                ):
                    self.controllers_future = self.controllers.call_async(
                        ListControllers.Request()
                    )
                    self.controllers_future.add_done_callback(self.check_controllers)
                self.last_check = now
            ready = (
                self.hardware_verified
                and self.external_control_ready()
                and self.controllers_active
                and now - self.checked_at < 2.5
                and self.record.service_is_ready()
                and self.start.service_is_ready()
                and (
                    self.simulation
                    or not self.fixed_real_target
                    or self.prepare.service_is_ready()
                )
                and self.publisher.get_subscription_count() > 0
                and now - self.status_at < 2.0
                and self.status.get("state") == "IDLE"
                and self.status.get("motion_ready", False)
            )
            if ready:
                self.pose()
                if self.ready_since is None:
                    self.ready_since = now
                elif now - self.ready_since >= self.settle:
                    if not self.simulation and self.fixed_real_target:
                        self.contact_rotation = self.pose()[:3, :3].copy()
                        self.report(
                            "PREPARE_INITIAL", "规划到配置的真机初始位置与选定朝向"
                        )
                        self.prepare_requested_at = monotonic()
                        self.request(
                            self.prepare,
                            lambda: self.report(
                                "WAIT_INITIAL", "等待初始位置到位并记录原位"
                            ),
                        )
                    else:
                        self.report("RECORD_HOME", "记录本次测试原位")
                        self.request(self.record, self.publish_target)
            else:
                self.ready_since = None
                if now - self.last_ready_log > 5.0:
                    self.last_ready_log = now
                    if not self.external_control_ready():
                        reason = "等待机械臂外控程序运行（robot_task_running=true）"
                    else:
                        reason = self.status.get(
                            "readiness_message",
                            "等待检测控制器发布数据和运动接口就绪状态",
                        )
                    self.get_logger().info(f"{reason}；当前不发送运动请求")
        except Exception as error:
            # 模型 TF 在初始化时可能尚未到达，等待启动期限，而不是抢先生成目标。
            if (
                self.stage == "WAIT_READY"
                and monotonic() - self.started < self.startup_timeout
            ):
                self.ready_since = None
                return
            self.fail(str(error))

    def running(self):
        self.task_started = monotonic()
        self.last_task_tick = self.task_started
        self.task_elapsed = 0.0
        self.report("RUNNING", "检测任务执行中，等待接触保持和返回结果")


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = VirtualDetectionTask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.stage not in ("DONE", "FAILED"):
            node.fail("测试节点退出")
            end = monotonic() + 0.5
            while rclpy.ok() and monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
