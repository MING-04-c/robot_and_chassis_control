#!/usr/bin/env python3
"""测厚探头：规划接近、Z 向导纳接触、保持、垂直撤离和返回。"""
from copy import deepcopy
from pathlib import Path
import os
import tempfile
import yaml
import json
import math
import signal
from time import monotonic
from uuid import uuid4
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from eli_cs_robot_moveit_config.joint_postures import load_postures, joint_constraints
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool, Empty
from std_srvs.srv import Trigger, SetBool
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import WrenchStamped
from geminipro_camer_calibration.live_tf import LiveRobotTransforms
from geminipro_camer_calibration.geometry import (
    matrix,
    components,
    rpy_degrees_rotation,
    pose_matrix,
    transform_matrix,
    to_pose,
    tool_goal,
)

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class RetryableApproachError(RuntimeError):
    """仅表示预测点不可达或受限接近未接触；安全故障不使用此异常。"""
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class DetectController(Node):
    def __init__(self):
        super().__init__("detect_controller")
        defaults = {
            "base_frame": "base_link",
            "tool_frame": "tool0",
            "probe_frame": "terminal_link_2",
            "planning_group": "cs_manipulator",
            "lock_tool_orientation": False,
            "target_topic": "/geminipro_calibration/target_base",
            "targets_topic": "/geminipro_calibration/targets_base",
            "targets_file": "~/.ros/detect_control/targets.yaml",
            "measurement_results_file": "~/.ros/detect_control/thickness_results.jsonl",
            "max_targets": 100,
            "home_joint_tolerance": 0.01,
            # admittance: 现有连续导纳接近；moveit_force_guard: 普通 MoveIt 单条直线接近，
            # 仅用力传感器做 20N 停止保护。
            "touch_control_mode": "moveit_force_guard",
            "admittance_namespace": "/admittance_controller",
            "raw_wrench_topic": "/sensor/wrench",
            "force_filter_alpha": 0.25,
            "force_bias_sample_count": 50,
            "target_source": "camera",
            "initial_position_enabled": False,
            "initial_position": [-0.4, 0.1, 0.4],
            "initial_orientation_mode": "current",
            "initial_rpy_deg": [0.0, 0.0, 0.0],
            "fixed_position": [0.4, 0.0, 0.3],
            "fixed_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "contact_force": 20.0,
            "overload_force": 40.0,
            "initial_force_limit": 5.0,
            "hold_seconds": 5.0,
            "approach_axis": "base_negative_z",
            "continue_until_contact": False,
            "joint_postures_enabled": False,
            "recover_approach_failure": False,
            "require_thickness_measurement": False,
            "recovery_settle_seconds": 0.5,
            "point_results_file": "~/.ros/detect_control/point_results.jsonl",
            "approach_clearance": 0.02,
            "max_penetration": 0.005,
            "approach_speed": 0.002,
            "moveit_cartesian_step": 0.005,
            "max_tracking_error": 0.005,
            "lateral_tolerance": 0.002,
            "orientation_tolerance": 0.03,
            "pose_tolerance": 0.003,
            "data_timeout": 0.3,
            "target_max_age": 30.0,
            "phase_timeout": 60.0,
            "service_timeout": 5.0,
            "rate": 20.0,
            "velocity_scaling": 0.1,
            "acceleration_scaling": 0.1,
            "workspace_min": [-1.0, -1.0, 0.0],
            "workspace_max": [1.0, 1.0, 1.5],
        }
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if self.cfg["touch_control_mode"] not in ("admittance", "moveit_force_guard"):
            raise ValueError("touch_control_mode 必须为 admittance 或 moveit_force_guard")
        if not 0.0 < float(self.cfg["force_filter_alpha"]) <= 1.0:
            raise ValueError("force_filter_alpha 必须在 (0, 1] 内")
        if int(self.cfg["force_bias_sample_count"]) < 1:
            raise ValueError("force_bias_sample_count 必须为正整数")
        self.touch_control_mode = self.cfg["touch_control_mode"]
        self.postures = load_postures(self.declare_parameter('joint_postures_file', str(
            Path(get_package_share_directory('eli_cs_robot_moveit_config'))/'config/arm_joint_postures.yaml')).value)
        self.ground_ready = False
        self.ground_subscription = self.create_subscription(Bool, '/robot_control/ground_ready',
            lambda message: setattr(self, 'ground_ready', message.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # 未提供度制参数时兼容旧 YAML 的 orientation_tolerance（弧度）。
        degrees = self.declare_parameter(
            "orientation_tolerance_deg", math.degrees(self.cfg["orientation_tolerance"])
        ).value
        if not math.isfinite(degrees) or not 0 < degrees <= 180:
            raise ValueError("orientation_tolerance_deg 必须在 (0, 180] 度范围内")
        self.cfg["orientation_tolerance"] = math.radians(degrees)
        if self.cfg["initial_orientation_mode"] not in ("current", "fixed"):
            raise ValueError("initial_orientation_mode 必须为 current 或 fixed")
        if self.cfg['approach_axis'] not in ('base_negative_z', 'tool_positive_z'):
            raise ValueError('approach_axis 必须为 base_negative_z 或 tool_positive_z')
        self.axis = None
        self.fixed_initial_rotation = rpy_degrees_rotation(self.cfg["initial_rpy_deg"])
        for name, value in self.cfg.items():
            if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} 必须是有限正数")
        if (
            not self.cfg["initial_force_limit"]
            < self.cfg["contact_force"]
            < self.cfg["overload_force"]
        ):
            raise ValueError(
                "要求 initial_force_limit < contact_force < overload_force"
            )
        if self.cfg["target_source"] not in ("camera", "fixed"):
            raise ValueError("target_source 必须是 camera 或 fixed")
        for key in ("velocity_scaling", "acceleration_scaling"):
            if self.cfg[key] > 1:
                raise ValueError(f"{key} 必须在 (0,1] 内")
        low, high = np.array(self.cfg["workspace_min"]), np.array(
            self.cfg["workspace_max"]
        )
        if (
            low.shape != (3,)
            or high.shape != (3,)
            or not np.isfinite([low, high]).all()
            or not np.all(low < high)
        ):
            raise ValueError("工作空间必须是三个有限且 min < max 的边界")
        self.live_tf = LiveRobotTransforms()
        self.buffer = self.live_tf.buffer
        self.phase = "IDLE"
        self.detail = "等待目标和 start 服务"
        self.since = monotonic()
        self.operation = None
        self.goal_handle = None
        self.outstanding_actions = set()
        self.targets = []
        self.target_rejection = None
        self.batch = []
        self.point_index = 0
        self.completed_points = 0
        self.recorded_home = None
        self.capture_joints = None
        self.initial_position_ready = False
        self.initial_orientation_xyzw = None
        self.initial_pose = None
        self.targets_file = Path(self.cfg["targets_file"]).expanduser()
        if not 1 <= self.cfg["max_targets"] <= 1000:
            raise ValueError("max_targets 必须在 1～1000 之间")
        self.joints = None
        self.joints_at = -math.inf
        self.force = None
        self.force_filter = None
        self.force_bias = None
        self.force_bias_samples = []
        self.bias_joint_reference = None
        self.bias_stationary_since = None
        self.force_at = -math.inf
        self.raw_at = -math.inf
        self.stop_confirmed = True
        self.generation = 0
        self.last_status = 0.0
        self.stop_future = None
        self.contact_stop_requested = False
        self.moveit_stop_since = None
        self.moveit_stop_reference = None
        self.moveit_stop_joint_stamp = None
        namespace = self.cfg["admittance_namespace"].rstrip("/")
        self.target_pub = self.create_publisher(
            PoseStamped, namespace + "/tracking_target_pose", 10
        )
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.window_pub = self.create_publisher(Bool, "~/measurement_active", 10)
        self.contact_pub = self.create_publisher(PoseStamped, "~/contact_pose", 10)
        self.measure_pub = self.create_publisher(Empty, "/detactor/acquire_trigger", 10)
        self.thickness_result = None
        self.retry_context = None
        self.batch_id = ""
        self.recovery_reference = None
        self.recovery_quiet_since = None
        self.point_result_pub = self.create_publisher(String, "~/point_result",
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.measurement_started_ns = None
        self.thickness_pub = self.create_publisher(String, "~/thickness_result", 10)
        self.create_subscription(String, "/detactor/measurement", self.on_thickness, 10)
        self.enable = self.create_client(SetBool, namespace + "/enable")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.create_subscription(
            PoseStamped, self.cfg["target_topic"], self.on_target, 10
        )
        self.create_subscription(
            PoseArray, self.cfg["targets_topic"], self.on_targets, 10
        )
        self.create_subscription(
            JointState, "/joint_states", self.on_joints, qos_profile_sensor_data
        )
        self.create_subscription(
            WrenchStamped,
            namespace + "/measured_wrench",
            self.on_force,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            WrenchStamped,
            self.cfg["raw_wrench_topic"],
            self.on_raw,
            qos_profile_sensor_data,
        )
        self.safety_inhibited = False
        self.create_subscription(Bool, "/robot_control/safety_inhibit", self.on_safety_inhibit,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, "~/start", self.start)
        self.create_service(Trigger, "~/cancel", self.cancel)
        self.create_service(Trigger, "~/record_home", self.record_home)
        self.create_service(Trigger, "~/prepare_detection", lambda req, response: self.prepare_posture('detection_ready', response))
        self.create_service(Trigger, "~/prepare_travel", lambda req, response: self.prepare_posture('travel', response))
        self.create_service(Trigger, "~/prepare_initial", self.prepare_initial)
        self.create_service(Trigger, "~/clear_targets", self.clear_targets)
        self.create_service(Trigger, "~/load_targets", self.load_targets)
        self.tick_timer = self.create_timer(1 / self.cfg["rate"], self.tick)
        self.get_logger().info(
            "检测接口就绪；不会因收到目标而自动运动。接触阈值 20N 默认可配置。"
        )

    def destroy_node(self):
        self.live_tf.close()
        return super().destroy_node()

    @property
    def active(self):
        return self.phase not in ("IDLE", "DONE", "FAULT", "RETRY_REQUIRED", "READY_DETECTION", "READY_TRAVEL")

    def state(self, phase, detail=""):
        if self.phase == "HOLD" and phase != "HOLD":
            result = {"schema": "paint_thickness_task_result_v1",
                      "available": self.thickness_result is not None,
                      "point_index": self.point_index + 1, "measurement": self.thickness_result,
                      "next_phase": phase}
            if self.thickness_result is None:
                result["reason"] = "接触保持期间未收到有效测厚回包"
                self.get_logger().warning("本检测点未获得厚度读数；不会复用上一点读数")
            self.thickness_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
            try:
                path = Path(self.cfg["measurement_results_file"]).expanduser()
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(result, ensure_ascii=False, allow_nan=False) + "\n")
            except OSError as error:
                self.get_logger().error(f"厚度结果保存失败：{error}")
        self.phase, self.detail, self.since = phase, detail, monotonic()
        self.get_logger().info(f"{phase}: {detail}")
        self.publish_status()
        self.window_pub.publish(Bool(data=phase == "HOLD"))

    def on_thickness(self, message):
        """只将本次接触触发后收到的有效单值关联到当前检测点。"""
        if self.phase != "HOLD" or self.measurement_started_ns is None:
            return
        try:
            reading = json.loads(message.data)
            value = reading["value"]
            stamp = reading["received_at_ros"]
            received_ns = int(stamp["sec"]) * 10**9 + int(stamp["nanosec"])
            if (reading.get("schema") != "paint_thickness_measurement_v1" or
                    isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value) or received_ns < self.measurement_started_ns or
                    reading.get("unit") not in ("unknown", "um", "mm")):
                return
        except (ValueError, KeyError, TypeError):
            return
        self.thickness_result = reading
        result = {"schema": "paint_thickness_task_result_v1", "available": True,
                  "point_index": self.point_index + 1, "measurement": reading}
        self.thickness_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
        self.get_logger().info(f'检测点 {self.point_index + 1} 厚度：{value} {reading["unit"]}')

    def on_safety_inhibit(self, message):
        self.safety_inhibited = message.data
        if message.data and self.active:
            self.fail("机器人安全暂停；检测接触阶段取消，需要重新准备检测任务")

    def motion_readiness(self):
        """返回初始运动能否开始及等待原因；只读取状态，不触发运动。"""
        if self.cfg.get('joint_postures_enabled', False) and (
                not self.ground_ready or self.ground_subscription.get_publisher_count() == 0):
            return False, "等待8cm地面碰撞场景确认"
        if self.safety_inhibited:
            return False, "安全恢复锁定中"
        if self.active or self.outstanding_actions:
            return False, "等待当前运动结束"
        if not self.stop_confirmed or self.stop_future is not None:
            return False, "等待上次停止确认"
        if not self.fresh():
            return False, "等待新鲜关节、原始力和零偏初始化后的滤波力数据"
        if self.touch_control_mode == "moveit_force_guard" and self.force_bias is None:
            return False, "等待普通 MoveIt 力保护完成传感器零偏采样"
        if np.linalg.norm(self.force) >= self.cfg["initial_force_limit"]:
            return False, "初始受力超过允许值，请检查探头接触或力零偏"
        if (self.touch_control_mode == "admittance" and
                not self.enable.service_is_ready()):
            return False, "等待导纳服务"
        if (not self.move.server_is_ready() or
                not self.cartesian.service_is_ready() or
                not self.execute.server_is_ready()):
            return False, "等待 MoveIt 规划和执行接口"
        return True, "关节、力数据和运动接口已就绪"

    def publish_status(self):
        motion_ready, readiness_message = self.motion_readiness()
        message = String()
        message.data = json.dumps(
            {
                "state": self.phase,
                "batch_id": self.batch_id,
                "retry_context": self.retry_context,
                "returned_home": self.phase in ("DONE", "RETRY_REQUIRED"),
                "motion_ready": motion_ready,
                "readiness_message": readiness_message,
                "message": self.detail,
                "touch_control_mode": self.touch_control_mode,
                "force_bias_ready": self.force_bias is not None if self.touch_control_mode == "moveit_force_guard" else None,
                "force_z_N": None if self.force is None else float(self.force[2]),
                "force_axial_N": None if self.force is None else self.axial_force(self.force),
                "approach_axis": self.cfg['approach_axis'],
                "approach_axis_base": None if self.axis is None else self.axis.tolist(),
                "continue_until_contact": self.cfg['continue_until_contact'],
                "max_extra_travel_m": self.cfg['max_penetration'],
                "contact_force_N": self.cfg["contact_force"],
                "queued_points": len(self.targets),
                "target_rejection": self.target_rejection,
                "batch_points": len(self.batch),
                "current_point": self.point_index + 1 if self.batch else 0,
                "completed_points": self.completed_points,
                "thickness_measurement": self.thickness_result,
                "home_recorded": self.recorded_home is not None,
                "initial_position_ready": self.initial_position_ready,
                "initial_tool_orientation_xyzw": self.initial_orientation_xyzw,
            },
            ensure_ascii=False,
        )
        self.status_pub.publish(message)
        self.last_status = monotonic()

    def stamp_fresh(self, stamp, timeout):
        if stamp.sec == 0 and stamp.nanosec == 0:
            return False
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        return -0.1 <= age <= timeout

    def check_target(self, message, check_age=True):
        if message.header.frame_id != self.cfg["base_frame"]:
            raise ValueError("检测点必须已经转换到 base_frame")
        if check_age and not self.stamp_fresh(
            message.header.stamp, self.cfg["target_max_age"]
        ):
            raise ValueError("新检测点观测时间缺失或已过期")
        transform = pose_matrix(message.pose)
        if not self.in_workspace(transform):
            raise ValueError("TARGET_UNREACHABLE: 检测点超过工作空间")
        return transform

    def store_targets(self, targets):
        """原子保存基座检测坐标；不保存相机相对坐标供以后重新解释。"""
        records = []
        for message in targets:
            p, q = message.pose.position, message.pose.orientation
            records.append(
                {
                    "position": [p.x, p.y, p.z],
                    "quaternion_xyzw": [q.x, q.y, q.z, q.w],
                    "observed_at": {
                        "sec": message.header.stamp.sec,
                        "nanosec": message.header.stamp.nanosec,
                    },
                }
            )
        data = {
            "schema": "detect_targets_v1",
            "units": "m",
            "frame_id": self.cfg["base_frame"],
            "targets": records,
        }
        self.targets_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", dir=self.targets_file.parent, prefix=".targets-", delete=False
            ) as stream:
                temporary = Path(stream.name)
                yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.targets_file)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def on_target(self, message):
        try:
            if self.active:
                raise ValueError("任务执行中不修改已锁存的检测点队列")
            self.check_target(message)
            if any(
                np.allclose(
                    pose_matrix(x.pose), pose_matrix(message.pose), atol=1e-7, rtol=0
                )
                for x in self.targets
            ):
                return
            if len(self.targets) >= self.cfg["max_targets"]:
                raise ValueError("检测点数量达到上限")
            targets = [*self.targets, deepcopy(message)]
            self.store_targets(targets)
            self.targets = targets
            self.publish_status()
        except Exception as error:
            self.get_logger().warning(f"检测点未存入：{error}")

    def on_targets(self, message):
        try:
            if self.active:
                raise ValueError("任务执行中不修改已锁存的检测点队列")
            if not 1 <= len(message.poses) <= self.cfg["max_targets"]:
                raise ValueError("检测点数组为空或超过数量上限")
            targets = []
            for pose in message.poses:
                item = PoseStamped(header=deepcopy(message.header), pose=deepcopy(pose))
                self.check_target(item)
                targets.append(item)
            self.store_targets(targets)
            self.targets = targets
            self.target_rejection = None
            self.publish_status()
        except Exception as error:
            self.target_rejection = {"message": str(error), "stamp": {
                "sec": message.header.stamp.sec, "nanosec": message.header.stamp.nanosec}}
            self.publish_status()
            self.get_logger().warning(f"整组检测点未存入：{error}")

    def record_home(self, request, response):
        del request
        try:
            if self.active or self.outstanding_actions:
                raise ValueError("运动中不能覆盖原位")
            if (
                self.joints is None
                or monotonic() - self.joints_at > self.cfg["data_timeout"]
            ):
                raise ValueError("关节数据未就绪")
            self.current_tool()
            self.capture_joints = deepcopy(self.joints)
            self.recorded_home = deepcopy(self.joints)
            if self.cfg.get('joint_postures_enabled', False):
                for name, value in zip(JOINTS, self.postures['travel']):
                    self.recorded_home.position[self.recorded_home.name.index(name)] = value
            response.success, response.message = True, (
                "已保存拍照关节；返回位使用配置的行驶姿态" if self.cfg.get('joint_postures_enabled', False)
                else "已记录当前六轴关节原位")
            self.publish_status()
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def prepare_initial(self, request, response):
        if self.safety_inhibited:
            response.success, response.message = False, "安全恢复锁定中"
            return response
        """规划 tool0 到初始 XYZ 与选定朝向，到位后记录实际关节原位。"""
        del request
        accepted = False
        try:
            if not self.cfg["initial_position_enabled"]:
                raise ValueError("当前配置未启用固定初始位置")
            if self.active or self.outstanding_actions or not self.stop_confirmed:
                raise ValueError("任务未结束或上次停止尚未确认")
            if self.stop_future is not None:
                raise ValueError("仍在等待停止响应")
            if (
                not self.fresh()
                or np.linalg.norm(self.force) >= self.cfg["initial_force_limit"]
            ):
                raise ValueError("初始移动前需要新鲜关节/力数据，且探头未接触")
            if (self.touch_control_mode == "admittance" and
                    not self.enable.service_is_ready()) or not self.move.server_is_ready():
                raise ValueError("导纳或 MoveIt 尚未就绪")
            goal = self.current_tool()
            position = np.asarray(self.cfg["initial_position"], dtype=float)
            if position.shape != (3,) or not np.isfinite(position).all():
                raise ValueError("initial_position 必须为三个有限米制坐标")
            goal[:3, 3] = position
            if self.cfg["initial_orientation_mode"] == "fixed":
                goal[:3, :3] = self.fixed_initial_rotation
            mounting = transform_matrix(
                self.buffer.lookup_transform(
                    self.cfg["tool_frame"], self.cfg["probe_frame"], Time()
                ).transform
            )
            if not self.in_workspace(goal) or not self.in_workspace(goal @ mounting):
                raise ValueError("初始 tool0 或探头位置超过工作空间")
            self.mounting = mounting
            self.initial_pose = self.current_tool()
            if self.cfg["lock_tool_orientation"] and not self.rotation_ok(goal, self.initial_pose):
                raise ValueError("固定初始位置要求旋转，与保持当前朝向模式冲突")
            self.initial_cartesian_goal = goal
            self.initial_orientation_xyzw = components(goal)[1]
            self.initial_position_ready = False
            self.recorded_home = None
            self.batch = []
            self.completed_points = 0
            self.generation += 1
            self.operation = None
            self.state("PREPARE_INITIAL", "关闭导纳，准备规划到固定初始 tool0 位置")
            accepted = True
            self.set_enabled(False,
                lambda: self.plan_pose(
                    goal, on_reached=self.verify_initial, phase="MOVE_INITIAL"
                ),
            )
            response.success, response.message = (
                True,
                "初始位置任务已接收，等待实际到位确认",
            )
        except Exception as error:
            if accepted:
                self.fail(str(error))
            response.success, response.message = False, str(error)
        return response

    def verify_initial(self):
        self.initial_motion_finished = self.get_clock().now()
        self.state("VERIFY_INITIAL", "等待新 TF 确认固定初始位置并记录原位")

    def initial_arrived(self):
        current = self.current_tool(after=self.initial_motion_finished)
        if np.linalg.norm(
            current[:3, 3] - self.initial_cartesian_goal[:3, 3]
        ) > self.cfg["pose_tolerance"] or not self.rotation_ok(
            current, self.initial_cartesian_goal
        ):
            raise RuntimeError("实际 tool0 尚未达到固定初始位置或朝向")
        if np.linalg.norm(self.force) >= self.cfg["initial_force_limit"]:
            raise RuntimeError("初始点仍有外力，不能记录未接触原位")
        self.recorded_home = deepcopy(self.joints)
        self.initial_position_ready = True
        self.state("IDLE", "固定初始 tool0 位置已到达，关节原位已记录")

    def clear_targets(self, request, response):
        del request
        try:
            if self.active:
                raise ValueError("任务执行中不能清空检测点")
            self.store_targets([])
            self.targets = []
            response.success, response.message = True, "已清空内存和 YAML 检测点"
            self.publish_status()
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def load_targets(self, request, response):
        del request
        try:
            if self.active:
                raise ValueError("任务执行中不能加载检测点")
            data = yaml.safe_load(self.targets_file.read_text())
            if (
                data.get("schema") != "detect_targets_v1"
                or data.get("units") != "m"
                or data.get("frame_id") != self.cfg["base_frame"]
            ):
                raise ValueError("检测点 YAML 的 schema、单位或坐标系不正确")
            if not 1 <= len(data["targets"]) <= self.cfg["max_targets"]:
                raise ValueError("检测点 YAML 为空或超过上限")
            targets = []
            for record in data["targets"]:
                item = PoseStamped()
                item.header.frame_id = self.cfg["base_frame"]
                item.pose = to_pose(
                    matrix(record["position"], record["quaternion_xyzw"])
                )
                self.check_target(item, check_age=False)
                targets.append(item)
            self.targets = targets
            response.success, response.message = (
                True,
                f"已加载 {len(targets)} 个基座检测点",
            )
            self.publish_status()
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def on_joints(self, message):
        if (
            len(message.name) == len(message.position)
            and all(j in message.name for j in JOINTS)
            and all(math.isfinite(x) for x in message.position)
            and self.stamp_fresh(message.header.stamp, self.cfg["data_timeout"])
        ):
            self.joints = deepcopy(message)
            self.joints_at = monotonic()
            positions = np.array([
                message.position[message.name.index(name)] for name in JOINTS
            ])
            moving = (len(message.velocity) == len(message.name) and any(
                abs(message.velocity[message.name.index(name)]) > 0.01 for name in JOINTS))
            changed = (self.bias_joint_reference is None or
                       np.max(np.abs(positions - self.bias_joint_reference)) > 0.0005)
            if moving or changed:
                self.bias_joint_reference = positions
                self.bias_stationary_since = monotonic()

    def on_raw(self, message):
        values = [
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
        ]
        if all(math.isfinite(x) for x in values) and self.stamp_fresh(
            message.header.stamp, self.cfg["data_timeout"]
        ):
            self.raw_at = monotonic()
            if self.touch_control_mode == "moveit_force_guard":
                try:
                    transformed = self.raw_force_in_base(message)
                    if self.force_bias is None:
                        stationary = (self.bias_stationary_since is not None and
                                      monotonic() - self.bias_stationary_since >= 0.5)
                        if (not self.active and stationary and
                                np.linalg.norm(transformed) < self.cfg["initial_force_limit"]):
                            self.force_bias_samples.append(transformed)
                            if len(self.force_bias_samples) >= int(self.cfg["force_bias_sample_count"]):
                                self.force_bias = np.mean(self.force_bias_samples, axis=0)
                                self.force_filter = None
                                self.get_logger().info(
                                    f"普通 MoveIt 力保护零偏已建立：{self.force_bias.tolist()} N")
                        elif not self.active:
                            self.force_bias_samples.clear()
                    transformed = (transformed if self.force_bias is None
                                   else transformed - self.force_bias)
                    # 合力过载用去偏后的未滤波数据，不能被低通滤波压低而漏掉瞬时过载。
                    if self.active and np.linalg.norm(transformed) >= self.cfg["overload_force"]:
                        self.fail("原始去偏合力超过保护阈值")
                        return
                    alpha = float(self.cfg["force_filter_alpha"])
                    self.force_filter = (
                        transformed if self.force_filter is None
                        else alpha * transformed + (1.0 - alpha) * self.force_filter
                    )
                    self.force = self.force_filter.copy()
                    self.force_at = monotonic()
                    self.handle_force_sample(self.force)
                except Exception as error:
                    if self.active:
                        self.fail(f"原始力数据无法转换到基座坐标：{error}")

    def raw_force_in_base(self, message):
        """将传感器坐标系的力旋转到 base_frame；力的平移不影响三轴力。"""
        values = np.array([
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
        ], dtype=float)
        frame = message.header.frame_id
        if not frame:
            raise ValueError("原始 Wrench 的 frame_id 为空")
        if frame == self.cfg["base_frame"]:
            return values
        transform = transform_matrix(
            self.buffer.lookup_transform(self.cfg["base_frame"], frame, Time()).transform
        )
        return transform[:3, :3] @ values

    def on_force(self, message):
        values = np.array(
            [message.wrench.force.x, message.wrench.force.y, message.wrench.force.z]
        )
        if (
            message.header.frame_id != self.cfg["base_frame"]
            or not np.isfinite(values).all()
            or not self.stamp_fresh(message.header.stamp, self.cfg["data_timeout"])
        ):
            if self.active:
                self.fail("力数据无效、过期或坐标系不匹配")
            return
        mode = self.cfg.get("touch_control_mode", "admittance")
        if mode != "admittance":
            return
        self.force, self.force_at = values, monotonic()
        self.handle_force_sample(values)

    def handle_force_sample(self, values):
        if not self.active:
            return
        if np.linalg.norm(values) >= self.cfg["overload_force"]:
            self.fail("合力超过保护阈值")
            return
        if self.axial_force(values) < self.cfg["contact_force"]:
            return
        mode = self.cfg.get("touch_control_mode", "admittance")
        if mode == "admittance" and self.phase in ("DESCEND", "COMPLIANT_HOLD"):
            # 在收到力回调时立即停止，不等待下一次推进定时器。
            self.operation = None
            self.state("STOP_CONTACT", "已达到接近轴方向的接触力阈值，停止导纳输出")
            self.set_enabled(False, self.begin_hold)
        elif mode == "moveit_force_guard" and self.phase in (
                "MOVEIT_PLAN_DESCENT", "MOVEIT_DESCEND", "VERIFY_MOVEIT_SEGMENT",
                "MOVEIT_WAIT_CONTACT"):
            self.request_moveit_force_stop()

    def fresh(self):
        now = monotonic()
        limit = self.cfg["data_timeout"]
        return (
            self.joints is not None
            and self.force is not None
            and all(now - x <= limit for x in (self.joints_at, self.force_at, self.raw_at))
        )

    def current_tool(self, after=None):
        tf = self.buffer.lookup_transform(
            self.cfg["base_frame"], self.cfg["tool_frame"], Time()
        )
        if not self.stamp_fresh(tf.header.stamp, self.cfg["data_timeout"]):
            age = (
                self.get_clock().now() - Time.from_msg(tf.header.stamp)
            ).nanoseconds / 1e9
            raise ValueError(
                f"机器人 TF 已过期：{age:.3f} s，限制 {self.cfg['data_timeout']:.3f} s"
            )
        if after is not None and Time.from_msg(tf.header.stamp) < after:
            raise ValueError("等待规划执行结束之后的机器人 TF")
        return transform_matrix(tf.transform)

    def direction_for(self, transform):
        """输入 tool0 基座位姿，输出单位接近向量（在基座坐标系中表达）。"""
        if self.cfg['approach_axis'] == 'base_negative_z':
            return np.array([0., 0., -1.])
        direction = np.asarray(transform[:3, 2], dtype=float)
        length = np.linalg.norm(direction)
        if not np.isfinite(direction).all() or length < 1e-9:
            raise ValueError('无法从 tool0 姿态得到有效局部+Z方向')
        return direction / length

    def axial_force(self, force):
        """返回接近轴上的力绝对值，单位N；不把基座Fz误当作倾斜探头轴向力。"""
        axis = self.axis if self.axis is not None else np.array([0., 0., -1.])
        return abs(float(np.dot(force, axis)))

    def in_workspace(self, transform):
        p = transform[:3, 3]
        return bool(
            np.all(p >= self.cfg["workspace_min"])
            and np.all(p <= self.cfg["workspace_max"])
        )

    def start(self, request, response):
        if self.safety_inhibited:
            response.success, response.message = False, "安全恢复锁定中"
            return response
        del request
        try:
            if (
                self.active
                or self.stop_future is not None
                and not self.stop_future.done()
            ):
                raise ValueError("任务或停止请求尚未结束")
            if self.outstanding_actions:
                raise ValueError(
                    "上一次 MoveIt 运动还没有返回结束结果，暂不能开始新任务"
                )
            if not self.stop_confirmed:
                raise ValueError("上次停止尚未确认，请检查控制器后重新启动任务节点")
            if self.cfg.get('joint_postures_enabled', False) and not self.motion_readiness()[0]:
                raise ValueError(self.motion_readiness()[1])
            if not self.fresh():
                raise ValueError(
                    "等待新鲜关节、原始力和去零滤波力；先启动对应控制模式的驱动与传感器"
                )
            if self.touch_control_mode == "moveit_force_guard" and self.force_bias is None:
                raise ValueError("普通 MoveIt 力保护尚未完成传感器零偏采样")
            if np.linalg.norm(self.force) >= self.cfg["initial_force_limit"]:
                raise ValueError("启动前外力过大；请在探头未接触时正确清零")
            planning_services = [self.cartesian]
            if self.touch_control_mode == "admittance":
                planning_services.append(self.ik)
            if (
                (self.touch_control_mode == "admittance" and
                 not self.enable.service_is_ready())
                or not all(c.service_is_ready() for c in planning_services)
                or not self.move.server_is_ready()
                or not self.execute.server_is_ready()
            ):
                raise ValueError("MoveIt/导纳服务尚未就绪")
            if (self.touch_control_mode == "admittance" and
                    self.target_pub.get_subscription_count() == 0):
                raise ValueError(
                    "导纳缺少 tracking_target_pose 接口；请先编译并重启更新后的控制节点"
                )
            if self.cfg["target_source"] == "fixed":
                item = PoseStamped()
                item.header.frame_id = self.cfg["base_frame"]
                item.pose = to_pose(
                    matrix(
                        self.cfg["fixed_position"], self.cfg["fixed_quaternion_xyzw"]
                    )
                )
                targets = [item]
            else:
                if not self.targets:
                    raise ValueError("请先存入检测点，或通过 load_targets 加载 YAML")
                targets = deepcopy(self.targets)
            mounting = transform_matrix(
                self.buffer.lookup_transform(
                    self.cfg["tool_frame"], self.cfg["probe_frame"], Time()
                ).transform
            )
            batch = []
            for item in targets:
                target = self.check_target(item, check_age=False)
                final = tool_goal(target, mounting)
                direction = self.direction_for(final)
                pre = final.copy()
                pre[:3, 3] -= direction*self.cfg["approach_clearance"]
                far = final.copy()
                far[:3, 3] += direction*self.cfg["max_penetration"]
                if not all(self.in_workspace(t) for t in (target, final, pre, far, pre @ mounting, far @ mounting)):
                    raise ValueError(
                        "TARGET_UNREACHABLE: 某检测点、预接近点或最大探入点超过工作空间；整组不执行"
                    )
                batch.append((pre, final))
            self.mounting = mounting
            self.initial_joints = deepcopy(
                self.recorded_home if self.recorded_home is not None else self.joints
            )
            self.initial_pose = self.current_tool()
            if self.cfg["lock_tool_orientation"] and any(
                    not self.rotation_ok(final, self.initial_pose) for _, final in batch):
                raise ValueError("目标朝向与启动时 tool0 朝向不一致；保持朝向模式不执行旋转")
            self.batch_id = uuid4().hex
            self.retry_context = None
            self.axis = None
            self.thickness_result = None
            self.contact_stop_requested = False
            self.force_filter = None
            self.batch = batch
            self.point_index = 0
            self.completed_points = 0
            self.pre, self.final = self.batch[0]
            self.generation += 1
            self.operation = None
            self.state(
                "PREPARE",
                "关闭导纳输出后进行预接近规划"
                if self.touch_control_mode == "admittance"
                else "使用 MoveIt 碰撞检查规划到预接近点",
            )
            self.set_enabled(False, self.begin_batch)
            response.success, response.message = (
                True,
                "任务已接收；通过 /detect_controller/status 查看执行结果",
            )
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def home_reached(self):
        return all(
            abs(
                self.joints.position[self.joints.name.index(j)]
                - self.initial_joints.position[self.initial_joints.name.index(j)]
            )
            <= self.cfg["home_joint_tolerance"]
            for j in JOINTS
        )

    def begin_batch(self):
        if self.cfg.get('joint_postures_enabled', False):
            self.plan_joints(self.postures['detection_ready'], lambda: self.plan_pose(self.pre), 'DETECTION_GATEWAY')
        elif self.home_reached():
            self.plan_pose(self.pre)
        else:
            # 显式记录原位后机器人可能已被移动；先回同一原位再开始第一点。
            self.return_home(initial=True)

    def point_returned(self, initial=False):
        if not self.home_reached():
            raise RuntimeError("MoveIt 返回成功但实际关节尚未回到记录原位")
        if initial:
            self.plan_pose(self.pre)
            return
        if self.retry_context is not None:
            self.state("RETRY_REQUIRED", "已停止、撤离并返回原位；等待新照片和新检测点")
            return
        self.publish_point_result()
        if self.cfg.get("require_thickness_measurement", False) and self.thickness_result is None:
            self.retry_context = {"code": "measurement_missing", "message": "本次接触未取得新测厚读数"}
            self.state("RETRY_REQUIRED", "已回原位但本次测厚无有效回包；等待重拍")
            return
        self.completed_points += 1
        if self.completed_points == len(self.batch):
            self.state("DONE", "全部检测点测完并返回同一原位；导纳输出保持关闭")
            return
        if np.linalg.norm(self.force) >= self.cfg["initial_force_limit"]:
            raise RuntimeError("回原位后仍有较大外力，停止后续检测点")
        self.point_index += 1
        self.pre, self.final = self.batch[self.point_index]
        if self.cfg.get('joint_postures_enabled', False):
            self.plan_joints(self.postures['detection_ready'], lambda: self.plan_pose(self.pre), 'DETECTION_GATEWAY')
        else:
            self.plan_pose(self.pre)

    def publish_point_result(self):
        """实际回原位后发布本点冻结的读数，不用回位途中其他查询的全局最新值覆盖。"""
        stamp = self.get_clock().now().to_msg()
        result = {
            "schema": "paint_detection_point_result_v1", "result_id": f"{self.batch_id}_{self.point_index + 1}",
            "batch_id": self.batch_id, "point_index": self.point_index + 1,
            "success": self.thickness_result is not None or not self.cfg.get("require_thickness_measurement", False),
            "returned_home": True, "measurement": deepcopy(self.thickness_result),
            "completed_at_ros": {"sec": stamp.sec, "nanosec": stamp.nanosec},
        }
        encoded = json.dumps(result, ensure_ascii=False, allow_nan=False)
        self.point_result_pub.publish(String(data=encoded))
        try:
            path = Path(self.cfg["point_results_file"]).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as stream:
                stream.write(encoded + "\n")
        except OSError as error:
            self.get_logger().error(f"回位测厚结果保存失败：{error}")

    def begin_recovery(self, error):
        """仅可恢复的接近失败：取消旧增量，先确认停止，之后才可能撤离和返回。"""
        if not self.cfg.get("recover_approach_failure", False) or self.safety_inhibited:
            self.fail(str(error))
            return
        self.retry_context = {"code": error.code, "message": str(error),
                              "point_index": self.point_index + 1,
                              "from_approach": self.phase in (
                                  "DESCEND", "COMPLIANT_HOLD", "MOVEIT_PLAN_DESCENT",
                                  "MOVEIT_DESCEND", "VERIFY_MOVEIT_SEGMENT")}
        self.generation += 1
        self.operation = None
        self.stop_confirmed = False
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.state("RECOVERY_STOP", "接近受限，先关闭导纳并等待旧运动完全结束")
        self.set_enabled(False, self.recovery_stopped)

    def recovery_stopped(self):
        self.stop_confirmed = True
        self.recovery_reference = None
        self.recovery_quiet_since = None
        self.state("RECOVERY_WAIT_STOP", "确认实际静止与力数据有效后再撤离")

    def recovery_step(self):
        if self.safety_inhibited or not self.stop_confirmed:
            raise RuntimeError("恢复期间安全锁定或停止尚未确认")
        if monotonic()-self.since > self.cfg["service_timeout"]:
            raise RuntimeError("旧运动结束/实际静止确认超时，不自动回位")
        if self.outstanding_actions:
            return
        force_limit = self.cfg["contact_force"] if self.retry_context["from_approach"] else self.cfg["initial_force_limit"]
        if np.linalg.norm(self.force) >= force_limit:
            raise RuntimeError("恢复前外力超过允许范围，保持停止")
        positions = np.array([self.joints.position[self.joints.name.index(j)] for j in JOINTS])
        moving = len(self.joints.velocity) == len(self.joints.name) and any(
            abs(self.joints.velocity[self.joints.name.index(j)]) > .01 for j in JOINTS)
        if self.recovery_reference is None or moving or np.max(np.abs(positions-self.recovery_reference)) > .0005:
            self.recovery_reference = positions
            self.recovery_quiet_since = monotonic()
            return
        if monotonic()-self.recovery_quiet_since < self.cfg["recovery_settle_seconds"]:
            return
        self.current_tool()  # 仍必须具有新鲜TF，不能凭规划返回值推断当前位置。
        if self.retry_context["from_approach"]:
            self.retreat()
        else:
            self.return_home()

    def cancel(self, request, response):
        del request
        if self.active:
            self.fail("用户取消；保持当前位置，不自动返回")
        response.success, response.message = True, self.detail
        return response

    def wait(self, future, callback, timeout=None):
        self.operation = (
            future,
            callback,
            monotonic(),
            timeout or self.cfg["service_timeout"],
            self.generation,
        )

    def set_enabled(self, enabled, callback):
        if self.cfg.get("touch_control_mode", "admittance") == "moveit_force_guard":
            if enabled:
                raise RuntimeError("moveit_force_guard 模式不允许启用导纳")
            callback()
            return
        req = SetBool.Request()
        req.data = enabled

        def done(result):
            if not result.success:
                raise RuntimeError(f"导纳开关失败：{result.message}")
            callback()

        self.wait(self.enable.call_async(req), done)

    def action(self, client, goal, callback):
        # 直到动作结束结果到达前，不允许新任务与取消中的旧动作重叠。
        generation = self.generation
        token = object()
        self.outstanding_actions.add(token)
        observation = {}
        try:
            future = client.send_goal_async(goal)
        except Exception:
            self.outstanding_actions.discard(token)
            raise

        def observe(f):
            if observation:
                return
            try:
                handle = f.result()
                observation["handle"] = handle
                if not handle.accepted:
                    self.outstanding_actions.discard(token)
                    return
                result_future = handle.get_result_async()
                observation["result"] = result_future
                result_future.add_done_callback(
                    lambda _: self.outstanding_actions.discard(token)
                )
                if generation != self.generation:
                    handle.cancel_goal_async()
                else:
                    self.goal_handle = handle
            except Exception as error:
                observation["error"] = error
                self.outstanding_actions.discard(token)

        future.add_done_callback(observe)

        def accepted(handle):
            observe(future)
            if "error" in observation:
                raise observation["error"]
            if not handle.accepted:
                raise RuntimeError("MoveIt 拒绝运动请求")

            def finished(result):
                self.goal_handle = None
                if (
                    result.status != GoalStatus.STATUS_SUCCEEDED
                    or result.result.error_code.val != MoveItErrorCodes.SUCCESS
                ):
                    code = result.result.error_code.val
                    if self.phase == "PREPOSITION" and code in (MoveItErrorCodes.NO_IK_SOLUTION, MoveItErrorCodes.PLANNING_FAILED):
                        raise RetryableApproachError("target_unreachable", f"预接近点不可达：{code}")
                    raise RuntimeError(f"MoveIt 执行失败：{code}")
                callback()

            self.wait(observation["result"], finished, self.cfg["phase_timeout"])

        self.wait(future, accepted)

    def orientation_path_constraints(self):
        """返回启动时 tool0 朝向的路径约束；输入为当前任务快照，输出 MoveIt Constraints。"""
        constraints = Constraints()
        if not self.cfg.get("lock_tool_orientation", False):
            return constraints
        reference = getattr(self, 'initial_pose', None)
        if reference is None:
            reference = self.current_tool()
        constraint = OrientationConstraint()
        constraint.header.frame_id = self.cfg['base_frame']
        constraint.link_name = self.cfg['tool_frame']
        constraint.orientation = to_pose(reference).orientation
        constraint.absolute_x_axis_tolerance = self.cfg['orientation_tolerance']
        constraint.absolute_y_axis_tolerance = self.cfg['orientation_tolerance']
        constraint.absolute_z_axis_tolerance = self.cfg['orientation_tolerance']
        constraint.weight = 1.0
        constraints.name = 'keep_initial_tool_orientation'
        constraints.orientation_constraints = [constraint]
        return constraints

    def motion_request(self):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.cfg["planning_group"]
        req.pipeline_id = "ompl"
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = self.cfg["velocity_scaling"]
        req.max_acceleration_scaling_factor = self.cfg["acceleration_scaling"]
        req.start_state.joint_state = deepcopy(self.joints)
        req.start_state.is_diff = True
        req.path_constraints = self.orientation_path_constraints()
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def plan_pose(self, transform, on_reached=None, phase="PREPOSITION"):
        self.state(
            phase,
            (
                "MoveIt 规划到固定初始位置"
                if phase == "MOVE_INITIAL"
                else "MoveIt 规划到探头目标的预接近点"
            ),
        )
        goal = self.motion_request()
        constraints = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.cfg["base_frame"]
        pc.link_name = self.cfg["tool_frame"]
        pc.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.cfg["pose_tolerance"] / 2]
        pc.constraint_region.primitives = [sphere]
        pc.constraint_region.primitive_poses = [to_pose(transform)]
        oc = OrientationConstraint()
        oc.header.frame_id = self.cfg["base_frame"]
        oc.link_name = self.cfg["tool_frame"]
        oc.orientation = to_pose(transform).orientation
        # 长探头会放大 tool0 姿态误差，给位置及旋转各留出误差预算。
        lever_arm = float(np.linalg.norm(self.mounting[:3, 3]))
        planning_angle = min(
            self.cfg["orientation_tolerance"],
            self.cfg["pose_tolerance"] / (4.0 * max(lever_arm, 1e-6)),
        )
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = (
            oc.absolute_z_axis_tolerance
        ) = planning_angle
        oc.weight = 1.0
        constraints.position_constraints = [pc]
        constraints.orientation_constraints = [oc]
        goal.request.goal_constraints = [constraints]
        self.action(self.move, goal, on_reached or self.verify_preposition)

    def verify_preposition(self):
        self.preposition_finished = self.get_clock().now()
        self.state("VERIFY_PREPOSITION", "等待执行完成后的新 TF，确认实际探头到位")

    def begin_descend(self):
        current = self.current_tool(after=self.preposition_finished)
        if (
            np.linalg.norm(current[:3, 3] - self.pre[:3, 3])
            > self.cfg["pose_tolerance"]
        ):
            raise RuntimeError("预接近点实际位置未到达")
        if not self.rotation_ok(current, self.pre):
            raise RuntimeError("预接近姿态未到达")
        actual_probe = current @ self.mounting
        expected_probe = self.pre @ self.mounting
        if (
            np.linalg.norm(actual_probe[:3, 3] - expected_probe[:3, 3])
            > self.cfg["pose_tolerance"]
        ):
            raise RuntimeError("预接近时实际探头坐标未到达，不能仅检查 tool0 位置")
        self.command = current.copy()
        self.line_start = current.copy()
        self.contact_stop_requested = False
        self.moveit_stop_since = None
        self.moveit_stop_reference = None
        self.axis = self.direction_for(current)
        delta = self.final[:3, 3]-current[:3, 3]
        self.nominal_distance = float(np.dot(delta, self.axis))
        lateral = delta-self.nominal_distance*self.axis
        if self.nominal_distance <= 0 or np.linalg.norm(lateral) > self.cfg['pose_tolerance']:
            raise RuntimeError('名义检测点不在当前接近轴前方，或预接近横向误差超限')
        self.travel_limit = self.nominal_distance+self.cfg['max_penetration']
        self.get_logger().info(
            f"接近方向={self.cfg['approach_axis']}，基座单位向量={self.axis.tolist()}；"
            f"名义行程={self.nominal_distance*1000.:.1f}mm，额外行程上限={self.cfg['max_penetration']*1000.:.1f}mm")
        if self.cfg.get("touch_control_mode", "admittance") == "moveit_force_guard":
            self.state("MOVEIT_WAIT_CONTACT", "准备一次性规划沿接近轴的无碰撞直线路径")
        else:
            self.publish_target(self.command)
            # 单独的命令话题和服务之间可能跨通道乱序；先让控制器锁存当前点，再启用。
            self.state("ARM_DESCEND", "锁存当前 tool0 位姿后启用导纳")

    def rotation_ok(self, a, b):
        cosine = np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1) / 2, -1.0, 1.0)
        return math.acos(float(cosine)) <= self.cfg["orientation_tolerance"]

    def publish_target(self, transform):
        msg = PoseStamped()
        msg.header.frame_id = self.cfg["base_frame"]
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = to_pose(transform)
        self.target_pub.publish(msg)

    def begin_hold(self):
        self.contact = self.current_tool()
        self.thickness_result = None
        self.measurement_started_ns = self.get_clock().now().nanoseconds
        self.state("HOLD", f'停止位置推进，保持 {self.cfg["hold_seconds"]} 秒')
        message = PoseStamped()
        message.header.frame_id = self.cfg["base_frame"]
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose = to_pose(self.contact @ self.mounting)
        self.contact_pub.publish(message)
        if self.measure_pub.get_subscription_count():
            self.measure_pub.publish(Empty())
        else:
            self.get_logger().warning(
                "测厚采集节点未连接：本次仅执行接触保持，不会生成真实厚度读数"
            )

    def request_moveit_force_stop(self):
        """取消当前普通 MoveIt 轨迹；动作确认结束后进入保持阶段。"""
        if self.contact_stop_requested or self.phase in ("HOLD", "STOP_CONTACT"):
            return
        self.contact_stop_requested = True
        self.moveit_stop_joint_stamp = Time.from_msg(self.joints.header.stamp).nanoseconds
        self.moveit_stop_since = None
        self.moveit_stop_reference = None
        self.generation += 1
        self.operation = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.state("STOP_CONTACT", f"达到{self.cfg['contact_force']:g}N，取消当前 MoveIt 轨迹并等待停止确认")

    def moveit_stop_ready(self):
        # 只使用取消之后的新关节反馈，不能把取消前的静止快照当成已经停稳。
        if (self.moveit_stop_joint_stamp is None or
                Time.from_msg(self.joints.header.stamp).nanoseconds <= self.moveit_stop_joint_stamp):
            self.moveit_stop_since = None
            return False
        positions = np.array([
            self.joints.position[self.joints.name.index(j)] for j in JOINTS
        ])
        moving = len(self.joints.velocity) == len(self.joints.name) and any(
            abs(self.joints.velocity[self.joints.name.index(j)]) > 0.01 for j in JOINTS)
        if (self.moveit_stop_reference is None or moving or
                np.max(np.abs(positions - self.moveit_stop_reference)) > 0.0005):
            self.moveit_stop_reference = positions
            self.moveit_stop_since = monotonic()
            return False
        return self.moveit_stop_since is not None and monotonic() - self.moveit_stop_since >= 0.2

    def verify_moveit_segment(self):
        self.moveit_segment_finished = self.get_clock().now()
        self.state("VERIFY_MOVEIT_SEGMENT", "等待接近轨迹结束后的新 TF")

    def plan_moveit_descent(self):
        """一次性规划受碰撞检查的直线接近轨迹，执行期间由力阈值取消。"""
        if self.contact_stop_requested:
            return
        current = self.current_tool()
        if self.axial_force(self.force) >= self.cfg['contact_force']:
            self.begin_hold()
            return
        limit = self.travel_limit if self.cfg['continue_until_contact'] else self.nominal_distance
        target = current.copy()
        target[:3, 3] = self.line_start[:3, 3] + self.axis * limit
        if not self.in_workspace(target) or not self.in_workspace(target @ self.mounting):
            raise RetryableApproachError('target_unreachable', 'MoveIt 接近终点超出工作空间')
        self.state('MOVEIT_PLAN_DESCENT', 'MoveIt 计算单条直线接近路径；避障检查覆盖整条路径')
        req = GetCartesianPath.Request()
        req.header.frame_id = self.cfg['base_frame']
        req.start_state.joint_state = deepcopy(self.joints)
        req.start_state.is_diff = True
        req.group_name = self.cfg['planning_group']
        req.link_name = self.cfg['tool_frame']
        req.waypoints = [to_pose(target)]
        req.max_step = float(self.cfg['moveit_cartesian_step'])
        req.jump_threshold = 1.5
        req.avoid_collisions = True
        req.path_constraints = self.orientation_path_constraints()
        req.cartesian_speed_limited_link = self.cfg['tool_frame']
        req.max_cartesian_speed = float(self.cfg['approach_speed'])
        req.max_velocity_scaling_factor = self.cfg['velocity_scaling']
        req.max_acceleration_scaling_factor = self.cfg['acceleration_scaling']

        def planned(result):
            if result.error_code.val != MoveItErrorCodes.SUCCESS or result.fraction < 0.999:
                raise RetryableApproachError(
                    'target_unreachable',
                    f'MoveIt 无法生成完整无碰撞接近路径，完成比例={result.fraction:.3f}')
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = result.solution
            self.state('MOVEIT_DESCEND',
                       f'沿接近轴以 {self.cfg["approach_speed"]:g} m/s 执行无碰撞路径；{self.cfg["contact_force"]:g}N 时取消')
            self.action(self.execute, goal, self.verify_moveit_segment)

        self.wait(self.cartesian.call_async(req), planned, self.cfg['phase_timeout'])

    def verify_moveit_descent(self):
        current = self.current_tool(after=self.moveit_segment_finished)
        delta = current[:3, 3] - self.line_start[:3, 3]
        actual_distance = float(np.dot(delta, self.axis))
        lateral = delta - actual_distance * self.axis
        if np.linalg.norm(lateral) > self.cfg['lateral_tolerance'] or not self.rotation_ok(current, self.line_start):
            raise RuntimeError('MoveIt 接近时发生横向偏移或姿态超限')
        limit = self.travel_limit if self.cfg['continue_until_contact'] else self.nominal_distance
        if actual_distance > limit + self.cfg['pose_tolerance']:
            raise RuntimeError('实际探入行程超过限制')
        if abs(actual_distance - limit) > self.cfg['pose_tolerance']:
            raise RuntimeError('MoveIt 接近轨迹执行后未到达设定行程')
        if self.axial_force(self.force) >= self.cfg['contact_force']:
            self.begin_hold()
            return
        if self.cfg['continue_until_contact']:
            raise RetryableApproachError(
                'travel_exhausted',
                f'额外接近行程已用完，轴向力仍未达到{self.cfg["contact_force"]:g}N')
        raise RetryableApproachError(
            'contact_not_reached', '到达名义检测点但轴向力未达到接触阈值')

    def descend_step(self):
        current = self.current_tool()
        delta = current[:3, 3]-self.line_start[:3, 3]
        actual_distance = float(np.dot(delta, self.axis))
        lateral = delta-actual_distance*self.axis
        if np.linalg.norm(lateral) > self.cfg['lateral_tolerance'] or not self.rotation_ok(current, self.line_start):
            raise RuntimeError('沿接近轴运动时横向偏移或姿态超限')
        if actual_distance > self.travel_limit+self.cfg['pose_tolerance']:
            raise RuntimeError('实际探入行程超过限制')
        if not self.in_workspace(current) or not self.in_workspace(current @ self.mounting):
            raise RuntimeError('实际 tool0 或探针触点超出工作空间')
        if np.linalg.norm(current[:3, 3]-self.command[:3, 3]) > self.cfg['max_tracking_error']:
            raise RuntimeError('导纳跟踪误差过大')
        if self.phase == 'COMPLIANT_HOLD':
            return
        commanded_distance = float(np.dot(self.command[:3, 3]-self.line_start[:3, 3], self.axis))
        continuing = self.cfg['continue_until_contact']
        limit = self.travel_limit if continuing else self.nominal_distance
        if commanded_distance >= limit-1e-7:
            if continuing:
                raise RetryableApproachError("travel_exhausted", f'额外接近行程已用完，轴向力仍未达到{self.cfg["contact_force"]:g}N')
            self.state('COMPLIANT_HOLD', '已到检测目标，维持阻抗控制；等待接触力达到阈值')
            return
        candidate = self.command.copy()
        travel = min(limit, commanded_distance+self.cfg['approach_speed']/self.cfg['rate'])
        candidate[:3, 3] = self.line_start[:3, 3]+self.axis*travel
        if not self.in_workspace(candidate) or not self.in_workspace(candidate @ self.mounting):
            raise RetryableApproachError('target_unreachable', '下一接近增量超出 tool0 或探针工作空间')
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.cfg["planning_group"]
        req.ik_request.ik_link_name = self.cfg["tool_frame"]
        req.ik_request.robot_state.joint_state = deepcopy(self.joints)
        req.ik_request.pose_stamped.header.frame_id = self.cfg["base_frame"]
        req.ik_request.pose_stamped.pose = to_pose(candidate)
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.nanosec = 100000000

        def checked(result):
            if result.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                raise RetryableApproachError("target_unreachable", "接近增量没有无碰撞IK解")
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError("接近增量IK服务发生异常")
            self.command = candidate
            self.publish_target(candidate)

        self.wait(self.ik.call_async(req), checked)

    def retreat(self):
        self.state("RETRACT", "MoveIt 计算沿接近轴反向撤离的路径")
        current = self.current_tool()
        retreat = current.copy()
        distance = max(self.cfg['approach_clearance'], float(np.dot(current[:3, 3]-self.pre[:3, 3], self.axis)))
        retreat[:3, 3] -= self.axis*distance
        if not self.in_workspace(retreat) or not self.in_workspace(retreat @ self.mounting):
            raise RuntimeError('反向撤离目标超出工作空间；保持停止，不强行返回')
        req = GetCartesianPath.Request()
        req.header.frame_id = self.cfg["base_frame"]
        req.start_state.joint_state = deepcopy(self.joints)
        req.start_state.is_diff = True
        req.group_name = self.cfg["planning_group"]
        req.link_name = self.cfg["tool_frame"]
        req.waypoints = [to_pose(retreat)]
        req.max_step = 0.001
        req.jump_threshold = 1.5
        req.avoid_collisions = True
        req.path_constraints = self.orientation_path_constraints()
        req.cartesian_speed_limited_link = self.cfg["tool_frame"]
        req.max_cartesian_speed = 0.005
        # Jazzy 支持这些字段；限制撤离轨迹的关节速度和加速度。
        req.max_velocity_scaling_factor = self.cfg["velocity_scaling"]
        req.max_acceleration_scaling_factor = self.cfg["acceleration_scaling"]

        def planned(result):
            if (
                result.error_code.val != MoveItErrorCodes.SUCCESS
                or result.fraction < 0.999
            ):
                raise RuntimeError("沿接近轴反向撤离路径不完整，保持当前位置")
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = result.solution
            self.action(self.execute, goal, self.return_home)

        self.wait(self.cartesian.call_async(req), planned, self.cfg["phase_timeout"])

    def prepare_posture(self, name, response):
        """公开准备接口：只接受已配置关节目标，动作完成后有实际到位确认。"""
        try:
            if not self.cfg.get('joint_postures_enabled', False):
                raise ValueError('本配置未启用双关节姿态')
            ready, reason = self.motion_readiness()
            if not ready:
                raise ValueError(reason)
            self.generation += 1
            self.operation = None
            def move():
                if name == 'travel':
                    self.plan_joints(self.postures['detection_ready'],
                        lambda: self.plan_joints(self.postures['travel'],
                            lambda: self.state('READY_TRAVEL', '已到行驶关节姿态'), 'TRAVEL_POSTURE'),
                        'RETURN_GATEWAY')
                else:
                    def capture_ready():
                        if self.capture_joints is None:
                            self.state('READY_DETECTION', '已到检测准备关节姿态')
                        else:
                            positions = [self.capture_joints.position[self.capture_joints.name.index(j)] for j in JOINTS]
                            self.plan_joints(positions,
                                lambda: self.state('READY_DETECTION', '已恢复本轮原拍照关节姿态'),
                                'CAPTURE_POSTURE')
                    self.plan_joints(self.postures['detection_ready'], capture_ready, 'DETECTION_GATEWAY')
            self.set_enabled(False, move)
            response.success, response.message = True, '关节姿态任务已接受，等待实际到位'
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def plan_joints(self, positions, callback, phase):
        self.joint_stage_target = list(positions)
        self.joint_stage_callback = callback
        self.state(phase, '按六轴关节目标规划；不使用末端XYZ求逆解')
        goal = self.motion_request()
        goal.request.path_constraints = Constraints()
        goal.request.goal_constraints = [joint_constraints(positions)]
        def finished():
            self.joint_stage_finished = self.get_clock().now()
            self.state('VERIFY_JOINT_POSTURE', '确认实际关节到位及新鲜TF')
        self.action(self.move, goal, finished)

    def verify_joint_posture(self):
        error = max(abs(self.joints.position[self.joints.name.index(name)]-value)
                    for name, value in zip(JOINTS, self.joint_stage_target))
        if error > self.cfg['home_joint_tolerance']:
            raise RuntimeError('关节准备姿态尚未实际到位')
        self.current_tool(after=self.joint_stage_finished)

    def return_home(self, initial=False):
        self.verify_initial_home = initial
        if self.cfg.get('joint_postures_enabled', False):
            self.plan_joints(self.postures['detection_ready'],
                lambda: self.return_home_joints(initial), 'RETURN_GATEWAY')
        else:
            self.return_home_joints(initial)

    def return_home_joints(self, initial=False):
        self.verify_initial_home = initial
        self.state('RETURN', '按关节角度返回行驶原位' if self.cfg.get('joint_postures_enabled', False)
                   else '规划返回任务开始时的六轴关节位置')
        goal = self.motion_request()
        goal.request.path_constraints = Constraints()
        positions = [self.initial_joints.position[self.initial_joints.name.index(j)] for j in JOINTS]
        goal.request.goal_constraints = [joint_constraints(positions)]
        self.action(self.move, goal,
            lambda: self.state('VERIFY_HOME', '确认实际返回关节原位且外力回落后再继续'))

    def fail(self, detail):
        self.stop_confirmed = False
        self.generation += 1
        self.operation = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.state("FAULT", detail)
        if self.touch_control_mode == "moveit_force_guard":
            # MoveIt 轨迹取消由 action handle 处理；此模式没有导纳停止服务。
            self.stop_confirmed = not bool(self.outstanding_actions)
            return
        if self.enable.service_is_ready():
            req = SetBool.Request()
            req.data = False
            self.stop_future = self.enable.call_async(req)
            self.stop_requested = monotonic()
        else:
            self.get_logger().error("导纳停止服务不可用，请使用机器人停止功能")

    def tick(self):
        try:
            if monotonic() - self.last_status > 0.5:
                self.publish_status()
            if self.stop_future is not None:
                if self.stop_future.done():
                    result = self.stop_future.result()
                    self.stop_future = None
                    self.stop_confirmed = result.success
                    if not result.success:
                        self.get_logger().error("停止保持失败：" + result.message)
                elif monotonic() - self.stop_requested > self.cfg["service_timeout"]:
                    self.get_logger().error("停止服务超时，请使用机器人停止功能")
                    self.stop_future = None
            if not self.active:
                return
            if self.cfg.get('joint_postures_enabled', False) and (
                    not self.ground_ready or self.ground_subscription.get_publisher_count() == 0):
                raise RuntimeError('地面碰撞场景确认丢失')
            if not self.fresh():
                raise RuntimeError("关节、原始力或滤波力数据超时")
            if self.cfg["lock_tool_orientation"] and not self.rotation_ok(self.current_tool(), self.initial_pose):
                raise RuntimeError("运动途中 tool0 朝向偏离启动姿态，已取消检测")
            if (
                self.phase not in ("HOLD", "COMPLIANT_HOLD", "STOP_CONTACT")
                and monotonic() - self.since > self.cfg["phase_timeout"]
            ):
                raise RuntimeError("当前阶段超时")
            if self.phase in ("DESCEND", "COMPLIANT_HOLD"):
                # 等待增量 IK 或实际到位时，持续发送上一个已验证目标；不增加行程。
                # 数据断流、超载和阶段超时仍由前面的检查和力回调立即停止。
                self.publish_target(self.command)
            if self.phase == "MOVEIT_DESCEND":
                current = self.current_tool()
                delta = current[:3, 3] - self.line_start[:3, 3]
                distance = float(np.dot(delta, self.axis))
                limit = self.travel_limit if self.cfg['continue_until_contact'] else self.nominal_distance
                if (np.linalg.norm(delta-distance*self.axis) > self.cfg['lateral_tolerance']
                        or not self.rotation_ok(current, self.line_start)):
                    raise RuntimeError('MoveIt 接近过程中发生横向偏移或姿态超限')
                if distance > limit + self.cfg['pose_tolerance']:
                    raise RuntimeError('MoveIt 接近过程中实际探入行程超过限制')
            if self.operation:
                future, callback, since, timeout, generation = self.operation
                if future.done():
                    self.operation = None
                    if generation == self.generation:
                        callback(future.result())
                elif monotonic() - since > timeout:
                    raise RuntimeError("服务或动作响应超时")
                return
            if self.phase == "STOP_CONTACT" and self.touch_control_mode == "moveit_force_guard":
                if monotonic() - self.since > self.cfg["service_timeout"]:
                    raise RuntimeError("接触停止后的 MoveIt 轨迹取消或停稳确认超时")
                if self.outstanding_actions:
                    return
                if self.moveit_stop_ready():
                    self.begin_hold()
                return
            if self.phase == "ARM_DESCEND" and monotonic() - self.since > 0.2:
                self.set_enabled(
                    True, lambda: self.state("DESCEND", f"沿 {self.cfg['approach_axis']} 低速导纳接近；达到接触力后停止推进")
                )
            elif self.phase == "VERIFY_INITIAL":
                try:
                    self.initial_arrived()
                except Exception as error:
                    if monotonic() - self.since > self.cfg["service_timeout"]:
                        raise RuntimeError(f"初始位置到位确认超时：{error}") from error
            elif self.phase == "VERIFY_PREPOSITION":
                try:
                    self.begin_descend()
                except Exception as error:
                    if monotonic() - self.since > self.cfg["service_timeout"]:
                        raise RuntimeError(f"预接近到位确认超时：{error}") from error
            elif self.phase == "RECOVERY_WAIT_STOP":
                self.recovery_step()
            elif self.phase == "VERIFY_JOINT_POSTURE":
                try:
                    self.verify_joint_posture()
                except Exception as error:
                    if monotonic()-self.since > self.cfg['service_timeout']:
                        raise RuntimeError(f'关节姿态到位确认超时：{error}') from error
                else:
                    self.joint_stage_callback()
            elif self.phase == "VERIFY_HOME":
                if (
                    self.home_reached()
                    and np.linalg.norm(self.force) < self.cfg["initial_force_limit"]
                ):
                    self.point_returned(initial=self.verify_initial_home)
                elif monotonic() - self.since > self.cfg["service_timeout"]:
                    raise RuntimeError("返回后未确认原位或外力未回落，停止后续检测点")
            elif self.phase in ("DESCEND", "COMPLIANT_HOLD"):
                self.descend_step()
            elif self.phase == "MOVEIT_WAIT_CONTACT":
                self.plan_moveit_descent()
            elif self.phase == "VERIFY_MOVEIT_SEGMENT":
                self.verify_moveit_descent()
            elif (
                self.phase == "HOLD"
                and monotonic() - self.since >= self.cfg["hold_seconds"]
            ):
                self.retreat()
        except RetryableApproachError as error:
            self.begin_recovery(error)
        except Exception as error:
            self.fail(str(error))


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
        node = DetectController()
        executor.add_node(node)
        while rclpy.ok() and not stopping:
            executor.spin_once(timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        stopping = True
    finally:
        try:
            if node is not None:
                node.tick_timer.cancel()
                try:
                    # 不在信号处理函数中打断 ROS 锁；先取消并请求当前位置保持。
                    if rclpy.ok() and (node.active or node.outstanding_actions):
                        node.get_logger().info('收到退出请求，取消检测并请求当前位置保持')
                        node.fail('节点退出')
                        deadline = monotonic()+1.
                        while (rclpy.ok() and not expedite and monotonic() < deadline
                               and ((node.stop_future is not None and not node.stop_future.done())
                                    or node.outstanding_actions)):
                            executor.spin_once(timeout_sec=.05)
                        if ((node.stop_future is not None and not node.stop_future.done())
                                or node.outstanding_actions):
                            node.get_logger().warning('退出保持未确认；请检查机器人停止状态')
                finally:
                    executor.remove_node(node)
                    node.destroy_node()
        finally:
            executor.shutdown(timeout_sec=.5)
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
