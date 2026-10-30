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
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
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


class DetectController(Node):
    def __init__(self):
        super().__init__("detect_controller")
        defaults = {
            "base_frame": "base_link",
            "tool_frame": "tool0",
            "probe_frame": "terminal_link_2",
            "planning_group": "cs_manipulator",
            "target_topic": "/geminipro_calibration/target_base",
            "targets_topic": "/geminipro_calibration/targets_base",
            "targets_file": "~/.ros/detect_control/targets.yaml",
            "measurement_results_file": "~/.ros/detect_control/thickness_results.jsonl",
            "max_targets": 100,
            "home_joint_tolerance": 0.01,
            "admittance_namespace": "/admittance_controller",
            "raw_wrench_topic": "/sensor/wrench",
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
            "approach_clearance": 0.02,
            "max_penetration": 0.005,
            "approach_speed": 0.002,
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
        # 未提供度制参数时兼容旧 YAML 的 orientation_tolerance（弧度）。
        degrees = self.declare_parameter(
            "orientation_tolerance_deg", math.degrees(self.cfg["orientation_tolerance"])
        ).value
        if not math.isfinite(degrees) or not 0 < degrees <= 180:
            raise ValueError("orientation_tolerance_deg 必须在 (0, 180] 度范围内")
        self.cfg["orientation_tolerance"] = math.radians(degrees)
        if self.cfg["initial_orientation_mode"] not in ("current", "fixed"):
            raise ValueError("initial_orientation_mode 必须为 current 或 fixed")
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
        self.batch = []
        self.point_index = 0
        self.completed_points = 0
        self.recorded_home = None
        self.initial_position_ready = False
        self.initial_orientation_xyzw = None
        self.targets_file = Path(self.cfg["targets_file"]).expanduser()
        if not 1 <= self.cfg["max_targets"] <= 1000:
            raise ValueError("max_targets 必须在 1～1000 之间")
        self.joints = None
        self.joints_at = -math.inf
        self.force = None
        self.force_at = -math.inf
        self.raw_at = -math.inf
        self.stop_confirmed = True
        self.generation = 0
        self.last_status = 0.0
        self.stop_future = None
        namespace = self.cfg["admittance_namespace"].rstrip("/")
        self.target_pub = self.create_publisher(
            PoseStamped, namespace + "/tracking_target_pose", 10
        )
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.window_pub = self.create_publisher(Bool, "~/measurement_active", 10)
        self.contact_pub = self.create_publisher(PoseStamped, "~/contact_pose", 10)
        self.measure_pub = self.create_publisher(Empty, "/detactor/acquire_trigger", 10)
        self.thickness_result = None
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
        self.create_service(Trigger, "~/start", self.start)
        self.create_service(Trigger, "~/cancel", self.cancel)
        self.create_service(Trigger, "~/record_home", self.record_home)
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
        return self.phase not in ("IDLE", "DONE", "FAULT")

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

    def motion_readiness(self):
        """返回初始运动能否开始及等待原因；只读取状态，不触发运动。"""
        if self.active or self.outstanding_actions:
            return False, "等待当前运动结束"
        if not self.stop_confirmed or self.stop_future is not None:
            return False, "等待上次停止确认"
        if not self.fresh():
            return False, "等待新鲜关节、原始力和零偏初始化后的滤波力数据"
        if np.linalg.norm(self.force) >= self.cfg["initial_force_limit"]:
            return False, "初始受力超过允许值，请检查探头接触或力零偏"
        if not self.enable.service_is_ready() or not self.move.server_is_ready():
            return False, "等待导纳服务和 MoveIt 运动接口"
        return True, "关节、力数据和运动接口已就绪"

    def publish_status(self):
        motion_ready, readiness_message = self.motion_readiness()
        message = String()
        message.data = json.dumps(
            {
                "state": self.phase,
                "motion_ready": motion_ready,
                "readiness_message": readiness_message,
                "message": self.detail,
                "force_z_N": None if self.force is None else float(self.force[2]),
                "contact_force_N": self.cfg["contact_force"],
                "queued_points": len(self.targets),
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
            raise ValueError("检测点超过工作空间")
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
            self.publish_status()
        except Exception as error:
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
            self.recorded_home = deepcopy(self.joints)
            response.success, response.message = True, "已记录当前六轴关节原位"
            self.publish_status()
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def prepare_initial(self, request, response):
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
            if not self.enable.service_is_ready() or not self.move.server_is_ready():
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
            self.set_enabled(
                False,
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
        self.force, self.force_at = values, monotonic()
        if self.active and np.linalg.norm(values) >= self.cfg["overload_force"]:
            self.fail("合力超过保护阈值")
        elif (
            self.phase in ("DESCEND", "COMPLIANT_HOLD")
            and abs(values[2]) >= self.cfg["contact_force"]
        ):
            # 在收到力回调时立即停止，不等待下一次推进定时器。
            self.operation = None
            self.state("STOP_CONTACT", "已达到 Z 向接触力阈值，停止导纳输出")
            self.set_enabled(False, self.begin_hold)

    def fresh(self):
        now = monotonic()
        limit = self.cfg["data_timeout"]
        return (
            self.joints is not None
            and self.force is not None
            and all(
                now - x <= limit for x in (self.joints_at, self.force_at, self.raw_at)
            )
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

    def in_workspace(self, transform):
        p = transform[:3, 3]
        return bool(
            np.all(p >= self.cfg["workspace_min"])
            and np.all(p <= self.cfg["workspace_max"])
        )

    def start(self, request, response):
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
            if not self.fresh():
                raise ValueError(
                    "等待新鲜关节、原始力和去零滤波力；先启动驱动、传感器和导纳"
                )
            if np.linalg.norm(self.force) >= self.cfg["initial_force_limit"]:
                raise ValueError("启动前外力过大；请在探头未接触时正确清零")
            if (
                not all(
                    c.service_is_ready() for c in (self.enable, self.ik, self.cartesian)
                )
                or not self.move.server_is_ready()
                or not self.execute.server_is_ready()
            ):
                raise ValueError("MoveIt/导纳服务尚未就绪")
            if self.target_pub.get_subscription_count() == 0:
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
                pre = final.copy()
                pre[2, 3] += self.cfg["approach_clearance"]
                far = final.copy()
                far[2, 3] -= self.cfg["max_penetration"]
                if not all(self.in_workspace(t) for t in (target, final, pre, far)):
                    raise ValueError(
                        "某检测点、预接近点或最大探入点超过工作空间；整组不执行"
                    )
                batch.append((pre, final))
            self.mounting = mounting
            self.initial_joints = deepcopy(
                self.recorded_home if self.recorded_home is not None else self.joints
            )
            self.initial_pose = self.current_tool()
            self.batch = batch
            self.point_index = 0
            self.completed_points = 0
            self.pre, self.far = self.batch[0]
            self.generation += 1
            self.operation = None
            self.state("PREPARE", "关闭导纳输出后进行预接近规划")
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
        if self.home_reached():
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
        self.completed_points += 1
        if self.completed_points == len(self.batch):
            self.state("DONE", "全部检测点测完并返回同一原位；导纳输出保持关闭")
            return
        if np.linalg.norm(self.force) >= self.cfg["initial_force_limit"]:
            raise RuntimeError("回原位后仍有较大外力，停止后续检测点")
        self.point_index += 1
        self.pre, self.far = self.batch[self.point_index]
        self.plan_pose(self.pre)

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
                    raise RuntimeError(
                        f"MoveIt 执行失败：{result.result.error_code.val}"
                    )
                callback()

            self.wait(observation["result"], finished, self.cfg["phase_timeout"])

        self.wait(future, accepted)

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
                else "MoveIt 规划到探头目标正上方"
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
        self.hold_z = self.far[2, 3]
        self.bottom_z = self.hold_z - self.cfg["max_penetration"]
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

    def descend_step(self):
        current = self.current_tool()
        if np.linalg.norm(current[:2, 3] - self.line_start[:2, 3]) > self.cfg[
            "lateral_tolerance"
        ] or not self.rotation_ok(current, self.line_start):
            raise RuntimeError("垂直接近时 XY 或姿态偏离限制")
        if current[2, 3] < self.bottom_z - self.cfg["pose_tolerance"]:
            raise RuntimeError("实际探入行程超过限制")
        if (
            np.linalg.norm(current[:3, 3] - self.command[:3, 3])
            > self.cfg["max_tracking_error"]
        ):
            raise RuntimeError("导纳跟踪误差过大")
        if self.phase == "COMPLIANT_HOLD":
            return
        if self.command[2, 3] <= self.hold_z + 1e-7:
            self.state("COMPLIANT_HOLD", "已到检测目标，维持阻抗控制；等待接触力达到阈值")
            return
        candidate = self.command.copy()
        candidate[2, 3] = max(
            self.hold_z,
            candidate[2, 3] - self.cfg["approach_speed"] / self.cfg["rate"],
        )
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.cfg["planning_group"]
        req.ik_request.ik_link_name = self.cfg["tool_frame"]
        req.ik_request.robot_state.joint_state = deepcopy(self.joints)
        req.ik_request.pose_stamped.header.frame_id = self.cfg["base_frame"]
        req.ik_request.pose_stamped.pose = to_pose(candidate)
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.nanosec = 100000000

        def checked(result):
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError("接近增量没有无碰撞 IK 解")
            self.command = candidate
            self.publish_target(candidate)

        self.wait(self.ik.call_async(req), checked)

    def retreat(self):
        self.state("RETRACT", "MoveIt 计算沿基座 +Z 的垂直撤离路径")
        current = self.current_tool()
        retreat = current.copy()
        retreat[2, 3] = self.pre[2, 3]
        if retreat[2, 3] <= current[2, 3]:
            raise RuntimeError("撤离目标不在当前位置上方")
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
                raise RuntimeError("垂直撤离路径不完整，保持当前位置")
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = result.solution
            self.action(self.execute, goal, self.return_home)

        self.wait(self.cartesian.call_async(req), planned, self.cfg["phase_timeout"])

    def return_home(self, initial=False):
        self.verify_initial_home = initial
        self.state("RETURN", "规划返回任务开始时的六轴关节位置")
        goal = self.motion_request()
        constraints = Constraints()
        for joint in JOINTS:
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = self.initial_joints.position[
                self.initial_joints.name.index(joint)
            ]
            jc.tolerance_above = jc.tolerance_below = 0.005
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints = [constraints]
        self.action(
            self.move,
            goal,
            lambda: self.state("VERIFY_HOME", "确认已返回记录原位且外力回落后再继续"),
        )

    def fail(self, detail):
        self.stop_confirmed = False
        self.generation += 1
        self.operation = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.state("FAULT", detail)
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
            if not self.fresh():
                raise RuntimeError("关节、原始力或滤波力数据超时")
            if (
                self.phase not in ("HOLD", "COMPLIANT_HOLD")
                and monotonic() - self.since > self.cfg["phase_timeout"]
            ):
                raise RuntimeError("当前阶段超时")
            if self.phase in ("DESCEND", "COMPLIANT_HOLD"):
                # 等待增量 IK 或实际到位时，持续发送上一个已验证目标；不增加行程。
                # 数据断流、超载和阶段超时仍由前面的检查和力回调立即停止。
                self.publish_target(self.command)
            if self.operation:
                future, callback, since, timeout, generation = self.operation
                if future.done():
                    self.operation = None
                    if generation == self.generation:
                        callback(future.result())
                elif monotonic() - since > timeout:
                    raise RuntimeError("服务或动作响应超时")
                return
            if self.phase == "ARM_DESCEND" and monotonic() - self.since > 0.2:
                self.set_enabled(
                    True, lambda: self.state("DESCEND", "沿基座 −Z 低速导纳接近")
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
            elif (
                self.phase == "HOLD"
                and monotonic() - self.since >= self.cfg["hold_seconds"]
            ):
                self.retreat()
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
                    if rclpy.ok() and node.active:
                        node.get_logger().info('收到退出请求，取消检测并请求当前位置保持')
                        node.fail('节点退出')
                        deadline = monotonic()+1.
                        while (rclpy.ok() and not expedite and monotonic() < deadline
                               and node.stop_future is not None and not node.stop_future.done()):
                            executor.spin_once(timeout_sec=.05)
                        if node.stop_future is not None and not node.stop_future.done():
                            node.get_logger().warning('退出保持未确认；请检查机器人停止状态')
                finally:
                    executor.remove_node(node)
                    node.destroy_node()
        finally:
            executor.shutdown(timeout_sec=.5)
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
