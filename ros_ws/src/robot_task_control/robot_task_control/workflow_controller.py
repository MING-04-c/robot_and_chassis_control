#!/usr/bin/env python3
"""Execute a YAML-defined chassis and robot motion workflow."""

import json
from copy import deepcopy
import math
import os
import signal
import subprocess
import sys
from pathlib import Path
from time import monotonic
from uuid import uuid4

import rclpy
import yaml
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseArray, WrenchStamped
from nav_msgs.msg import Odometry
from masc_chassis_msgs.msg import TaskStatus
from masc_chassis_msgs.srv import TaskControl
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String, Empty
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener


CONTROL_COMMANDS = {
    "chassis_execute": TaskControl.Request.EXECUTE,
    "chassis_stop": TaskControl.Request.STOP,
}
SUPPORTED_ACTIONS = set(CONTROL_COMMANDS) | {
    "delay",
    "wait_chassis",
    "arm_poses",
    "arm_move",
    "hk_capture",
    "final_capture",
    "line_follow",
    "wait_chassis_stopped",
    "save_line_route",
    "arm_posture",
    "continue_line_follow",
}


class WorkflowController(Node):
    def __init__(self):
        super().__init__("task_controller")
        workflow_file = str(self.declare_parameter("workflow_file", "").value)
        if not workflow_file:
            raise ValueError("workflow_file parameter is required")
        self.standard_output = str(self.declare_parameter("standard_route_output", "").value)
        self.capture_process = None
        self.capture_only = False
        self.capture_config_file = str(self.declare_parameter("capture_config_file", "").value)
        self.workflow_path = Path(workflow_file).resolve()
        self.workflow = self.load_workflow(self.workflow_path)
        self.workflow_name = str(self.workflow.get("name", "unnamed_workflow"))
        self.steps = deepcopy(self.workflow["steps"])
        self.original_steps = deepcopy(self.steps)
        if self.standard_output and not any(step['action'] == 'save_line_route' for step in self.steps):
            raise ValueError('teach模式需要包含save_line_route的流程，不能关闭round_trip后直接结束录制')
        self.manage_line_stream = bool(self.declare_parameter('manage_line_stream', False).value)
        self.line_streaming = True
        self.line_stream_wait = None
        self.line_warmup_deadline = None
        self.line_warmup_future = None
        self.manage_admittance = bool(self.declare_parameter('manage_admittance', True).value)
        self.joint_posture_only = bool(self.declare_parameter('joint_posture_only', False).value)
        if self.joint_posture_only and any(step['action'] in ('arm_move', 'arm_poses') for step in self.steps):
            raise ValueError('joint_posture_only流程只允许关节姿态动作，不允许笛卡尔/导纳动作')
        self.station_context = None
        self.station_resume_requested = False
        self.station_rearmed = True
        self.posture_status = None
        self.posture_status_time = None
        self.posture_request_id = None
        self.posture_clients = {name: self.create_client(Trigger, '/arm_joint_postures/'+name)
                                for name in ('travel', 'detection_ready')}
        self.posture_cancel_client = self.create_client(Trigger, '/arm_joint_postures/cancel')
        self.create_subscription(String, '/arm_joint_postures/status', self.on_posture_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Empty, '/line_follow/resume', self.on_station_resume, 10)
        self.tasks = self.load_task_table(self.workflow)
        self.validate_task_references()
        self.auto_start = bool(
            self.declare_parameter(
                "auto_start", bool(self.workflow.get("auto_start", False))
            ).value
        )
        self.step_timeout = float(
            self.declare_parameter("step_timeout_sec", 90.0).value
        )
        if not math.isfinite(self.step_timeout) or self.step_timeout <= 0.0:
            raise ValueError("step_timeout_sec must be finite and positive")
        self.control_service = str(
            self.declare_parameter("control_service", "/chassis/task_control").value
        )
        self.status_topic = str(
            self.declare_parameter("chassis_status_topic", "/chassis/task_status").value
        )
        self.target_topic = str(
            self.declare_parameter(
                "target_topic", "/moveit_pose_executor/target_poses"
            ).value
        )
        self.execute_service = str(
            self.declare_parameter(
                "execute_service", "/moveit_pose_executor/execute_latest"
            ).value
        )
        self.compliant_target_topic = str(
            self.declare_parameter(
                "compliant_target_topic",
                "/admittance_controller/target_poses",
            ).value
        )
        self.compliant_wrench_topic = str(
            self.declare_parameter(
                "compliant_wrench_topic",
                "/admittance_controller/measured_wrench",
            ).value
        )
        self.status_subscription = self.create_subscription(
            TaskStatus, self.status_topic, self.on_chassis_status, 20
        )
        self.target_publisher = self.create_publisher(PoseArray, self.target_topic, 10)
        self.compliant_target_publisher = self.create_publisher(
            PoseArray, self.compliant_target_topic, 10
        )
        self.compliant_wrench_subscription = self.create_subscription(
            WrenchStamped,
            self.compliant_wrench_topic,
            self.on_compliant_wrench,
            qos_profile_sensor_data,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.control_client = self.create_client(TaskControl, self.control_service)
        self.use_velocity_mux = bool(
            self.declare_parameter("use_velocity_mux", False).value
        )
        self.stop_pid_before_follow = bool(
            self.declare_parameter("stop_pid_before_follow", True).value
        )
        self.mux_run_ready = not self.use_velocity_mux
        self.mux_enable_client = self.create_client(
            SetBool, "/chassis_velocity_mux/enable"
        )
        self.mux_mode_client = self.create_client(
            SetBool, "/chassis_velocity_mux/line_follow_mode"
        )
        self.follow_enable_client = self.create_client(
            SetBool, "/line_follow_controller/enable"
        )
        self.follow_parameters = AsyncParameterClient(self, '/line_follow_controller')
        self.route_checkpoint_client = self.create_client(Trigger, '/line_follow_controller/save_route')
        self.route_checkpoint_file = None
        self.follow_resume_publisher = self.create_publisher(Empty, '/line_follow/resume', 10)
        self.follow_resume_sent = float('-inf')
        self.require_image_receiver = bool(
            self.declare_parameter("require_image_receiver", False).value
        )
        self.image_ready_client = self.create_client(
            Trigger, "/hk_image_transfer/ready"
        )
        self.capture_client = self.create_client(Trigger, "/dataset_capture/capture")
        self.line_stream_client = self.create_client(SetBool, "/line_camera/camera_node/streaming")
        self.follow_status = None
        self.follow_status_time = None
        self.wheel_odom = None
        self.wheel_odom_received = None
        self.stationary_since = None
        odom_topic = str(self.declare_parameter("chassis_odom_topic", "/wheel_odom").value)
        if any(step['action'] == 'wait_chassis_stopped' for step in self.steps):
            self.create_subscription(Odometry, odom_topic, self.on_wheel_odom, 10)
        self.create_subscription(
            String, "/line_follow/status", self.on_follow_status, 10
        )
        self.execute_client = self.create_client(Trigger, self.execute_service)
        self.admittance_enable_client = self.create_client(
            SetBool, "/admittance_controller/enable"
        )
        self.require_arm_ready = bool(
            self.declare_parameter("require_arm_ready", False).value
        )
        self.robot_task_running = False
        self.arm_controllers_active = False
        self.arm_controllers_checked = float("-inf")
        self.arm_check_requested = float("-inf")
        self.arm_check_future = None
        self.arm_wait_logged = False
        self.arm_controllers_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.robot_task_subscription = self.create_subscription(
            Bool,
            "/io_and_status_controller/robot_task_running",
            self.on_robot_task_running,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.check_devices_on_startup = bool(
            self.declare_parameter("check_devices_on_startup", False).value
        )
        self.device_check_timeout = float(
            self.declare_parameter("device_check_timeout_sec", 120.0).value
        )
        self.device_ready_stable_sec = float(
            self.declare_parameter("device_ready_stable_sec", 1.0).value
        )
        if (
            not math.isfinite(self.device_check_timeout)
            or self.device_check_timeout <= 0
            or not math.isfinite(self.device_ready_stable_sec)
            or self.device_ready_stable_sec < 0
        ):
            raise ValueError(
                "device check timing parameters must be finite; timeout > 0 and stable duration >= 0"
            )
        self.check_arm_devices = any(
            step["action"] in ("arm_move", "arm_poses", "arm_posture") for step in self.steps
        )
        self.check_posture_devices = any(step['action'] == 'arm_posture' for step in self.steps)
        self.check_admittance_devices = any(
            step["action"] == "arm_move" and step["control_mode"] == "admittance"
            for step in self.steps
        )
        if self.check_admittance_devices and not self.manage_admittance:
            raise ValueError('包含导纳动作的任务必须启用manage_admittance及对应节点')
        self.check_moveit_devices = any(
            step["action"] == "arm_poses"
            or (step["action"] == "arm_move" and step["control_mode"] == "moveit")
            for step in self.steps
        )
        self.check_line_devices = any(
            step["action"] == "line_follow" for step in self.steps
        )
        self.follow_ready_client = self.create_client(
            Trigger, "/line_follow_controller/ready"
        )
        self.reset_device_checks()
        self.start_server = self.create_service(
            Trigger, "~/start", self.on_start_request
        )

        self.stop_server = self.create_service(Trigger, "~/stop", self.on_stop_request)
        self.capture_server = self.create_service(Trigger, "~/capture", self.on_capture_request)
        self.safety_inhibited = False
        self.safety_checkpoint = None
        self.recovery_arm_target = None
        self.recovery_hold_step = None
        self.create_subscription(Bool, "/robot_control/safety_inhibit", self.on_safety_inhibit,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, "~/safety_pause", self.on_safety_pause)
        self.create_service(Trigger, "~/safety_resume", self.on_safety_resume)
        self.generation = 0
        self.run_id = uuid4().hex
        self.route_checkpoint_file = None
        self.execution_ids = {}
        self.cancelled_execution_ids = set()
        self.step_started = monotonic()

        self.active = self.auto_start
        self.failed = False
        self.last_failure = ""
        self.step_index = 0
        self.phase = "ready"
        self.deadline_ns = 0
        self.seen_running = False
        self.running_task_ids = set()
        self.latest_chassis_status = {}
        self.last_compliant_wrench_ns = None
        self.waiting_for_wrench_logged = False
        self.arm_control_mode = None
        self.timer = self.create_timer(0.1, self.advance)
        self.workflow_status_pub = self.create_publisher(
            String, "~/status", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.create_timer(.5, self.publish_workflow_status)
        self.get_logger().info(
            f"Loaded workflow '{self.workflow_name}' with {len(self.steps)} steps "
            f"from {workflow_file}"
        )
        if self.auto_start:
            self.get_logger().info("Workflow will start automatically")
        else:
            self.get_logger().info("Call /task_controller/start to begin")

    def publish_workflow_status(self):
        """发布任务状态快照，供工控机监听；不会触发任务。"""
        step = self.current_step()
        self.workflow_status_pub.publish(String(data=json.dumps({
            "schema": "robot_workflow_status_v1",
            "workflow": "camera_capture_only" if self.capture_only else self.workflow_name,
            "capture_only": self.capture_only,
            "run_id": getattr(self, "run_id", ""), "active": self.active,
            "failed": self.failed, "failure_reason": self.last_failure,
            "safety_inhibited": self.safety_inhibited,
            "safety_checkpoint": self.safety_checkpoint is not None,
            "phase": self.phase,
            "step_index": int(self.phase == "capture_complete") if self.capture_only else self.step_index,
            "step_count": 1 if self.capture_only else len(self.steps),
            "action": step["action"] if step else None,
            "follow_state": self.follow_status,
            "posture": step.get('posture') if step else None,
            "drive_direction": step.get('drive_direction') if step else None,
            "devices_ready": self.devices_checked,
            "wait_seconds_remaining": (max(0., (self.deadline_ns-self.get_clock().now().nanoseconds)/1e9)
                                       if self.phase == 'waiting_delay' else None),
            "route_checkpoint_file": self.route_checkpoint_file,
            "stamp_ns": self.get_clock().now().nanoseconds,
        }, ensure_ascii=False)))

    @staticmethod
    def load_workflow(path):
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(document, dict) or not isinstance(
            document.get("workflow"), dict
        ):
            raise ValueError("workflow YAML must contain a 'workflow' mapping")
        workflow = document["workflow"]
        steps = workflow.get("steps")
        if not isinstance(steps, list) or not steps:
            raise ValueError("workflow.steps must be a non-empty list")
        # YAML锚点共享站点动作，展开后按普通步骤执行，不复制维护点位。
        def flatten(items):
            return [deepcopy(step) for item in items
                    for step in (flatten(item) if isinstance(item, list) else [item])]
        steps = flatten(steps)
        station_steps = flatten(workflow.get('station_steps', []))
        workflow['station_steps'] = station_steps
        if workflow.get('arm_postures_before_chassis', False):
            steps = [part for step in steps for part in (
                [{'action': 'arm_posture', 'posture': 'travel', 'step_timeout_sec': 600.}, step]
                if step.get('action') in ('line_follow', 'chassis_execute') else [step])]
        workflow['steps'] = steps
        for index, step in enumerate(steps+station_steps):
            if not isinstance(step, dict):
                raise ValueError(f"workflow step {index} must be a mapping")
            action = step.get("action")
            if action not in SUPPORTED_ACTIONS:
                raise ValueError(
                    f"workflow step {index} has unsupported action: {action}"
                )
            if action in ("chassis_execute", "wait_chassis") and not step.get("task"):
                raise ValueError(f"workflow step {index} requires task")
            timeout = float(step.get("step_timeout_sec", 90.0))
            if not math.isfinite(timeout) or timeout <= 0.0:
                raise ValueError(f"workflow step {index} has invalid step_timeout_sec")
            seconds = float(step.get("seconds", -1.0)) if action == "delay" else 0.0
            if action == "delay" and (not math.isfinite(seconds) or seconds < 0.0):
                raise ValueError(f"workflow step {index} has invalid seconds")
            if action == "final_capture" and "config_file" in step:
                if not isinstance(step["config_file"], str) or not step["config_file"].strip():
                    raise ValueError(f"workflow step {index} requires a valid config_file path")
            if action == 'arm_posture' and step.get('posture') not in ('travel', 'detection_ready'):
                raise ValueError('arm_posture requires travel or detection_ready')
            if action == 'line_follow' and step.get('service_intermediate_stations', False):
                if not station_steps or not workflow.get('arm_postures_before_chassis', False):
                    raise ValueError('站点动作需要station_steps和行驶姿态保护')
                if step.get('skip_intermediate_markers', False):
                    raise ValueError('站点执行与自动跳站不能同时启用')
            if action == 'line_follow' and type(step.get('finish_at_first_marker', False)) is not bool:
                raise ValueError('finish_at_first_marker必须是bool')
            if action == 'line_follow' and step.get('finish_at_first_marker', False):
                if step.get('drive_direction') != 'forward' or step.get('service_intermediate_stations', False):
                    raise ValueError('首条横线到站仅支持显式forward，且不能同时插入逐站动作')
            if action == "line_follow":
                if 'drive_direction' in step and step['drive_direction'] not in ('forward', 'reverse'):
                    raise ValueError(f'workflow step {index} drive_direction must be forward or reverse')
                if type(step.get('skip_intermediate_markers', False)) is not bool:
                    raise ValueError(f'workflow step {index} skip_intermediate_markers must be boolean')
                if type(step.get('return_to_start', False)) is not bool:
                    raise ValueError(f'workflow step {index} return_to_start must be boolean')
                until_untrusted = step.get("until_untrusted", False)
                if not isinstance(until_untrusted, bool):
                    raise ValueError(f"workflow step {index} until_untrusted must be boolean")
                if not isinstance(step.get("require_final_marker", False), bool):
                    raise ValueError(f"workflow step {index} require_final_marker must be boolean")
                if step.get("require_final_marker", False) and not until_untrusted:
                    raise ValueError(f"workflow step {index} final marker mode requires until_untrusted")
                boundary_wait = float(step.get("boundary_wait_sec", 0.0))
                if not math.isfinite(boundary_wait) or not 0 <= boundary_wait <= 30:
                    raise ValueError(f"workflow step {index} boundary_wait_sec must be 0..30")
                progress_timeout = float(step.get("progress_timeout_sec", 3.0))
                if not math.isfinite(progress_timeout) or progress_timeout <= 0.0:
                    raise ValueError(
                        f"workflow step {index} has invalid progress_timeout_sec"
                    )
                duration = float(step.get("seconds", -1.0))
                if not until_untrusted and (not math.isfinite(duration) or duration <= 0.0):
                    raise ValueError(
                        f"workflow step {index} line_follow.seconds must be positive"
                    )
            if action == "arm_poses":
                WorkflowController.validate_pose_values(step.get("poses"), index)
            if action == "wait_chassis_stopped":
                for name, default in (("stable_seconds", 1.0), ("linear_tolerance", 0.005),
                                      ("angular_tolerance", 0.02), ("data_timeout_sec", 0.5)):
                    value = float(step.get(name, default))
                    if not math.isfinite(value) or value <= 0:
                        raise ValueError(f"workflow step {index} invalid {name}")
            if action == "arm_move":
                WorkflowController.validate_arm_move(step, index)
        return workflow

    def load_task_table(self, workflow):
        table_name = workflow.get("task_table")
        if not table_name:
            return {}
        table_path = Path(str(table_name))
        if not table_path.is_absolute():
            table_path = self.workflow_path.parent / table_path
        document = yaml.safe_load(table_path.read_text(encoding="utf-8"))
        if not isinstance(document, dict) or not isinstance(
            document.get("tasks"), dict
        ):
            raise ValueError("task table YAML must contain a 'tasks' mapping")
        tasks = document["tasks"]
        for task_name, task_config in tasks.items():
            if not isinstance(task_name, str) or not task_name:
                raise ValueError("task names must be non-empty strings")
            if task_config is not None and not isinstance(task_config, dict):
                raise ValueError(f"task '{task_name}' config must be a mapping")
        self.get_logger().info(
            f"Loaded {len(tasks)} callable chassis task(s) from {table_path}"
        )
        return tasks

    def validate_task_references(self):
        for index, step in enumerate(self.steps):
            task_name = step.get("task")
            if task_name is not None and task_name not in self.tasks:
                raise ValueError(
                    f"workflow step {index} references unknown task: {task_name}"
                )

    @staticmethod
    def resolve_task(step):
        return str(step.get("task", ""))

    @staticmethod
    def validate_pose_values(poses, index):
        if not isinstance(poses, list) or not poses:
            raise ValueError(f"workflow step {index} requires poses")
        for pose in poses:
            if not isinstance(pose, list) or len(pose) != 6:
                raise ValueError(
                    f"each pose in workflow step {index} must contain 6 numbers"
                )
            if not all(math.isfinite(float(value)) for value in pose):
                raise ValueError(f"workflow step {index} contains invalid pose data")

    @staticmethod
    def validate_arm_move(step, index):
        if not isinstance(step.get('skip_if_at_target', False), bool):
            raise ValueError(f'workflow step {index} skip_if_at_target must be boolean')
        degrees = float(step.get('arrival_orientation_tolerance_deg', 1.0))
        if not math.isfinite(degrees) or not 0 < degrees <= 180:
            raise ValueError(f'workflow step {index} invalid arrival_orientation_tolerance_deg')
        if step.get("control_mode") not in ("moveit", "admittance"):
            raise ValueError(
                f"workflow step {index} control_mode must be moveit or admittance"
            )
        target = step.get("target_pose")
        if not isinstance(target, dict):
            raise ValueError(f"workflow step {index} requires target_pose")
        if target.get("position_mode", "absolute") not in ("absolute", "relative"):
            raise ValueError(
                f"workflow step {index} target_pose.position_mode must be absolute or relative"
            )
        position = target.get("position")
        if (
            not isinstance(position, list)
            or len(position) != 3
            or not all(math.isfinite(float(value)) for value in position)
        ):
            raise ValueError(
                f"workflow step {index} target_pose.position needs [x, y, z]"
            )
        orientation = target.get("orientation")
        if orientation != "current":
            if (
                not isinstance(orientation, list)
                or len(orientation) != 3
                or not all(math.isfinite(float(value)) for value in orientation)
            ):
                raise ValueError(
                    f"workflow step {index} target_pose.orientation needs "
                    "current or [roll, pitch, yaw] in degrees"
                )
        expected = step.get("expected_start_position")
        if expected is not None and (
            not isinstance(expected, list)
            or len(expected) != 3
            or not all(math.isfinite(float(value)) for value in expected)
        ):
            raise ValueError(
                f"workflow step {index} expected_start_position must contain "
                "three finite values"
            )
        if (
            float(step.get("start_position_tolerance", 0.10)) <= 0.0
            or float(step.get("max_position_distance", 0.10)) <= 0.0
            or float(step.get("arrival_tolerance", 0.01)) <= 0.0
            or float(step.get("motion_timeout_sec", 20.0)) <= 0.0
            or float(step.get("progress_timeout_sec", 5.0)) <= 0.0
        ):
            raise ValueError(
                f"workflow step {index} arm position limits and timeouts "
                "must be positive"
            )
        delay = float(step.get("after_arrival_delay_sec", 0.0))
        if not math.isfinite(delay) or delay < 0.0:
            raise ValueError(
                f"workflow step {index} after_arrival_delay_sec "
                "must be finite and non-negative"
            )

    def on_safety_inhibit(self, message):
        self.safety_inhibited = message.data
        if message.data:
            self.pause_for_safety()

    def pause_for_safety(self):
        """保存步骤和已解析的绝对目标；旧异步回调通过 generation 作废。"""
        if self.safety_checkpoint is not None or self.capture_only:
            return
        step = self.current_step()
        holding = not self.active and self.arm_control_mode == "admittance"
        if not self.active and not holding:
            return
        target = None
        if (step and step["action"] == "arm_move" and self.phase != "ready"
                and getattr(self, "pending_arm_target", None) is not None):
            target = deepcopy(self.pending_arm_target)
        self.safety_checkpoint = dict(step_index=self.step_index, step=deepcopy(step),
                                      target=target, holding=holding, run_id=self.run_id)
        self.fail_workflow("机器人安全暂停；等待人工确认恢复")
        self.phase = "safety_paused"
        # 仅保存诊断摘要；进程重启后不自动恢复未完成运动。
        checkpoint = Path.home()/".ros/robot_control/workflow_safety_checkpoint.json"
        try:
            checkpoint.parent.mkdir(parents=True, exist_ok=True)
            data = dict(workflow=str(self.workflow_path), run_id=self.run_id,
                        step_index=self.step_index, step=step, holding=holding)
            if target is not None:
                from rosidl_runtime_py.convert import message_to_ordereddict
                data['resolved_target'] = message_to_ordereddict(target)
            temporary = checkpoint.with_suffix('.tmp')
            temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2))
            temporary.replace(checkpoint)
        except OSError as error:
            self.get_logger().error(f"无法保存安全暂停诊断文件：{error}")

    def on_safety_pause(self, request, response):
        self.pause_for_safety()
        response.success = True
        response.message = "Task paused; checkpoint retained" if self.safety_checkpoint else "No arm task to resume"
        return response

    def on_safety_resume(self, request, response):
        checkpoint = self.safety_checkpoint
        if self.safety_inhibited or checkpoint is None:
            response.success = checkpoint is None and not self.safety_inhibited
            response.message = "No checkpoint" if checkpoint is None else "Safety inhibit is still active"
            return response
        if self.workflow.get('arm_postures_before_chassis', False):
            response.success = False
            response.message = '联动姿态任务已中止锁定；完成安全恢复后重新启动联动launch，不原地续跑'
            return response
        step = checkpoint['step']
        if not checkpoint['holding'] and (step is None or step['action'] not in ('arm_move', 'delay')):
            response.success = False
            response.message = "This step needs an explicit task restart; checkpoint retained (no side effects replayed)"
            return response
        self.generation += 1
        self.step_index = checkpoint['step_index']
        self.recovery_arm_target = deepcopy(checkpoint['target'])
        if checkpoint['holding']:
            # 独立柔顺任务早已完成时，只在当前姿态重新进入柔顺，不回报警前的姿态。
            self.recovery_hold_step = dict(action='arm_move', control_mode='admittance',
                frame_id='base_link', end_effector_frame='tool0', max_position_distance=.01,
                target_pose=dict(position_mode='relative', position=[0., 0., 0.], orientation='current'))
            self.recovery_arm_target = None
        self.safety_checkpoint = None
        self.failed, self.active, self.phase = False, True, 'ready'
        self.last_failure = ''
        self.step_started = monotonic()
        self.mux_run_ready = not self.use_velocity_mux
        self.reset_device_checks()
        response.success = True
        response.message = "Resuming saved arm step with fresh state; old trajectories discarded"
        return response

    def current_step(self):
        if self.recovery_hold_step is not None:
            return self.recovery_hold_step
        if self.capture_only:
            return None if self.phase == "capture_complete" else {
                "action": "final_capture", "step_timeout_sec": 300.,
                **({"config_file": self.capture_config_file} if self.capture_config_file else {}),
            }
        if self.step_index >= len(self.steps):
            return None
        return self.steps[self.step_index]

    def on_start_request(self, request, response):
        if self.safety_inhibited or self.safety_checkpoint is not None:
            response.success = False
            response.message = "Safety pause is latched; use /protective_stop_guard/confirm_recovery"
            return response
        if self.workflow.get("camera_only", False):
            return self.on_capture_request(request, response)
        del request
        if self.capture_process is not None and self.capture_process.poll() is None:
            response.success = False
            response.message = "previous camera capture is still stopping"
            return response
        if self.active and self.station_context is not None:
            self.station_resume_requested = True
            response.success, response.message = True, '已收到继续请求；本站动作完成并到行驶姿态后继续循迹'
            return response
        if self.active:
            response.success = False
            response.message = "workflow is already running"
            return response
        if self.capture_only:
            self.capture_only = False
            self.step_index = 0
        if self.step_index >= len(self.steps) and not self.failed:
            self.step_index = 0
        if self.failed:
            self.step_index = 0
            self.failed = False
        self.steps = deepcopy(self.original_steps)
        self.station_context = None
        self.station_resume_requested = False
        self.station_rearmed = True
        self.generation += 1
        self.run_id = uuid4().hex
        self.route_checkpoint_file = None
        self.last_failure = ""
        self.mux_run_ready = not self.use_velocity_mux
        self.execution_ids.clear()
        self.cancelled_execution_ids.clear()
        self.running_task_ids.clear()
        self.latest_chassis_status.clear()
        self.step_started = monotonic()
        self.phase = "ready"
        self.active = True
        self.reset_device_checks()
        if self.line_stream_client.service_is_ready():
            resume = SetBool.Request()
            resume.data = True
            self.line_stream_client.call_async(resume)
        response.success = True
        response.message = f"workflow '{self.workflow_name}' started"
        self.get_logger().info(response.message)
        return response

    def on_capture_request(self, request, response):
        del request
        if self.active or (self.capture_process is not None and self.capture_process.poll() is None):
            response.success = False
            response.message = "任务或相机采集正在运行，不能并发单独采集"
            return response
        self.capture_only = True
        self.active, self.failed = True, False
        self.generation += 1
        self.run_id = uuid4().hex
        self.route_checkpoint_file = None
        self.last_failure = ""
        self.step_started = monotonic()
        self.phase = "ready"
        response.success = True
        response.message = "camera capture accepted; run_id="+self.run_id
        self.get_logger().info(response.message)
        return response

    def on_stop_request(self, request, response):
        del request
        capture_only = self.capture_only
        self.safety_checkpoint = None
        self.recovery_arm_target = self.recovery_hold_step = None
        self.fail_workflow("stopped by user")
        response.success = True
        if capture_only:
            response.message = "Camera capture stop requested; motion drivers unchanged"
            return response
        response.message = (
            "Stop requested for chassis and admittance; see controller logs"
        )
        return response

    def add_callback(self, future, callback):
        generation = self.generation
        index = self.step_index

        def guarded(result):
            if (
                self.active
                and generation == self.generation
                and index == self.step_index
            ):
                callback(result)

        future.add_done_callback(guarded)

    def on_follow_status(self, message):
        try:
            self.follow_status = json.loads(message.data).get("state")
            self.follow_status_time = monotonic()
        except (ValueError, AttributeError):
            self.follow_status = None

    def request_bool(self, client, value, callback):
        request = SetBool.Request()
        request.data = value
        self.add_callback(client.call_async(request), callback)

    def checked_bool(self, future, next_phase):
        try:
            response = future.result()
            if not response.success:
                self.fail_workflow(f"control mode rejected: {response.message}")
                return
        except Exception as exc:
            self.fail_workflow(f"control mode failed: {exc}")
            return
        self.phase = next_phase

    def on_mux_run_ready(self, future):
        self.checked_bool(future, "ready")
        if not self.failed:
            self.mux_run_ready = True

    def advance_final_capture(self, step):
        if self.phase == "ready":
            from ament_index_python.packages import get_package_share_directory
            config = Path(step.get("config_file") or (
                Path(get_package_share_directory("robot_task_control")) / "config/capture/final_capture.yaml")).expanduser()
            if not config.is_absolute():
                config = self.workflow_path.parent / config
            # 相机仅在这个步骤启动；重编码、保存和TCP不阻塞控制器ROS回调。
            try:
                self.capture_process = self.launch_capture_process(
                    [sys.executable, "-m", "robot_task_control.final_capture",
                     "--config", str(config.expanduser()), "--run-id", self.run_id,
                     *(["--allow-no-line-camera"] if self.capture_only else [])],
                )
                self.phase = "final_capture_running"
            except OSError as error:
                self.fail_workflow(f"Cannot start final capture: {error}")
        elif self.phase == "final_capture_running":
            code = self.capture_process.poll()
            if code is None:
                return
            self.capture_process = None
            if code == 0:
                self.get_logger().info("HK和RGB-D已采集并完成TCP发送")
                self.complete_step()
            else:
                self.fail_workflow(f"任务末尾拍照/发送失败，退出码{code}；本地数据保留，查看采集日志和status.json")

    def launch_capture_process(self, command):
        return subprocess.Popen(command, start_new_session=True)

    def stop_capture_process(self, wait=False):
        process = self.capture_process
        if process is None:
            return
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            if wait:
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=3)

    def destroy_node(self):
        self.stop_capture_process(wait=True)
        return super().destroy_node()

    def advance_hk_capture(self):
        if self.phase == "ready" and self.require_image_receiver:
            if not self.image_ready_client.service_is_ready():
                return
            self.phase = "image_receiver_pending"
            self.add_callback(
                self.image_ready_client.call_async(Trigger.Request()),
                lambda result: self.checked_bool(result, "capture_ready"),
            )
            return
        if (
            self.phase in ("ready", "capture_ready")
            and self.capture_client.service_is_ready()
        ):
            self.phase = "capture_pending"
            self.add_callback(
                self.capture_client.call_async(Trigger.Request()),
                self.on_capture_response,
            )

    def on_capture_response(self, future):
        try:
            response = future.result()
            if not response.success:
                self.fail_workflow(f"HK capture failed: {response.message}")
                return
            self.get_logger().info(f"HK capture completed: {response.message}")
            self.complete_step()
        except Exception as exc:
            self.fail_workflow(f"HK capture service failed: {exc}")

    def advance_line_follow(self, step):
        if not self.use_velocity_mux:
            self.fail_workflow("line_follow requires use_velocity_mux=true")
            return
        if self.phase == 'ready' and 'drive_direction' in step:
            if not self.follow_enable_client.service_is_ready():
                return
            self.phase = 'follow_direction_disable_pending'
            self.request_bool(self.follow_enable_client, False,
                              lambda result: self.checked_bool(result, 'follow_direction_set'))
            return
        if self.phase == 'follow_direction_set':
            if not self.follow_parameters.services_are_ready():
                return
            self.phase = 'follow_direction_pending'
            self.add_callback(self.follow_parameters.set_parameters([
                Parameter('drive_direction', value=step['drive_direction']),
                Parameter('reverse_return_to_start', value=step.get('return_to_start', False))]), self.on_follow_direction)
            return
        if self.phase in ("ready", 'follow_prepare'):
            if not (
                self.follow_enable_client.service_is_ready()
                and self.mux_mode_client.service_is_ready()
            ):
                return
            self.follow_status = None
            self.follow_status_time = None
            self.follow_tracking_seconds = 0.0
            self.follow_last_tick = monotonic()
            self.follow_last_tracking = monotonic()
            self.follow_boundary_since = None
            self.follow_station_confirm_since = None
            if not self.stop_pid_before_follow:
                self.phase = "follow_select"
                return
            if not self.control_client.service_is_ready():
                return
            self.cancelled_execution_ids.update(self.execution_ids.values())
            request = TaskControl.Request()
            request.command = TaskControl.Request.STOP
            self.phase = "follow_pid_stop_pending"
            self.add_callback(
                self.control_client.call_async(request), self.on_pid_stopped_for_follow
            )
        elif self.phase == "follow_select":
            self.phase = "follow_select_pending"
            self.request_bool(
                self.mux_mode_client,
                True,
                lambda result: self.checked_bool(result, "follow_enable"),
            )
        elif self.phase == "follow_enable":
            if (self.workflow.get('arm_postures_before_chassis', False)
                    or self.workflow.get('lock_chassis_between_steps', False)):
                self.phase = "follow_output_pending"
                self.request_bool(self.mux_enable_client, True,
                                  lambda result: self.checked_bool(result, 'follow_enable_controller'))
            else:
                self.phase = 'follow_enable_pending'
                self.request_bool(self.follow_enable_client, True,
                                  lambda result: self.checked_bool(result, 'follow_tracking'))
        elif self.phase == 'follow_enable_controller':
            self.phase = 'follow_enable_pending'
            self.request_bool(self.follow_enable_client, True,
                              lambda result: self.checked_bool(result, 'follow_tracking'))
        elif self.phase == "follow_tracking":
            now = monotonic()
            dt = min(now - self.follow_last_tick, 0.2)
            self.follow_last_tick = now
            fresh = (
                self.follow_status_time is not None
                and now - self.follow_status_time <= 1.0
            )
            if fresh and self.follow_status and self.follow_status.startswith('start_alignment_failed'):
                self.fail_workflow('起步横线对齐失败，底盘保持停车：'+self.follow_status)
                return
            if fresh and self.follow_status and self.follow_status.startswith('standard_'):
                self.fail_workflow('标准路线重放停止，底盘保持停车：'+self.follow_status)
                return
            if fresh and self.follow_status and self.follow_status.startswith('start_alignment_'):
                self.follow_last_tracking = now
                self.follow_boundary_since = None
                self.follow_station_confirm_since = None
                return  # 对齐期间只走受限微调，不计入循迹进度超时。
            if fresh and self.follow_status and self.follow_status.startswith('charging_dock_'):
                if self.follow_status == 'charging_dock_confirming':
                    self.follow_last_tracking = now
                    return
                self.fail_workflow('倒车充电停止：'+self.follow_status)
                return
            if fresh and self.follow_status == "reverse_finish_goal_not_centered":
                self.fail_workflow(
                    "倒车终点横线已到，但横向或朝向尚未达标；底盘保持停车，返程未完成，不执行后续步骤"
                )
                return  # 必须早于计时结束判断，不能用运行时长替代最终居中。
            if (fresh and self.follow_status == "stop_marker_intermediate"
                    and not step.get('finish_at_first_marker', False)):
                if step.get('service_intermediate_stations', False) and self.station_rearmed:
                    self.begin_station(step)
                    return
                if step.get('skip_intermediate_markers', False) and now-self.follow_resume_sent >= 1.:
                    self.follow_resume_publisher.publish(Empty())
                    self.follow_resume_sent = now
                    self.get_logger().info('返程途中站自动继续，仅在原出发点结束倒车')
                self.follow_last_tracking = now
                self.follow_boundary_since = None
                self.follow_station_confirm_since = None
                return  # 等待 /line_follow/resume，新消息到达前既不完成也不累计行走时间。
            if fresh and self.follow_status in (
                    "stop_marker_settling", "stop_marker_confirming", "stop_marker_averaging", "stop_marker_overshoot",
                    "stop_marker_missing", "stop_marker_unknown"):
                if self.follow_station_confirm_since is None:
                    self.follow_station_confirm_since = now
                if now-self.follow_station_confirm_since > 5.0:
                    self.fail_workflow("停止线位置在5秒内未稳定到停车容差内；保持停车，请检查位置误差")
                    return
                self.follow_last_tracking = now
                self.follow_boundary_since = None
                return  # 已输出零速，停车确认使用独立期限，不触发普通行走进度超时。
            self.follow_station_confirm_since = None
            if fresh and self.follow_status == "reverse_visual_finish_aligning":
                self.follow_last_tracking = now
                self.follow_boundary_since = None
                return  # 纵向已停，视觉仍在横移/摆正；不计行走时长，也不误报完成。
            if fresh and self.follow_status in ("tracking", "stop_marker_approaching", "stop_marker_missing_approaching", "reverse_route_tracking", "reverse_visual_finish_tracking"):
                self.station_rearmed = True
                self.follow_tracking_seconds += dt
                self.follow_last_tracking = now
                if self.follow_boundary_since is not None:
                    self.get_logger().info("视觉恢复有效，继续本轮循迹")
                self.follow_boundary_since = None
            until_untrusted = step.get("until_untrusted", False)
            boundary = (until_untrusted and self.follow_tracking_seconds > 0.0 and fresh
                        and self.follow_status in {
                            "line_lost", "insufficient_rows", "no_forward_line",
                            "insufficient_forward_span", "near_line_missing", "empty_centerline",
                            "heading_out_of_range", "horizontal_or_unknown_line",
                        })
            if boundary and self.follow_boundary_since is None:
                self.follow_boundary_since = now
                self.get_logger().info(
                    f"视觉暂不可行，底盘已停车；等待最多 {step.get('boundary_wait_sec', 0.0)} 秒恢复："
                    f"{self.follow_status}")
            if not boundary and self.follow_status not in ("tracking", "reacquiring"):
                self.follow_boundary_since = None
            waiting_at_boundary = (self.follow_boundary_since is not None and fresh
                                   and (boundary or self.follow_status == "reacquiring"))
            boundary_confirmed = (waiting_at_boundary and now - self.follow_boundary_since >=
                                  float(step.get("boundary_wait_sec", 0.0)))
            timed_finish = (not until_untrusted and
                            self.follow_tracking_seconds >= float(step["seconds"]))
            route_complete = fresh and self.follow_status == "reverse_route_complete"
            final_marker = fresh and (self.follow_status == "stop_marker_final" or
                step.get('finish_at_first_marker', False) and self.follow_status == 'stop_marker_intermediate')
            if boundary_confirmed and step.get("require_final_marker", False) and not (final_marker or route_complete):
                self.fail_workflow("目标停止横线尚未确认，主线持续不可用；不会执行后续机械臂点位")
                return
            if boundary_confirmed or timed_finish or final_marker or route_complete:
                if route_complete:
                    self.get_logger().info("已返回记录路线起点，结束倒车并等待底盘停稳")
                if final_marker:
                    self.get_logger().info("目标停止横线已到位停车，结束循迹并等待底盘停稳")
                if boundary_confirmed:
                    self.get_logger().info(f"可信实测段不足，结束本次循迹：{self.follow_status}")
                self.phase = "follow_disable_pending"
                self.request_bool(
                    self.follow_enable_client,
                    False,
                    lambda result: self.checked_bool(result, "follow_restore_pid"),
                )
            elif not waiting_at_boundary and now - self.follow_last_tracking > float(
                step.get("progress_timeout_sec", 3.0)
            ):
                self.fail_workflow(f"line following unavailable: {self.follow_status}")
        elif self.phase == "follow_restore_pid":
            self.phase = "follow_restore_pending"
            self.request_bool(self.mux_mode_client, False, self.on_follow_finished)

    def on_pid_stopped_for_follow(self, future):
        self.checked_bool(future, "follow_select")

    def on_follow_direction(self, future):
        try:
            results = future.result().results
            if not results or not all(result.successful for result in results):
                raise RuntimeError('; '.join(result.reason for result in results))
        except Exception as error:
            self.fail_workflow('循迹方向切换失败：'+str(error))
            return
        self.get_logger().info('循迹方向已切换为 '+self.current_step()['drive_direction'])
        self.phase = 'follow_prepare'

    def on_route_checkpoint(self, future):
        try:
            result = future.result()
            if not result.success:
                raise RuntimeError(result.message)
            self.route_checkpoint_file = json.loads(result.message)['snapshot_file']
            if self.standard_output:
                from .standard_route import publish_standard
                saved = publish_standard(self.route_checkpoint_file, self.standard_output, self.run_id)
                self.get_logger().info('标准路线已保存：'+saved['standard_file']+'；版本：'+saved['version_dir'])
        except Exception as error:
            self.fail_workflow('返程路线封存失败：'+str(error))
            return
        self.get_logger().info('后续倒车使用封存路线：'+self.route_checkpoint_file)
        self.complete_step()

    def on_follow_finished(self, future):
        self.checked_bool(future, "ready")
        if not self.failed:
            if (self.workflow.get('arm_postures_before_chassis', False)
                    or self.workflow.get('lock_chassis_between_steps', False)):
                self.phase = 'follow_finish_mux_pending'
                self.request_bool(self.mux_enable_client, False, self.on_final_mux_stopped)
            else:
                self.complete_step()

    def on_final_mux_stopped(self, future):
        self.checked_bool(future, 'ready')
        if not self.failed:
            self.complete_step()

    def on_posture_status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                return
            self.posture_status, self.posture_status_time = status, monotonic()
        except (ValueError, TypeError):
            pass

    def advance_arm_posture(self, posture, next_phase=None):
        """请求仅代表排队；匹配本次ID且实际关节到位的DONE才可放行。"""
        if self.phase in ('ready', 'station_travel'):
            client = self.posture_clients[posture]
            if not client.service_is_ready():
                return
            self.posture_request_id = None
            self.phase = 'posture_accept_pending'
            self.add_callback(client.call_async(Trigger.Request()), self.on_posture_accepted)
        elif self.phase == 'posture_wait':
            status = self.posture_status or {}
            if self.posture_status_time is None or monotonic()-self.posture_status_time > 3.:
                self.fail_workflow('关节姿态服务状态失联，底盘不放行')
                return
            if status.get('request_id') != self.posture_request_id:
                return
            if status.get('state') == 'FAILED':
                self.fail_workflow('关节姿态失败：'+str(status.get('error')))
            elif status.get('state') == 'DONE' and status.get('posture') == posture:
                self.arm_control_mode = 'moveit'
                if next_phase is None:
                    self.complete_step()
                else:
                    self.phase = next_phase

    def on_posture_accepted(self, future):
        try:
            result = future.result()
            if not result.success:
                raise RuntimeError(result.message)
            self.posture_request_id = json.loads(result.message)['request_id']
            if not self.posture_request_id:
                raise ValueError('空request_id')
            self.phase = 'posture_wait'
        except Exception as error:
            self.fail_workflow('关节姿态请求失败：'+str(error))

    def on_station_resume(self, message):
        del message
        if self.active and self.station_context is not None:
            self.station_resume_requested = True

    def begin_station(self, step):
        """仅关闭底盘输出，不关闭循迹节点，前进路线/里程计记录保持连续。"""
        if not self.mux_enable_client.service_is_ready():
            self.fail_workflow('站点无法关闭底盘速度输出')
            return
        self.phase = 'station_mux_stop_pending'
        self.request_bool(self.mux_enable_client, False, lambda future: self.on_station_stopped(future, step))

    def on_station_stopped(self, future, step):
        self.checked_bool(future, 'ready')
        if self.failed:
            return
        self.station_context = {'step': deepcopy(step), 'tracking_seconds': self.follow_tracking_seconds}
        self.station_resume_requested = False
        self.station_rearmed = False
        station = deepcopy(self.workflow['station_steps'])
        self.steps[self.step_index:self.step_index+1] = station+[{'action': 'continue_line_follow'}]
        self.step_started = monotonic()
        self.get_logger().info('途中站：底盘输出已锁定，执行本站机械臂动作，完成后等待开始消息')

    def advance_station_continue(self):
        if self.station_context is None:
            self.fail_workflow('途中站上下文丢失')
            return
        if self.phase == 'ready':
            self.phase = 'station_wait_start'
            self.get_logger().info('本站动作完成，停在中转姿态；等待下一次开始消息')
        if self.phase == 'station_wait_start':
            self.step_started = monotonic()  # 人工等待没有运动，不消耗动作超时。
            if not self.station_resume_requested:
                return
            self.phase = 'station_travel'
        if self.phase in ('station_travel', 'posture_accept_pending', 'posture_wait'):
            self.advance_arm_posture('travel', next_phase='station_resume_mux')
        elif self.phase == 'station_resume_mux':
            self.phase = 'station_resume_mux_pending'
            self.request_bool(self.mux_enable_client, True, self.on_station_released)

    def on_station_released(self, future):
        self.checked_bool(future, 'follow_tracking')
        if self.failed:
            return
        context = self.station_context
        self.steps[self.step_index] = context['step']
        self.follow_tracking_seconds = context['tracking_seconds']
        self.follow_status, self.follow_status_time = None, None
        self.follow_last_tick = self.follow_last_tracking = monotonic()
        self.follow_boundary_since = self.follow_station_confirm_since = None
        self.step_started = monotonic()
        self.station_context = None
        self.station_resume_requested = False
        self.follow_resume_publisher.publish(Empty())
        self.get_logger().info('行驶关节姿态已实际到位，继续循迹；路线记录沿用，不重新启停驱动')

    def on_compliant_wrench(self, message):
        del message
        self.last_compliant_wrench_ns = self.get_clock().now().nanoseconds
        self.waiting_for_wrench_logged = False

    def on_chassis_status(self, message):
        task_name = message.task
        if message.execution_id != self.execution_ids.get(task_name):
            return
        data = {"task": task_name, "state": message.state, "message": message.message}
        self.latest_chassis_status[task_name] = data
        if message.state == TaskStatus.RUNNING:
            self.running_task_ids.add(task_name)
        if (
            message.state == TaskStatus.FAILED
            and self.active
            and message.execution_id not in self.cancelled_execution_ids
        ):
            self.fail_workflow(f"chassis task '{task_name}' failed: {message.message}")
            return
        step = self.current_step()
        if (
            self.active
            and self.phase == "waiting_chassis"
            and step is not None
            and step["action"] == "wait_chassis"
        ):
            self.process_waiting_chassis_status(step, data)

    def process_waiting_chassis_status(self, step, data):
        expected_task = self.resolve_task(step)
        if data.get("task") != expected_task:
            return
        state = data.get("state")
        if state == 1:
            if not self.seen_running:
                self.get_logger().info(f"Chassis task '{expected_task}' is running")
            self.seen_running = True
            return
        if state == 3:
            self.fail_workflow(
                f"chassis task '{expected_task}' failed: " f"{data.get('message', '')}"
            )
        elif state == 2:
            self.get_logger().info(f"Chassis task '{expected_task}' completed")
            self.complete_step()

    def prepare_step_camera(self, action):
        """底盘驱动常驻；关节/等待阶段暂停USB采集，循迹前恢复并等待新视觉数据。"""
        if not self.manage_line_stream or action not in ('line_follow', 'arm_posture', 'delay'):
            return True
        desired = action == 'line_follow'
        now = monotonic()
        if self.line_stream_wait is not None:
            goal, future, deadline = self.line_stream_wait
            if not future.done():
                if now > deadline:
                    self.fail_workflow('USB相机启停服务响应超时，底盘保持停车')
                return False
            self.line_stream_wait = None
            try:
                if not future.result().success:
                    raise RuntimeError(future.result().message)
            except Exception as error:
                self.fail_workflow('USB相机启停失败：'+str(error))
                return False
            self.line_streaming = goal
            self.get_logger().info('按需恢复循迹相机，等待新视觉反馈' if goal else '按需暂停循迹相机，保留底盘/机械臂驱动及里程计')
            self.line_warmup_deadline = now+15. if goal else None
            self.line_warmup_future = None
        if self.line_streaming != desired:
            if not self.line_stream_client.service_is_ready():
                self.fail_workflow('缺少循迹相机streaming服务，不能切换任务')
                return False
            request = SetBool.Request()
            request.data = desired
            self.line_stream_wait = (desired, self.line_stream_client.call_async(request), now+5.)
            return False
        if desired and self.line_warmup_deadline is not None:
            if now > self.line_warmup_deadline:
                self.fail_workflow('循迹相机恢复后15秒仍无有效新反馈，底盘保持停车')
                return False
            if self.line_warmup_future is not None and self.line_warmup_future.done():
                try:
                    ready = self.line_warmup_future.result().success
                except Exception:
                    ready = False
                self.line_warmup_future = None
                if ready:
                    self.line_warmup_deadline = None
                    return True
            if self.line_warmup_future is None and self.follow_ready_client.service_is_ready():
                self.line_warmup_future = self.follow_ready_client.call_async(Trigger.Request())
            return False
        return True

    def advance(self):
        if self.safety_inhibited or not self.active or self.failed:
            return
        if self.capture_only:
            # 独立拍照不启用底盘选择器、导纳或机器人动作，也不要求运动驱动在线。
            if monotonic()-self.step_started > 300.:
                self.fail_workflow("独立相机采集超过300秒；文件保留")
            else:
                self.advance_final_capture(self.current_step())
            return
        if not self.devices_checked and not self.advance_device_checks():
            return
        step = self.current_step()
        if step is None:
            self.active = False
            self.get_logger().info(f"Workflow '{self.workflow_name}' completed")
            if self.arm_control_mode == "admittance":
                self.get_logger().info(
                    "Admittance remains enabled at the last arm target"
                )
            return
        action = step["action"]
        now_ns = self.get_clock().now().nanoseconds

        if (action == "line_follow" and self.phase == "follow_tracking"
                and self.follow_status in (
                    "stop_marker_intermediate", "stop_marker_settling", "stop_marker_confirming", "stop_marker_averaging",
                    "stop_marker_overshoot", "stop_marker_missing", "stop_marker_unknown")
                and self.follow_status_time is not None
                and monotonic()-self.follow_status_time <= 1.0):
            # 只暂停人工站点等待的时钟；状态失联后仍恢复超时保护。
            self.step_started += max(0.0, monotonic()-self.follow_last_tick)
        if monotonic() - self.step_started > float(
            step.get("step_timeout_sec", self.step_timeout)
        ):
            self.fail_workflow(
                f"step {self.step_index} timed out in phase {self.phase}"
            )
            return
        if not self.prepare_step_camera(action):
            return
        if not self.mux_run_ready:
            if self.phase == "ready" and self.mux_enable_client.service_is_ready():
                if not self.mux_mode_client.service_is_ready():
                    return
                self.phase = "mux_start_pending"
                self.request_bool(
                    self.mux_mode_client,
                    False,
                    lambda result: self.checked_bool(result, "mux_enable_run"),
                )
            elif self.phase == "mux_enable_run":
                self.phase = "mux_enable_pending"
                self.request_bool(self.mux_enable_client, True, self.on_mux_run_ready)
            return
        if action == 'arm_posture':
            self.advance_arm_posture(step['posture'])
        elif action == 'continue_line_follow':
            self.advance_station_continue()
        elif action == "final_capture":
            self.advance_final_capture(step)
        elif action == 'save_line_route':
            if self.phase == 'ready' and self.route_checkpoint_client.service_is_ready():
                self.phase = 'route_checkpoint_pending'
                self.add_callback(self.route_checkpoint_client.call_async(Trigger.Request()), self.on_route_checkpoint)
        elif action == "hk_capture":
            self.advance_hk_capture()
        elif action == "wait_chassis_stopped":
            self.advance_chassis_stopped(step, now_ns)
        elif action == "line_follow":
            self.advance_line_follow(step)
        elif action in CONTROL_COMMANDS:
            self.advance_chassis_control(step, action, now_ns)
        elif action == "delay":
            if self.phase == "ready":
                seconds = float(step.get("seconds", 0.0))
                self.deadline_ns = now_ns + int(seconds * 1_000_000_000)
                self.phase = "waiting_delay"
                self.get_logger().info(f"Waiting {seconds:.1f}s")
            elif self.phase == "waiting_delay" and now_ns >= self.deadline_ns:
                self.complete_step()
        elif action == "wait_chassis":
            if self.phase == "ready":
                task_name = self.resolve_task(step)
                if task_name not in self.execution_ids:
                    self.fail_workflow(
                        f"task '{task_name}' has not been executed in this run"
                    )
                    return
                self.seen_running = task_name in self.running_task_ids
                self.phase = "waiting_chassis"
                self.get_logger().info(
                    f"Waiting for chassis task '{task_name}' to complete"
                )
                cached = self.latest_chassis_status.get(task_name)
                if cached is not None:
                    self.process_waiting_chassis_status(step, cached)
        elif action == "arm_poses":
            self.advance_arm_poses(step, now_ns)
        elif action == "arm_move":
            self.advance_arm_move(step, now_ns)

    def on_wheel_odom(self, message):
        self.wheel_odom = message
        self.wheel_odom_received = monotonic()
        step = self.current_step()
        if step is not None and step['action'] == 'wait_chassis_stopped':
            twist = message.twist.twist
            values = [twist.linear.x, twist.linear.y, twist.linear.z,
                      twist.angular.x, twist.angular.y, twist.angular.z]
            if (not all(math.isfinite(v) for v in values) or
                    math.sqrt(sum(v*v for v in values[:3])) > step.get('linear_tolerance', 0.005) or
                    math.sqrt(sum(v*v for v in values[3:])) > step.get('angular_tolerance', 0.02)):
                self.stationary_since = None

    def advance_chassis_stopped(self, step, now_ns):
        """只以新鲜轮速反馈确认停车；超时由工作流统一中止，不能用固定延时冒充停稳。"""
        now = monotonic()
        if self.phase == 'ready':
            self.stationary_since = None
            self.phase = 'waiting_stationary'
            self.get_logger().info('等待底盘轮速反馈连续停稳后继续任务')
        timeout = float(step.get('data_timeout_sec', 0.5))
        if (self.wheel_odom is None or self.wheel_odom_received is None or
                self.wheel_odom_received < self.step_started or now - self.wheel_odom_received > timeout):
            self.stationary_since = None
            return
        stamp = self.wheel_odom.header.stamp
        age = (now_ns - stamp.sec * 10**9 - stamp.nanosec) / 1e9
        twist = self.wheel_odom.twist.twist
        values = [twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z]
        if (stamp.sec == 0 and stamp.nanosec == 0 or not -0.1 <= age <= timeout or
                not all(math.isfinite(v) for v in values) or
                math.sqrt(sum(v*v for v in values[:3])) > float(step.get('linear_tolerance', 0.005)) or
                math.sqrt(sum(v*v for v in values[3:])) > float(step.get('angular_tolerance', 0.02))):
            self.stationary_since = None
            return
        if self.stationary_since is None:
            self.stationary_since = now
        if now - self.stationary_since >= float(step.get('stable_seconds', 1.0)):
            self.get_logger().info('底盘轮速反馈已连续停稳，继续后续步骤')
            self.complete_step()

    def reset_device_checks(self):
        self.devices_checked = not self.check_devices_on_startup
        self.devices_check_started = monotonic()
        self.devices_stable_since = None
        self.device_check_logged = float("-inf")
        self.device_check_requested = float("-inf")
        self.device_check_result_time = float("-inf")
        self.device_check_future = None
        self.line_devices_ready = False
        self.line_devices_reason = "waiting for chassis/USB readiness service"

    def on_line_devices_ready(self, future):
        if future is not self.device_check_future:
            return
        try:
            response = future.result()
            self.line_devices_ready = response.success
            self.line_devices_reason = response.message
            self.device_check_result_time = monotonic()
        except Exception as exc:
            self.line_devices_ready = False
            self.line_devices_reason = str(exc)
        if not self.line_devices_ready:
            self.devices_stable_since = None

    def advance_device_checks(self):
        now = monotonic()
        missing = []
        if self.check_arm_devices:
            if not self.arm_ready():
                missing.append("robot external control/controllers")
            if self.manage_admittance and not self.joint_posture_only and not self.admittance_enable_client.service_is_ready():
                missing.append("arm control mode service")
            if self.check_posture_devices and not all(
                    client.service_is_ready() for client in self.posture_clients.values()):
                missing.append('arm joint posture services')
            # 只有实际包含导纳动作的流程依赖零偏完成后的力反馈。
            # 普通 MoveIt 点位规划仍保留外控、控制器、TF 和执行接口检查。
            if self.check_admittance_devices:
                if self.compliant_target_publisher.get_subscription_count() == 0:
                    missing.append("admittance target subscriber")
                now_ns = self.get_clock().now().nanoseconds
                if (
                    self.last_compliant_wrench_ns is None
                    or now_ns - self.last_compliant_wrench_ns > 500_000_000
                ):
                    missing.append("force bias/fresh wrench data")
            if self.check_moveit_devices and (
                not self.execute_client.service_is_ready()
                or self.target_publisher.get_subscription_count() == 0
            ):
                missing.append("MoveIt target subscriber/execution service")
            try:
                self.tf_buffer.lookup_transform("base_link", "tool0", Time())
            except TransformException:
                missing.append("robot TCP transform")
        if self.check_line_devices:
            future = self.device_check_future
            if (
                future is not None
                and not future.done()
                and now - self.device_check_requested > 1.5
            ):
                self.follow_ready_client.remove_pending_request(future)
                future.cancel()
                self.line_devices_ready = False
                self.line_devices_reason = "readiness service timeout"
            if (
                future is None or future.done()
            ) and now - self.device_check_requested >= 0.3:
                if self.follow_ready_client.service_is_ready():
                    self.device_check_requested = now
                    self.device_check_future = self.follow_ready_client.call_async(
                        Trigger.Request()
                    )
                    self.add_callback(
                        self.device_check_future, self.on_line_devices_ready
                    )
            if not self.line_devices_ready or now - self.device_check_result_time > 1.0:
                missing.append("chassis/USB tracking: " + self.line_devices_reason)
            if not all(
                client.service_is_ready()
                for client in (
                    self.follow_enable_client,
                    self.mux_enable_client,
                    self.mux_mode_client,
                )
            ):
                missing.append("line follower/velocity selector services")
        if missing:
            self.devices_stable_since = None
            if now - self.device_check_logged >= 5.0:
                self.get_logger().info("Device check waiting: " + "; ".join(missing))
                self.device_check_logged = now
        elif self.devices_stable_since is None:
            self.devices_stable_since = now
        elif now - self.devices_stable_since >= self.device_ready_stable_sec:
            self.devices_checked = True
            self.step_started = now
            self.get_logger().info("All required devices ready; starting workflow")
            return True
        if now - self.devices_check_started > self.device_check_timeout:
            self.fail_workflow(
                "device startup check timed out: "
                + ("; ".join(missing) or "inputs unstable")
            )
        return False

    def advance_chassis_control(self, step, action, now_ns):
        del now_ns
        if (action == 'chassis_execute' and self.phase == 'ready'
                and self.workflow.get('arm_postures_before_chassis', False)):
            if not self.mux_enable_client.service_is_ready():
                return
            self.phase = 'chassis_output_pending'
            self.request_bool(self.mux_enable_client, True,
                              lambda result: self.checked_bool(result, 'chassis_output_ready'))
            return
        if self.phase == 'chassis_output_ready':
            self.phase = 'ready'
        if self.phase != "ready" or not self.control_client.service_is_ready():
            return
        if any(
            self.get_node_names().count(name) > 1
            for name in ("task_controller", "target_pid_controller")
        ):
            self.fail_workflow(
                "duplicate task/PID controllers; close the previous launch"
            )
            return
        request = TaskControl.Request()
        request.command = CONTROL_COMMANDS[action]
        request.task = self.resolve_task(step)
        if action == "chassis_execute":
            request.execution_id = str(uuid4())
            self.execution_ids[request.task] = request.execution_id
            self.running_task_ids.discard(request.task)
            self.latest_chassis_status.pop(request.task, None)
            if self.status_subscription.get_publisher_count() == 0:
                return
        if action == "chassis_stop":
            self.cancelled_execution_ids.update(self.execution_ids.values())
        self.phase = "control_pending"
        self.add_callback(
            self.control_client.call_async(request), self.on_control_response
        )
        self.get_logger().info(f"Calling local {action}: {request.task}")

    def on_control_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.fail_workflow(f"local chassis service failed: {exc}")
            return
        if not response.success:
            self.fail_workflow(f"local chassis command rejected: {response.message}")
            return
        self.complete_step()

    def advance_arm_poses(self, step, now_ns):
        if self.phase == "ready" and self.arm_control_mode == "admittance":
            if not self.admittance_enable_client.service_is_ready():
                return
            self.phase = "arm_disable_pending"
            request = SetBool.Request()
            request.data = False
            self.add_callback(
                self.admittance_enable_client.call_async(request),
                self.on_arm_disable_response,
            )
            return
        if self.phase == "ready":
            if (
                not self.execute_client.service_is_ready()
                or self.target_publisher.get_subscription_count() == 0
            ):
                return
            target = self.make_pose_array(step)
            target.header.stamp = self.get_clock().now().to_msg()
            self.target_publisher.publish(target)
            delivery_delay = float(step.get("delivery_delay", 0.2))
            self.deadline_ns = now_ns + int(delivery_delay * 1_000_000_000)
            self.phase = "arm_delivery"
            self.get_logger().info(f"Published {len(target.poses)} robot pose(s)")
        elif self.phase == "arm_delivery" and now_ns >= self.deadline_ns:
            self.phase = "arm_pending"
            future = self.execute_client.call_async(Trigger.Request())
            self.add_callback(future, self.on_arm_response)
            self.get_logger().info("MoveIt robot motion requested")

    def on_arm_disable_response(self, future):
        try:
            response = future.result()
            if not response.success:
                self.fail_workflow(f"cannot disable admittance: {response.message}")
                return
        except Exception as exc:
            self.fail_workflow(f"cannot disable admittance: {exc}")
            return
        self.arm_control_mode = "moveit"
        self.phase = "ready"

    def make_arm_target(self, step):
        base_frame = str(step.get("frame_id", "base_link"))
        end_effector_frame = str(step.get("end_effector_frame", "tool0"))
        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame, end_effector_frame, Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for {base_frame} -> {end_effector_frame} TF: {exc}"
            )
            return None

        translation = transform.transform.translation
        expected = step.get("expected_start_position")
        if expected is not None:
            tolerance = float(step.get("start_position_tolerance", 0.10))
            error = math.sqrt(
                (translation.x - float(expected[0])) ** 2
                + (translation.y - float(expected[1])) ** 2
                + (translation.z - float(expected[2])) ** 2
            )
            if error > tolerance:
                self.fail_workflow(
                    "current TCP position differs from the configured start "
                    f"by {error:.3f} m (limit {tolerance:.3f} m)"
                )
                return None

        recovered_target = self.recovery_arm_target
        if recovered_target is not None:
            step = deepcopy(step)
            saved = recovered_target.poses[0]
            step['target_pose']['position'] = [saved.position.x, saved.position.y, saved.position.z]
            step['target_pose']['position_mode'] = 'absolute'
        x, y, z = map(float, step["target_pose"]["position"])
        if step["target_pose"].get("position_mode", "absolute") == "relative":
            # 偏移在 frame_id 中表达，并且每一步只在起始时解析一次。
            x += translation.x
            y += translation.y
            z += translation.z
        distance = math.sqrt(
            (x - translation.x) ** 2
            + (y - translation.y) ** 2
            + (z - translation.z) ** 2
        )
        maximum = float(step.get("max_position_distance", 0.10))
        if distance > maximum:
            self.fail_workflow(
                f"arm target is {distance:.3f} m from the current TCP "
                f"(limit {maximum:.3f} m)"
            )
            return None

        target = PoseArray()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = base_frame
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        orientation = step["target_pose"]["orientation"]
        if orientation == "current":
            pose.orientation = transform.transform.rotation
        else:
            roll, pitch, yaw = map(math.radians, map(float, orientation))
            cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
            cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
            cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
            pose.orientation.x = sr * cp * cy - cr * sp * sy
            pose.orientation.y = cr * sp * cy + sr * cp * sy
            pose.orientation.z = cr * cp * sy - sr * sp * cy
            pose.orientation.w = cr * cp * cy + sr * sp * sy
        if recovered_target is not None:
            pose.orientation = deepcopy(recovered_target.poses[0].orientation)
        target.poses.append(pose)
        self.get_logger().info(
            f"Arm target in {base_frame}: current=[{translation.x:.3f}, "
            f"{translation.y:.3f}, {translation.z:.3f}] -> "
            f"target=[{x:.3f}, {y:.3f}, {z:.3f}] m"
        )
        return target

    def on_robot_task_running(self, message):
        self.robot_task_running = message.data

    def arm_ready(self):
        if not self.require_arm_ready:
            return True
        now = monotonic()
        if (
            self.arm_check_future is None or self.arm_check_future.done()
        ) and now - self.arm_check_requested >= 1.0:
            if self.arm_controllers_client.service_is_ready():
                self.arm_check_requested = now
                self.arm_check_future = self.arm_controllers_client.call_async(
                    ListControllers.Request()
                )
                self.arm_check_future.add_done_callback(self.on_arm_controllers)
        ready = (
            self.robot_task_running
            and self.robot_task_subscription.get_publisher_count() > 0
            and self.arm_controllers_active
            and now - self.arm_controllers_checked <= 2.5
        )
        if not ready and not self.arm_wait_logged:
            self.get_logger().info(
                "Waiting for robot external control and active arm controllers"
            )
            self.arm_wait_logged = True
        if ready:
            self.arm_wait_logged = False
        return ready

    def on_arm_controllers(self, future):
        try:
            states = {item.name: item.state for item in future.result().controller}
            required = (
                "scaled_joint_trajectory_controller",
                "joint_state_broadcaster",
                "speed_scaling_state_broadcaster",
                "tcp_pose_broadcaster",
            )
            self.arm_controllers_active = all(
                states.get(name) == "active" for name in required
            )
            self.arm_controllers_checked = monotonic()
        except Exception as exc:
            self.arm_controllers_active = False
            self.get_logger().warn(f"Cannot check arm controller readiness: {exc}")

    def advance_arm_move(self, step, now_ns):
        mode = step["control_mode"]
        if self.phase == "ready":
            if not self.arm_ready():
                return
            if self.manage_admittance and not self.admittance_enable_client.service_is_ready():
                return
            if mode == "admittance":
                if self.compliant_target_publisher.get_subscription_count() == 0:
                    return
                wrench_age = (
                    None
                    if self.last_compliant_wrench_ns is None
                    else (now_ns - self.last_compliant_wrench_ns) / 1_000_000_000.0
                )
                if wrench_age is None or wrench_age > 0.5:
                    if not self.waiting_for_wrench_logged:
                        self.get_logger().warn(
                            "Waiting for force-sensor bias and fresh wrench data"
                        )
                        self.waiting_for_wrench_logged = True
                    return
            elif not self.execute_client.service_is_ready():
                return
            target = self.make_arm_target(step)
            if target is None:
                return
            self.pending_arm_target = target
            self.pending_arm_skip = False
            if mode == "moveit" and step.get("skip_if_at_target", False):
                try:
                    actual = self.tf_buffer.lookup_transform(
                        target.header.frame_id,
                        step.get("end_effector_frame", "tool0"),
                        Time(),
                    ).transform
                except TransformException:
                    return
                self.pending_arm_skip = self.arm_target_reached(actual, target.poses[0], step)
            self.phase = "arm_mode_pending"
            if self.manage_admittance:
                request = SetBool.Request()
                request.data = mode == "admittance"
                future = self.admittance_enable_client.call_async(request)
                self.add_callback(future, self.on_arm_mode_response)
            else:
                self.arm_mode_ready()
        elif self.phase == "arm_delivery" and now_ns >= self.deadline_ns:
            self.phase = "arm_pending"
            future = self.execute_client.call_async(Trigger.Request())
            self.add_callback(future, self.on_arm_response)
            self.get_logger().info("MoveIt robot motion requested")
        elif self.phase == "arm_tracking":
            self.check_arm_tracking(step, now_ns)
        elif self.phase == "arm_arrival_delay" and now_ns >= self.deadline_ns:
            self.get_logger().info("Arm arrival delay finished; continuing workflow")
            self.complete_step()

    @staticmethod
    def arm_target_reached(actual, goal, step):
        """比较同一坐标系中的实际 TCP 与目标，位置和方向必须同时满足误差要求。"""
        position = (actual.translation.x, actual.translation.y, actual.translation.z)
        target = (goal.position.x, goal.position.y, goal.position.z)
        distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(position, target)))
        q1 = (actual.rotation.x, actual.rotation.y, actual.rotation.z, actual.rotation.w)
        q2 = (goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w)
        norm = math.sqrt(sum(x * x for x in q1) * sum(x * x for x in q2))
        if not math.isfinite(norm) or norm <= 1e-12:
            return False
        # q 和 -q 表示同一方向；使用最短旋转角，避免欧拉角跨越 ±180° 的歧义。
        cosine = abs(sum(a * b for a, b in zip(q1, q2))) / norm
        angle = 2.0 * math.acos(min(1.0, cosine))
        return (
            distance <= float(step.get("arrival_tolerance", 0.01))
            and angle <= math.radians(float(step.get("arrival_orientation_tolerance_deg", 1.0)))
        )

    def on_arm_mode_response(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.fail_workflow(f"arm control mode service failed: {exc}")
            return
        if not response.success:
            self.fail_workflow(f"arm control mode rejected: {response.message}")
            return
        self.arm_mode_ready()

    def arm_mode_ready(self):
        """模式已确认后投递目标；纯MoveIt任务不需要导纳开关服务。"""
        mode = self.current_step()["control_mode"]
        self.arm_control_mode = mode
        if mode == 'moveit' and self.pending_arm_skip:
            self.get_logger().info('当前位置及方向已达到初始目标，跳过重复运动')
            self.finish_arm_arrival(self.get_clock().now().nanoseconds)
            return
        if mode == "admittance":
            self.compliant_target_publisher.publish(self.pending_arm_target)
            self.get_logger().info(
                "Admittance target published; waiting for actual TCP arrival"
            )
            self.arm_start_ns = self.get_clock().now().nanoseconds
            self.arm_last_progress_ns = self.arm_start_ns
            self.arm_best_error = float("inf")
            self.phase = "arm_tracking"
            return
        self.target_publisher.publish(self.pending_arm_target)
        delivery_delay = float(self.current_step().get("delivery_delay", 0.2))
        self.deadline_ns = self.get_clock().now().nanoseconds + int(
            delivery_delay * 1_000_000_000
        )
        self.phase = "arm_delivery"
        self.get_logger().info(
            "MoveIt target published: "
            f"{self.pending_arm_target.poses[0].position.x:.3f}, "
            f"{self.pending_arm_target.poses[0].position.y:.3f}, "
            f"{self.pending_arm_target.poses[0].position.z:.3f} m"
        )

    def check_arm_tracking(self, step, now_ns):
        frame = self.pending_arm_target.header.frame_id
        tip = str(step.get("end_effector_frame", "tool0"))
        elapsed = (now_ns - self.arm_start_ns) / 1_000_000_000.0
        stalled = (now_ns - self.arm_last_progress_ns) / 1_000_000_000.0
        try:
            transform = self.tf_buffer.lookup_transform(frame, tip, Time())
        except TransformException as exc:
            if elapsed >= float(step.get("motion_timeout_sec", 20.0)):
                self.stop_admittance_after_failure(
                    f"arm TF {frame} -> {tip} unavailable for {elapsed:.1f}s: {exc}"
                )
            return
        actual = transform.transform.translation
        goal = self.pending_arm_target.poses[0].position
        error = math.sqrt(
            (goal.x - actual.x) ** 2
            + (goal.y - actual.y) ** 2
            + (goal.z - actual.z) ** 2
        )
        if error <= float(step.get("arrival_tolerance", 0.01)):
            self.get_logger().info(
                f"Arm entered compliant target region; position error={error:.3f} m. "
                "Admittance control remains enabled."
            )
            self.finish_arm_arrival(now_ns)
            return
        if error < self.arm_best_error - 0.002:
            self.arm_best_error = error
            self.arm_last_progress_ns = now_ns
        if elapsed >= float(step.get("motion_timeout_sec", 20.0)) or stalled >= float(
            step.get("progress_timeout_sec", 5.0)
        ):
            self.stop_admittance_after_failure(
                f"arm did not settle within {float(step.get('arrival_tolerance', 0.01)):.3f} m: "
                f"error={error:.3f} m, best={self.arm_best_error:.3f} m, "
                f"elapsed={elapsed:.1f}s, no progress for {stalled:.1f}s"
            )

    def stop_admittance_after_failure(self, message):
        self.fail_workflow(message)

    def finish_arm_arrival(self, now_ns):
        """Wait once after arrival while retaining the selected control mode."""
        seconds = float(self.current_step().get("after_arrival_delay_sec", 0.0))
        if seconds == 0.0:
            self.complete_step()
            return
        self.deadline_ns = now_ns + int(seconds * 1_000_000_000)
        self.phase = "arm_arrival_delay"
        self.get_logger().info(
            f"Arm arrived; waiting {seconds:.1f}s before the next workflow step. "
            f"Control mode remains {self.arm_control_mode}."
        )

    def make_pose_array(self, step):
        target = PoseArray()
        target.header.frame_id = str(step.get("frame_id", "base_link"))
        degrees = bool(step.get("angles_in_degrees", True))
        for values in step["poses"]:
            x, y, z, roll, pitch, yaw = map(float, values)
            if degrees:
                roll, pitch, yaw = map(math.radians, (roll, pitch, yaw))
            cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
            cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
            cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.x = sr * cp * cy - cr * sp * sy
            pose.orientation.y = cr * sp * cy + sr * cp * sy
            pose.orientation.z = cr * cp * sy - sr * sp * cy
            pose.orientation.w = cr * cp * cy + sr * sp * sy
            target.poses.append(pose)
        return target

    def on_arm_response(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.fail_workflow(f"MoveIt service call failed: {exc}")
            return
        if response.success:
            self.get_logger().info(f"Robot motion completed: {response.message}")
            if self.current_step()["action"] == "arm_move":
                self.finish_arm_arrival(self.get_clock().now().nanoseconds)
            else:
                self.complete_step()
        else:
            self.fail_workflow(f"Robot motion failed: {response.message}")

    def complete_step(self):
        self.recovery_arm_target = None
        if self.recovery_hold_step is not None:
            self.recovery_hold_step = None
            self.active = False
            self.phase = "recovered_compliant_hold"
            return
        if self.capture_only:
            self.active = False
            self.phase = "capture_complete"
            self.get_logger().info("独立相机采集和TCP发送完成")
            return
        self.step_started = monotonic()
        self.step_index += 1
        self.phase = "ready"

    def fail_workflow(self, message):
        self.stop_capture_process()
        if (self.workflow.get('arm_postures_before_chassis', False)
                and self.posture_cancel_client.service_is_ready()):
            self.posture_cancel_client.call_async(Trigger.Request())
        self.last_failure = str(message)
        self.generation += 1
        if self.capture_only:
            self.failed, self.active, self.phase = True, False, "failed"
            self.get_logger().error("独立相机采集失败："+str(message))
            return
        for client in (self.mux_enable_client, self.follow_enable_client):
            if client.service_is_ready():
                request = SetBool.Request()
                request.data = False
                client.call_async(request)
        if self.control_client.service_is_ready():
            request = TaskControl.Request()
            request.command = TaskControl.Request.STOP
            self.control_client.call_async(request)
        if self.admittance_enable_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self.admittance_enable_client.call_async(request)
        self.failed = True
        self.active = False
        self.phase = "failed"
        self.get_logger().error(f"Workflow '{self.workflow_name}' stopped: {message}")
        step = self.current_step()
        if step and step["action"] in ("chassis_execute", "wait_chassis"):
            self.get_logger().error(
                "Stopped at chassis step; subsequent robot motion steps were not executed"
            )


def main():
    rclpy.init()
    node = WorkflowController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
