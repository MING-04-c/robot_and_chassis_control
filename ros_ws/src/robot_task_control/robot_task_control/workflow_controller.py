#!/usr/bin/env python3
"""Execute a YAML-defined chassis and robot motion workflow."""

import json
import math
from pathlib import Path
from time import monotonic
from uuid import uuid4

import rclpy
import yaml
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseArray, WrenchStamped
from masc_chassis_msgs.msg import TaskStatus
from masc_chassis_msgs.srv import TaskControl
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String
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
    "line_follow",
}


class WorkflowController(Node):
    def __init__(self):
        super().__init__("task_controller")
        workflow_file = str(self.declare_parameter("workflow_file", "").value)
        if not workflow_file:
            raise ValueError("workflow_file parameter is required")
        self.workflow_path = Path(workflow_file).resolve()
        self.workflow = self.load_workflow(self.workflow_path)
        self.workflow_name = str(self.workflow.get("name", "unnamed_workflow"))
        self.steps = self.workflow["steps"]
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
        self.require_image_receiver = bool(
            self.declare_parameter("require_image_receiver", False).value
        )
        self.image_ready_client = self.create_client(
            Trigger, "/hk_image_transfer/ready"
        )
        self.capture_client = self.create_client(Trigger, "/dataset_capture/capture")
        self.follow_status = None
        self.follow_status_time = None
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
            step["action"] in ("arm_move", "arm_poses") for step in self.steps
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
        self.generation = 0
        self.execution_ids = {}
        self.cancelled_execution_ids = set()
        self.step_started = monotonic()

        self.active = self.auto_start
        self.failed = False
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
        self.get_logger().info(
            f"Loaded workflow '{self.workflow_name}' with {len(self.steps)} steps "
            f"from {workflow_file}"
        )
        if self.auto_start:
            self.get_logger().info("Workflow will start automatically")
        else:
            self.get_logger().info("Call /task_controller/start to begin")

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
        for index, step in enumerate(steps):
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
            if action == "line_follow":
                progress_timeout = float(step.get("progress_timeout_sec", 3.0))
                if not math.isfinite(progress_timeout) or progress_timeout <= 0.0:
                    raise ValueError(
                        f"workflow step {index} has invalid progress_timeout_sec"
                    )
                duration = float(step.get("seconds", -1.0))
                if not math.isfinite(duration) or duration <= 0.0:
                    raise ValueError(
                        f"workflow step {index} line_follow.seconds must be positive"
                    )
            if action == "arm_poses":
                WorkflowController.validate_pose_values(step.get("poses"), index)
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

    def current_step(self):
        if self.step_index >= len(self.steps):
            return None
        return self.steps[self.step_index]

    def on_start_request(self, request, response):
        del request
        if self.active:
            response.success = False
            response.message = "workflow is already running"
            return response
        if self.step_index >= len(self.steps) and not self.failed:
            self.step_index = 0
        if self.failed:
            self.step_index = 0
            self.failed = False
        self.generation += 1
        self.mux_run_ready = not self.use_velocity_mux
        self.execution_ids.clear()
        self.cancelled_execution_ids.clear()
        self.running_task_ids.clear()
        self.latest_chassis_status.clear()
        self.step_started = monotonic()
        self.phase = "ready"
        self.active = True
        self.reset_device_checks()
        response.success = True
        response.message = f"workflow '{self.workflow_name}' started"
        self.get_logger().info(response.message)
        return response

    def on_stop_request(self, request, response):
        del request
        self.fail_workflow("stopped by user")
        response.success = True
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
        if self.phase == "ready":
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
            self.phase = "follow_enable_pending"
            self.request_bool(
                self.follow_enable_client,
                True,
                lambda result: self.checked_bool(result, "follow_tracking"),
            )
        elif self.phase == "follow_tracking":
            now = monotonic()
            dt = min(now - self.follow_last_tick, 0.2)
            self.follow_last_tick = now
            fresh = (
                self.follow_status_time is not None
                and now - self.follow_status_time <= 1.0
            )
            if fresh and self.follow_status == "tracking":
                self.follow_tracking_seconds += dt
                self.follow_last_tracking = now
            if self.follow_tracking_seconds >= float(step["seconds"]):
                self.phase = "follow_disable_pending"
                self.request_bool(
                    self.follow_enable_client,
                    False,
                    lambda result: self.checked_bool(result, "follow_restore_pid"),
                )
            elif now - self.follow_last_tracking > float(
                step.get("progress_timeout_sec", 3.0)
            ):
                self.fail_workflow(f"line following unavailable: {self.follow_status}")
        elif self.phase == "follow_restore_pid":
            self.phase = "follow_restore_pending"
            self.request_bool(self.mux_mode_client, False, self.on_follow_finished)

    def on_pid_stopped_for_follow(self, future):
        self.checked_bool(future, "follow_select")

    def on_follow_finished(self, future):
        self.checked_bool(future, "ready")
        if not self.failed:
            self.complete_step()

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

    def advance(self):
        if not self.active or self.failed:
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

        if monotonic() - self.step_started > float(
            step.get("step_timeout_sec", self.step_timeout)
        ):
            self.fail_workflow(
                f"step {self.step_index} timed out in phase {self.phase}"
            )
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
        if action == "hk_capture":
            self.advance_hk_capture()
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
            if (
                not self.admittance_enable_client.service_is_ready()
                or self.compliant_target_publisher.get_subscription_count() == 0
            ):
                missing.append("admittance node")
            now_ns = self.get_clock().now().nanoseconds
            if (
                self.last_compliant_wrench_ns is None
                or now_ns - self.last_compliant_wrench_ns > 500_000_000
            ):
                missing.append("force bias/fresh wrench data")
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
            if not self.admittance_enable_client.service_is_ready():
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
            self.phase = "arm_mode_pending"
            request = SetBool.Request()
            request.data = mode == "admittance"
            future = self.admittance_enable_client.call_async(request)
            self.add_callback(future, self.on_arm_mode_response)
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

    def on_arm_mode_response(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.fail_workflow(f"arm control mode service failed: {exc}")
            return
        if not response.success:
            self.fail_workflow(f"arm control mode rejected: {response.message}")
            return
        mode = self.current_step()["control_mode"]
        self.arm_control_mode = mode
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
        self.step_started = monotonic()
        self.step_index += 1
        self.phase = "ready"

    def fail_workflow(self, message):
        self.generation += 1
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
