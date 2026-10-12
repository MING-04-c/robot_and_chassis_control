#!/usr/bin/env python3
"""Execute a YAML-defined chassis and robot motion workflow."""

import json
import math
from pathlib import Path

import rclpy
import yaml
from eli_cs_robot_serial.srv import ChassisControl
from geometry_msgs.msg import Pose, PoseArray, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener


CONTROL_COMMANDS = {
    "chassis_execute": ChassisControl.Request.EXECUTE_TASK,
    "chassis_continue": ChassisControl.Request.CONTINUE,
    "chassis_stop": ChassisControl.Request.STOP,
}
SUPPORTED_ACTIONS = set(CONTROL_COMMANDS) | {
    "delay",
    "wait_chassis",
    "arm_poses",
    "arm_move",
}


class WorkflowController(Node):
    def __init__(self):
        super().__init__("workflow_controller")
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
                "auto_start", bool(self.workflow.get("auto_start", True))
            ).value
        )
        self.retry_period = float(
            self.declare_parameter("retry_period_sec", 1.0).value
        )
        self.control_service = str(
            self.declare_parameter(
                "control_service", "/computer_pi/chassis/control"
            ).value
        )
        self.online_topic = str(
            self.declare_parameter("online_topic", "/computer_pi/online").value
        )
        self.status_topic = str(
            self.declare_parameter(
                "chassis_status_topic", "/computer_pi/chassis_status"
            ).value
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
        if self.retry_period <= 0.0:
            raise ValueError("retry_period_sec must be positive")

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.online_subscription = self.create_subscription(
            Bool, self.online_topic, self.on_online, latched_qos
        )
        self.status_subscription = self.create_subscription(
            String, self.status_topic, self.on_chassis_status, 20
        )
        self.target_publisher = self.create_publisher(
            PoseArray, self.target_topic, 10
        )
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
        self.control_client = self.create_client(
            ChassisControl, self.control_service
        )
        self.execute_client = self.create_client(Trigger, self.execute_service)
        self.admittance_enable_client = self.create_client(
            SetBool, "/admittance_controller/enable"
        )
        self.start_server = self.create_service(
            Trigger, "~/start", self.on_start_request
        )

        self.online = False
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
            self.get_logger().info("Call /workflow_controller/start to begin")

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
            if action in ("chassis_execute", "wait_chassis") and not step.get(
                "task"
            ):
                raise ValueError(f"workflow step {index} requires task")
            if action == "delay" and float(step.get("seconds", -1.0)) < 0.0:
                raise ValueError(f"workflow step {index} has invalid seconds")
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
        self.phase = "ready"
        self.active = True
        response.success = True
        response.message = f"workflow '{self.workflow_name}' started"
        self.get_logger().info(response.message)
        return response

    def on_online(self, message):
        was_online = self.online
        self.online = bool(message.data)
        if self.online and not was_online:
            self.get_logger().info("Chassis connection is online")
        elif not self.online and was_online:
            self.get_logger().warn("Chassis connection is offline; workflow paused")

    def on_compliant_wrench(self, message):
        del message
        self.last_compliant_wrench_ns = self.get_clock().now().nanoseconds
        self.waiting_for_wrench_logged = False

    def on_chassis_status(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Ignoring invalid chassis JSON: {exc}")
            return
        if data.get("type") != "chassis_status":
            return
        task_name = data.get("task")
        if not isinstance(task_name, str) or not task_name:
            return
        self.latest_chassis_status[task_name] = data
        if data.get("state") == 1:
            self.running_task_ids.add(task_name)

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
                self.get_logger().info(
                    f"Chassis task '{expected_task}' is running"
                )
            self.seen_running = True
            return
        require_running = bool(step.get("require_running", True))
        if state in (2, 3) and require_running and not self.seen_running:
            self.get_logger().warn(
                f"Ignoring stale terminal state for '{expected_task}' because "
                "this workflow has not observed running"
            )
            return
        if state == 3:
            self.fail_workflow(
                f"chassis task '{expected_task}' failed: "
                f"{data.get('message', '')}"
            )
        elif state == 2:
            self.get_logger().info(
                f"Chassis task '{expected_task}' completed"
            )
            self.complete_step()

    def advance(self):
        if not self.active or self.failed:
            return
        step = self.current_step()
        if step is None:
            self.active = False
            self.get_logger().info(
                f"Workflow '{self.workflow_name}' completed"
            )
            if self.arm_control_mode == "admittance":
                self.get_logger().info(
                    "Admittance remains enabled at the last arm target"
                )
            return
        action = step["action"]
        now_ns = self.get_clock().now().nanoseconds

        if action in CONTROL_COMMANDS:
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

    def advance_chassis_control(self, step, action, now_ns):
        if self.phase == "retry" and now_ns < self.deadline_ns:
            return
        if self.phase not in ("ready", "retry"):
            return
        if not self.online or not self.control_client.service_is_ready():
            return
        if not self.check_chassis_command_sources():
            return
        request = ChassisControl.Request()
        request.command = CONTROL_COMMANDS[action]
        request.task = self.resolve_task(step)
        if action == "chassis_execute":
            self.running_task_ids.discard(request.task)
            self.latest_chassis_status.pop(request.task, None)
        self.phase = "control_pending"
        future = self.control_client.call_async(request)
        future.add_done_callback(self.on_control_response)
        detail = f" task={request.task}" if request.task else ""
        self.get_logger().info(f"Sending {action}{detail}")

    def check_chassis_command_sources(self):
        """Reject duplicate workflow/bridge instances before sending motion commands."""
        duplicates = []
        for name in ("workflow_controller", "computer_pi_status_node", "computer_pi_tcp_node"):
            count = self.get_node_names().count(name)
            if count > 1:
                duplicates.append(f"{name} x{count}")
        if duplicates:
            self.fail_workflow(
                "duplicate chassis command nodes: " + ", ".join(duplicates)
                + "; close the old workflow/computer_pi launch before restarting. "
                "No new chassis command was sent"
            )
            return False
        return True

    def on_control_response(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.schedule_retry(f"chassis control service failed: {exc}")
            return
        if not response.success:
            self.schedule_retry(
                f"chassis control rejected: {response.message}"
            )
            return
        self.get_logger().info(
            f"Chassis control queued as {response.command_id}"
        )
        self.complete_step()

    def schedule_retry(self, message):
        self.get_logger().error(message)
        self.phase = "retry"
        self.deadline_ns = (
            self.get_clock().now().nanoseconds
            + int(self.retry_period * 1_000_000_000)
        )

    def advance_arm_poses(self, step, now_ns):
        if self.phase == "ready":
            if not self.execute_client.service_is_ready():
                return
            target = self.make_pose_array(step)
            target.header.stamp = self.get_clock().now().to_msg()
            self.target_publisher.publish(target)
            delivery_delay = float(step.get("delivery_delay", 0.2))
            self.deadline_ns = now_ns + int(delivery_delay * 1_000_000_000)
            self.phase = "arm_delivery"
            self.get_logger().info(
                f"Published {len(target.poses)} robot pose(s)"
            )
        elif self.phase == "arm_delivery" and now_ns >= self.deadline_ns:
            self.phase = "arm_pending"
            future = self.execute_client.call_async(Trigger.Request())
            future.add_done_callback(self.on_arm_response)
            self.get_logger().info("MoveIt robot motion requested")

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

    def advance_arm_move(self, step, now_ns):
        mode = step["control_mode"]
        if self.phase == "ready":
            if not self.admittance_enable_client.service_is_ready():
                return
            if mode == "admittance":
                if self.compliant_target_publisher.get_subscription_count() == 0:
                    return
                wrench_age = (
                    None if self.last_compliant_wrench_ns is None
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
            future.add_done_callback(self.on_arm_mode_response)
        elif self.phase == "arm_delivery" and now_ns >= self.deadline_ns:
            self.phase = "arm_pending"
            future = self.execute_client.call_async(Trigger.Request())
            future.add_done_callback(self.on_arm_response)
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
        self.deadline_ns = (
            self.get_clock().now().nanoseconds
            + int(delivery_delay * 1_000_000_000)
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
        if (
            elapsed >= float(step.get("motion_timeout_sec", 20.0))
            or stalled >= float(step.get("progress_timeout_sec", 5.0))
        ):
            self.stop_admittance_after_failure(
                f"arm did not settle within {float(step.get('arrival_tolerance', 0.01)):.3f} m: "
                f"error={error:.3f} m, best={self.arm_best_error:.3f} m, "
                f"elapsed={elapsed:.1f}s, no progress for {stalled:.1f}s"
            )

    def stop_admittance_after_failure(self, message):
        if self.admittance_enable_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self.admittance_enable_client.call_async(request)
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
        self.step_index += 1
        self.phase = "ready"

    def fail_workflow(self, message):
        self.failed = True
        self.active = False
        self.phase = "failed"
        self.get_logger().error(
            f"Workflow '{self.workflow_name}' stopped: {message}"
        )
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
