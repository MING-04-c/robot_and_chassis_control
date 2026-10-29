#!/usr/bin/env python3
"""Exercise the real PID and workflow against synthetic wheel odometry, without CAN."""
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from uuid import uuid4

# Run on an isolated domain so the test cannot address deployed controllers.
os.environ["ROS_DOMAIN_ID"] = "187"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["ROS_LOG_DIR"] = "/tmp/robot_task_control_test_logs"

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseArray, TransformStamped, Twist, WrenchStamped
from masc_chassis_msgs.msg import TaskStatus
from masc_chassis_msgs.srv import TaskControl
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
from robot_task_control.workflow_controller import WorkflowController
from robot_task_control.velocity_mux import VelocityMux


class LocalControlTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.workflow = Path(self.temp.name) / "workflow.yaml"
        (Path(self.temp.name) / "tasks.yaml").write_text("tasks:\n  forward: {}\n")
        self.workflow.write_text(
            yaml.safe_dump(
                {
                    "workflow": {
                        "name": "local_test",
                        "auto_start": False,
                        "task_table": "tasks.yaml",
                        "steps": [
                            {"action": "chassis_execute", "task": "forward"},
                            {"action": "wait_chassis", "task": "forward"},
                            {"action": "chassis_execute", "task": "forward"},
                            {"action": "wait_chassis", "task": "forward"},
                        ],
                    }
                }
            )
        )
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                f"workflow_file:={self.workflow}",
                "-p",
                "step_timeout_sec:=3.0",
            ]
        )
        self.node = Node("synthetic_wheels")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.controller = WorkflowController()
        self.executor.add_node(self.controller)
        self.odom_pub = self.node.create_publisher(Odometry, "/wheel_odom", 10)
        self.command = Twist()
        self.commands = []
        self.node.create_subscription(Twist, "/cmd_vel", self.on_command, 10)
        self.statuses = []
        self.node.create_subscription(
            TaskStatus, "/chassis/task_status", self.statuses.append, 20
        )
        self.client = self.node.create_client(TaskControl, "/chassis/task_control")
        self.x = self.y = self.yaw = 0.0
        self.addCleanup(lambda: rclpy.shutdown() if rclpy.ok() else None)
        self.feedback = True
        self.integrate = True
        self.last_tick = time.monotonic()
        self.node.create_timer(0.02, self.tick)
        binary = (
            Path(get_package_prefix("masc_chassis_ros_driver"))
            / "lib/masc_chassis_ros_driver/target_pid_controller_node"
        )
        self.log = open(Path(self.temp.name) / "pid.log", "w+")
        pid_remap = [
            (
                "/robot_control/pid/cmd_vel"
                if self._testMethodName == "test_camera_workflow_mux_and_capture"
                else "/cmd_vel"
            )
        ][0]
        self.pid = subprocess.Popen(
            [
                str(binary),
                "--ros-args",
                "-p",
                "task_names:=[forward]",
                "-p",
                "task_delta_x_m:=[0.02]",
                "-p",
                "task_delta_y_m:=[0.0]",
                "-p",
                "task_delta_yaw_rad:=[0.0]",
                "-p",
                "kp_xy:=4.0",
                "-p",
                "kd_xy:=0.0",
                "-p",
                "position_deadband:=0.001",
                "-p",
                "task_timeout_sec:=1.5",
                "-p",
                "odom_timeout_sec:=0.15",
                "-p",
                "status_period_sec:=0.03",
                "-r",
                "wheel_odom:=/wheel_odom",
                "-r",
                f"cmd_vel:={pid_remap}",
            ],
            stdout=self.log,
            stderr=subprocess.STDOUT,
        )
        self.wait_for(lambda: self.client.service_is_ready(), 8)
        self.pump(0.3)

    def tearDown(self):
        self.pid.terminate()
        self.pid.wait(timeout=5)
        self.log.close()
        self.executor.shutdown()
        self.controller.destroy_node()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.temp.cleanup()

    def on_command(self, msg):
        self.command = msg
        self.commands.append(msg)

    def tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        if self.integrate:
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.x += (c * self.command.linear.x - s * self.command.linear.y) * dt
            self.y += (s * self.command.linear.x + c * self.command.linear.y) * dt
            self.yaw += self.command.angular.z * dt
        if self.feedback:
            odom = Odometry()
            odom.header.stamp = self.node.get_clock().now().to_msg()
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
            odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
            self.odom_pub.publish(odom)

    def pump(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.executor.spin_once(timeout_sec=0.02)

    def wait_for(self, condition, seconds=4):
        until = time.monotonic() + seconds
        while not condition() and time.monotonic() < until:
            self.executor.spin_once(timeout_sec=0.02)
        self.assertTrue(condition(), "condition timed out")

    def call(self, command=TaskControl.Request.EXECUTE, task="forward"):
        request = TaskControl.Request()
        request.command, request.task = command, task
        request.execution_id = str(uuid4())
        future = self.client.call_async(request)
        self.wait_for(future.done)
        return request.execution_id, future.result()

    def test_repeated_workflow_and_stale_completion(self):
        start = self.node.create_client(Trigger, "/task_controller/start")
        self.wait_for(start.service_is_ready)
        future = start.call_async(Trigger.Request())
        self.wait_for(future.done)
        self.assertTrue(future.result().success)
        self.wait_for(lambda: self.controller.phase == "waiting_chassis")
        # Old COMPLETED for the same name must not advance this execution.
        index = self.controller.step_index
        stale = TaskStatus(
            task="forward", execution_id="previous-run", state=TaskStatus.COMPLETED
        )
        self.controller.on_chassis_status(stale)
        self.assertEqual(index, self.controller.step_index)
        self.wait_for(lambda: not self.controller.active, 6)
        self.assertFalse(self.controller.failed)
        completed = [s for s in self.statuses if s.state == TaskStatus.COMPLETED]
        self.assertEqual(len(completed), 2)
        self.assertNotEqual(completed[0].execution_id, completed[1].execution_id)
        self.assertAlmostEqual(self.x, 0.04, delta=0.003)

    def test_rejections_stop_and_odom_loss(self):
        _, response = self.call(task="unknown")
        self.assertFalse(response.success)
        execution, response = self.call()
        self.assertTrue(response.success)
        _, response = self.call()
        self.assertFalse(response.success)
        self.assertEqual(response.message, "controller_busy")
        _, response = self.call(command=TaskControl.Request.STOP)
        self.assertTrue(response.success)
        self.wait_for(
            lambda: any(
                s.execution_id == execution and s.state == TaskStatus.FAILED
                for s in self.statuses
            )
        )
        execution, response = self.call()
        self.assertTrue(response.success)
        self.feedback = False
        self.wait_for(
            lambda: any(
                s.execution_id == execution and s.message == "odometry_timeout"
                for s in self.statuses
            )
        )
        self.pump(0.1)
        self.assertEqual(self.command.linear.x, 0.0)
        _, response = self.call()
        self.assertFalse(response.success)
        self.assertEqual(response.message, "odometry_unavailable")

    def test_motion_timeout_and_workflow_stop(self):
        self.integrate = False
        execution, response = self.call()
        self.assertTrue(response.success)
        self.wait_for(
            lambda: any(
                s.execution_id == execution and s.message == "motion_timeout"
                for s in self.statuses
            )
        )
        self.pump(0.1)
        self.assertEqual(self.command.linear.x, 0.0)
        self.controller.on_start_request(Trigger.Request(), Trigger.Response())
        self.wait_for(lambda: self.controller.phase == "waiting_chassis")
        stop = self.node.create_client(Trigger, "/task_controller/stop")
        self.wait_for(stop.service_is_ready)
        future = stop.call_async(Trigger.Request())
        self.wait_for(future.done)
        self.assertTrue(future.result().success)
        self.assertTrue(self.controller.failed)
        self.pump(0.2)
        self.assertEqual(self.command.linear.x, 0.0)

    def test_arm_modes_and_planned_chassis_stop(self):
        # Fake only the arm hardware/MoveIt endpoints; exercise real workflow callbacks and TF.
        self.arm_xyz = [0.4, 0.0, 0.3]
        self.arm_enable_calls = []
        self.arm_executions = 0
        self.latest_arm_target = None
        broadcaster = TransformBroadcaster(self.node)
        wrench_pub = self.node.create_publisher(
            WrenchStamped, "/admittance_controller/measured_wrench", 10
        )

        def feedback():
            transform = TransformStamped()
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.header.frame_id, transform.child_frame_id = "base_link", "tool0"
            (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ) = self.arm_xyz
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)
            wrench = WrenchStamped()
            wrench.header.stamp = transform.header.stamp
            wrench_pub.publish(wrench)

        def enable(request, response):
            self.arm_enable_calls.append(request.data)
            response.success = True
            return response

        def target(msg):
            pose = msg.poses[0].position
            self.arm_xyz = [pose.x, pose.y, pose.z]

        def moveit_target(msg):
            self.latest_arm_target = msg

        def execute(request, response):
            self.assertIsNotNone(self.latest_arm_target)
            target(self.latest_arm_target)
            self.arm_executions += 1
            response.success = True
            return response

        self.node.create_timer(0.02, feedback)
        self.node.create_service(SetBool, "/admittance_controller/enable", enable)
        self.node.create_service(
            Trigger, "/moveit_pose_executor/execute_latest", execute
        )
        self.node.create_subscription(
            PoseArray, "/admittance_controller/target_poses", target, 10
        )
        self.node.create_subscription(
            PoseArray, "/moveit_pose_executor/target_poses", moveit_target, 10
        )
        arm = {
            "action": "arm_move",
            "control_mode": "admittance",
            "target_pose": {
                "position_mode": "relative",
                "position": [0.01, 0.0, 0.0],
                "orientation": "current",
            },
            "after_arrival_delay_sec": 0.1,
            "arrival_tolerance": 0.001,
        }
        self.controller.steps = [
            {"action": "chassis_execute", "task": "forward"},
            {"action": "chassis_stop"},
            arm,
            {"action": "arm_poses", "poses": [[0.42, 0.0, 0.3, 0.0, 0.0, 0.0]]},
            arm,
            dict(arm, control_mode="moveit"),
        ]
        self.controller.on_start_request(Trigger.Request(), Trigger.Response())
        self.wait_for(lambda: not self.controller.active, 6)
        self.assertFalse(self.controller.failed)
        self.assertEqual(self.arm_enable_calls, [True, False, True, False])
        self.assertEqual(self.arm_executions, 2)
        self.assertAlmostEqual(self.arm_xyz[0], 0.44, places=6)

    def test_workflow_service_timeout(self):
        # A missing arm executor must fail within the step deadline without sending a target.
        self.controller.steps = [
            {
                "action": "arm_poses",
                "poses": [[0.4, 0.0, 0.3, 0.0, 0.0, 0.0]],
                "step_timeout_sec": 0.2,
            }
        ]
        self.controller.on_start_request(Trigger.Request(), Trigger.Response())
        self.wait_for(lambda: self.controller.failed, 1.0)
        self.assertEqual(self.controller.step_index, 0)

    def test_camera_workflow_mux_and_capture(self):
        mux = VelocityMux()
        self.executor.add_node(mux)
        self.addCleanup(mux.destroy_node)
        self.controller.use_velocity_mux = True
        self.controller.mux_run_ready = False
        following = [False]
        captures = []
        follow_commands = self.node.create_publisher(
            Twist, "/robot_control/line_follow/cmd_vel", 10
        )
        follow_status = self.node.create_publisher(String, "/line_follow/status", 10)

        def enable(request, response):
            following[0] = request.data
            response.success = True
            return response

        def feedback():
            msg = Twist()
            msg.linear.x = 0.01 if following[0] else 0.0
            follow_commands.publish(msg)
            follow_status.publish(
                String(
                    data=json.dumps(
                        {"state": "tracking" if following[0] else "disabled"}
                    )
                )
            )

        def capture(request, response):
            captures.append(self.x)
            response.success = True
            response.message = "saved mock HK photo"
            return response

        self.node.create_service(SetBool, "/line_follow_controller/enable", enable)
        self.node.create_service(Trigger, "/dataset_capture/capture", capture)
        self.node.create_timer(0.02, feedback)
        self.controller.steps = [
            {"action": "line_follow", "seconds": 0.2},
            {"action": "chassis_execute", "task": "forward"},
            {"action": "wait_chassis", "task": "forward"},
            {"action": "hk_capture"},
        ]
        self.controller.on_start_request(Trigger.Request(), Trigger.Response())
        self.wait_for(lambda: not self.controller.active, 6)
        self.assertFalse(self.controller.failed)
        self.assertFalse(following[0])
        self.assertEqual(mux.source, "pid")
        self.assertEqual(len(captures), 1)
        self.assertGreater(captures[0], 0.018)
        self.assertTrue(any(cmd.linear.x > 0.02 for cmd in self.commands))
        # The unselected follower continuously published zero while PID still arrived.
        self.controller.on_stop_request(Trigger.Request(), Trigger.Response())
        self.pump(0.1)
        self.assertFalse(mux.enabled)
        self.assertEqual(self.command.linear.x, 0.0)
        self.controller.require_image_receiver = True
        self.controller.steps = [{"action": "hk_capture"}]

        def disconnected(request, response):
            response.success = False
            response.message = "no image receiver connected"
            return response

        self.node.create_service(Trigger, "/hk_image_transfer/ready", disconnected)
        self.controller.on_start_request(Trigger.Request(), Trigger.Response())
        self.wait_for(lambda: self.controller.failed, 3)
        self.assertEqual(len(captures), 1)

    def test_marker_station_wait_pauses_timeout_and_requires_final_marker(self):
        mux = VelocityMux()
        self.executor.add_node(mux)
        self.addCleanup(mux.destroy_node)
        self.controller.use_velocity_mux = True
        self.controller.stop_pid_before_follow = False
        flags = {'following': False, 'state': 'tracking'}
        publisher = self.node.create_publisher(String, '/line_follow/status', 10)
        commands = self.node.create_publisher(Twist, '/robot_control/line_follow/cmd_vel', 10)
        def enable(request, response):
            flags['following'] = request.data
            response.success = True
            return response
        def feedback():
            state = flags['state'] if flags['following'] else 'disabled'
            publisher.publish(String(data=json.dumps({'state': state})))
            cmd = Twist()
            cmd.linear.x = .01 if state in ('tracking', 'stop_marker_approaching') else 0.
            commands.publish(cmd)
        self.node.create_service(SetBool, '/line_follow_controller/enable', enable)
        self.node.create_timer(.02, feedback)
        step = {'action': 'line_follow', 'until_untrusted': True, 'require_final_marker': True,
                'boundary_wait_sec': .1, 'progress_timeout_sec': .4, 'step_timeout_sec': 2.0}
        self.controller.steps = [step]
        self.controller.on_start_request(Trigger.Request(), Trigger.Response())
        self.wait_for(lambda: flags['following'], 1.5)
        self.pump(.2)
        flags['state'] = 'stop_marker_intermediate'
        self.pump(2.3)  # 超过总超时和进度超时，人工站点等待仍不能结束或失败。
        self.assertTrue(self.controller.active)
        self.assertFalse(self.controller.failed)
        self.assertEqual(self.controller.step_index, 0)
        flags['state'] = 'stop_marker_approaching'
        self.pump(.2)
        flags['state'] = 'stop_marker_final'
        self.wait_for(lambda: not self.controller.active, 1)
        self.assertFalse(self.controller.failed)
        self.assertFalse(flags['following'])
        # 未看到 T 形终点时，普通丢线不能误当成完成再启动机械臂。
        self.controller.steps = [step, {'action': 'arm_poses', 'poses': [[.1, .2, .3, 0., 0., 0.]]}]
        self.controller.step_index = 0
        flags['state'] = 'tracking'
        self.controller.on_start_request(Trigger.Request(), Trigger.Response())
        self.wait_for(lambda: flags['following'], 1.5)
        self.pump(.2)
        flags['state'] = 'insufficient_forward_span'
        self.wait_for(lambda: self.controller.failed, 1)
        self.assertEqual(self.controller.step_index, 0)

    def test_minimal_profiles_without_distance_pid(self):
        # Exercise the actual three YAML files with no PID service available.
        self.pid.terminate()
        self.pid.wait(timeout=5)
        mux = VelocityMux()
        self.executor.add_node(mux)
        self.addCleanup(mux.destroy_node)
        following = [False]
        arm_enabled = [False]
        following_at = [None]
        until_boundary = [False]
        saw_boundary_recovery = [False]
        arm_enabled_at = [None]
        arm_expected = [False]
        arm_targets = []
        broadcaster = TransformBroadcaster(self.node)
        wrench_pub = self.node.create_publisher(
            WrenchStamped, "/admittance_controller/measured_wrench", 10
        )
        follow_pub = self.node.create_publisher(
            Twist, "/robot_control/line_follow/cmd_vel", 10
        )
        status_pub = self.node.create_publisher(String, "/line_follow/status", 10)

        def feedback():
            transform = TransformStamped()
            transform.header.stamp = self.node.get_clock().now().to_msg()
            transform.header.frame_id, transform.child_frame_id = "base_link", "tool0"
            transform.transform.translation.x = 0.4
            transform.transform.translation.z = 0.3
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)
            wrench_pub.publish(WrenchStamped(header=transform.header))
            command = Twist()
            elapsed = time.monotonic() - following_at[0] if following_at[0] is not None else 0.0
            boundary = (until_boundary[0] and following[0] and
                        (0.5 < elapsed < 1.5 or elapsed >= 2.0))
            if until_boundary[0] and following[0] and 1.5 <= elapsed < 2.0:
                saw_boundary_recovery[0] = True
            command.linear.x = 0.01 if following[0] and not boundary else 0.0
            follow_pub.publish(command)
            status_pub.publish(
                String(
                    data=json.dumps(
                        {"state": "insufficient_forward_span" if boundary else
                         "tracking" if following[0] else "disabled"}
                    )
                )
            )

        def follow_enable(request, response):
            if request.data and arm_expected[0]:
                self.assertTrue(
                    arm_enabled[0], "AGV must start after admittance is enabled"
                )
                self.assertIsNotNone(arm_enabled_at[0])
                self.assertGreaterEqual(time.monotonic() - arm_enabled_at[0], 1.9)
            following[0] = request.data
            following_at[0] = time.monotonic() if request.data else None
            response.success = True
            return response

        def arm_enable(request, response):
            arm_enabled[0] = request.data
            if request.data:
                arm_enabled_at[0] = time.monotonic()
            response.success = True
            return response

        self.node.create_timer(0.02, feedback)
        self.node.create_service(
            SetBool, "/line_follow_controller/enable", follow_enable
        )
        self.node.create_service(SetBool, "/admittance_controller/enable", arm_enable)
        self.node.create_subscription(
            PoseArray, "/admittance_controller/target_poses", arm_targets.append, 10
        )
        self.controller.stop_pid_before_follow = False
        for name, with_arm, with_follow in (
            ("arm_admittance_only", True, False),
            ("line_follow_only", False, True),
            ("arm_admittance_line_follow", True, True),
        ):
            with self.subTest(profile=name):
                arm_expected[0] = with_arm
                config = WorkflowController.load_workflow(
                    Path(__file__).parents[1] / "config" / f"{name}.yaml"
                )
                self.controller.steps = config["steps"]
                until_boundary[0] = name == "line_follow_only"
                self.controller.step_index = 0
                self.controller.use_velocity_mux = with_follow
                # Arm-only must complete even without using mux services.
                self.controller.on_start_request(Trigger.Request(), Trigger.Response())
                before_targets = len(arm_targets)
                before_commands = len(self.commands)
                self.wait_for(lambda: not self.controller.active, 10)
                self.assertFalse(self.controller.failed)
                self.assertFalse(following[0])
                if name == "line_follow_only":
                    self.assertTrue(saw_boundary_recovery[0], "短暂失线不能直接结束，恢复后应继续本轮")
                self.assertEqual(len(arm_targets) - before_targets, int(with_arm))
                if with_arm:
                    target = arm_targets[-1].poses[0].position
                    self.assertAlmostEqual(target.x, 0.4)
                    self.assertAlmostEqual(target.z, 0.3)
                    self.assertTrue(arm_enabled[0])
                if with_follow:
                    self.assertTrue(
                        any(
                            msg.linear.x > 0.0
                            for msg in self.commands[before_commands:]
                        )
                    )
                else:
                    self.assertFalse(
                        any(
                            msg.linear.x > 0.0
                            for msg in self.commands[before_commands:]
                        )
                    )
                self.controller.on_stop_request(Trigger.Request(), Trigger.Response())
                self.pump(0.1)
                self.assertFalse(arm_enabled[0])
                self.assertEqual(self.command.linear.x, 0.0)


if __name__ == "__main__":
    unittest.main()
