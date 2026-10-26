#!/usr/bin/env python3
"""Check device readiness, automatic admittance and chassis startup order."""
import os
from pathlib import Path
import time
import unittest

os.environ["ROS_DOMAIN_ID"] = "188"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["ROS_LOG_DIR"] = "/tmp/auto-admittance-test-logs"

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseArray, TransformStamped, Twist, WrenchStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
from robot_task_control.workflow_controller import WorkflowController
from robot_task_control.velocity_mux import VelocityMux


class DeviceStartupTest(unittest.TestCase):
    def test_all_devices_before_any_task(self):
        config = (
            Path(__file__).resolve().parents[1]
            / "config/arm_admittance_line_follow.yaml"
        )
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                f"workflow_file:={config}",
                "-p",
                "auto_start:=true",
                "-p",
                "require_arm_ready:=true",
                "-p",
                "check_devices_on_startup:=true",
                "-p",
                "use_velocity_mux:=true",
                "-p",
                "stop_pid_before_follow:=false",
                "-p",
                "enabled_on_startup:=false",
            ]
        )
        executor = SingleThreadedExecutor()
        fake = Node("fake_arm_endpoints")
        controller = WorkflowController()
        executor.add_node(fake)
        executor.add_node(controller)
        mux = VelocityMux()
        executor.add_node(mux)
        flags = {
            "controllers": False,
            "task": False,
            "wrench": False,
            "line_ready": False,
            "following": False,
        }
        calls, targets, commands, follow_calls = [], [], [], []
        fake.create_subscription(Twist, "/cmd_vel", commands.append, 10)
        follow_command = fake.create_publisher(
            Twist, "/robot_control/line_follow/cmd_vel", 10
        )
        follow_status = fake.create_publisher(String, "/line_follow/status", 10)
        tf = TransformBroadcaster(fake)
        wrench = fake.create_publisher(
            WrenchStamped, "/admittance_controller/measured_wrench", 10
        )
        task = fake.create_publisher(
            Bool,
            "/io_and_status_controller/robot_task_running",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        def list_controllers(request, response):
            for name in (
                "scaled_joint_trajectory_controller",
                "joint_state_broadcaster",
                "speed_scaling_state_broadcaster",
                "tcp_pose_broadcaster",
            ):
                response.controller.append(
                    ControllerState(
                        name=name,
                        state="active" if flags["controllers"] else "inactive",
                    )
                )
            return response

        def enable(request, response):
            calls.append(request.data)
            response.success = True
            return response

        def ready(request, response):
            response.success = flags["line_ready"]
            response.message = "ready" if response.success else "camera_timeout"
            return response

        def follow_enable(request, response):
            if request.data:
                self.assertEqual(calls, [True])
                self.assertTrue(targets)
            follow_calls.append(request.data)
            flags["following"] = request.data
            response.success = True
            return response

        fake.create_service(Trigger, "/line_follow_controller/ready", ready)
        fake.create_service(SetBool, "/line_follow_controller/enable", follow_enable)

        def feedback():
            transform = TransformStamped()
            transform.header.stamp = fake.get_clock().now().to_msg()
            transform.header.frame_id, transform.child_frame_id = "base_link", "tool0"
            transform.transform.translation.x = 0.4
            transform.transform.translation.z = 0.3
            transform.transform.rotation.w = 1.0
            tf.sendTransform(transform)
            task.publish(Bool(data=flags["task"]))
            command = Twist()
            command.linear.x = 0.01 if flags["following"] else 0.0
            follow_command.publish(command)
            follow_status.publish(
                String(
                    data=(
                        '{"state":"tracking"}'
                        if flags["following"]
                        else '{"state":"disabled"}'
                    )
                )
            )
            if flags["wrench"]:
                wrench.publish(WrenchStamped(header=transform.header))

        fake.create_service(
            ListControllers, "/controller_manager/list_controllers", list_controllers
        )
        fake.create_service(SetBool, "/admittance_controller/enable", enable)
        fake.create_subscription(
            PoseArray, "/admittance_controller/target_poses", targets.append, 10
        )
        fake.create_timer(0.02, feedback)

        def pump(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)

        try:
            flags["wrench"] = True
            pump(1.3)
            self.assertEqual(calls, [])
            flags["controllers"] = True
            pump(1.3)
            self.assertEqual(calls, [])  # External robot task still stopped.
            flags["wrench"] = False
            pump(0.7)
            flags["task"] = True
            pump(0.7)
            self.assertEqual(calls, [])  # No fresh post-bias wrench.
            flags["wrench"] = True
            pump(1.0)
            self.assertEqual(
                calls, []
            )  # Arm ready, camera/chassis check still failing.
            self.assertFalse(mux.enabled)
            self.assertFalse(any(msg.linear.x for msg in commands))
            flags["line_ready"] = True
            pump(0.5)
            self.assertEqual(calls, [])  # Not stable for one second yet.
            flags["line_ready"] = False
            pump(0.4)
            self.assertEqual(calls, [])
            flags["line_ready"] = True
            deadline = time.monotonic() + 8
            while controller.active and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
            self.assertFalse(controller.active)
            self.assertFalse(controller.failed)
            self.assertEqual(calls, [True])
            self.assertEqual(len(targets), 1)
            self.assertAlmostEqual(targets[0].poses[0].position.x, 0.4)
            self.assertAlmostEqual(targets[0].poses[0].position.z, 0.3)
            self.assertTrue(controller.devices_checked)
            self.assertEqual(follow_calls, [True, False])
            self.assertTrue(any(msg.linear.x > 0 for msg in commands))
            self.assertEqual(commands[-1].linear.x, 0.0)
            controller.on_stop_request(Trigger.Request(), Trigger.Response())
            pump(0.3)
            self.assertEqual(calls, [True, False])
            self.assertFalse(controller.active)
        finally:
            executor.shutdown()
            controller.destroy_node()
            mux.destroy_node()
            fake.destroy_node()
            rclpy.shutdown()

    def test_device_timeout_does_not_enter_task(self):
        config = (
            Path(__file__).resolve().parents[1]
            / "config/arm_admittance_line_follow.yaml"
        )
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                f"workflow_file:={config}",
                "-p",
                "auto_start:=true",
                "-p",
                "require_arm_ready:=true",
                "-p",
                "check_devices_on_startup:=true",
                "-p",
                "device_check_timeout_sec:=0.1",
            ]
        )
        controller = WorkflowController()
        try:
            controller.devices_check_started = time.monotonic() - 1.0
            controller.advance()
            self.assertTrue(controller.failed)
            self.assertFalse(controller.active)
            self.assertFalse(controller.devices_checked)
            self.assertIsNone(controller.arm_control_mode)
            self.assertEqual(controller.step_index, 0)
        finally:
            controller.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
