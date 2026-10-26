#!/usr/bin/env python3
"""Test automatic standalone start with synthetic ROS endpoints only."""
import importlib.util
import os
from pathlib import Path
import time
import unittest
from unittest.mock import Mock, patch

os.environ['ROS_DOMAIN_ID'] = '186'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/standalone-admittance-test'

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

spec = importlib.util.spec_from_file_location('auto_start', Path(__file__).resolve().parents[1]
                                            / 'scripts/auto_admittance_start.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class AutoStartTest(unittest.TestCase):
    def test_waits_and_starts_once_without_manual_services(self):
        rclpy.init(args=['--ros-args', '-p', 'settle_seconds:=0.0'])
        fake = Node('fake_standalone_arm')
        start = module.AutoAdmittanceStart()
        executor = SingleThreadedExecutor()
        executor.add_node(fake)
        executor.add_node(start)
        flags = {'running': False, 'active': False, 'wrench': True}
        calls = []
        task = fake.create_publisher(Bool, '/io_and_status_controller/robot_task_running',
                                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        wrench = fake.create_publisher(WrenchStamped, '/admittance_controller/measured_wrench', 10)

        def feedback():
            task.publish(Bool(data=flags['running']))
            if flags['wrench']:
                wrench.publish(WrenchStamped())

        def controllers(request, response):
            for name in ('scaled_joint_trajectory_controller', 'joint_state_broadcaster',
                         'speed_scaling_state_broadcaster', 'tcp_pose_broadcaster'):
                response.controller.append(ControllerState(
                    name=name, state='active' if flags['active'] else 'inactive'))
            return response

        def hold(request, response):
            calls.append(request)
            response.success = len(calls) > 1  # First TF lookup is not ready yet.
            response.message = 'captured' if response.success else 'TF not ready'
            return response

        fake.create_timer(0.02, feedback)
        fake.create_service(ListControllers, '/controller_manager/list_controllers', controllers)
        fake.create_service(Trigger, '/admittance_controller/hold_current_pose', hold)

        def pump(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)

        try:
            pump(1.2)
            self.assertEqual(calls, [])
            flags['active'] = True
            pump(1.2)
            self.assertEqual(calls, [])  # Robot task is stopped.
            flags['wrench'] = False
            pump(0.7)
            flags['running'] = True
            pump(0.7)
            self.assertEqual(calls, [])  # Post-bias force data is stale.
            flags['wrench'] = True
            deadline = time.monotonic() + 5
            while not start.done and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
            self.assertTrue(start.done)
            self.assertEqual(len(calls), 2)
            pump(1.2)
            self.assertEqual(len(calls), 2)  # Completed start is never repeated.
        finally:
            executor.shutdown()
            start.destroy_node()
            fake.destroy_node()
            rclpy.shutdown()


    def test_stability_delay_restarts_after_disconnect(self):
        rclpy.init(args=['--ros-args', '-p', 'wait_only:=true', '-p', 'settle_seconds:=2.0'])
        start = module.AutoAdmittanceStart()
        start.task_sub = Mock()
        start.task_sub.get_publisher_count.return_value = 1
        start.controllers = Mock()
        start.controllers.service_is_ready.return_value = False
        start.hold = Mock()
        start.started = 100.0
        start.running = True
        start.controllers_active = True
        try:
            with patch.object(module, 'monotonic') as clock:
                def tick(at):
                    clock.return_value = at
                    start.checked = at
                    start.tick()
                tick(100.0)
                tick(101.5)
                self.assertFalse(start.done)
                start.on_task(Bool(data=False))
                self.assertIsNone(start.ready_since)
                start.on_task(Bool(data=True))
                tick(102.0)
                tick(103.9)
                self.assertFalse(start.done)
                tick(104.0)
                self.assertTrue(start.done)
                self.assertTrue(start.success)
                start.hold.call_async.assert_not_called()  # Startup gate never enables motion.
        finally:
            start.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
