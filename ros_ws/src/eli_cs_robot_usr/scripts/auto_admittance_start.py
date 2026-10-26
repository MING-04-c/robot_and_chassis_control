#!/usr/bin/env python3
"""Enable admittance once at the current TCP pose after startup data is ready."""
import math
import sys
from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class AutoAdmittanceStart(Node):
    def __init__(self):
        super().__init__('auto_admittance_start')
        self.done = False
        self.success = False
        self.wait_only = bool(self.declare_parameter('wait_only', False).value)
        self.settle_seconds = float(self.declare_parameter('settle_seconds', 5.0).value)
        if not math.isfinite(self.settle_seconds) or self.settle_seconds < 0:
            raise ValueError('settle_seconds must be finite and nonnegative')
        self.wait_for_all_controllers = bool(self.declare_parameter('wait_for_all_controllers', False).value)
        self.ready_since = None
        self.started = monotonic()
        self.running = False
        self.last_wrench = float('-inf')
        self.checked = float('-inf')
        self.controllers_active = False
        self.check_future = None
        self.hold_future = None
        self.last_log = float('-inf')
        self.check_requested = float('-inf')
        self.hold_requested = float('-inf')
        self.task_sub = self.create_subscription(
            Bool, '/io_and_status_controller/robot_task_running', self.on_task,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.wrench_sub = self.create_subscription(
            WrenchStamped, '/admittance_controller/measured_wrench', self.on_wrench,
            qos_profile_sensor_data)
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.hold = self.create_client(Trigger, '/admittance_controller/hold_current_pose')
        self.timer = self.create_timer(0.2, self.tick)

    def on_task(self, message):
        self.running = message.data
        if not self.running:
            self.ready_since = None

    def on_wrench(self, message):
        del message
        self.last_wrench = monotonic()

    def on_controllers(self, future):
        try:
            states = {item.name: item.state for item in future.result().controller}
            required = ('scaled_joint_trajectory_controller', 'joint_state_broadcaster',
                        'speed_scaling_state_broadcaster', 'tcp_pose_broadcaster')
            if self.wait_for_all_controllers:
                required += ('io_and_status_controller', 'force_torque_sensor_broadcaster')
            self.controllers_active = all(states.get(name) == 'active' for name in required)
            if self.wait_for_all_controllers:
                self.controllers_active = self.controllers_active and all(
                    states.get(name) == 'inactive'
                    for name in ('forward_position_controller', 'freedrive_controller'))
            self.checked = monotonic()
            if not self.controllers_active:
                self.ready_since = None
        except Exception as error:
            self.controllers_active = False
            self.get_logger().warn(f'Controller readiness query failed: {error}')

    def tick(self):
        if self.done:
            return
        now = monotonic()
        if self.wait_only and now - self.started > 120.0:
            self.get_logger().error('Robot startup timed out before a stable connection was confirmed')
            self.done = True
            return
        if self.hold_future is not None:
            if not self.hold_future.done():
                if now - self.hold_requested > 5.0:
                    # Do not resend an enable request whose outcome is unknown.
                    self.get_logger().error('Automatic enable timed out; check admittance node logs')
                    self.done = True
                return
            try:
                response = self.hold_future.result()
                if response.success:
                    self.get_logger().info('Admittance automatically enabled at the current TCP pose')
                    self.success = True
                    self.done = True
                    return
                self.get_logger().warn(f'Waiting for current TCP pose: {response.message}')
            except Exception as error:
                self.get_logger().error(f'Automatic enable failed: {error}')
                self.done = True
                return
            self.hold_future = None
            self.hold_requested = now  # Retry a rejected TF lookup after one second.
        if self.check_future is not None and not self.check_future.done() and now - self.check_requested > 3.0:
            self.controllers.remove_pending_request(self.check_future)
            self.check_future.cancel()
        if (self.check_future is None or self.check_future.done()) and now - self.check_requested >= 1.0:
            if self.controllers.service_is_ready():
                self.check_requested = now
                self.check_future = self.controllers.call_async(ListControllers.Request())
                self.check_future.add_done_callback(self.on_controllers)
        ready = (self.running and self.task_sub.get_publisher_count() > 0
                 and self.controllers_active and now - self.checked < 2.5
                 and (self.wait_only or now - self.last_wrench < 0.5))
        if not ready:
            self.ready_since = None
        elif self.ready_since is None:
            self.ready_since = now
            self.get_logger().info(f'Robot ready; waiting {self.settle_seconds:.1f}s for startup to settle')
        if ready and now - self.ready_since < self.settle_seconds:
            return
        if ready and self.wait_only:
            self.get_logger().info('Robot connection stable; starting the control system')
            self.success = True
            self.done = True
            return
        if ready and self.hold.service_is_ready() and now - self.hold_requested >= 1.0:
            self.hold_requested = now
            self.hold_future = self.hold.call_async(Trigger.Request())
        elif now - self.last_log >= 5.0:
            self.last_log = now
            self.get_logger().info('Waiting for external control and active controllers' if self.wait_only
                                   else 'Waiting for external control, active controllers and force bias data')


def main():
    rclpy.init()
    node = AutoAdmittanceStart()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.success else 1


if __name__ == '__main__':
    sys.exit(main())
