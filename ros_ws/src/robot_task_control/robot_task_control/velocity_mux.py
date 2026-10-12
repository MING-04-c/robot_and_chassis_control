#!/usr/bin/env python3
"""Give one local controller ownership of the chassis velocity output."""
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool


class VelocityMux(Node):
    def __init__(self):
        super().__init__('chassis_velocity_mux')
        self.timeout = float(self.declare_parameter('command_timeout_sec', 0.1).value)
        if not 0.02 <= self.timeout <= 0.5:
            raise ValueError('command_timeout_sec must be 0.02..0.5')
        self.enabled = True
        self.source = 'pid'
        self.command = None
        self.received = None
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        for source in ('pid', 'line_follow'):
            self.create_subscription(
                Twist, f'/robot_control/{source}/cmd_vel',
                lambda message, source=source: self.on_command(source, message), 10)
        self.create_service(SetBool, '~/line_follow_mode', self.on_mode)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.create_timer(0.02, self.tick)

    def reset(self):
        self.command = self.received = None
        self.publisher.publish(Twist())

    def on_command(self, source, message):
        if self.enabled and source == self.source:
            self.command = message
            self.received = monotonic()

    def on_mode(self, request, response):
        self.source = 'line_follow' if request.data else 'pid'
        self.reset()
        response.success = True
        response.message = self.source
        return response

    def on_enable(self, request, response):
        self.enabled = request.data
        self.reset()
        response.success = True
        response.message = 'enabled' if self.enabled else 'stopped'
        return response

    def tick(self):
        command = Twist()
        if (self.enabled and self.received is not None
                and monotonic() - self.received <= self.timeout):
            command = self.command
        self.publisher.publish(command)


def main():
    rclpy.init()
    node = VelocityMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.reset()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
