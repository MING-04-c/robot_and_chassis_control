#!/usr/bin/env python3
"""Drive the chassis from camer_diver observations; stop on lost/stale input."""

import json
import time
from dataclasses import replace

import rclpy
from geometry_msgs.msg import PolygonStamped, Twist
from masc_chassis_msgs.msg import ChassisStatus, CommStatus
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool

from line_follow_control import ControlConfig, line_command, slew


class LineFollowNode(Node):
    def __init__(self):
        super().__init__('line_follow_controller')
        fixed = ParameterDescriptor(read_only=True)
        defaults = ControlConfig()
        values = {name: self.declare_parameter(name, value).value
                  for name, value in vars(defaults).items()}
        self.config = ControlConfig(**values)
        self.config.validate()
        self.enabled = self.declare_parameter('enabled', True).value
        self.dry_run = self.declare_parameter('dry_run', False, fixed).value
        self.input_timeout = self.declare_parameter('input_timeout_sec', 0.5, fixed).value
        self.comm_timeout = self.declare_parameter('comm_timeout_sec', 2.5, fixed).value
        self.control_period = self.declare_parameter('control_period_sec', 0.02, fixed).value
        self.reacquire_frames = self.declare_parameter('reacquire_frames', 3, fixed).value
        if not 0.01 <= self.control_period <= 0.05:
            raise ValueError('control_period_sec must be 0.01..0.05 (driver timeout is 0.1 s)')
        if not 0.1 <= self.input_timeout <= 1.0 or not 1.0 <= self.comm_timeout <= 5.0:
            raise ValueError('input timeout must be 0.1..1; comm timeout must be 1..5')
        if self.reacquire_frames < 1:
            raise ValueError('reacquire_frames must be >= 1')
        topics = {}
        for name, default in (
            ('line_status_topic', '/camera/line/status'),
            ('centerline_topic', '/camera/line/centerline'),
            ('cmd_vel_topic', '/cmd_vel'),
            ('comm_status_topic', '/masc_chassis/comm_status'),
            ('chassis_status_topic', '/masc_chassis/chassis_status'),
        ):
            topics[name] = self.declare_parameter(name, default, fixed).value
        if self.dry_run:
            topics['cmd_vel_topic'] = '/line_follow/cmd_vel'
        self.status = None
        self.status_time = self.path_time = self.comm_time = self.chassis_time = None
        self.path_header = None
        self.path_valid = False
        self.connected = False
        self.chassis_allowed = False
        self.good_frames = 0
        self.previous_v = self.previous_w = 0.0
        self.last_tick = time.monotonic()
        self.reason = None
        self.cmd_pub = self.create_publisher(Twist, topics['cmd_vel_topic'], 10)
        self.state_pub = self.create_publisher(String, '/line_follow/status', 10)
        self.create_subscription(String, topics['line_status_topic'], self.on_status, 10)
        self.create_subscription(PolygonStamped, topics['centerline_topic'],
                                 self.on_path, qos_profile_sensor_data)
        self.create_subscription(CommStatus, topics['comm_status_topic'], self.on_comm, 10)
        self.create_subscription(ChassisStatus, topics['chassis_status_topic'],
                                 self.on_chassis, 10)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(self.control_period, self.tick)
        self.get_logger().info(
            f'Line following ready: enabled={self.enabled}, '
            f'forward_speed={self.config.forward_speed:.3f} m/s; '
            f'{topics["line_status_topic"]} -> {topics["cmd_vel_topic"]}')

    def on_parameters(self, parameters):
        updates = {}
        enabled = self.enabled
        try:
            for parameter in parameters:
                if parameter.name in vars(self.config):
                    updates[parameter.name] = parameter.value
                elif parameter.name == 'enabled':
                    enabled = parameter.value
            candidate = replace(self.config, **updates)
            candidate.validate()
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        self.config = candidate
        if enabled != self.enabled:
            self.good_frames = 0
        self.enabled = enabled
        if not enabled or candidate.forward_speed == 0.0:
            self.stop()
        return SetParametersResult(successful=True)

    def on_enable(self, request, response):
        results = self.set_parameters([rclpy.parameter.Parameter('enabled', value=request.data)])
        response.success = results[0].successful
        response.message = 'enabled' if self.enabled else 'disabled'
        return response

    def on_status(self, message):
        try:
            status = json.loads(message.data)
            _, _, reason = line_command(status, self.config)
        except (ValueError, TypeError):
            status, reason = None, 'invalid_json'
        self.status = status
        self.status_time = time.monotonic()
        if reason == 'tracking':
            self.good_frames = min(self.reacquire_frames, self.good_frames + 1)
        else:
            self.good_frames = 0
            self.stop()

    def on_path(self, message):
        self.path_header = message.header
        self.path_time = time.monotonic()
        self.path_valid = bool(message.polygon.points)
        if not self.path_valid:
            self.good_frames = 0
            self.stop()

    def on_comm(self, message):
        self.connected = message.comm_state == CommStatus.COMM_STATE_CONNECTED
        self.comm_time = time.monotonic()
        if not self.connected and not self.dry_run:
            self.good_frames = 0
            self.stop()

    def on_chassis(self, message):
        # Park is allowed: the driver's auto_unlock_on_cmd releases it.
        self.chassis_allowed = (
            message.chassis_state in (
                ChassisStatus.CHASSIS_STATE_COMM_CONTROL, ChassisStatus.CHASSIS_STATE_PARK)
            and message.chassis_fault == 0
            and message.obstacle_status_flags == 0
            and not message.excessive_tilt
            and not message.manual_charging
            and not message.dock_charging
        )
        self.chassis_time = time.monotonic()
        if not self.chassis_allowed and not self.dry_run:
            self.good_frames = 0
            self.stop()

    def blocked_reason(self, now):
        if not self.enabled:
            return 'disabled'
        if not self.dry_run and (
                not self.connected or self.comm_time is None
                or now - self.comm_time > self.comm_timeout):
            return 'chassis_disconnected'
        if not self.dry_run and (
                not self.chassis_allowed or self.chassis_time is None
                or now - self.chassis_time > self.input_timeout):
            return 'chassis_not_ready'
        for receipt in (self.status_time, self.path_time):
            if receipt is None or now - receipt > self.input_timeout:
                return 'camera_timeout'
        if not self.path_valid or self.path_header is None:
            return 'empty_centerline'
        if self.path_header.frame_id == 'synthetic_demo' and not self.dry_run:
            return 'synthetic_input'
        stamp = self.path_header.stamp
        age = (self.get_clock().now().nanoseconds - stamp.sec * 1_000_000_000
               - stamp.nanosec) / 1e9
        if age < -0.1 or age > self.input_timeout:
            return 'stale_image_stamp'
        return None

    def stop(self):
        self.previous_v = self.previous_w = 0.0
        self.cmd_pub.publish(Twist())

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, self.control_period * 2.0)
        self.last_tick = now
        reason = self.blocked_reason(now)
        v = w = 0.0
        if reason is None:
            v, w, reason = line_command(self.status, self.config)
            if reason == 'tracking' and self.good_frames < self.reacquire_frames:
                v = w = 0.0
                reason = 'reacquiring'
        else:
            self.good_frames = 0
        if reason == 'tracking':
            v = slew(self.previous_v, v, self.config.linear_acceleration, dt)
            w = slew(self.previous_w, w, self.config.angular_acceleration, dt)
        else:
            v = w = 0.0
        self.previous_v, self.previous_w = v, w
        command = Twist()
        command.linear.x, command.angular.z = v, w
        self.cmd_pub.publish(command)
        if reason != self.reason:
            self.reason = reason
            self.get_logger().info(f'Line following state: {reason}')
        if not hasattr(self, 'last_state') or now - self.last_state >= 0.5:
            self.last_state = now
            self.state_pub.publish(String(data=json.dumps({
                'state': reason, 'enabled': self.enabled,
                'forward_speed_mps': self.config.forward_speed,
                'command_linear_x_mps': v, 'command_angular_z_radps': w,
            }, separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LineFollowNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
