#!/usr/bin/env python3
"""Read the launch terminal and publish bounded, expiring velocity commands."""

import os
import select
import signal
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from keyboard_control import KeyboardControl


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.device = self.declare_parameter('terminal_device', '').value
        topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.control = KeyboardControl(
            self.declare_parameter('linear_speed', 0.05).value,
            self.declare_parameter('angular_speed', 0.2).value,
            self.declare_parameter('key_timeout', 0.35).value)
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.stop_requested = False

    def publish(self, vx=0.0, vy=0.0, w=0.0):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = vx, vy, w
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    terminal = None
    settings = None
    previous = {}
    code = 0
    try:
        node = KeyboardTeleop()
        def stop(signum, frame):
            node.stop_requested = True
        previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
        if not node.device:
            raise RuntimeError('No keyboard terminal. Run launch in an interactive terminal.')
        terminal = os.open(node.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        if not os.isatty(terminal):
            raise RuntimeError('Keyboard input device is not a terminal')
        settings = termios.tcgetattr(terminal)
        tty.setcbreak(terminal)
        # Discard keys typed before the controller was ready.
        termios.tcflush(terminal, termios.TCIFLUSH)
        node.get_logger().info(
            'KEYBOARD READY: W/S forward/back; A/D strafe left/right; Q/E rotate left/right; '
            'Z/C reverse turns; SPACE/X stop; +/- speed; Ctrl+C exit. '
            f'linear={node.control.linear_speed:.3f} m/s, angular={node.control.angular_speed:.3f} rad/s, '
            f'no movement key for {node.control.timeout:.2f}s -> stop.')
        escape_until = 0.0
        while not node.stop_requested and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            readable, _, _ = select.select([terminal], [], [], 0.02)
            if readable:
                data = os.read(terminal, 64)
                if not data:
                    break
                # Escape sequences (e.g. arrow keys) are unsupported and stop movement.
                if b'\x1b' in data:
                    escape_until = time.monotonic() + 0.1
                    node.control.key(' ', time.monotonic())
                elif time.monotonic() < escape_until:
                    node.control.key(' ', time.monotonic())
                else:
                    for key in data.decode('ascii', errors='ignore'):
                        if key in ('\x03', '\x04'):
                            node.stop_requested = True
                            break
                        node.control.key(key, time.monotonic())
                        if key in ('+', '=', '-', '_'):
                            node.get_logger().info(
                                f'Speed: {node.control.linear_speed:.3f} m/s, '
                                f'{node.control.angular_speed:.3f} rad/s')
            if not node.stop_requested:
                node.publish(*node.control.command(time.monotonic()))
    except Exception as error:
        code = 1
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'Keyboard control failed: {error}', flush=True)
    finally:
        try:
            if node is not None and rclpy.ok():
                for _ in range(3):
                    node.publish()
                    time.sleep(0.02)
        finally:
            if terminal is not None:
                if settings is not None:
                    termios.tcsetattr(terminal, termios.TCSADRAIN, settings)
                os.close(terminal)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
