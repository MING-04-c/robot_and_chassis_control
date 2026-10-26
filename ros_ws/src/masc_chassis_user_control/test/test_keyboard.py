#!/usr/bin/env python3
"""Pure mapping checks and real launch with a pseudo-terminal and isolated topic."""

import math
import os
import pty
import signal
import subprocess
import sys
import tempfile
import termios
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from keyboard_control import KeyboardControl


def pure_checks():
    control = KeyboardControl()
    for key, expected in {'w': (0.05, 0.0, 0.0), 's': (-0.05, 0.0, 0.0),
                          'a': (0.0, 0.05, 0.0), 'd': (0.0, -0.05, 0.0),
                          'q': (0.0, 0.0, 0.2), 'e': (0.0, 0.0, -0.2)}.items():
        control.key(key, 1.0)
        assert control.command(1.1) == expected
        assert control.command(1.36) == (0.0, 0.0, 0.0)
    control.key('w', 2)
    control.key(' ', 2.01)
    assert control.command(2.02) == (0, 0, 0)
    control.key('+', 3)
    assert control.command(3) == (0, 0, 0)
    assert abs(control.linear_speed - 0.055) < 1e-9
    for _ in range(100):
        control.key('+', 3)
    assert control.linear_speed == 0.3 and control.angular_speed == 0.8
    for value in (-1, math.nan, math.inf):
        try:
            KeyboardControl(linear_speed=value)
        except ValueError:
            pass
        else:
            raise AssertionError('invalid speed accepted')


def main():
    pure_checks()
    rclpy.init()
    harness = Node('keyboard_test_harness')
    messages = []
    harness.create_subscription(Twist, '/keyboard_test/cmd_vel', messages.append, 10)
    master, slave = pty.openpty()
    original = termios.tcgetattr(slave)
    def wait(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(harness, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError('timed out waiting for keyboard output')
    with tempfile.TemporaryFile(mode='w+') as log:
        child = subprocess.Popen([
            'ros2', 'launch', 'masc_chassis_user_control', 'keyboard_teleop.launch.py',
            'start_driver:=false', 'cmd_vel_topic:=/keyboard_test/cmd_vel',
        ], stdin=slave, stdout=log, stderr=log, start_new_session=True)
        try:
            wait(lambda: bool(messages))
            assert messages[-1].linear.x == 0
            os.write(master, b'w')
            wait(lambda: messages[-1].linear.x > 0)
            assert messages[-1].linear.x == 0.05
            wait(lambda: messages[-1].linear.x == 0, 2)
            os.write(master, b'a')
            wait(lambda: messages[-1].linear.y > 0)
            assert messages[-1].linear.x == 0 and messages[-1].angular.z == 0
            os.write(master, b'd')
            wait(lambda: messages[-1].linear.y < 0)
            assert messages[-1].linear.x == 0 and messages[-1].angular.z == 0
            wait(lambda: messages[-1].linear.y == 0, 2)
            os.write(master, b'q')
            wait(lambda: messages[-1].angular.z > 0)
            assert messages[-1].linear.x == 0 and messages[-1].linear.y == 0
            os.write(master, b'e')
            wait(lambda: messages[-1].angular.z < 0)
            assert messages[-1].linear.x == 0 and messages[-1].linear.y == 0
            os.write(master, b' ')
            wait(lambda: messages[-1].angular.z == 0, 2)
            # Split arrow escape must not become uppercase A -> strafe left.
            os.write(master, b'\x1b')
            until = time.monotonic() + 0.04
            while time.monotonic() < until:
                rclpy.spin_once(harness, timeout_sec=0.01)
            mark = len(messages)
            os.write(master, b'[A')
            until = time.monotonic() + 0.3
            while time.monotonic() < until:
                rclpy.spin_once(harness, timeout_sec=0.02)
            assert all(m.linear.x == 0 and m.linear.y == 0 and m.angular.z == 0 for m in messages[mark:])
            os.write(master, b's')
            wait(lambda: messages[-1].linear.x < 0)
            child.send_signal(signal.SIGINT)
            wait(lambda: child.poll() is not None, 8)
            # Drain final zero commands emitted during shutdown.
            until = time.monotonic() + 0.3
            while time.monotonic() < until:
                rclpy.spin_once(harness, timeout_sec=0.02)
            assert messages[-1].linear.x == 0 and messages[-1].linear.y == 0 and messages[-1].angular.z == 0
            assert termios.tcgetattr(slave) == original, 'terminal settings not restored'
            print('PASS: real keyboard launch with no driver; forward/reverse, lateral A/D without yaw, rotation Q/E without translation, timeout, space stop, split arrow rejection, Ctrl+C zero and terminal restoration')
        except Exception:
            log.seek(0)
            print(log.read(), flush=True)
            raise
        finally:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
            os.close(master)
            os.close(slave)
            harness.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
