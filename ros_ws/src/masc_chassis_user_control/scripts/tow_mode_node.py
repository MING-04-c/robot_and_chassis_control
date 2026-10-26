#!/usr/bin/env python3
"""Own the tow session and keep its driver alive until exit has been requested."""

import os
import signal
import subprocess
import time

import rclpy
from masc_chassis_msgs.msg import ChassisStatus
from masc_chassis_msgs.srv import SetTowMode
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class TowSession(Node):
    def __init__(self):
        super().__init__('tow_mode_session')
        self.start_driver = self.declare_parameter('start_driver', True).value
        self.driver_params = self.declare_parameter('driver_params', '').value
        self.timeout = float(self.declare_parameter('transition_timeout_sec', 8.0).value)
        if not 1.0 <= self.timeout <= 10.0:
            raise ValueError('transition_timeout_sec must be 1..10')
        self.client = self.create_client(SetTowMode, '/masc_chassis/set_tow_mode')
        self.state = None
        self.state_time = 0.0
        self.last_status = None
        self.create_subscription(ChassisStatus, '/masc_chassis/chassis_status', self.on_status, 10)
        self.driver = None
        self.attempted = False
        self.stop_requested = False

    def on_status(self, message):
        self.last_status = message
        if message.chassis_state != self.state:
            self.get_logger().info(f'Chassis feedback: {self.describe(message)}')
        self.state = message.chassis_state
        self.state_time = time.monotonic()

    @staticmethod
    def describe(message):
        if message is None:
            return 'no chassis feedback'
        names = {0: 'unknown', 1: 'estop', 2: 'tow', 3: 'fault', 4: 'self_check',
                 5: 'park', 6: 'remote_control', 7: 'comm_control'}
        return (f'state={message.chassis_state}({names.get(message.chassis_state, "invalid")}), '
                f'brake={message.brake_active}, motion_mode={message.current_motion_mode}, '
                f'fault=0x{message.chassis_fault:08x}, raw=0x{message.system_status_flags:08x}')

    def open_driver(self):
        if not self.start_driver:
            self.get_logger().warning(
                'External driver must use command.transmit_enabled:=false for tow-only operation')
            return
        if self.client.wait_for_service(timeout_sec=0.8):
            raise RuntimeError('Driver already running; use start_driver:=false with the existing driver')
        # A separate process group keeps launch SIGINT away from the driver until cleanup.
        self.driver = subprocess.Popen([
            'ros2', 'run', 'masc_chassis_ros_driver', 'masc_chassis_ros_driver_node',
            '--ros-args', '--params-file', self.driver_params,
            '-r', '__node:=masc_chassis', '-r', '~/odom:=/wheel_odom',
            '-r', '~/cmd_vel:=/tow_session/unused_cmd_vel',
            '-p', 'auto_lock.enabled:=false',
            '-p', 'command.transmit_enabled:=false',
        ], start_new_session=True)

    def transition(self, enable):
        deadline = time.monotonic() + self.timeout
        future = None
        first_sent = None
        accepted = False
        attempts = 0
        retry_at = 0.0
        last_diagnostic = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if enable and self.stop_requested:
                return False
            if self.driver is not None and self.driver.poll() is not None:
                self.get_logger().error('Owned driver exited before tow transition completed')
                return False
            if (future is None and attempts < 3 and now >= retry_at
                    and self.client.service_is_ready()):
                if enable:
                    self.attempted = True  # A lost ACK can still mean tow was enabled.
                if first_sent is None:
                    first_sent = now
                attempts += 1
                retry_at = now + 1.0
                self.get_logger().info(f'Tow request enable={enable}, attempt {attempts}/3')
                future = self.client.call_async(SetTowMode.Request(enable=enable))
            rclpy.spin_once(self, timeout_sec=0.1)
            if future is not None and future.done():
                result = future.result()
                accepted = accepted or (result is not None and result.success)
                self.get_logger().info('Tow service response: ' + (
                    f'success={result.success}, message={result.message}' if result is not None
                    else 'no response'))
                future = None
            if accepted and first_sent is not None and self.state_time > first_sent:
                if enable and self.state == ChassisStatus.CHASSIS_STATE_TOW:
                    self.get_logger().info('Tow mode confirmed. Hold the vehicle; Ctrl+C exits tow mode.')
                    return True
                if not enable and self.state not in (
                        ChassisStatus.CHASSIS_STATE_TOW, ChassisStatus.CHASSIS_STATE_UNKNOWN):
                    self.get_logger().info('Tow mode exit confirmed by chassis feedback.')
                    return True
            if now - last_diagnostic >= 1.0:
                last_diagnostic = now
                age = max(0.0, time.monotonic() - self.state_time) if self.state_time else None
                self.get_logger().info(f'Waiting for tow enable={enable}; '
                                       f'{self.describe(self.last_status)}, feedback_age={age}')
        self.get_logger().error(f'Tow transition timeout: enable={enable}, '
                                f'attempts={attempts}, send_accepted={accepted}; '
                                f'{self.describe(self.last_status)}')
        return False

    def close_driver(self):
        if self.driver is None:
            return
        try:
            os.killpg(self.driver.pid, signal.SIGINT)
            self.driver.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(self.driver.pid, signal.SIGTERM)
            try:
                self.driver.wait(timeout=1)
            except subprocess.TimeoutExpired:
                os.killpg(self.driver.pid, signal.SIGKILL)
                self.driver.wait()
        except ProcessLookupError:
            pass


def main(args=None):
    # Keep ROS communication alive after SIGINT so the exit request can complete.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    session = TowSession()
    def stop(signum, frame):
        session.stop_requested = True
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    code = 0
    try:
        session.open_driver()
        if not session.transition(True):
            if not session.stop_requested:
                raise RuntimeError('Tow entry not confirmed; check driver/CAN/chassis feedback')
        while not session.stop_requested:
            rclpy.spin_once(session, timeout_sec=0.1)
            if session.driver is not None and session.driver.poll() is not None:
                raise RuntimeError('Owned chassis driver unexpectedly exited')
    except Exception as error:
        session.get_logger().error(str(error))
        code = 1
    finally:
        if session.attempted:
            session.get_logger().info('Requesting tow exit before closing the driver...')
            try:
                if not session.transition(False):
                    session.get_logger().error('TOW EXIT NOT CONFIRMED. Check chassis state before leaving vehicle.')
                    code = 1
            except Exception as error:
                session.get_logger().error(f'Tow exit failed: {error}')
                code = 1
        session.close_driver()
        session.destroy_node()
        rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
