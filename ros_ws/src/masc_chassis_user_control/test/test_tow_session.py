#!/usr/bin/env python3
"""Exercise Ctrl+C cleanup against fake ROS services, never real hardware."""

import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from masc_chassis_msgs.msg import ChassisStatus
from masc_chassis_msgs.srv import SetTowMode
from rclpy.node import Node


def main():
    rclpy.init()
    harness = Node('tow_test_harness')
    requests = []
    mode = ChassisStatus.CHASSIS_STATE_COMM_CONTROL
    confirm_enter = True
    confirm_exit = True
    delay_entry = False
    def handle(request, response):
        nonlocal mode
        requests.append(request.enable)
        if request.enable and confirm_enter and (not delay_entry or requests.count(True) >= 2):
            mode = ChassisStatus.CHASSIS_STATE_TOW
        elif not request.enable and confirm_exit:
            mode = ChassisStatus.CHASSIS_STATE_COMM_CONTROL
        response.success = True
        return response
    harness.create_service(SetTowMode, '/masc_chassis/set_tow_mode', handle)
    publisher = harness.create_publisher(ChassisStatus, '/masc_chassis/chassis_status', 10)
    harness.create_timer(0.05, lambda: publisher.publish(ChassisStatus(chassis_state=mode)))
    script = Path(__file__).resolve().parents[1] / 'scripts/tow_mode_node.py'
    def spin_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(harness, timeout_sec=0.05)
        assert predicate(), 'timeout waiting for fake tow session'
    try:
        for early_cancel, reject_exit, delayed in ((False, False, False), (True, False, False),
                                                  (False, True, False), (False, False, True)):
            requests.clear()
            mode = ChassisStatus.CHASSIS_STATE_COMM_CONTROL
            delay_entry = delayed
            confirm_enter = not early_cancel
            confirm_exit = not reject_exit
            with tempfile.TemporaryFile(mode='w+') as output:
                child = subprocess.Popen([
                    sys.executable, str(script), '--ros-args', '-p', 'start_driver:=false',
                    '-p', 'transition_timeout_sec:=4.0'], stdout=output, stderr=output)
                try:
                    spin_until(lambda: requests.count(True) >= (2 if delayed else 1), 8)
                    # Give normal entry feedback time to reach the child.
                    until = time.monotonic() + (0.1 if early_cancel else 0.4)
                    while time.monotonic() < until:
                        rclpy.spin_once(harness, timeout_sec=0.05)
                    child.send_signal(signal.SIGINT)
                    spin_until(lambda: child.poll() is not None, 6)
                    first_exit = requests.index(False)
                    assert all(requests[:first_exit]) and not any(requests[first_exit:]), requests
                    assert first_exit == (2 if delayed else 1), requests
                    assert 1 <= requests.count(False) <= 3, requests
                    assert child.returncode == (1 if reject_exit else 0), child.returncode
                    output.seek(0)
                    log = output.read()
                    assert ('TOW EXIT NOT CONFIRMED' if reject_exit else
                            'Tow mode exit confirmed') in log, log
                except Exception:
                    output.seek(0)
                    print(output.read(), flush=True)
                    raise
                finally:
                    if child.poll() is None:
                        child.kill()
                        child.wait()
        print('PASS: Ctrl+C exits tow, cancellation during entry cleans up, unconfirmed exit reports failure, unconfirmed entry retries; fake services only')
    finally:
        harness.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
