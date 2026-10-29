#!/usr/bin/env python3
"""用本机模拟传感器验证上电档位、切换、退出及断线；不连接实际设备。"""
import os
os.environ['ROS_DOMAIN_ID'] = '181'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOG_DIR'] = '/tmp/sensor-unit-lifecycle-ros'

from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import tempfile
import threading
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from eli_cs_robot_serial.srv import SetSensorUnit


class FakeSensor:
    """模拟设备上电为 kg，35 翻转档位，普通 TCP 重连不改变硬件档位。"""
    def __init__(self):
        self.listener = socket.socket()
        self.listener.bind(('127.0.0.1', 0))
        self.listener.listen()
        self.port = self.listener.getsockname()[1]
        self.unit = 'kg'
        self.commands = []
        self.stopped = threading.Event()
        self.drop = threading.Event()
        self.connections = 0
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        connection = None
        data = bytearray()
        streaming = debug = False
        try:
            while not self.stopped.is_set():
                if connection is None:
                    if not select.select([self.listener], [], [], 0.01)[0]:
                        continue
                    connection, _ = self.listener.accept()
                    self.connections += 1
                    data.clear()
                if self.drop.is_set():
                    self.drop.clear()
                    connection.close()
                    connection = None
                    continue
                if select.select([connection], [], [], 0.01)[0]:
                    try:
                        chunk = connection.recv(4096)
                    except ConnectionError:
                        chunk = b''
                    if not chunk:
                        connection.close()
                        connection = None
                        continue
                    data.extend(chunk)
                    while len(data) >= 5:
                        assert data[:2] == b'\xaa\x55' and data[3:5] == b'\r\n'
                        code = data[2]
                        del data[:5]
                        self.commands.append(code)
                        if code == 0x35:
                            self.unit = 'N' if self.unit == 'kg' else 'kg'
                        elif code == 0x32:
                            debug = True
                        elif code == 0x31:
                            debug = False
                        elif code == 0x02:
                            streaming = True
                        elif code == 0x01:
                            streaming = False
                if debug and streaming:
                    scale = 9.80665 if self.unit == 'N' else 1.0
                    values = [value * scale for value in (1, 2, 3, 0.1, 0.2, 0.3)]
                    try:
                        connection.sendall(b'\xaa\x55\x02' + struct.pack('>6f', *values) + b'\r\n')
                    except (ConnectionError, OSError):
                        connection.close()
                        connection = None
        finally:
            if connection is not None:
                connection.close()

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=2)
        self.listener.close()


class UnitLifecycleTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('unit_lifecycle_observer')
        self.wrenches = []
        self.states = []
        self.node.create_subscription(WrenchStamped, '/sensor/wrench', self.wrenches.append, 10)
        self.node.create_subscription(String, '/sensor/unit_status',
                                      lambda msg: self.states.append(msg.data),
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.sensor = FakeSensor()
        self.log = tempfile.TemporaryFile(mode='w+')
        self.process = None

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait(self, condition, seconds=8):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if condition():
                return
        self.log.seek(0)
        self.fail('模拟传感器等待超时\n' + self.log.read())

    def start(self, target='N'):
        self.wrenches.clear()
        self.states.clear()
        installed = Path(get_package_prefix('eli_cs_robot_serial')) / 'lib/eli_cs_robot_serial/sensor_node'
        binary = Path(os.environ.get('SENSOR_NODE_BINARY', str(installed)))
        self.process = subprocess.Popen([
            str(binary), '--ros-args', '-p', 'transport:=tcp', '-p', 'host:=127.0.0.1',
            '-p', f'port:={self.sensor.port}', '-p', 'publish_wrench:=true',
            '-p', f'target_unit:="{target}"',
            '-p', 'command_delay_ms:=20', '-p', 'unit_switch_delay_ms:=20',
            '-p', 'zero_wait_ms:=20', '-p', 'stop_exit_delay_ms:=20',
        ], stdout=self.log, stderr=subprocess.STDOUT)
        self.wait(lambda: self.wrenches and any('state=READY;' in s for s in self.states))

    def stop(self, signum=signal.SIGINT):
        self.process.send_signal(signum)
        self.assertEqual(self.process.wait(timeout=5), 0)
        self.process = None
        self.pump(0.1)

    def call(self, name, service, request):
        client = self.node.create_client(service, name)
        try:
            self.wait(client.service_is_ready)
            future = client.call_async(request)
            self.wait(future.done)
            response = future.result()
            self.assertTrue(response.success, response.message)
            return response
        finally:
            self.node.destroy_client(client)

    def assert_si(self):
        self.assertAlmostEqual(self.wrenches[-1].wrench.force.x, 9.80665, places=4)
        self.assertAlmostEqual(self.wrenches[-1].wrench.torque.x, 0.980665, places=4)

    def test_start_stop_restart_and_explicit_switches(self):
        self.start()
        self.assertEqual(self.sensor.unit, 'N')
        self.assertEqual(self.sensor.commands[:5], [0x32, 0x01, 0x35, 0x30, 0x02])
        self.assertFalse(any('state=BEFORE;' in s or 'state=AFTER;' in s for s in self.states))
        self.assert_si()
        self.call('/sensor/stop', Trigger, Trigger.Request())
        self.call('/sensor/start', Trigger, Trigger.Request())
        self.wait(lambda: self.sensor.commands[-1] == 0x02)
        self.assertEqual(self.sensor.commands.count(0x35), 1)  # 不重复翻转。
        self.call('/sensor/calibrate_unit', Trigger, Trigger.Request())
        self.assertEqual(self.sensor.commands.count(0x35), 1)  # 兼容接口也不采样比较。
        self.call('/sensor/set_unit', SetSensorUnit, SetSensorUnit.Request(unit='kgf'))
        self.assertEqual(self.sensor.unit, 'kg')
        self.pump(0.2)
        self.assert_si()  # kg 档仍只在软件中换算一次到 SI。
        self.call('/sensor/set_unit', SetSensorUnit, SetSensorUnit.Request(unit='N'))
        self.assertEqual(self.sensor.unit, 'N')
        self.call('/sensor/set_unit', SetSensorUnit, SetSensorUnit.Request(unit='N'))
        self.assertEqual(self.sensor.commands.count(0x35), 3)
        self.stop()
        self.assertEqual(self.sensor.unit, 'kg')
        self.assertEqual(self.sensor.commands[-2:], [0x35, 0x31])
        self.assertEqual(self.sensor.commands.count(0x35), 4)
        self.start()  # 同一设备不重新上电，正常退出后再启动仍正确。
        self.assertEqual(self.sensor.commands.count(0x35), 5)
        self.assert_si()
        self.stop()
        self.assertEqual(self.sensor.unit, 'kg')

    def test_kg_target_needs_no_toggle(self):
        self.start('kgf')
        self.assertEqual(self.sensor.unit, 'kg')
        self.assertEqual(self.sensor.commands.count(0x35), 0)
        self.assert_si()
        self.stop(signal.SIGTERM)
        self.assertEqual(self.sensor.commands.count(0x35), 0)

    def test_reconnect_does_not_guess_or_toggle(self):
        self.start()
        self.sensor.drop.set()
        self.wait(lambda: self.sensor.connections >= 2)
        self.wait(lambda: any('device_unit=unknown' in s for s in self.states))
        self.pump(0.3)
        count = len(self.wrenches)
        self.pump(0.3)
        self.assertEqual(len(self.wrenches), count)
        self.assertEqual(self.sensor.commands.count(0x35), 1)
        response = self.node.create_client(Trigger, '/sensor/start')
        future = response.call_async(Trigger.Request())
        self.wait(future.done)
        self.assertFalse(future.result().success)
        self.node.destroy_client(response)
        self.stop()
        self.assertEqual(self.sensor.commands.count(0x35), 1)
        self.assertEqual(self.sensor.unit, 'N')  # 档位未知时退出不能盲目翻转。
        self.log.seek(0)
        self.assertIn('退出时未能恢复 kg', self.log.read())

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.sensor.close()
        self.node.destroy_node()
        rclpy.shutdown()
        self.log.close()


if __name__ == '__main__':
    unittest.main()
