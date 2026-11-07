#!/usr/bin/env python3
"""仅在本机隔离ROS域模拟TCP测厚仪；不启动机器人、底盘或真实仪器。"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
from time import monotonic
import unittest

os.environ['ROS_DOMAIN_ID'] = '193'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/detactor-tcp-test-ros'

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from std_msgs.msg import Bool, String, UInt8MultiArray, Float64
from std_srvs.srv import Trigger


class DetectorTcpTest(unittest.TestCase):
    def test_launch_modes_keep_original_interfaces_and_baudrate(self):
        import importlib.util
        import yaml
        from launch import LaunchDescription, LaunchService
        from launch.actions import OpaqueFunction, SetLaunchConfiguration
        from launch_ros.utilities import evaluate_parameters, normalize_parameters
        from rclpy.utilities import remove_ros_args
        device = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((device/'config/detactor.yaml').read_text())
        self.assertEqual(config['serial_node']['ros__parameters']['baudrate'], 115200)
        self.assertEqual(remove_ros_args(args=['check', '--ros-args', '--params-file', str(device/'config/detactor.yaml')]), ['check'])
        for transport in ('tcp', 'serial', None):
            seen = []
            spec = importlib.util.spec_from_file_location('detector_launch_test', device/'launch/detactor.launch.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            def fake_node(**kwargs):
                def inspect(context):
                    params = evaluate_parameters(context, normalize_parameters(kwargs['parameters']))
                    seen.append((kwargs['executable'], [value for value in params if isinstance(value, dict)]))
                    return []
                return OpaqueFunction(function=inspect, condition=kwargs.get('condition'))
            module.Node = fake_node
            service = LaunchService()
            service.include_launch_description(LaunchDescription([
                *([SetLaunchConfiguration('transport', transport)] if transport else []), module.generate_launch_description()]))
            self.assertEqual(service.run(), 0)
            values = dict(seen)
            self.assertEqual(set(values), {'detactor_node', 'tcp_node' if transport == 'tcp' else 'serial_node'})
            self.assertTrue(values['detactor_node'][0]['require_transport_connection'])
            if transport == 'tcp':
                self.assertEqual(values['tcp_node'][0]['host'], '169.254.199.90')
                self.assertEqual(values['tcp_node'][0]['port'], 9001)

    def test_direct_probe_keeps_unsolicited_packets_and_ascii_response_distinct(self):
        import importlib.util
        import threading
        path = Path(__file__).resolve().parents[1]/'tools/detactor_tcp_probe.py'
        spec = importlib.util.spec_from_file_location('detector_probe_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        received = []
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0))
            server.listen(1)
            port = server.getsockname()[1]
            def respond():
                peer, _ = server.accept()
                with peer:
                    peer.settimeout(2.)
                    peer.sendall(bytes.fromhex('40 23 24 06 00 00 00 fa 00 00 01 90 91 41 24 25'))
                    command = b''
                    while len(command) < 6:
                        command += peer.recv(6-len(command))
                    received.append(command)
                    peer.sendall(b'.e50.36;49.95;101.30<:>')
                    import time
                    time.sleep(.3)
            worker = threading.Thread(target=respond, daemon=True)
            worker.start()
            report = module.probe('127.0.0.1', port, .1, .1, output=lambda _: None)
            worker.join(timeout=2.)
        self.assertEqual(received, [b'.d0<:>'])
        self.assertEqual(report['received_before_command'], 16)
        self.assertFalse(report['ascii_frame_before_command'])
        self.assertTrue(report['ascii_frame_after_command'])
        self.assertEqual(report['events'][-1]['text'], '.e50.36;49.95;101.30<:>')

    def test_serial_task_waits_for_fresh_reply_and_times_out_without_reusing_cache(self):
        """伪终端模拟原始115200串口，既验证实际串口节点，也验证延迟响应任务。"""
        import pty
        import select
        master, slave = pty.openpty()
        device = os.ttyname(slave)
        prefix = Path(get_package_prefix('eli_cs_robot_serial'))/'lib/eli_cs_robot_serial'
        config = Path(get_package_share_directory('eli_cs_robot_serial'))/'device/detactor/config/detactor.yaml'
        processes, logs = [], []
        with tempfile.TemporaryDirectory() as directory:
            rclpy.init(args=[])
            node = Node('detector_serial_task_test')
            readings, results, values, connected = [], [], [], []
            retained = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(String, '/detactor/measurement', lambda m: readings.append(json.loads(m.data)), retained)
            node.create_subscription(String, '/detactor/task_result', lambda m: results.append(json.loads(m.data)), retained)
            node.create_subscription(Float64, '/detactor/latest_value', lambda m: values.append(m.data), retained)
            node.create_subscription(Bool, '/detactor/connected', lambda m: connected.append(m.data), retained)
            acquire = node.create_client(Trigger, '/detactor/acquire_latest')
            def wait_for(check, timeout=8.):
                end = monotonic()+timeout
                while not check() and monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.02)
                self.assertTrue(check(), '伪串口采集等待超时')
            def incoming():
                return bool(select.select([master], [], [], 0)[0])
            def frame():
                return b'.e50.36;49.95;49.89;40.06;45.69;50.30;47.22;50.33;49.27;101.30<:>'
            try:
                # task_once等待真正串口打开后触发，启动期间不丢第一次查询。
                for name, params in (
                    ('serial_node', ['device:='+device]),
                    ('detactor_node', ['auto_request:=false', 'task_once:=true',
                                       'acquisition_timeout:=1.0', 'data_directory:='+directory])):
                    log = open(Path(directory)/(name+'.log'), 'w+')
                    logs.append(log)
                    command = [str(prefix/name), '--ros-args', '--params-file', str(config)]
                    for value in params:
                        command += ['-p', value]
                    processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
                wait_for(lambda: connected and connected[-1] and incoming())
                self.assertEqual(os.read(master, 64), b'.d0<:>')
                os.write(master, frame()[:18])
                for _ in range(3):
                    rclpy.spin_once(node, timeout_sec=.02)
                self.assertFalse(readings)
                os.write(master, frame()[18:])
                wait_for(lambda: results and results[-1]['success'] and values)
                self.assertAlmostEqual(values[-1], 101.30)
                self.assertEqual(results[-1]['measurement']['sequence'], 1)
                wait_for(acquire.service_is_ready)
                future = acquire.call_async(Trigger.Request())
                wait_for(incoming)
                self.assertEqual(os.read(master, 64), b'.d0<:>')
                self.assertFalse(future.done(), '不能立即把旧缓存当本次采集成功')
                other = acquire.call_async(Trigger.Request())
                wait_for(other.done)
                self.assertFalse(other.result().success)
                os.write(master, b'\x00'+frame()+b'\x00')  # 帧外填充保留字节；同值仍是新回包，串口不能崩溃。
                wait_for(future.done)
                self.assertTrue(future.result().success)
                fresh = json.loads(future.result().message)
                self.assertEqual(fresh['sequence'], 2)
                self.assertEqual(fresh['value'], 101.3)
                timed_out = acquire.call_async(Trigger.Request())
                wait_for(incoming)
                self.assertEqual(os.read(master, 64), b'.d0<:>')
                wait_for(timed_out.done)
                self.assertFalse(timed_out.result().success)
                wait_for(lambda: results and not results[-1]['success'])
                self.assertIsNone(results[-1]['measurement'])
                self.assertEqual(len(readings), 2)
                self.assertFalse(incoming(), 'task_once不能偷偷变成周期查询')
            finally:
                node.destroy_node()
                rclpy.shutdown()
                for process in processes:
                    process.send_signal(signal.SIGINT)
                for process in processes:
                    try:
                        process.wait(timeout=3.)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2.)
                for log in logs:
                    log.seek(0)
                    if any(process.returncode not in (0, -2) for process in processes):
                        print(log.read())
                    log.close()
                os.close(master)
                os.close(slave)

    def test_real_nodes_fragmented_coalesced_frames_and_reconnection(self):
        """验证原协议字节、最后一次读数及断线半帧隔离，测试进程仅连127.0.0.1。"""
        with tempfile.TemporaryDirectory() as directory, socket.socket() as server:
            server.bind(('127.0.0.1', 0))
            server.listen(2)
            server.settimeout(5.)
            port = server.getsockname()[1]
            prefix = Path(get_package_prefix('eli_cs_robot_serial'))/'lib/eli_cs_robot_serial'
            config = Path(get_package_share_directory('eli_cs_robot_serial'))/'device/detactor/config/detactor.yaml'
            processes, connections, logs = [], [], []
            rclpy.init(args=[])
            node = Node('detector_tcp_test')
            states, readings, raw, values, hex_chunks = [], [], [], [], []
            node.create_subscription(String, '/detactor/rx_hex', lambda m: hex_chunks.append(bytes.fromhex(m.data)),
                QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            node.create_subscription(Float64, '/detactor/latest_value', lambda m: values.append(m.data),
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            node.create_subscription(UInt8MultiArray, '/detactor/rx_bytes', lambda m: raw.append(bytes(m.data)), 100)
            node.create_subscription(Bool, '/detactor/connected', lambda m: states.append(m.data),
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            node.create_subscription(String, '/detactor/measurement', lambda m: readings.append(json.loads(m.data)),
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            acquire = node.create_client(Trigger, '/detactor/acquire')
            latest = node.create_client(Trigger, '/detactor/latest_measurement')

            def wait_for(check, timeout=8.):
                deadline = monotonic()+timeout
                ready = check()
                while not ready and monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.02)
                    ready = check()
                self.assertTrue(ready, '等待本机模拟测厚仪事件超时')

            def trigger():
                future = acquire.call_async(Trigger.Request())
                wait_for(future.done)
                return future.result()

            def frame(value):
                return ('.e'+';'.join(str(v) for v in range(value-9, value+1))+'<:>').encode()

            try:
                for name, params in (
                        ('tcp_node', ['mode:=client', 'host:=127.0.0.1', 'port:='+str(port),
                                      'reconnect_period:=1.0', 'connect_timeout:=0.2']),
                        ('detactor_node', ['auto_request:=false', 'data_directory:='+directory])):
                    log = open(Path(directory)/(name+'.log'), 'w+')
                    logs.append(log)
                    command = [str(prefix/name), '--ros-args', '--params-file', str(config)]
                    for value in params:
                        command += ['-p', value]
                    processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                      start_new_session=True))
                peer, _ = server.accept()
                connections.append(peer)
                peer.settimeout(5.)
                wait_for(lambda: states and states[-1] and acquire.service_is_ready())
                # 首次DDS连接状态到达后再触发；失败只允许重复测试请求，不重放设备命令。
                wait_for(lambda: trigger().success)
                self.assertEqual(peer.recv(64), b'.d0<:>')
                payload = b'\x00\x00'+frame(10)+b'\x00\x00'
                peer.sendall(payload[:1])
                rclpy.spin_once(node, timeout_sec=.05)
                peer.sendall(payload[1:-2])
                rclpy.spin_once(node, timeout_sec=.05)
                peer.sendall(payload[-2:])
                wait_for(lambda: readings and readings[-1]['value'] == 10.)
                peer.sendall(frame(20)+frame(30))
                wait_for(lambda: readings[-1]['value'] == 30. and values and values[-1] == 30.)
                self.assertEqual(readings[-1]['sequence'], 3)
                wait_for(lambda: (Path(directory)/'latest.txt').exists() and
                         (Path(directory)/'latest.txt').read_text().strip().split(';')[0] == '30')
                displayed = [float(v) for v in (Path(directory)/'latest.txt').read_text().strip().split(';')]
                self.assertEqual(displayed, list(range(30, 20, -1)), '多个新值同时到达后仍须最新在前')
                self.assertEqual(readings[-1]['unit'], 'unknown')
                future = latest.call_async(Trigger.Request())
                wait_for(future.done)
                self.assertEqual(json.loads(future.result().message)['value'], 30.)
                # NUL原始字节仍完整保留，通讯进程不能因String序列化崩溃。
                self.assertTrue(b''.join(raw).startswith(payload), '原始NUL填充字节必须完整保留')
                malformed = frame(35).replace(b'35<:>', b'3\x005<:>')
                previous = len(raw)
                peer.sendall(malformed)
                wait_for(lambda: len(raw) > previous)
                rclpy.spin_once(node, timeout_sec=.1)
                self.assertEqual(readings[-1]['value'], 30.)
                self.assertEqual(values[-1], 30., '无效帧不得覆盖最新单值')
                self.assertIsNone(processes[0].poll())
                self.assertIsNone(processes[1].poll())
                count = len(values)
                unknown = bytes.fromhex('40 23 24 06 00 00 00 fa 00 00 01 90 91 41 24 25')
                peer.sendall(unknown[:5])
                rclpy.spin_once(node, timeout_sec=.05)
                peer.sendall(unknown[5:])
                wait_for(lambda: unknown in b''.join(raw) and unknown in b''.join(hex_chunks))
                rclpy.spin_once(node, timeout_sec=.1)
                self.assertEqual(len(values), count, '未识别协议不能发布为厚度')
                # 断线前留下半帧，新连接必须从新帧开始。
                peer.sendall(b'.e999;')
                rclpy.spin_once(node, timeout_sec=.1)
                states.clear()
                peer.shutdown(socket.SHUT_RDWR)
                peer.close()
                wait_for(lambda: False in states)
                wait_for(lambda: not trigger().success)
                peer, _ = server.accept()
                connections.append(peer)
                peer.settimeout(5.)
                wait_for(lambda: states[-1] is True)
                self.assertTrue(trigger().success)
                self.assertEqual(peer.recv(64), b'.d0<:>')
                peer.sendall(frame(40))
                wait_for(lambda: readings[-1]['value'] == 40.)
                self.assertEqual(readings[-1]['sequence'], 5)
                wait_for(lambda: values[-1] == 40.)
                # 数值相同的新有效回包也发布；证明没有只在数值变化时才更新。
                received = len(values)
                peer.sendall(frame(40))
                wait_for(lambda: len(values) > received and readings[-1]['sequence'] == 6)
                self.assertEqual(values[-1], 40.)
                # 新订阅者可立即取得缓存的最新有效值，不需要等下一次测量。
                retained = []
                subscription = node.create_subscription(Float64, '/detactor/latest_value',
                    lambda m: retained.append(m.data),
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                wait_for(lambda: retained == [40.])
                node.destroy_subscription(subscription)
                # 按用户串口助手截图回放带小数点的原文本协议。
                screenshot_reply = (
                    b'.e50.36;49.95;49.89;40.06;45.69;50.30;47.22;50.33;49.27;50.36;'
                    b'101.56;106.98;50.34;101.56;101.56;101.11;50.30;50.65;50.16;50.54;'
                    b'101.19;101.07;102.61;101.55;101.32;101.07;101.09;100.69;101.30;101.32;101.30<:>')
                peer.sendall(screenshot_reply)
                wait_for(lambda: readings[-1]['sequence'] == 7 and values[-1] == 101.30)
                self.assertEqual(readings[-1]['value'], 101.30)
                wait_for(lambda: (Path(directory)/'latest.txt').read_text().split(';')[0] == '101.30')
            finally:
                for process in processes:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                for process in processes:
                    try:
                        code = process.wait(timeout=4.)
                        self.assertEqual(code, 0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                        self.fail('TCP节点退出超时')
                for connection in connections:
                    connection.close()
                node.destroy_node()
                rclpy.shutdown()
                for log in logs:
                    log.seek(0)
                    print(log.read())
                    log.close()


if __name__ == '__main__':
    unittest.main()
