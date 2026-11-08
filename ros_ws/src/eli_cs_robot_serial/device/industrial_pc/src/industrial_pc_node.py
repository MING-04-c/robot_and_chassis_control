#!/usr/bin/env python3
"""工控机TCP桥接：检测点只发布观测；独立签名命令启动/停止/继续已加载任务。"""

from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import socket
import threading
import tempfile
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Pose, PoseArray
from masc_chassis_msgs.msg import BmsStatus, ChassisStatus, CommStatus
from std_msgs.msg import Bool, Empty, String
from rcl_interfaces.msg import Log
from runtime_monitor import RuntimeMonitor
from std_srvs.srv import Trigger

from industrial_pc_protocol import (
    ProtocolError, authenticate, canonical, envelope, load_key, read_message,
    validate_result, validate_thickness, validate_detection_result, validate_thickness_request, validate_command, decode, write_message,
)


class IndustrialPC(Node):
    def __init__(self):
        super().__init__("industrial_pc")

        def param(name, value):
            return self.declare_parameter(name, value).value

        host = param("bind_host", "0.0.0.0")
        port = int(param("port", 9010))
        timeout = float(param("socket_timeout", 5.0))
        self.frame = param("camera_optical_frame", "camera_color_optical_frame")
        self.publish_demo = param("publish_demo_points", True)
        self.output = Path(param("output_dir", "~/robot_data/industrial_pc/results")).expanduser()
        key_file = param("key_file", "")
        self.key = load_key(key_file) if param("use_shared_key", False) else None
        if (not 1 <= port <= 65535 or not math.isfinite(timeout) or timeout <= 0 or
                not self.frame or self.frame != self.frame.strip()):
            raise ValueError("端口、超时或光学坐标系参数无效")
        self.output.mkdir(parents=True, exist_ok=True)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.result_pub = self.create_publisher(String, "~/result", retained)
        self.status_pub = self.create_publisher(String, "~/status", retained)
        self.points_pub = self.create_publisher(PoseArray, "~/camera_points", 10)
        self.demo_pub = self.create_publisher(PoseArray, "~/demo_camera_points", 10)
        self.point_result_dir = self.output / "detection_results"
        self.point_result_dir.mkdir(exist_ok=True)
        self.detection_results = {}
        # 落盘后再提供给监听端；桥接重启后仍能补收最近100条结果。
        for path in sorted(self.point_result_dir.glob('*.json'), key=lambda x: x.stat().st_mtime)[-100:]:
            try:
                result = validate_detection_result(json.loads(path.read_text()))
                self.detection_results[result['result_id']] = result
            except (OSError, ValueError):
                self.get_logger().warning('忽略无效逐点结果文件：'+str(path))
        self.latest_thickness = None
        self.observations = {}
        self.enable_status_monitor = param('enable_status_monitor', True)
        self.monitor = RuntimeMonitor() if self.enable_status_monitor else None
        self.monitor_normal = self.monitor_abnormal = None
        if self.enable_status_monitor:
            self.normal_pub = self.create_publisher(String, '/robot_status/normal', retained)
            self.abnormal_pub = self.create_publisher(String, '/robot_status/abnormal', retained)
            self.create_subscription(String, '/arm_joint_postures/status',
                                     lambda msg: self.cache_status('arm', msg), retained)
            self.create_subscription(String, '/protective_stop_guard/status',
                                     lambda msg: self.cache_status('safety', msg), retained)
            self.create_subscription(Bool, '/robot_control/safety_inhibit',
                                     lambda msg: self.monitor.receive('inhibit', {'inhibited': msg.data}), retained)
            self.create_subscription(Log, '/rosout', self.on_ros_log, 50)
        self.power_feedback = {}
        self.charging_layout = str(param('charging_status_layout', 'bitfield32'))
        if self.charging_layout not in ('bitfield32', 'byte_flags'):
            raise ValueError('charging_status_layout只能是bitfield32或byte_flags')
        self.create_subscription(BmsStatus, '/masc_chassis/bms_status',
                                 lambda msg: self.cache_power('bms', msg), 10)
        self.create_subscription(ChassisStatus, '/masc_chassis/chassis_status',
                                 lambda msg: self.cache_power('system', msg), 10)
        self.create_subscription(CommStatus, '/masc_chassis/comm_status',
                                 lambda msg: self.cache_power('comm', msg), 10)
        self.last_result = None
        self.pending_controls = {}
        self.command_dir = self.output / "commands"
        self.command_dir.mkdir(exist_ok=True)
        self.start_task = self.create_client(Trigger, "/task_controller/start")
        self.stop_task = self.create_client(Trigger, "/task_controller/stop")
        self.capture_camera = self.create_client(Trigger, "/task_controller/capture")
        self.resume_pub = self.create_publisher(Empty, "/line_follow/resume", 10)
        self.create_subscription(String, "/task_controller/status",
                                 lambda msg: self.cache_status("task", msg), retained)
        self.create_subscription(String, "/line_follow/status",
                                 lambda msg: self.cache_status("line_follow", msg), 10)
        self.create_subscription(String, "/detactor/measurement", self.on_thickness, retained)
        self.create_subscription(String, "/task_controller/detection_result", self.on_detection_result,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.requests = queue.Queue(maxsize=16)
        self.stop_event = threading.Event()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server.bind((host, port))
            self.server.listen(4)
            self.server.settimeout(0.5)
        except Exception:
            self.server.close()
            raise
        self.socket_timeout = timeout
        self.thread = threading.Thread(target=self.serve, name="industrial-pc-tcp", daemon=True)
        self.thread.start()
        self.create_timer(0.02, self.process_requests)
        self.create_timer(.1, self.check_control_timeouts)
        if self.enable_status_monitor:
            self.create_timer(.5, self.update_monitor)
        self.status("LISTENING", endpoint=f"{host}:{port}")
        self.get_logger().info(
            f"工控机结果接口监听 {host}:{port}；保存到 {self.output}；"
            "检测点只发布观测；独立任务请求可调用已加载任务的 start/stop/resume/capture"
        )

    def on_detection_result(self, message):
        """回原位后形成的逐点结果持久化，通过9010状态查询交给麒麟常驻监听。"""
        try:
            result = validate_detection_result(json.loads(message.data))
            identity = result['result_id']
            path = self.point_result_dir/(hashlib.sha256(identity.encode()).hexdigest()+'.json')
            if path.exists():
                if json.loads(path.read_text()) != result:
                    raise ProtocolError('逐点结果ID重复但内容不同')
            else:
                self.save_json(path, result)
            self.detection_results[identity] = result
            while len(self.detection_results) > 100:
                self.detection_results.pop(next(iter(self.detection_results)))
        except (OSError, ValueError) as error:
            self.get_logger().error('逐点测厚结果接收失败：'+str(error))

    def cache_status(self, name, message):
        try:
            value = decode(message.data.encode("utf-8"))
            self.observations[name] = (value, time.monotonic())
            if self.monitor is not None:
                self.monitor.receive(name, value)
        except ProtocolError:
            pass

    def robot_status(self):
        """缓存不存在或过期时明确显示，不用旧状态冒充当前就绪。"""
        output = {}
        for name in ("task", "line_follow"):
            value, received = self.observations.get(name, (None, None))
            age = time.monotonic()-received if received is not None else None
            output[name] = {"available": value is not None, "fresh": age is not None and age < 2,
                            "age_seconds": age, "data": value}
        return {**output, "latest_result": self.last_result,
                "latest_thickness": self.thickness_snapshot(),
                "chassis_power": self.chassis_power_snapshot(),
                "status_monitor_enabled": self.enable_status_monitor,
                "normal_status": self.monitor_normal, "abnormal_status": self.monitor_abnormal,
                "detection_results": list(self.detection_results.values()),
                "start_service_ready": self.start_task.service_is_ready(),
                "stop_service_ready": self.stop_task.service_is_ready(),
                "capture_service_ready": self.capture_camera.service_is_ready()}

    def on_ros_log(self, message):
        if self.monitor is None:
            return
        if message.name == 'task_controller' and message.msg.endswith('stopped by user'):
            return  # 明确的人工停止属于正常运行状态，不当成设备故障。
        if message.level >= Log.ERROR:
            self.monitor.log_error(message.name, message.msg, message.level,
                                   message.file, message.function, message.line)

    def update_monitor(self):
        if self.monitor is None:
            return
        self.monitor_normal, self.monitor_abnormal = self.monitor.snapshot(self.chassis_power_snapshot())
        self.normal_pub.publish(String(data=canonical(self.monitor_normal).decode()))
        self.abnormal_pub.publish(String(data=canonical(self.monitor_abnormal).decode()))

    def cache_power(self, name, message):
        """缓存已有底盘反馈，不开CAN口、不发送运动命令。"""
        self.power_feedback[name] = (message, time.monotonic())
        if self.monitor is None:
            return
        if name == 'system':
            self.monitor.receive('chassis', {'fault': int(message.chassis_fault)})
        elif name == 'comm':
            self.monitor.receive('comm', {'state': int(message.comm_state), 'fail_reason': int(message.fail_reason)})

    def power_sample(self, name, timeout):
        message, received = self.power_feedback.get(name, (None, None))
        if message is None:
            return None, False, None
        age = time.monotonic()-received
        stamp = message.header.stamp
        stamp_ns = stamp.sec*10**9+stamp.nanosec
        source_age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
        fresh = age < timeout and stamp_ns > 0 and -.1 <= source_age < timeout
        return message, fresh, max(age, source_age)

    def chassis_power_snapshot(self):
        """电量来自0x206；充电按配置解析0x204，失联/超时返回null。"""
        bms, bms_fresh, bms_age = self.power_sample('bms', 3.)
        system, system_fresh, system_age = self.power_sample('system', 1.)
        comm, comm_fresh, _ = self.power_sample('comm', 3.)
        connected = bool(comm_fresh and comm.comm_state == CommStatus.COMM_STATE_CONNECTED)
        bms_valid = bool(connected and bms_fresh and 0 <= bms.bat_soc <= 100
                         and math.isfinite(bms.bat_voltage) and math.isfinite(bms.bat_current)
                         and bms.bms_health not in (BmsStatus.BMS_HEALTH_TIMEOUT,
                                                    BmsStatus.BMS_HEALTH_UNKNOWN))
        system_valid = bool(connected and system_fresh)
        raw = int(system.system_status_flags) if system is not None else None
        charging, low_battery = None, None
        if system_valid:
            if self.charging_layout == 'bitfield32':
                charging, low_battery = bool(raw & (1 << 3)), bool(raw & (1 << 2))
            else:
                charging = bool(system.manual_charging or system.dock_charging)
                low_battery = bool(system.low_battery)
        return {
            'available': bms_valid and system_valid,
            'connected': connected,
            'battery_percent': int(bms.bat_soc) if bms_valid else None,
            'charging': charging,
            'charging_state': 'unknown' if charging is None else ('charging' if charging else 'not_charging'),
            'low_battery': low_battery,
            'voltage_v': float(bms.bat_voltage) if bms_valid else None,
            'current_a': float(bms.bat_current) if bms_valid else None,
            'bms_fresh': bms_valid, 'system_fresh': system_valid,
            'bms_age_seconds': bms_age, 'system_age_seconds': system_age,
            'charging_status_layout': self.charging_layout,
            'system_status_flags': raw,
        }

    @staticmethod
    def save_json(path, value):
        """同目录原子落盘；命令回执必须先持久化，再回到网络线程。"""
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=path.parent, mode="wb", delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(canonical(value))
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def finish_command(self, identity, ack):
        entry = self.pending_controls.pop(identity, None)
        if entry is None:
            return
        entry["record"]["ack"] = ack
        try:
            self.save_json(entry["path"], entry["record"])
        except OSError as error:
            ack = self.ack(False, message_id=identity, execution_unknown=True,
                           reason="命令已提交，但回执保存失败："+str(error))
        entry["job"]["ack"] = ack
        entry["job"]["done"].set()

    def check_control_timeouts(self):
        for identity, entry in list(self.pending_controls.items()):
            if time.monotonic() >= entry["deadline"]:
                self.finish_command(identity, self.ack(
                    False, message_id=identity, execution_unknown=True,
                    reason="任务服务3秒内未回复；不要换新ID盲目重试，请先查询任务状态"))

    def handle_command(self, job, packet, payload):
        """固定接口桥接；同一ID永久去重，网络重试不会再次启动任务。"""
        action = validate_command(payload)
        identity = payload["message_id"]
        if action in ('status_normal', 'status_abnormal'):
            if not self.enable_status_monitor:
                job['ack'] = self.ack(False, message_id=identity,
                                      status_monitor_enabled=False, reason='状态汇总监视未启用；可用status查询基础任务状态')
                return False
            if self.monitor_normal is None:
                self.update_monitor()
            field = 'normal_status' if action == 'status_normal' else 'abnormal_status'
            job['ack'] = self.ack(True, message_id=identity, **{field:
                self.monitor_normal if field == 'normal_status' else self.monitor_abnormal})
            return False
        if action == "status":
            job["ack"] = self.ack(True, message_id=identity, robot_status=self.robot_status())
            return False
        path = self.command_dir / (hashlib.sha256(identity.encode()).hexdigest()+".json")
        if path.exists():
            record = decode(path.read_bytes())
            if record.get("payload_sha256") != packet["payload_sha256"]:
                raise ProtocolError("命令 message_id 已使用但内容不同")
            job["ack"] = dict(record.get("ack") or self.ack(
                False, message_id=identity, execution_unknown=True,
                reason="该ID的命令仍在处理或曾被中断；查询状态，不重复执行"))
            job["ack"]["disposition"] = "duplicate"
            return False
        # 在调用 ROS 之前写入 pending。进程若中途退出，同ID也不会再次执行。
        record = {"payload_sha256": packet["payload_sha256"], "action": action, "ack": None}
        self.save_json(path, record)
        self.pending_controls[identity] = {"job": job, "record": record, "path": path,
                                           "deadline": time.monotonic()+3}
        job["deferred"] = True
        if action == "resume":
            line = self.robot_status()["line_follow"]
            data = line["data"] or {}
            if not line["fresh"] or data.get("state") != "stop_marker_intermediate":
                self.finish_command(identity, self.ack(False, message_id=identity,
                                    reason="当前不在有效途中停车等待状态，不能继续"))
            else:
                self.resume_pub.publish(Empty())
                self.finish_command(identity, self.ack(True, message_id=identity,
                                    action=action, disposition="published", robot_motion_authorized=True,
                                    message="继续消息已发布，执行进展请查看状态"))
            return True
        client = {"start": self.start_task, "stop": self.stop_task,
                  "capture": self.capture_camera}[action]
        if not client.service_is_ready():
            self.finish_command(identity, self.ack(False, message_id=identity,
                                reason="任务服务未启动；请先在树莓派启动对应任务launch"))
            return True
        def finished(future):
            try:
                result = future.result()
                ack = self.ack(bool(result.success), message_id=identity, action=action,
                               message=result.message, disposition="executed",
                               robot_motion_authorized=bool(result.success) and action != "capture")
            except Exception as error:
                ack = self.ack(False, message_id=identity, execution_unknown=True, reason=str(error))
            self.finish_command(identity, ack)
        try:
            client.call_async(Trigger.Request()).add_done_callback(finished)
        except Exception as error:
            self.finish_command(identity, self.ack(False, message_id=identity, reason=str(error)))
        return True

    def status(self, state, **fields):
        self.status_pub.publish(String(data=canonical({"state": state, **fields}).decode()))

    def on_thickness(self, message):
        try:
            self.latest_thickness = validate_thickness(decode(message.data.encode("utf-8")))
        except ProtocolError as error:
            self.get_logger().warning("忽略无效厚度读数：" + str(error))

    def thickness_snapshot(self):
        """返回当前进程收到的最近单值，无数据明确返回 unavailable。"""
        if self.latest_thickness is None:
            return {"available": False, "measurement": None}
        stamp = self.latest_thickness["received_at_ros"]
        age = (self.get_clock().now().nanoseconds -
               (stamp["sec"] * 10**9 + stamp["nanosec"])) / 1e9
        return {"available": True, "measurement": self.latest_thickness,
                "age_seconds": age, "unit_confirmed": self.latest_thickness["unit"] != "unknown"}

    def serve(self):
        """网络线程只收发和排队；ROS 发布与结果处理在执行器线程完成。"""
        while not self.stop_event.is_set():
            try:
                connection, peer = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with connection:
                connection.settimeout(self.socket_timeout)
                while not self.stop_event.is_set():
                    try:
                        message = read_message(connection)
                        done = threading.Event()
                        job = {"message": message, "peer": peer[0], "done": done}
                        self.requests.put_nowait(job)
                        if not done.wait(self.socket_timeout):
                            break
                        write_message(connection, envelope(job["ack"], self.key))
                    except (EOFError, socket.timeout, OSError):
                        break
                    except (ProtocolError, queue.Full) as error:
                        try:
                            write_message(connection, envelope(self.ack(False, reason=str(error)), self.key))
                        except OSError:
                            pass
                        break

    @staticmethod
    def ack(ok, **fields):
        return {"schema": "paint_v54_pi_result_ack_v1", "ok": ok,
                "robot_motion_authorized": False, **fields}

    def process_requests(self):
        """处理一帧，校验后持久化并发布，再向主机返回接收回执。"""
        try:
            job = self.requests.get_nowait()
        except queue.Empty:
            return
        identity = ""
        try:
            packet = job["message"]
            payload = authenticate(packet, self.key)
            if payload.get("schema") in ("paint_robot_task_request_v1", "paint_robot_status_request_v1"):
                identity = payload.get("message_id", "")
                self.handle_command(job, packet, payload)
                return
            if payload.get("schema") == "paint_thickness_latest_request_v1":
                validate_thickness_request(payload)
                identity = payload["message_id"]
                job["ack"] = self.ack(True, message_id=identity,
                                      latest_thickness=self.thickness_snapshot())
                return
            kind, points, stamp = validate_result(payload)
            identity = payload["message_id"]
            # 文件名使用 ID 的哈希，重复 ID 内容必须完全一致，重启后仍能去重。
            path = self.output / (hashlib.sha256(identity.encode()).hexdigest() + ".json")
            if path.exists():
                previous = json.loads(path.read_text())
                if previous["message"]["payload_sha256"] != packet["payload_sha256"]:
                    raise ProtocolError("message_id 已存在但内容不同")
                job["ack"] = self.ack(True, message_id=identity, disposition="duplicate",
                                      kind=kind, point_count=len(points), published=False)
                self.status("DUPLICATE", message_id=identity)
                return
            record = {
                "received_at_pi": datetime.now(timezone.utc).isoformat(),
                "peer": job["peer"], "kind": kind, "message": packet,
            }
            # 只在完整写入后出现正式文件；不覆写历史结果。
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(
                    mode="w", encoding="utf-8", dir=self.output, prefix=".result-", delete=False
                ) as stream:
                    temporary = Path(stream.name)
                    stream.write(canonical(record).decode() + "\n")
                    stream.flush()
                    os.fsync(stream.fileno())
                temporary.replace(path)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
            self.result_pub.publish(String(data=canonical(payload).decode()))
            self.last_result = {"message_id": identity, "kind": kind, "point_count": len(points)}
            published = False
            if points and (kind != "demo" or self.publish_demo):
                output = PoseArray()
                output.header.frame_id = self.frame
                output.header.stamp.sec, output.header.stamp.nanosec = stamp["sec"], stamp["nanosec"]
                for xyz in points:
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = [float(x) / 1000 for x in xyz]
                    pose.orientation.w = 1.0  # 占位姿态；协议只定义点位置，不代表接触朝向。
                    output.poses.append(pose)
                (self.demo_pub if kind == "demo" else self.points_pub).publish(output)
                published = True
            job["ack"] = self.ack(True, message_id=identity, disposition="stored",
                                  kind=kind, point_count=len(points), published=published)
            self.status("RECEIVED", message_id=identity, kind=kind,
                        point_count=len(points), observation_stamp_available=stamp["sec"] > 0)
            self.get_logger().info(f"已接收 {identity}：{kind}，{len(points)} 点")
        except Exception as error:
            job["ack"] = self.ack(False, message_id=identity, reason=str(error))
            self.status("REJECTED", message_id=identity, reason=str(error))
            self.get_logger().warning("工控机结果被拒绝：" + str(error))
        finally:
            # 结果接收 ACK 也附带此刻最新单值，主机不需要解析历史十值窗口。
            if not job.get("deferred"):
                job["ack"].setdefault("latest_thickness", self.thickness_snapshot())
                job["done"].set()

    def close(self):
        self.stop_event.set()
        self.server.close()
        self.thread.join(timeout=self.socket_timeout + 1.0)


def main():
    # ros2 launch 与进程组可能先后发出 SIGINT；只设置退出标志，避免清理时二次中断。
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    previous = {number: signal.signal(number, request_stop)
                for number in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = IndustrialPC()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for number, handler in previous.items():
            signal.signal(number, handler)


if __name__ == "__main__":
    main()
