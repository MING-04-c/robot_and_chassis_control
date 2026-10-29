#!/usr/bin/env python3
"""接收工控机三维检测结果；仅落盘和发布观测，不启动机械臂任务。"""

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

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String

from industrial_pc_protocol import (
    ProtocolError, authenticate, canonical, envelope, load_key, read_message,
    validate_result, validate_thickness, validate_thickness_request, decode, write_message,
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
        self.key = load_key(param("key_file", "~/.config/robot_control/industrial_pc.key"))
        if (not 1 <= port <= 65535 or not math.isfinite(timeout) or timeout <= 0 or
                not self.frame or self.frame != self.frame.strip()):
            raise ValueError("端口、超时或光学坐标系参数无效")
        self.output.mkdir(parents=True, exist_ok=True)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.result_pub = self.create_publisher(String, "~/result", retained)
        self.status_pub = self.create_publisher(String, "~/status", retained)
        self.points_pub = self.create_publisher(PoseArray, "~/camera_points", 10)
        self.demo_pub = self.create_publisher(PoseArray, "~/demo_camera_points", 10)
        self.latest_thickness = None
        self.create_subscription(String, "/detactor/measurement", self.on_thickness, retained)
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
        self.status("LISTENING", endpoint=f"{host}:{port}")
        self.get_logger().info(
            f"工控机结果接口监听 {host}:{port}；保存到 {self.output}；"
            "演示数据与正式观测分话题发布，不发送机器人运动命令"
        )

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
