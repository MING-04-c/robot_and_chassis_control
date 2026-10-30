#!/usr/bin/env python3
"""工控机检测结果协议：纯 Python 编解码，可直接复制到主机使用。"""

import hashlib
import hmac
import json
import math
import os
from pathlib import Path
import re
import struct

PROTOCOL = "paint_v54_camera_coordinates_tcp_v1"
MAX_MESSAGE_BYTES = 262144
FRAME = "CAMERA_PINHOLE_PRELIMINARY"


class ProtocolError(ValueError):
    """格式、完整性或签名校验失败。"""


def validate_command(payload):
    """校验任务/状态请求；只允许固定动作，不能传入任意 ROS 服务或系统命令。"""
    identity = payload.get("message_id")
    if not isinstance(identity, str) or not re.fullmatch(r"[A-Za-z0-9_.-]{1,128}", identity):
        raise ProtocolError("message_id 缺失或无效")
    schema = payload.get("schema")
    if schema == "paint_robot_status_request_v1":
        if set(payload) != {"schema", "message_id"}:
            raise ProtocolError("状态查询包含未知字段")
        return "status"
    if (schema != "paint_robot_task_request_v1" or
            set(payload) != {"schema", "message_id", "action"} or
            payload.get("action") not in ("start", "stop", "resume", "capture")):
        raise ProtocolError("任务请求仅支持 start/stop/resume/capture")
    return payload["action"]


def validate_thickness(reading):
    """校验最新单值回包；时间是串口接收时间，不冒充仪器测量时间。"""
    if not isinstance(reading, dict) or reading.get("schema") != "paint_thickness_measurement_v1":
        raise ProtocolError("厚度读数 schema 无效")
    value = reading.get("value")
    if (isinstance(value, bool) or not isinstance(value, (float, int)) or
            not math.isfinite(value) or reading.get("unit") not in ("unknown", "um", "mm")):
        raise ProtocolError("厚度值或单位无效")
    sequence = reading.get("sequence")
    stamp = reading.get("received_at_ros")
    if (type(sequence) is not int or sequence < 1 or not isinstance(stamp, dict) or
            type(stamp.get("sec")) is not int or stamp["sec"] < 0 or
            type(stamp.get("nanosec")) is not int or not 0 <= stamp["nanosec"] < 10**9):
        raise ProtocolError("厚度序号或接收时间无效")
    # 只取约定字段，禁止顺带传入历史窗口。
    return {"schema": reading["schema"], "value": value, "unit": reading["unit"],
            "sequence": sequence, "received_at_ros": dict(stamp)}


def validate_thickness_request(payload):
    """校验主机的只读查询，保持与现有签名封装兼容。"""
    if (payload.get("schema") != "paint_thickness_latest_request_v1" or
            not isinstance(payload.get("message_id"), str) or
            re.fullmatch(r"[A-Za-z0-9_.-]{1,128}", payload["message_id"]) is None):
        raise ProtocolError("最新厚度查询格式或 message_id 无效")


def canonical(value):
    """输入 JSON 对象，输出 UTF-8 规范字节；不允许 NaN/Infinity。"""
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode("utf-8")


def load_key(path):
    """读取两端共享的 UTF-8 密钥；路径为空时读取环境变量。"""
    key = (Path(path).expanduser().read_text().strip() if path else
           os.environ.get("INDUSTRIAL_PC_SHARED_KEY", "").strip())
    if len(key.encode("utf-8")) < 32:
        raise ProtocolError("请设置至少 32 字节共享密钥文件，或 INDUSTRIAL_PC_SHARED_KEY")
    return key.encode("utf-8")


def envelope(payload, key):
    """默认生成SHA256完整性封装；提供key时才追加兼容旧模式的HMAC。"""
    body = canonical(payload)
    return {"protocol": PROTOCOL, "payload": payload,
            "payload_sha256": hashlib.sha256(body).hexdigest(),
            **({"hmac_sha256": hmac.new(key, body, hashlib.sha256).hexdigest()} if key is not None else {})}


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError("JSON 对象包含重复字段：" + key)
        result[key] = value
    return result


def decode(body):
    """输入完整消息字节，返回 JSON 对象；拒绝重复字段和非有限常量。"""
    def invalid(value):
        raise ProtocolError("JSON 不允许 " + value)
    try:
        value = json.loads(body.decode("utf-8"), object_pairs_hook=_unique_object,
                           parse_constant=invalid)
    except (UnicodeError, ValueError, RecursionError) as error:
        raise ProtocolError("无效 JSON：" + str(error)) from error
    if not isinstance(value, dict):
        raise ProtocolError("消息必须是 JSON 对象")
    return value


def authenticate(message, key):
    """校验完整性并返回payload；仅显式提供key时验证HMAC。"""
    if message.get("protocol") != PROTOCOL or not isinstance(message.get("payload"), dict):
        raise ProtocolError("协议名称或 payload 无效")
    payload = message["payload"]
    body = canonical(payload)
    checks = [("payload_sha256", hashlib.sha256(body).hexdigest())]
    if key is not None:
        checks.append(("hmac_sha256", hmac.new(key, body, hashlib.sha256).hexdigest()))
    elif message.get("hmac_sha256"):
        raise ProtocolError("当前为无密钥模式，请同步更新两端配置 use_shared_key=false")
    for name, expected in checks:
        actual = message.get(name)
        if not isinstance(actual, str) or not hmac.compare_digest(actual, expected):
            raise ProtocolError(name + " 校验失败")
    return payload


def read_exact(connection, count):
    """读取固定字节数，处理 TCP 拆包；半包断连直接报错。"""
    data = bytearray()
    while len(data) < count:
        chunk = connection.recv(count - len(data))
        if not chunk:
            raise EOFError("TCP 对端断开或消息不完整")
        data.extend(chunk)
    return bytes(data)


def read_message(connection):
    """读取 uint32 大端长度及 JSON，长度不含自身四字节。"""
    size = struct.unpack("!I", read_exact(connection, 4))[0]
    if not 1 <= size <= MAX_MESSAGE_BYTES:
        raise ProtocolError("消息长度超出 1～262144 字节")
    return decode(read_exact(connection, size))


def write_message(connection, message):
    """写入一帧 JSON；与读取函数配合支持拆包和连续多帧。"""
    body = canonical(message)
    if not 1 <= len(body) <= MAX_MESSAGE_BYTES:
        raise ProtocolError("消息过大")
    connection.sendall(struct.pack("!I", len(body)) + body)


def _numbers(value, count, name):
    if (not isinstance(value, list) or len(value) != count or
            any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
        raise ProtocolError(name + " 必须是有限数字数组")
    return value


def validate_result(payload):
    """验证结果语义，返回分类、按原顺序的毫米坐标、原始 ROS 观测时间。"""
    identity = payload.get("message_id")
    if not isinstance(identity, str) or not re.fullmatch(r"[A-Za-z0-9_.-]{1,128}", identity):
        raise ProtocolError("message_id 缺失或无效")
    if payload.get("coordinate_frame") != FRAME:
        raise ProtocolError("仅接收 CAMERA_PINHOLE_PRELIMINARY 光学坐标")
    if (payload.get("robot_motion_authorized") is not False or
            payload.get("robot_base_coordinate_calibrated") is not False or
            "robot_command" not in payload or payload["robot_command"] is not None):
        raise ProtocolError("结果只能包含观测数据，不能包含机器人运动命令")
    schema = payload.get("schema")
    status = payload.get("status")
    demo = schema == "paint_v54_xyz_demo_v1"
    if demo:
        if payload.get("demo_only") is not True or status != "DEMO_ONLY_NOT_VALID_MEASUREMENT":
            raise ProtocolError("演示数据必须明确标注 demo_only 和演示状态")
        kind = "demo"
    elif schema == "paint_v54_pi_remote_result_v1":
        digest = payload.get("capture_manifest_sha256")
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ProtocolError("实际结果必须关联 capture_manifest_sha256")
        if payload.get("demo_only", False) is not False:
            raise ProtocolError("模拟数据必须使用演示 schema")
        if status == "OK_3D_POINTS":
            kind = "measurement"
            if payload.get("depth_source") != "MEASURED":
                raise ProtocolError("正式三维结果必须声明 depth_source=MEASURED")
        elif isinstance(status, str) and status.startswith("HOLD_"):
            kind = "hold"
        else:
            raise ProtocolError("未知 status；有效三维点统一使用 OK_3D_POINTS")
    else:
        raise ProtocolError("不支持的结果 schema")
    records = payload.get("points_xyz_camera_mm")
    if not isinstance(records, list) or len(records) > 100:
        raise ProtocolError("points_xyz_camera_mm 必须是最多 100 点的数组")
    if (kind == "hold" and records) or (kind != "hold" and not records):
        raise ProtocolError("HOLD 结果必须为空，其他结果必须有三维点")
    points, ids = [], set()
    for item in records:
        if not isinstance(item, dict):
            raise ProtocolError("每个点必须为带 point_id 和 xyz_camera_mm 的对象")
        point_id = item.get("point_id")
        if not isinstance(point_id, str) or not point_id or point_id in ids:
            raise ProtocolError("point_id 缺失或重复")
        ids.add(point_id)
        xyz = _numbers(item.get("xyz_camera_mm"), 3, "xyz_camera_mm")
        if xyz[2] <= 0:
            raise ProtocolError("光学坐标 Z 深度必须为正")
        if "uv" in item:
            _numbers(item["uv"], 2, "uv")
        points.append(xyz)
    # 不使用主机处理时间或接收时间冒充拍照时间，旧结果没有该字段则返回零时间。
    stamp = payload.get("observed_at_ros", {"sec": 0, "nanosec": 0})
    if (not isinstance(stamp, dict) or set(stamp) != {"sec", "nanosec"} or
            type(stamp["sec"]) is not int or not 0 <= stamp["sec"] <= 2147483647 or
            type(stamp["nanosec"]) is not int or not 0 <= stamp["nanosec"] < 1000000000):
        raise ProtocolError("observed_at_ros 必须为 ROS 原始采集时间 sec/nanosec")
    return kind, points, stamp
