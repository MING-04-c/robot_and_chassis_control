#!/usr/bin/env python3
"""主机端命令行工具：生成密钥、发送实际结果或明确标记的演示点，无需 ROS。"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import secrets
import socket
import sys
import time
from uuid import uuid4

from industrial_pc_protocol import (
    authenticate, decode, envelope, load_key, read_message, validate_result, write_message,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    keygen = commands.add_parser("keygen", help="生成共享密钥；不会覆盖已有文件")
    keygen.add_argument("--key-file", required=True)
    for command in ("send", "demo", "thickness"):
        child = commands.add_parser(command)
        child.add_argument("--host", required=True, help="树莓派 IP")
        child.add_argument("--port", type=int, default=9010)
        child.add_argument("--key-file", default="", help="留空为默认无密钥模式")
        if command != "thickness":
            child.add_argument("--file", required=True, help="结果 JSON 或原始 DEMO_XYZ 文件")
        child.add_argument("--retries", type=int, default=2)
    args = parser.parse_args()
    if args.command == "keygen":
        path = Path(args.key_file).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        # 创建时即限制权限，不打印密钥内容。
        import os
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as stream:
            stream.write(secrets.token_hex(32) + "\n")
        print("共享密钥已生成：" + str(path))
        return
    if not 0 <= args.retries <= 10:
        parser.error("--retries 必须在 0～10 之间")
    key = load_key(args.key_file) if args.key_file else None
    value = (decode(Path(args.file).expanduser().read_bytes())
             if args.command != "thickness" else None)
    if args.command == "thickness":
        packet = envelope({"schema": "paint_thickness_latest_request_v1",
                           "message_id": uuid4().hex}, key)
    elif args.command == "demo":
        # 只允许包装原本就标记为 DEMO 的文件，不能把 HOLD 伪造成成功结果。
        if value.get("schema") != "paint_v54_xyz_demo_v1":
            parser.error("demo 仅接受 paint_v54_xyz_demo_v1 文件")
        value = dict(value)
        value["message_id"] = uuid4().hex
        value["created_at_host"] = datetime.now(timezone.utc).isoformat()
        packet = envelope(value, key)
    elif "protocol" in value:
        authenticate(value, key)  # 现有签名封装原样发送，禁止默默替换签名。
        packet = value
    else:
        packet = envelope(value, key)  # 允许主机传入未封装的实际 payload。
    if args.command != "thickness":
        validate_result(packet["payload"])
    # 一次调用内重试复用同一 ID 和同一报文，防止重复入队。
    for attempt in range(args.retries + 1):
        try:
            with socket.create_connection((args.host, args.port), timeout=8) as connection:
                connection.settimeout(10)
                write_message(connection, packet)
                ack = authenticate(read_message(connection), key)
            if ack.get("schema") != "paint_v54_pi_result_ack_v1":
                raise ValueError("返回消息不是结果 ACK")
            print(json.dumps(ack, ensure_ascii=False, indent=2))
            if ack.get("ok") is not True:
                raise ValueError("树莓派拒绝该结果")
            if ack.get("message_id") != packet["payload"]["message_id"]:
                raise ValueError("ACK message_id 不匹配")
            return
        except (OSError, EOFError) as error:
            if attempt == args.retries:
                raise
            print(f"通信失败，保持原 ID 重试：{error}", file=sys.stderr)
            time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, EOFError) as error:
        print("ERROR: " + str(error), file=sys.stderr)
        sys.exit(1)
