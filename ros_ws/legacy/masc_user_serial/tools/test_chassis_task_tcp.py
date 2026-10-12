#!/usr/bin/env python3
"""Small supervisor-side client for end-to-end chassis task testing."""

import argparse
import json
import select
import socket
import time
import uuid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="169.254.199.100")
    parser.add_argument("--port", type=int, default=9001)
    parser.add_argument("--task", default="forward_10cm")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    sequence = 0
    started = time.monotonic()
    event_id = "task-%s-%s" % (args.task, uuid.uuid4().hex[:10])
    command_acked = False
    saw_running = False
    last_command = 0.0
    last_heartbeat = 0.0
    buffer = bytearray()

    def send(sock, message):
        nonlocal sequence
        sequence += 1
        envelope = {
            "version": 1,
            "type": message["type"],
            "sequence": sequence,
            "timestamp_ns": time.time_ns(),
            "source": "supervisor",
        }
        envelope.update({key: value for key, value in message.items() if key != "type"})
        sock.sendall((json.dumps(envelope, separators=(",", ":")) + "\n").encode())

    command = {
        "type": "control_command",
        "event_id": event_id,
        "command": "execute_task",
        "task": args.task,
    }

    print("Connecting to %s:%d ..." % (args.host, args.port))
    with socket.create_connection((args.host, args.port), timeout=5.0) as sock:
        sock.setblocking(False)
        print("Connected. Sending task:", args.task)
        while time.monotonic() - started < args.timeout:
            now = time.monotonic()
            if now - last_heartbeat >= 1.0:
                send(sock, {
                    "type": "heartbeat",
                    "uptime_ms": int((now - started) * 1000),
                    "status": "ok",
                })
                last_heartbeat = now
            if not command_acked and now - last_command >= 0.5:
                send(sock, command)
                last_command = now

            readable, _, _ = select.select([sock], [], [], 0.1)
            if not readable:
                continue
            chunk = sock.recv(65536)
            if not chunk:
                raise ConnectionError("底盘关闭了 TCP 连接")
            buffer.extend(chunk)
            while b"\n" in buffer:
                line, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)
                if not line.strip():
                    continue
                message = json.loads(line.decode("utf-8"))
                if message.get("type") == "ack" and message.get("ack_event_id") == event_id:
                    command_acked = True
                    print("Command ACK received")
                    continue
                if message.get("type") != "chassis_status":
                    continue
                if message.get("task") != args.task:
                    continue
                state = message.get("state")
                print("state=%s progress=%s message=%s" % (
                    message.get("state_name"), message.get("progress", "-"),
                    message.get("message", "")))
                if state == 1:
                    saw_running = True
                if state in (2, 3):
                    send(sock, {"type": "ack", "ack_event_id": message["event_id"]})
                    if state == 2 and saw_running:
                        print("PASS: task completed and terminal ACK sent")
                        return 0
                    if state == 2:
                        raise RuntimeError("收到 completed 前没有收到 running")
                    raise RuntimeError("底盘任务异常: %s" % message.get("message", "unknown"))
        raise TimeoutError("等待任务完成超时 %.1f 秒" % args.timeout)


if __name__ == "__main__":
    raise SystemExit(main())
