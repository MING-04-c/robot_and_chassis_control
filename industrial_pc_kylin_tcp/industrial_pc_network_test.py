#!/usr/bin/env python3
"""麒麟工控机网口与 TCP 分阶段检查；仅使用 Python 3 标准库，不修改网络配置。"""

import argparse
from datetime import datetime, timezone
import errno
import ipaddress
import json
import math
from pathlib import Path
import platform
import shutil
import socket
import struct
import subprocess
import sys
import time


class CheckReport:
    def __init__(self):
        self.data = {"schema": "industrial_pc_network_check_v1",
                     "checked_at_utc": datetime.now(timezone.utc).isoformat(),
                     "system": platform.platform(), "checks": []}

    def add(self, name, ok, message, **details):
        state = "PASS" if ok is True else "FAIL" if ok is False else "INFO"
        print("[{}] {}：{}".format(state, name, message), flush=True)
        self.data["checks"].append({"name": name, "ok": ok, "message": message, **details})

    def save(self, path, success):
        self.data["success"] = success
        if path:
            path = Path(path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(self.data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print("检查报告：" + str(path))


def command_json(arguments):
    """执行只读 ip 命令，返回 JSON；不可用时返回 None 供标准库降级。"""
    executable = shutil.which("ip")
    if not executable:
        return None
    try:
        result = subprocess.run([executable] + arguments, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, universal_newlines=True, timeout=3)
        return json.loads(result.stdout) if result.returncode == 0 else None
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return None


def read_sysfs(path):
    try:
        return path.read_text().strip()
    except OSError:
        return None


def primary_ipv4(interface):
    """Linux ioctl 降级查询主 IPv4；不配置网卡、不发送业务数据。"""
    import fcntl
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as handle:
        request = struct.pack("256s", interface.encode("utf-8")[:15])
        response = fcntl.ioctl(handle.fileno(), 0x8915, request)  # SIOCGIFADDR
        return socket.inet_ntoa(response[20:24])


def interfaces(report):
    """返回网卡列表及全部可查询的 IPv4 地址，同时打印检查信息。"""
    root = Path("/sys/class/net")
    if not root.is_dir():
        raise RuntimeError("本工具需要 Linux 的 /sys/class/net，当前无法读取网卡列表")
    addresses = command_json(["-j", "-4", "address", "show"])
    by_name = {item["ifname"]: [value["local"] for value in item.get("addr_info", [])
                              if value.get("family") == "inet"]
               for item in addresses} if addresses is not None else {}
    if addresses is None:
        report.add("网卡查询", None, "ip JSON 查询不可用，使用 sysfs/ioctl；每张网卡仅查询主 IPv4")
    values = []
    for folder in sorted(root.iterdir()):
        ips = by_name.get(folder.name, [])
        if addresses is None:
            try:
                ips = [primary_ipv4(folder.name)]
            except OSError:
                ips = []
        raw_flags = read_sysfs(folder / "flags")
        item = {"interface": folder.name, "ipv4": ips,
                "state": read_sysfs(folder / "operstate"),
                "carrier": read_sysfs(folder / "carrier"),
                "mac": read_sysfs(folder / "address"),
                "admin_up": bool(int(raw_flags, 0) & 1) if raw_flags else None}
        values.append(item)
        report.add("网卡 " + folder.name, None,
                   "state={} carrier={} IPv4={}".format(item["state"], item["carrier"], ",".join(ips) or "未配置"),
                   **item)
    return values


def check_interface(args, report):
    """先检查网卡、链路与本机 IP，再尝试创建并绑定本地 TCP socket。"""
    values = interfaces(report)
    candidates = [item for item in values if
                  (item["interface"] == args.interface if args.interface else args.local_ip in item["ipv4"])]
    if len(candidates) != 1:
        report.add("选择网口", False,
                   "找不到唯一匹配网口；请确认本机 IP 并使用 --interface 指定实际网卡名")
        return False
    item = candidates[0]
    report.data["selected_interface"] = item["interface"]
    report.data["local_ip"] = args.local_ip
    if args.local_ip not in item["ipv4"]:
        report.add("本机 IP", False, "{} 未配置在 {}，不能绑定".format(args.local_ip, item["interface"]))
        return False
    report.add("本机 IP", True, "{} 位于 {}".format(args.local_ip, item["interface"]))
    if item["admin_up"] is False or item["carrier"] == "0" or item["state"] in ("down", "lowerlayerdown", "notpresent"):
        report.add("网口链路", False, "网口未启用或无载波；检查网线、交换机及网卡启用状态")
        return False
    if item["admin_up"] is None or item["state"] not in ("up", "unknown"):
        report.add("网口链路", None, "无法完整确认链路状态，继续以实际 socket 测试为准")
    else:
        report.add("网口链路", True, "系统报告接口已启用；这不代表对端服务已启动")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as handle:
            handle.bind((args.local_ip, 0))
            endpoint = handle.getsockname()
        report.add("调用网口", True, "成功创建 TCP socket 并绑定 {}:{}".format(*endpoint))
        return True
    except OSError as error:
        report.add("调用网口", False, str(error), errno=error.errno)
        return False


def failure_hint(error):
    code = error.errno
    if isinstance(error, socket.timeout) or code == errno.ETIMEDOUT:
        return "连接超时：检查对端开机、网线、地址、路由及防火墙；不能据此单独认定网卡故障"
    return {
        errno.ECONNREFUSED: "连接被拒绝：检查对端该端口是否正在监听，或是否被防火墙主动拒绝",
        errno.ENETUNREACH: "网络不可达：检查本机 IP、掩码与路由",
        errno.EHOSTUNREACH: "主机不可达：检查对端地址、链路、邻居解析与路由",
        errno.EADDRNOTAVAIL: "本机没有配置该源 IP",
        errno.EACCES: "系统安全策略拒绝 socket 操作，检查麒麟安全策略及应用权限",
        errno.EPERM: "系统安全策略拒绝 socket 操作，检查麒麟安全策略及应用权限",
    }.get(code, str(error))


def check_connections(args, report):
    """绑定本机源 IP 后逐一测试 TCP 握手，不发送照片或检测结果。"""
    route = command_json(["-j", "route", "get", args.host, "from", args.local_ip])
    if route:
        device = route[0].get("dev")
        report.add("路由", None, "目标 {}，出口 {}".format(args.host, device), route=route)
        if device and device != report.data["selected_interface"]:
            report.add("路由出口", False, "系统路由出口与选择网口不一致，请先调整网络配置")
            return False
    else:
        report.add("路由", None, "无法读取路由信息，继续尝试绑定源 IP 连接")
    passed = True
    for port in args.ports:
        start = time.monotonic()
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as handle:
                handle.settimeout(args.timeout)
                handle.bind((args.local_ip, 0))
                handle.connect((args.host, port))
                source = handle.getsockname()
            report.add("TCP {}:{}".format(args.host, port), True,
                       "连接成功，源 {}:{}，耗时 {:.1f} ms；仅证明 TCP 握手成功".format(
                           source[0], source[1], (time.monotonic() - start) * 1000))
        except OSError as error:
            passed = False
            report.add("TCP {}:{}".format(args.host, port), False,
                       failure_hint(error), errno=error.errno)
    return passed


def listen_once(args, report):
    """开一个独立测试端口等待一次连接；不冒充任何业务协议服务。"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.local_ip, args.port))
            server.listen(1)
            server.settimeout(args.wait)
            report.add("测试监听", True, "{}:{}，等待一次连接，最长 {} 秒".format(
                args.local_ip, args.port, args.wait))
            connection, peer = server.accept()
            with connection:
                report.add("接收到连接", True, "对端 {}:{}；未收发业务数据".format(*peer))
        return True
    except OSError as error:
        report.add("测试监听/接收", False, failure_hint(error), errno=error.errno)
        return False


def ipv4(value):
    try:
        address = ipaddress.IPv4Address(value)
        if address.is_unspecified or address.is_multicast or int(address) == 0xffffffff:
            raise ValueError("必须是具体主机地址")
        return str(address)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error))


def port_number(value):
    number = int(value)
    if not 1 <= number <= 65535:
        raise argparse.ArgumentTypeError("端口必须在 1～65535 之间")
    return number


def positive_seconds(value):
    number = float(value)
    if not math.isfinite(number) or not 0 < number <= 3600:
        raise argparse.ArgumentTypeError("秒数必须是大于 0、不超过 3600 的有限值")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command")
    for name in ("interfaces", "connect", "listen"):
        child = commands.add_parser(name)
        child.add_argument("--interface", help="实际网口名，例如 enp3s0；留空按本机 IP 查找")
        child.add_argument("--local-ip", type=ipv4, default="169.254.199.120", help="运行本工具这台机器的 IP")
        child.add_argument("--report", help="保存 JSON 检查报告")
        if name == "connect":
            child.add_argument("--host", "--pi-ip", dest="host", type=ipv4, required=True, help="对端 IP；通常填树莓派地址")
            child.add_argument("--ports", type=port_number, nargs="+", default=[9002, 9010])
            child.add_argument("--timeout", type=positive_seconds, default=3.0)
        if name == "listen":
            child.add_argument("--port", type=port_number, default=19030, help="独立连通性测试端口，不使用业务端口")
            child.add_argument("--wait", type=positive_seconds, default=30.0)
    args = parser.parse_args()
    if args.command is None:
        parser.error("请选择 interfaces、connect 或 listen")
    report = CheckReport()
    try:
        success = check_interface(args, report)
        if success and args.command == "connect":
            success = check_connections(args, report)
        elif success and args.command == "listen":
            success = listen_once(args, report)
    except (OSError, RuntimeError, ValueError) as error:
        report.add("检查中断", False, str(error))
        success = False
    except KeyboardInterrupt:
        report.add("检查中断", False, "用户取消")
        success = False
    try:
        report.save(args.report, success)
    except OSError as error:
        print("[FAIL] 保存报告：" + str(error), file=sys.stderr)
        return 1
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
