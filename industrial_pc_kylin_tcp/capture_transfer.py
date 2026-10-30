#!/usr/bin/env python3
"""将已完成的 V5.4 数据集无损打包，经 TCP 发送/接收并校验。主机接收不依赖 ROS/SDK。"""

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import socket
import struct
import tempfile
import time
import zipfile

MAGIC = b"RGBD"
MAX_ARCHIVE = 128 * 1024 * 1024
MAX_HEADER = 65536
CORE = {"rgb_001.png", "d2c_depth_raw.npy", "camera_parameters_same_session.json"}
GROUP_FILES = CORE | {
    "capture_manifest.json",
    "CAPTURE_COMPLETE.json",
    "d2c_depth_raw_uint16.png",
    "depth_png_check.json",
}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_archive(path):
    """检查 ZIP 路径、体积及每组原始数据哈希；不解压，返回组数。"""
    with zipfile.ZipFile(path) as archive:
        items = archive.infolist()
        if (
            not 1 <= len(items) <= 1000
            or sum(i.file_size for i in items) > 256 * 1024 * 1024
        ):
            raise ValueError("ZIP 文件数量或展开体积超限")
        names = {item.filename for item in items}
        if len(names) != len(items):
            raise ValueError("ZIP 有重复文件名")
        if archive.testzip() is not None:
            raise ValueError("ZIP 成员 CRC 校验失败")
        folders = set()
        for item in items:
            p = PurePosixPath(item.filename)
            if (
                p.is_absolute()
                or ".." in p.parts
                or "\\" in item.filename
                or item.file_size > 32 * 1024 * 1024
                or (item.external_attr >> 16) & 0o170000 == 0o120000
            ):
                raise ValueError("ZIP 存在无效成员")
            if item.filename == "dataset_summary.json":
                continue
            if (
                len(p.parts) != 3
                or not p.parts[0].startswith("camera_v5_4_")
                or not p.parts[1].startswith("attempt_")
                or p.name not in GROUP_FILES
            ):
                raise ValueError("ZIP 成员不是 V5.4 采集文件：" + item.filename)
            folders.add(str(p.parent))
        if not folders:
            raise ValueError("ZIP 没有完整采集组")
        for folder in folders:

            def read(name):
                return archive.read(folder + "/" + name)

            manifest_bytes = read("capture_manifest.json")
            manifest = json.loads(manifest_bytes)
            done = json.loads(read("CAPTURE_COMPLETE.json"))
            if (
                manifest.get("schema") != "paint_rgbd_v5_legacy_capture_v1"
                or done.get("sha256_capture_manifest")
                != hashlib.sha256(manifest_bytes).hexdigest()
                or set(manifest.get("source_files_sha256", {})) != CORE
                or manifest.get("robot_motion_authorized") is not False
                or done.get("robot_motion_authorized") is not False
            ):
                raise ValueError("ZIP 采集清单/完成标记无效")
            for name, digest in manifest["source_files_sha256"].items():
                if hashlib.sha256(read(name)).hexdigest() != digest:
                    raise ValueError("ZIP 原始文件校验失败：" + name)
        return len(folders)


def pack(dataset, output):
    from capture_v54 import verify_capture

    dataset, output = Path(dataset).resolve(), Path(output)
    markers = sorted(dataset.glob("camera_v5_4_*/attempt_*/CAPTURE_COMPLETE.json"))
    if not markers:
        raise ValueError("没有已完成的 V5.4 采集组")
    files = []
    for marker in markers:
        verify_capture(marker.parent)
        for name in sorted(GROUP_FILES):
            path = marker.parent / name
            if not path.is_file() or path.is_symlink():
                raise ValueError("采集文件缺失或为符号链接：" + str(path))
            files.append(path)
    summary = dataset / "dataset_summary.json"
    if summary.is_file() and not summary.is_symlink():
        if json.loads(summary.read_text())["count"] != len(markers):
            raise ValueError("数据集清单组数不一致")
        files.append(summary)
    with zipfile.ZipFile(output, "x", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in files:
            archive.write(path, str(path.relative_to(dataset)))
    count = verify_archive(output)
    with output.with_suffix(output.suffix + ".sha256").open("x") as stream:
        stream.write(sha256(output) + "  " + output.name + "\n")
    print(f"PACK_PASS groups={count} archive={output}")


def read_exact(connection, count):
    data = bytearray()
    while len(data) < count:
        chunk = connection.recv(count - len(data))
        if not chunk:
            raise EOFError("TCP 数据未完整接收")
        data.extend(chunk)
    return bytes(data)


def write_json(connection, value):
    body = json.dumps(value, ensure_ascii=False).encode("utf-8")
    connection.sendall(struct.pack("!I", len(body)) + body)


def read_json(connection):
    size = struct.unpack("!I", read_exact(connection, 4))[0]
    if not 1 <= size <= MAX_HEADER:
        raise ValueError("TCP 消息头长度无效")
    return json.loads(read_exact(connection, size))


def send(path, host, port):
    path = Path(path)
    count = verify_archive(path)
    size = path.stat().st_size
    if not 1 <= size <= MAX_ARCHIVE:
        raise ValueError("ZIP 超过 TCP 单包体积限制")
    with socket.create_connection((host, port), timeout=10) as connection:
        connection.settimeout(30)
        connection.sendall(MAGIC)
        write_json(connection, {"version": 1, "size": size, "sha256": sha256(path)})
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                connection.sendall(chunk)
        ack = read_json(connection)
        if ack.get("ok") is not True:
            raise ValueError("主机拒绝接收：" + str(ack))
        print(f"TCP_SEND_PASS groups={count} host={host}:{port} ack={ack}")


def receive(bind, port, output):
    """接收一个数据包，校验后保存 ZIP 并回执；不自动解压或覆盖现有文件。"""
    output = Path(output).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((bind, port))
        server.listen(1)
        server.settimeout(300)
        print(f"TCP_LISTEN {bind}:{server.getsockname()[1]}", flush=True)
        connection, _ = server.accept()
        temporary = None
        with connection:
            connection.settimeout(30)
            try:
                if read_exact(connection, 4) != MAGIC:
                    raise ValueError("TCP 协议标识不匹配")
                header = read_json(connection)
                size = header.get("size")
                if (
                    header.get("version") != 1
                    or type(size) is not int
                    or not 1 <= size <= MAX_ARCHIVE
                ):
                    raise ValueError("TCP 版本或数据长度无效")
                with tempfile.NamedTemporaryFile(
                    dir=output, suffix=".part", delete=False
                ) as stream:
                    temporary = Path(stream.name)
                    remaining = size
                    while remaining:
                        chunk = read_exact(connection, min(1024 * 1024, remaining))
                        stream.write(chunk)
                        remaining -= len(chunk)
                if sha256(temporary) != header.get("sha256"):
                    raise ValueError("TCP ZIP SHA256 校验失败")
                count = verify_archive(temporary)
                destination = output / f"rgbd_received_{time.time_ns()}.zip"
                # 独占目标文件名，避免覆盖；临时文件完成校验后才对外可见。
                os.link(temporary, destination)
                temporary.unlink()
                temporary = None
                write_json(
                    connection,
                    {
                        "ok": True,
                        "groups": count,
                        "sha256": sha256(destination),
                        "saved_to": str(destination),
                    },
                )
                print(
                    f"TCP_RECEIVE_PASS groups={count} archive={destination}", flush=True
                )
            except Exception as error:
                try:
                    write_json(connection, {"ok": False, "error": str(error)})
                except OSError:
                    pass
                raise
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    p = commands.add_parser("pack")
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p = commands.add_parser("send")
    p.add_argument("--archive", type=Path, required=True)
    p.add_argument("--host", required=True)
    p.add_argument("--port", type=int, default=9030)
    p = commands.add_parser("receive")
    p.add_argument("--bind", default="127.0.0.1")
    p.add_argument("--port", type=int, default=9030)
    p.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "pack":
        pack(args.dataset, args.output)
    elif args.command == "send":
        send(args.archive, args.host, args.port)
    else:
        receive(args.bind, args.port, args.output)


if __name__ == "__main__":
    main()
