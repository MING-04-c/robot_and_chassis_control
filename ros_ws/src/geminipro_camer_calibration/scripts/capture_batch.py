#!/usr/bin/env python3
"""复用正在运行的相机，按主机 V5.4 格式批量拍照；不启动或停止驱动。"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from orbbec_camera_msgs.srv import GetCaptureFrame

from capture_v54 import RetryCapture, save_capture, verify_capture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="全新的数据集目录")
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--interval", type=float, default=1.0, help="组间隔秒数")
    parser.add_argument("--camera-namespace", default="camera")
    args = parser.parse_args()
    if not 1 <= args.count <= 100 or not 0 <= args.interval <= 3600:
        parser.error("count 应为 1～100，interval 应为 0～3600 秒")
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node("geminipro_capture_batch")
    client = node.create_client(
        GetCaptureFrame, f"/{args.camera_namespace.strip('/')}/capture_frame"
    )
    groups = []
    try:
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError("未找到 SDK 拍照服务，请先启动相机 launch")
        for index in range(args.count):
            session = output / f"camera_v5_4_{time.time_ns()}"
            for attempt in range(1, 4):
                future = client.call_async(GetCaptureFrame.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=26)
                if not future.done() or future.result() is None:
                    raise RuntimeError("SDK 拍照服务超时")
                try:
                    folder = save_capture(
                        future.result(), session / f"attempt_{attempt:02d}"
                    )
                except RetryCapture:
                    if attempt == 3:
                        raise
                    continue
                manifest = verify_capture(folder)
                groups.append(
                    {
                        "directory": str(folder.relative_to(output)),
                        "depth_scale_mm_per_raw": manifest["depth_scale_mm_per_raw"],
                        "sdk_backend": manifest["sdk_backend"],
                    }
                )
                print(f"[{index + 1}/{args.count}] {folder}", flush=True)
                break
            if index + 1 < args.count:
                time.sleep(args.interval)
        (output / "dataset_summary.json").write_text(
            json.dumps(
                {
                    "schema": "geminipro_v54_dataset_v1",
                    "created_at": datetime.now().astimezone().isoformat(),
                    "count": len(groups),
                    "groups": groups,
                    "note": "RGB + SDK raw uint16 depth + same-session parameters; Z_mm = raw * depth_scale_mm_per_raw",
                },
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        print("DATASET_COMPLETE", output, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
