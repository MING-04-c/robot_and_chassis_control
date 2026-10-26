"""将当前 C++ SDK 的一次 FrameSet 响应保存为主机 V5.4 数据格式。"""

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


class RetryCapture(ValueError):
    """本组图像损坏或没有有效深度，可重新请求完整 FrameSet。"""


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def verify_capture(directory):
    """检查保存目录的原始文件、清单和完成标记，返回清单。"""
    directory = Path(directory)
    manifest = json.loads((directory / "capture_manifest.json").read_text())
    done = json.loads((directory / "CAPTURE_COMPLETE.json").read_text())
    names = {"rgb_001.png", "d2c_depth_raw.npy", "camera_parameters_same_session.json"}
    if (
        manifest.get("schema") != "paint_rgbd_v5_legacy_capture_v1"
        or set(manifest.get("source_files_sha256", {})) != names
        or done.get("sha256_capture_manifest")
        != sha256(directory / "capture_manifest.json")
        or manifest.get("robot_motion_authorized") is not False
        or done.get("robot_motion_authorized") is not False
    ):
        raise ValueError("V5.4 文件清单或完成标记无效")
    for name, digest in manifest["source_files_sha256"].items():
        path = directory / name
        if path.is_symlink() or not path.is_file() or sha256(path) != digest:
            raise ValueError("原始文件哈希不一致：" + name)
    return manifest


def save_capture(response, directory):
    """输入 SDK 服务响应和全新 attempt_XX 目录，验证后保存；返回保存目录。"""
    if not response.success:
        if response.message.startswith("No valid fresh SDK FrameSet"):
            raise RetryCapture("SDK 等待完整帧超时：" + response.message)
        raise ValueError("SDK 拍照失败：" + response.message)
    meta = json.loads(response.metadata_json)
    cam = meta["sdk_raw_camera_param"]
    scale = meta["depth_scale_mm_per_raw"]
    if (
        isinstance(scale, bool)
        or not isinstance(scale, (int, float))
        or not np.isfinite(scale)
        or not 0 < scale < 100
    ):
        raise ValueError("本次 SDK 深度比例无效")
    if meta.get("actual_align_mode") != "ALIGN_D2C_SW_MODE":
        raise ValueError("本次 SDK 未确认软件 D2C")
    if type(cam.get("isMirrored")) is not bool:
        raise ValueError("缺少 SDK 实际镜像标志")
    for name in ("rgbIntrinsic", "depthIntrinsic"):
        k = cam[name]
        if (
            (k["width"], k["height"]) != (640, 480)
            or not np.all(np.isfinite([k[f] for f in ("fx", "fy", "cx", "cy")]))
            or k["fx"] <= 0
            or k["fy"] <= 0
            or not -1 < k["cx"] < 641
            or not -1 < k["cy"] < 481
        ):
            raise ValueError("本次 SDK 内参不符合 640×480 输出：" + name)
    if any(
        abs(cam["rgbIntrinsic"][f] - cam["depthIntrinsic"][f]) > 1e-3
        for f in ("fx", "fy", "cx", "cy")
    ):
        raise ValueError("RGB 和 D2C 内参不一致")
    for name in ("rgbDistortion", "depthDistortion"):
        if not cam[name] or not np.all(np.isfinite(list(cam[name].values()))):
            raise ValueError("SDK 畸变参数无效：" + name)
    transform = cam["transform"]
    rotation = np.asarray(transform["rot"], dtype=np.float64)
    translation = np.asarray(transform["trans"], dtype=np.float64)
    if (
        rotation.shape != (9,)
        or translation.shape != (3,)
        or not np.all(np.isfinite(rotation))
        or not np.all(np.isfinite(translation))
    ):
        raise ValueError("SDK 原始变换参数无效")
    if (
        meta.get("system_timestamp_unit") != "ms"
        or meta["rgb_system_timestamp_raw"] <= 0
        or meta["depth_system_timestamp_raw"] <= 0
        or abs(meta["rgb_system_timestamp_raw"] - meta["depth_system_timestamp_raw"])
        > 100
    ):
        raise ValueError("SDK FrameSet 时间戳诊断检查失败")
    if response.color.format != "jpeg":
        raise ValueError("SDK 彩色图不是请求的 MJPG")
    rgb = cv2.imdecode(np.frombuffer(response.color.data, np.uint8), cv2.IMREAD_COLOR)
    if rgb is None or rgb.shape != (480, 640, 3):
        raise RetryCapture("本组 MJPG 解码失败，需重新采集")
    image = response.depth_raw
    if (
        (image.width, image.height) != (640, 480)
        or image.encoding != "16UC1"
        or image.step != 1280
        or len(image.data) != 640 * 480 * 2
    ):
        raise ValueError("SDK 原始深度格式不正确")
    raw = np.frombuffer(image.data, dtype=">u2" if image.is_bigendian else "<u2")
    raw = raw.reshape(480, 640).astype(np.uint16, copy=True)
    if not np.any((raw > 0) & (raw < 65535)):
        raise RetryCapture("本组深度没有有效像素，需重新采集")
    # PY/LEGACY 字符串是主机 V5.4 的协议枚举别名；实际后端另行如实记录。
    backend = {
        key: meta[key]
        for key in ("sdk_api", "sdk_version", "actual_align_mode", "session_id")
    }
    params = {
        "schema": "orbbec_d2c_parameter_snapshot_v1",
        "source": "SDK_getCameraParam_in_SAME_capture_session",
        "device_serial": meta["device_serial"],
        "device_serial_evidence": "CPP_SDK_DeviceInfo_serialNumber",
        "firmware_log": None,
        "align_mode": "OB_PY_ALIGN_D2C_SW_MODE",
        "image_size_wh": [640, 480],
        "rgb_intrinsic": cam["rgbIntrinsic"],
        "d2c_intrinsic": cam["depthIntrinsic"],
        "rgb_distortion": cam["rgbDistortion"],
        "d2c_distortion": cam["depthDistortion"],
        "sdk_isMirrored": cam["isMirrored"],
        "sdk_raw_transform_direction_and_units": "UNVERIFIED",
        "sdk_raw_camera_param": cam,
        "actual_depth_mirror_consistency_verified": False,
        "actual_rgb_mirror_consistency_verified": False,
        "rgb_depth_timestamp_sync_verified": False,
        "rgbd_geometric_registration_accuracy_verified": False,
        "hand_eye_verified": False,
        "robot_motion_authorized": False,
        "sdk_backend": backend,
        "compatibility_enum_note": "PY/LEGACY labels are V5.4 wire-format aliases; sdk_backend identifies the actual C++ SDK, not Python SDK 1.1.4",
    }
    pairing = {
        key: value
        for key, value in meta.items()
        if key not in ("sdk_raw_camera_param", "depth_scale_mm_per_raw")
    }
    pairing.update(
        source="ONE_SDK_FRAMESET_LEGACY_SW_D2C",
        exposure_sync_verified=False,
        rgbd_geometric_registration_verified=False,
        robot_motion_authorized=False,
    )
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    if not cv2.imwrite(str(directory / "rgb_001.png"), rgb):
        raise OSError("RGB PNG 写入失败")
    with (directory / "d2c_depth_raw.npy").open("xb") as stream:
        np.save(stream, raw, allow_pickle=False)
    write_json(directory / "camera_parameters_same_session.json", params)
    # PNG/NPY 保留完全相同的 SDK 原始 uint16 数值，不乘比例，也不做伪彩。
    if not cv2.imwrite(str(directory / "d2c_depth_raw_uint16.png"), raw):
        raise OSError("原始深度 PNG 写入失败")
    if (
        not np.array_equal(cv2.imread(str(directory / "rgb_001.png")), rgb)
        or not np.array_equal(
            np.load(directory / "d2c_depth_raw.npy", allow_pickle=False), raw
        )
        or not np.array_equal(
            cv2.imread(str(directory / "d2c_depth_raw_uint16.png"), -1), raw
        )
    ):
        raise OSError("图像落盘回读检查失败")
    names = ["rgb_001.png", "d2c_depth_raw.npy", "camera_parameters_same_session.json"]
    manifest = {
        "schema": "paint_rgbd_v5_legacy_capture_v1",
        "rgb_file": names[0],
        "depth_file": names[1],
        "camera_params_file": names[2],
        "depth_scale_mm_per_raw": scale,
        "intrinsic_used": cam["rgbIntrinsic"],
        "rgb_depth_pairing": pairing,
        "source_files_sha256": {name: sha256(directory / name) for name in names},
        "robot_motion_authorized": False,
        "sdk_backend": backend,
    }
    write_json(directory / "capture_manifest.json", manifest)
    write_json(
        directory / "depth_png_check.json",
        {
            "schema": "orbbec_v54_extra_uint16_depth_png_v1",
            "depth_png": "d2c_depth_raw_uint16.png",
            "depth_png_sha256": sha256(directory / "d2c_depth_raw_uint16.png"),
            "original_depth_npy_sha256": manifest["source_files_sha256"][
                "d2c_depth_raw.npy"
            ],
            "same_grid_same_raw_uint16_values": True,
            "depth_scale_mm_per_raw": scale,
            "preview_only": False,
            "robot_motion_authorized": False,
        },
    )
    write_json(
        directory / "CAPTURE_COMPLETE.json",
        {
            "sha256_capture_manifest": sha256(directory / "capture_manifest.json"),
            "robot_motion_authorized": False,
        },
    )
    verify_capture(directory)
    return directory
