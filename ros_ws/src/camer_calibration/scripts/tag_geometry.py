#!/usr/bin/env python3
"""AprilTag 检测、内参读取和位姿计算；不依赖 ROS 或图形界面。"""

from pathlib import Path

import cv2
import numpy as np
import yaml

FAMILIES = {
    name: getattr(cv2.aruco, "DICT_APRILTAG_" + name[3:])
    for name in ("tag16h5", "tag25h9", "tag36h10", "tag36h11")
}


def load_intrinsics(filename):
    """读取本包输出或 ROS plumb_bob YAML，返回 K、畸变和 (宽, 高)。"""
    try:
        data = yaml.safe_load(Path(filename).expanduser().read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise ValueError("标定 YAML 格式错误：" + str(error)) from error
    if not isinstance(data, dict):
        raise ValueError("标定文件必须是 YAML 字典")
    if (
        data.get("distortion_model", "plumb_bob") != "plumb_bob"
        or data.get("camera_model", "opencv_pinhole_5") != "opencv_pinhole_5"
    ):
        raise ValueError("当前只支持普通针孔/plumb_bob 模型")

    def array(key):
        value = data[key]
        return np.asarray(
            value["data"] if isinstance(value, dict) else value, np.float64
        )

    matrix = array("camera_matrix").reshape(3, 3)
    distortion = array("distortion_coefficients").reshape(-1, 1)
    width, height = data["image_width"], data["image_height"]
    if (
        not isinstance(width, int)
        or not isinstance(height, int)
        or min(width, height) <= 0
    ):
        raise ValueError("标定分辨率必须是正整数")
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion)):
        raise ValueError("内参或畸变含无效数值")
    if (
        matrix[0, 0] <= 0
        or matrix[1, 1] <= 0
        or not np.allclose(matrix[2], [0, 0, 1])
        or len(distortion) not in (4, 5)
    ):
        raise ValueError("内参矩阵或畸变参数格式不正确")
    return matrix, distortion, (width, height)


def make_detector(family):
    """输入 AprilTag 家族，返回兼容 OpenCV 4.6 及新版的检测函数。"""
    if family not in FAMILIES:
        raise ValueError("不支持的 Tag 家族：" + family)
    dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
    # OpenCV 4.6 虽暴露同名类型，仍须用 create() 工厂构造有效参数。
    parameters = (
        cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "ArucoDetector")
        else cv2.aruco.DetectorParameters_create()
    )
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    # 增加局部阈值的尺度，兼顾大图和光照不均；不放宽编码纠错条件。
    parameters.adaptiveThreshWinSizeMax = 53
    parameters.adaptiveThreshWinSizeStep = 10
    parameters.perspectiveRemovePixelPerCell = 8
    if hasattr(cv2.aruco, "ArucoDetector"):
        detect = cv2.aruco.ArucoDetector(dictionary, parameters).detectMarkers
    else:
        detect = lambda gray: cv2.aruco.detectMarkers(
            gray, dictionary, parameters=parameters
        )

    def detect_with_border(gray):
        corners, ids, rejected = detect(gray)
        # 碰边时补白尝试解码，仅用于提示；节点会禁止靠边候选用于测距。
        edge = np.concatenate((gray[0], gray[-1], gray[:, 0], gray[:, -1]))
        if ids is None and np.count_nonzero(edge < 80) >= 3:
            padding = 24
            padded = cv2.copyMakeBorder(
                gray, padding, padding, padding, padding, cv2.BORDER_CONSTANT, value=255
            )
            padded_corners, padded_ids, _ = detect(padded)
            if padded_ids is not None:
                return (
                    tuple(corner - padding for corner in padded_corners),
                    padded_ids,
                    rejected,
                )
        return corners, ids, rejected

    return detect_with_border


def estimate_tag(corners, size_mm, matrix, distortion):
    """输入 TL/TR/BR/BL 像素角点、黑框边长毫米和内参，返回 R、t_mm、rvec、RMS_px。

    Tag 原点在中心，X 向右、Y 向上；IPPE_SQUARE 顺序必须保持一致。
    t 表示 Tag 中心在相机光学坐标系中的位置，不是相机在 Tag 系中的坐标。
    """
    half = size_mm / 2.0
    points = np.array(
        [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
        np.float64,
    )
    pixels = np.asarray(corners, np.float64).reshape(4, 2)
    ok, rotations, translations, *_ = cv2.solvePnPGeneric(
        points, pixels, matrix, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    candidates = []
    if ok:
        for rotation, translation in zip(rotations, translations):
            rotation_matrix, _ = cv2.Rodrigues(rotation)
            camera_points = points @ rotation_matrix.T + translation.reshape(1, 3)
            if not np.all(np.isfinite(camera_points)) or np.any(
                camera_points[:, 2] <= 0
            ):
                continue
            predicted, _ = cv2.projectPoints(
                points, rotation, translation, matrix, distortion
            )
            rms = float(
                np.sqrt(
                    np.mean(np.sum((predicted.reshape(4, 2) - pixels) ** 2, axis=1))
                )
            )
            candidates.append(
                (rms, rotation_matrix, translation.ravel(), rotation.ravel())
            )
    if not candidates:
        raise ValueError("没有有限且位于相机前方的位姿解")
    rms, rotation_matrix, translation, rotation = min(
        candidates, key=lambda item: item[0]
    )
    return rotation_matrix, translation, rotation, rms


def quaternion_from_rvec(rotation):
    """轴角旋转向量转为 ROS 的四元数 (x,y,z,w)。"""
    angle = float(np.linalg.norm(rotation))
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    xyz = np.asarray(rotation) * (np.sin(angle / 2) / angle)
    return (*map(float, xyz), float(np.cos(angle / 2)))
