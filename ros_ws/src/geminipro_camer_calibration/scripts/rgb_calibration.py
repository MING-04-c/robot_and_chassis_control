"""从保存的 RGB-D 组中的彩色图计算 RGB 内参；不修改深度出厂标定。"""

import csv
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def calibrate_rgb(dataset, output, cols, rows, square_mm):
    """输入采集目录、输出目录、内角点列行和格长毫米，返回标准 ROS 内参文件路径。"""
    if cols < 2 or rows < 2 or not np.isfinite(square_mm) or square_mm <= 0:
        raise ValueError("请填写真实棋盘格列、行和实测格长")
    images = sorted(
        path
        for path in Path(dataset).glob("camera_v5_4_*/attempt_*/rgb_001.png")
        if (path.parent / "CAPTURE_COMPLETE.json").is_file()
    )
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    obj = np.zeros((cols * rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    corners_list, accepted, report = [], [], []
    resolution = None
    for path in images:
        image = cv2.imread(str(path))
        record = {"file": str(path), "status": "unreadable"}
        report.append(record)
        if image is None:
            continue
        size = (image.shape[1], image.shape[0])
        if resolution is None:
            resolution = size
        if size != resolution:
            record["status"] = "resolution_mismatch"
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(
            gray,
            (cols, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        record["status"] = "detected" if ok else "corners_not_found"
        if ok:
            corners = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4),
            )
            corners_list.append(corners)
            accepted.append(record)
            cv2.drawChessboardCorners(image, (cols, rows), corners, True)
        folder = output / "corners"
        folder.mkdir(exist_ok=True)
        cv2.imwrite(
            str(folder / (path.parent.parent.name + "_" + path.parent.name + ".png")),
            image,
        )
    (output / "detection_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    if any(row["status"] in ("unreadable", "resolution_mismatch") for row in report):
        raise ValueError("存在损坏照片或混合分辨率，请检查 detection_report.json")
    if len(corners_list) < 10:
        raise ValueError(
            f"有效棋盘格仅 {len(corners_list)} 张，至少需要 10 张；报告：{output}"
        )
    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        [obj.copy() for _ in corners_list], corners_list, resolution, None, None
    )
    if (
        not np.isfinite(rms)
        or not np.all(np.isfinite(matrix))
        or not np.all(np.isfinite(distortion))
    ):
        raise ValueError("标定结果无效，请检查照片视角变化和棋盘格规格")
    errors = []
    for record, observed, rotation, translation in zip(
        accepted, corners_list, rotations, translations
    ):
        predicted, _ = cv2.projectPoints(obj, rotation, translation, matrix, distortion)
        error = float(
            np.sqrt(
                np.mean(
                    np.sum(
                        (observed.reshape(-1, 2) - predicted.reshape(-1, 2)) ** 2,
                        axis=1,
                    )
                )
            )
        )
        record["rms_px"] = error
        errors.append(error)
    threshold = max(float(np.median(errors)) * 2, 1e-6)
    for record in accepted:
        record["review"] = record["rms_px"] > threshold
    result = {
        "image_width": resolution[0],
        "image_height": resolution[1],
        "camera_name": "camera",
        "distortion_model": "plumb_bob",
        "camera_matrix": {"rows": 3, "cols": 3, "data": matrix.ravel().tolist()},
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(distortion.ravel()),
            "data": distortion.ravel().tolist(),
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": np.eye(3).ravel().tolist(),
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": np.column_stack((matrix, np.zeros(3))).ravel().tolist(),
        },
    }
    path = output / "rgb_camera_info.yaml"
    path.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
    (output / "report.json").write_text(
        json.dumps(
            {
                "rms_px": float(rms),
                "board_cols": cols,
                "board_rows": rows,
                "square_size_mm": square_mm,
                "images": report,
                "note": "RMS 为每角点欧氏重投影误差的均方根，单位像素；没有自动删除偏高误差照片。仅 RGB 内参，未标定深度尺度/配准或手眼关系。",
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    with (output / "per_image_errors.csv").open(
        "w", newline="", encoding="utf-8-sig"
    ) as stream:
        writer = csv.DictWriter(
            stream, ["file", "status", "rms_px", "review"], extrasaction="ignore"
        )
        writer.writeheader()
        writer.writerows(report)
    for image_path in images:
        image = cv2.imread(str(image_path))
        if image is not None:
            compare = np.hstack((image, cv2.undistort(image, matrix, distortion)))
            cv2.imwrite(
                str(
                    output
                    / (
                        image_path.parent.parent.name
                        + "_"
                        + image_path.parent.name
                        + "_undistort.png"
                    )
                ),
                compare,
            )
    return str(path), float(rms)
