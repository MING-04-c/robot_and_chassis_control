#!/usr/bin/env python3
"""从本地棋盘格照片标定普通针孔相机，不连接相机或机械臂。"""

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import yaml

IMAGE_EXTENSIONS = {".png", ".bmp", ".jpg", ".jpeg", ".tif", ".tiff"}


def read_image(path):
    """输入图片路径，返回 BGR uint8 图像；无法读取返回 None，兼容中文路径。"""
    try:
        data = np.fromfile(path, dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR) if data.size else None
    except (OSError, cv2.error):
        return None


def save_image(path, image):
    """输入输出路径和图像，保存 PNG；写入失败时抛出异常。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError(f"图像编码失败：{path}")
    encoded.tofile(path)


def object_points(cols, rows, square_size_mm):
    """输入内角点列、行及格长，返回棋盘格平面上的 N×3 毫米坐标。"""
    points = np.zeros((cols * rows, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size_mm
    return points


def detect_corners(image, pattern):
    """输入 BGR 图像和 (列数, 行数)，返回亚像素 N×1×2 角点或 None。"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def reprojection_error(observed, projected):
    """输入对应的像素坐标，返回欧氏距离的 RMS、均值、最大值及平方和。"""
    residual = observed.reshape(-1, 2) - projected.reshape(-1, 2)
    squared = np.sum(residual.astype(np.float64) ** 2, axis=1)
    distance = np.sqrt(squared)
    return {
        "rms_px": float(np.sqrt(np.mean(squared))),
        "mean_distance_px": float(np.mean(distance)),
        "max_distance_px": float(np.max(distance)),
        "squared_error_sum_px2": float(np.sum(squared)),
    }


def load_config():
    """解析 YAML 和命令行覆盖值，返回已校验的配置；YAML 相对路径以其目录为准。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True, help="YAML 配置路径")
    parser.add_argument("--image-dir", type=Path, help="标定照片目录")
    parser.add_argument("--output-dir", type=Path, help="新的空输出目录")
    parser.add_argument("--validation-dir", type=Path, help="独立验证照片目录")
    parser.add_argument("--board-cols", type=int, help="每行内角点数")
    parser.add_argument("--board-rows", type=int, help="每列内角点数")
    parser.add_argument("--square-size-mm", type=float, help="实测方格边长，毫米")
    args = parser.parse_args()
    config_path = args.config.expanduser().resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("配置必须是 YAML 字典")
    for key in (
        "image_dir",
        "output_dir",
        "board_cols",
        "board_rows",
        "square_size_mm",
    ):
        if getattr(args, key) is not None:
            config[key] = getattr(args, key)
        if config.get(key) is None:
            raise ValueError(f"缺少配置：{key}")
    if args.validation_dir is not None:
        config["validation_dir"] = args.validation_dir
    for key in ("image_dir", "output_dir", "validation_dir"):
        value = config.get(key)
        if value is None:
            continue
        path = Path(value).expanduser()
        # 命令行路径按当前工作目录解析，配置文件路径按配置所在目录解析。
        if not path.is_absolute():
            path = (
                Path.cwd() if getattr(args, key) is not None else config_path.parent
            ) / path
        config[key] = str(path.resolve())
    for key in ("board_cols", "board_rows", "min_views"):
        value = config.get(key, 10)
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < (3 if key == "min_views" else 2)
        ):
            raise ValueError(
                f"{key} 必须是有效整数，最小值为 {'3' if key == 'min_views' else '2'}"
            )
        config[key] = value
    for key, default in (("square_size_mm", None), ("undistort_alpha", 1.0)):
        value = float(config.get(key, default))
        if not math.isfinite(value) or (key == "square_size_mm" and value <= 0):
            raise ValueError(f"{key} 必须是有效数值")
        config[key] = value
    if not 0 <= config["undistort_alpha"] <= 1:
        raise ValueError("undistort_alpha 必须在 0 到 1 之间")
    warning = config.get("error_warning_px")
    if warning is not None:
        warning = float(warning)
        if not math.isfinite(warning) or warning <= 0:
            raise ValueError("error_warning_px 必须大于零或为 null")
    config["error_warning_px"] = warning
    image_dir = Path(config["image_dir"])
    output_dir = Path(config["output_dir"])
    if not image_dir.is_dir():
        raise ValueError(f"照片目录不存在：{image_dir}")
    if config.get("validation_dir"):
        validation_dir = Path(config["validation_dir"])
        if not validation_dir.is_dir() or validation_dir == image_dir:
            raise ValueError("独立验证目录必须存在，且不能与标定目录相同")
    if output_dir.exists() and (not output_dir.is_dir() or any(output_dir.iterdir())):
        raise ValueError("输出目录必须为空，请使用新的目录，以免覆盖或混入旧结果")
    config["config_file"] = str(config_path)
    return config


def scan_images(directory, group, pattern, output, expected_size=None):
    """扫描单层照片目录，返回逐图记录、成功角点和统一尺寸；不缩放、不删图。"""
    paths = sorted(
        p
        for p in Path(directory).iterdir()
        if p.suffix.lower() in IMAGE_EXTENSIONS and p.is_file()
    )
    if not paths:
        raise ValueError(f"没有支持的照片：{directory}")
    records, views = [], []
    for index, path in enumerate(paths):
        record = {"group": group, "file": str(path), "status": "", "review": False}
        records.append(record)
        image = read_image(path)
        if image is None:
            record["status"] = "unreadable"
            continue
        size = (image.shape[1], image.shape[0])
        record["image_size"] = list(size)
        if expected_size is None:
            expected_size = size
        if size != expected_size:
            record["status"] = "resolution_mismatch"
            continue
        corners = detect_corners(image, pattern)
        visual = image.copy()
        if corners is None:
            record["status"] = "corners_not_found"
        else:
            record["status"] = "detected"
            cv2.drawChessboardCorners(visual, pattern, corners, True)
            views.append((path, corners, record))
        filename = f"{index:04d}_{path.stem}.png"
        record["visualization"] = f"corners/{group}/{filename}"
        save_image(output / record["visualization"], visual)
        print(f"[{group}] {path.name}: {record['status']}")
    return records, views, expected_size


def write_report(output, report):
    """保存可审计 JSON 与逐图 CSV，包含失败原因和偏高误差提示。"""
    (output / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    fields = [
        "group",
        "file",
        "status",
        "review",
        "rms_px",
        "mean_distance_px",
        "max_distance_px",
    ]
    with (output / "per_image_errors.csv").open(
        "w", newline="", encoding="utf-8-sig"
    ) as stream:
        writer = csv.DictWriter(stream, fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(report["images"])


def save_comparisons(views, output, group, matrix, distortion, new_matrix):
    """对所有成功检测照片生成原图/去畸变对比，保持原分辨率且不裁剪。"""
    for index, (path, _, record) in enumerate(views):
        image = read_image(path)
        if image is None:
            raise RuntimeError(f"生成对比时原图已不可读：{path}")
        corrected = cv2.undistort(image, matrix, distortion, None, new_matrix)
        comparison = np.hstack((image, corrected))
        cv2.putText(
            comparison,
            "Original | Undistorted (new K, no crop)",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )
        record["comparison"] = f"comparisons/{group}/{index:04d}_{path.stem}.png"
        save_image(output / record["comparison"], comparison)


def run_calibration(config):
    """输入校验后的配置，输出 K、畸变、误差报告、角点与对比图片；不剔除高误差视图。"""
    output = Path(config["output_dir"])
    output.mkdir(parents=True, exist_ok=True)
    pattern = (config["board_cols"], config["board_rows"])
    records, views, size = scan_images(
        config["image_dir"], "calibration", pattern, output
    )
    validation_views = []
    if config.get("validation_dir"):
        validation_records, validation_views, _ = scan_images(
            config["validation_dir"], "validation", pattern, output, size
        )
        records.extend(validation_records)
    report = {
        "success": False,
        "opencv_version": cv2.__version__,
        "config": config,
        "images": records,
        "failed_images": [r["file"] for r in records if r["status"] != "detected"],
    }
    write_report(output, report)
    invalid = [
        r for r in records if r["status"] in ("unreadable", "resolution_mismatch")
    ]
    if invalid:
        raise ValueError(
            "有不可读取或分辨率不一致的照片，已写入 report.json，请检查后使用新的输出目录重跑"
        )
    if len(views) < config["min_views"]:
        raise ValueError(
            f"成功检测仅 {len(views)} 张，少于 min_views={config['min_views']}，请查看角点图并补拍"
        )
    obj = object_points(*pattern, config["square_size_mm"])
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        [obj.copy() for _ in views], [v[1] for v in views], size, None, None
    )
    if not all(
        np.all(np.isfinite(a))
        for a in (matrix, distortion, np.asarray(rvecs), np.asarray(tvecs))
    ) or not math.isfinite(rms):
        raise ValueError("标定产生非有限数值，请检查视角变化、角点和标定板")
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("标定得到无效焦距")
    for (_, corners, record), rotation, translation in zip(views, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rotation, translation, matrix, distortion)
        record.update(reprojection_error(corners, projected))
        record.update(
            status="calibrated",
            board_to_camera_rvec_rad=rotation.ravel().tolist(),
            board_to_camera_tvec_mm=translation.ravel().tolist(),
        )
    errors = np.array([v[2]["rms_px"] for v in views])
    median = float(np.median(errors))
    mad = float(np.median(np.abs(errors - median)))
    threshold = config["error_warning_px"]
    if threshold is None:
        threshold = max(median * 2, median + 3 * 1.4826 * mad, 1e-6)
    for _, _, record in views:
        record["review"] = record["rms_px"] > threshold
    for _, corners, record in validation_views:
        ok, rotation, translation = cv2.solvePnP(obj, corners, matrix, distortion)
        if not ok:
            record["status"] = "pose_estimation_failed"
            continue
        projected, _ = cv2.projectPoints(obj, rotation, translation, matrix, distortion)
        record.update(reprojection_error(corners, projected))
        record.update(status="validated_pose_only", review=record["rms_px"] > threshold)
    new_matrix, roi = cv2.getOptimalNewCameraMatrix(
        matrix, distortion, size, config["undistort_alpha"], size
    )
    all_corners = np.concatenate([v[1].reshape(-1, 2) for v in views])
    grid = np.zeros((3, 3), np.int32)
    coverage = np.full((size[1], size[0], 3), 255, np.uint8)
    for x, y in all_corners:
        col = min(2, max(0, int(x * 3 / size[0])))
        row = min(2, max(0, int(y * 3 / size[1])))
        grid[row, col] += 1
        cv2.circle(coverage, (round(float(x)), round(float(y))), 2, (0, 100, 0), -1)
    save_image(output / "corner_coverage.png", coverage)
    save_comparisons(views, output, "calibration", matrix, distortion, new_matrix)
    save_comparisons(
        validation_views, output, "validation", matrix, distortion, new_matrix
    )
    result = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "camera_model": "opencv_pinhole_5",
        "image_width": size[0],
        "image_height": size[1],
        "camera_matrix": matrix.tolist(),
        "distortion_order": ["k1", "k2", "p1", "p2", "k3"],
        "distortion_coefficients": distortion.ravel().tolist(),
        "opencv_rms_px": float(rms),
        "new_camera_matrix_uncropped": new_matrix.tolist(),
        "valid_pixel_roi_xywh": list(map(int, roi)),
        "config": config,
    }
    (output / "calibration.yaml").write_text(
        yaml.safe_dump(result, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    np.savez(
        output / "calibration.npz",
        camera_matrix=matrix,
        distortion_coefficients=distortion,
        image_size=np.array(size),
        new_camera_matrix=new_matrix,
    )
    total_sse = sum(v[2]["squared_error_sum_px2"] for v in views)
    report.update(
        success=True,
        calibration_views=len(views),
        validation_views=len(validation_views),
        opencv_rms_px=float(rms),
        recomputed_rms_px=float(np.sqrt(total_sse / (len(views) * len(obj)))),
        review_threshold_px=float(threshold),
        corner_counts_grid_3x3=grid.tolist(),
    )
    report["review_images"] = [r["file"] for r in records if r["review"]]
    report["failed_images"] = [
        r["file"]
        for r in records
        if r["status"] not in ("calibrated", "validated_pose_only")
    ]
    report["warnings"] = []
    if np.any(grid == 0):
        report["warnings"].append("部分九宫格区域没有角点覆盖，请补拍边缘和角落")
    if not validation_views:
        report["warnings"].append("没有独立验证照片；训练误差不能代表实际定位精度")
    if not (0 <= matrix[0, 2] < size[0] and 0 <= matrix[1, 2] < size[1]):
        report["warnings"].append("主点在画面之外，请复查标定板规格和视角变化")
    write_report(output, report)
    print(
        f"\nOpenCV 总体 RMS = {rms:.6f} px；复算 = {report['recomputed_rms_px']:.6f} px"
    )
    print(f"内参矩阵 K：\n{matrix}\n畸变 [k1,k2,p1,p2,k3]：{distortion.ravel()}")
    for record in records:
        print(
            f"{record['group']}/{Path(record['file']).name}: {record['status']}, RMS={record.get('rms_px', 'N/A')} px"
            + ("，误差偏高，请检查" if record["review"] else "")
        )
    for warning in report["warnings"]:
        print(f"提示：{warning}")
    print(f"结果：{output}。success 表示计算完成，不表示达到实际定位精度要求。")
    return result


def main():
    """命令行入口：失败返回非零退出码并显示中文错误原因。"""
    try:
        run_calibration(load_config())
    except (ValueError, OSError, cv2.error, yaml.YAMLError, RuntimeError) as error:
        raise SystemExit(f"标定失败：{error}") from error


if __name__ == "__main__":
    main()
