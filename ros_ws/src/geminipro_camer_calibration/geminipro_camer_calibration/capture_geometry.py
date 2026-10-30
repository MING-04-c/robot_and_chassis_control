"""使用拍照时冻结的光学坐标变换，生成基座下的探针触碰位姿。"""

import numpy as np


def checked_transform(value):
    """输入 4×4 米制刚体变换；返回验证后的 ndarray。"""
    transform = np.asarray(value, dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0., 0., 0., 1.], atol=1e-8)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1., atol=1e-6)):
        raise ValueError('拍照变换必须为有限的 4×4 刚体矩阵')
    return transform


def frozen_probe_targets(camera_base, probe_base, points_mm):
    """输入拍照时 base_T_camera、base_T_probe 和光学毫米点；输出基座米制探针位姿列表。

    只转换位置；接触朝向保持拍照时探针朝向。tool0 补偿统一由 detect_control 完成。
    """
    camera, probe = checked_transform(camera_base), checked_transform(probe_base)
    points = np.asarray(points_mm, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or not 1 <= len(points) <= 100
            or not np.isfinite(points).all() or np.any(points[:, 2] <= 0)
            or np.any(points[:, 2] > 3000.)):
        raise ValueError('检测点必须为 1～100 个有限光学毫米坐标，深度在 0～3000 mm 内')
    targets = []
    for point in points:
        goal = probe.copy()
        goal[:3, 3] = (camera @ np.r_[point / 1000., 1.])[:3]
        targets.append(goal)
    return targets
