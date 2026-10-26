"""齐次变换约定：A_T_B 把 B 坐标转换到 A；位置统一为米。"""

import math
import numpy as np


def matrix(position, quaternion):
    """输入 XYZ 和 XYZW 四元数，返回 4×4 齐次矩阵；拒绝非法数值。"""
    p = np.asarray(position, dtype=float)
    q = np.asarray(quaternion, dtype=float)
    if (
        p.shape != (3,)
        or q.shape != (4,)
        or not np.isfinite(p).all()
        or not np.isfinite(q).all()
    ):
        raise ValueError("位姿必须包含有限 XYZ 与 XYZW")
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        raise ValueError("四元数不能为零")
    x, y, z, w = q / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    result[:3, 3] = p
    return result


def components(transform):
    """输入刚性 4×4 矩阵，返回 XYZ 和归一化 XYZW。"""
    t = np.asarray(transform, dtype=float)
    r = t[:3, :3]
    if (
        t.shape != (4, 4)
        or not np.isfinite(t).all()
        or not np.allclose(r.T @ r, np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(r), 1.0)
    ):
        raise ValueError("不是有效刚性变换")
    # 最大对角分支避免接近 180° 时数值不稳定。
    if np.trace(r) > 0:
        s = math.sqrt(np.trace(r) + 1) * 2
        q = [
            (r[2, 1] - r[1, 2]) / s,
            (r[0, 2] - r[2, 0]) / s,
            (r[1, 0] - r[0, 1]) / s,
            s / 4,
        ]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1 + r[i, i] - r[j, j] - r[k, k]) * 2
        q = np.zeros(4)
        q[i], q[j], q[k], q[3] = (
            s / 4,
            (r[j, i] + r[i, j]) / s,
            (r[k, i] + r[i, k]) / s,
            (r[k, j] - r[j, k]) / s,
        )
    q = np.asarray(q) / np.linalg.norm(q)
    return t[:3, 3].tolist(), q.tolist()


def pose_matrix(pose):
    """输入 ROS Pose，输出齐次矩阵。"""
    p, q = pose.position, pose.orientation
    return matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def transform_matrix(transform):
    """输入 ROS Transform，输出齐次矩阵。"""
    p, q = transform.translation, transform.rotation
    return matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def to_pose(transform):
    """输入齐次矩阵，输出 ROS Pose。"""
    from geometry_msgs.msg import Pose

    p, q = components(transform)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = p
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def tool_goal(base_target, tool_to_probe):
    """输入期望探头位姿与 tool0→探头安装变换，输出期望 tool0 位姿。"""
    return base_target @ np.linalg.inv(tool_to_probe)


def rpy_degrees_rotation(rpy):
    """输入固定轴 Rx、Ry、Rz（度），输出 Rz(Rz) @ Ry(Ry) @ Rx(Rx) 旋转矩阵。"""
    angles = np.asarray(rpy, dtype=float)
    if angles.shape != (3,) or not np.isfinite(angles).all():
        raise ValueError("RPY 必须为三个有限角度，单位度")
    roll, pitch, yaw = np.deg2rad(angles) / 2.0
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    quaternion = [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]
    return matrix([0.0, 0.0, 0.0], quaternion)[:3, :3]
