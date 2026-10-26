#!/usr/bin/env python3
"""以相机 joint 为原点，把检测点转换到探头触碰所需的 tool0 位姿。"""
from pathlib import Path
from time import monotonic
import yaml
import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PointStamped, PoseArray
from tf2_ros import Buffer, TransformListener, TransformException
from geminipro_camer_calibration.geometry import (
    matrix,
    pose_matrix,
    transform_matrix,
    to_pose,
    tool_goal,
)


class TargetTransform(Node):
    def __init__(self):
        super().__init__("camera_target_transform")

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.base = param("base_frame", "base_link")
        self.tool = param("tool_frame", "tool0")
        self.probe = param("probe_frame", "terminal_link_2")
        self.camera = param("observation_frame", "terminal_link_1")
        self.point_orientation = param("point_orientation_mode", "current_probe")
        self.point_quaternion = param("point_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        if self.point_orientation not in ("current_probe", "fixed_base"):
            raise ValueError(
                "point_orientation_mode 必须是 current_probe 或 fixed_base"
            )
        matrix([0.0, 0.0, 0.0], self.point_quaternion)
        self.max_age = float(param("max_observation_age", 5.0))
        path = param("extrinsic_file", "")
        self.extrinsic = None
        if path:
            data = yaml.safe_load(Path(path).expanduser().read_text())
            if (
                data["parent_frame"] != self.tool
                or data["child_frame"] != self.camera
                or data.get("units") != "m"
            ):
                raise ValueError("外参必须是 tool0→观测坐标系，且单位为 m")
            self.extrinsic = matrix(data["translation"], data["quaternion_xyzw"])
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError("max_observation_age 必须为正")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.base_pub = self.create_publisher(
            PoseStamped, "/geminipro_calibration/target_base", 10
        )
        self.tool_pub = self.create_publisher(
            PoseStamped, "/geminipro_calibration/target_tool0", 10
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, "/geminipro_calibration/tool0_goal", 10
        )
        self.create_subscription(
            PoseStamped, "/geminipro_calibration/camera_target", self.receive, 10
        )
        self.create_subscription(
            PointStamped, "/geminipro_calibration/camera_point", self.receive, 10
        )
        self.batch_pubs = {
            "base": self.create_publisher(
                PoseArray, "/geminipro_calibration/targets_base", 10
            ),
            "tool": self.create_publisher(
                PoseArray, "/geminipro_calibration/targets_tool0", 10
            ),
            "goal": self.create_publisher(
                PoseArray, "/geminipro_calibration/tool0_goals", 10
            ),
        }
        self.create_subscription(
            PoseArray,
            "/geminipro_calibration/camera_points",
            lambda m: self.receive(m, points_only=True),
            10,
        )
        self.create_subscription(
            PoseArray, "/geminipro_calibration/camera_targets", self.receive, 10
        )
        self.pending = None
        self.create_timer(0.05, self.process_pending)
        self.get_logger().info(
            f"等待相对于 {self.camera} 的 camera_point / camera_target，使用观测时刻 TF 转换"
        )

    def publish(self, publisher, frame, stamp, value):
        message = PoseStamped()
        message.header.frame_id, message.header.stamp = frame, stamp
        message.pose = to_pose(value)
        publisher.publish(message)

    def receive(self, message, points_only=False):
        self.pending_points_only = points_only or isinstance(message, PointStamped)
        self.pending = message
        self.pending_since = monotonic()
        self.process_pending()

    def process_pending(self):
        message = self.pending
        if message is None:
            return
        try:
            if message.header.frame_id != self.camera:
                raise ValueError(f"输入坐标系必须为 {self.camera}")
            stamp = Time.from_msg(message.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if stamp.nanoseconds == 0 or age < -0.1 or age > self.max_age:
                raise ValueError("缺少观测时间戳，或观测已过期")
            base_tool = transform_matrix(
                self.buffer.lookup_transform(self.base, self.tool, stamp).transform
            )
            if self.extrinsic is not None:
                tool_camera = self.extrinsic
            else:
                tool_camera = transform_matrix(
                    self.buffer.lookup_transform(
                        self.tool, self.camera, stamp
                    ).transform
                )
            tool_probe = transform_matrix(
                self.buffer.lookup_transform(self.tool, self.probe, Time()).transform
            )
            is_batch = isinstance(message, PoseArray)
            items = (
                message.poses
                if is_batch
                else [
                    message.point if isinstance(message, PointStamped) else message.pose
                ]
            )
            if not 1 <= len(items) <= 100:
                raise ValueError("检测点组为空或超过 100 点")
            base_camera = base_tool @ tool_camera
            base_rotation = (base_tool @ tool_probe)[:3, :3]
            if self.point_orientation == "fixed_base":
                base_rotation = matrix([0.0, 0.0, 0.0], self.point_quaternion)[:3, :3]
            results = []
            for item in items:
                if self.pending_points_only:
                    point = item.position if is_batch else item
                    camera_target = matrix(
                        [point.x, point.y, point.z], [0.0, 0.0, 0.0, 1.0]
                    )
                    camera_target[:3, :3] = base_camera[:3, :3].T @ base_rotation
                else:
                    camera_target = pose_matrix(item)
                base_target = base_camera @ camera_target
                results.append(
                    (
                        base_target,
                        tool_camera @ camera_target,
                        tool_goal(base_target, tool_probe),
                    )
                )
            # 整组全部成功后再发布，不能把半组检测点送进任务队列。
            if is_batch:
                for index, (name, frame) in enumerate(
                    (("base", self.base), ("tool", self.tool), ("goal", self.base))
                ):
                    output = PoseArray()
                    output.header.frame_id = frame
                    output.header.stamp = message.header.stamp
                    output.poses = [to_pose(r[index]) for r in results]
                    self.batch_pubs[name].publish(output)
            else:
                base_target, tool_target, goal = results[0]
                self.publish(
                    self.base_pub, self.base, message.header.stamp, base_target
                )
                self.publish(
                    self.tool_pub, self.tool, message.header.stamp, tool_target
                )
                self.publish(self.goal_pub, self.base, message.header.stamp, goal)
            self.pending = None
        except TransformException as error:
            # 相机与 TF 回调存在到达先后差异，短暂等待对应时刻 TF。
            if monotonic() - self.pending_since > 1.0:
                self.pending = None
                self.get_logger().warning(
                    f"目标 TF 等待超时：{error}", throttle_duration_sec=2
                )
        except Exception as error:
            self.pending = None
            self.get_logger().warning(f"目标未发布：{error}", throttle_duration_sec=2)


def main():
    rclpy.init()
    node = TargetTransform()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
