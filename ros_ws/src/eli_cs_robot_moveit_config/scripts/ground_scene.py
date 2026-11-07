#!/usr/bin/env python3
"""将base_link高度8cm及以下添加为碰撞地面，仅豁免固定安装底座。"""
from copy import deepcopy
import math
from time import monotonic
import signal
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from moveit_msgs.msg import CollisionObject, AllowedCollisionEntry
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Bool

OBJECT_ID = 'robot_base_ground'


def ground_object(frame, top, size=6., depth=2.):
    """返回6m方形地面实体，顶面为给定基座高度；覆盖机器人完整可达范围。"""
    obj = CollisionObject(id=OBJECT_ID)
    obj.header.frame_id = frame
    obj.operation = CollisionObject.ADD
    obj.pose.orientation.w = 1.
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size, size, depth])]
    pose = Pose()
    pose.orientation.w = 1.
    pose.position.z = top-depth/2.
    obj.primitive_poses = [pose]
    return obj


def allow_fixed_mount(matrix, fixed_links):
    """保留其他碰撞配置，只允许地面与固定底座相交，运动连杆无豁免。"""
    matrix = deepcopy(matrix)
    for name in [OBJECT_ID, *fixed_links]:
        if name not in matrix.entry_names:
            matrix.entry_names.append(name)
            for entry in matrix.entry_values:
                entry.enabled.append(False)
            matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
    index = matrix.entry_names.index(OBJECT_ID)
    for other, name in enumerate(matrix.entry_names):
        allowed = name in fixed_links
        matrix.entry_values[index].enabled[other] = allowed
        matrix.entry_values[other].enabled[index] = allowed
    return matrix


class GroundScene(Node):
    def __init__(self):
        super().__init__('robot_ground_scene')
        self.frame = self.declare_parameter('base_frame', 'base_link').value
        self.top = float(self.declare_parameter('ground_height', .08).value)
        if not math.isfinite(self.top) or not 0. <= self.top <= .5:
            raise ValueError('ground_height必须为0～0.5m的有限基座高度')
        self.fixed = list(self.declare_parameter('fixed_mount_links', ['base_link', 'base_link_inertia']).value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.ready = self.create_publisher(Bool, '/robot_control/ground_ready',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ready.publish(Bool(data=False))
        self.pending, self.stage = None, 'read'
        self.create_timer(.5, self.tick)

    def matches(self, obj):
        """MoveIt可能将对象归一到world；转换回基座后校验，避免反复重建地面。"""
        if (obj.id != OBJECT_ID or not obj.primitives or not obj.primitive_poses
                or list(obj.primitives[0].dimensions) != [6., 6., 2.]):
            return False
        origin = TransformStamped()
        origin.transform.translation.x = obj.pose.position.x
        origin.transform.translation.y = obj.pose.position.y
        origin.transform.translation.z = obj.pose.position.z
        origin.transform.rotation = obj.pose.orientation
        pose = do_transform_pose(obj.primitive_poses[0], origin)
        if obj.header.frame_id != self.frame:
            try:
                pose = do_transform_pose(pose, self.buffer.lookup_transform(self.frame, obj.header.frame_id, Time()))
            except TransformException:
                return False
        q = pose.orientation
        vertical = abs(1.-2.*(q.x*q.x+q.y*q.y))
        return vertical > 1.-1e-8 and abs(pose.position.z+1.-self.top) < 1e-8

    def tick(self):
        if not self.get_scene.service_is_ready() or not self.apply.service_is_ready():
            self.ready.publish(Bool(data=False))
            return
        if self.pending is None:
            req = GetPlanningScene.Request()
            req.components.components = req.components.WORLD_OBJECT_GEOMETRY | req.components.ALLOWED_COLLISION_MATRIX
            self.pending, self.stage = self.get_scene.call_async(req), 'read'
            self.since = monotonic()
            return
        if not self.pending.done():
            if monotonic()-self.since > 3.:
                self.ready.publish(Bool(data=False))
                self.pending = None
            return
        pending, self.pending = self.pending, None
        try:
            result = pending.result()
        except Exception as error:
            self.ready.publish(Bool(data=False))
            self.get_logger().warning('地面场景服务暂不可用，将重试：'+str(error))
            return
        if self.stage == 'apply':
            self.ready.publish(Bool(data=bool(result.success)))
            if result.success:
                self.get_logger().info('规划地面已生效：'+self.frame+' Z≤'+str(self.top)+'m；仅固定底座允许相交')
            return
        scene = result.scene
        matrix = scene.allowed_collision_matrix
        if OBJECT_ID in matrix.entry_names:
            index = matrix.entry_names.index(OBJECT_ID)
            allowed = matrix.entry_values[index].enabled
            exclusions_ok = all(enabled == (name in self.fixed)
                                for name, enabled in zip(matrix.entry_names, allowed))
            if exclusions_ok and any(self.matches(obj) for obj in scene.world.collision_objects):
                self.ready.publish(Bool(data=True))
                return
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.world.collision_objects = [ground_object(self.frame, self.top)]
        req.scene.allowed_collision_matrix = allow_fixed_mount(scene.allowed_collision_matrix, self.fixed)
        self.pending, self.stage = self.apply.call_async(req), 'apply'
        self.since = monotonic()


def main():
    rclpy.init()
    node = GroundScene()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
