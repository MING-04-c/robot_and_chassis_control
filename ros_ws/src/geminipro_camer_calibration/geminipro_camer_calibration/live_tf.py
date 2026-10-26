"""独立处理实时机器人 TF，避免任务/力回调积压旧位姿。"""

from threading import Thread
from uuid import uuid4
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener


class LiveRobotTransforms:
    def __init__(self):
        # 独立节点不继承 launch 的同名节点重映射，也不发布重复 rosout。
        self.node = Node(
            "robot_tf_" + uuid4().hex[:12],
            use_global_arguments=False,
            enable_rosout=False,
            start_parameter_services=False,
        )
        self.buffer = Buffer()
        self.listener = TransformListener(
            self.buffer,
            self.node,
            qos=QoSProfile(depth=5, durability=DurabilityPolicy.VOLATILE),
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.closed = False
        self.thread = Thread(target=self.spin, daemon=True)
        self.thread.start()

    def spin(self):
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        # 在 context 与节点仍有效时解除订阅并释放监听器。
        self.listener.unregister()
        self.listener = None
        self.node.destroy_node()
