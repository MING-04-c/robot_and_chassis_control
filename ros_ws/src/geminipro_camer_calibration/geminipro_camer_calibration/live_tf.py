"""独立处理实时机器人 TF，避免任务/力回调积压旧位姿。"""

from threading import Thread
from uuid import uuid4
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException, ShutdownException
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
            while not self.closed and self.node.context.ok():
                self.executor.spin_once(timeout_sec=0.1)
        except (ExternalShutdownException, ShutdownException):
            pass

    def close(self):
        if self.closed:
            return
        self.closed = True
        # 先唤醒并等待 spin 退出，再销毁执行器，避免两个线程同时清理等待集。
        self.executor.wake()
        self.thread.join(timeout=1.0)
        if self.thread.is_alive():
            self.node.get_logger().warning('TF线程未在1秒内退出，交由ROS context收尾')
            return
        self.executor.shutdown(timeout_sec=0.5)
        # 在 context 与节点仍有效时解除订阅并释放监听器。
        self.listener.unregister()
        self.listener = None
        self.node.destroy_node()
