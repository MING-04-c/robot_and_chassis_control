"""两组关节姿态往返入口；实现与联合任务共用，避免两套控制逻辑。"""
from robot_task_control.joint_posture_control import JointPostureControl as JointPostureTest, main
