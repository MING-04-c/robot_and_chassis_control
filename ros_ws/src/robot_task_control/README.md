# 任务控制包

本包负责组织任务顺序、速度选择及按需采集。机械臂、底盘、相机、力传感器驱动仍由各自功能包提供；工控机 TCP 通讯唯一实现在 `eli_cs_robot_serial/device/industrial_pc/`。

所有操作命令集中在 [机械臂与底盘 ROS 控制命令](../../机械臂与底盘ROS控制命令.md)，本文件只解释目录与入口，避免两份说明不断追加后互相矛盾。

## 常用入口

| launch 文件 | 用途 | 开始方式 |
|---|---|---|
| arm_admittance_only | 仅机械臂导纳 | 就绪后自动 |
| line_follow_only | 仅底盘循迹 | 就绪后自动；auto_start:=false 仅等待 |
| keyboard_teleop | 底盘键盘控制 | 键盘 |
| line_follow_arm_test | 整体联动：循迹、四点机械臂、末尾拍照发送 | 默认自动；remote_control:=true 等主机开始 |
| camera_capture_remote | 只采图发送，不加载运动驱动 | 麒麟 start_capture.py |
| camera_detection_real | 相机点探针检测，默认TCP、可选本地YAML | TCP收到匹配结果后自动；local就绪后自动 |

表中六个常用入口均以 `.launch.py` 结尾。整体联动统一使用 `line_follow_arm_test.launch.py`：默认就绪后自动执行；`remote_control:=true` 加载TCP并等待麒麟 `start_task.py`。可显式传 `auto_start:=false` 暂停自动开始。

旧 `line_follow_arm_remote.launch.py` 已合并删除；旧两秒导纳＋循迹入口及其运行配置已移除。

## 目录

```text
launch/                 常用任务入口
  internal/             robot_system、detection_system：共用驱动与控制栈
  tests/                像素点仿真入口
config/                 常用任务流程及本地检测点
  capture/              最终拍照、独立采集及旧 HK 采集参数
  tests/                到达测试、像素仿真配置和样本
robot_task_control/
  workflow_controller.py    YAML 工作流执行器
  velocity_mux.py           底盘 PID/循迹速度选择
  camera_detection_task.py  两模式真机限力测试
  final_capture.py          顺序开启相机、稳定采图、关闭后发送
  testing/                  仿真与旧到达测试节点
rviz/                   检测显示配置
test/                   本包回归测试，非任务启动入口
```

`internal/robot_system.launch.py` 供单步和整体联动复用；`internal/detection_system.launch.py` 供探针检测复用。两者不是日常任务入口。旧综合示例及示例底盘任务表已删除，启动核心默认使用当前整体联动配置。`config/tests/` 只供仿真及内部调试使用。

海康转发壳 `robot_task_control/hk_image_transfer.py` 已移除：启动核心直接调用串口通讯包的 `industrial_pc_hk_transfer.py`。旧 `ros2 run robot_task_control hk_image_transfer` 名称仅通过 entry point 直接指向唯一实现，未保留第二份代码。原来引用不存在接收工具的 HK 测试已迁到通讯包目录并修正引用。

## 各流程的职责

- 工作流控制器：驱动就绪检查、任务顺序、失败/取消处理；自定义 `chassis_execute` 流程需自行提供 `task_table`，实际位移参数仍在底盘驱动包。
- 速度选择器：只转发被选中的 PID 或循迹速度，切换/输入过期时发零速。
- 相机点检测：TCP模式稳定采图并绑定本次结果，local模式直接使用YAML；两者冻结相机/探针位姿，将毫米点变成基座目标；detect_control 负责 tool0 补偿、碰撞规划、限力接近、保持和逐点回原位。
- 最终采图：复用 HK/深度驱动及串口包的传输实现，不启动常驻预览和点云。
- 主机通讯：start/stop/resume/capture、状态、检测点观测和最新厚度缓存；不把普通点位回包直接变成运动指令。

`camera_detection_real` 默认接收本次照片的主机点位并自动触碰；`target_source:=local` 保留原本地给点测试。它与整体联动任务二选一，不能同时占用机器人、相机或9010。

## 测试与进一步说明

没有重复删除不同职责的测试：`test_camera_point_tasks` 检查坐标/本地流程/退出，`test_arm_description_launch` 检查模型与启动组合，`test_arm_jazzy_startup` 检查假硬件加载，`test_device_startup` 检查设备就绪门控，`test_local_control` 检查工作流和速度选择，`test_final_capture` 检查采集稳定性和进程边界。涉及 ROS 联调的测试按文件单独运行，不连接真实运动硬件。

- [相机点两模式真机测试](../../docs/tasks/CAMERA_DETECTION_REAL.md)
- [四点联动与拍照流程](../../docs/tasks/LINE_FOLLOW_ARM_TEST.md)
- [像素点触碰仿真](../../docs/tasks/PIXEL_POINT_SIMULATION.md)
- [主机通讯实现](../eli_cs_robot_serial/device/industrial_pc/README.md)

首次目录调整后需清除旧安装路径中的同名 launch，避免 ROS2 递归查找出现重复；本次工作区已完成迁移。其他机器更新代码时先备份自行修改过的安装配置，再只清理 `build/robot_task_control`、`install/robot_task_control` 并重新构建该包。本轮另已清理被删除入口及旧示例的安装副本。旧 remote 命令改用 `line_follow_arm_test.launch.py remote_control:=true`；旧 detection_test 是内部检测栈，不再作为常用入口。
