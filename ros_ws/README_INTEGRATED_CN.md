# 本机整合入口

新的联合控制包为 [robot_task_control](src/robot_task_control/README.md)，部署步骤、参数、流程说明和无硬件测试见该文档。

- 机械臂：保留 eli_cs_robot_* 的驱动、MoveIt、导纳与外设控制代码。
- 底盘：保留 masc_chassis_* 的 CAN SDK、ROS 驱动和 PID，使用本机类型化任务服务与状态。
- 任务：新增 robot_task_control，以原 eli_to_chassis_control 的 YAML 和机械臂逻辑为基础。
- 两机控制通信：归档到 [legacy](legacy/README.md)，不参与构建；HK 照片发送单独恢复，沿用 MIMG 协议。
- 摄像头：HK 拍照与照片外传，USB 蓝线循迹，独立图像话题和速度选择器。
- 构建：scripts/build_raspberry_pi.sh 从源码编译机械臂 SDK 与必要 ROS 包；scripts/raspberry_pi_env.bash 设置运行环境。

旧 README 中笔记本路径、两机 TCP 启动方式和通信话题属于历史部署说明；整合后的启动以新包文档为准。

底盘更新已包含新版循迹、USB 自适应曝光与独立键盘入口 `ros2 launch robot_task_control keyboard_teleop.launch.py`，见 [更新记录](docs/chassis/INTEGRATION_20261014.md)。

## Jazzy 编译兼容

机械臂控制器按 `joint_trajectory_controller` 版本选择轨迹缓冲区和状态发布接口。校准包使用 C++17，不再引用未使用的 `ament_index_cpp` 头文件；驱动也已移除不再提供的 `hardware_interface/visibility_control.h` 引用。

MoveIt 用户包在可用时保留原 IPTP 时间参数化；没有 IPTP 的版本使用笛卡尔路径服务返回的时间信息，并在执行前检查时间戳非负且严格递增。速度和加速度比例仍由 MoveGroup 配置传入。

Jazzy 启动时，每个控制器 spawner 都通过 `--param-file` 接收替换过 `tf_prefix` 的配置，避免出现 `joints` 为空或 TCP 广播器 `frame_id` 为空。MoveIt OMPL 使用已安装的 `moveit_configs_utils` 默认管线配置，以匹配该版本的插件名称及参数类型。

真机驱动入口现使用系统提供的 `controller_manager/ros2_control_node` 加载 Elite 硬件插件，沿用当前 ROS 发行版的控制循环、时钟与超周期处理。旧的 `eli_ros2_control_node` 副本已删除，`controller_manager` 仅作为运行时依赖，不再编译、链接重复控制循环或引入其多余的构建依赖。

外控断开时，硬件接口会输出 `External control lost`，包括机器人运行/安全状态、三个外控 socket 的汇总连接状态、上次及本次连接期间最大命令写入间隔，以及配置的超时。发送失败或写入间隔达到超时一半时会限频输出 `External command write`。这些信息用于区分调度停顿、发送失败与机器人任务状态变化；外控接收超时仍沿用原配置（默认 200 ms）。

如果日志提示找不到力矩广播器或位置控制器插件，安装运行依赖后重启启动会话：

```bash
sudo apt install ros-jazzy-force-torque-sensor-broadcaster ros-jazzy-position-controllers
```

无硬件启动检查（先加载 ROS 与 `install/setup.bash`）：

```bash
python3 src/robot_task_control/test/test_arm_jazzy_startup.py
```

检查在独立 ROS 域内使用 mock 硬件，确认四个必要控制器激活、OMPL 初始化和 MoveIt IK 服务可用，不发送运动命令；不代替真机联调。

海康末端相机的 ROS 2 实时内参标定与 AprilTag 测距界面见 [camer_calibration](src/camer_calibration/README.md)，入口为 `ros2 launch camer_calibration hk_tag_view.launch.py`，也支持读取 MVS 本地照片；不需要启动机器人任务。USB 循迹包现名为 `ruier_camer_diver`。
