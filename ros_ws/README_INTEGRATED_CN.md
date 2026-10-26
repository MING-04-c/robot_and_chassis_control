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


## 新末端模型同步

当前六轴默认模型入口仍为 `eli_cs_robot_description/urdf/cs.urdf.xacro`，已包含新增的三件末端装配。真机驱动、MoveIt、导纳、联合任务及六轴模型预览/仿真继续共用该入口，无需修改原来的启动命令。

- 末端几何和固定安装关系：`eli_cs_robot_description/urdf/cs_macro_terminal.xacro`，仅定义末端，不复制机械臂运动链。
- STL：`eli_cs_robot_description/meshes/terminal/`，由本次 `robot/meshes/` 同步而来；后续正式部署维护这一份包内资源即可。构建、安装与运行不再依赖包外 `../../../robot` 路径。
- 默认装配到 `tool0`，平移为零，绕 Z 轴旋转 -90°，沿用用户模型；两条内部固定关节的位置和方向也保持原值。
- SRDF 仅排除同一刚性末端内部及其腕部安装面的碰撞，其余机械臂与新末端的碰撞检查保持启用。
- 当前 TCP 继续使用 `tool0`。添加可视/碰撞几何不会自动改变示教器的 TCP、负载或质心；新工具实际接触点和负载参数需按实物确定，不能由 STL 外形推断。

原 `robot/cs_with_terminal.urdf.xacro` 预览入口已转到同一份描述，避免预览和真机各维护一套模型。`robot/robot.urdf` 和原 CAD 网格保留作为原始资料。五轴机器人使用独立模型，未套用这套六轴装配。

旧海康标定包 `src/camer_calibration/` 已清空，原标定包的生成物也已移除。新的深度相机入口为 [geminipro_camer_calibration](src/geminipro_camer_calibration/README.md)，驱动在 `src/gemini_pro_ros2/`。在 `ros_ws` 构建并加载环境后，运行 `ros2 launch geminipro_camer_calibration gemini_pro_view.launch.py`，打开 RGB、对齐深度、实时帧率和拍照/棋盘格标定窗口。当前参数来自深度相机自身，不沿用海康内参；相机到机器人末端的外参尚未标定。

深度相机按需拍照已增加主机 V5.4 格式适配：从同一 SDK FrameSet 保存原始深度、现场比例和同会话参数，文件名及校验结构兼容主机；树莓派仍运行 ROS 2 和随驱动提供的 ARM64 C++ SDK。详见相机包说明。

## 相机外参与测厚接触流程

`terminal_joint_1` 是相机安装坐标，`terminal_joint_2` 是测厚探头检测点；机器人控制点仍为 `tool0`。
`geminipro_camer_calibration` 根据观测时刻的模型 TF 转换单个或整组检测点，输出基座目标和补偿探头偏移后的 tool0 目标。
`detect_control` 保存整组基座检测点并只记录一次原位；按顺序逐点规划接近、20 N 停止、保持 5 秒、撤离，每点测完均返回同一原位，再开始下一点。
完整配置和接口见 `src/detect_control/README.md`；默认只启动任务接口，收到 start 服务才运行。
默认以相机 joint（terminal_link_1）为检测坐标原点，通过模型 TF 换算触碰位置，不要求先做手眼标定；检测坐标必须与该 joint 的原点和轴向一致。光学坐标与 joint 不一致时需额外提供二者变换。
