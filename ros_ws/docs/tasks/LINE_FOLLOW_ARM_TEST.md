# 底盘循迹与机械臂四点联动测试

入口：`robot_task_control/launch/line_follow_arm_test.launch.py`。流程定义：`robot_task_control/config/line_follow_arm_test.yaml`。

## 动作顺序

1. 启动机械臂驱动、MoveIt、力传感器、底盘驱动、USB 循迹相机、识别、循迹控制及速度选择器。按现有就绪流程检查设备、外控和控制器，驱动连续稳定等待默认 5 秒。
2. 设备就绪后自动开始任务，无需调用 start 服务。读取 `/wheel_odom`，连续 1 秒满足线速度 ≤0.005 m/s、角速度 ≤0.02 rad/s，才进入机械臂初始准备。反馈必须新鲜，旧的零速消息不能放行。
3. 比较 tool0 实际位置和方向与初始点 P1；位置误差 ≤5 mm **且**方向误差 ≤1°时跳过重复移动，否则 MoveIt 规划并运动到 P1。只位置相同、角度不同，仍需运动到目标方向。初始点规划失败就停止流程，不开始循迹。
4. 底盘沿识别轨迹行走。90°横线两侧均伸出主线才作为停止标记，接近时降速，横线交点到单帧相机图像高度50%时停车（默认容差图高1%，480高约±5像素）。前方仍有主线为途中站，保持停车等待新的 `/line_follow/resume` 消息，收到后才继续；不在途中站执行后续机械臂点位。前方无主线、视野和连续确认足够的 T 形标记为最终站，居中停车后才进入停稳确认。无法区分时停住，不猜成最终站。当前 YAML 要求 `require_final_marker: true`：普通丢线5秒未恢复会中止，不能再当成正常走完；行走超时默认90秒，途中站等消息的时间不计入，但通信或图像失效仍会中止。
5. 再次等待底盘连续停稳 1 秒。10 秒内不能确认停稳，就中止，不开始后面的机械臂运动。
6. MoveIt 依次规划、执行 P2、P3、P4。每个点完成后等待 1 秒。每步必须成功才继续，不循环、不自动返回初始点。

| 点 | tool0 XYZ（米） | 固定轴 RPY（度） | 用途 |
|---|---|---|---|
| P1 | -0.05, 0.2, 0.475 | 0, 0, 90 | 初始点，循迹前到位 |
| P2 | -0.13, 0.35, 0.8 | 0, 0, 90 | 循迹停稳后的第二点 |
| P3 | 0.3, 0.165, 0.8 | 0, 0, 0 | 第四点，流程终点 |

坐标系统一为机械臂 `base_link`，控制点为 `tool0`，旋转约定为 `Rz(yaw) Ry(pitch) Rx(roll)`。这些不是世界坐标：底盘移动后，点位随机械臂基座一起移动。这里使用 MoveIt 点到点规划，不要求两点间为直线，不执行触碰或 20 N 接触检测；规划使用当前更新后的机器人碰撞模型，现场桌面等环境物体仍需加入规划场景。

## 启动与测试

在工作区根目录的 `ros_ws` 中执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_task_control
source install/setup.bash
sudo bash scripts/setup_can.sh auto
ros2 launch robot_task_control line_follow_arm_test.launch.py \
  robot_ip:=169.254.199.150 sensor_host:=169.254.199.80 launch_rviz:=true
```

本入口包含驱动，不能与已有同一套驱动同时启动。CAN 配置脚本需要在 launch 前运行；launch 不自动执行该脚本。该入口不启动海康拍照及照片传输；沿用现有机械臂控制框架，力传感器仍会启动并报告自身状态，但本流程全为 MoveIt 位置规划，不等待零偏和力数据；只有包含导纳动作的工作流才检查力反馈。默认在 launch 后等待设备就绪，然后自动执行四点流程。

启动前确认轨迹沿途及机械臂运动空间可用。launch 后会检查真机外控、控制器、MoveIt 接口、TCP 坐标变换和循迹设备，就绪后自动开始，无需打开另一个终端调用服务。

观察初始点到位、底盘前进、底盘停稳、机械臂第二第四点运动的顺序。状态和反馈：

```bash
ros2 node info /task_controller
ros2 topic echo /line_follow/status
ros2 topic echo /wheel_odom --once
```

终止工作流可调用：

```bash
ros2 service call /task_controller/stop std_srvs/srv/Trigger '{}'
```

此服务停止工作流和底盘，不保证取消已提交的 MoveIt 轨迹；需要立即停止机械臂时使用机器人本身的停止/急停。

## 修改轨迹

编辑 YAML 中的 `position`、`orientation`；修改初始点就是修改第一个 `arm_move`。第一点的 `skip_if_at_target` 控制到位跳过，`arrival_tolerance` 单位米，`arrival_orientation_tolerance_deg` 单位度。`max_position_distance` 是相对当前 TCP 的位移保护，初始点默认 1 m，后三步 0.6 m，不是工作空间可达性保证。

底盘结束方式由 `line_follow.until_untrusted` 控制，末端/丢线等待由 `boundary_wait_sec` 控制，总超时由 `step_timeout_sec` 控制；速度和缺口通过规则仍由原来的循迹配置控制。修改包内 YAML 后重新编译并重新启动。也可复制到外部文件后传入，避免每次修改重建：

```bash
ros2 launch robot_task_control line_follow_arm_test.launch.py \
  workflow_file:=/绝对路径/my_test.yaml
```

软件验证与真机验证分开记录：模拟消息测试只验证工作流顺序、停稳门槛、点位和方向转换；IK/规划检查也不能代替从真实关节初始状态、真实障碍物环境出发的试运行。规划失败时查看 MoveIt 原因，不能跳过碰撞检查来强行执行。

## 本次软件验证记录

工作流模拟测试通过三种初始条件：不在 P1、已经在 P1、位置相同但角度不同；验证了初始准备先于循迹、旧零速数据和仍在移动的反馈不放行、后续点位及角度按顺序发送。未启动真机运动。

原 P1～P3 三个目标分别通过碰撞感知 IK 检查；P1→P2 的完整路径规划在模拟检查中返回 FAILURE（99999），使用关节目标 5 秒和位姿目标 10 秒的检查均未成功。P2→P3 因前段未通过而尚未验证。分别存在逆解不等于整段路径可行，以上点位保留用户输入，未绕过碰撞检查。真机能否完成本轮任务仍需核对关节初始状态与规划场景；若同样失败，需要查看具体路径限制，再调整点位或增加中间点。

新增 P4 的 XYZ 为 [0.3, 0.2, 0.6] m，RPY 为 [-160, 0, -77] 度，追加于 P3 后执行；该点及 P3→P4 路径尚未进行可达性或真机验证，运行时仍须由 MoveIt 规划成功才执行。

## 途中站继续消息

初次 launch 就绪后仍自动开始。途中站到位后，发布一次新的开始消息才能继续，提前发布的消息不保留，最终站不接受此消息继续：

```bash
ros2 topic pub --once /line_follow/resume std_msgs/msg/Empty '{}'
```

恢复后对刚经过的横线去重，不会原地重复停车；后续横线仍需停车。查看 `/line_follow/status` 的 `stop_marker` 字段、`state=stop_marker_intermediate/final`，预览显示 STOP TARGET 和标记类别。中间位置以左上角单帧相机图像为准，不是四宫格预览整体高度的中间。参数位于 `masc_chassis_user_control/config/line_follow.yaml` 的 `stop_marker_*`。

该目标是图像位置闭环，实际停车存在图像帧率和底盘刹车延迟，不能承诺恰好零像素。先看 `error_ratio` 再调整容差或目标比例；过冲会停车报告，不自动倒车。最终横线后应有足够可见地面用于判断“没有继续主线”；画面边缘看不全时先显示 unknown。
