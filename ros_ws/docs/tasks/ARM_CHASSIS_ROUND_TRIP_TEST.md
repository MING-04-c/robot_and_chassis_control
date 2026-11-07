# 机械臂与小车整体往返实车测试

入口复用 `robot_task_control/line_follow_arm_test.launch.py`，选择 `test_profile:=return_charge`。
本轮验证机械臂姿态切换、视觉循迹、实际路线录制与返程，以及TCP通信。
不执行四个笛卡尔点、拍照、测厚或20N触碰动作；不加载导纳、力传感器、HK相机、深度相机和RViz。

9010 TCP桥开启，额外正常/异常状态汇总关闭（不订阅 `/rosout`，不启动汇总定时器）。
机械臂实际到位、设备就绪、保护停止和底盘运动保护继续生效。
基础任务状态仍可经TCP查询；关闭汇总不代表设备健康。

## 动作顺序与通过条件

| 阶段 | 实际动作 | 通过条件 |
|---|---|---|
| 1 | 加载机械臂驱动、MoveIt、底盘、USB循迹和TCP | 设备就绪，底盘连续停稳1秒 |
| 2 | 机械臂运动到行车位 | 本次关节任务实际到位，才放行底盘 |
| 3 | 摄像头停车观察、小步对齐出发横线 | `start_alignment_complete`，随后才建立记录首点 |
| 4 | 小车前进循迹并记录实际里程计路线 | 忽略出发横线，在第一条有效停止横线到位停车 |
| 5 | 确认底盘停稳并封存路线 | 保存完成；机械臂动作期间不重置里程计、不覆盖已封存路线 |
| 6 | 机械臂运动到过渡位 | 本次关节任务实际到位 |
| 7 | 等待5秒 | 从过渡位实际到位后开始计时，保持底盘停车 |
| 8 | 机械臂运动到行车位 | 本次实际到位后，才允许返程 |
| 9 | 小车沿本次记录路线倒车返回 | 通过原弯道；末段视觉摆正、横移回中、找横线并接近充电位 |
| 10 | 充电确认、底盘停稳，结束任务 | 连续新反馈确认充电；工作流完成，机械臂保持行车位，不自动循环 |

默认全程自动执行，不需要在到站、等待结束或返程前另发开始指令。
本测试复用充电返程：最终以充电反馈确认完成，单独看见横线或里程计回到起点不算整体通过。

两组姿态共用 `eli_cs_robot_moveit_config/config/arm_joint_postures.yaml`，角度单位为度：

| 名称 | 基座、肩部、肘部、手腕1、手腕2、手腕3 |
|---|---|
| 行车位 `travel` | -45, 15, -71, 4.4, 148, 75 |
| 过渡位 `detection_ready` | -45, 0, 0, 0, 90, 0 |

姿态通过MoveIt规划和碰撞检查；规划失败、关节反馈失效或未实际到位时停止流程。
当前场景包含地面，现场桌面及其他障碍仍需与实际碰撞场景一致。

## 启动前

1. 结束旧的机械臂/底盘/联合任务，避免重复启动驱动或占用9010端口。
2. 将小车放回起步横线附近，保证相连主线与横线都在画面内。不能从上次倒车卡住的位置直接重启整套往返。
3. 确认机械臂外控条件、底盘通信和现场运动空间。需要时手动配置CAN：

```bash
cd ~/project/robot_and_chassis_control/ros_ws
sudo bash scripts/setup_can.sh auto
```

源码修改后构建一次；本次修改已完成构建时可跳过构建命令：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select masc_chassis_user_control eli_cs_robot_serial robot_task_control --symlink-install
source install/setup.bash
```

## 本轮推荐启动命令

**启动后设备就绪即自动动作。** TCP开启不要求工控机在线；没有连接也会自动执行。
循迹设定速度0.03m/s；起步对齐、离桩、弯道和末段可能更慢。

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch robot_task_control line_follow_arm_test.launch.py \
  test_profile:=return_charge \
  start_tcp:=true enable_status_monitor:=false remote_control:=false \
  launch_admittance:=false launch_force_sensor:=false launch_rviz:=false \
  line_show_window:=true line_follow_speed:=0.03 \
  line_stop_marker_target_ratio:=0.5 \
  reverse_route_max_curvature:=5.0 \
  charging_status_layout:=byte_flags \
  route_file:=~/.ros/line_follow/arm_chassis_round_trip_test.json \
  2>&1 | tee /tmp/arm_chassis_round_trip_test.log
```

`byte_flags` 与当前循迹实车配置一致。只有已核实固件使用整个uint32状态码bit3充电时，
才改用 `charging_status_layout:=bitfield32`；底盘控制器和TCP桥会使用同一个参数。

测试文件与默认 `last_route.json` 分开。再次使用同名测试文件会覆盖该测试文件，
任务封存的快照另存于 `~/.ros/line_follow/snapshots/`；多轮对比可分别使用
`arm_chassis_round_trip_test_01.json`、`_02.json`、`_03.json`。

## 现场怎么观察

- 机械臂未确认到行车位前，小车应始终停车。
- 起步出现 `waiting_for_stop`、`settling` 或单步完成时，尚未完成整体对齐；以 `start_alignment_complete` 为准。
- 去程重点看直线是否持续偏到一侧、出弯是否反复左右摆动。记录的是实际行驶轨迹，不是胶带中心线；明显偏线不能算录制合格。
- 到站先确认小车停稳，再观察机械臂到过渡位。完整5秒应从实际到位后计算，不能从发出运动请求时算起。
- 等待结束后先看机械臂回行车位，再看小车倒车。相机在机械臂动作/等待期间按需暂停，恢复返程前须等有效图像；不会重置路线。
- 倒车重点观察上次剩余约0.85m时的弯道。途中曲率上限现在独立为5/m，末段仍保持原2/m限制；角速度、15cm偏离和目标方向保护继续有效。
- 最终检查充电确认及整个工作流完成，随后至少观察5秒，确认无再次运动。

需要看基础运行状态时，在另一个已source的终端选择一个话题观察即可：

```bash
ros2 topic echo /task_controller/status
ros2 topic echo /arm_joint_postures/status
ros2 topic echo /line_follow/status
```

## TCP通信检查

树莓派默认监听 `0.0.0.0:9010`；网络示例为工控机 `169.254.199.120`、树莓派 `169.254.199.100`。
可先在树莓派检查监听：

```bash
ss -ltnp 'sport = :9010'
```

工控机使用已有工具目录的 `start_task.py` 查询基础任务状态：

```bash
python3 start_task.py --host 169.254.199.100 --action status
```

应收到成功回包，其中 `robot_status.status_monitor_enabled` 为false，
`normal_status` / `abnormal_status` 为null；基础 `task`、`line_follow` 和底盘电量数据仍按实际反馈提供。
此时不要运行额外的 `monitor_status.py`；专门的normal/abnormal查询会明确返回“未启用”，而非伪造正常状态。

本轮自动开始，工控机只查询即可。若要先联机检查、再由工控机开始，将启动命令中的
`remote_control:=false` 改成true；随后只发送一次：

```bash
python3 start_task.py --host 169.254.199.100 --action start
```

该参数只改变首次开始方式；到站后的过渡、等待、回行车位及返程仍自动执行。

## 中止与复测记录

普通任务停止可调用：

```bash
ros2 service call /task_controller/stop std_srvs/srv/Trigger '{}'
```

也可在工控机执行 `python3 start_task.py --host 169.254.199.100 --action stop`。
需要立即停止实体设备时使用设备停止/急停。异常后先保留日志、查明原因，再回到实物起点重测；不要在途中反复start。

每轮记录：路线文件名、去程是否居中/摆动、到站位置、过渡位到位时间、5秒等待是否满足、
回行车位确认、原故障弯道是否通过、充电确认及失败状态。保留
`/tmp/arm_chassis_round_trip_test.log` 和对应路线/快照；下一轮换日志名和路线名，便于对比。

流程来源：`robot_task_control/config/line_follow_arm_charge_test.yaml`。
`arm_postures_before_chassis` 自动在两次底盘行驶前插入行车位并等待实际到位。
软件回归检查不代表本整体流程已通过真机验收，最终以以上实车观察结果为准。
