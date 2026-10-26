# Eli CS 末端位姿与柔顺控制

本包通过 MoveIt 将末端位姿转换为关节轨迹，并在到达目标后根据六维力传感器的数据进行平移柔顺控制。

真机分开启动时，先在一个终端保持 `eli_cs_robot_driver elite_control.launch.py` 运行，再在另一个已加载 ROS 与工作区环境的终端执行：

```bash
ros2 launch eli_cs_robot_usr compliant_control_real.launch.py launch_rviz:=false sensor_host:=169.254.199.80
```

此入口默认自动等待外控程序运行、四个机械臂控制器激活以及零偏采集后的新鲜力数据，然后捕获当前 TCP 位姿开启导纳，无需手动调用服务。零偏采集时工具应无外部接触。默认不启动目标输入、轨迹执行器或阶段协调器；需要这些功能时显式启用对应参数，并设置 `auto_start_admittance:=false`。统一任务入口会关闭这个独立自动开启节点，由工作流管理导纳。

独立自动开启时，就绪条件需连续保持 `arm_control_start_delay` 秒（默认 10 秒），中途断线或数据失效会重新计时。需要延长时在启动命令后加 `arm_control_start_delay:=15.0`。联合入口则先等机械臂连接稳定，再加载控制部分，避免同时启动所有节点。

真机入口禁止 MoveIt 再次发布 `/robot_description`，硬件描述只由驱动的 robot_state_publisher 发布，避免规划模型中的默认连接参数覆盖真机参数。

真机柔顺参数位于：

```text
config/compliant_real.yaml
```

## 当前计算模型

每个柔顺轴使用准静态关系：

```text
effective_force = sign(F) * max(abs(F) - force_deadband, 0)
equilibrium_offset = effective_force / stiffness
```

最终命令位置为：

```text
command_xyz = raw_target_xyz + compliance_offset
```

真机试验配置 `compliance_force_sign: [-1.0, -1.0, 1.0]`，在力转换到 `base_link` 并去零、滤波后，反转 X、Y 的导纳输入，Z 保持原符号。该参数每项只能是 `-1.0` 或 `1.0`；默认值 `[1.0, 1.0, 1.0]`。这是试验用的基座响应方向修正，尚不等同于传感器安装方向校准。

`/admittance_controller/measured_wrench` 保留转换、去零和滤波后的基座测量值；每秒的 `Force direction` 日志另列出 `sensor_raw`、`base_raw`、`base_filtered` 和带符号修正的 `compliance_input`（死区和限幅之前）。

单独测试时，关闭目标发布器启动 `compliant_control_real.launch.py`，等待 `Force bias ready`，末端无外力时调用以下服务，以当前 TCP 位姿作为目标并开启导纳：

```bash
ros2 service call /admittance_controller/hold_current_pose std_srvs/srv/Trigger '{}'
```

停止导纳输出：

```bash
ros2 service call /admittance_controller/enable std_srvs/srv/SetBool '{data: false}'
```

关闭导纳或 Ctrl+C 退出时，节点先停止流式输出，用最近 250 ms 内的实际关节位置发送一个 0.2 秒、末点速度和加速度为零的保持轨迹，覆盖尚未完成的 IK 目标。Ctrl+C 路径保留 ROS 上下文 0.3 秒后退出。尚未发送过运动命令时不发保持轨迹，避免覆盖其他控制源；关节反馈过期时会报错，不使用旧位置作为保持点。正常收尾日志为 `Admittance stopped: sent current joint positions with zero final velocity`。

`max_joint_command_step` 使用统一比例缩放：先求完整 IK 关节增量，再按最大增量缩小整个向量，保留各关节的运动比例。逐轴独立截断会改变末端的移动方向和姿态。该限幅不保证严格的笛卡尔直线路径，实机仍需观察到位误差和姿态保持效果。

换姿态测试时，先关闭导纳输出。新姿态下末端静止且无外力后调用 `/admittance_controller/tare`（`std_srvs/srv/Trigger`），等待 `Force bias ready`，再调用 `hold_current_pose`。当前零偏采集针对固定姿态，没有工具质量、重心模型的动态重力补偿。

日志 `overload_hold=true` 表示合力曾达到过载阈值，导纳偏移被清零并保持触发时的 TCP 位姿。当前阈值为 400 N，恢复要求合力低于 3 N；仍有数牛顿以上残余读数时会继续保持。不要在施加外力时做 tare。

实际偏移还会受合力限幅、单轴位移上限、偏移速度上限和过载保护影响。

## 参数说明

### `stiffness`

虚拟刚度，单位为 `N/m`。数值越小，同样外力产生的位移越大。

已知期望力和期望位移时，可按下式计算：

```text
stiffness = (expected_force - force_deadband) / expected_offset
```

当前配置：

```yaml
stiffness: [1323.3, 1323.3, 1323.3]
force_deadband: [1.5, 1.5, 1.5]
```

因此：

```text
5 N   -> (5 - 1.5) / 1323.3 = 0.00265 m，约 2.7 mm
20 N  -> (20 - 1.5) / 1323.3 = 0.0140 m，约 1.4 cm
50 N  -> (50 - 1.5) / 1323.3 = 0.0366 m，约 3.7 cm
100 N -> (100 - 1.5) / 1323.3 = 0.0744 m，约 7.4 cm
200 N -> (200 - 1.5) / 1323.3 = 0.150 m，约 15 cm
```

单一线性刚度不能同时精确满足“5 N 位移 1 cm”和“200 N 位移 15 cm”。如果更关注小力灵敏度，5 N 对应 1 cm 时应使用：

```yaml
stiffness: [350.0, 350.0, 350.0]
```

但此时约 `54 N` 就会达到 `15 cm` 位移上限。

### `force_deadband`

力死区，单位为 `N`，用于消除传感器零点波动和小幅噪声。

```yaml
force_deadband: [1.5, 1.5, 1.5]
```

- 末端在无外力时漂移：适当增大。
- 小力完全没有响应：适当减小。
- 调整死区后需重新计算 `stiffness`。

### `compliance_time_constant`

力到柔顺平衡位移的一阶平滑时间常数，单位为秒。

```yaml
compliance_time_constant: 0.10
```

- 减小：目标偏移响应更快，但对力噪声更敏感。
- 增大：响应更平滑，但手感更慢。
- 当前真机配置为 `0.10 s`，需结合力噪声和实际轨迹跟随情况逐步调整。

### `force_filter_alpha` 与力方向

当前为 `0.15`，每次收到传感器数据时执行 `filtered += alpha * (sample - filtered)`。增大会减少滤波延迟，同时传入更多噪声。它与 `compliance_time_constant` 共同影响力到目标偏移的响应时间。

`/sensor/wrench` 表达在传感器坐标系；`/admittance_controller/measured_wrench` 表达在 `base_link`，导纳按这个话题的力方向计算正向偏移。安装旋转由 `sensor_roll`、`sensor_pitch`、`sensor_yaw` 指定，单位为弧度，联合测试 launch 也支持这三个参数。当前默认 yaw 为 `+π/2`，即传感器 +X 指向 tool0 +Y、传感器 +Y 指向 tool0 −X。方向校准时应先关闭导纳输出，逐轴施力，对照这两个话题和实际安装轴向，再修改安装旋转。

### `max_compliance_speed`

柔顺偏移本身的最大变化速度，单位为 `m/s`。

```yaml
max_compliance_speed: [0.08, 0.08, 0.08]
```

当前每轴最多按 `8 cm/s` 改变柔顺目标。这个参数只限制目标生成速度，不代表机械臂必然能以同样速度跟随。

### `trajectory_duration`

每条真机关节轨迹点的名义执行时间，单位为秒。

```yaml
trajectory_duration: 0.06
```

- 减小：机械臂更快追赶 `command_xyz`，关节动作也会更激进。
- 增大：跟踪更柔和，但 `actual_xyz` 的滞后增大。
- 真机建议不要一次大幅减小，可按 `0.10 -> 0.08 -> 0.06 s` 逐级测试。

### `max_compliance_offset`

每个轴相对原始目标的最大柔顺位移，单位为米。

```yaml
max_compliance_offset: [0.15, 0.15, 0.15]
```

当前 X/Y/Z 每轴最多偏移 `15 cm`。达到单轴上限后，该轴保持最大偏移；撤力后返回原始目标。上限必须根据当前位姿的实际工作空间设置。

### `force_norm_limit`

参与柔顺计算的三轴合力上限。多轴受力时会按比例统一缩放，以保持受力方向。

```yaml
force_norm_limit: 200.0
```

### `overload_force_threshold` 和 `protection_release_force`

```yaml
overload_force_threshold: 400.0
protection_release_force: 3.0
```

合力达到 `400 N` 时，控制器立即锁定进入保护时的当前 TCP 位姿，不再增加偏移。合力降到 `3 N` 以下后解除保护。

两个阈值独立：过载判断使用限幅前、去零并滤波后的合力；`force_norm_limit` 只限制导纳方程的输入。当前 `force_norm_limit=200 N`、`overload_force_threshold=400 N`，所以 200~400 N 的推力仍会生成让位目标，目标偏移不会随推力无限增大，每轴仍最多 15 cm。达到 400 N 时仍立即保持当前位置，并不保证先完成让位运动。

### `force_filter_alpha`

力传感器一阶低通滤波系数，取值范围为 `(0, 1]`。

```yaml
force_filter_alpha: 0.05
```

- 增大：力更快进入控制器，但噪声和冲击更明显。
- 减小：力更平滑，但延迟更大。
- 建议先保持 `0.05`，只在确认传感器原始数据平稳后再提高。

### 其他参数

```yaml
compliant_axes: "xyz"
max_joint_command_step: 0.03
hold_orientation_during_compliance: true
bias_sample_count: 200
```

- `compliant_axes`：允许柔顺的平移轴，例如 `"z"` 或 `"xyz"`。
- `max_joint_command_step`：单次关节命令的最大变化，单位为弧度。
- `hold_orientation_during_compliance`：柔顺过程中保持进入柔顺阶段时的末端姿态。
- `bias_sample_count`：启动时采集的力零偏样本数；采样期间必须保持末端无负载。

## 推荐调参顺序

1. 确认无外力时 `/sensor/wrench` 接近零，并保持 `force_deadband=1.5 N`。
2. 只开启一个柔顺轴，例如 `compliant_axes: "z"`，验证力方向。
3. 根据期望的“力-位移”关系计算 `stiffness`。
4. 设置与工作空间相符的 `max_compliance_offset`。
5. 通过 `compliance_time_constant` 和 `max_compliance_speed` 调整目标偏移的响应。
6. 如果 `command_xyz` 变化快、`actual_xyz` 跟踪慢，再小幅减小 `trajectory_duration`。
7. 最后设置 `force_norm_limit` 和 `overload_force_threshold`，不要用提高保护阈值的方式掩盖跟踪问题。

每次只改一类参数，先用小力测试，确认方向、回零和姿态保持都正常后再逐步加力。

## 编译和启动

修改 YAML 后需要重新编译并重启柔顺控制节点：

```bash
cd ~/project/ls65_ros/ls65/eli_ros_ws
colcon build --packages-select eli_cs_robot_usr
source install/setup.bash

ros2 launch eli_cs_robot_usr compliant_control_real.launch.py \
  launch_rviz:=true \
  launch_sensor:=true \
  interactive_input:=true
```

参数是在节点启动时读取的。直接修改 YAML 不会改变已在运行的控制器，必须重启。

## 检查运行参数

```bash
ros2 param get /admittance_controller stiffness
ros2 param get /admittance_controller force_deadband
ros2 param get /admittance_controller compliance_time_constant
ros2 param get /admittance_controller max_compliance_speed
ros2 param get /admittance_controller max_compliance_offset
ros2 param get /admittance_controller overload_force_threshold
```

慢速查看实际生成的末端命令：

```bash
while true; do
  ros2 topic echo --once /admittance_controller/commanded_pose
  sleep 1
done
```

控制器日志中的主要字段：

- `force`：去零偏和滤波后的基坐坐标系外力。
- `offset`：当前柔顺偏移。
- `raw_target_xyz`：用户设置的原始末端目标。
- `command_xyz`：真正用于 IK 的柔顺目标。
- `actual_xyz`：从 TF 获得的当前末端位置。
- `overload_hold=true`：已进入过载保护，当前位姿被锁定。

## 真机测试注意事项

- 确认 `scaled_joint_trajectory_controller` 为 `active`。
- 确认 `/io_and_status_controller/robot_task_running` 为 `true`。
- 首次启动时保持末端无负载，等待 `Force bias ready` 后再施力。
- 从 `5 N` 开始逐步增加，当前过载保持阈值为 `400 N`。
- 确保机械臂在当前方向有足够的工作空间，并随时准备急停。
- 如果末端方向错误、姿态明显改变、关节突跳或无法撤力回零，立即停止测试，不要继续增大外力。

## 树莓派本机联合部署

见 `robot_task_control/README.md` 和 `ros_ws/README_INTEGRATED_CN.md`。真机控制不再强制依赖 Gazebo；使用本包的仿真 launch 时需另外构建 `eli_cs_robot_simulation_gz` 及其依赖。
