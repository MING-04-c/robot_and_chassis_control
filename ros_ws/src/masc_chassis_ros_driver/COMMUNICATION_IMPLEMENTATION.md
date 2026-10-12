# YAML 底盘可执行任务表

## 1. 唯一控制器

任务表只使用 `masc_chassis_ros_driver` 内的 `target_pid_controller_node`。
总启动将 `chassis_enable_pid` 默认设为 `true`，不会启动
`masc_chassis_user_control`。这样只有 driver 内置 PID 向 `/cmd_vel` 发布任务速度。

任务定义在 `config/ros2/task_plans.yaml`，默认包含：

```yaml
task_names: [forward_10cm, backward_10cm, left_10cm, right_10cm,
             rotate_left_90deg, rotate_right_90deg]
task_delta_x_m: [0.10, -0.10, 0.0, 0.0, 0.0, 0.0]
task_delta_y_m: [0.0, 0.0, 0.10, -0.10, 0.0, 0.0]
task_delta_yaw_rad: [0.0, 0.0, 0.0, 0.0, 1.5707963268, -1.5707963268]
```

`x` 正负表示前后，`y` 正负表示左右，`yaw` 正负表示左转和右转。增加任务时
在四个数组相同位置各增加一项即可。主机可发送任意表内任务，没有固定顺序。

## 2. 完整数据流

```text
主机 169.254.199.250
  │ TCP execute_task(plan_id)
  ▼
status_tcp_node（169.254.199.100:9001）
  │ /chassis/task_start = 任务名
  ▼
target_pid_controller_node
  │ 读取 /wheel_odom
  │ 按 YAML 的 dx、dy、dyaw 计算目标
  │ 20 ms 一次 PID，发布 /cmd_vel
  ▼
masc_chassis_ros_driver_node → CAN → 实际底盘
  │
  ├─ /masc_chassis/chassis_status：底盘固件状态
  ├─ /chassis_arrival_status：running 和 progress
  └─ /target_reached = true：到达 10 cm 目标
       ▼
status_tcp_node → TCP completed → 主机 ACK → idle
```

## 3. 状态变化

| 阶段 | TCP 状态 | 底盘行为 |
|---|---|---|
| 尚未接任务 | `idle(0)` | 不运动 |
| 收到 `select_plan(test)` | 仍为空闲 | 只保存方案并回复 ACK |
| 收到 `continue` | `running(1)` | 从当前位姿生成正前方 0.10 m 目标 |
| 运动中 | 每 500 ms 上报 `running` | PID 持续发布速度，progress 随里程增加 |
| 到达 | `completed(2)` | 先发布全零 Twist，再发布到达信号 |
| 主机确认 | `idle(0)` | 停止重发 completed，等待下一次任务 |
| 里程计丢失或超时 | `error(3)` | 立即发布全零 Twist |
| 收到 stop | `error(3)` | 清除 PID 目标并立即发布全零 Twist |

底盘固件的 `chassis_state`、`motion_mode` 和 `fault_code` 由
`/masc_chassis/chassis_status` 填充进 TCP 状态的 `details`，不在任务控制器中伪造。

## 4. 关键参数

文件：`config/ros2/target_pid_params.yaml`

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `test_distance_m` | `0.10` | 方案 1 前进距离，单位 m |
| `max_linear_speed` | `0.05` | 最大线速度，单位 m/s |
| `position_deadband` | `0.005` | 距目标 5 mm 内视为到达 |
| `odom_timeout_sec` | `0.5` | 超过此时间无里程计则异常停车 |
| `task_timeout_sec` | `10.0` | 任务最长运行时间 |
| `status_period_sec` | `0.5` | running 进度更新周期 |

## 5. 启动与观察

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch masc_user_serial masc_tcp.launch.py
```

检查只能看到一个任务 PID 节点：

```bash
ros2 node list | grep -E 'target_pid|user_control'
```

正常情况下应看到 `/target_pid_controller`，不应看到
`/masc_chassis_user_control`。

观察任务信号：

```bash
ros2 topic echo /chassis/test_start
ros2 topic echo /chassis_arrival_status
ros2 topic echo /target_reached
ros2 topic echo /cmd_vel
ros2 topic echo /wheel_odom
```

实车测试前应确保底盘正前方至少留出 0.5 m 空间，且 `/wheel_odom` 正常更新。
