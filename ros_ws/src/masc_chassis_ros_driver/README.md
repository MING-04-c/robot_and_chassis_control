# MASC 底盘 ROS 驱动 API

固定 TCP `test` 方案（从当前位置向正前方运动 10 cm）的控制链、状态变化和参数见
[COMMUNICATION_IMPLEMENTATION.md](COMMUNICATION_IMPLEMENTATION.md)。可直接交给主机端 Codex
的实现提示词见 [SUPERVISOR_PROMPT.md](SUPERVISOR_PROMPT.md)。

多机协同（苏州）有限公司-MASC 驱动API。

`masc_chassis_ros_driver` 是 MASC 底盘 CAN SDK 的 ROS 通讯驱动节点。同时支持
ROS1 和 ROS2，外部安装、编译和基础启动方式见上层
[`README.md`](../README.md)。

本文档只说明 ROS API、话题、服务和参数配置。

## 节点

默认 launch 中节点名为：

```text
masc_chassis
```

驱动内部使用私有话题和服务名，例如 `~/odom`、`~/chassis_status`。默认节点名下实际话题
路径为 `/masc_chassis/odom`、`/masc_chassis/chassis_status`。

## 发布的话题

| 话题 | 类型 | 默认检查频率 / 实际发布 | 说明 |
|---|---|---:|---|
| `/masc_chassis/odom` | `nav_msgs/Odometry` | `50 Hz` | 基于 CAN 运动反馈积分的里程计，实际发布随 `0x200` 更新 |
| `/masc_chassis/chassis_status` | `masc_chassis_msgs/ChassisStatus` | `10 Hz` | 底盘状态、运动模式、故障原始值 |
| `/masc_chassis/bms_status` | `masc_chassis_msgs/BmsStatus` | `1 Hz` | BMS/电池状态 |
| `/masc_chassis/comm_status` | `masc_chassis_msgs/CommStatus` | `1 Hz` | CAN SDK 连接状态、错误计数、版本信息 |
| `/masc_chassis/drive_motors` | `masc_chassis_msgs/DriveMotorArray` | `50 Hz` | 驱动轮速度、电流、位置 |
| `/masc_chassis/drive_motor_status` | `masc_chassis_msgs/DriveMotorStatusArray` | `10 Hz` | 驱动轮温度、驱动器故障 |
| `/masc_chassis/servo_motors` | `masc_chassis_msgs/ServoMotorArray` | `50 Hz` | 舵机角度、电流、标定状态 |
| `/masc_chassis/servo_motor_status` | `masc_chassis_msgs/ServoMotorStatusArray` | `10 Hz` | 舵机温度、驱动器故障 |
| `/masc_chassis/imu` | `sensor_msgs/Imu` | `50 Hz` | IMU 姿态四元数 |
| `/masc_chassis/rc_status` | `masc_chassis_msgs/RcStatus` | `20 Hz` | 遥控器通道原始值 |
| `/masc_chassis/can_frame_stats` | `masc_chassis_msgs/CanFrameStatsArray` | `1 Hz` | 各 CAN ID 接收频率和解析统计 |

如果修改节点名或 namespace，实际话题路径会随 ROS 命名空间规则变化。

反馈类话题使用“新数据触发发布”策略：CAN SDK callback 更新缓存时递增内部 sequence，
ROS timer 只按配置频率检查缓存；如果 sequence 没有变化，就不重复发布旧数据。因此
`publish_rate.*_hz` 是最高检查频率，不保证高于实际 CAN 接收频率。例如默认 `odom_hz=100`
只是 100 Hz 检查缓存；如果 `motion_control_feedback (0x200)` 实际为 50 Hz，
`/masc_chassis/odom` 最多约 50 Hz。
`comm_status` 和 `can_frame_stats` 属于通讯诊断状态，仍按配置频率周期发布。
`can_frame_stats.stats[].frequency` 由 SDK 按 CAN ID 接收间隔计算，并经过简单低通平滑；
它表示 CAN 接收频率，不表示 ROS topic 发布频率。

## 订阅的话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` / `geometry_msgs/TwistStamped` | 底盘速度命令，默认 launch 将私有 `~/cmd_vel` 重映射到该话题 |

`command.stamped_control=false` 时订阅 `geometry_msgs/Twist`；
`command.stamped_control=true` 时订阅 `geometry_msgs/TwistStamped`，并使用其中的
`twist` 字段作为底盘速度命令。`cmd_vel` 只更新最新命令和接收时间，timeout 按驱动收到
消息的时间计算。驱动通过 command timer 周期调用 `sdk.SetMotionCommand()`，默认周期为
`20 ms`。如果命令超过 `command.timeout_ms` 未更新，驱动会继续发送零目标。

`auto_lock` 是停稳自动锁止功能。启用后，驱动在默认底盘模式下检测到命令低通后归零、
`motion_control_feedback` 实际速度停稳，并持续超过 `auto_lock.delay_ms` 后，自动发送
`MOTION_MODE_PARK`。该功能只对支持运动模式切换的全轮转向/舵轮底盘生效。
`auto_lock.auto_unlock_on_cmd=true` 时，锁止后收到新的非零 `cmd_vel` 会先请求切回默认底盘模式，
并在模式反馈确认前继续发送零目标。设置为 `false` 时，上层需要调用 `set_motion_mode`
切回默认底盘模式。

## 服务话题

| 服务 | 类型 | 说明 |
|---|---|---|
| `/masc_chassis/set_motion_mode` | `masc_chassis_msgs/SetMotionMode` | 切换运动模式，仅 `chassis_type_version` 为 `0x61`、`0x62` 或 `0x63` 时支持 |
| `/masc_chassis/set_tow_mode` | `masc_chassis_msgs/SetTowMode` | 进入或退出拖车模式 |
| `/masc_chassis/clear_fault` | `masc_chassis_msgs/ClearFault` | 清除指定故障 |
| `/masc_chassis/calibrate_servo` | `masc_chassis_msgs/CalibrateServo` | 触发固件舵机自动标定 |

服务在未连接 CAN 时返回 `success=false`。连接正常且 SDK 写入命令成功时返回
`success=true`。服务响应只表示驱动是否发送了请求，执行结果需要通过反馈话题观察。
如果 `system_version_feedback.chassis_type_version` 不是 `0x61`、`0x62` 或 `0x63`，
`set_motion_mode` 会返回 `success=false`。

ROS2 调用示例：

```bash
ros2 service call /masc_chassis/set_motion_mode \
  masc_chassis_msgs/srv/SetMotionMode "{motion_mode: 0}"

ros2 service call /masc_chassis/set_tow_mode \
  masc_chassis_msgs/srv/SetTowMode "{enable: true}"

ros2 service call /masc_chassis/clear_fault \
  masc_chassis_msgs/srv/ClearFault "{fault_id: 0}"

ros2 service call /masc_chassis/calibrate_servo \
  masc_chassis_msgs/srv/CalibrateServo "{servo_id: 0}"
```

ROS1 调用示例：

```bash
rosservice call /masc_chassis/set_motion_mode "motion_mode: 0"
rosservice call /masc_chassis/set_tow_mode "enable: true"
rosservice call /masc_chassis/clear_fault "fault_id: 0"
rosservice call /masc_chassis/calibrate_servo "servo_id: 0"
```

## 运动模式

`SetMotionMode.motion_mode` 使用 `masc_chassis_msgs/SetMotionMode` 中的常量：
该服务只对 `chassis_type_version` 为 `0x61`、`0x62` 或 `0x63` 的底盘开放。

| 值 | 名称 | 说明 |
|---|---|---|
| `0` | `MOTION_MODE_DEFAULT_CHASSIS` | 默认底盘模式 |
| `1` | `MOTION_MODE_PARK` | 驻车 |
| `2` | `MOTION_MODE_ACKERMANN` | 阿克曼 |
| `3` | `MOTION_MODE_SPIN` | 原地旋转 |
| `4` | `MOTION_MODE_LATERAL` | 横移 |
| `5` | `MOTION_MODE_DIAGONAL` | 斜移 |

## 消息映射

### 时间戳

SDK callback 进入 ROS driver 时立即调用 ROS `Now()` 打时间戳并写入 cache。发布 timer
使用 cache 中的时间作为 `header.stamp`，避免把发布周期延迟误认为数据接收时间。


### Odometry

`MotionFeedback` 只转换为 `nav_msgs/Odometry`，不额外发布运动反馈 topic。

`odom.publish_tf=true` 时，驱动同时发布 `odom.frame_id -> odom.child_frame_id` 的 TF。

### ServoMotor

SDK 中舵机角度按 degree 解析，ROS 话题 `ServoMotor.angle` 发布为 rad。

### sensor_msgs/Imu

SDK 中 IMU 姿态角按 degree 解析，ROS 话题 `/masc_chassis/imu` 使用标准
`sensor_msgs/Imu` 发布。驱动将 SDK 中的 roll/pitch/yaw 转换为 `orientation` 四元数；
`angular_velocity` 和 `linear_acceleration` 当前协议未提供，因此 covariance 首项置为 `-1`。
IMU 温度不属于 `sensor_msgs/Imu` 字段，当前只保留在 SDK 解析数据中。

### RcStatus

`RcStatus.online` 表示遥控器在线状态。`RcStatus.channels` 发布遥控器 `CH0-CH6` 原始值，
范围 `-100~100`。驱动不解释每个通道的业务含义。

### BmsStatus

CAN SDK 缺失的 BMS 字段使用默认值 `0`。`battery_fault == 0` 时发布
`BMS_HEALTH_GOOD`，非 0 时发布 `BMS_HEALTH_UNSPEC_FAILURE`，BMS feedback 超时后发布
`BMS_HEALTH_TIMEOUT`。

## 参数设置

参数文件：

* ROS1：`config/ros1/driver_params.yaml`
* ROS2：`config/ros2/driver_params.yaml`

ROS1 参数文件使用普通 YAML，驱动会把参数名里的 `.` 转换为 `/`，例如
`can.interface_name` 对应 `can/interface_name`。

ROS2 参数文件使用：

```yaml
/**:
  ros__parameters:
```

### CAN 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `can.interface_name` | `can0` | SocketCAN 接口名 |
| `can.bitrate` | `500000` | CAN bitrate，系统 CAN 网口仍需提前配置 |
| `can.expected_protocol_version` | `10` | 期望底盘协议版本 |
| `can.connect_timeout_ms` | `1000` | 连接和版本握手超时 |
| `can.auto_reconnect` | `true` | 连接断开后是否自动重连 |
| `can.reconnect_period_ms` | `1000` | 重连间隔 |

### 命令参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `command.period_ms` | `20` | 运动命令发送周期 |
| `command.timeout_ms` | `100` | `cmd_vel` 超时时间 |
| `command.stamped_control` | `false` | `false` 订阅 `geometry_msgs/Twist`，`true` 订阅 `geometry_msgs/TwistStamped` |
| `command.max_linear_x` | `1.5` | x 方向速度限幅，单位 `m/s` |
| `command.max_linear_y` | `1.0` | y 方向速度限幅，单位 `m/s` |
| `command.max_angular_z` | `2.0` | z 轴角速度限幅，单位 `rad/s` |
| `command.max_steering_angle` | `2.0` | 预留转向角限幅，单位 `rad` |

### 停稳自动锁止参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `auto_lock.enabled` | `true` | 是否启用停稳自动锁止 |
| `auto_lock.delay_ms` | `3000` | 命令和反馈都停稳后，等待多久切换到驻车模式 |
| `auto_lock.cmd_filter_alpha` | `0.2` | 自动锁止判定用命令低通系数，不改变实际发送命令 |
| `auto_lock.cmd_zero_linear_threshold` | `0.01` | 低通后线速度吸附为零的阈值，单位 `m/s` |
| `auto_lock.cmd_zero_angular_threshold` | `0.01` | 低通后角速度/转向角吸附为零的阈值，单位 `rad/s` / `rad` |
| `auto_lock.feedback_linear_stop_threshold` | `0.02` | 反馈线速度停稳阈值，单位 `m/s` |
| `auto_lock.feedback_angular_stop_threshold` | `0.02` | 反馈角速度停稳阈值，单位 `rad/s` |
| `auto_lock.auto_unlock_on_cmd` | `true` | 锁止后收到非零命令时，是否自动切回默认底盘模式 |

### 里程计参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `odom.frame_id` | `odom` | odom frame |
| `odom.child_frame_id` | `base_link` | base frame |
| `odom.publish_tf` | `false` | 是否发布 `odom -> base_link` TF |
| `odom.linear_scale` | `1.0` | 里程计线速度比例 |
| `odom.angular_scale` | `1.0` | 里程计角速度比例 |

### IMU 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `imu.frame_id` | `imu_link` | `/masc_chassis/imu` 的 header frame_id |

### 发布频率参数

以下参数表示 timer 检查频率。反馈类 topic 只有在对应 CAN 数据 sequence 更新后才发布，
实际发布频率不会超过 CAN 数据更新频率。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `publish_rate.odom_hz` | `100.0` | odom 最高检查频率 |
| `publish_rate.chassis_status_hz` | `10.0` | chassis status 最高检查频率 |
| `publish_rate.motor_hz` | `50.0` | 电机实时反馈最高检查频率 |
| `publish_rate.motor_status_hz` | `10.0` | 电机状态最高检查频率 |
| `publish_rate.imu_hz` | `50.0` | IMU 状态最高检查频率 |
| `publish_rate.rc_hz` | `20.0` | 遥控器状态最高检查频率 |
| `publish_rate.bms_hz` | `1.0` | BMS 最高检查频率 |
| `publish_rate.comm_hz` | `1.0` | 通讯状态发布频率 |
| `publish_rate.frame_stats_hz` | `1.0` | CAN 帧统计发布频率 |

### 数据时效参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `freshness.motion_feedback_timeout_ms` | `200` | motion feedback 超时判定 |
| `freshness.system_feedback_timeout_ms` | `500` | system/motion mode feedback 超时判定 |
| `freshness.bms_feedback_timeout_ms` | `2500` | BMS feedback 超时判定 |
| `freshness.imu_feedback_timeout_ms` | `100` | IMU feedback 超时判定 |
| `freshness.rc_feedback_timeout_ms` | `250` | RC feedback 超时判定 |

## ROS 通讯检查

ROS 节点、话题、`comm_status`、`can_frame_stats` 和常见异常判断见
[`../docs/ros_comm_check.md`](../docs/ros_comm_check.md)。
