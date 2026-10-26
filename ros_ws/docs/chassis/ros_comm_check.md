# MASC 底盘 ROS 通讯检查

本文用于确认 ROS 节点、话题和 CAN SDK 通讯状态是否正常。

检查前请先确认 CAN 网口已经 `UP`，并且 CAN 层能收到版本反馈：

```bash
candump -tz can0,205:7FF
```

如果系统没有 `candump`，请先安装 `can-utils`。CAN 接口配置和 CAN 层测试见
[`can_usage.md`](can_usage.md)。

## ROS2 检查命令

确认节点已经启动：

```bash
ros2 node list | grep masc_chassis
```

确认话题已经创建：

```bash
ros2 topic list | grep /masc_chassis
```

查看通讯状态：

```bash
ros2 topic echo --once /masc_chassis/comm_status
```

查看 CAN 接收统计：

```bash
ros2 topic echo --once /masc_chassis/can_frame_stats
```

查看底盘状态：

```bash
ros2 topic echo --once /masc_chassis/chassis_status
```

查看发布频率：

```bash
ros2 topic hz /masc_chassis/comm_status
ros2 topic hz /masc_chassis/can_frame_stats
ros2 topic hz /masc_chassis/odom
```

## ROS1 检查命令

确认节点已经启动：

```bash
rosnode list | grep masc_chassis
```

确认话题已经创建：

```bash
rostopic list | grep /masc_chassis
```

查看通讯状态：

```bash
rostopic echo -n 1 /masc_chassis/comm_status
```

查看 CAN 接收统计：

```bash
rostopic echo -n 1 /masc_chassis/can_frame_stats
```

查看底盘状态：

```bash
rostopic echo -n 1 /masc_chassis/chassis_status
```

查看发布频率：

```bash
rostopic hz /masc_chassis/comm_status
rostopic hz /masc_chassis/can_frame_stats
rostopic hz /masc_chassis/odom
```

## `comm_status` 判断方法

`/masc_chassis/comm_status` 是 CAN SDK 通讯状态。重点看下面字段：

| 字段 | 正常值 / 判断 |
|---|---|
| `comm_state` | `4` 表示已连接；`2` 表示连接中；`3` 表示协议版本不匹配 |
| `fail_reason` | `0` 表示无失败原因；非 0 时按 `FAIL_REASON_*` 常量判断 |
| `motion_command_timeout_active` | 没有持续收到 `cmd_vel` 时可能为 `true`，此时 SDK 发送零目标 |
| `connection_attempt_count` | 开启自动重连后会随连接尝试增加 |
| `transport_error_count` | 持续增加通常表示 CAN 接口异常、bus-off 或设备断开 |
| `tx_error_count` / `rx_error_count` | 持续增加表示发送或接收失败 |
| `chassis_protocol_version` | CAN V10 应为 `10` |
| `chassis_type_version` | 底盘类型，`97/98/99` 分别对应 `0x61/0x62/0x63` |

`comm_state` 常量：

| 值 | 说明 |
|---:|---|
| `0` | 未知 |
| `1` | 未连接 |
| `2` | 连接中 |
| `3` | 协议版本不匹配 |
| `4` | 已连接 |

`fail_reason` 常量：

| 值 | 说明 |
|---:|---|
| `0` | 无失败原因 |
| `1` | CAN 端口打开失败 |
| `2` | 等待版本反馈超时 |
| `3` | 协议版本不匹配 |
| `4` | CAN 读取失败 |
| `5` | CAN 写入失败 |
| `6` | transport 错误 |
| `255` | 未知错误 |

## `can_frame_stats` 判断方法

`/masc_chassis/can_frame_stats` 用于确认各 CAN ID 是否持续收到且能被解析。常见应关注：

| CAN ID | 说明 |
|---|---|
| `0x205` | 系统版本反馈，连接握手必须收到 |
| `0x204` | 系统状态和故障反馈 |
| `0x200` | 运动反馈，odom 依赖该帧 |
| `0x206` / `0x207` | BMS 反馈 |
| `0x251-0x258` | 驱动轮反馈 |
| `0x271-0x278` | 舵机反馈 |
| `0x300` | IMU 反馈 |
| `0x301` | 遥控器反馈 |

每项统计重点看：

* `rx_count` 是否持续增加。
* `parse_error_count` 是否为 `0` 或不再增加。
* `drop_count` 是否为 `0` 或不再增加。
* `frequency` 是否接近底盘实际 CAN 发送频率。
* `last_rx_age_ms` 是否小于该帧的超时阈值。

如果 `0x205` 没有出现，驱动无法完成连接握手。先回到 CAN 层检查 `candump`、bitrate、
CANH/CANL 和终端电阻。

如果 `rx_count` 增加但 `parse_error_count` 也增加，通常表示当前固件协议和驱动协议不一致，
或 DLC 与当前协议定义不一致。

## `chassis_status` 判断方法

`/masc_chassis/chassis_status` 是解析后的底盘运行状态。常用字段：

| 字段 | 说明 |
|---|---|
| `chassis_state` | 底盘显示状态，见 `CHASSIS_STATE_*` 常量 |
| `current_motion_mode` | 当前运动模式，见 `MOTION_MODE_*` 常量 |
| `motion_mode_switching` | 是否处于运动模式切换中 |
| `brake_active` | 抱闸是否激活 |
| `low_battery` | 是否低电量 |
| `manual_charging` / `dock_charging` | 是否处于手动充电或充电桩充电 |
| `main_power_active` | 动力电源是否接通 |
| `obstacle_status_flags` | 障碍状态 flags |
| `system_status_flags` | `0x204 Byte0-Byte3` 原始状态 |
| `chassis_fault` | 底盘故障原始值 |

`chassis_state` 常量：

| 值 | 说明 |
|---:|---|
| `0` | 未知 |
| `1` | 急停 |
| `2` | 拖车 |
| `3` | 故障 |
| `4` | 自检 |
| `5` | 停车 |
| `6` | 遥控 |
| `7` | 通讯控制 |

## 常见现象

| 现象 | 建议检查 |
|---|---|
| `comm_state=2` 长时间不变 | `candump -tz can0,205:7FF` 是否能看到 `0x205` |
| `comm_state=3` | `chassis_protocol_version` 和参数 `can.expected_protocol_version` 是否一致 |
| `fail_reason=1` | CAN 接口名是否存在，例如 `ip link show type can` |
| `transport_error_count` 持续增加 | CAN 设备是否断开、接口是否 bus-off、驱动是否有权限打开 socket |
| `can_frame_stats` 没有 `0x200` | 固件是否发送运动反馈，或 CAN 过滤/接线是否异常 |
| `odom` 没有数据 | `0x200` 是否有频率，`freshness.motion_feedback_timeout_ms` 是否过小 |
| `chassis_status` 没有数据 | `0x204` 是否有频率且解析错误是否为 0 |

无 CAN 设备或接口未配置时，节点仍可启动，`comm_status.comm_state` 会显示未连接或连接中。
