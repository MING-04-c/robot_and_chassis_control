# 操控电脑与底盘电脑通信协议 v1

## 1. 角色和连接

| 设备 | 地址 | 角色 |
|---|---|---|
| 底盘/相机电脑（对方） | `169.254.199.100/24` | TCP 服务端，采集底盘状态和相机数据，执行控制命令 |
| 机器人操控电脑（本机） | `169.254.199.250/24` | TCP 客户端，接收状态和图像，发送底盘控制命令 |

| 端口 | 方向 | 内容 | 分帧方式 |
|---:|---|---|---|
| `9001/TCP` | 双向 | 底盘状态、控制命令、心跳、ACK、相机信息 | UTF-8 NDJSON，每条 JSON 后加 `\n` |
| `9002/TCP` | 对方 → 本机 | JPEG 图像 | `MIMG` 二进制帧 |

对方必须在 `169.254.199.100` 上监听 9001 和 9002。本机主动连接。断线后本机自动重连。

## 2. 9001 公共格式

每条消息必须是单行紧凑 JSON，并以一个 LF 字节 `0x0A` 结束。TCP 是字节流，接收端必须缓存并按 `\n` 拆包，不能假设一次 `recv()` 对应一条消息。单条 JSON 不得超过 65536 字节。

所有 9001 消息都包含：

```json
{"version":1,"type":"heartbeat","sequence":1,"timestamp_ns":1790593050000000000,"source":"robot"}
```

| 字段 | 类型 | 要求 |
|---|---|---|
| `version` | integer | 固定为 `1` |
| `type` | string | 消息类型 |
| `sequence` | integer | 发送进程内单调递增；重启后允许重新计数 |
| `timestamp_ns` | integer | Unix 时间纳秒 |
| `source` | string | 对方使用 `robot`，本机使用 `supervisor` |

未知字段应忽略。无法识别的版本或类型应丢弃，不能改变当前状态。

## 3. 对方必须发送的消息

### 3.1 底盘状态 `chassis_status`

对方必须在以下时机发送：

1. 接受新任务时立即发送。
2. 状态变化时立即发送。
3. `running` 期间每 500 ms 发送一次当前快照。
4. `completed` 或 `error` 后每 500 ms 重发，直到收到本机 ACK。
5. TCP 重连后，如果仍有未确认的终态，立即重发。

运行中示例：

```json
{"version":1,"type":"chassis_status","sequence":100,"timestamp_ns":1790593000100000000,"source":"robot","event_id":"chassis-nav-42-running","task":"nav-42","target_id":"station_b","state":1,"state_name":"running","progress":0.63,"message":"moving_to_target","details":{"chassis_state":7,"motion_mode":0,"fault_code":0}}
```

到达示例：

```json
{"version":1,"type":"chassis_status","sequence":101,"timestamp_ns":1790593015000000000,"source":"robot","event_id":"chassis-nav-42-completed","task":"nav-42","target_id":"station_b","state":2,"state_name":"completed","progress":1.0,"message":"target_reached","details":{"chassis_state":7,"motion_mode":0,"fault_code":0}}
```

异常示例：

```json
{"version":1,"type":"chassis_status","sequence":102,"timestamp_ns":1790593016000000000,"source":"robot","event_id":"chassis-nav-42-error","task":"nav-42","target_id":"station_b","state":3,"state_name":"error","message":"motion_fault","details":{"chassis_state":9,"motion_mode":0,"fault_code":1001}}
```

必填字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `event_id` | string | 事件唯一 ID；同一个终态重发时保持不变 |
| `task` | string | 本次任务 ID；同一任务全过程保持不变 |
| `state` | integer | `0` 空闲、`1` 运行、`2` 到达、`3` 异常 |
| `state_name` | string | 必须与 state 对应：`idle/running/completed/error` |

可选字段：`target_id`、`progress`、`message`、`details`。

本机收到 `state=2` 时发布 `/computer_pi/chassis_arrived=true`；其他状态以及离线时发布 `false`。本机收到 `state=2` 或 `state=3` 后自动返回 ACK。

### 3.2 心跳 `heartbeat`

对方每 1 秒发送一次：

```json
{"version":1,"type":"heartbeat","sequence":200,"timestamp_ns":1790593050000000000,"source":"robot","uptime_ms":123456,"status":"ok"}
```

本机连续 3 秒未收到对方任何有效 9001 消息时，判定对方离线：

- `/computer_pi/online=false`
- `/computer_pi/status` 发布本地状态 `state=4, state_name=offline`
- `/computer_pi/chassis_arrived=false`

`offline(4)` 是本机生成的连接状态，对方的 `chassis_status.state` 仍只使用 `0–3`。

### 3.3 相机信息 `camera_info`

相机启动、参数变化、客户端连接时立即发送，运行期间每 1 秒发送一次：

```json
{"version":1,"type":"camera_info","sequence":300,"timestamp_ns":1790593060000000000,"source":"robot","camera_id":"hik_camera_0","online":true,"image_topic":"/camera/image","width":1440,"height":1080,"source_encoding":"rgb8","transport_encoding":"jpeg","jpeg_quality":80,"configured_max_fps":10.0,"measured_fps":9.8,"image_port":9002}
```

相机不可用时仍发送 `camera_info`，设置 `online=false`，并可用 `message` 说明原因。

### 3.4 控制命令确认 `ack`

对方收到并接受本机的 `control_command` 后必须立即回复：

```json
{"version":1,"type":"ack","sequence":400,"timestamp_ns":1790593070000000000,"source":"robot","ack_event_id":"control-a1b2c3"}
```

`ack_event_id` 必须等于控制命令的 `event_id`。ACK 表示命令已接收，不代表动作已经完成；动作进度和完成结果继续使用 `chassis_status` 报告。

对方必须按 `event_id` 对重复控制命令去重。重复收到同一个命令时不能重复创建任务，但仍需再次回复 ACK。

## 4. 本机发送给对方的消息

### 4.1 底盘控制 `control_command`

继续当前暂停的任务：

```json
{"version":1,"type":"control_command","sequence":500,"timestamp_ns":1790593080000000000,"source":"supervisor","event_id":"control-a1b2c3","command":"continue"}
```

停止当前任务：

```json
{"version":1,"type":"control_command","sequence":501,"timestamp_ns":1790593081000000000,"source":"supervisor","event_id":"control-d4e5f6","command":"stop"}
```

直接调用底盘任务：

```json
{"version":1,"type":"control_command","sequence":502,"timestamp_ns":1790593082000000000,"source":"supervisor","event_id":"control-g7h8i9","command":"execute_task","task":"forward_10cm"}
```

| `command` | `task` | 对方行为 |
|---|---|---|
| `continue` | 省略 | 继续当前暂停的任务 |
| `stop` | 省略 | 停止当前任务并进入安全停止状态 |
| `execute_task` | 必填 | 携带 `task` 直接调用并启动对应任务 |

任务调用只使用 `task`，例如 `forward_10cm`。对方发送后续
`chassis_status` 时必须原样返回同一个 `task`。`event_id` 用于区分和确认
每条消息，不表示任务类型。

本机会每 500 ms 重发未确认的控制命令。重发时 `event_id` 和业务内容不变，`sequence` 与 `timestamp_ns` 更新。收到匹配 ACK 后停止重发；离线期间暂停重发，连接恢复后继续。

### 4.2 底盘终态 ACK

本机收到对方 `completed` 或 `error` 状态后回复：

```json
{"version":1,"type":"ack","sequence":503,"timestamp_ns":1790593083000000000,"source":"supervisor","ack_event_id":"chassis-nav-42-completed"}
```

### 4.3 本机心跳

本机每 1 秒发送一次：

```json
{"version":1,"type":"heartbeat","sequence":504,"timestamp_ns":1790593084000000000,"source":"supervisor","uptime_ms":654321,"status":"ok"}
```

## 5. 9002 JPEG 图像

对方在 TCP 客户端连接后连续发送图像，无需等待请求。每帧格式：

| 字段 | 长度 | 格式 |
|---|---:|---|
| Magic | 4 字节 | ASCII `MIMG` |
| Header length | 4 字节 | unsigned 32-bit，大端序 |
| JSON header | `header_length` 字节 | UTF-8 JSON，不带换行 |
| JPEG payload | `payload_length` 字节 | 完整 JPEG 文件 |

Header 示例：

```json
{"version":1,"type":"image","sequence":600,"stamp_ns":1790593090000000000,"frame_id":"camera_optical_frame","topic":"/camera/image","width":1440,"height":1080,"source_encoding":"rgb8","encoding":"jpeg","payload_length":183421}
```

限制：

- `header_length <= 65536`
- `1 <= payload_length <= 33554432`
- 每个字段必须精确读满，不能依赖单次 `recv()`
- 一张图片必须是一份完整 JPEG

## 6. 对方实现清单

对方程序必须完成：

1. 在 `169.254.199.100:9001` 监听 TCP，按 NDJSON 收发。
2. 每秒发送 `heartbeat`。
3. 按状态变化和 500 ms 周期发送 `chassis_status`。
4. 对 `completed/error` 保持同一个 `event_id` 重发，直到收到本机 ACK。
5. 接收并解析 `control_command`：`execute_task`、`continue`、`stop`。
6. 按 `event_id` 去重控制命令，并对每次接收回复 ACK。
7. 每秒发送 `camera_info`。
8. 在 `169.254.199.100:9002` 发送 MIMG JPEG 帧。
9. 断线重连后继续发送心跳、当前底盘状态和未确认终态。

## 7. ROS 接口

本机总启动：

```bash
ros2 launch eli_cs_robot_serial computer_pi.launch.py
```

本机主要接口：

| ROS 接口 | 类型 | 用途 |
|---|---|---|
| `/computer_pi/chassis_status` | `std_msgs/msg/String` | 完整底盘状态 JSON |
| `/computer_pi/chassis_arrived` | `std_msgs/msg/Bool` | 是否到达 |
| `/computer_pi/online` | `std_msgs/msg/Bool` | 9001 状态链路是否在线 |
| `/computer_pi/chassis/control` | `eli_cs_robot_serial/srv/ChassisControl` | 继续、停止、选择方案 |
| `/computer_pi/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG 图像话题 |
| `/computer_pi/camera/image` | `sensor_msgs/msg/Image` | 解码后的 BGR 图像 |
| `/computer_pi/camera/capture` | `std_srvs/srv/Trigger` | 保存当前截图 |
