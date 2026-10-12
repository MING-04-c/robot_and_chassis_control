# 底盘任务 TCP 协议 v1

## 1. 网络

- 底盘电脑：`169.254.199.100:9001`，TCP 服务端。
- 操控主机：`169.254.199.250`，TCP 客户端。
- 编码：UTF-8 NDJSON，每条 JSON 后必须添加 `\n`。
- 接收端必须缓存 TCP 字节并按照 `\n` 拆包。

## 2. 直接执行任务

主机发送：

```json
{"version":1,"type":"control_command","event_id":"control-xxxx","command":"execute_task","task":"forward_10cm"}
```

字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `version` | integer | 固定为 `1` |
| `type` | string | 固定为 `control_command` |
| `event_id` | string | 命令唯一 ID，重发时保持不变 |
| `command` | string | 执行任务固定为 `execute_task` |
| `task` | string | `task_plans.yaml` 中的任务名称 |

底盘按 `event_id` 去重。重复命令不会重复启动运动，但每次都回复 ACK。

底盘在 YAML 的 `task_names` 中查找任务，并按相同下标读取：

- `task_delta_x_m`
- `task_delta_y_m`
- `task_delta_yaw_rad`

查表成功后立即启动运动。未知任务不执行，也不回复成功 ACK。

## 3. 命令 ACK

底盘接受命令后立即回复：

```json
{"version":1,"type":"ack","sequence":1,"timestamp_ns":1790000000000000000,"source":"robot","ack_event_id":"control-xxxx"}
```

`ack_event_id` 必须等于控制命令的 `event_id`。ACK 表示命令已接收，任务是否完成由
`chassis_status` 表示。

## 4. 底盘状态

运行中：

```json
{"version":1,"type":"chassis_status","sequence":2,"timestamp_ns":1790000000100000000,"source":"robot","event_id":"status-forward_10cm-running","task":"forward_10cm","state":1,"state_name":"running","progress":0.35,"message":"executing_task"}
```

完成：

```json
{"version":1,"type":"chassis_status","sequence":3,"timestamp_ns":1790000002000000000,"source":"robot","event_id":"status-forward_10cm-completed-001","task":"forward_10cm","state":2,"state_name":"completed","progress":1.0,"message":"target_reached"}
```

异常：

```json
{"version":1,"type":"chassis_status","sequence":4,"timestamp_ns":1790000002100000000,"source":"robot","event_id":"status-forward_10cm-error-001","task":"forward_10cm","state":3,"state_name":"error","message":"odometry_timeout"}
```

空闲：

```json
{"version":1,"type":"chassis_status","sequence":5,"timestamp_ns":1790000002200000000,"source":"robot","event_id":"status-idle-001","task":"","state":0,"state_name":"idle","progress":0.0,"message":"waiting_for_task"}
```

状态值：

| state | state_name | 含义 |
|---:|---|---|
| `0` | `idle` | 空闲 |
| `1` | `running` | 执行中 |
| `2` | `completed` | 已完成 |
| `3` | `error` | 异常 |

运行期间每 500 ms 发送 running。completed/error 使用同一个 `event_id` 每 500 ms
重发，直到收到主机 ACK。

主机确认终态：

```json
{"version":1,"type":"ack","ack_event_id":"status-forward_10cm-completed-001"}
```

收到 ACK 后底盘停止重发终态，并发送 idle。

## 5. 停止

```json
{"version":1,"type":"control_command","event_id":"control-stop-xxxx","command":"stop"}
```

底盘回复 ACK、清除 PID 目标、发布零速度，并上报
`message=stopped_by_supervisor` 的 error 状态。

## 6. 任务表

文件：`masc_chassis_ros_driver/config/ros2/task_plans.yaml`

默认任务：

- `forward_10cm`
- `backward_10cm`
- `left_10cm`
- `right_10cm`
- `rotate_left_90deg`
- `rotate_right_90deg`

四个 YAML 数组必须长度相同。增加任务时，在四个数组的相同位置增加名称、x、y、
yaw。x 正负表示前后，y 正负表示左右，yaw 正负表示左转和右转。

## 7. 完整流程

```text
主机                         底盘
  |--- execute_task(task) ---->|
  |<---------- ACK ------------|
  |<------ running(task) -------|
  |<------ running(task) -------|
  |<----- completed(task) ------|
  |------ terminal ACK -------->|
  |<--------- idle -------------|
```

TCP 连接保持不断，双方继续心跳。底盘每秒发送 heartbeat 和 camera_info；图像通过
`169.254.199.100:9002` 的 MIMG/JPEG 协议传输。
