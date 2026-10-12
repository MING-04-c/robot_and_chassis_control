# 给主机端 Codex 的提示词

把下面整段复制到主机电脑的 Codex：

```text
你需要实现机器人操控主机一侧的 ROS 2/TCP 底盘任务客户端。

网络角色：
- 本机（操控主机）IP：169.254.199.250/24
- 底盘电脑 IP：169.254.199.100/24
- 本机作为 TCP 客户端连接 169.254.199.100:9001
- 使用 UTF-8 NDJSON，每条紧凑 JSON 末尾必须有一个 \n
- TCP 接收必须缓存字节并按 \n 拆包，不能把一次 recv 当成一条消息

所有消息公共字段：
- version 固定为 1
- sequence 为本进程内递增整数
- timestamp_ns 为 Unix 时间纳秒
- 本机 source 固定使用 supervisor

可执行任务名为：`forward_10cm`、`backward_10cm`、`left_10cm`、
`right_10cm`、`rotate_left_90deg`、`rotate_right_90deg`。没有固定顺序。
优先使用一条消息直接启动任务：

{"version":1,"type":"control_command","event_id":"control-唯一值","command":"execute_task","task":"forward_10cm"}

底盘收到后立即 ACK 并执行。未收到 ACK 时每 500 ms 使用相同 event_id 重发。
后续流程如下：

1. TCP 连接成功后，每秒发送一次 heartbeat。
2. 生成唯一 event_id，发送：
   {"version":1,"type":"control_command","sequence":1,"timestamp_ns":当前时间,"source":"supervisor","event_id":"control-select-唯一值","command":"select_plan","plan_id":"forward_10cm"}
3. 如果 500 ms 内没收到 ack_event_id 等于该 event_id 的 ACK，保持 event_id 和业务字段不变并重发，只更新 sequence 和 timestamp_ns。
4. 收到选择方案 ACK 后，再生成新的唯一 event_id，发送：
   {"version":1,"type":"control_command","sequence":2,"timestamp_ns":当前时间,"source":"supervisor","event_id":"control-continue-唯一值","command":"continue"}
5. continue 同样每 500 ms 重发，直到收到匹配 ACK。
6. 收到底盘 task 等于当前任务、state=1 后记录本轮已经开始。
7. 只有收到当前任务 running 后，才接受同一 task 的 completed。
8. 收到 completed 后立即回复：
   {"version":1,"type":"ack","sequence":递增值,"timestamp_ns":当前时间,"source":"supervisor","ack_event_id":"收到的 completed.event_id"}
9. ACK 发出后，在主机本地等待业务要求的秒数，然后触发机械臂下一步动作。机械臂结果不需要发回底盘。
10. 收到 state=3/error 时，也要 ACK 对应 event_id，但禁止触发机械臂，并把错误显示出来。
11. TCP 不要在任务结束后关闭，继续心跳并等待下一次任务。
12. 连续 3 秒收不到底盘有效消息时，标记底盘离线，清除本轮 running 标志，禁止触发机械臂并自动重连。

停止按钮发送新的唯一 event_id：
{"version":1,"type":"control_command","sequence":递增值,"timestamp_ns":当前时间,"source":"supervisor","event_id":"control-stop-唯一值","command":"stop"}
stop 每 500 ms 重发直到收到匹配 ACK。

`forward_10cm` 向前 0.10 m，`backward_10cm` 向后 0.10 m。每项到达后底盘先停车，再发送 completed；主机确认前进完成后才能选择后退方案。

请给出完整可运行代码、依赖、启动文件和测试方法。不要使用 SSH；直接通过 169.254.199.100:9001 建立 TCP 连接。
```
