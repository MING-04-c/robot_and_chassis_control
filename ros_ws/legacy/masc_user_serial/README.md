
# MASC TCP 总启动

## CAN 开机自动初始化（只需安装一次）

`can0` 的波特率和 UP 状态在树莓派重启后不会自动保留。不要把需要 root 权限的
`ip link` 命令直接放进普通 ROS launch；安装 systemd 服务后，系统会在每次开机时
自动把 `can0` 配置为 500 kbit/s，并设置 100 ms bus-off 自动恢复：

```bash
cd /home/tiema/project/masc_chassis/chassis_ros_ws
sudo ./scripts/install_can_service.sh
```

安装只需执行一次。下面的总 launch 默认启动底盘驱动、PID 和 9001 TCP，不启动摄像头：

```bash
source install/setup.bash
ros2 launch masc_user_serial masc_tcp.launch.py
```

查看 CAN 服务与接口状态：

```bash
systemctl status masc-chassis-can.service --no-pager
ip -details -statistics link show can0
```

如果修改了 `scripts/setup_can.sh`，再次执行安装命令即可更新系统中的副本。

需要同时启动摄像头和 9002 图像 TCP 时，显式打开两个开关：

```bash
ros2 launch masc_user_serial masc_tcp.launch.py \
  start_hik_camera:=true image_stream:=true
```

默认网络配置：本机网线 `169.254.199.100`，上位机 `169.254.199.250`，状态端口 `9001`，
图片端口 `9002`。如果只要 TCP 通信、不启动底盘驱动，可以关闭底盘开关：

```bash
ros2 launch masc_user_serial masc_tcp.launch.py \
  start_chassis_driver:=false
```

上位机发回的非 ACK 状态 JSON 会发布到 `/tcp_rx`：

```bash
ros2 topic echo /tcp_rx
```

## 相机图片 TCP 输出

显式设置 `start_hik_camera:=true image_stream:=true` 后，总 launch 会同时启动底盘驱动、海康相机、9001 状态/控制服务和9002图像服务。组合启动强制传入 `show_image:=false`，因此不会启动 OpenCV 预览窗口。图像节点订阅 `/camera/image`，默认发送 JPEG，通过网线接口 `169.254.199.100:9002` 输出；状态节点在 `169.254.199.100:9001` 输出相机与底盘状态，并接收操控电脑的底盘控制命令。两个端口都只接受来自 `169.254.199.250` 的连接。

```bash
ros2 launch masc_user_serial masc_tcp.launch.py \
  start_hik_camera:=true image_stream:=true \
  image_topic:=/camera/image image_max_fps:=60.0
```

默认使用 JPEG 图片模式，JPEG 质量为 80，对端收到可直接打开的完整 `.jpg` 图片。默认最多传输 60 帧/秒，慢客户端会跳过中间帧。需要原始 ROS 图像字节时可加 `image_format:=raw`。默认不启动图像服务；已有其他相机进程、只需要 9002 图像 TCP 时，可单独加 `image_stream:=true`。另一台电脑用网线直连的方法、逐字节协议和 Python 接收程序见 [CAMERA_TCP_PROTOCOL.md](CAMERA_TCP_PROTOCOL.md)。接收程序在 `tools/receive_camera_tcp.py`。

只启动图像服务、不启动9001状态服务时，可运行：

```bash
ros2 run masc_user_serial camera_image_tcp_node.py
```

对端电脑设置网线地址为 `169.254.199.250/24` 后运行：

```bash
python3 receive_camera_tcp.py --host 169.254.199.100 --port 9002 --latest latest.jpg
# 如果发送端选择 image_format:=raw，则改为 --latest latest.raw
```

如果网线 IP 改变，用 `image_listen_ip:=本机IP image_allowed_client_ip:=对端IP` 覆盖启动参数。

## 统一通信协议

相机信息、JPEG 图片、底盘到达状态以及底盘控制命令见
[MASC_COMMUNICATION_PROTOCOL.md](MASC_COMMUNICATION_PROTOCOL.md)。该文档可以直接交给上位机端实现；
9001 状态协议和9002图片协议均已接入总 launch。

任务由 `task_plans.yaml` 配置，默认包含前后、左右和左右旋转。上位机发送
`command=execute_task` 和任务名即可直接启动；也兼容 `select_plan + continue`。
节点向 `/chassis/task_start` 发布方案名称，
并立即开始每 500 ms 上报 `running`。实际底盘任务
到达后，执行节点向 `/target_reached` 发布 `true`，TCP 节点随后每 500 ms
上报 `completed`，直到收到上位机 ACK。

```bash
# 仅用于联调：模拟底盘真实到达
ros2 topic pub --once /target_reached std_msgs/msg/Bool '{data: true}'

# 查看上位机是否在最近 3 秒内发过有效消息
ros2 topic echo /tcp/supervisor_online
```

`stop` 会立即回复 ACK、向 `/cmd_vel` 发布一次全零 `Twist`，并上报
`message=stopped_by_supervisor` 的 error 终态。总 launch 默认启用
`masc_chassis_ros_driver` 内的 `target_pid_controller_node`；该节点收到
`/chassis/task_start` 后，根据 YAML 中的 x、y、yaw 增量执行平移或旋转，
到达后自动发布 `/target_reached=true`。
