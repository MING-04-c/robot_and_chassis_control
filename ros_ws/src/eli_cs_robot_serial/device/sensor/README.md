# 六维力传感器

该节点支持 USB-RS485 串口和 TCP-RS485 网口转换器，两种方式使用相同的传感器协议与 ROS 接口。

## 自动启动顺序

当 `auto_start: true` 时，连接成功后依次发送：

1. `AA 55 32 0D 0A`：进入 debug 模式。
2. `AA 55 30 0D 0A`：清零校准（`zero_before_request: true` 时）。
3. `AA 55 02 0D 0A`：开始发送数据。

清零命令会返回多行校准结果，因此节点默认等待 1 秒后才发送 `02`。可通过 `zero_wait_ms` 调整。

节点正常退出时依次发送 `AA 55 01 0D 0A`（停止采集）和 `AA 55 31 0D 0A`（退出 debug）。两条命令之间默认等待 1 秒，可通过 `stop_exit_delay_ms` 调整。

## ROS 接口

- `/sensor/rx`：收到的原始数据，以十六进制字符串显示。
- `/sensor/rx_bytes`：收到的真实字节数组。
- `/sensor/raw_rx`：底层每次读取到的数据块，以 HEX 字符串原样显示，不做帧处理。
- `/sensor/raw_rx_bytes`：底层每次读取到的数据块，以真实字节数组原样发布，不做帧处理。
- `/sensor/wrench`：解析后的六维力；只有明确数据帧格式后才能在 YAML 中开启。
- `sensor_plot.launch.py`：实时显示 Fx/Fy/Fz/Mx/My/Mz 六条曲线。
- `/sensor/start`：进入 debug、清零并开始发送。
- `/sensor/stop`：先停止采集，再退出 debug。
- `/sensor/zero`：发送清零校准命令。
- `/sensor/toggle_unit`：执行 `01 -> 35 -> 30 -> 02`，切换单位并重新开始采集。
- `/sensor/set_baud`：发送 YAML 中 `baud_code` 指定的波特率切换命令。
- `/sensor/write`：手动发送任意 HEX 命令。

`/sensor/wrench.header.frame_id` 默认是 `force_sensor`，表示数值沿传感器自身
XYZ 轴表达。若传感器相对机器人 `tool0` 有旋转，必须发布
`tool0 -> force_sensor` 的固定 TF；不能只把 `frame_id` 改成 `tool0`，否则
导纳控制的受力方向会错误。

所有传感器协议命令都固定以 HEX 原始字节发送，不会追加换行或转换成 ASCII 文本。`AA 55 35 0D 0A` 不包含目标单位，因此只能表示“切换单位”。启动前应确认设备当前单位。
若设备当前是 kgf、希望以后默认使用 N/N·m，应先以 `unit:=kg` 的节点运行一次 `/sensor/toggle_unit`，确认设备已切到 N 后重启传感器节点，并使用 `sensor_tcp.yaml` 中的 `unit: "N"`。不要在每次启动时自动调用该服务；重复切换会回到 kgf。
`baud_code: 1` 已知代表 460800；代码保留 2、3，但使用前必须查设备手册。

## 启动

USB-RS485：

```bash
ros2 launch eli_cs_robot_serial sensor_usb.launch.py
```

TCP-RS485：

```bash
ros2 launch eli_cs_robot_serial sensor_tcp.launch.py
```

485 转网口设备应配置为 TCP Server，并将串口侧参数设置为设备要求的
`460800 8N1`。ROS 节点作为 TCP Client 连接网口设备。IP 和端口可以在
启动时直接覆盖：

```bash
ros2 launch eli_cs_robot_serial sensor_tcp.launch.py \
  host:=192.168.1.100 \
  port:=4001
```

连接成功后，TCP 与 USB 使用完全相同的原始十六进制命令、数据帧解析、
ROS 话题和服务。TCP 断开后节点会自动重连，并在重连成功后重新执行
`32 -> 30 -> 02` 启动流程。

监听原始回包：

```bash
ros2 topic echo /sensor/rx
```

手动控制：

```bash
ros2 service call /sensor/zero std_srvs/srv/Trigger "{}"
ros2 service call /sensor/start std_srvs/srv/Trigger "{}"
ros2 service call /sensor/stop std_srvs/srv/Trigger "{}"
ros2 service call /sensor/toggle_unit std_srvs/srv/Trigger "{}"转换单位的
ros2 service call /sensor/set_baud std_srvs/srv/Trigger "{}"
ros2 service call /sensor/write eli_cs_robot_serial/srv/WriteSerial "{data: 'AA 55 30 0D 0A'}"
```

单位切换流程中普通命令的停顿由 `unit_switch_delay_ms` 控制，清零后的等待由 `zero_wait_ms` 控制。

设备数据帧支持两种形式：29 字节二进制帧（帧头 `AA 55`、命令码 `02/03`、24 字节大端 IEEE754 浮点、帧尾 `0D 0A`），以及 debug 模式下的 `channels: v1,v2,v3,v4,v5,v6` ASCII 行。USB/TCP 配置默认开启 `publish_wrench`。`unit` 声明设备当前输出单位：`N` 表示 N/N·m，`kg` 表示 kgf/kgf·m；节点会将 `kg` 数值乘以 `9.80665`，使 `/sensor/wrench` 始终为 N/N·m。该参数不会切换设备单位。

当前已知协议为 29 字节大端浮点帧，USB 配置会发布 `/sensor/wrench`。启动实时曲线：

```bash
ros2 launch eli_cs_robot_serial sensor_plot.launch.py
```
