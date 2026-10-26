# 六维力传感器

该节点支持 USB-RS485 串口和 TCP-RS485 网口转换器，两种方式使用相同的传感器协议与 ROS 接口。

## 自动启动顺序

当 `auto_start: true` 时，连接成功后依次发送：

1. `AA 55 32 0D 0A`：进入 debug 模式。
2. 自动单位检测：默认每组采样前清零并等待完成，先采 100 帧，停止采集并切换单位，再次清零、稳定后采 100 帧。
3. 比例验证通过后才执行原有清零（`zero_before_request: true`），然后重新开始采集，发布统一单位数据。

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
- `/sensor/toggle_unit`：兼容旧服务名，现执行与 `/sensor/calibrate_unit` 相同的前后采样验证，不再猜测切换后的单位。
- `/sensor/set_baud`：发送 YAML 中 `baud_code` 指定的波特率切换命令。
- `/sensor/write`：手动发送任意 HEX 命令。

`/sensor/wrench.header.frame_id` 默认是 `force_sensor`，表示数值沿传感器自身
XYZ 轴表达。若传感器相对机器人 `tool0` 有旋转，必须发布
`tool0 -> force_sensor` 的固定 TF；不能只把 `frame_id` 改成 `tool0`，否则
导纳控制的受力方向会错误。

所有传感器协议命令都固定以 HEX 原始字节发送，不会追加换行或转换成 ASCII 文本。`AA 55 35 0D 0A` 不包含目标单位，因此只能表示“切换单位”。自动检测通过两组读数识别单位；人工模式下才由操作员确认单位。
默认每次连接/重连自动识别单位，不使用上次上电的单位假设。详见下方自动识别说明。
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

设备数据帧支持两种形式：29 字节二进制帧（帧头 `AA 55`、命令码 `02/03`、24 字节大端 IEEE754 浮点、帧尾 `0D 0A`），以及 debug 模式下的 `channels: v1,v2,v3,v4,v5,v6` ASCII 行。USB/TCP 配置默认开启 `publish_wrench`。仅在 `auto_calibrate_unit: false` 人工模式中，`unit` 声明设备当前输出单位：`N` 表示 N/N·m，`kg` 表示 kgf/kgf·m；节点会将 `kg` 数值乘以 `9.80665`，使 `/sensor/wrench` 始终为 N/N·m。该参数不会切换设备单位。

当前已知协议为 29 字节大端浮点帧，USB 配置会发布 `/sensor/wrench`。启动实时曲线：

```bash
ros2 launch eli_cs_robot_serial sensor_plot.launch.py
```


## 上电后自动识别单位

默认 `auto_calibrate_unit: true`、`target_unit: N`。连接 → debug → 清零/等待/清缓存 → 切换前 100 帧 → 停止采集/清缓存 → `35` 切换一次 → 再次清零/等待/清缓存 → 切换后 100 帧 → 比较 → 对齐目标单位（如需再切换则再次清零、额外采 100 帧复核）→ 可选清零 → READY → 发布 `/sensor/wrench`。单位未确认或识别失败期间不发布控制用力，原始字节/HEX 数据继续保留。失败不自动反复切换。

按标准换算，同一负载下 N 数值约为 kgf 的 9.80665 倍，大数是 N。比例接近 9.80665 才识别当前档位；若当前设备为 kgf，则软件乘 9.80665，统一输出力 N、力矩 N·m，硬件最后停留在 `target_unit` 指定档位，ROS 话题仍为 N/N·m。`kg` 是旧配置名称，实际指 kgf/kgf·m，不是质量 kg，也不是 N/m 或 kg/m。力矩沿用既有协议假设为 N·m 或 kgf·m；有足够力矩信号时同步检查缩放比例，低信号轴不能据此获得完整标定。

识别前保持机械臂不动、姿态和负载不变，必须有两档都明显非零的稳定读数（例如已固定工具的重力）；比较期间不触碰，也不要从其他程序插入清零命令。默认实验选项会在每组采样前自动清零。每组清零会消去固定静载，可能只剩近零噪声，不能保证还能识别 9.80665 倍。需要比较原有静载时，将 `unit_calibration_zero_each_group` 设为 false；无有效信号时仍不认定单位。该流程只是单位档位识别，不是量程、耦合矩阵等完整六维标定，也不代表设备精度。

```bash
ros2 launch eli_cs_robot_serial sensor_tcp.launch.py
ros2 topic echo /sensor/unit_status
# 识别失败后，保持稳定非零负载，再重新检测：
ros2 service call /sensor/calibrate_unit std_srvs/srv/Trigger '{}'
```

`/sensor/unit_status` 保留最新状态，显示 BEFORE、AFTER、READY 或 FAILED，以及两组原始六维均值、比例、`samples=100+100`、设备单位和软件换算系数。`device_unit=N` 对应 `scale=1`；`device_unit=kg` 对应 `scale=9.80665`。服务返回成功仅表示接收请求，识别是否成功以 READY 为准。检测期间拒绝插入清零、改波特率或原始写命令；`/sensor/stop` 和退出节点可以取消检测。

USB/TCP YAML 参数：`unit_calibration_samples: 100`；`unit_calibration_timeout: 5.0`（每组秒数）；`unit_calibration_min_force: 0.05`（两档单轴幅值或三轴绝对值合计的最小值）；`unit_calibration_ratio_tolerance: 0.2`（比例相对误差 20%）；`unit_calibration_max_noise: 0.1`（单轴标准差/幅值最大 10%；合计比较使用标准差范数/三轴绝对值合计）。现在优先使用稳定单轴证据：Fx/Fy/Fz 任一轴两组读数都超过最小幅值、各组波动比例合格、保持同号且比值接近 9.80665 即可提供单位方向，不要求另外两个近零或噪声轴也有同比例。也接受 `abs(Fx)+abs(Fy)+abs(Fz)` 的两组均值合计接近此比例，避免正负抵消；不把力矩混入合计。有明确稳定信号的其他轴不能与所选单位证据矛盾，稳定力矩通道也按原有单位协议复核。容差均为可调整的初始配置。

本方法依赖设备 `35` 命令确实在 N/N·m 与 kgf/kgf·m 两档之间切换，。当主动/设备自行清零后只剩零点噪声时，不能据此保证识别成功。若比例不符，应核对实际协议和标准负载。`auto_calibrate_unit: false` 可保留旧人工单位声明方式，此模式没有自动验证。


## 设置启动和运行时目标单位

`target_unit` 可选 `N`、`kgf`（兼容 `kg`）或 `keep`。`keep` 表示保留前两组识别后得到的档位，不保证保持识别前的档位。启动时未传该参数就使用 USB/TCP YAML 的配置，默认 N；`unit` 是旧人工模式的输入声明，与自动模式目标 `target_unit` 不同。

```bash
# 自动识别并让硬件最终停在 N 档：
ros2 launch eli_cs_robot_serial sensor_tcp.launch.py target_unit:=N
# 自动识别并让硬件最终停在 kgf 档：
ros2 launch eli_cs_robot_serial sensor_tcp.launch.py target_unit:=kgf
# 已启动节点，运行时指定 kgf（不需重复启动节点）：
ros2 service call /sensor/set_unit eli_cs_robot_serial/srv/SetSensorUnit '{unit: kgf}'
# 运行时改回 N：
ros2 service call /sensor/set_unit eli_cs_robot_serial/srv/SetSensorUnit '{unit: N}'
```

`/sensor/set_unit` 返回成功仅表示请求已接收。它会重新进行两组单位识别，再根据目标决定是否需要额外切换；额外切换后再采 100 帧复核比例和实际档位，复核失败也不发布力数据。等待 `/sensor/unit_status` 中 `state=READY`、`ready=true`、`device_unit` 符合目标后才算完成。重新检测期间要求机械臂静止、稳定非零负载，且暂停控制用力输出；不要在机器人接近/接触任务执行中改单位。若前一次清零后读数接近零，应先准备稳定非零负载再调用服务。`auto_calibrate_unit: false` 的启动人工模式不自动设置目标档位；调用该服务仍会主动启动检测流程。

联合任务或检测任务中不额外启动第二个传感器节点；在原传感器 YAML 中设置 `target_unit`，或调用上述服务切换即可。不要仅通过 `ros2 param set` 改显示参数，它不会触发硬件切换流程。

日志中的 `basis` 给出实际证据轴（例如 `Fx`、`Fz`、`sum_abs(Fx,Fy,Fz)`），`axis_ratios` 为三轴各自的大/小倍数，`sum_abs_ratio` 为三轴绝对值合计的倍数。近零分母不用于比较。只出现单次大值或 2～3 倍变化不能据此认定 N/kgf；默认仍要求约 9.80665 倍及稳定性。实际切换后所有轴都几乎不变时仍拒绝，不通过放宽规则猜测单位。


### 大小与波动辅助候选

直接倍数检查失败时，同时输出两组每轴平均绝对值 `mean(abs(x))` 和标准差 `std(x)`。如果同一轴或三轴合计的大小、波动均沿相同方向明显增大，日志给出 `fallback_candidate_after=N`（切换后组更大）或 `kg`（切换前组更大），并列出 `fallback_basis`。为避免对几乎相同的数据强行排序，两项倍数均需至少 1.25，且差异超过按样本数估计的三倍统计误差；这是辅助筛选，不是设备精度或单位确认概率。其他轴给出相反候选时回到 unknown。

该候选不修改自动确认结果，状态仍为 FAILED/ready=false，且 `requires_manual_confirmation=true`，不发布控制用力。原因是换负载、触碰、不同滤波或噪声变化也可能同时改变大小和波动，不能证明设备切换了物理单位。在设备手册/标准负载确认当前档位后，可用已有人工模式 `auto_calibrate_unit: false` 配合 `unit: "N"` 或 `unit: "kg"`；不要仅凭候选就恢复机器人运动。两组都接近原数值时不给候选。


## 绘图数据来源

`sensor_plot.launch.py` 默认 `data_source:=auto`：单位尚未确认或识别失败时，显示 `/sensor/raw_values` 的原始六维读数，标题明确标记 `RAW / UNVERIFIED`，坐标轴单位为 raw。这只用于观察，原始话题不会被接入阻抗控制。单位确认且收到 `/sensor/wrench` 后自动切换到 N/N·m，并清空旧曲线，避免混画不同单位。超过 1 秒没有新数据时显示 No new data。

```bash
ros2 launch eli_cs_robot_serial sensor_plot.launch.py
# 持续观察设备原始读数，包括单位切换前后的变化：
ros2 launch eli_cs_robot_serial sensor_plot.launch.py data_source:=raw
# 仅观察已确认的 N/N·m 数据：
ros2 launch eli_cs_robot_serial sensor_plot.launch.py data_source:=verified
```

`/sensor/raw_values` 为 Float64MultiArray，依次 Fx、Fy、Fz、Mx、My、Mz，保留设备原始数值、未做软件单位换算。新增该话题后需重启传感器节点，旧进程不会自动增加发布者。绘图 CSV 保留前七列，并追加 source、device_unit、unit_state，区分原始/已确认单位数据。Ctrl+C 使用退出标记统一关闭 GUI，避免在 Tk 重绘回调中抛 KeyboardInterrupt。


### 每组采样前清零实验

`unit_calibration_zero_each_group: true`（当前默认）让首组、切换后的第二组及可选目标单位复核组均先执行 `30` 清零、等待 `zero_wait_ms`、清理缓存，再发送 `02` 开始采 100 帧。日志增加 ZERO_BEFORE、ZERO_AFTER、ZERO_VERIFY_TARGET；5 秒单组超时从开始采样计时，不包含清零等待。单位成功确认后仍按 `zero_before_request` 决定是否执行最终清零。

此选项用于按现场要求对比清零后的读数，不会把零噪声自动当成有效单位证据。固定工具重力可能被清零消除，仍可能 FAILED。设为 false 可恢复每组不清零的静载对比；原始曲线与均值、标准差日志继续保留。
