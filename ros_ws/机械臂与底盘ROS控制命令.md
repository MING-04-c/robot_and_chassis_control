# 机械臂与底盘 ROS 2 控制命令合集

适用当前工作区和 ROS 2 Jazzy。以下命令在树莓派执行，各种任务入口按需选择，不要同时启动重复的驱动、导纳或底盘控制节点。

## 1. 每个新终端先加载环境

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

直接加载以上两个 setup.bash 即可，不要求执行 `scripts/raspberry_pi_env.bash`。

修改代码后重新编译并加载：

```bash
colcon build
source install/setup.bash
```

当前默认机械臂 IP 为 `169.254.199.150`，力传感器为 `169.254.199.80:9001`。如果使用 `local_ip`，填写树莓派连接机械臂的实际网口地址，不要填写工控机地址。

## 2. 使用底盘前：配置和检查 CAN

在底盘任务启动前执行；CAN 已由开机服务正常配置时，可先检查状态，不必重复配置。

```bash
sudo bash scripts/setup_can.sh auto
```

明确使用 SocketCAN 的 can0、500 kbit/s 时：

```bash
sudo bash scripts/setup_can.sh socketcan --iface can0 --bitrate 500000
```

以上两种配置方式选一种。检查状态和底盘反馈：

```bash
bash scripts/setup_can.sh status --iface can0
bash scripts/setup_can.sh test --iface can0
```

ROS 任务 launch 不会替你执行 `setup_can.sh`。仅机械臂任务不需要 CAN。

## 3. 机械臂：整套启动并自动进入导纳

```bash
ros2 launch robot_task_control arm_admittance_only.launch.py \
  robot_ip:=169.254.199.150 sensor_host:=169.254.199.80
```

包含机械臂驱动、MoveIt、力传感器和导纳控制。驱动就绪后默认连续稳定等待 5 秒，再加载控制部分；控制器、外控和新鲜力数据就绪后，自动以当前位置进入柔顺控制。无需另外调用 enable 服务。默认不启动底盘和相机。

需要查看模型时，在启动命令末尾添加：

```text
launch_rviz:=true
```

## 4. 机械臂：驱动和导纳分开启动

此方式与第 3 节二选一。

终端 1：只连接机械臂驱动。

```bash
ros2 launch eli_cs_robot_driver elite_control.launch.py \
  robot_ip:=169.254.199.150 headless_mode:=true launch_rviz:=false
```

终端 2：加载环境后，检查驱动和外控。

```bash
ros2 control list_controllers
ros2 topic echo /io_and_status_controller/robot_task_running --once
```

`robot_task_running` 应为 `true`；控制器 active 不能单独证明外控正在运行。

确认驱动正常后，终端 2 启动导纳：

```bash
ros2 launch eli_cs_robot_usr compliant_control_real.launch.py \
  sensor_host:=169.254.199.80 auto_start_admittance:=true launch_rviz:=false
```

该入口加载控制部分和力传感器，复用终端 1 的驱动，就绪后自动进入导纳。若已单独启动力传感器，添加 `launch_sensor:=false`。

## 5. 单独循迹（寻迹 / xunji，仅底盘）

独立入口是 `line_follow_only.launch.py`，包含底盘驱动、USB 摄像头、识别和循迹控制，不启动机械臂、力传感器或海康相机。

先完成第 2 节 CAN 配置。

终端 1：启动底盘、USB 相机、识别、循迹及速度选择器。

```bash
ros2 launch robot_task_control line_follow_only.launch.py \
  line_camera_device:=/dev/video0 \
  line_show_window:=true line_follow_speed:=0.075
```

终端 2：加载环境后开始任务。

```bash
ros2 service call /task_controller/start std_srvs/srv/Trigger '{}'
```

独立循迹按每帧图像更新可行范围：跟随近端实测线段，短缺口只有两端实测段的位置和切向吻合，或延长线在缺口内部相交时，才按同一条线续接。前方累计缺口默认最多图高30%，已确认续接默认不额外降低速度；单端、错位或过大缺口截断。可信段缩短时减速，视觉不足时立即零速等待，最多5秒内恢复则继续，未恢复才结束本轮。光线变好、实测段延长就继续推进。默认本轮总超时90秒，超时也停车；可在 `config/line_follow_only.yaml` 修改。设备路径按实际修改；关闭预览用 `line_show_window:=false`。缺口参数位于 `src/masc_chassis_user_control/config/line_follow.yaml`，橙线 `GAP: JOINED` 标记已确认续接的缺口。

窗口红色叉标记当前连续实测段末端，不是实际米制刹车位置。图像中的距离尚未做地面标定；缺线阈值和减速仍需实车低速验证。5秒内恢复不需要再次 start；本轮已结束后，再次调用 start 才开始新一轮；曝光节点继续运行。联合入口仍保持原来的有效循迹 2 秒测试配置。

反光缺口两侧各需至少图高10%的连续实测线段支撑。近端跨度门槛现为图高5%，有效比例门槛为6%，允许短近端线段继续低速推进；这些是图像比例阈值，不是置信概率。

两段延长线在缺口内部相交时，也可作为同一条线低速续接，转角默认不超过约34度。
上游 `ruier_camer_diver/config/line_tracker.yaml` 的 `extended_join_*` 和循迹参数配套生效，
保留两段真实检测，不把中间缺口涂成已观测蓝线。交点在缺口以外或仅有单端不放行。

已参照 PDF 加入局部明度补偿、自适应霍夫线段方向拟合，以及控制端的多点稳健拟合。
处理逻辑和开关说明见 [PDF 参考改动](docs/chassis/LINE_FOLLOW_REFERENCE_20261029.md)。

窗口第三行 INPUT 显示当前输入状态，LAST BLOCK 保留上次阻塞原因，v 为实际发出的前进速度。`CONTROL: disabled / INPUT: tracking` 表示图像输入可行但任务未使能，可用 `/line_follow/status` 查看同名诊断字段。等待时长通过 `config/line_follow_only.yaml` 的 `boundary_wait_sec` 调整。

## 6. 机械臂导纳＋底盘循迹联合任务

先完成 CAN 配置，退出前面独立启动的重复节点后执行：

```bash
ros2 launch robot_task_control arm_admittance_line_follow.launch.py \
  robot_ip:=169.254.199.150 sensor_host:=169.254.199.80 \
  line_camera_device:=/dev/video0 \
  line_show_window:=true line_follow_speed:=0.075
```

包含机械臂驱动、MoveIt、力传感器、导纳、底盘驱动和 USB 循迹。不需要海康相机。默认流程：

```text
驱动就绪 → 连续稳定等待 5 秒 → 加载控制部分
→ 机械臂、力数据、底盘和循迹图像就绪检查
→ 当前位置导纳保持 2 秒 → 底盘累计有效循迹 2 秒
→ 底盘停车，机械臂继续保持柔顺
```

默认自动开始，不用再调用 start。需要先就绪、后手动开始时，启动命令添加 `auto_start:=false`，再使用第 5 节的 start 命令。

若第 4 节的机械臂驱动和导纳都已经正常运行，保留那两个终端，使用以下命令复用它们：

```bash
ros2 launch robot_task_control arm_admittance_line_follow.launch.py \
  start_arm_driver:=false start_moveit:=false \
  line_camera_device:=/dev/video0 line_show_window:=true
```

## 7. 底盘键盘控制

先退出其他底盘任务会话，避免多个控制源。完成 CAN 配置后，在交互终端运行：

```bash
ros2 launch robot_task_control keyboard_teleop.launch.py \
  linear_speed:=0.05 angular_speed:=0.2
```

等待 `KEYBOARD READY` 后操作：

| 按键 | 动作 |
|---|---|
| W / S | 前进 / 后退 |
| A / D | 左 / 右横移 |
| Q / E | 原地转向 |
| Z / C | 后退转弯 |
| 空格 / X | 停车 |
| + / - | 调速 |
| Ctrl+C | 停车并退出 |

默认 0.35 秒未收到移动按键就输出零速度。已有独立底盘驱动且没有其他控制源时，可添加 `start_driver:=false`。

## 8. 检测到达流程测试

### 模拟硬件测试

```bash
ros2 launch robot_task_control detection_test.launch.py
```

默认启动 mock 机械臂、MoveIt、导纳、检测控制器和 RViz，使用独立 ROS 域 186。它是模拟硬件测试，不是 Gazebo 物理仿真。

另一个终端查看状态：

```bash
ROS_DOMAIN_ID=186 ros2 topic echo /detect_controller/status
```

### 真机检测测试

**以下命令就绪后会自动执行真机运动。** 默认包含驱动、控制部分和力传感器，不需要提前运行第 3、4 节。

```bash
ros2 launch robot_task_control detection_test.launch.py \
  simulation:=false robot_ip:=169.254.199.150 sensor_host:=169.254.199.80
```

当前测试配置：base_link 下的 tool0 初始位置为 `[-0.4, 0.1, 0.4]` m，检测位置为 `[-0.3, 0.1, 0.177]` m，保持启动时朝向。先到目标上方，再沿基座 −Z 导纳接近；到目标后柔顺保持，达到 20 N 后保持 5 秒、撤离、返回初始关节位置。未达到阈值时继续等待接触。

已有完整机械臂控制系统时复用：

```bash
ros2 launch robot_task_control detection_test.launch.py \
  simulation:=false start_arm_driver:=false start_arm_control:=false
```

初始点、检测点、角度限制和接触力配置在 [detection_test.yaml](src/robot_task_control/config/detection_test.yaml)。以上检测入口不启动底盘。

## 9. 力传感器单独测试

仅在没有其他节点启动力传感器时执行：

```bash
ros2 launch eli_cs_robot_serial sensor_tcp.launch.py target_unit:=N
```

另一个终端绘图：

```bash
ros2 launch eli_cs_robot_serial sensor_plot.launch.py
```

查看控制用力数据：

```bash
ros2 topic echo /sensor/wrench
```

单位识别失败时，控制话题可能暂停；绘图 auto 模式可显示原始观察数据。具体失败原因看传感器日志，不要把原始读数直接当成已确认的 N。

## 10. 状态查看与任务停止

### 厚度仪读数和 TCP 调用

真机 `detection_test.launch.py` 和 `detect_control/detect.launch.py` 现在默认启动测厚仪，关闭周期轮询，仅在接触保持开始时触发一次。已有测厚驱动时添加 `start_thickness_detector:=false`，避免重复占用串口；模拟检测不启用真实厚度仪。串口默认 `/dev/ttyCH341USB0`，在 `src/eli_cs_robot_serial/device/detactor/config/detactor.yaml` 核对设备路径，并把 `measurement_unit` 按仪器实际设置改为 `um` 或 `mm`；未确认时为 `unknown`，不自动换算。

单独启用厚度仪（已有驱动时不要重复执行）：

```bash
ros2 launch eli_cs_robot_serial detactor.launch.py auto_request:=false
```

手动请求一次读数，再查看最新有效回包：

```bash
ros2 service call /detactor/acquire std_srvs/srv/Trigger '{}'
ros2 topic echo /detactor/measurement
```

`acquire` 成功仅表示采集请求已排队发送，实际读数以回包为准。读取缓存、不发起新采集：

```bash
ros2 service call /detactor/latest_measurement std_srvs/srv/Trigger '{}'
```

任务每个点的厚度结果：

```bash
ros2 topic echo /detect_controller/thickness_result
```

保持结束时结果追加保存至 `~/.ros/detect_control/thickness_results.jsonl`。没有本次新回包时明确记录未获得读数，不复用上一点数据。接收时间只表示串口回包时间，设备返回历史窗口时，无法据此证明探头刚完成一次新的物理测量。

TCP 通讯节点使用现有入口启动（需要已准备好的双方共享密钥）：

```bash
ros2 launch eli_cs_robot_serial industrial_pc.launch.py
```

工控机使用更新后的客户端查询最新厚度：

```bash
python3 industrial_pc_client.py thickness \
  --host 树莓派实际IP --port 9010 --key-file industrial_pc.key
```

返回 `latest_thickness`：`available`、单值 `measurement.value`、单位、序号、接收时间及 `age_seconds`。尚未收到有效回包时 `available:false`，不发送默认零值；重复查询不触发测量。最新缓存没有自动过期删除，主机按时间判断能否用于当前检测。发送检测点结果时，ACK 也会附带此刻最新厚度，不发送历史十值数组。此接口返回仪器最新回包，不保证属于主机当前提交的检测点；逐点关联以任务结果为准。

以下命令各自持续运行，可分终端查看，Ctrl+C 退出查看：

```bash
ros2 node list
ros2 control list_controllers
ros2 topic echo /chassis/task_status
ros2 topic echo /line_follow/status
ros2 topic echo /detect_controller/status
```

停止第 3、5、6 节的工作流：

```bash
ros2 service call /task_controller/stop std_srvs/srv/Trigger '{}'
```

退出整套启动会话，在对应 launch 终端按 Ctrl+C。任务 stop 不等于硬件急停，也不会退出驱动节点；检测任务使用自己的控制器，不使用上述工作流 stop 接口。

## 11. 常用参数与配置位置

| 参数 / 配置 | 用途 |
|---|---|
| `robot_ip:=...` | 机械臂地址 |
| `local_ip:=...` | 树莓派机械臂通信网口地址，用于支持该参数的驱动/任务入口 |
| `sensor_host:=...` | 力传感器地址 |
| `arm_control_start_delay:=5.0` | 驱动连续就绪后的稳定等待时间 |
| `launch_rviz:=true` | 在支持该参数的机械臂入口显示模型 |
| `line_camera_device:=/dev/video0` | USB 循迹相机设备 |
| `line_follow_speed:=0.075` | 循迹速度，m/s |
| `config/arm_admittance_only.yaml` | 仅导纳流程 |
| `config/arm_admittance_line_follow.yaml` | 导纳＋循迹流程与持续时间 |
| `config/line_follow_only.yaml` | 仅循迹流程与持续时间 |

表中相对配置路径位于 `src/robot_task_control/`。修改源码配置后重新构建并加载环境。某个入口的完整参数可用下列方式查看，不会启动设备：

```bash
ros2 launch robot_task_control arm_admittance_line_follow.launch.py --show-args
ros2 launch robot_task_control detection_test.launch.py --show-args
```

## 循迹后机械臂四点联动测试

[正式测试流程](docs/tasks/LINE_FOLLOW_ARM_TEST.md)：先停稳并到初始点，再循迹：途中横线停车等待开始消息，最终 T 形停止横线居中后停稳，再依次走第二、第三、第四点；短暂丢线等待恢复，未见最终横线时不能当成正常完成。包含驱动，不要与已有驱动重复启动。CAN 按第 2 节先配置。

```bash
ros2 launch robot_task_control line_follow_arm_test.launch.py launch_rviz:=true
```

设备就绪后自动执行，无需调用 start 服务。

位置以 base_link/tool0 为准，单位米；RPY 角度单位度。修改 `src/robot_task_control/config/line_follow_arm_test.yaml` 后重建、重启。

## 力传感器单位初始化

厂家确认重新上电默认 kg 档。现在启动传感器自动切换一次到 N，正常退出自动切回 kg，不再比较 100+100 帧；导纳和检测任务复用同一默认配置。传感器刚上电或上次正常退出后，直接启动对应任务即可。发生异常退出或中途断线后，先重新给传感器上电再重启节点，不能把网络重连当作硬件重新上电。[详细说明](src/eli_cs_robot_serial/device/sensor/README.md)。

## 途中停止点继续循迹

launch 首次仍自动开始。十字形途中站居中停车后发一次新消息，才继续走；提前发消息或最终 T 形站点不会恢复：

```bash
ros2 topic pub --once /line_follow/resume std_msgs/msg/Empty '{}'
```

停止目标是单帧相机画面高度50%，默认容差图高1%；近端自动减速。查看 `ros2 topic echo /line_follow/status` 中 `stop_marker` 的类别和位置误差。[正式任务说明](docs/tasks/LINE_FOLLOW_ARM_TEST.md)。
