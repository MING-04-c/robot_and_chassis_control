# 机械臂、底盘与主机通讯命令

适用于当前 ROS 2 Jazzy 工作区。每次选择一个任务入口，避免重复启动同一驱动、相机或 TCP 端口。

## 1. 环境与编译

每个树莓派终端先执行：

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

修改源码配置后：

```bash
colcon build --packages-select robot_task_control
source install/setup.bash
```

不需要额外 source `scripts/raspberry_pi_env.bash`。默认地址：机器人 `169.254.199.150`，力传感器 `169.254.199.80:9001`，树莓派 `169.254.199.100`，麒麟 `169.254.199.120`。

单独连接真机时，`ros2 launch eli_cs_robot_driver elite_control.launch.py` 已默认自动完成 dashboard 上电、待机、释放抱闸、确认 RUNNING，再启动控制器；不再需要另开终端调用 power_on/brake_release。原有 IP、型号参数不变；仿真硬件不执行上电操作。只需连接、暂不自动上电时可加 `auto_power_on:=false`。

## 2. 入口速查

以下均在树莓派执行，命令前缀是 `ros2 launch robot_task_control`。

| 文件名 | 功能 | 是否自动开始 |
|---|---|---|
| arm_admittance_only.launch.py | 机械臂导纳 | 是 |
| line_follow_only.launch.py | 前进/后退循迹 | 就绪后自动 |
| keyboard_teleop.launch.py | 底盘键盘 | 按键控制 |
| line_follow_arm_test.launch.py | 整体联动：循迹、四点运动、末尾拍照发送 | 默认是；remote_control:=true 等主机 start |
| camera_capture_remote.launch.py | 只采图发送 | 否，主机发 capture |
| camera_detection_real.launch.py | 相机点限力测试：默认TCP，可选local | TCP收到本次结果后自动；local就绪后自动 |
| pixel_point_simulation.launch.py | 像素点换算与触碰仿真 | 是 |

六个常用入口放在 `launch/`，像素仿真放在 `launch/tests/`。`launch/internal/` 的 robot_system 和 detection_system 是共用驱动栈。旧 remote 入口已合并为整体联动的参数，旧两秒导纳＋循迹流程及历史综合示例已移除。

## 3. 底盘 CAN、循迹和键盘

本次开机尚未配置 CAN 时先执行一次；launch 不会代为运行：

```bash
sudo bash scripts/setup_can.sh auto
bash scripts/setup_can.sh status --iface can0
```

如需指定 SocketCAN：

```bash
sudo bash scripts/setup_can.sh socketcan --iface can0 --bitrate 500000
```

单独循迹：

```bash
ros2 launch robot_task_control line_follow_only.launch.py \
  line_camera_device:=/dev/video0 line_show_window:=true line_follow_speed:=0.075
```

倒车原路返回：先执行上面的前进循迹，到站锁停并实际停稳，看到“前进路线已停稳保存”后关闭旧 launch，保持小车位置、朝向不变，再启动：

```bash
ros2 launch robot_task_control line_follow_only.launch.py drive_direction:=reverse line_show_window:=true
```

前进会自动把实际里程计路线保存到 `~/.ros/line_follow/last_route.json`（至少10cm，结束并停稳后标记为有效；行驶中草稿不可用于倒车）。倒车默认用记录路线控制车后轨迹，在可信直线段加入小幅视觉横移修正，同时通过前置相机横线暂停；图像丢线不当作正常倒车结束。默认7.5cm/s，上限8cm/s，`line_follow_speed:=0.04` 可调慢；剩余60cm内连续确认近端可纠偏、剩余路径为小弯/直线且转向速率稳定后切相机找中心；未满足条件时最后30cm减速对齐，不按距离强制接管（最多3cm/s），末段未居中时先停纵向、横移回中并旋转摆正，连续3帧合格后再倒向横线；横线到位且对齐确认通过才完成返程。驱动重启归零时会将记录终点对齐当前里程计，前提是中间没有移动小车。默认 `route_visual_strafe=true`，横移上限2.5cm/s、旋转上限0.06rad/s；横向偏差不再用倒车画弧线追回。`reverse_steering_sign` 仅在旧相机模式整体反号，不影响前段记录路线追踪。

两个命令可用同一个 `route_file:=/绝对路径/route.json` 指定文件。倒车状态为 `reverse_route_tracking` / `reverse_visual_finish_aligning`（横移/摆正，纵向已停）/ `reverse_route_complete`，剩余距离见 `/line_follow/status`；路线可通过 `/line_follow/recorded_path` 在 RViz 查看。没有有效路线、里程计超时/跳变、偏离超限时停车。默认回程在新横线处居中锁停，等待继续消息；视觉阶段中剩余30cm内的横线按终点处理，不再等待继续；里程计到起点而未见横线时最多再后退10cm，超限故障停车。相机配对数据超时也暂停。里程计会有漂移，不能据2cm判定容差承诺实车精度。


倒车横线暂停后继续（工控机发送 `resume`，不是重复启动任务）：

```bash
ros2 topic pub --once /line_follow/resume std_msgs/msg/Empty '{}'
```

同一出发横线会去重。仅回归原里程计倒车可在启动命令增加 `route_camera_stop:=false`；默认启用，不额外启动摄像头进程。

默认驱动和视觉就绪后自动开始，不需要调用 start 服务。若只想预览、暂不行走，启动时加 `auto_start:=false`。

途中站继续：

```bash
ros2 topic pub --once /line_follow/resume std_msgs/msg/Empty '{}'
```

十字形中途站停车后才接受继续；最终 T 形站不恢复。到位状态锁定，后续漏检和图像超时不会把已到站改回寻找横线；途中站继续仍需当前输入有效且横线可见。单独循迹停车位置默认图高46%，位置允许提前图高3%或越线6%（480高约提前14px、越线29px），三帧平均、两次有效到位确认；改为正中可传 `line_stop_marker_target_ratio:=0.5`。窗口 `CONTROL: disabled` 时先检查任务是否启动；`LAST BLOCK` 是上一次阻塞原因。[循迹逻辑说明](docs/chassis/LINE_FOLLOW_REFERENCE_20261029.md)。

底盘键盘：

```bash
ros2 launch robot_task_control keyboard_teleop.launch.py linear_speed:=0.05 angular_speed:=0.2
```

W/S 前后，A/D 横移，Q/E 转向，空格/X 停车，Ctrl+C 退出。已有驱动时可加 `start_driver:=false`。

## 4. 机械臂导纳与联动

整套驱动、MoveIt、力传感器和导纳：

```bash
ros2 launch robot_task_control arm_admittance_only.launch.py
```

就绪后在当前位置自动进入柔顺，不需要调用 enable。连接连续就绪后默认再等5秒启动控制部分。可加 `robot_ip:=... sensor_host:=... launch_rviz:=true`。

驱动与控制分开启动时，两个终端分别执行：

```bash
ros2 launch eli_cs_robot_driver elite_control.launch.py \
  robot_ip:=169.254.199.150 headless_mode:=true launch_rviz:=false
```

```bash
ros2 launch eli_cs_robot_usr compliant_control_real.launch.py auto_start_admittance:=true launch_rviz:=false
```

这与整套入口二选一。已有力传感器节点时给控制入口加 `launch_sensor:=false`。检查连接：

```bash
ros2 control list_controllers
ros2 topic echo /io_and_status_controller/robot_task_running --once
```

外控必须为 true，控制器 active 不能单独证明可以运动。

循迹与四点机械臂、末尾采图发送：

```bash
ros2 launch robot_task_control line_follow_arm_test.launch.py launch_rviz:=true
```

先停稳并到第一点，循迹至最终横线，依次执行后续点，再采集发送 HK 和 RGB/深度。坐标/度制 RPY 在 `config/line_follow_arm_test.yaml`，相机稳定性与传输参数在 `config/capture/final_capture.yaml`。详细步骤见[正式流程](docs/tasks/LINE_FOLLOW_ARM_TEST.md)。主机发开始信号时，在同一命令后加 `remote_control:=true`。

## 5. 与麒麟主机通讯：完整步骤

通讯代码在 `src/eli_cs_robot_serial/device/industrial_pc/`；项目根目录 `industrial_pc_kylin_tcp.zip` 为可复制的麒麟工具包。默认无密钥，两端使用同一版本，麒麟用 Python3 执行即可，不依赖 ROS。

| 通道 | 地址/端口 | 用途 |
|---|---|---|
| 控制与结果 | Pi 169.254.199.100:9010 | 主机发 start/stop/resume/capture、点位，查询状态和最新厚度 |
| HK | Pi 169.254.199.100:9002 | 主机作为客户端接收 Pi 照片 |
| RGB/深度 | 麒麟 169.254.199.120:9030 | 主机监听，Pi 上传采集包 |

### A. 主机发开始信号，执行完整联动任务

树莓派先完成 CAN 配置，再启动：

```bash
ros2 launch robot_task_control line_follow_arm_test.launch.py remote_control:=true
```

包含驱动、工作流和9010通讯，等待主机开始，不自动运动。

麒麟进入解压后的工具目录，终端一持续运行：

```bash
cd /实际路径/industrial_pc_kylin_tcp
python3 listen_robot.py
```

终端二发开始，按需要执行以下某一条：

```bash
python3 start_task.py
python3 start_task.py --action status
python3 start_task.py --action resume
python3 start_task.py --action stop
```

`start` 开始已加载任务，`resume` 只继续途中停车站，`stop` 停止任务但不退出驱动。网络回执表示请求受理，完成与否看监听窗口的 phase/状态。若回执为执行结果未知，先查询状态；重发同一命令用 `--message-id 原编号`，不要换新ID重复启动。

### B. 主机只请求采图，不运动

如果 A 已运行且任务空闲，直接在麒麟发 `python3 start_capture.py`，不用额外开入口。

只测试相机时，树莓派改用：

```bash
ros2 launch robot_task_control camera_capture_remote.launch.py
```

麒麟终端一保持 `python3 listen_robot.py`，终端二：

```bash
python3 start_capture.py
```

该入口不加载机械臂或底盘驱动。顺序等待稳定、拍一张 HK、关闭 HK，再拍 RGB/原始深度、关闭深度相机，最后发送。A/B 不能同时启动，不要另开相机预览或常驻9002发送节点。

照片位置：Pi `~/robot_data/task_final/2026.10.30.20.29/`；麒麟 `工具目录/received/2026.10.30.20.29/`（年月日时分示例）。下面直接分 `hk/` 和 `RGB/`：`hk/20.29.12_345_hk.jpg`，`RGB/20.29.12_345_rgb.png`、`RGB/20.29.12_345_depth.png`。同一分钟多次采集按秒/毫秒文件名前缀区分，不增加目录层级；原始16位深度不转成8位，内参、比例与关联哈希集中在隐藏 `.capture_index.json`。

### C. 主机返回检测点

```bash
python3 send_points.py --file /实际路径/result.json
```

正常真实结果使用 `paint_v54_pi_remote_result_v1`，光学坐标统一毫米，关联本次 `capture_manifest_sha256`。旧照片/演示数据不能当成当前真实测量。[协议与格式](../industrial_pc_kylin_tcp/TCP通讯协议_麒麟工控机.md)。

通讯桥本身只保存和发布观测。要让探针触碰，使用第6节 `camera_detection_real` 默认TCP模式：它采新图、记录位姿并等待本次匹配结果，收到后自动检测。四点联动remote入口不自动触碰回传点。

只收点、查询状态或厚度，不运行任务时，树莓派可单独启通讯：

```bash
ros2 launch eli_cs_robot_serial industrial_pc.launch.py start_hk_transfer:=false
```

该入口不加载工作流，单独运行时主机 start/capture 会提示对应服务未就绪。A/B 已包含它，不要重复启动。查看接收结果：

```bash
ros2 topic echo /industrial_pc/result
ros2 topic echo /industrial_pc/status
```

### D. 通讯排查

麒麟：

```bash
python3 industrial_pc_network_test.py interfaces
python3 industrial_pc_network_test.py connect --host 169.254.199.100 --ports 9010
python3 start_task.py --action status
```

Pi：

```bash
ss -lntp | rg '9010|9002'
```

9010应随通讯节点监听；9002只在采图发送 HK 时出现，平时未监听正常。9030由麒麟的监听脚本提供。权限不足时用 `python3 文件名.py`，不必依赖可执行位；不要把目录名当成脚本。

## 6. 相机点真机限力测试：默认接收 / 本地给点

**两模式都可能自动执行真机运动。** 先退出重复驱动、相机与TCP程序，把机械臂放在拍照姿态，探针未接触。

默认TCP模式，麒麟先运行 `python3 listen_robot.py`，树莓派运行：

```bash
ros2 launch robot_task_control camera_detection_real.launch.py
```

包含驱动、MoveIt、真实力传感器、导纳、检测器和TCP。就绪后自动稳定拍HK/RGB/深度并发送，冻结拍照位姿，等待主机 `python3 send_points.py --file 本次结果.json`。匹配本次清单哈希后自动换算并触碰，无需start_task或start_capture。过期批次、DEMO和HOLD不会启动触碰；等待失败不回退本地点。

直接使用YAML点位，保持之前的本地测试：

```bash
ros2 launch robot_task_control camera_detection_real.launch.py target_source:=local
```

local不启动相机/TCP，读取 `config/camera_detection_real.yaml` 的 `camera_points_mm`，相对于启动时相机光学坐标，保持当前探针朝向。两种模式共用：到上方20mm、沿基座−Z以2mm/s接近、20N停止推进、保持5秒、逐点返回原位；不是恒力20N伺服。探针触点是terminal_link_2。

已有驱动可加 `start_arm_driver:=false`，已有完整控制栈再加 `start_arm_control:=false`。默认不开测厚仪，需厚度读数可加 `start_thickness_detector:=true`。具体步骤、主机像素点发送和参数见[两模式检测说明](docs/tasks/CAMERA_DETECTION_REAL.md)。

假硬件仿真：

```bash
ros2 launch robot_task_control pixel_point_simulation.launch.py
```

使用独立域191和模拟接触力，不连接真机。[仿真说明](docs/tasks/PIXEL_POINT_SIMULATION.md)。旧到达测试已收进 `internal/detection_system.launch.py` 共用驱动栈；真机触碰统一使用上面的两模式入口。

## 7. 相机、力传感器与厚度仪单独调试

不要与已包含同一设备的任务同时启动。

海康预览：

```bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py show_image:=true
```

深度相机预览：

```bash
ros2 launch geminipro_camer_calibration gemini_pro_view.launch.py
```

力传感器与绘图，分别在两个终端运行：

```bash
ros2 launch eli_cs_robot_serial sensor_tcp.launch.py target_unit:=N
ros2 launch eli_cs_robot_serial sensor_plot.launch.py
```

厂家确认硬件重新上电默认kg档；当前默认启动切换一次到N，正常退出切回kg。异常退出或断线后先给传感器重新上电，不能把网络重连当成重新上电。控制话题 `/sensor/wrench` 是SI单位，原始绘图数据不能直接当N。[传感器说明](src/eli_cs_robot_serial/device/sensor/README.md)。

厚度仪单独启动、请求一次并读缓存：

```bash
ros2 launch eli_cs_robot_serial detactor.launch.py auto_request:=false
ros2 service call /detactor/acquire std_srvs/srv/Trigger '{}'
ros2 topic echo /detactor/measurement
ros2 service call /detactor/latest_measurement std_srvs/srv/Trigger '{}'
```

以上后续命令在另一个加载环境的终端运行。仪器单位在串口包 `device/detactor/config/detactor.yaml` 设置，未确认时是unknown；请求受理不表示已经收到新读数。

检测每点接触保持开始时触发厚度采集，结果发布 `/detect_controller/thickness_result`，保存到 `~/.ros/detect_control/thickness_results.jsonl`。没有本次新回包就明确记录缺失。麒麟查询 Pi 缓存的最新单值（不触发采集）：

```bash
python3 industrial_pc_client.py thickness --host 169.254.199.100 --port 9010
```

默认无需 `--key-file`。返回值含单位、序号和 age_seconds；最新缓存不保证属于当前检测点，逐点关联看检测结果。

## 8. 查看状态与退出

各查看命令分终端运行：

```bash
ros2 node list
ros2 control list_controllers
ros2 topic echo /task_controller/status
ros2 topic echo /line_follow/status
ros2 topic echo /detect_controller/status
```

工作流任务停止：

```bash
ros2 service call /task_controller/stop std_srvs/srv/Trigger '{}'
```

本地或旧检测任务取消：

```bash
ros2 service call /detect_controller/cancel std_srvs/srv/Trigger '{}'
```

在启动 launch 的终端按 Ctrl+C 退出整套节点，RViz窗口的 Ctrl+C 不会关闭启动终端。当前本地检测退出先取消、有限等待，再清理TF；卡住的子进程由launch升级终止。进程退出不等于停止已确认，仍运动时用示教器停止。

125Hz告警表示控制循环超过8ms，是否因此停下须看外控状态和检测器的FAULT原因；到目标且未达到20N时，`COMPLIANT_HOLD` 会维持柔顺等待，不再主动向下穿过目标。

常用参数无需运行设备即可查看：

```bash
ros2 launch robot_task_control camera_detection_real.launch.py --show-args
ros2 launch robot_task_control line_follow_arm_test.launch.py remote_control:=true --show-args
```

文中 `config/` 相对路径均位于 `src/robot_task_control/`。配置目录按常用流程、capture和tests分开；旧remote入口统一为 `line_follow_arm_test.launch.py remote_control:=true`。

循迹正常前进设定仍为 0.075 m/s。实测近端已覆盖前瞻点时，不因更远处断线或重复的疑似遮挡标记额外降速。停止横线尚远、主线有效时，确认过的横线漏检最多以 0.008 m/s 缓行 0.6 秒，并受同次漏检5mm速度指令积分限值约束；接近停车容差、持续漏检或主线丢失仍停车。相关参数在 `masc_chassis_user_control/config/line_follow.yaml` 的 `stop_marker_dropout_grace_sec`，设为 0 关闭短时缓行。

直线回中：默认允许半图宽4%的偏差（640宽约13px）；连续5张新图持续偏向同一侧并超过约19px时，允许小幅转向慢慢回中。弯道中心死区仍放宽至半图宽30%，无需另调开关或调用服务。查看日志 `centering=true` 可确认持续偏移纠正正在生效；这不改变停车横线的到位容差。

记录路线倒车已取代旧的前置相机倒车；旧横线离站与反号参数不参与默认回程控制。

横线漏停排查：`/line_follow/status` 的 `stop_detection` 显示长线候选数量及拒绝原因；移动时5秒一次的 `Steering` 日志含 `stop_candidates`、`stop_rejected`。短横臂合并已撤回；转向平滑及到位锁停保持原有设置。

倒车最后直线默认 `route_visual_finish:=true`，进入时日志显示“进入最后相机倒车”。前段横线停车仍用 `resume` 继续；最后视觉阶段要求横线到位且连续3帧对齐确认通过才报 `reverse_route_complete`；未对齐仍可横移/摆正，不会继续纵向越线。若报 `reverse_finish_search_limit` 表示超过允许的找线范围，并非正常完成。只关闭最后视觉纠偏可传 `route_visual_finish:=false`，此时保留相机途中横线保险并按里程计起点终停。

前进横线也执行目标保持和提前零速确认：已经发现的停车目标漏检时不换追下一条，最新观测进入容差便先停稳等多帧确认。途中站仍等 `resume`，最终站结束；不改变前进巡航速度及弯道控制参数。

启动前进时已经停在横线上，会自动略过当前这根出发横线并行走，不用发送 `resume`；下一条横线仍正常停车。运行途中已停车的站点继续仍使用 `/line_follow/resume`。启动时尚在远处的横线不属于出发线，会正常靠近停车。

前进停在途中横线时，即使任务仍启用并等待继续，实际反馈速度归零后也自动保存有效返回路线，日志显示“前进路线已停稳保存”。发送继续消息前会立即把快照恢复为行驶中无效状态。退出ROS时保留已确认的停稳快照，不因时钟已关闭或最后一帧超时清除有效标记；未确认停稳的记录不会因此升级为可返回。

### 循迹站点记录

每次任务按行驶顺序编号横线，终端和画面显示识别、忽略、到站、通过数量；途中站继续后保留计数，同一根横线漏检后重新出现不会重复登记。当前计数及历史见 `/line_follow/status`，新事件见 `/line_follow/station_events`。本轮日志保存到 `~/.ros/line_follow/last_station_log.json`；前进路线同时记录站点事件，下一轮倒车可查看前进站点逆序。旧路线没有这部分记录时，不能从旧文件恢复场地总横线数，需要下一轮前进采集。途中站仍等待新开始/继续消息，增加记录不改变停车规则。

倒车地面映射可选配置：`masc_chassis_user_control/config/line_follow.yaml` 的 `reverse_ground_mapping_file` 指定实测对应点JSON。为空时界面及状态标明 `image_uncalibrated`，只按图像误差伺服；有实测映射时显示 `ground_meters`。文件格式、实测坐标要求见该功能包 README 的“可选实测地面映射”，勿直接使用示例坐标。

## 自动前进循迹、执行中间任务、倒车返回

```bash
ros2 launch robot_task_control line_follow_only.launch.py round_trip:=true line_show_window:=true
```

启动后自动前进；途中横线仍等待开始/继续消息，最终横线才结束前进。任务随后确认底盘停稳、封存路线、执行中间步骤，最后切换倒车；返程途中站自动继续，只在原出发点完成。路线只记录本轮前进，不把中间等待或机械臂/TCP处理追加进去。

流程在 `src/robot_task_control/config/line_follow_round_trip.yaml`，当前中间步骤为2秒等待。后续把已有机械臂/采集/TCP动作插入 `save_line_route` 与 reverse步骤之间即可，必须等待动作成功完成才开始倒车。该测试入口没有启动机械臂和TCP进程。不要在中间重启底盘驱动、清零里程计或另外启动前进录制。

封存文件保留在 `~/.ros/line_follow/snapshots/last_route_*.json`，本轮路径也发布到 `/task_controller/status` 的 `route_checkpoint_file`。即使后续启动新一轮前进、覆盖 `last_route.json`，已封存副本仍保留。发现等待期间底盘被移动或里程计不连续时，会拒绝自动套用旧路线。
