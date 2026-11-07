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
| line_follow_arm_test.launch.py | 整体联动：行驶位、逐站循迹、机械臂四点、中转等待 | 默认是；remote_control:=true 等主机 start |
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

真机驱动默认带公共保护停止恢复器。报警后保持原 launch，移除外力并确认现场安全，调用一次：

```bash
ros2 service call /protective_stop_guard/confirm_recovery std_srvs/srv/Trigger '{}'
```

它会执行一次解锁、恢复外控并重新准备已保存的单点机械臂步骤。独立柔顺恢复时捕获当前姿态；检测接触、循迹、多路点和采集发送步骤不自动重放。失败保持暂停，不自动反复解锁。恢复前需连续停稳1秒，导纳去零偏合力低于5N。查看进度及取消：

```bash
ros2 topic echo /protective_stop_guard/status
ros2 service call /protective_stop_guard/cancel_recovery std_srvs/srv/Trigger '{}'
```



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

循迹逐站与机械臂姿态、四点动作联动：

```bash
ros2 launch robot_task_control line_follow_arm_test.launch.py launch_rviz:=true
```

先确认底盘停稳，机械臂到行驶关节姿态后循迹；每条横线站点停车，经中转姿态执行四点，再回中转等待。途中站收到下一次 `/task_controller/start`（或主机开始信号）后，先回行驶位并确认到位，再继续小车；最终站完成后结束。关节角在 `eli_cs_robot_moveit_config/config/arm_joint_postures.yaml`，四点XYZ/度制RPY在 `config/line_follow_arm_test.yaml`，默认不拍照。详细步骤见[正式流程](docs/tasks/LINE_FOLLOW_ARM_TEST.md)。主机控制时加 `remote_control:=true`。

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

包含驱动、MoveIt、真实力传感器、限力检测器、串口测厚和TCP；默认不启动导纳节点。就绪后自动稳定拍HK/RGB/深度并发送，冻结拍照位姿，等待主机 `python3 send_points.py --file 本次结果.json`。匹配本次清单哈希后自动换算并触碰，无需start_task或start_capture。过期批次、DEMO和HOLD不会启动触碰；等待失败不回退本地点。

直接使用YAML点位，保持之前的本地测试：

```bash
ros2 launch robot_task_control camera_detection_real.launch.py target_source:=local
```

local不启动相机/TCP，读取 `config/camera_detection_real.yaml` 的 `camera_points_mm`，相对于启动时相机光学坐标，保持当前探针朝向。两种模式共用：沿tool0局部+Z反方向距目标20mm预接近，再沿该轴以2mm/s下探；20N停止推进，保持5秒采厚，逐点返回原位；不是恒力20N伺服。预测点之后最多再下探50mm。探针触点是terminal_link_2。

已有驱动可加 `start_arm_driver:=false`，已有完整控制栈再加 `start_arm_control:=false`。默认启动串口测厚；已有同一个测厚节点时可加 `start_thickness_detector:=false`，避免重复打开串口。具体步骤、主机像素点发送和参数见[两模式检测说明](docs/tasks/CAMERA_DETECTION_REAL.md)。

### 相机光心前方16cm → 探头触碰任务

点位配置是 `robot_task_control/config/camera_probe_touch.yaml`：`camera_points_mm: [0.0, 0.0, 160.0]`，表示启动时相机光学坐标下的点，不是基座坐标，也不是 tool0 的目标。机器人需先停在实际拍照姿态；不能用其他姿态拍摄的旧点套用当前 TF。

```bash
ros2 launch robot_task_control camera_detection_real.launch.py task_profile:=camera_probe_touch launch_rviz:=false
```

此配置默认使用本地点位，不启动相机、工控机通讯或底盘；包含真机驱动、MoveIt、六维力传感器和限力检测器；默认不启动导纳。就绪并稳定2秒后自动执行，无需调用 start 服务。以冻结的 `base_link→terminal_link_1` 把相机点变换到基座，以 `tool0→terminal_link_2` 补偿探针偏移，使实际探针触点到达检测点。各到位目标沿用启动时 tool0 朝向，姿态容差1度；中途允许规划器调整姿态，不锁定整条路径。预接近到位和下探时仍检查实际姿态；规划失败或下探姿态超限会取消。

流程：记录原位 → 沿 tool0 局部 +Z 反方向距目标20mm处预接近 → 冻结此轴并沿 +Z 以2mm/s推进 → 轴向力绝对值达到20N停止推进 → 保持5秒 → 沿同轴反方向撤离 → 返回记录的关节原位。未达到20N时可继续越过名义点，最多额外20mm（`max_penetration: 0.02`）；用完仍未达到20N则报错停止，不强行返回。过载、数据超时、碰撞等保护保留。新点继续修改同一 YAML，毫米单位，多点按 x,y,z 连续填写，逐点返回原位。转换报告在 `~/.ros/camera_probe_touch/`；转换后的探针基座目标保存在 `~/.ros/detect_control/camera_probe_touch_targets.yaml`。需要同步读取测厚仪时加 `start_thickness_detector:=true`。

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


### 漆膜测厚仪串口及单次任务

默认串口 `/dev/ttyCH341USB0`，115200/8N1，发送ASCII `.d0<:>`：

```bash
# 独立连续采集，无机械臂运动。
ros2 launch eli_cs_robot_serial detactor.launch.py
# 单次自动采集任务，之后保留节点供再次调用；不要与上一条同时运行。
ros2 launch eli_cs_robot_serial detactor.launch.py task_once:=true
```

另开已加载环境的终端：

```bash
ros2 service call /detactor/acquire_latest std_srvs/srv/Trigger '{}'
ros2 topic echo /detactor/latest_value std_msgs/msg/Float64 --qos-durability transient_local
ros2 topic echo /detactor/measurement std_msgs/msg/String --qos-durability transient_local
```

`acquire_latest`等待本次有效回包才成功，3秒未回包则失败；不会把旧缓存当成新采集。
最新值为设备整帧最后一个数，保留小数；新回包即使数值相同也更新时间和序号。
串口可加 `device:=/dev/ttyUSB0`。检测任务已启动测厚节点时，不再单独启动。

### 相机点检测、5cm限位及回位结果

```bash
# 麒麟先常驻监听：
python3 listen_robot.py
# 树莓派：加载环境后，包含驱动、串口测厚、稳定拍照和通讯。
ros2 launch robot_task_control camera_detection_real.launch.py launch_rviz:=false
```

收到匹配本次照片哈希的真实检测点后自动执行。流程：冻结拍照位姿→坐标转换→预接近→沿tool0局部+Z下探→
20N停止→保持5秒并采厚→撤离→确认回到本轮同一关节原位→回传本点测厚结果。

`camera_detection_real.yaml`和`camera_probe_touch.yaml`均设 `max_penetration: 0.05`：
这是预测点之后额外50mm，不是整个接近动作仅50mm。
行程用完或无碰撞IK无解时，先确认停止及数据有效，再规划撤离、返回原位；
TCP模式回位成功后重新稳定拍照、发送、等待新哈希对应检测点再执行，最多额外重试3次。
`max_reacquire_retries`可配置。主机必须保持同一物理点的point_id稳定，已成功点不重复检测。
无回包也不会冒充成功，将在安全回位后重取点。
过载、碰撞/安全锁定、状态断流、撤离/回位失败均停止，不强行回位重试。

本地固定点模式仍可用 `task_profile:=camera_probe_touch`；会安全回位，但没有真实重拍/取点来源，
失败后需更新点位再启动，不会用同一旧坐标无限尝试。

树莓派逐点结果话题：

```bash
ros2 topic echo /task_controller/detection_result std_msgs/msg/String --qos-durability transient_local
```

更新后的麒麟监听脚本自动收到并保存 `output_dir/detection_results/`，显示点ID、厚度和是否成功；
树莓派通讯结果也持久化，重连可补收最近100条。结果绑定接触保持阶段的读数，在实际回原位之后发出。

## 双关节姿态往返仿真（行驶位 ↔ 检测中转位）

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_task_control joint_posture_test.launch.py
```

此入口默认假硬件和独立ROS域190；显式`simulation:=false`切换真机；自动从第一姿态初始化，先校验两姿态与双向路径，再执行第一→第二→第一，结束后保留RViz。无需调用开始服务。报告：`/tmp/joint_posture_test_report.json`；成功状态为`DONE`，失败包含碰撞物体或规划错误码。不需要画面时加`launch_rviz:=false`。

两姿态在 `src/eli_cs_robot_moveit_config/config/arm_joint_postures.yaml`，单位度，顺序：基座、肩部、肘部、手腕1、手腕2、手腕3：

| 用途 | 六轴关节角（度） |
| --- | --- |
| 行驶姿态 travel | −45，15，−71，4.4，148，75 |
| 检测中转 detection_ready | −45，0，0，0，90，0 |

第二姿态仅是中转，不代替最终拍照/触碰朝向。启用双姿态的相机检测任务中，收到目标后先按关节角到中转位，再规划到检测预接近点；检测完先撤离，再按关节角到中转位、行驶位。逐点回传在实际到达行驶位后进行。若需要重拍，则从行驶位经中转位恢复本轮原拍照关节姿态再采图，不在中转位直接沿水平tool0轴触碰。

两份相机检测profile已启用`joint_postures_enabled: true`；通用检测配置默认仍关闭，旧仿真不强制采用新返回位。`/detect_controller/prepare_travel`可由后续联合任务在底盘行驶前调用；服务返回“接受”后仍须等`READY_TRAVEL`确认实际到位。

共用MoveIt入口自动添加`robot_base_ground`碰撞体，顶面是`base_link`的Z=0.08m；完整机械臂和新增末端都参与碰撞检测。只允许固定底座`base_link/base_link_inertia`与地面相交，运动连杆没有豁免。地面为覆盖机器人可达范围的6m×6m实体，向下延伸2m。双姿态检测在地面未确认时不启动规划。

真机双关节往返：先退出重复的机器人launch，再执行`ros2 launch robot_task_control joint_posture_test.launch.py simulation:=false robot_ip:=169.254.199.150`。默认包含真机驱动（headless、自动上电/释放抱闸）、MoveIt、8cm地面和RViz；不启动相机/底盘/力传感器。默认速度缩放10%。等待外控、真实硬件插件、scaled轨迹控制器、安全状态和地面就绪后再等5秒；当前位置不是行驶位时先规划到行驶位，再中转、返回行驶位。启动后自动执行，无需调开始服务。已有驱动可加`start_arm_driver:=false`，已有MoveIt再加`start_moveit:=false`，但共用地面节点及安全监控必须存在。真机报告`/tmp/joint_posture_real_report.json`。本次开发验证不执行实体机械臂。


### 底盘电量与充电遥测

底盘驱动运行后，TCP桥自动读取电量和系统状态。麒麟更新工具包后用`python3 listen_robot.py`持续显示，或`python3 start_task.py --action status`单次查询；返回`robot_status.chassis_power.battery_percent`和`charging`，未知值为null。无需额外CAN程序或采集服务。

```bash
ros2 topic echo /masc_chassis/bms_status --once
ros2 topic echo /masc_chassis/chassis_status --once
```

充电默认按本次协议截图0x204整个uint32的bit3解析；如果实机固件使用现有SDK的Byte1手动/桩充电标志，将`industrial_pc.yaml`的`charging_status_layout`改成`byte_flags`。详细字段与超时规则见麒麟工具包协议文档。

## 倒车充电模式

复用现有入口，默认关闭：

```bash
ros2 launch robot_task_control line_follow_only.launch.py \
  drive_direction:=reverse charging_mode:=true line_show_window:=true
```

需要已保存的同一路线前进记录（默认`~/.ros/line_follow/last_route.json`），不是任意起点的未知路线倒车。前进记录与普通倒车入口不变。`charging_mode:=false`仍使用原来的横线终停规则。

流程：先按记录路线返程，在剩余约60cm、近端视觉连续3帧可信且剩余路线为直线/缓弯时进入末段。使用画面上方靠车头的局部主线，先单独摆正，再单独横移回中；每次切换等待新帧与停稳，确认回中后只发直线倒车指令。方向目标3°；有实测地面映射时回中目标1.5cm，无映射时为半图宽的0.04（640宽约13像素），不能把像素比例当成厘米。末段未看到横线时不高于0.02m/s，看到横线或到达横线后不高于0.008m/s。

横线仅表示已接近充电位，仍沿线直退搜索充电信号，不再把到横线等同充电成功。横线平均位置明显越过目标时允许低速前挪，累计不超过5cm，换向前先停稳。前置相机只能观测当前可见胶带，剩余记录路线和原起点围栏仍有效。

充电信号来自现有`/masc_chassis/chassis_status`：循迹任务默认`charging_status_layout:=byte_flags`，使用本机已实测的SDK手动/桩充电标志；另一版32位协议才选择`charging_status_layout:=bitfield32`，取原始状态码bit3。独立TCP桥的布局也应与实际SDK一致。充电首帧立即发零速，连续3个不同源时间戳的新反馈确认后报告`reverse_route_complete`，`completion_source=charging_signal`。首次充电信号后也保持停车等待确认，抖动持续2秒不能确认则锁停报错；完成保持锁停，信号抖动不会重新发车；只有横线到位位置附近10cm内、或记录起点10cm内才报告成功；其他位置出现充电位也会停车，但报告位置异常而非完成。

保护：横线到位后实际累计搜索行程最多10cm，同时保留记录起点外最多10cm围栏；距剩余路线的里程计横向偏离最多10cm。有地面映射时视觉横向偏离也限制10cm，无映射用半图宽0.35做独立图像保护。超过35°朝向误差、输入失效、末段总用时超过90秒、横线后30秒仍没充电，均停止并报错，不绕开围栏继续找桩。10cm是基于里程计/已标定视觉的相对测量限度，不承诺未标定情况下的绝对定位精度。额外搜索距离可在`line_follow.yaml`的`charging_search_distance`调小（范围0.01～0.10米）。

查看：

```bash
ros2 topic echo /line_follow/status
```

`charging_dock`内含对齐阶段、横线后累计行程、限度及故障；`charging_signal`为true/false/null。常见故障：`charging_dock_search_limit`（横线后搜索超限）、`charging_dock_alignment_out_of_range`（视觉偏离过大）、`charging_dock_status_unavailable`（充电反馈过期/未知）、`charging_dock_charge_not_detected`（横线后等待超时）、`reverse_finish_deviation`（里程计偏离围栏）。报错后不自动重试，先检查实机位置、坐标方向和充电位定义，再重新启动任务。

正式测试先确认充电标志，再以空载/足够空间观察：未摆正不纵向后退；回中后直退；横线附近缓慢搜索；充电首帧停车并确认；没有充电时不能靠超距继续。开发测试仅使用隔离ROS域上的模拟状态与速度话题，不驱动实车。

## 自动测试：首条横线到站、中转等待5秒、倒车返回充电

复用同一个联合launch，选择`test_profile:=return_charge`，不新增重复入口：

允许从充电桩出发：机械臂先到行驶位，再以不超过0.02m/s循迹离桩；连续3帧确认停止充电后恢复正常速度。仍充电时累计前进上限30cm、总等待25s，超限锁停。手动充电、急停、故障或传感数据失效仍禁止运动。

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_task_control line_follow_arm_test.launch.py \
  test_profile:=return_charge line_show_window:=true launch_rviz:=true
```

此命令包含真机驱动，设备就绪后自动执行：

1. 确认底盘停稳，机械臂按关节角到行驶位`[-45, 15, -71, 4.4, 148, 75]°`。
2. 前进循迹；出发处已经存在、被离站逻辑忽略的横线不算目标。第一条有效横线到位即停车，即使它被识别成途中站，也不等待额外开始消息。
3. 确认轮速停稳，封存刚才的前进路线。随后机械臂到中转位`[-45, 0, 0, 0, 90, 0]°`，服务核查实际关节到位后，工作流再等待完整5秒。
4. 经中转按关节目标回行驶位，确认实际到位后才放行底盘。原进程内切换为倒车，不重启驱动、不清除路线；沿封存路线回到出发区域。
5. 末段用倒车充电模式先摆正、横移回中，再直退；横线辅助定位，充电首帧停车、连续反馈确认后完成。横线后额外搜索最多10cm，偏离/超距/充电信号异常时锁停报错。原有充电模式的图像、里程计和安全保护仍有效。
6. 等待底盘停稳，任务结束；机械臂保持行驶位，不循环、不自动再次出发。

启动前仍须配置CAN（需要时执行`sudo bash scripts/setup_can.sh auto`），并退出重复驱动。该profile不执行原来的四个笛卡尔点，也不启动海康、深度相机、力传感器、笛卡尔点位执行器；默认加载9010状态监视/TCP桥，MoveIt与关节姿态任务仍运行以执行碰撞检查和关节轨迹。中转是过渡姿态，没有触碰/20N检测。

默认充电位按截图0x204 uint32 bit3解析；若已实测确认另一SDK布局，添加`charging_status_layout:=byte_flags`。需要主机发首次开始时可加`remote_control:=true`，此时会加载既有TCP桥并等待主机start；后续中转、等待和返程全自动。

流程配置：`src/robot_task_control/config/line_follow_arm_charge_test.yaml`；两组关节角共用`src/eli_cs_robot_moveit_config/config/arm_joint_postures.yaml`。`delay.seconds`控制到中转后的等待时间。`finish_at_first_marker: true`只用于显式forward步骤，要求停止横线确实到位，普通丢线不能代替到站。`arm_postures_before_chassis`在每个前进/倒车步骤前自动插入并确认行驶姿态，配置不重复写关节角。

查看流程与充电阶段：

```bash
ros2 topic echo /task_controller/status
ros2 topic echo /arm_joint_postures/status
ros2 topic echo /line_follow/status
```

本测试已经用本机模拟服务验证首条途中横线可完成前进、封存不丢失、中转实际到位后至少等待5秒、回行驶位后才倒车及充电完成后停止。姿态往返和充电控制沿用各自已通过的仿真/模拟检查；完整流程尚未驱动实车验证。


### 正常/异常运行状态监视

联合任务默认加载9010状态桥。麒麟更新工具包后运行`python3 monitor_status.py`，会分两次请求正常进度和异常状态；只看其中一路使用`--channel normal`或`--channel abnormal`。当前任务、等待倒计时、前进/倒车/充电阶段分别显示，异常带项目错误码和可用的设备原始码。完整字段和错误码表见麒麟工具包协议文档。树莓派本机可查看：

```bash
ros2 topic echo /robot_status/normal
ros2 topic echo /robot_status/abnormal
```

麒麟最新快照在output_dir/status/normal.json与abnormal.json，事件在events.jsonl。监视器不控制运动，不自动清除报警；未知或断线不会显示为正常。


树莓派联合往返测试优先关闭RViz，保留循迹画面：

```bash
ros2 launch robot_task_control line_follow_arm_test.launch.py \
  test_profile:=return_charge line_show_window:=true launch_rviz:=false
```

该配置提前准备USB相机和识别，机械臂连接稳定5秒、设备检查及行驶姿态到位的条件仍保留；
相机预热不会提前使能小车。`prepare_line_camera_early:=false` 可恢复相机原来的串行启动顺序。
图像识别及后台横线检测各默认最多2个OpenCV计算线程；开启RViz时默认显示刷新率降为10Hz，降低竞争。
速度选择器继续执行100ms输入超时停车，新增“输入超时/恢复”和“调度延迟”日志，
用于区分识别主动停车与速度发送断流。速度话题使用只保留最新一条的队列，不积压旧命令。


联合往返 `return_charge` 使用关节姿态专用模式：不加载未使用的柔顺控制节点、
笛卡尔执行器及力传感器（显式启用传感器时除外），也不等待柔顺使能服务。
MoveIt、模型/碰撞规划及安全监视保留，机械臂关节动作只在对应步骤请求后执行。
普通 `stations` 和实际柔顺/触碰任务仍保留原来的控制接口。
机械臂控制器分为“激活组”和“仅配置组”，由两个spawner顺序加载；任一批失败取消启动。
驱动、里程计与已封存返程路线在步骤切换间持续保留。

关节姿态运动和站点等待期间，USB相机采集按需暂停，下游图像识别不再持续计算；
恢复前进/倒车前先开启采集并等待有效视觉反馈，15秒仍未就绪则报错停车。
底盘和机械臂驱动、安全监视、里程计保持常驻；相机暂停不清空返程路线。
横线斜率计算改由NumPy执行，后台检测使用受限OpenCV并行，控制状态机仍保持单线程。


## 单独循迹：离桩、首条横线等待5秒、返程充电

先退出其他底盘/联合任务，避免同时向速度话题发送指令。启动后自动执行，无需调用开始服务：

```bash
source /opt/ros/jazzy/setup.bash
source ~/project/robot_and_chassis_control/ros_ws/install/setup.bash
ros2 launch robot_task_control line_follow_only.launch.py \
  round_trip:=true charging_mode:=true \
  line_show_window:=true line_stop_marker_target_ratio:=0.5
```

复用已有launch，选择 `config/line_follow_charge_test.yaml`：
底盘停稳→前进循迹离桩→第一条有效横线停稳→封存路线→等待5秒→沿记录路线倒车充电→停稳结束。
起步出发线按原规则忽略；到站横线既可以是途中横线，也可以是末端横线。
前进离桩仍为最高0.02m/s、连续3帧不充电后恢复正常循迹；超过30cm或25秒仍充电则锁停。
倒车保留视觉回中、横线定位、10cm额外搜索限制，连续充电反馈确认才完成。
不会启动机械臂、MoveIt、RViz、TCP或其他相机；整个往返期间USB循迹相机和里程计保持工作。
`round_trip:=true charging_mode:=false` 继续使用原来的“最终横线往返”流程。


## 标准路线录制与重放（新增模式，原有循迹保留）

```bash
# 慢速录制route_01：首条有效横线保存标准，停5秒后原路回充电
ros2 launch robot_task_control line_follow_only.launch.py \
  navigation_mode:=teach route_name:=route_01 line_show_window:=true

# 回到相同实物起点后重放：轨迹为主、视觉辅助
ros2 launch robot_task_control line_follow_only.launch.py \
  navigation_mode:=replay route_name:=route_01 line_show_window:=false
```

其他路线换 `route_name`；标准数据在 `~/.ros/line_follow/standards/<route_name>/`，每次录制保留历史版本。
录制0.03m/s、约1cm抽样，重放初始0.05m/s、图像识别上限8Hz；最后到位保留视觉和充电确认。
完整准备、质量检查、数据说明及限制见 [标准路线流程](docs/tasks/STANDARD_ROUTE.md)。

倒车转弯过头的试调参数（前进控制保留）：`reverse_route_lookahead:=0.26` 为倒车路径前瞻，
`reverse_finish_heading_gain:=0.60` 为末段摆正力度。已设为默认值；同一launch的倒车、teach及replay往返均可覆盖。
仍急纠偏时先单独尝试前瞻0.28m；出现切弯则退到0.24m。仅末段摆正过冲时先尝试增益0.50。
这些是待实车验证的起始值，不改变横线停车与充电判断。

相机检测的TCP/local两种来源现在默认 `touch_control_mode:=moveit_force_guard`：普通MoveIt直线执行，20N取消并确认停稳，保持5秒采厚。需要旧导纳时显式传 `touch_control_mode:=admittance`。这不是恒力20N控制；规划场景没有添加的真实障碍不会自动被识别。

`line_follow_arm_test.launch.py` 的四点MoveIt站点流程默认关闭导纳节点，姿态管理和点位执行不再依赖导纳开关服务。
自定义任务如含导纳动作，需显式 `launch_admittance:=true`。独立阻抗控制入口保留。
