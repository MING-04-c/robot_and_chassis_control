# MASC 蓝线循迹控制

本包将 `camer_diver` 的蓝线观测转换为底盘速度。相机看到的蓝线向右偏时右转，
向左偏时左转，向前速度可配置。使用 `linear.x` 前进和 `angular.z` 转向，
不使用横移。默认速度为 **0.05 m/s = 5 cm/s**，适用于先检查方向的低速调试。
图像偏差是归一化图像量，不是米；当前是前瞻目标比例转向控制，不提供导航定位或避障规划。

## 编译和启动

在工作空间中编译：

```bash
cd ~/project/masc_chassis/chassis_ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select camer_diver masc_chassis_user_control
source install/setup.bash
```

停止之前运行的 `masc_tcp.launch.py`、单独的底盘 launch 和其它 `/cmd_vel` 发布者，
再用一条命令启动 USB 相机、蓝线识别、底盘驱动和循迹控制：

```bash
ros2 launch masc_chassis_user_control line_follow.launch.py forward_speed:=0.05
```

不必另外启动底盘驱动。本 launch 显式设置 `enable_pid=false`，不启动距离任务 PID。
不启动海康驱动和 TCP 服务，预览窗口默认关闭。`can0` 仍由系统 CAN 服务配置，启动前确认接口为 UP。
想让小车循迹运动时同时显示相机四格画面，开启 `show_window`：

```bash
ros2 launch masc_chassis_user_control line_follow.launch.py show_window:=true forward_speed:=0.05
```

`show_window:=false` 关闭画面（默认），相机识别和小车循迹仍运行。
开启画面不需要另外启动相机 launch。窗口需要本机桌面显示环境。

相机使用 `camer_diver/config/camera.yaml`，默认 `/dev/video0`、640x480、20 fps。
相机向前拍摄地面且画面竖直、没有镜像；如果左右镜像，应修正图像或调整 `steering_sign`。

**默认 enabled=true：底盘连接、底盘模式允许且连续三帧识别有效后会运动。**
若先查看识别结果而不让小车动，使用：

```bash
ros2 launch masc_chassis_user_control line_follow.launch.py enabled:=false show_window:=true
```

确认后启用或暂停：

```bash
ros2 service call /line_follow_controller/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /line_follow_controller/enable std_srvs/srv/SetBool '{data: false}'
```

只检查算法输出、不启动底盘驱动：

```bash
ros2 launch masc_chassis_user_control line_follow.launch.py dry_run:=true show_window:=true
ros2 topic echo /line_follow/cmd_vel
```

`dry_run` 强制发布到 `/line_follow/cmd_vel`，不发布 `/cmd_vel`，且跳过底盘反馈检查。
已有相机及识别节点时可以 `start_camera:=false`；它们必须发布默认的 `/camera/line/*` 话题。
不要让两个相机驱动同时占用同一个视频设备。

## 在线调速与配置

前进速度的单位是米/秒，可以在线改，不需要重启：

```bash
ros2 param set /line_follow_controller forward_speed 0.08
# 0.08 m/s = 8 cm/s；设置为 0.0 会停车
ros2 param set /line_follow_controller forward_speed 0.0
```

运行中的参数调整不会自动保存，长期使用请修改 `config/line_follow.yaml`，重新构建。
`forward_speed` 上限由 `max_forward_speed` 设置（默认 0.30 m/s）。弯道、估计目标和
前瞻回退时会自动减速，所以实际指令可能低于设置速度。

| 参数 | 默认值 | 单位/用途 |
| --- | --- | --- |
| `forward_speed` | 0.05 | m/s，前进速度 |
| `steering_gain` | 0.8 | 偏差到角速度的比例，偏差越大转向越快 |
| `steering_sign` | -1.0 | 右侧正偏差产生负角速度（右转）；镜像可设 +1.0 |
| `max_angular_speed` | 0.5 | rad/s，转向上限 |
| `error_deadband` | 0.02 | 归一化偏差死区，减轻直线摆动 |
| `linear_acceleration` | 0.15 | m/s²，正常加减速上限 |
| `require_bottom_connected` | true | 要求胶带与画面底部连接 |
| `input_timeout_sec` | 0.5 | 秒，视觉及底盘状态超时停车 |
| `reacquire_frames` | 3 | 丢线后连续有效帧数 |

例如转向太慢可以小幅提高 `steering_gain`，左右摆动明显则降低它或增加死区。
默认要求胶带连接画面底部；相机安装角度使底部始终看不到胶带时应先调整相机，
也可有意识地关闭此要求：

```bash
ros2 param set /line_follow_controller require_bottom_connected false
```

## 数据和停车条件

```text
USB 相机 /camera/image_raw
  -> camer_diver /camera/line/status + /camera/line/centerline
  -> line_follow_controller /cmd_vel（50 Hz）
  -> masc_chassis_ros_driver -> CAN -> 底盘
```

`line/status` 为 std_msgs/String JSON，读取 `error_normalized`、`lost`、
`orientation`、`bottom_connected`、`valid_row_fraction` 及估计/处理耗时标志。
中心线 PolygonStamped 的点是像素坐标，此处只检查非空、时间戳和输入来源；
不把它当成米制路线。两条视觉输入都必须持续更新。

丢线、输入超时、无效/横向胶带、过慢处理、CAN 未连接、底盘故障/急停/遥控模式、
障碍标志或充电时立即发送零速度；物理停车仍受底盘制动及惯性影响。
默认允许通信控制和驻车状态，由底盘驱动的自动解锁功能处理驻车。
重新找到有效线后自动恢复；人工暂停后必须手动重新启用。
实车模式拒绝 `synthetic_demo` 输入。遮挡估计时减速，但视觉识别不是障碍物安全判断。
横向胶带、交叉口和急弯不做路线选择，横向胶带停车并显示原因。

查看当前输出/状态：

```bash
ros2 topic echo /line_follow/status
ros2 topic echo /cmd_vel
ros2 topic echo /camera/line/status
ros2 topic echo --once /masc_chassis/chassis_status
```

状态例如 `tracking`（循迹）、`line_lost`（丢线）、`camera_timeout`（断流）、
`bottom_not_connected`（底部无胶带）、`chassis_not_ready`（底盘状态禁止运动）。

## 测试

```bash
colcon test --packages-select masc_chassis_user_control
colcon test-result --verbose
```

离线控制测试覆盖转向正负、速度及角速度限幅、丢线/无效输入停车、估计段减速和参数拒绝。
ROS 集成测试在独立通信域中模拟视觉和底盘反馈，验证断流、急停、断连、旧图像、启停及在线调速；速度只发送到测试话题。
实车的相机安装、颜色阈值和控制增益需要现场调试，不代表已经完成硬件循迹验证。
