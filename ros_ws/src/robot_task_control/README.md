# 树莓派本机任务控制与两路摄像头

机械臂、底盘、力传感器、HK 拍照和 USB 循迹在同一台树莓派运行。HK 照片通过网线传给外部主机做缺陷检测；树莓派不启动 YOLO。机械臂和底盘不再通过两机 TCP 控制桥协调。

## 两路摄像头与控制入口

| 部件 | 本机接口 | 工作流作用 |
|---|---|---|
| HK 相机 | `/camera/image` | 视觉缺陷检测的图像来源 |
| HK 拍照 | `/dataset_capture/capture`，Trigger | 触发一次拍照，新照片保存成功才继续 |
| 已保存照片 | `/dataset_capture/captured_image` | 每次成功保存后发布对应图像 |
| 图像 TCP | `hk_image_transfer`，默认 9002 | 复用原 MIMG 协议传给外部主机 |
| USB 循迹相机 | `/line_camera/image_raw` | 蓝线识别，不使用 HK 图像 |
| USB 识别结果 | `/line_camera/line/status`、`/line_camera/line/centerline` | 提供循迹方向与有效性 |
| 循迹开关 | `/line_follow_controller/enable`，SetBool | 默认关闭，由流程开启 |
| 底盘任务 | `/chassis/task_control`、`/chassis/task_status` | 本机 PID 位移任务与结果 |

`chassis_velocity_mux` 是统一部署下唯一向 `/cmd_vel` 发布命令的节点。PID 输出到 `/robot_control/pid/cmd_vel`，循迹输出到 `/robot_control/line_follow/cmd_vel`；选中一路才转发。切换时先清除缓存、发送零速度；选中源超过 0.1 秒没有新命令时输出零速度。循迹开始前停止尚在执行的 PID 任务；循迹结束先关闭循迹，再交还 PID。流程失败/用户 stop 时关闭转发、循迹和导纳，并请求 PID 停止。

## YAML 工作流

`config/workflow.yaml` 默认顺序：USB 循迹 2 秒 → 前进 10 cm → 左移 10 cm → 后退 10 cm → 机械臂沿基座 +Y 移动 5 cm → HK 拍照。每步完成后才继续；默认不自动开始。

新增动作：

```yaml
- action: line_follow
  seconds: 2.0
  progress_timeout_sec: 3.0

- action: hk_capture
  step_timeout_sec: 30.0
```

`line_follow.seconds` 是累计收到有效 `tracking` 状态的时间，不是目标行驶距离。丢线、视觉过期或底盘未就绪时，循迹节点立即发零速度；连续没有 tracking 超过 `progress_timeout_sec` 则流程失败。默认速度为 0.05 m/s，可由启动参数 `line_follow_speed` 调整。

`hk_capture` 使用 `robot_task_control/config/hk_capture.yaml`，默认每次触发只拍一张，曝光为 40000 μs，与现有 HK 参数一致。独立的数据集多曝光配置仍保留，但统一工作流不使用它。触发并设置曝光后必须收到新帧，不能用无新帧时的旧图假报成功。返回成功表示该次新照片已保存并发布到照片话题；不是外部主机保存确认。

启动图像传输时，工作流在拍照前检查已有接收主机连接；没连接则失败，不静默执行拍照。拍照模式不按视频 FPS 限流，也不会用新照片覆盖待发旧照片；待发送照片按顺序排队，队列最多 8 张，满时记录错误且保留本地文件。仅允许一个外部接收连接；套接字发送失败的待发照片保留到重新连接后重发。沿用原协议的无 ACK 行为，不保证外部主机已将照片落盘。照片同时保存在树莓派供补取。如果现场要求逐张确认、断线补传，需要双方另加文件传输确认协议。

保留动作：`chassis_execute`、`wait_chassis`、`chassis_stop`、`delay`、`arm_move`、`arm_poses`。底盘每次执行具有独立 execution_id，拒绝旧终态。`config/chassis_tasks.yaml` 是允许引用的名称；实际底盘位移只定义在 `masc_chassis_ros_driver/config/ros2/task_plans.yaml`。

机械臂继续使用 MoveIt / 导纳、相对/绝对目标、当前姿态、距离限制与到位超时。最后的导纳运动完成后仍保持柔顺。每步默认总超时 90 秒，可通过单步 `step_timeout_sec` 修改。停止后再次 start 会从第一步重来，相对运动会再次叠加。

## 构建与统一启动

使用 64 位 Ubuntu / ARM64，在树莓派重新编译，不复制笔记本 build/install，也不要混用 ROS 2 发行版。需要 MoveIt、ros2_control、ROS 图像依赖、OpenCV、Boost、X11 等系统依赖；脚本现在构建 15 个包，包含 HK、USB 识别和循迹。

```bash
source /opt/ros/<实际发行版>/setup.bash
cd /你的路径/robot_and_chassis_control/ros_ws
rosdep install --from-paths $(colcon list --base-paths src \
  --packages-up-to robot_task_control --paths-only) --ignore-src -r -y
bash scripts/build_raspberry_pi.sh
source scripts/raspberry_pi_env.bash
```

SDK 从源码编译并安装到 `ros_ws/local/elite_sdk`；默认每次编译 2 个文件，内存不足可用 `ROBOT_BUILD_JOBS=1`。每个新终端先 source ROS，再 source `scripts/raspberry_pi_env.bash`。按既有 `scripts/setup_can.sh` 配置 CAN，核对底盘配置、机械臂与力传感器地址。

树莓派端（以下地址沿用旧部署默认，按实际网口修改）：

```bash
ros2 launch robot_task_control robot_system.launch.py \
  robot_ip:=169.254.199.150 local_ip:=169.254.199.100 \
  sensor_host:=169.254.199.80 \
  image_listen_ip:=169.254.199.100 image_receiver_ip:=169.254.199.250 \
  line_camera_device:=/dev/video0
```

统一启动默认开启两路摄像头、拍照节点、图像发送、速度选择器、底盘、机械臂驱动与 MoveIt/导纳；循迹输出默认禁用，只有执行 `line_follow` 才开启。默认关闭所有本机预览窗口与 RViz。

外部检测主机端（接收脚本不需要 ROS，可从 `robot_task_control/tools` 复制）：

```bash
python3 receive_camera_tcp.py --host 169.254.199.100 --port 9002 \
  --latest latest.jpg --save-dir hk_photos
```

接收主机连接且各设备就绪后，在树莓派另一个已 source 环境的终端：

```bash
ros2 service call /task_controller/start std_srvs/srv/Trigger '{}'
ros2 topic echo /chassis/task_status
ros2 topic echo /line_follow/status
ros2 service call /task_controller/stop std_srvs/srv/Trigger '{}'
```

参数：

- `start_hk_camera` / `start_line_camera` / `start_image_transfer`：关闭不需要的功能；流程中相应动作也要删除，否则等待服务超时。
- `image_port`：默认 9002；`image_format`：jpeg / raw；`image_jpeg_quality`：默认 95。raw 保留 ROS 图像字节，接收时 `--latest latest.raw`。
- `transfer_image_topic`：默认 `/dataset_capture/captured_image`，仅发送工作流触发的新照片；USB `/line_camera/image_raw` 持续提供循迹视频，不通过 HK 图片传输链路发送。若另需 HK 连续视频，必须同时设置 `image_delivery_mode:=stream transfer_image_topic:=/camera/image`；此时才使用视频 FPS 限流和最新帧缓存。
- `image_delivery_mode`：默认 photos，按触发照片排队发送；`image_max_fps` 只在 stream 模式生效。
- `hk_output_dir`：默认 `~/robot_data/hk`；`line_camera_device`：默认 `/dev/video0`，建议现场使用固定设备路径。
- `workflow_file` / `chassis_params` / `task_plans`：覆盖流程、底盘配置与任务表。
- `start_arm_driver` / `start_chassis` / `start_moveit`：使用已启动的对应节点。外部 PID 与循迹节点仍必须使用这里的私有速度话题，不能另向 `/cmd_vel` 发布。

`stop` 不是硬件急停；既有 MoveIt execute_latest 服务不提供取消正在执行的轨迹，MoveIt 模式在途轨迹需通过机械臂控制器停止。底盘 TF 默认关闭，避免与机械臂 base_link 重名；本次未建立整车 URDF。力传感器和漆膜厚度采集器通信继续保留，厚度采集器仍使用自己的 detactor.launch.py。

## 验证与限制

当前 ARM64 / ROS 2 Jazzy 已编译 HK 驱动、USB 图像识别、循迹控制、底盘和新任务控制器。无硬件测试覆盖真实 PID 的到位/超时/停止、工作流模式切换、速度选择、拍照保存与失败、原 MIMG JPEG/raw 接收兼容。

```bash
source install/setup.bash
/usr/bin/python3 src/robot_task_control/test/test_local_control.py
/usr/bin/python3 src/robot_task_control/test/test_hk_capture_transfer.py
colcon test --packages-select camer_diver masc_chassis_user_control
colcon test-result --verbose
```

尚未连接真实两路摄像头、外部检测主机、机械臂和 CAN 底盘做联合验证；当前环境仍缺少完整机械臂 MoveIt 构建依赖。树莓派上双相机带宽、实时性与检测画质需要实测。Gazebo 和旧底盘/机械臂控制通信桥不参与部署；原图像传输代码已独立恢复，归档源仍保留在 legacy。
