# Gemini Pro RGB-D 预览与主机 V5.4 兼容采集

本包提供 ROS 2 预览、按需拍照和可选的 RGB 棋盘格标定。驱动位于 `../gemini_pro_ros2/`，内部包为 `orbbec_camera`、`orbbec_camera_msgs` 和 `orbbec_description`。当前树莓派使用驱动随附的 ARM64 Orbbec **C++ SDK v1（实测 1.10.35）**，不需要切换为主机的 Python 3.9 / Python SDK 1.1.4，也没有修改主机提供的冻结源码。

兼容目标是 `深度相机拍照程序和参数/` 中主机 V5.4 的数据格式与语义。预览读取 ROS 图像话题；**拍照直接向驱动请求下一组完整 SDK FrameSet，不从两个图像话题近似配对，不从毫米深度反推原始值**。不启动机器人或底盘。

## 编译与启动

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
MAKEFLAGS="-j1 -l1" CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --base-paths src --packages-select orbbec_camera_msgs orbbec_description orbbec_camera geminipro_camer_calibration --executor sequential --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch geminipro_camer_calibration gemini_pro_view.launch.py
```

更新后需要重启旧 launch。窗口左侧 RGB、右侧软件对齐深度伪彩，显示两路 ROS 接收 FPS、当前 RGB 内参和中心 Z 深度（毫米）。窗口约 10 Hz 刷新，接收 FPS 与界面刷新率不同。

点击“保存一组 RGB-D 与参数”即可拍照，等待 SDK 和保存期间窗口继续刷新。默认输出 `~/geminipro_captures`，关闭窗口会结束该 launch 启动的相机。

- 无桌面：`show_window:=false`，状态话题 `/geminipro_calibration/status`。
- 已有相机驱动：`start_camera:=false`；该驱动必须已更新并设置 `enable_v54_capture: true`，提供 `/camera/capture_frame`。
- 支持 `serial_number`、`camera_namespace`、`camera_params` 和 `output_dir` 覆盖。
- GUI 依赖：`sudo apt install python3-tk python3-pil.imagetk`。其他依赖见 `package.xml`。
- USB 规则在 `install/orbbec_camera/share/orbbec_camera/udev/99-obsensor-libusb.rules`。不要同时运行旧 Python 程序占用同一设备。

无窗口拍照入口仍可使用：

```bash
ros2 service call /geminipro_calibration/capture std_srvs/srv/Trigger '{}'
```

## 请求配置与实机约束

配置文件为 `config/gemini_pro_capture.yaml`：

| 项目 | 当前请求 |
| --- | --- |
| RGB | 640×480、30 FPS、MJPG（格式 ID 5） |
| 原始深度流 | 640×400、30 FPS、Y11（格式 ID 11） |
| D2C | `ALIGN_D2C_SW_MODE`，SDK 输出 640×480 uint16 |
| 翻转 | ROS 彩色/深度翻转均关闭，保留 SDK 实际镜像标志 |
| SDK 帧同步选项 | 关闭，当前 SV1301S_U3 不支持；同一 FrameSet 不等于已认证曝光同步 |
| 曝光/增益/白平衡 | 主机原程序未固定、未记录，保持驱动默认 |

实机枚举确认当前设备支持上述格式。主机旧 SDK 的 Profile 索引 1/16 不代表 ARM64 SDK 的枚举顺序，本适配按尺寸、帧率、格式选择和检查，不强行使用旧索引。SDK 输出不符时拒绝拍照，不降级到另一个尺寸并伪称兼容。当前实机输入深度 Profile 为 Y11，SDK 软件对齐后返回的帧格式为 Y16（ID 8）；输出按 uint16 原值保存，JSON 记录返回帧的实际格式，这不代表输入 Profile 被改成 Y16。

30 FPS 是请求值，实际接收 FPS 随 USB、计算负载波动；当前设备连接报告为 USB2.0。预览经过 ROS 驱动转换的深度为毫米；保存文件保留 **SDK 当次输出、尚未进行 ROS 毫米转换的 uint16 深度**，两者不要混淆。

## 拍照过程

驱动新增的 `orbbec_camera_msgs/srv/GetCaptureFrame` 服务只在收到拍照请求时复制数据，平时不额外发送原始深度流。

1. 请求后跳过 5 组完整、新鲜且时间戳递增的 FrameSet。
2. 从同一个 SDK FrameSet 获取 MJPG 彩色帧和软件 D2C 深度帧，复制原始数据。
3. 现场读取该深度帧 `getValueScale()` 和当前 Pipeline `getCameraParam()`，保留时间戳、序列号和 SDK 版本。
4. 检查尺寸、RGB/D2C 内参一致性、有效深度、深度比例和畸变。单次 SDK 等待最多 22.5 秒。
5. MJPG 解码损坏或整幅深度无效时，最多重新请求 3 次完整 FrameSet；配置/参数错误直接失败。
6. 保存后回读 PNG/NPY 验证数值一致，生成源文件 SHA256，最后写完成标记；失败数据不参与后续标定。

C++ SDK 明确将 `systemTimeStamp()` 定义为毫秒，因此当前适配保存该单位，并沿用 100 的时间差诊断上限。原主机 SDK 的时间戳单位未独立认证，不据此宣称两个 SDK 的原始时间基完全一样。主机后续处理运动目标前，还需独立验证同步和配准误差。

## 输出文件与主机兼容

```text
~/geminipro_captures/
└── camera_v5_4_时间戳/
    └── attempt_01/                 # 坏帧重试成功时可能为 attempt_02/03
        ├── rgb_001.png
        ├── d2c_depth_raw.npy
        ├── camera_parameters_same_session.json
        ├── capture_manifest.json
        ├── CAPTURE_COMPLETE.json
        ├── d2c_depth_raw_uint16.png
        └── depth_png_check.json
```

前五个文件沿用 V5.4 文件名、schema 和必要字段；清单 `source_files_sha256` 恰好包含 RGB、原始深度 NPY、参数 JSON 三份源文件。后两个文件提供无损原始深度 PNG 及其一致性证据。取完成标记所在的 `attempt_XX` 目录交给主机读取。

深度计算方式：`Z_mm = d2c_depth_raw[v, u] × depth_scale_mm_per_raw`。比例每次从实际 SDK 深度帧读取，不能因某次实测为 1 就写死；0 和 65535 不作为有效测距值。

参数 JSON 保留 `rgb_intrinsic`、`d2c_intrinsic`、两路畸变、`sdk_isMirrored` 和 SDK 原始参数结构；清单保留 `intrinsic_used`、时间戳与同一 FrameSet 来源。**每次采集都读取当前会话，不复制主机旧内参数值。**

为兼容原校验器，`align_mode=OB_PY_ALIGN_D2C_SW_MODE` 和 `source=ONE_SDK_FRAMESET_LEGACY_SW_D2C` 保留为 V5.4 协议枚举别名，分别映射到当前 C++ SDK 的软件 D2C 和同一 FrameSet。它们不表示实际运行 Python SDK。`sdk_backend`、`sdk_api`、`sdk_version`、`actual_align_mode` 和 `session_id` 如实记录当前后端；没有把 C++ SDK 1.10.35 写成 Python SDK 1.1.4。

可直接使用原文件夹中未修改的 `print_orbbec_v54_capture_parameters.py --capture /完整路径/attempt_01` 审计。该旧工具关于 Profile 索引的打印文案固定为旧平台值；本次真实后端应看 JSON 的 `sdk_backend`。兼容格式不代表与旧主机采集结果逐字节相同，也不等于验证过绝对测量精度。

## 是否需要重新标定

当前任务优先采集并保存相机出厂内参、D2C 参数和实际比例，不需要先准备棋盘格。若另外需要重算 RGB 镜头内参，窗口保留棋盘格功能：

1. 固定分辨率、镜头、对焦和安装，准备平整棋盘格并测量方格边长（毫米）。
2. 保存 20～30 组不同位置、距离和倾角的 RGB-D，覆盖中心、边角，至少 10 张完整角点检测成功。
3. 填实际内角点列、行及格长。10×7 个方格对应 9×6 个内角点，仅作计数示例。
4. 点击“计算 RGB 内参”，读取新格式已完成的 `camera_v5_4_*/attempt_*/rgb_001.png`，输出角点图、失败列表、逐图 RMS、去畸变对比和标准 ROS 内参 YAML。

RMS = sqrt(sum[每个角点的横向误差² + 纵向误差²]/角点总数)，单位像素。偏高视图标记供人工检查，不自动删除。此功能不替换设备深度尺度、D2C 外参，不执行手眼标定，也不自动设置机器人 TCP 或相机安装位姿。

## 画面检查

黑色深度区域表示无有效深度；中心显示 `invalid depth` 时不能据此计算距离。将相机对准工作距离内的实测平面，核对有效区域和测量误差。伪彩仅用于显示，不作为深度数值。中心 Z 是沿光轴的深度，不是机器人基座到目标的距离。

此前 ROS 近似配对版的 `capture_*/depth_mm.npy` 等文件不是本协议输出，不会通过改名或伪造原始比例自动转换。应使用更新后的驱动重新拍照。

## 本次验证记录

2026-10-19，在当前连接的 SV1301S_U3、ARM64 C++ SDK 1.10.35 上，完成安装后的 launch 预览和连续三组真实采集；三组均通过原文件夹中未修改的参数审计脚本及 `pi5_capture_only.verify_original_capture()`。本次现场比例为 1.0，原始深度 PNG/NPY 数值一致，退出正常。另用非 1.0 比例的合成数据验证未将比例写死，并检查错误内参、文件篡改、GUI 异步拍照及坏帧重试。棋盘格生成图片验证了新目录结构下角点结果不会互相覆盖。以上验证不代表完成几何精度、手眼标定或主机整套检测算法联调。

## 批量采集与 TCP 整组传输

已经启动相机时，可以复用其 `/camera/capture_frame` 服务批量采集，不另启驱动，不改变窗口的默认保存路径。每组均重新获取完整 SDK FrameSet 和当次参数：

```bash
ros2 run geminipro_camer_calibration capture_batch.py --count 3 --interval 1 --output ~/Desktop/rgbd_batch_01
ros2 run geminipro_camer_calibration capture_transfer.py pack --dataset ~/Desktop/rgbd_batch_01 --output ~/Desktop/rgbd_batch_01.zip
```

输出目录和 ZIP 路径必须是新的，不覆盖已有数据。批量目录还会生成 `dataset_summary.json`，记录实际组数、每组目录、深度比例和 SDK 后端。ZIP 包含所有组的 RGB、原始 NPY、参数、manifest、完成标记和原始深度 PNG，旁边的 `.zip.sha256` 用于独立核对压缩包。压缩不会改变解压后的像素、深度数组和 JSON 文件内容。

主机端复制 `scripts/capture_transfer.py` 后即可接收；接收模式只用 Python 标准库，不依赖 ROS、相机 SDK、OpenCV 或 NumPy。先在主机执行：

```bash
python3 capture_transfer.py receive --bind 0.0.0.0 --port 9030 --output ./received_rgbd
```

再在树莓派发送（示例 IP `192.168.1.100` 请换成实际主机 IP）：

```bash
ros2 run geminipro_camer_calibration capture_transfer.py send --archive ~/Desktop/rgbd_batch_01.zip --host 169.254.199.120 --port 9030
```

接收程序等待一个数据包，验证压缩包 SHA256、ZIP 成员及每组源文件清单，完成后保存 `rgbd_received_时间戳.zip` 并返回确认；一次接收完成即退出。发送端只有收到成功确认才打印 `TCP_SEND_PASS`。接收后解压 ZIP，主机处理程序读取各 `camera_v5_4_*/attempt_XX` 目录中的原 V5.4 文件即可。

传输协议为：4 字节 `RGBD` + 4 字节大端 JSON 头长度 + UTF-8 JSON `{version:1,size:ZIP字节数,sha256:ZIP哈希}` + ZIP 原始字节。回执为 4 字节大端 JSON 长度 + JSON `{ok,groups,sha256,saved_to}`；失败回执包含 `ok:false,error`。配套使用本工具的 send/receive，默认端口 9030。

2026-10-19 已复用运行中的相机，将三组真实数据保存到桌面 `geminipro_rgbd_20261019_01`；三组均通过主机原版校验器。其 ZIP 的本机 TCP 回环收发验证通过，收发端 SHA256 完全一致；错误哈希拒绝及临时文件清理也已验证。外部主机的 IP/端口与完整检测流程尚待实际对接。

## 相机 joint 检测坐标转换为机器人触碰目标

这里的工作是**检测点坐标转换**，默认不要求手眼标定。以 Joint_1 的子坐标系 `terminal_link_1` 为相机原点，检测得到相对于它的 XYZ；Joint_2 的子坐标系 `terminal_link_2` 是测厚探头检测点。控制器仍控制 `tool0`，必须补偿探头与 tool0 的偏移。

统一 `A_T_B` 表示把 B 中坐标转换到 A，位置单位米：

```
base_T_camera = base_T_tool0(观测时刻) × tool0_T_terminal_link_1
base_p_detection = base_T_camera × [camera_x, camera_y, camera_z, 1]
base_T_tool0_goal = base_T_probe_target × inverse(tool0_T_terminal_link_2)
```

后一个公式同时考虑位置和朝向，不是简单把探头 Z 偏移从基座 Z 中减去。机器人旋转后，安装偏移也会随之旋转。

### 启动与输入

先运行机器人驱动/模型 TF，再启动转换节点；本 launch 不启动相机、不控制机器人：

```bash
ros2 launch geminipro_camer_calibration target_transform.launch.py
```

推荐输入 `/geminipro_calibration/camera_point`，类型 `geometry_msgs/PointStamped`：

- `header.frame_id: terminal_link_1`，即相机 Joint_1 的子坐标系。
- `header.stamp` 是观测图像对应的 ROS 时间，不能填零。节点用这个时刻的机器人位姿转换，避免机器人移动后用新位姿解释旧照片。
- `point.x/y/z` 是相对于相机 joint 的检测点坐标，单位**米**。主机如果计算的是毫米，先除以 1000。

发布示意 Python（`node` 是发送程序的 ROS 节点，XYZ 需来自实际检测）：

```python
from geometry_msgs.msg import PointStamped
publisher = node.create_publisher(PointStamped, '/geminipro_calibration/camera_point', 10)
message = PointStamped()
message.header.frame_id = 'terminal_link_1'
message.header.stamp = image_observation_stamp  # 原图对应时间，不是旧照片处理结束的时间
message.point.x = detected_x_m
message.point.y = detected_y_m
message.point.z = detected_z_m
publisher.publish(message)
```

只有 XYZ 不能唯一确定接触姿态，因此默认**保持观测时探头的朝向**，只改变接触位置；拍照前先把探头朝向准备好。若现场需固定基座下接触朝向，可使用节点参数 `point_orientation_mode:=fixed_base` 和 `point_quaternion_xyzw`（XYZW）。

也支持旧的 `/geminipro_calibration/camera_target`（`PoseStamped`）：此接口的姿态明确表示相机 joint 下**期望探头接触朝向**。不要把视觉标记的任意姿态直接当作接触朝向。

### 输出与触碰

以下输出均为 `PoseStamped`，保留观测时间：

| 话题 | 坐标含义 |
|---|---|
| `/geminipro_calibration/target_base` | 基座下期望探头检测点位姿，交给 detect_control |
| `/geminipro_calibration/target_tool0` | 观测时刻 tool0 下的检测点，供核对；不是基座运动指令 |
| `/geminipro_calibration/tool0_goal` | 补偿探头安装偏移后，机械臂需要达到的基座 tool0 位姿 |

`detect_control` 接收 `target_base` 后生成预接近点，沿基座 −Z 导纳接近；20 N 停止，保持 5 秒，沿 +Z 撤离并回初始关节位置。转换节点收到坐标只更新目标；触碰任务通过 `/detect_controller/start` 开始。详见 `detect_control/README.md`。

默认拒绝超过 5 秒的观测和缺失时间戳；允许短暂等待对应时刻 TF 到达。跨主机需要同步时钟，并通过 TCP 保留原图时间戳。TCP 数据接收程序负责把检测坐标发布到上述 ROS 话题。

### joint 坐标和 SDK 光学坐标

必须同时对应**原点和轴向**。默认输入已经在 `terminal_link_1` 中表达。如果 SDK 的点在光学坐标中（通常 X 右、Y 下、Z 前），而模型 joint 的原点或轴向不同，必须先做光学坐标到 joint 的转换，不能只修改 `frame_id`。

若以后需要使用独立的实测 `tool0→光学坐标` 外参，可选启动：

```bash
ros2 launch geminipro_camer_calibration target_transform.launch.py \
  observation_frame:=camera_color_optical_frame \
  extrinsic_file:=/绝对路径/tool0_to_camera.yaml
```

外参 YAML 格式为 `units: m`、`parent_frame: tool0`、`child_frame: camera_color_optical_frame`、`translation: [x,y,z]`、`quaternion_xyzw: [qx,qy,qz,qw]`。设置该文件时优先用该实测变换；不设置文件时直接查模型 TF。

### 多个检测点

`/geminipro_calibration/camera_points` 接收 `geometry_msgs/PoseArray`：所有 `poses[].position` 是同一张图观测时相对于 `terminal_link_1` 的 XYZ，单位米；该接口忽略各点 orientation，采用上文的探头朝向策略。整组必须共用观测时间戳。转换成功后一次发布 `/geminipro_calibration/targets_base`，由检测控制器整组保存；任一点无效时整组不发布。

若每点都提供明确接触姿态，使用 `/geminipro_calibration/camera_targets`（PoseArray）。对应的整组输出还包括 `targets_tool0` 与 `tool0_goals`。输入顺序就是检测顺序，每次的避障路径由 MoveIt 规划，不在检测点之间直接连线。

此前的手眼求解脚本已移除，当前默认入口是 `target_transform.launch.py`。原 RGB-D 采集、内参检查及 RGB 棋盘格工具用途独立，继续保留。
