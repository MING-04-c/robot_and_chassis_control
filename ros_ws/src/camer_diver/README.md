# camer_diver

ROS 2 Jazzy / Ubuntu 24.04 USB UVC 相机驱动，C++17，直接使用 Linux V4L2。
包名按要求保留为 `camer_diver`。这是一份独立实现，不依赖旧 luvcview、SDL 或 AVI 库。
支持单平面 capture 设备的 MJPG、YUYV、UYVY；不支持 H264、多平面设备及需要
libcamera 管线的树莓派 CSI 摄像头。

## 编译

当前机器已有主要开发依赖；在新系统中可安装：

```bash
sudo apt install build-essential python3-colcon-common-extensions \
  ros-jazzy-rclcpp ros-jazzy-sensor-msgs ros-jazzy-cv-bridge \
  ros-jazzy-camera-info-manager ros-jazzy-ament-cmake-gtest \
  libopencv-dev v4l-utils
```

外层 `camer_diver` 是工作区，本功能包位于 `src/camer_diver`。在工作区根目录编译：

```bash
cd /home/tiema/project/Linux系统/camer_diver
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

迁入已有工作区时，只复制本功能包 `src/camer_diver`，不要复制外层工作区和生成目录。
在目标工作区根目录执行：

```bash
colcon build --packages-select camer_diver --symlink-install
source install/setup.bash
```

## 摄像头准备

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
ls -l /dev/video0
```

必须选真正的视频采集节点，有些相机同时提供元数据节点。
当前开发环境未暴露 `/dev/video*`，编译和离线测试不代表完成实机验证。
如果报 Permission denied，检查设备所属组；通常需要 `sudo usermod -aG video "$USER"`
并重新登录。不要用 sudo 启动 ROS 节点。

## 启动

默认 640x480 MJPG / 30fps，命名空间 `/camera`：

```bash
ros2 launch camer_diver camera.launch.py
```

指定自己的参数文件或命名空间：

```bash
ros2 launch camer_diver camera.launch.py \
  params_file:=/absolute/path/camera.yaml namespace:=front_camera
```

直接启动并覆盖参数（此命令不读取默认 YAML）：

```bash
ros2 run camer_diver camera_node --ros-args -r __ns:=/camera \
  -p video_device:=/dev/video0 -p pixel_format:=MJPG \
  -p image_width:=1280 -p image_height:=720 -p framerate:=30
```

相机只支持 YUYV 时使用 `-p pixel_format:=YUYV`；尺寸和帧率要参考枚举结果。
驱动接受相机协商出的实际尺寸，打印实际尺寸和帧率；不接受格式静默回退。
参数为启动时只读参数，修改后需要重启。

## 话题与参数

| 话题（默认 launch） | 类型 | 内容 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | BGR8 像素数据 |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG 数据 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 标定或未标定相机信息 |

三个发布器均使用 SensorDataQoS（Best effort、Volatile），订阅者应使用兼容 QoS。
所有输出使用同一 ROS 接收时间戳和 `frame_id`；当前不提供硬件曝光时间同步。
光学坐标系约定为 x 向右、y 向下、z 向前，机器人到光学坐标系的 TF 由机器人描述发布。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `video_device` | `/dev/video0` | 可使用 `/dev/v4l/by-id/` 下的稳定路径 |
| `pixel_format` | `MJPG` | `MJPG` / `YUYV` / `UYVY` |
| `image_width`, `image_height` | `640`, `480` | 1..4096，YUV 宽度须为偶数 |
| `framerate` | `30` | 整数 1..240，实际由相机协商 |
| `frame_id` | `camera_optical_frame` | 输出帧名，不自动发布 TF |
| `camera_name` | `usb_camera` | 标定管理器的相机名称 |
| `camera_info_url` | 空 | `file:///绝对路径/标定.yaml`；空时尝试默认标定路径 |
| `publish_raw` | `true` | 解码后发布 BGR8 |
| `publish_compressed` | `true` | 发布 JPEG |
| `jpeg_quality` | `85` | 1..100，仅影响 YUYV/UYVY 转 JPEG |

使用 `MJPG` 且 `publish_raw:=false` 时，压缩帧原样转发，避免解码/重新编码；
消费者需要支持相机生成的 JPEG。`jpeg_quality` 无法改变相机生成的 MJPEG 质量。
输出的 JPEG 话题采用标准 compressed image 命名，可由 image_transport 消费；
此包直接发布消息，不加载额外的压缩插件。
标定文件分辨率不匹配时发布未标定 CameraInfo（K[0]=0），避免使用错误内参。

## 显示与记录

```bash
sudo apt install ros-jazzy-rqt-image-view
ros2 run rqt_image_view rqt_image_view
```

选择 `/camera/image_raw`，如果无图像，检查查看器的 Reliability 是否设置为 Best effort。

```bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 bag record /camera/image_raw/compressed /camera/camera_info
```

rosbag 保存 ROS 消息和时间戳，不是 MP4/AVI；视频文件导出应由单独的录像节点处理。
树莓派跨网络传输建议压缩话题，例如 1280x720 BGR8/30fps 的原始像素约 83 MB/s。

## 测试与实现

```bash
colcon test --packages-select camer_diver --event-handlers console_direct+
colcon test-result --verbose
```

离线测试覆盖 MJPEG 解码、YUYV/UYVY 转色、带填充的行跨度、短帧及设备初始化失败。
`src/v4l2_capture.cpp` 负责非阻塞 V4L2、mmap、缓冲队列和资源释放；
`src/frame_conversion.cpp` 负责图像校验和转换；`src/camera_node.cpp` 负责 ROS 发布与标定。
定时器每 2ms 非阻塞检查帧，不阻塞 ROS 服务和退出处理。损坏帧丢弃并告警；
设备断开或不可恢复的 ioctl 错误使节点以非零状态退出，需要重新启动，不自动重连。
当前不提供曝光/增益等 ROS 参数，可在启动前用 `v4l2-ctl --list-ctrls` 查看控制项，
再通过 `v4l2-ctl -d /dev/video0 --set-ctrl=控制名=值` 设置受支持的控制。

V4L2 mmap 接口参考：<https://docs.kernel.org/userspace-api/media/v4l/mmap.html>。

## 蓝线循迹扫描可视化

### 接上摄像头实时查看（推荐）

现在提供 `line_tracker_node`，订阅摄像头图像并实时绘制扫描边界、中线和识别状态。
重新编译一次：

```bash
cd /home/tiema/project/Linux系统/camer_diver
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch camer_diver line_tracking.launch.py
```

这个 launch 同时启动相机和循迹节点，桌面窗口直接显示四格画面：

1. 原图叠加：青色左边界、紫色右边界、黄色有效扫描中点、红色无效行、绿色前瞻目标。
2. 蓝色二值掩膜：白色为蓝线、黑色为其他像素，最适合检查颜色阈值。
3. 颜色证据图：绿色为强蓝色，橙色为弱蓝候选；候选并不一定被最终接纳。
4. 可选 Canny 边缘图：默认关闭，同样不参与蓝线中线定位。

启动前在 `config/camera.yaml` 设置设备、分辨率和相机支持的格式。
不能同时运行两个相机驱动占用同一个 `/dev/videoX`。
该 launch 强制开启相机的 `publish_raw`，保证循迹节点能收到 BGR 图像。
窗口按 `q` / `Esc` 退出循迹节点；整个系统按终端 `Ctrl+C` 停止。

无桌面/通过 SSH 运行时：

```bash
ros2 launch camer_diver line_tracking.launch.py show_window:=false
```

在有桌面的电脑上用 `rqt_image_view` 订阅 `/camera/line/debug_image`；
图像发布采用 Best effort QoS，查看器需要兼容该 QoS。
跨机器时 ROS_DOMAIN_ID、网络发现等配置需一致；原始调试画面较大，优先本机查看。

如果已有其他相机驱动发布图像，只启动处理节点，不再占用摄像头：

```bash
ros2 run camer_diver line_tracker_node --ros-args -r __ns:=/camera \
  --params-file config/line_tracker.yaml \
  -p input_topic:=/your_camera/image_raw -p show_window:=true
```

支持 cv_bridge 可转换到 BGR8 的 `sensor_msgs/Image` 输入，不直接订阅压缩图像。

### 颜色阈值与实时调参

默认值为 H=100..130、S>=70、V>=18，强蓝色还需满足 RGB 相对蓝色优势，保留暗蓝而拒绝轻微偏蓝的灰色。
`illumination_support=true` 时，另检测 S>=12、V>=8 且蓝通道相对占优的弱候选。
弱候选的连通区域必须含至少 `weak_seed_pixels=20` 个强蓝色像素，
且补充像素距强蓝色不超过 `weak_support_distance=6`，避免向整片背景扩张。
孤立的淡蓝区域或全段没有强颜色证据时可能仍被拒绝；连接的蓝色背景仍可能干扰。
纯白反光或严重过曝没有可靠颜色证据，算法不会凭空补出中线。

桌面窗口中，**左键点击左上原图里的蓝线中间**：对周围 7x7 像素取样，
自动调整 hue_low/hue_high、saturation_min、value_min，并显示实际 H/S/V。
取样区域至少一半像素须满足明确蓝色证据，饱和度阈值不会被取样降到 70 以下。
选择真实蓝线，尽量避开边缘和纯白反光。取样只对本次运行生效，不自动保存参数。
鼠标取样仅支持本地 OpenCV 窗口，rqt_image_view 不会触发此回调。

`config/line_tracker.yaml` 保存默认参数。在线可直接修改，无需重启处理节点：

```bash
ros2 param set /camera/line_tracker hue_low 100
ros2 param set /camera/line_tracker hue_high 130
ros2 param set /camera/line_tracker saturation_min 70
ros2 param set /camera/line_tracker value_min 18
ros2 param set /camera/line_tracker show_canny true
ros2 param set /camera/line_tracker allow_reacquire false
```

OpenCV H 为 0..179，S/V 为 0..255。默认阈值只是蓝色起始范围，要用实际掩膜调参。
调参结果不会自动写回 YAML，需要手动保存，或用 `ros2 param dump /camera/line_tracker`
导出参数到文件。相机设备、窗口开关、超时为启动参数，其余识别参数动态生效。

HSV 色相/饱和度分割通常能区分蓝线与白/灰/黄色地板；蓝色地板或蓝色杂物仍会干扰。
ROI 蓝色面积过大时显示 `BLUE_INTERFERENCE`，但局部蓝色杂物不能仅靠颜色彻底排除。
阴影、反光、自动白平衡和曝光变化仍需在实车上验证。

### 对博客预处理建议的取舍

实际流程为：缩小图像 → 中值/高斯/不滤波 → HSV 与 RGB 相对蓝色优势联合筛选 → 有限距离弱蓝补充 → 形态学去噪 → ROI →
小连通区域过滤 → 自底向上扫描同一蓝色区间 → 有效性判定 → 可视化。

- 保留可调中值滤波，默认 5x5 加强去噪，细线可调回 3x3；高斯滤波可选。
  中值更适合孤立噪点，但不是所有噪声下都最好。
- 强蓝色也必须满足 `B > G`、`B > R` 和 `(2B-G-R)/(B+G+R+1) >= strong_blue_excess_min`（默认 0.12），
  不能只依赖在灰色上不稳定的 HSV 色相。默认饱和度下限为 70。
- 弱蓝只补充距强蓝色种子 `weak_support_distance`（默认 6）像素以内的区域，
  不再因为整片背景连通于几个强蓝像素就全部接纳。长段泛白会保守拒绝，但仍可计算剩余可见段偏移。
- 开/闭运算后与原始颜色证据取交集，不把没有蓝色证据的孔洞或桥接像素填成蓝色。
  开运算默认 3x3 去除细碎毛刺；极细线可调 `open_kernel=1` 关闭。
- 用连通区域面积代替固定轮廓宽高比/实体率筛选。弯曲、细长赛道可能被博客的固定范围误删。
- 保留灰度计算供 Canny 诊断使用，窗口显示强/弱颜色证据；主分割不采用灰度自适应阈值。
- 不叠加 Sobel、Scharr、Canny 三套处理，也不把背景灰度中位数估计的 Canny 阈值视作通用自适应保证。
- `show_canny=true` 开启灰度边缘诊断，默认 `canny_auto_threshold=true` 按 ROI 灰度中位数
  及 `canny_sigma=0.33` 估计双阈值；关闭自动估计可用 `canny_low/high`。
  全黑画面仍保证高阈值大于低阈值。Canny 不替代蓝线颜色分割，
  但 `use_occlusion_edges=true` 时会辅助区分“疑似遮挡”和“仅有颜色缺口”。
- 博客“Canny 连接断裂”是基于梯度连通性的滞后阈值筛选，不会自动跨越任意缺失线段。
- “STM32 >100fps”依赖分辨率、芯片、采集和实现，不能作为本项目性能承诺。JSON 状态提供处理耗时供测量。

### 扫描规则与急弯限制

默认 ROI 覆盖全图，不要求胶带出现在中央、顶部或底部。先按旋转包围框长度和平均蓝色饱和度
对连通区域评分，选一个候选胶带，避免在多个不相连色块之间跳转；这仍是启发式筛选，
同样鲜蓝且细长的背景物体可能误识别，并非可靠的赛道语义分类。
评分前增加几何门槛：旋转包围框长边 / 内部厚度至少 `min_line_aspect=2`；
厚度取各截面的最大内部距离乘 2，再取中位数，不用斜向胶带被放大的横向弦宽。
纵向候选按行、横向候选按列评估，至少 `min_row_coverage=0.65` 的截面合格，至少 `min_row_fill=0.75`
的像素属于同一候选；中线跳变按局部宽度和行距归一化检查，阈值 `max_shape_center_step=2`。
采用局部行宽而非整个包围框宽度，允许胶带横向弯曲。紧凑斑点或多孔噪声会被拒绝，
没有合格候选时清空掩膜和路径，不回退使用被拒绝的色块。极短或复杂急弯线段仍可能被拒绝。
进一步使用 `min_core_fraction=0.45` 检查实心内部像素比例，拒绝细枝/锯齿噪声；
内部像素是距候选边缘至少 `max(1.5, 内部厚度*0.2)` 像素的点。
`max_width_spread=2.5` 限制内部截面宽度的 90%/10% 分位数比，避免忽宽忽窄的碎色块。
先排除两端自然收窄的截面，避免将斜胶带末端/被电线切出的尖端误当作异常线宽。
纵向胶带从底向上按行扫描；横向胶带从右向左按列扫描，状态为 `HORIZONTAL`，目标取可见中线的横向中点。
横向状态仅表示已看见胶带，不等同于正确的转向目标或交叉口分类。
首次找到胶带前不会因空行停止。首次在 `initial_search_radius` 内
（默认 640，覆盖默认处理图宽度）、后续在 `search_radius` 内找最近蓝像素，
再扫描这一段的左右边界，后续行使用上一有效行的中点作为起点；
关闭 `allow_reacquire` 可检查严格扫描模式。多个蓝色区间不会合并为全行最左/最右像素。
边界触及图像边缘、线太窄/太宽或中点跳变过大时，本行标记无效，不更新有效起点。
找到胶带以后，超过 `max_missing_rows` 连续无效行才停止往上续接。
多个断开的线段优先选一个候选；仅当短间隔两端线宽相近、
局部方向及横向位置匹配时，保留两段真实蓝色证据。扫描跨越短缺口的容忍范围
取 `max(max_missing_rows, ceil(max_occlusion_gap/row_step))`。
短碎片如果仅因细长度不足而被拒绝，可在完整合格胶带的端点方向、位置和厚度支持下接纳，
标记 `context_supported=true`；短片实心比例允许降至常规阈值的 80%，补偿两端边界占比。
单独的紧凑色块仍不会作为起始胶带。
JSON `candidates` 输出每个候选的 `rejection`、`aspect`、`core_fraction`、
`section_coverage`、`width_spread`，`selected_candidate` 为初始选择的候选 ID。
`accepted` 表示通过单体或上下文几何检查，不代表该候选被选为路线。

### 短遮挡与观测/估计区别

`max_occlusion_gap=72`（处理图像像素）是短缺口插值的硬上限；`0` 关闭。
实际允许长度为 `min(max_occlusion_gap, max(3*row_step, 1.5*局部截面宽度))`，
较窄胶带不会因为提高硬上限而接入过长缺口。颜色缺口和可信方向仍必须匹配。
需要沿扫描方向两端均有完整有效截面，两端宽度比不超过 1.8，横向对齐符合
`max_occlusion_offset=12`；宽截面、岔路/急弯跳变和触边拒绝行不会被当成遮挡补齐。
部分遮挡导致一行只剩窄蓝边时，低于附近行宽中位数 70% 的截面不再作为真实中线。
估计截面不填蓝色掩膜、不计入 `valid_rows`，显示橙色点。
Canny 在方向和宽度匹配的片段间独立检查边缘，不再要求先生成估计中线。
它寻找预计胶带宽度之外的非蓝边缘，以及内部相对两侧背景更暗的非蓝边缘，至少 8 个支持像素时
报告 `occlusion_suspected=true`、状态 `OCCLUDED`；否则为 `GAP_ESTIMATED`。
右下图的橙色边缘为支持证据，`occlusion_edge_pixels` 为支持数量。
即使存在这些边缘也只能说“疑似遮挡”，背景纹理/阴影也可能产生类似证据，不能证明物体存在。
`use_occlusion_edges=false` 可关闭这项辅助；`show_canny=false` 只关闭显示，不关闭辅助计算。
`inferred_rows` 记录估计行数；`target_estimated=true` 表示当前偏差目标来自估计。
`line/centerline` 只发布观测点，`line/estimated_centerline` 只发布估计点。
长缺口、只见一端或不匹配的两段不会补线，但仍可能输出剩余可见段的偏移。
`OCCLUDED` 只代表疑似遮挡，可能没有估计中线；必须另查 `inferred_rows` 和 `target_estimated`。
`candidates[].accepted` 表示形状或上下文检查合格，`retained` 才表示进入最终掩膜。
`joins` 提供片段/锚点编号、间隔、允许间隔、宽度比、外推误差以及拒绝原因：
`scan_axis_mismatch`（扫描方向不同）、`overlapping_axis_ranges`（范围重叠）、
`width_mismatch`（宽度不匹配）、`tangent_mismatch`（方向不匹配）、`gap_too_long`（缺口过长）。
空拒绝原因表示续接通过；每帧最多记录 64 次检查。端部拟合优先使用完整截面、
避开尖角，并用中位数斜率降低少量异常端点的影响。短片段不能独立成为目标，只能匹配已确认胶带。
完全看不到胶带时立即丢线，不延用上一帧路线。这只是视觉路径估计，
不识别异物是否可跨越，也不负责避障；下游控制应对估计段降置信度/减速或停车。

优先取 ROI 从底往上 `lookahead_ratio` 处的邻近有效行。如果该处没有胶带，
在实际有效中线的上下范围内按同一比例选择目标行，设置 `target_fallback=true`。
只要至少有 `min_valid_rows` 个有效扫描行，就可输出局部中线和偏差，不要求底部连接。
`bottom_connected` 独立报告是否在底部紧邻的 `bottom_anchor_tolerance_rows` 行内找到中线；
`require_bottom_anchor=true` 可恢复底部连接要求。`lost=false` 仅代表可测偏移，不保证可安全行驶。
偏差计算为 `error_pixels = target_x - (image_width - 1) / 2`，向左为负、向右为正，
界面黄色横段显示目标点与图像中心之间的距离。归一化偏差范围约 -1..1。
它是图像空间偏差，不是米、转角或电机控制量。

此版本根据候选方向选择行/列扫描。宽度/跳变检查会拒绝部分急弯歧义，**没有实现骨架中线或完整法线中线**，
也不能可靠区分弯道、岔路和交叉口。`PARTIAL` 仅表示部分行被拒绝，不代表完成路型分类。
`valid_row_fraction` 是有效行比例，不是统计概率或通行安全指标。
一次处理结束后超过 `frame_timeout_sec=0.5` 秒没有完成新一帧时，报告 `STALE_FRAME` 并清空路径。
处理超过该预算时 JSON 报告 `processing_over_budget=true`，避免把单次处理过慢立即伪装成断流。
图像仍保留原始时间戳，实际新鲜度应结合时间戳和 `processing_ms` 判断；这不保证实时控制时延。
此节点不发布 `/cmd_vel`，没有自动驾驶小车。

### 输出话题

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/camera/line/debug_image` | Image / BGR8 | 四格实时调试画面 |
| `/camera/line/mask` | Image / mono8 | 蓝色掩膜，处理图尺寸 |
| `/camera/line/edges` | Image / mono8 | 开启 Canny 时发布 |
| `/camera/line/centerline` | PolygonStamped | 观测中点，纵向从底到顶、横向从右到左；丢线时为空 |
| `/camera/line/estimated_centerline` | PolygonStamped | 短遮挡的估计中点，与真实观测分开发布；无估计/丢线/断流时为空 |
| `/camera/line/status` | String | JSON：状态、丢线、有效行比例、目标点、偏差、处理耗时 |
| `/camera/line/lost` | Bool | 初始未收到图像、处理错误或断流也为 true |

`centerline` 使用 PolygonStamped 作为点序列容器，不是闭合多边形。
点的 x/y 是**缩小后的处理图像素坐标**，z=0；header 继承相机时间戳和来源标识，
不能按 header.frame_id 把这些像素直接解释为三维光学坐标或送入 TF。
需要米制路径时必须结合相机标定和地面投影，并使用明确的坐标消息。
丢线时 JSON 的 target/error 字段为 null，不能复用上次有效偏差。

```bash
ros2 topic echo /camera/line/status
ros2 topic echo /camera/line/lost
```

识别不到时先看掩膜和 JSON 的 `lost_reason` / `first_rejection`：

- `no_blue_pixels`：颜色、ROI 或面积过滤没有留下蓝色，检查取样、颜色证据图与阈值。
- `no_bottom_anchor`：仅在 `require_bottom_anchor=true` 时，底部连接要求未满足。
- `insufficient_valid_rows`：颜色候选存在，但可用扫描行不足，检查线宽、边界和搜索范围。
- `shape_rejected`：颜色候选不是合格线形，例如紧凑色块、有效行覆盖不足或中线跳变。
- `first_rejection`：第一处失败原因，例如 boundary clipped、too narrow、too wide、seed outside blue。

自定义 ROI 外的蓝线不参与计算；边界被画面截断的扫描行仍无效，因为无法确定完整横截面中点。
如果直照使原始相机画面明显泛白，应先改善曝光/白平衡或补光角度，再调阈值。
控制名称需用 `v4l2-ctl -d /dev/video0 --list-ctrls` 查看实际设备，不能硬套其他摄像头控制编号。

### 无摄像头的测试模式

```bash
ros2 launch camer_diver line_tracking.launch.py demo:=true
```

这是有明确 `SYNTHETIC DEMO` 标识的合成图像源，每 6 秒切换完整蓝线、断线和无线场景；
仅用于检查实时发布和调试窗口，不能替代真实摄像头验证。

运行 ROS 通信集成测试（本机通信，无摄像头，测试后自动结束处理节点）：

```bash
ROS_DOMAIN_ID=137 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 src/camer_diver/test/test_live_ros.py
```

验证未收到帧、有效路径、在线参数校验、丢线清空路径和断流状态。

### 旧的离线算法动画

在功能包目录用浏览器打开 `visualization/line_scan.html`，无需启动 ROS 或安装额外依赖。
这是算法示意工具，还不是实车循迹节点。

- 点击“播放扫描”查看逐像素找边界的动画，“下一行”可逐行检查。
- 切换缓弯、90° 急弯、画面边缘和断线场景。
- 黄色为扫描中点，绿色为生成蓝线时已知的几何中心，红色表示丢线/无效中点。
- 可修改蓝线宽度、扫描行间隔和地板颜色，切换二值掩膜视图。
- 在本地页面地址后加 `#bend`，直接查看急弯处暂停示例。
- `visualization/line_scan_preview.png` 是急弯暂停预览。

扫描起点落在蓝线外时，第一对跳变不保证是同一段蓝线的左右边界，示意会标记无效。
若安装了 GJS，可执行 `gjs visualization/test_line_scan.js` 验证 HTML 中的扫描逻辑。
