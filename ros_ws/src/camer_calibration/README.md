# 海康工业相机内参标定

本包提供 ROS 2 海康相机实时标定/AprilTag 测距界面，并保留读取 MVS 本地照片的离线标定程序。不连接机械臂、不执行手眼标定、不发送运动命令。USB 循迹包 `ruier_camer_diver` 与本工具相互独立。

## ROS 2 实时界面：启动后直接看 AprilTag

入口为 `hk_tag_view.launch.py`，默认启动海康驱动和中文操作窗口。**不启动机械臂、底盘或循迹**。窗口能选择内参文件、自动识别 AprilTag 家族和 ID、填写真实尺寸，以及采集棋盘格并计算内参，不需要手动调用服务才能使用。

首次编译：

```bash
sudo apt install python3-opencv python3-numpy python3-yaml python3-tk python3-pil.imagetk ros-jazzy-cv-bridge ros-jazzy-rviz2
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select camer_calibration
source install/setup.bash
ros2 launch camer_calibration hk_tag_view.launch.py
```

依赖现有 `hik_camera_ros2_driver` 已安装/编译。如果还没有编译海康驱动，可在上面的构建命令中加上 `hik_camera_ros2_driver`。默认订阅 `/camera/image`，与工作区内海康驱动保持一致；不使用普通 USB 摄像头接口。

默认相机配置已从 `hik_camera_ros2_driver-main/config` 同步到本包：`camera_params.yaml` 保留原像素格式 `BayerRG8`、曝光 40000 μs、增益 0、请求帧率 60 Hz、空 ADC 设置及原 QoS；`camera_info.yaml` 完整保留原内参与畸变。驱动仍直接复用 `hik_camera_ros2_driver_node`，不另复制一份驱动源码。

launch 默认让驱动和测距节点读取同一份内参；用 `calibration_file:=/绝对路径/新结果.yaml` 覆盖时，驱动的 `camera_info_url` 也同步到该内参。离线程序的结果格式会转换为标准 ROS CameraInfo 临时文件，数值保持一致，退出 launch 时清理。窗口运行中重新加载内参只更新测距节点；要同步驱动 CameraInfo，使用新文件路径重启本 launch。

当前未知 Tag 家族、ID 和真实边长：

- 默认 `tag_family:=auto`，尝试 `tag16h5`、`tag25h9`、`tag36h10`、`tag36h11` 四种 OpenCV 支持的家族，并显示识别出的家族和 ID。不能保证任意黑白标记都属于这四类，未识别不代表相机距离为零。
- 默认 `tag_id:=-1`，只在画面中恰好识别到一个候选时选择；多个候选不会随意取第一个，需要在窗口指定家族与 ID。
- 默认 `tag_size_mm:=0.0`，表示**尚未测量**。用尺子量打印后黑色外边框的实际一边长，不包含外围白边；不能从截图像素数推算毫米数。
- 默认使用从 `hik_camera_ros2_driver-main/config` 复制的现有 `camera_info.yaml`；数据沿用原文件，不代表本次重新完成了标定。当前图像分辨率必须与其中的 2448×2048 一致。重新标定后应改用新结果。
- 没有真实内参或实测边长时，只显示检测结果，坐标和距离为“— mm”；不会使用假定焦距或示例尺寸输出测量。

### 窗口怎么操作

1. 已有内参：点击“选择内参文件”，选择前述离线工具生成的 `calibration.yaml`；也支持 ROS `plumb_bob` 格式 YAML。填写实际 Tag 黑框边长，再点击“应用测量设置”。
2. 未做内参：把 AprilTag 换为棋盘格，在窗口底部填写真实内角点列、行和格长。改变棋盘格位置/角度/距离，逐张点击“采集一张棋盘格”。默认至少 10 张才允许计算，实际建议采集 20～30 张有代表性的照片。
3. 点击“计算内参并加载”。后台计算成功后自动加载新内参，窗口会显示结果路径。采集和结果默认保存在 `~/camera_calibration`，分别按棋盘规格和计算批次建立目录；不会清空旧数据。
4. 换回 AprilTag，填写其实际边长并应用设置，即可看坐标和距离。棋盘格格长与 Tag 黑框边长是两个不同的参数。
5. 界面里的测量参数只对当前进程有效。重新启动时可通过 launch 参数传入已确定的内参和尺寸，或再次在窗口设置。

画面显示 Tag 边框、ID、中心坐标轴；数字区域显示的是 **Tag 中心在相机光学坐标系中的位置**：

```text
X：相机右方为正
Y：相机下方为正
Z：沿镜头朝前为正（光轴方向深度）
D：sqrt(X² + Y² + Z²)，相机光学中心到 Tag 中心的直线距离
```

窗口与状态消息使用 **毫米**。这不是相机外壳表面到纸张的尺量距离，也不是相机在机械臂基座坐标系中的位置。PnP 返回物体到相机的变换，坐标约定见 [OpenCV PnP 官方文档](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)。平面标记接近正视时姿态可能出现多解/抖动；程序选择角点处于相机前方且重投影误差较小的 IPPE 解，不能用低误差承诺实际定位精度。

### 可选的 RViz 三维视图

```bash
ros2 launch camer_calibration hk_tag_view.launch.py use_rviz:=true
```

会在操作窗口之外打开 RViz，预设显示相机坐标轴、Tag 三轴、相机到 Tag 的连线、XYZ/距离文字和检测图像；ROS/RViz 长度统一使用 **米**，文字仍标毫米。普通使用只开默认操作窗口即可。

已经单独运行海康驱动时，避免重复打开同一台相机：

```bash
ros2 launch camer_calibration hk_tag_view.launch.py start_camera:=false
```

如果中文显示成方框，可安装 `fonts-noto-cjk` 并重开窗口。关闭操作窗口会结束本 launch 启动的相机和可选 RViz。

无显示器运行时使用 `show_window:=false`，通过 ROS 话题获取结果；此时不要启用 RViz。SSH 下要打开窗口，需要有效的桌面显示环境。可通过 `image_topic`、`camera_frame`、`camera_params` 指定已有图像话题、光学坐标系和海康驱动参数文件。

| 话题/服务 | 内容 |
| --- | --- |
| `/camera_calibration/debug_image` | 带检测标记和坐标轴的图像 |
| `/camera_calibration/tag_pose` | 有效目标的 PoseStamped，相机光学坐标系，平移单位米 |
| `/camera_calibration/tag_status` | JSON：有效性、原因、家族、ID、XYZ 毫米、直线距离毫米、重投影 RMS 像素、图像时间戳及后台标定状态 |
| `/camera_calibration/markers` | RViz 标记；目标丢失时删除 |
| `/camera_calibration/capture` | Trigger：按当前棋盘格参数采集一张清晰照片 |
| `/camera_calibration/calibrate` | Trigger：启动后台内参计算；返回成功只表示接受任务，完成状态见 `tag_status` |
| `/camera_calibration/reload_calibration` | Trigger：重新读取当前指定的内参文件 |

窗口上方和图像底部实时显示相机接收 FPS、检测处理 FPS、累计接收帧数，RViz 检测图也带相同叠字。FPS 采用最近 1 秒内的事件间隔计算；样本不足或超过图像超时时间未更新时归零。接收 FPS 是此 ROS 节点收到图像的速度，不是相机硬件内部帧率；处理 FPS 是实际完成图像检测的速度，不能用窗口刷新率代替。状态 JSON 同时提供 `receive_fps`、`process_fps` 和 `received_frames`。

默认处理上限 10 Hz。识别丢失、多个候选、分辨率不一致或图像超过 0.5 秒不新鲜时，窗口清空测量数值，状态 `valid=false`，不再发布新位姿，并删除 RViz 标记；`tag_pose` 历史消息不会撤回，使用者应检查图像时间戳与有效状态。程序不发布持续保留旧目标的 TF。

`max_reprojection_error_px` 默认 3 像素，只是运行时拒绝明显不良拟合的可调条件，不是精度合格标准。未知实际尺度、标定不佳、Tag 弯曲、光照或运动模糊都会影响结果。当前未验证真实 CS050 相机上的测距精度。

## 已知设备与待确认信息

目前用户提供：型号简称 **CS050**、固定对焦。完整型号、镜头焦距、采集分辨率尚待确认；本工具不会据此推断传感器规格。运行前还需要确认：

- 棋盘格每行内角点数 `board_cols`、每列内角点数 `board_rows`。
- 印刷后实测的方格边长 `square_size_mm`，单位毫米。
- MVS 保存的是原始采集尺寸，是否启用了 ROI、像素合并、缩放、翻转或旋转。
- 镜头是否是普通镜头；本程序的五参数针孔模型不适用于所有鱼眼或极广角镜头。

`config/hk_example.yaml` 中 **9×6 内角点、20 mm 格长只是示例**，不是这台设备的已知参数。照片分辨率由实际图片读取，不预设。

## 1. 标定在解决什么问题

相机把空间点投影到像素平面。已知棋盘格上各角点之间的实际距离，再从不同角度拍摄，程序便能估计“这支镜头怎样把空间中的点映射到图像上”。同一张图只能提供有限约束，因此需要多种倾斜和位置。

内参矩阵为：

```text
K = [ fx   0  cx ]
    [  0  fy  cy ]
    [  0   0   1 ]
```

`fx、fy` 是以像素表示的横、纵焦距，不是镜头上标注的毫米焦距；`cx、cy` 是主点，通常靠近画面中心，但不能直接假定就是正中心。内参描述相机自身成像特性，不是机械臂坐标。

径向畸变让画面边缘的直线出现桶形或枕形弯曲，用 `k1、k2、k3` 描述；切向畸变用于描述镜头与成像面装配偏差带来的不对称偏移，用 `p1、p2` 描述。本程序按 OpenCV 顺序输出 `[k1, k2, p1, p2, k3]`，系数无量纲。模型依据见 [OpenCV 相机模型与标定 API](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)。

重投影误差的意思是：用估计的相机参数，把棋盘格的已知空间点重新投影回照片，比较预测像素与实际检测像素相差多少。它衡量拟合程度，**不是触碰点的毫米精度**。

后续 AprilTag 用途不同：它为桌面提供有身份的定位参照。已知 Tag 尺寸、相机内参和适当的畸变处理，才能估计标记相对于相机的位置和方向。用户上传的是定位标记图，不是棋盘格；不能仅凭截图认定其家族、ID 和打印尺寸。[AprilTag 官方位姿说明](https://github.com/AprilRobotics/apriltag#pose-estimation)列出了所需的内参和实际尺寸。

内参标定完成后，依然还需另外确定相机与工具、Tag 与触碰点等坐标关系，才能用于机械臂偏差补偿。本次不实现这些步骤。

## 2. 棋盘格怎么准备

推荐使用平整、尺寸可靠的标定板。普通纸打印可以用于初步实验，但应牢固贴在平整硬板上，避免弯曲、起皱、反光和重复打印造成的尺度变化。

- **内角点不是方格数**。完整矩形棋盘格若有 10 列×7 行方格，对应 9 列×6 行内角点；程序填 `board_cols: 9`、`board_rows: 6`。
- 方格边长测量黑白交界之间的实际距离。可连续测量多格总长度，再除以格数；横向、纵向都测，多处复核。不要测整张纸的大小，也不要把角点数量当格数来除。
- 打印选择实际大小/100%，关闭“适合页面”等缩放。打印机仍可能产生缩放和横纵比例偏差，所以应以打印后的实测值为准；明显非正方形时应重新制作。
- 棋盘格外侧保留白边，拍摄时尽量保证整个棋盘格可见。不要用发光屏幕代替实体板来完成最终标定。
- 可从 [OpenCV 标定图案说明](https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html)准备棋盘格，打印后自行测量。

棋盘格用于这次估计相机成像参数；AprilTag 用于后续识别桌面参考坐标。不要把同一张 AprilTag 图当作此程序的棋盘格输入。

## 3. 用 MVS 采集照片

1. 固定相机和镜头安装，确认已有对焦适合计划工作距离。当前已知为固定对焦，采集期间不要再调对焦或变焦；也尽量保持光圈一致。
2. 在 MVS 选择实际工作分辨率，记录宽高及 ROI、像素合并等设置。整个数据集保持一致，后续 AprilTag 也使用相同成像设置。
3. 初次建议准备 **20～30 张有效标定照片**，另外留出 **5～10 张独立验证照片**，不放进标定目录。这是采集建议，不是保证准确度的张数。
4. 保持相机不动、手动改变棋盘格位置也可以，无需机械臂配合。分别覆盖中心、上下左右及四角；不能所有照片都只拍画面中心。
5. 让棋盘格有近、中、远的距离变化，主要覆盖后续桌面观测距离；同时加入绕不同方向倾斜及平面内旋转。可以从约 15～40° 的倾斜尝试，不是必须达到的角度；以角点仍完整、清晰可辨为准。避免只有正视图或同一姿态连拍。
6. 大小适中：近距离图覆盖较大画面，但不截断棋盘格；远距离仍要能清楚分辨角点。边缘图也应保留完整角点。
7. 每次等待相机和标定板静止再保存。调整照明与曝光，避免运动模糊、黑白交界过曝、亮斑和阴影。先预览放大检查；自动曝光可用于初调，正式采集尽量锁定稳定的曝光与增益。
8. 用 MVS 保存实际采集图像，优先 PNG/BMP 或无损 TIFF，不保存预览窗口截图、不缩放、不裁剪。导出已转换的常规灰度或 BGR/RGB 图像；相机原始 Bayer/打包 RAW 数据需先经 MVS 正确转换。程序读取为 8 位 BGR，不保留高位深原始测量值。

每张图片只包含一块清楚的棋盘格。具体保存菜单随 MVS 版本而异；本程序不假设 `cv2.VideoCapture(0)` 能连接海康 GigE/USB 工业相机。

## 4. 安装和目录

以下是离线照片程序的用法。Ubuntu 下使用系统 Python 即可，不需要先启动 ROS、MVS 或机械臂：

```bash
sudo apt install python3-opencv python3-numpy python3-yaml
cd ~/project/robot_and_chassis_control/ros_ws/src/camer_calibration
mkdir -p photos validation
```

然后将标定照片放进 `photos`，留出的验证照片放进 `validation`。程序只扫描目录第一层，支持 PNG/BMP/JPG/JPEG/TIF/TIFF（不区分扩展名大小写）。不建议用 JPEG 压缩图作为最终数据。

```text
camer_calibration/
├── CMakeLists.txt                  # 可选的 ROS 安装支持
├── package.xml
├── README.md
├── config/hk_example.yaml          # 离线参数，需要按实物修改
├── config/camera_params.yaml      # 从原海康包复制的采集配置
├── config/camera_info.yaml        # 从原海康包复制的现有内参
├── config/tag_view.rviz            # 可选三维视图
├── launch/hk_tag_view.launch.py    # ROS 2 实时界面入口
├── scripts/calibrate_camera.py     # 离线计算，也被后台标定复用
├── scripts/tag_pose_node.py        # ROS 图像、测距、采集与标定
├── scripts/tag_geometry.py         # 检测和几何计算
├── scripts/tag_panel.py            # 中文操作窗口
├── photos/                        # 自行建立并放入 MVS 标定照片
├── validation/                    # 自行建立并放入独立验证照片
└── results/                       # 运行时产生
```

离线脚本只依赖 OpenCV、NumPy、PyYAML。函数输入输出见代码内中文说明；主要处理链为 `load_config → scan_images/detect_corners → calibrateCamera → reprojection_error → save_comparisons/write_report`。

## 5. 配置和执行

先打开 `config/hk_example.yaml` 修改规格。**下方值仍只是示例**：

```yaml
image_dir: ../photos
output_dir: ../results
validation_dir: ../validation  # 没有独立照片时设为 null
board_cols: 9                 # 每行 9 个内角点
board_rows: 6                 # 每列 6 个内角点
square_size_mm: 20.0          # 改成打印后实测格长，毫米
min_views: 10
error_warning_px: null        # 自动提示相对偏高的图片，不自动删图
undistort_alpha: 1.0
```

YAML 的相对路径以该 YAML 文件所在目录为起点；命令行覆盖的相对路径以当前工作目录为起点。

```bash
# 在 camer_calibration 目录执行；请先修改配置中的真实标定板参数。
python3 scripts/calibrate_camera.py --config config/hk_example.yaml
```

也可以用命令行覆盖路径和规格；这些规格同样仅演示参数格式：

```bash
python3 scripts/calibrate_camera.py \
  --config config/hk_example.yaml \
  --image-dir ./photos \
  --validation-dir ./validation \
  --output-dir ./results_trial_02 \
  --board-cols 9 --board-rows 6 --square-size-mm 20.0
```

输出目录必须不存在或为空，以免覆盖结果、混用旧的角点图。第二次运行选新的输出目录即可。不可读图片或分辨率不一致会中止标定并生成错误报告，**不会自动缩放图片**。角点检测失败的照片会列入报告；其他成功照片足够时继续计算。所有检测成功的标定照片都参与拟合，高误差照片不会自动删除。

无需编译就能使用脚本。如需 ROS 命令安装入口，可以在 `ros_ws` 下：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select camer_calibration
source install/setup.bash
# 使用源码目录中的自定义配置，方便将照片与结果保存在可写目录。
ros2 run camer_calibration calibrate_camera.py \
  --config "$PWD/src/camer_calibration/config/hk_example.yaml"
```

改名后的 USB 循迹包已同步更新为 `ruier_camer_diver`；使用它前需重新构建相关包，不使用旧 `install/camer_diver` 代表新源码。海康相机仍使用 `hik_camera_ros2_driver`，没有改为 USB 驱动。

## 6. 输出文件和误差定义

| 输出 | 内容 |
| --- | --- |
| `calibration.yaml` | 原始图像 K、五参数畸变、图像宽高、总体 RMS、配置、去畸变图新 K 和有效区域 |
| `calibration.npz` | NumPy 数组形式的 K、畸变、图像尺寸和新 K |
| `report.json` | 所有图片处理状态、逐图误差、偏高/失败列表、每张标定板位姿、九宫格覆盖数量 |
| `per_image_errors.csv` | 可表格查看的逐图误差和状态；失败图的误差为空 |
| `corners/calibration/`、`corners/validation/` | 角点检测可视化；未检测到的图也保存原图用于检查 |
| `comparisons/…` | 每张成功检测照片的左原图/右去畸变图 |
| `corner_coverage.png` | 所有标定图角点在原图坐标中的叠加分布 |

尺寸顺序为 **宽、高**。空间点和标定板平移单位是毫米，旋转向量单位为弧度；每张照片的标定板到相机位姿只是标定辅助结果，**不是手眼变换**。YAML 为本工具结果格式，不直接冒充 ROS `camera_info_manager` 的 YAML。

第 i 张图的第 j 个角点，检测坐标为 `(u,v)`，预测坐标为 `(û,v̂)`：

```text
d²_ij = (u_ij - û_ij)² + (v_ij - v̂_ij)²
每张图 RMS = sqrt(sum_j d²_ij / N_i)
总体 RMS   = sqrt(sum_i sum_j d²_ij / sum_i N_i)
平均距离   = mean_j sqrt(d²_ij)
最大距离   = max_j sqrt(d²_ij)
```

三种距离结果单位都是 **像素 px**。这里 RMS 分母是角点数，不是坐标分量数；也不是 `norm(残差)/角点数`。终端会同时打印 `calibrateCamera` 返回的总体 RMS 与按以上定义复算的 RMS。算法流程见 [OpenCV Python 标定教程](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)。

`error_warning_px: null` 时，偏高误差提示阈值为 `max(2×median, median+3×1.4826×MAD, 1e-6)`；MAD 是逐图 RMS 相对中位数的绝对偏差中位数。这只是人工复查提示，不是合格标准；所有图一起变差时也可能没有相对异常项。可按实验设定显式提示阈值，仍不会自动剔除。

独立验证只使用已标定的 K 和畸变，对每张验证图用 `solvePnP` 估计棋盘格位姿，然后计算误差；**不重新拟合内参，也不加入训练集**。这仍依赖同一张图估计外参，不能当作独立的三维长度精度认证。验证照片必须另拍，不能复制训练图充数。

## 7. 判断结果是否可用

不要只看一个固定的 RMS 阈值，也不要将“计算成功”理解为“满足触碰精度”。建议依次检查：

1. 逐张打开角点图，确认每个角点都落在真实交界上、排列一致；棋盘格规格、格长和平整度应可信。
2. 查看覆盖图与九宫格数量。四角和边缘应有真实观测，且照片有多种倾斜、距离。很多张中心重复图不能替代覆盖和视角多样性；非零覆盖也不能证明视角足够。
3. 结合实际分辨率与图像清晰度比较总体/逐图 RMS，查看明显偏高图片是否有模糊、反光或错误角点。程序不自动删图；人工确认后移出训练目录，用新输出目录重算，并记录原因。
4. 看独立验证图，尤其是实际工作距离及边缘处，是否也有合理误差、直线是否更直。去畸变黑边是映射结果，不能仅凭“看起来直了”判断所有区域正确。
5. 看参数是否合理：焦距为正、主点位置符合成像设置；不同批次、多样姿态得到的参数应有合理一致性。明显异常通常要先检查数据和模型，而不是单纯增加畸变阶数。
6. 最终定位要求必须在实际桌面、距离、Tag 尺寸、工具坐标条件下用独立量测验证。简化正视针孔几何下横向尺度约为 `Z/fx` 毫米/像素，但姿态、平面位置、Tag 角点与坐标变换都会增加误差；不能用这个近似承诺实际触碰精度。

后续处理原图时使用原始 K 和畸变；若先去畸变再做 AprilTag 位姿，应使用去畸变输出对应的**新 K 和零畸变**，不能重复矫正。本程序对比图不裁剪；若以后裁剪 ROI，主点还要减去裁剪原点。更改 ROI、缩放、分辨率、变焦、对焦或镜头安装后，应核对成像几何，必要时重标。

## 8. 常见问题

| 问题 | 检查方法 |
| --- | --- |
| 检测不到棋盘格 | 是否误用了 AprilTag；内角点是否误填为格数；是否完整可见、有白边、清晰、反光少 |
| `resolution_mismatch` | MVS 是否混用了预览截图、ROI 或不同采集尺寸；按同一设置重新保存，不混合缩放 |
| `unreadable` | 是否原始 Bayer/RAW、文件损坏或不支持的编码；先经 MVS 导出常规图片 |
| 有效照片不足 | 打开失败角点图，修正规格或补拍清晰、不同姿态的棋盘格 |
| RMS 很低但实际定位不稳 | 复查训练覆盖、独立验证、工作距离、板尺寸、Tag 尺寸及后续坐标关系 |
| 去畸变边缘异常/弯曲加重 | 复查角点、尺寸、镜头模型及边缘覆盖；普通五参数模型未必适合极广角 |
| 运行很慢 | 高分辨率角点搜索和标定优化较耗时；可先用少量有代表性的原尺寸照片确认规格，不要混用缩小图 |
| 输出目录不为空 | 指定新的 `--output-dir`；原照片与已生成结果不会被自动清空 |

实际相机参数尚未由真实照片估计，本包不附带虚构的 `calibration.yaml`，也不承诺未经实测的定位精度。
