V5.4 Orbbec 相机单独拍照模块——冻结源码交付
来源：paint_rgbd_two_point_V5_4_operator_release_20261019.zip，将下列五个源码文件 原字节拷贝，没有改变传感器配置、校验规则或数据内容。
推荐：继续使用用户当前已经实机通过的一键相机命令：
/home/abc/.local/bin/paint-v54 camera
或（不依赖当前工作目录）：
/home/abc/miniconda3/envs/industrial_ai/bin/python -B /media/abc/D6A0A29CA0A2831B/visual-inspection/Unet/run_paint_rgbd_unet_v5_4.py camera --max-capture-attempts 3
两种方式均调用 run_paint_rgbd_unet_v5_4.py:camera_only()，最终在隔离的 orbbec_py39 Python3.9 解释器执行 paint_rgbd_v5_4_camera_worker.py。
包里的文件
- run_paint_rgbd_unet_v5_4.py：原字节冻结源码；真正的相机调度、重试、SDK 子进程、sha256验证。
- paint_rgbd_v5_4_camera_worker.py：原字节冻结源码；启动 Pipeline，选流 Profile，SW-D2C，同一 Frameset 拍照，写 PNG/NPY/JSON。
- paint_rgbd_legacy_auto_recapture_v4.py：原字节冻结源码； Profile 常量、参数/数组检查，7×7/XYZ 备用辅助，但 camera 模式并不运行 U-Net/XYZ。
- orbbec_rgbd_auto_capture_v1.py：原字节冻结源码；SDK Profile 选择、RGB MJPG 解码及深度帧读取。
- orbbec_d2c_capture_with_provenance_v3.py：原字节冻结源码；SDK同会话参数、镜像与内参验证。
- capture_gearbox_rgbd_camera_only_v54.py：新增的轻量专用入口，直接复用原 camera_only()，可在本包完整解压到单独目录时使用，不导入 U-Net。
- print_orbbec_v54_capture_parameters.py：新增的只读审计小工具，校验某次拍照三份原始文件 SHA256，并显示该次真实内参和深度比例。
- SHA256SUMS.txt：逐文件SHA256。
SDK 和环境（没有打包在这里）
原项目已安装：/home/abc/miniconda3/envs/orbbec_py39/bin/python 和
/home/abc/orbbec_sdk_v114_unzip/OrbbecSDK_Python_v1.1.4_linux_x64_release/python3.9。
必须保留旧 SDK 的 Pipeline、ObTypes 和依赖共享库，实际路径也应存在。
本包是相机部分的详细原始源码和入口，不是新的无依赖驱动。不要解压覆盖已验证的 V5.4 工程。
请求 Profile，注意非曝光参数
配置	来源	值
RGB	V4 RGB_PROFILE	宽640、高480、fps30、MJPG id5、profile index1
Depth	V4 DEPTH_PROFILE	宽640、高400、fps30、格式id11、profile index16
对齐	worker 配置	OB_PY_ALIGN_D2C_SW_MODE，输出深度640×480
预热	调度器传参	跳过前5组完整 Frameset
采集等待	调度器/worker	最多30次，每次750ms
时间戳差	worker 默认	raw差值 <=100，为诊断上限，未独立认证单位/硬件同步
深度比例	SDK depthFrame.getValueScale()	每次现场读取，此前实测为1 mm/raw
内参和畸变	SDK pipeline.getCameraParam()	每次现场读取，写入同会话 JSON
曝光/增益/白平衡/激光	当前代码未设置也未记录这些实测控制项	未知，不能臆测


拍摄文件
相机启动后生成：
Unet/runs/paint_rgbd_v5_4_camera_smoke/camera_v5_4_*/attempt_01/
    rgb_001.png                         # RGB照片，注意cv2文件像素内部BGR顺序
    d2c_depth_raw.npy                   # D2C对齐的uint16深度，不能直接当伪彩色图片
    camera_parameters_same_session.json # 本次SDK相机内参、畸变、镜像原始记录
    capture_manifest.json               # 深度比例、XYZ实际用内参、RGB/深度时间戳、SHA256
    CAPTURE_COMPLETE.json                # 完成标记和manifest哈希
若遇解码错误，采集器可能重新申请同一会话的下一完整Frameset，或上层重新调用worker（最多3次）。只有看到 CAMERA_ONLY_RGBD_PASS_NO_UNET_NO_ROBOT 和退出码0才算拍照成功；并不证明真实配准误差或连续30FPS。
查看真实一次拍照参数
复制 camera 输出中的 OUT: /.../attempt_01，执行：
/home/abc/miniconda3/envs/industrial_ai/bin/python -B print_orbbec_v54_capture_parameters.py --capture '/完整路径/attempt_01'
这个检查只读文件，不连接相机，不改动模型，也不修改检测点。旧相机的真实参数应以 OUT 对应会话的 JSON 为准。
可选：另存可看的深度伪彩图片
原始 d2c_depth_raw.npy 是数值数据，不是可直接查看的 JPEG/PNG。
render_orbbec_v54_depth_preview.py 可在不改变原始深度文件的前提下创建伪彩 PNG：
/home/abc/miniconda3/envs/industrial_ai/bin/python -B render_orbbec_v54_depth_preview.py \
  --capture '/完整路径/attempt_01' \
  --out '/home/abc/桌面/depth_preview_NOT_METRIC.png'
预览图每帧按有效深度的第 5、95 百分位拉伸，不可从图片颜色推断绝对毫米，真正深度以 NPY×manifest 的比例为准。输出目录必须存在，禁止覆盖已存在的预览文件。
