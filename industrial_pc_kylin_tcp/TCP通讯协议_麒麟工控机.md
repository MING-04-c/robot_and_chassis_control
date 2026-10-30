# 树莓派与麒麟工控机TCP协议

树莓派169.254.199.100，麒麟169.254.199.120。当前默认无密钥，两端更新后直接运行；保留长度检查、SHA256、超时、重连与指令去重。

## 启动

Pi仅相机：`ros2 launch robot_task_control camera_capture_remote.launch.py`。
Pi完整任务：`ros2 launch robot_task_control line_follow_arm_test.launch.py remote_control:=true`。
二者只运行一种，已包含9010通讯；9002由拍照流程按需开启。

麒麟先 `python3 listen_robot.py`，另一终端 `python3 start_capture.py` 单独采集，或 `python3 start_task.py` 开始已加载的完整任务。更多命令见工具包QUICKSTART.md。

## 9010：命令、检测结果、状态

麒麟主动连接Pi:9010。每帧为uint32大端JSON字节长度，后跟UTF-8 JSON，大小1～262144字节；必须处理TCP拆包。

封装：

```json
{
  "protocol": "paint_v54_camera_coordinates_tcp_v1",
  "payload": {"schema":"paint_robot_task_request_v1","action":"capture","message_id":"capture-001"},
  "payload_sha256": "规范payload字节的64位SHA256"
}
```

规范字节算法：Python `json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')`。SHA256必须匹配；默认不包含hmac_sha256，也不读取密钥文件。JSON重复字段、NaN/Infinity、超长消息拒绝。

可选旧模式仅供兼容：两端明确启用use_shared_key=true后，按原格式追加HMAC-SHA256并使用同一key_file；不能混用模式。当前交付config和launch默认均关闭。

### 固定动作

| payload.schema | 字段 | Pi行为 |
|---|---|---|
| paint_robot_task_request_v1 | action=start，message_id | 调用/task_controller/start |
| 同上 | action=capture，message_id | 调用/task_controller/capture，只采图，不启动运动 |
| 同上 | action=resume，message_id | 当前有效途中站才发布/line_follow/resume |
| 同上 | action=stop，message_id | 调用/task_controller/stop |
| paint_robot_status_request_v1 | message_id | 返回缓存任务/循迹/最新厚度和接口就绪情况 |
| paint_thickness_latest_request_v1 | message_id | 返回最新一次厚度，不发送历史数组 |

服务受理后立即回ACK，采集/运动异步执行，不在TCP连接中等整轮完成。ACK payload.schema=paint_v54_pi_result_ack_v1，含ok、message_id、message/reason等。capture成功回执中的robot_motion_authorized=false。服务就绪不代表全部硬件就绪。

同一message_id同一内容持久去重，重连或通讯节点重启不重复执行；同ID不同内容拒绝。服务超时可能返回execution_unknown=true，应查询状态，不换新ID盲目重发。任务和单独采集互斥；接口不能传shell命令或任意ROS服务。

### 检测结果

继续支持paint_v54_pi_remote_result_v1与明确标记DEMO的旧格式。用 `python3 send_points.py --file result.json` 校验并发送，无需手动拼包。

正式结果坐标为CAMERA_PINHOLE_PRELIMINARY下的相机光学坐标，单位毫米：X向右、Y向下、Z向前。示例仅说明结构：

```json
{
  "schema":"paint_v54_pi_remote_result_v1",
  "message_id":"capture-001-result-001",
  "coordinate_frame":"CAMERA_PINHOLE_PRELIMINARY",
  "capture_manifest_sha256":"本次原始采集清单哈希",
  "status":"OK_3D_POINTS",
  "depth_source":"MEASURED",
  "points_xyz_camera_mm":[{"point_id":"P1","xyz_camera_mm":[100,50,800]}],
  "robot_motion_authorized":false,
  "robot_base_coordinate_calibrated":false,
  "robot_command":null
}
```

清单哈希从照片根目录 `.capture_index.json` 中对应任务的rgbd.capture_manifest_sha256读取。相机内参及深度比例也保存在同一索引。通讯桥收到点后仅保存和发布观测。若已启动camera_detection_real的TCP测试任务，它会使用拍照时冻结位姿，并在本次清单哈希匹配后自动触碰；其他流程不因收点自动运动。DEMO不能触发真实触碰。

## 9002：海康照片

麒麟主动连接Pi:9002。MIMG四字节标识 + uint32大端JSON头长度 + JSON头 + payload_length字节图像。当前任务为JPEG一张；头含version=1、encoding=jpeg、sequence、stamp_ns、width/height、payload_length、payload_sha256、capture_set_id、capture_index=1。

capture_set_id为后台任务编号；capture_name为跨通道配对名称，例如2026.10.30.20.29.12_345；它不是额外的目录层级。grouping=task_final_one_hk表示本轮只有一张，不等第二张HK。9002采完才开放，之前waiting_sender为正常等待。

接收器校验长度及存在时的照片SHA256，保存到 `2026.10.30.20.29/hk/20.29.12_345_hk.jpg`。旧的双HK流仍可接收，第二张文件后缀为 `_hk_02.jpg`。原HK通道无落盘ACK，以麒麟“保存完成”和实际文件为准。

## 9030：深度相机RGB-D

Pi主动连接麒麟:9030。RGBD四字节标识，后跟uint32大端JSON头长度及JSON头（version=1、size、sha256），再跟size字节ZIP，最大128MiB。

传输内部继续使用既有V5.4格式，同一SDK FrameSet的RGB/原始深度和参数，以原清单和哈希校验。接收端校验ZIP哈希、路径、文件大小和清单后，仅保留RGB和原始16位深度PNG；ZIP、NPY和多份清单不在照片目录重复保存。后台索引保留本次参数、原始清单及哈希。保存成功后用uint32长度头JSON回 `{ok:true, groups:1, sha256:..., saved_to:..., duplicate:false}`；失败ok=false，仍继续监听。

重复包按ZIP哈希去重，并检查已保存图片哈希，不新增一套目录。ZIP只作临时传输，不长期保留。`*_depth.png`原始像素值不变，Z毫米=像素值×depth_scale_mm_per_raw，比例从后台索引取，不硬编码1。

## 保存位置

两端使用相同的年月日时分目录，下面直接是hk与RGB；同一分钟的多轮图像用秒和毫秒文件名前缀区分：

```text
2026.10.30.20.29/
  hk/
    20.29.12_345_hk.jpg
  RGB/
    20.29.12_345_rgb.png
    20.29.12_345_depth.png
```

原日期/时间布局的旧索引仍可读取，不自动移动或覆盖旧图；新采集使用上面的目录。同一采集重发不新增文件。时间前缀相同但采集编号不同会追加编号，避免覆盖。后台索引通过paths记录实际图像路径，逻辑名hk.jpg/rgb.png/depth.png不再等于文件名。

Pi根目录：~/robot_data/task_final/。麒麟根目录：工具目录/received/。

.capture_index.json集中保存关联、内参、深度比例、原清单哈希和去重索引；.listener_status.json或.capture_status.json保存当前状态。均为后台隐藏文件，不放在每轮照片目录中。两台相机顺序曝光，目录关联不表示同步曝光。

旧接收目录整理：`python3 capture_storage.py --source received --output received_clean`，只复制三图及必要参数，不删除源文件。
