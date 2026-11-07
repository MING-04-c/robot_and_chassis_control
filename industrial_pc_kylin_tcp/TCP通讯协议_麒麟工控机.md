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

## 检测后测厚结果（串口仪器，TCP回传）

保持 `python3 listen_robot.py` 常驻。9010状态应答的 `robot_status.detection_results` 为最近100个逐点结果，schema=`paint_detection_point_result_v1`。字段包含result_id、batch_id、point_index、point_id、run_id、root_run_id、retry_count、capture_manifest_sha256、success、returned_home、measurement（原 `paint_thickness_measurement_v1`，含value/unit/sequence/received_at_ros）。只有实际回到原位后才产生此结果；success=false时measurement可为null，不用旧值填补。监听端自动按result_id去重，保存到output_dir/detection_results/并打印状态。服务端持久化后可在重连或监听重启时补收最近100条；更早记录保留在树莓派磁盘。

未接触/不可达时，树莓派安全回位后最多重新拍照取点3次。重拍必须返回新清单哈希对应的检测点，同一个物理点的point_id在整个任务中保持一致；已成功的点不重复执行。`send_points.py`用法不变。

## 底盘电量与充电状态（9010）

树莓派运行现有底盘驱动及`industrial_pc.launch.py`，或使用已经包含TCP桥的任务入口。麒麟保持`python3 listen_robot.py`运行，终端会显示`电量=73% 充电=充电中`；`python3 start_task.py --action status`可单次查询。这里的73%是格式示例，实际值来自底盘。监听快照保存在output_dir下的`.listener_status.json`。

状态应答新增`robot_status.chassis_power`，沿用原9010帧格式，无需新端口：

```json
{
  "available": true,
  "connected": true,
  "battery_percent": 73,
  "charging": true,
  "charging_state": "charging",
  "low_battery": false,
  "voltage_v": 48.2,
  "current_a": -1.5,
  "bms_fresh": true,
  "system_fresh": true,
  "charging_status_layout": "bitfield32",
  "system_status_flags": 8
}
```

以上全为示例。`battery_percent`是0x206 byte[0]的0～100百分比。`charging=true/false/null`分别表示充电中/未充电/未知；`charging_state`对应`charging/not_charging/unknown`。没有消息、底盘失联、SOC越界或数据超时，相关值返回null，不能把null当0%或未充电。BMS新鲜期限3秒、系统状态1秒、连接状态3秒，同时核对ROS消息源时间戳和本机接收时间。`available`要求电量与系统状态同时有效，两个`*_fresh`字段允许分别判断。

**充电布局存在版本差异。** 用户提供的协议截图把0x204 byte[0..3]定义成uint32状态码，bit3充电、bit2低电量；TCP默认`charging_status_layout=bitfield32`按此解析。当前运动驱动的SDK则把byte[0]作为状态枚举、byte[1]作为分立标志，包含手动充电和桩充电。这一版本选`byte_flags`，由`manual_charging || dock_charging`决定。原始uint32值始终放在`system_status_flags`供核对；此次仅增加只读遥测，不改变底盘运动驱动的协议和安全逻辑。实际接上充电器后应核对一次变化，确认截图适用于这台底盘。

树莓派可在`device/industrial_pc/config/industrial_pc.yaml`修改布局，或单独启动桥时传参：

```bash
ros2 launch eli_cs_robot_serial industrial_pc.launch.py charging_status_layout:=bitfield32
# 只有实际固件使用SDK的Byte1标志定义时改成byte_flags
```

已有任务加载TCP桥时无需重复启动。桥接只订阅`/masc_chassis/bms_status`、`/masc_chassis/chassis_status`和`/masc_chassis/comm_status`，不创建第二个CAN接收程序，不发送底盘命令。运动任务启动时不用额外调用采电量服务，麒麟监听器按原状态查询周期自动获取。

## 正常/异常分开的运行状态监视

树莓派监视器集成于9010 TCP桥，额外提供两个只读ROS话题：`/robot_status/normal`和`/robot_status/abnormal`（0.5秒刷新、各保留最新一条）。不创建第二个CAN连接，不触发运动或自动清除报警。

联合任务`line_follow_arm_test.launch.py`默认加载状态桥，仍按原`auto_start`设置运行。其他任务已有桥时直接使用；单独需要监视时执行`ros2 launch eli_cs_robot_serial industrial_pc.launch.py`，但不要同时启动两个占用9010端口的桥。联合任务可用`enable_status_monitor:=false`关闭默认状态桥；`remote_control:=true`仍会加载桥以接收主机命令。

麒麟替换新版工具包，执行：

```bash
python3 monitor_status.py
# 只看其中一路：
python3 monitor_status.py --channel normal
python3 monitor_status.py --channel abnormal
# 单次读取后退出：
python3 monitor_status.py --once
```

默认每周期发送**两个独立的只读请求/应答**，复用9010端口与原有长度头、SHA256、可选签名，不增加端口。请求体如下，message_id每次生成：

```json
{"schema":"paint_robot_status_request_v1","message_id":"example-normal-1","channel":"normal"}
```

异常一路使用`channel:"abnormal"`。应答仍是原`paint_v54_pi_result_ack_v1`，分别带`normal_status`或`abnormal_status`，两个字段不会混在这个分路应答中。旧的不带channel的状态请求继续兼容，在`robot_status`下附带两种快照；旧`listen_robot.py`也能显示故障码并保存两个最新状态文件。

正常数据schema为`robot_normal_status_v1`，包括monitor_id、sequence、updated_at（UTC）、state、healthy、workflow、run_id、phase、action、step_index/step_count、posture、drive_direction、line_state、arm_state、charging_dock、wait_seconds_remaining、power及各来源freshness。state包括unknown（未收到/失联）、initializing、idle、running、waiting、stopped（明确的人工停止）、completed、exception（结构化异常）、attention（近期错误日志）。未知状态不会标成正常或完成，healthy只反映已经监视的反馈，不替代硬件自检。

异常schema为`robot_abnormal_status_v1`，包括`has_fault`（结构化故障）、`has_abnormal`（结构化故障或近期ROS错误）、`active_faults`、`events`、`recent_error_logs`。每项给出`error_code`、source、message、native_error_code；项目码使用`robot_project_v1`命名空间。以下是**格式示例**，不代表当前设备故障：

```json
{"error_code":"E2101","source":"arm_joint_postures","message":"轨迹规划失败","native_error_code":-1,"severity":"error"}
```

`error_code`是本项目统一分类码，**不是厂家报警编号**。native_error_code仅携带明确提供的原始值：底盘chassis_fault位图、CAN fail_reason、机器人safety_mode、MoveIt规划/执行error_code等；没有原始码时为null。ROS ERROR/FATAL原文以E9001事件上报，带节点名、文件、函数和行号，不能凭日志猜一个厂家码。

| 项目码 | 含义 |
|---|---|
| E1001 | 任务执行失败 |
| E1002 | 任务状态失联 |
| E2001 | 机器人安全锁定 |
| E2101 | 机械臂关节任务失败 |
| E2102 | 机械臂任务状态失联 |
| E3001 | 循迹或倒车异常 |
| E3002 | 循迹状态失联 |
| E3101 | 倒车充电异常 |
| E4001 | 底盘故障 |
| E4002 | 底盘通讯异常 |
| E4003 | 底盘状态失联 |
| E9001 | ROS错误日志 |
| E9901 | 工控机TCP连接失败 |
| E9902 | 异常事件历史超出缓冲 |

报警使用`raised/cleared`事件和唯一event_id。相同故障持续存在不会每次生成新事件；明确正常反馈解除条件时发送cleared。cleared表示监视条件解除，不执行硬件解锁或旧任务续跑。已知安全锁定和底盘故障在反馈中断后仍保留，同时报告失联。可选的海康/深度相机未启动不会凭空报缺失；循迹/关节反馈的缺失结合当前步骤判断，并给初次数据3秒宽限。过期任务进度不显示为当前进度。

异常事件缓冲最近50条、近期ROS错误缓存20条（近期窗口15秒）；日志只是一条发生记录，不推断其已经恢复。高频重复相同日志合并。主机用monitor_id+event_id去重；sequence变化可追踪事件顺序。进程重启会更换monitor_id。断线后补收缓冲内事件，超出缓冲提示E9902，不能承诺无限历史或所有进程崩溃前的日志都能送达。

麒麟文件在config.json的output_dir下另建`status/`，不混入hk/RGB照片目录：

- `normal.json`：最新正常/运行进度快照。
- `abnormal.json`：最新异常快照。
- `events.jsonl`：监视器接收的增量异常/恢复/错误日志事件；5MiB轮换，保留一个`events.previous.jsonl`。
- `.monitor_cursor.json`：已处理事件游标，正常重连/重启不重复保存事件。异常断电窗口可按event_id再次去重。

TCP失联时麒麟显示E9901，并将当前进度写为unknown，不能继续把旧状态当当前状态。此时取不到树莓派的原始错误码，仅能提供本机socket errno。`monitor_status.py`完全只读；启动/停止仍使用原`start_task.py`。可与照片监听`listen_robot.py`同时运行。
