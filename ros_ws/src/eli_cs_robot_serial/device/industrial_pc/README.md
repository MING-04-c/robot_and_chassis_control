# 工控机（Industrial PC）检测结果通信

树莓派发送 RGB-D 采集包到主机仍使用 `geminipro_camer_calibration/capture_transfer.py` 的
RGBD/ZIP 通道。本目录新增独立的**主机 → 树莓派结果通道**；不启动相机、机器人驱动、
坐标转换或运动任务，不调用 `detect_controller/start`。

## 已提供文件的结论

`漆膜_Pi远程检测结果` 中实际结果是 `HOLD_NO_VALID_3D_POINTS`，原因是
`HOLD_MULTIPLE_SIZEABLE_PAINT_REGIONS`，三维点数组为空，不是成功检测。
`DEMO_XYZ_800mm.json` / `DEMO_XYZ_1200mm.json` 中分别有两个演示点，深度是假设值。
保留原文件，不修改 HOLD 状态、不把演示数据改称实测数据。
WPS 文档也明确这些演示坐标不能当作正式测量或机器人运动指令。

现有 `result_tcp_message.json` 的 protocol、payload、SHA256、HMAC 四字段保留。
其 payload 规范化后的 SHA256 与本实现一致；原文件的共享密钥未提供，不能验证其 HMAC。
主机应使用下面的双方共享密钥和本目录工具生成签名，不能用任意密钥验证旧签名。

## 启动树莓派接收端

```bash
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select eli_cs_robot_serial
source install/setup.bash

# 仅首次执行；已有文件会报错而不会覆盖。密钥不要提交到 Git。
ros2 run eli_cs_robot_serial industrial_pc_client.py keygen \
  --key-file ~/.config/robot_control/industrial_pc.key

ros2 launch eli_cs_robot_serial industrial_pc.launch.py
```

默认 `0.0.0.0:9010`，可通过 `bind_host:=树莓派网口IP port:=9010` 修改。
`key_file:=... output_dir:=... config_file:=...` 支持 launch 覆盖；留空遵循 YAML。
端口与原来的照片上传、力传感器 TCP 端口独立。两台机器的网络地址必须能互通。
`key_file` 留空字符串时可改用环境变量 `INDUSTRIAL_PC_SHARED_KEY`；内容至少 32 字节，
按 UTF-8 读取并去掉首尾空白，双方必须一致，不能在两台机器分别生成不同密钥。

工控机地址已确认：`169.254.199.120`。照片上传的 `--host` 使用此地址；
结果回传客户端的 `--host` 使用树莓派地址，两者不能混用。

## 主机模拟回传

主机只需 Python 3，无需 ROS 或相机 SDK。把 `src/industrial_pc_client.py` 和
`src/industrial_pc_protocol.py` 放在同一目录，并将树莓派生成的密钥文件安全复制到主机。
安装目录也提供同一份工具：`share/eli_cs_robot_serial/device/industrial_pc/host_tools/`。

```bash
# 主机执行，替换树莓派 IP、密钥路径和演示 JSON 的实际路径。
python3 industrial_pc_client.py demo \
  --host 树莓派IP --port 9010 \
  --key-file industrial_pc.key --file DEMO_XYZ_800mm.json
```

`demo` 保留原始 schema、深度来源与演示标志，只补消息 ID、处理时间及签名。
同一次调用内断线重试复用同一 ID 和报文；再次独立调用代表一条新演示消息。
联调期间也可以在树莓派上 `ros2 run eli_cs_robot_serial industrial_pc_client.py demo ...`。

实际 HOLD 文件可用未封装的 `result_human_readable.json`：

```bash
python3 industrial_pc_client.py send --host 树莓派IP \
  --key-file industrial_pc.key --file result_human_readable.json
```

`send` 接受 payload 文件并签名，或发送已经正确签名的 `result_tcp_message.json`。
已经封装的文件会先验证原签名，不会悄悄重新签名。主机处理程序也可直接导入
`envelope / write_message / read_message / authenticate`，参考 client 的调用顺序。

## TCP 协议 v1

- 树莓派是 TCP 服务端，主机是客户端；默认端口 9010。
- 每帧：`uint32 大端 JSON字节长度 + UTF-8 JSON`。长度不包括四字节头，最大 262144 字节。
- 不是换行分隔，不带 RGBD 魔数；同连接可连续发多帧，每帧等待一次 ACK。支持 TCP 拆包/粘包。
- socket 默认空闲超时 5 秒；主机应在 ACK 丢失时重发**相同消息 ID 和相同 payload**。
- 单个结果最多 100 点，维持主机的点顺序；通信层不负责路径规划。
- HOLD 是业务结果，接收成功仍返回 `ok:true`，不意味着有点或能运动。

封装：

```json
{
  "protocol": "paint_v54_camera_coordinates_tcp_v1",
  "payload": {"schema": "...", "message_id": "..."},
  "payload_sha256": "64位小写十六进制",
  "hmac_sha256": "64位小写十六进制"
}
```

签名算法：`body = json.dumps(payload, ensure_ascii=False, sort_keys=True,
separators=(',', ':'), allow_nan=False).encode('utf-8')`；
`payload_sha256 = SHA256(body)`；`hmac_sha256 = HMAC-SHA256(共享密钥, body)`。
ACK 使用相同封装/长度头和签名方式。其他语言要产生完全相同的 body 字节，建议先用随附 Python 工具对接。

### 正式测量 payload 示例（仅说明格式，不是现场测量）

```json
{
  "schema": "paint_v54_pi_remote_result_v1",
  "message_id": "本次结果的唯一ID",
  "status": "OK_3D_POINTS",
  "capture_name": "本次采集组名称",
  "capture_manifest_sha256": "原始采集清单文件的64位SHA256",
  "coordinate_frame": "CAMERA_PINHOLE_PRELIMINARY",
  "depth_source": "MEASURED",
  "observed_at_ros": {"sec": 1790000000, "nanosec": 123456789},
  "points_xyz_camera_mm": [
    {"point_id": "P1", "uv": [390, 370], "xyz_camera_mm": [107.47, 217.487, 800.0]}
  ],
  "robot_base_coordinate_calibrated": false,
  "robot_motion_authorized": false,
  "robot_command": null
}
```

上例数值来自演示，正式发送时必须换成实际深度算出的点和真实采集关联信息。
`capture_manifest_sha256` 必须是 64 位小写十六进制，接收端只核验格式并记录，
当前不据此自动匹配或授权某次机器人任务。
`observed_at_ros` 应回传树莓派照片的原始 ROS 采集时间，不能用主机处理时间替代。
当前旧文件没有此字段，仍可接收，但发布的点时间戳为零，不能直接用于运动转换。
真实采集时间与拍照时机械臂位姿的关联，必须在后续采集/规划集成时补齐。

HOLD 沿用原 schema 和 `HOLD_...` 状态，点数组必须为空。
DEMO 沿用 `paint_v54_xyz_demo_v1`、`demo_only:true`、
`status:DEMO_ONLY_NOT_VALID_MEASUREMENT`，不能走正式点话题。
三类结果均要求 `robot_motion_authorized:false`、
`robot_base_coordinate_calibrated:false`、`robot_command:null`。

ACK payload 示例：

```json
{
  "schema": "paint_v54_pi_result_ack_v1",
  "ok": true,
  "message_id": "对应请求ID",
  "disposition": "stored",
  "kind": "demo",
  "point_count": 2,
  "published": true,
  "robot_motion_authorized": false
}
```

`stored` 表示结果已保存且已调用相应 ROS 发布接口；不保证订阅方收到或机器人接受。
重复报文返回 `duplicate`、`published:false`，不会再次发布点；重启接收节点后仍按文件去重。
相同 ID 不同内容、签名错误、非法坐标或格式返回 `ok:false` 和 `reason`。
失败 ACK 不作为任务停止或运动完成确认。旧结果需要重新处理时必须显式形成新的结果 ID。

## ROS 接口及坐标

| 话题 | 类型 | 用途 |
|---|---|---|
| `/industrial_pc/result` | `std_msgs/String` | 最新完整 payload，包含 ID、原始毫米坐标及检测状态 |
| `/industrial_pc/status` | `std_msgs/String` | LISTENING / RECEIVED / DUPLICATE / REJECTED |
| `/industrial_pc/camera_points` | `geometry_msgs/PoseArray` | 正式测量观测，位置换算成米 |
| `/industrial_pc/demo_camera_points` | `geometry_msgs/PoseArray` | 演示点，位置换算成米，与正式点隔离 |

result/status 保留最新值，迟加入的订阅者使用 transient_local QoS 读取。
点话题为普通可靠发布，不在重连时自动回放。姿态四元数为单位四元数占位，
**协议未提供接触朝向**，不能当成已确定的探头朝向。

光学坐标约定：X 向图像右、Y 向下、Z 向前。默认 frame_id 是驱动的
`camera_color_optical_frame`；`terminal_link_1` 是模型的相机安装 joint，不应假设等同。
本次不把点直接接到 `/geminipro_calibration/camera_points` 或检测控制器。
以后接入 tool0 转换需确认：光学原点到 Joint_1 的实测外参、采集时的机器人位姿、
关联的采集清单和接触姿态。延迟返回结果也不能套用接收时的最新机械臂位姿。

```bash
ros2 topic echo /industrial_pc/status --qos-durability transient_local
ros2 topic echo /industrial_pc/demo_camera_points
```

落盘目录默认 `~/robot_data/industrial_pc/results/`，每个文件含原始签名消息、
接收时间、对端 IP、结果分类。文件名是消息 ID 的 SHA256，避免路径字符影响存储。
密钥不进入消息、日志或结果文件。

## 海康照片独立传输

海康拍照通信的唯一实现已移入本目录：`src/industrial_pc_hk_transfer.py`。
旧 `robot_task_control/hk_image_transfer` 入口仍可用，转到同一实现；旧主机接收工具路径保留。
两种入口不能同时绑定同一个 9002 端口。

| 通道 | 数据方向 | TCP 连接方向 | 端口 / 格式 |
|---|---|---|---|
| 深度采集包 | 树莓派 → 工控机 | 树莓派连接工控机 169.254.199.120 | 9030 / RGBD + ZIP |
| 深度检测结果 | 工控机 → 树莓派 | 工控机连接树莓派 | 9010 / 长度头 + 签名 JSON |
| 海康照片 | 树莓派 → 工控机 | 工控机 169.254.199.120 连接树莓派 | 9002 / MIMG |

三个独立 TCP 连接，文件和结果不混包。USB 循迹视频保持原有流程，不从此通道发送。
深度上传继续按已有 `capture_transfer.py` 命令执行，此 launch 不会自动扫描或上传深度文件。

树莓派只启动结果通信与海康照片传输：

```bash
ros2 launch eli_cs_robot_serial industrial_pc.launch.py start_hk_transfer:=true
```

已运行机械臂任务并启用了原海康传输节点时，沿用该节点即可，不重复启动
`start_hk_transfer:=true`。原整套任务的 `image_receiver_ip` 默认值已更新为
`169.254.199.120`，`image_listen_ip` 默认 `0.0.0.0`。
新海康配置在 `config/hk_transfer.yaml`；可覆盖 `hk_bind_host:=树莓派IP`、
`industrial_pc_ip:=169.254.199.120`、`hk_port:=9002`。
`industrial_pc_ip` 是海康连接允许的主机 IP，不是本机监听地址。

工控机接收照片（无需 ROS，仅接收保存不需 OpenCV）：

```bash
python3 industrial_pc_hk_receiver.py --host 树莓派IP --port 9002 --save-dir ./hk_photos
```

此脚本在 `src/` 和安装后的 `host_tools/` 下。
主机保存每张照片和同名 JSON 元数据，包含原始拍照帧时间戳、序列号、尺寸、话题和坐标系。
先连接接收器，再触发原来的拍照接口：

```bash
ros2 service call /dataset_capture/capture std_srvs/srv/Trigger '{}'
```

拍照服务需要已有海康驱动与 `dataset_capture_node.py`；通信 launch 不会替你启动这两个节点。
只订阅 `/dataset_capture/captured_image`，不会把 `/camera/image` 的连续预览全部发出去。
照片仍由原拍照节点保存在树莓派，本通道读取对应拍照输出发送。

### 海康 MIMG 帧格式及交付边界

沿用已有接收器：`MIMG` 四字节魔数 + uint32 大端 JSON 头长度 + UTF-8 JSON 头 + 图像字节。
默认头部 `version:1`、`encoding:jpeg`、`payload_length` 表示后续照片字节数，
`stamp_ns` 是原始图像时间戳。JPEG 默认质量 95，由拍照话题图像重新编码，
不是对本地保存 JPEG 文件的逐字节拷贝；原始像素可配置 `image_format:raw`（version 2）。

该兼容通道按允许的主机 IP 接入，**没有结果通道的 HMAC 和应用层保存 ACK**。
`/hk_image_transfer/ready` 只表示接收器已连接且照片队列有空间，不代表主机已成功落盘。
队列满、无接收器或进程退出时不能保证自动补传；原照片仍保留在树莓派拍照目录。
MIMG 不用于回传三维点，主机的检测结果应单独走 9010，不能发到照片端口。
# 最新漆膜厚度接口

在现有签名 TCP 9010 通道增加只读请求，无需增加端口：

```json
{"schema":"paint_thickness_latest_request_v1","message_id":"thickness-query-001"}
```

仍使用原有长度头、SHA256 和 HMAC 封装。ACK schema 保持
`paint_v54_pi_result_ack_v1`，回显 message_id，新增 `latest_thickness`：

```json
{"available":true,"unit_confirmed":true,"age_seconds":0.2,"measurement":{"schema":"paint_thickness_measurement_v1","sequence":12,"value":123.5,"unit":"um","received_at_ros":{"sec":1790000000,"nanosec":0}}}
```

以上数字仅为格式示例。未收到读数为 `{"available":false,"measurement":null}`。
只传设备最新有效回包的最后一个数，不传历史数组。接收时间为树莓派串口回包 ROS 时间，
不是设备测量时间；单位 unknown 表示未确认，数值不换算。age_seconds 原样显示时间差，
没有自动丢弃旧读数，也不把查询当成新测量。普通检测结果 ACK 也附带相同最新快照。
快照来自 `/detactor/measurement`，不代表与刚提交的相机检测点已完成逐点匹配。

主机命令（使用更新后的客户端）：

```bash
python3 industrial_pc_client.py thickness --host 树莓派实际IP --port 9010 --key-file industrial_pc.key
```

仪器单独启动：`ros2 launch eli_cs_robot_serial detactor.launch.py auto_request:=false`；
采集：`ros2 service call /detactor/acquire std_srvs/srv/Trigger '{}'`；
本地读取缓存：`ros2 service call /detactor/latest_measurement std_srvs/srv/Trigger '{}'`。
真机检测任务默认包含仪器启动，已有驱动时设 `start_thickness_detector:=false`。

