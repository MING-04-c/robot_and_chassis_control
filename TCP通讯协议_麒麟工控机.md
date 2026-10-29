# 麒麟工控机与树莓派 TCP 通讯协议及分步测试

版本：v1；整理日期：2026-10-27。本文供工控机端开发和联调使用，与项目现有代码一致。

已确认工控机 IP：**169.254.199.120**。树莓派的实际网口 IP 还需现场确认，
下文的 `树莓派实际IP` 必须替换，不能填成本机工控机的 169.254.199.120。
不预设麒麟版本、CPU 架构或网卡名，不修改网卡配置，也不需要 ROS、相机 SDK 或管理员权限。

## 交付文件与运行环境

项目根目录提供：

- `industrial_pc_network_test.py`：先检查网口，再测试 TCP 连接。
- `TCP通讯协议_麒麟工控机.md`：本协议。
- `industrial_pc_kylin_tcp.zip`：可整体复制给工控机的工具包，解压后即可运行。

工具包额外包含现有的三个通讯工具与共享协议模块：

- `industrial_pc_hk_receiver.py`：接收海康照片并保存。
- `capture_transfer.py`：接收深度采集 ZIP 包，校验后返回 ACK。
- `industrial_pc_client.py`、`industrial_pc_protocol.py`：发送检测结果，校验 ACK。
- `DEMO_XYZ_800mm.json`、`DEMO_XYZ_1200mm.json`、`HOLD_result.json`：联调用例。

建议 Python **3.8 或以上**，先执行 `python3 --version`。这些命令只使用标准库，
不需 pip 安装依赖；海康接收器可选的 `--show` 才需要 OpenCV/NumPy，本次不用该参数。
网口检查使用 Linux sysfs、`ip` 和 socket；没有 `ip` 命令时降级为 ioctl 查询主 IPv4。
工具已在当前 Ubuntu 环境用回环网络验证，**尚未在目标麒麟系统实测**。

## 一、先确认能不能调用网口

在麒麟工控机终端，进入工具所在目录：

```bash
python3 industrial_pc_network_test.py interfaces --report network_interfaces.json
```

工具按顺序检查：

1. 列出网卡名、IPv4、接口状态与载波状态。
2. 查找配置了 `169.254.199.120` 的网口。
3. 检查接口启用、是否有链路；状态不完整时明确提示。
4. 创建 `AF_INET/SOCK_STREAM` TCP socket 并绑定本机 IP 的临时端口。

出现 `[PASS] 调用网口` 表示系统允许程序使用该本机地址创建 socket，
**还不能证明对端已连接、应用协议正常或照片能收到**。

若有多个网口，显式指定实际网卡名，不能假设必然叫 eth0：

```bash
python3 industrial_pc_network_test.py interfaces \
  --interface 实际网卡名 --local-ip 169.254.199.120 --report network_interfaces.json
```

若不知道网卡名，第一条命令即使最后失败也会先列出所有可查询网卡。
也可只读查看：`ip link show`、`ip -4 addr show`。
若没有该 IP，先由现场人员核对网线、网口、IP/掩码及网络配置；工具不会替你修改。

## 二、再确认能不能连接树莓派

树莓派先启动通讯服务。共享密钥需要按后文准备；已经准备过则不重新生成：

```bash
# 树莓派执行，不是在麒麟工控机执行。
cd ~/project/robot_and_chassis_control/ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch eli_cs_robot_serial industrial_pc.launch.py start_hk_transfer:=true
```

若机器人任务已经启动了海康照片传输，不能重复启动同一个 9002 监听节点；
此时单独启动结果接收 `industrial_pc.launch.py` 即可，其 start_hk_transfer 默认 false。
通讯启动不会自动打开相机、触发拍照或驱动机器人。

然后麒麟工控机执行：

```bash
python3 industrial_pc_network_test.py connect \
  --host 树莓派实际IP --report tcp_connection.json
```

默认测试 **9002 和 9010**。也可以指定网卡、端口和每次连接超时：

```bash
python3 industrial_pc_network_test.py connect \
  --interface 实际网卡名 --local-ip 169.254.199.120 \
  --host 树莓派实际IP --ports 9002 9010 --timeout 3 --report tcp_connection.json
```

每次都会先执行第一阶段检查，绑定指定源 IP 后连接；如能读取路由，还会核对出口网卡。
只做 TCP 握手，不发送检测结果或照片，连上即关闭。
**应在开始拍照前运行检查**，避免临时测试连接占用照片通道的唯一接收客户端。
海康通道只允许源 IP 169.254.199.120，但 TCP 握手成功本身也不代表通过应用层 IP 检查。

- 返回码 `0`：本次选择的检查全部通过。
- 返回码 `1`：网口、连接或等待检查失败。
- 返回码 `2`：命令行参数错误。
- 可用 `echo $?` 查看上一条命令返回码；JSON 报告记录每项结果。
- `Connection refused`：检查对端服务监听或防火墙主动拒绝，不能简单认定网卡损坏。
- 超时：检查对端地址、供电、链路、路由与防火墙，不能仅凭超时定位故障。
- 本机绑定失败：检查 IP 是否真的配置在本机，以及麒麟的应用/网络安全策略。

## 三、可选：检查树莓派是否能反向连接工控机

实际深度上传由树莓派连接工控机，方向与上述两个端口不同。先用独立测试端口验证：

```bash
# 麒麟工控机执行，默认等待 30 秒、接到一次连接后退出。
python3 industrial_pc_network_test.py listen \
  --local-ip 169.254.199.120 --port 19030 --wait 30 --report reverse_connection.json
```

把同一份网络测试脚本放到树莓派，然后在等待期间执行：

```bash
# 树莓派执行；--local-ip 是树莓派本机地址。
python3 industrial_pc_network_test.py connect \
  --local-ip 树莓派实际IP --host 169.254.199.120 --ports 19030
```

`19030` 只是临时连通性测试端口，不是新增业务协议，不应当作深度接收程序。
通过后结束测试监听，按下文启动真正的 9030 深度接收程序。

## 四、业务通道总表

全部 IPv4 TCP，三条独立连接，不把海康照片、深度 ZIP 和结果 JSON 混在一个流里。

| 业务 | 数据方向 | TCP 服务端 | TCP 客户端 | 端口 |
|---|---|---|---|---|
| 海康触发拍照 | 树莓派 → 工控机 | 树莓派 | 工控机 169.254.199.120 | 9002 |
| 深度 RGB-D 采集 ZIP | 树莓派 → 工控机 | 工控机 169.254.199.120 | 树莓派 | 9030 |
| 深度检测点结果 | 工控机 → 树莓派 | 树莓派 | 工控机 169.254.199.120 | 9010 |

`0.0.0.0` 是服务端本地监听地址，不是客户端要连接的远端 IP。
9030 的服务端在工控机，不能拿它当树莓派端口测试。
USB 循迹视频不在本次传输协议内。9002 不用于海康检测结果回传；
9010 当前只定义深度相机三维点结果，未定义海康缺陷分类结果。

## 五、海康拍照协议 MIMG（9002）

帧结构：

```text
4 字节 ASCII "MIMG"
4 字节无符号大端整数：JSON 头字节数 H
H 字节 UTF-8 JSON 头
payload_length 字节照片内容
```

JPEG 头示例：

```json
{
  "version": 1,
  "type": "image",
  "sequence": 1,
  "stamp_ns": 1790000000123456789,
  "frame_id": "hk_camera_optical_frame",
  "topic": "/dataset_capture/captured_image",
  "width": 640,
  "height": 480,
  "source_encoding": "bgr8",
  "encoding": "jpeg",
  "payload_length": 123456
}
```

示例头的尺寸、时间、长度仅说明格式，应按实际收到的值解析。
JSON 头最大 65536 字节，JPEG 最大 32 MiB。`stamp_ns` 为拍照帧的原始时间戳。
照片由拍照话题图像重新编码为 JPEG，默认质量 95，不保证等于本地 JPEG 文件字节。
原始像素模式 version=2 另带 width/height/step/is_bigendian/encoding，
payload_length 必须为 step×height，最大 128 MiB；优先使用随附接收器避免手工处理行填充。

工控机执行并保持运行：

```bash
python3 industrial_pc_hk_receiver.py \
  --host 树莓派实际IP --port 9002 --save-dir ./hk_photos
```

树莓派的相机驱动与拍照节点已运行后，触发拍照：

```bash
ros2 service call /dataset_capture/capture std_srvs/srv/Trigger '{}'
```

工控机会保存每张照片及同名 JSON 元数据。只传触发拍照结果，不推送连续预览。
该兼容协议无 HMAC、无应用层保存 ACK；ready 只表示有客户端和队列空间。
队列满、无接收器或进程退出时不保证自动补传；原拍照文件仍在树莓派目录。

## 六、深度采集包协议 RGBD（9030）

工控机先运行：

```bash
python3 capture_transfer.py receive --bind 0.0.0.0 --port 9030 --output ./received_rgbd
```

接收器每次接收一个完整 ZIP 后退出，下一批需要再次启动。
仅执行 receive 不依赖 ROS、相机 SDK 或第三方 Python 模块。

树莓派发送已经完成的采集 ZIP：

```bash
ros2 run geminipro_camer_calibration capture_transfer.py send \
  --archive 实际采集包.zip --host 169.254.199.120 --port 9030
```

帧结构：

```text
4 字节 ASCII "RGBD"
4 字节无符号大端整数：JSON 头长度 H
H 字节 UTF-8 JSON：{"version":1,"size":ZIP字节数,"sha256":"ZIP的SHA256"}
size 字节 ZIP
```

头最大 65536 字节，ZIP 最大 128 MiB。接收器还检查 ZIP 路径/展开体积、
各组 capture_manifest、CAPTURE_COMPLETE 和原始图像/深度/参数哈希，不能发送任意测试 ZIP。
该版本的 ZIP 成员布局应遵守现有 `capture_transfer.py`，不要另写不兼容打包器。
采集包必须包含同组 RGB、原始深度、相机内参和采集清单，不用海康照片替代。

工控机校验并保存后回 ACK，格式为 **4 字节大端长度 + UTF-8 JSON**，不带 RGBD 魔数：

```json
{"ok": true, "groups": 1, "sha256": "实际ZIP的SHA256", "saved_to": "主机保存路径"}
```

失败为 `{"ok":false,"error":"原因"}`。此通道使用哈希完整性校验，无 HMAC。
收到深度 ZIP 不代表工控机已经完成检测，检测结果另走 9010。

## 七、深度检测点协议（9010）

每帧：**4 字节无符号大端 JSON 长度 + UTF-8 JSON**，长度不含四字节头，
最大 262144 字节。TCP 拆包、粘包必须按长度读取，不能假设一次 recv 得到完整消息。

封装字段：

```json
{
  "protocol": "paint_v54_camera_coordinates_tcp_v1",
  "payload": {},
  "payload_sha256": "64位小写SHA256",
  "hmac_sha256": "64位小写HMAC-SHA256"
}
```

规范字节与签名规则：

```python
body = json.dumps(payload, ensure_ascii=False, sort_keys=True,
                  separators=(',', ':'), allow_nan=False).encode('utf-8')
payload_sha256 = hashlib.sha256(body).hexdigest()
hmac_sha256 = hmac.new(shared_key_bytes, body, hashlib.sha256).hexdigest()
```

共享密钥按 UTF-8 文本去掉首尾空白后使用，至少 32 字节，不把十六进制文本再次解码成二进制。
两端用同一份密钥文件。ACK 使用相同长度头、封装和签名规则。
建议直接使用随附 Python 协议模块，避免不同语言 JSON 浮点格式不一致导致验签失败。

### payload 约定

- `message_id`：1～128 个 ASCII 字母、数字、下划线、点或减号；重试必须保持相同内容和 ID。
- `schema`：实际结果用 `paint_v54_pi_remote_result_v1`，演示用 `paint_v54_xyz_demo_v1`。
- `coordinate_frame`：固定 `CAMERA_PINHOLE_PRELIMINARY`；X 向右、Y 向下、Z 向前。
- `points_xyz_camera_mm`：最多 100 点，单位**毫米**，按主机给出的顺序保留。
  每项 `{"point_id":"P1","uv":[390,370],"xyz_camera_mm":[107.47,217.487,800.0]}`；uv 可省略，
  point_id 唯一，XYZ 必须有限，Z 大于零。上面的坐标来自演示，不是真实测量。
- 实际结果须有 `capture_manifest_sha256`：原始采集清单的 64 位小写 SHA256，用于关联拍照。
- 真实成功状态固定 `OK_3D_POINTS`，并声明 `depth_source:"MEASURED"`，点数组不能为空。
- 未获得有效三维点用 `HOLD_...` 状态，点数组必须为空，不能用假设深度填成实测成功。
- 演示须 `status:"DEMO_ONLY_NOT_VALID_MEASUREMENT"`、`demo_only:true`。
- `observed_at_ros:{"sec":整数,"nanosec":整数}`：原始拍照 ROS 时间，可选；
  旧数据缺少该字段时保留零时间戳，不能用接收时间或主机处理时间代替。
- 三类结果均须 `robot_motion_authorized:false`、`robot_base_coordinate_calibrated:false`、
  `robot_command:null`。本协议传观测点，不传运动指令，也未定义触碰姿态。

### 生成/共享密钥与发送演示

密钥只生成一次，之后安全复制同一文件到另一端，不能在两端各生成不同密钥。
若树莓派已有密钥，直接复用，不重新生成。

```bash
# 仅首次、尚无密钥时执行；文件已存在会拒绝覆盖。
python3 industrial_pc_client.py keygen --key-file industrial_pc.key
```

树莓派启动结果接收时指定同一密钥路径，例如：

```bash
ros2 launch eli_cs_robot_serial industrial_pc.launch.py \
  start_hk_transfer:=true key_file:=/实际路径/industrial_pc.key
```

麒麟工控机发送独立 DEMO：

```bash
python3 industrial_pc_client.py demo --host 树莓派实际IP --port 9010 \
  --key-file industrial_pc.key --file DEMO_XYZ_800mm.json
```

发送实际 HOLD：

```bash
python3 industrial_pc_client.py send --host 树莓派实际IP --port 9010 \
  --key-file industrial_pc.key --file HOLD_result.json
```

`send` 可接受未封装 payload 并签名，或已签名的完整封装；完整封装必须通过原签名校验，
不会自动更换签名。之前提供的旧 `result_tcp_message.json` 的密钥未知，
不能指望用新密钥验签；使用未封装 payload 由双方约定密钥生成新消息。

成功 ACK payload 例如：

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

重复报文返回 `disposition:"duplicate"`、`published:false`，不再次发布点；
重启后根据已保存文件继续去重。同 ID 不同内容、签名或坐标无效返回 `ok:false`、`reason`。
HOLD 可以成功保存并返回 `ok:true`，不表示有可执行检测点。
客户端须校验 ACK 签名、schema、message_id 和 ok；超时重试同一 ID/同一 payload。
接收端默认空闲超时 5 秒，示例客户端读取 ACK 超时 10 秒、网络异常默认重试 2 次。

## 八、联调完成的判断

分清三个层级：网口可绑定 → TCP 可连接 → 业务包被正确接收/校验。

- 网络工具 PASS 只验证前两层，不代表对端完成业务处理。
- 海康层以收到目标拍照帧并在主机保存照片和元数据为准。
- 深度层以完整 ZIP 校验成功、保存并返回成功 ACK 为准。
- 结果层以树莓派返回通过签名校验的对应消息 ACK 为准。
- 结果接收成功不代表坐标已转换为机械臂运动目标。光学外参、拍照时机械臂位姿、
  采集时间关联和接触朝向仍需要机器人侧后续处理，DEMO 点不会自动触碰。

项目内通讯实现位置：`ros_ws/src/eli_cs_robot_serial/device/industrial_pc/`；
深度 ZIP 实现在 `ros_ws/src/geminipro_camer_calibration/scripts/capture_transfer.py`。
交付 ZIP 内包含对应脚本副本，接口更新后应重新导出交付包，避免两端版本不一致。

## 九、读取最新漆膜厚度（9010，同一签名通道）

主机使用更新后的客户端执行：

```bash
python3 industrial_pc_client.py thickness \
  --host 树莓派实际IP --port 9010 --key-file industrial_pc.key
```

查询 payload 为 `{"schema":"paint_thickness_latest_request_v1","message_id":"query-001"}`，
使用第七节相同的长度头、SHA256/HMAC。只查询缓存，不触发新测量。
响应为原有 `paint_v54_pi_result_ack_v1`，回显 message_id，新增字段示例如下：

```json
{"latest_thickness":{"available":true,"unit_confirmed":true,"age_seconds":0.2,"measurement":{"schema":"paint_thickness_measurement_v1","sequence":12,"value":123.5,"unit":"um","received_at_ros":{"sec":1790000000,"nanosec":0}}}}
```

数字为格式示例。只返回最新单值，不发送滚动历史数组。
未收到有效回包时为 `{"available":false,"measurement":null}`，不会用零冒充厚度。
`unit` 来自仪器配置（um/mm/unknown），不隐式转换，unknown 时 `unit_confirmed:false`。
接收时间为树莓派收到串口回包的 ROS 时间，不是仪器物理测量时间。
age_seconds 是当前 ROS 时间减接收时间；缓存不自动过期，主机需按时间判断适用性。
普通检测点结果 ACK 也附带此刻最新厚度。此快照不保证属于刚提交的相机检测点；
逐点检测结果以树莓派 `/detect_controller/thickness_result` 的关联为准。
