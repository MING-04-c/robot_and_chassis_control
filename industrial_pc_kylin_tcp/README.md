# 麒麟通讯工具

Python 3.8及以上，无需ROS、相机SDK或额外pip依赖。树莓派169.254.199.100，麒麟169.254.199.120。

**当前默认无密钥，不需要industrial_pc.key。** 两边一起更新本版本；保留SHA256、长度检查、断线重连和重复指令去重。

完整启动步骤见 [QUICKSTART.md](QUICKSTART.md)。

## 常用命令

```bash
python3 listen_robot.py                 # 常驻：收图、显示任务与连接状态
python3 start_capture.py                # 另一终端：只采HK一张和RGB/深度一组，不执行运动
python3 start_task.py                   # 启动已加载的运动工作流
python3 send_points.py --file result.json
python3 start_task.py --action resume   # 途中站继续
python3 start_task.py --action stop
python3 start_task.py --action status
```

回执只表示受理；采集/运动完成看后续状态。同一请求重发复用message_id，新一轮用新编号。任务与独立采集互斥。

## 照片现在这样保存

```text
received/
  2026.10.30.20.29/
    hk/
      20.29.12_345_hk.jpg
    RGB/
      20.29.12_345_rgb.png
      20.29.12_345_depth.png
```

第一层包含北京时间年月日时分；第二层只有 `hk/` 和 `RGB/`，没有额外的日期或采集时间目录。HK放海康照片；RGB同时放深度相机的彩色图和原始16位深度图。同一分钟多次采集共用目录，文件名按秒、毫秒区分；同一轮三图使用相同时间前缀。时间完全相同的不同采集会追加编号，重发同一采集不额外保存。旧双HK流第二张用 `_hk_02.jpg` 区分。

目录和前缀以实际HK采集时间为准，不表示两相机同时曝光。Pi和主机通过采集编号及传输的capture_name关联。两端均需更新本版；目录改名不改变深度数值。

ZIP校验和解压完成后即清理，不重复保留NPY与清单。集中索引 `.capture_index.json`、锁和监听状态保留在根目录，保存图像配对、参数、深度比例和去重信息。旧日期/时间目录仍可读取，新采集才使用新目录；要整理旧图可使用后面的复制工具。

主机读取每组文件和参数：

```python
import json
from pathlib import Path
from capture_storage import file_path
root = Path('received')
index = json.loads((root/'.capture_index.json').read_text())
for task_id, record in index['sessions'].items():
    if 'rgbd' not in record:
        continue
    rgb_file = file_path(root, record, 'rgb.png')
    depth_file = file_path(root, record, 'depth.png')
    params = record['rgbd']['camera_parameters']
    scale = record['rgbd']['depth_scale_mm_per_raw']
    manifest_sha256 = record['rgbd']['capture_manifest_sha256']
    # 原始深度像素乘scale得到毫米；内参和比例按本轮实际数据读取。
```

## 配置和通道

config.json中的robot_host为树莓派IP；output_dir为照片目录，相对路径以config所在目录为基准。默认use_shared_key=false。

- 9010：麒麟连接Pi，命令/状态/检测点，带长度头和SHA256。
- 9002：麒麟连接Pi，HK照片。任务采完才临时开放；waiting_sender是正常等待。
- 9030：Pi连接麒麟，RGB-D包；校验并保存图片后才回ACK，监听持续运行。

HK原通道仍无落盘ACK，以麒麟“保存完成”和文件为准。深度收到校验ACK才认为发送成功。SHA256是完整性校验，不冒充身份认证。

## 整理旧接收数据

旧数据可复制成新格式，源目录不会修改：

```bash
python3 capture_storage.py --source received --output received_clean
```

检查新目录后再自行切换目录；不要在监听正在写入时整理。同目录不能作为输入和输出。

`DEMO_XYZ_800mm.json`等是演示，不是实测点；通讯桥仅发布检测点观测；若Pi运行camera_detection_real默认TCP测试，匹配本次照片的实测点会自动触发探针触碰。线缆格式见 [TCP通讯协议_麒麟工控机.md](TCP通讯协议_麒麟工控机.md)。
# 手选像素点回传（后续通讯使用）

Pi的camera_detection_real默认TCP模式会采新图并等主机点，匹配后自动触碰。手选像素测试需针对本次新图生成结果，再发送：

```bash
python3 send_points.py --pixel-test --file test_pixel_xyz.json --rgb-file received/2026.10.30.20.29/RGB/20.29.12_345_rgb.png
```

Pi改用target_source:=local时不启动通讯，也不使用主机点。

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
