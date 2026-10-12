# MASC 底盘 ROS 接口

MASC 是多机协同（苏州）有限公司的英文名。

`masc_chassis_msgs` 定义 MASC 底盘 CAN 驱动使用的 ROS1/ROS2 消息和服务接口。

这些接口面向上层 ROS 驱动输出。运动控制命令、运动反馈和原始 CAN 帧保留在 CAN SDK
或驱动内部，不作为 ROS 消息直接暴露。

## 消息分组

| 分组 | 接口 |
|---|---|
| 一次性控制服务 | `SetMotionMode`, `SetTowMode`, `ClearFault`, `CalibrateServo` |
| 底盘核心反馈 | `ChassisStatus` |
| BMS 反馈 | `BmsStatus` |
| 电机反馈 | `DriveMotor`, `DriveMotorArray`, `DriveMotorStatus`, `DriveMotorStatusArray`, `ServoMotor`, `ServoMotorArray`, `ServoMotorStatus`, `ServoMotorStatusArray` |
| 遥控反馈 | `RcStatus` |
| 通讯/调试 | `CommStatus`, `CanFrameStats`, `CanFrameStatsArray` |

## 接口约定

- ROS 发布的数据使用驱动对外单位：`m`, `m/s`, `rad`, `rad/s`, `V`, `A`, `degC`。
- `ServoMotor.angle` 使用弧度 `rad` 发布。CAN SDK 可以按固件/docx 中
  `servo_feedback (0x271-0x278)` 的 degree 单位解析，上层 ROS 驱动发布前转换为 rad。
- IMU 反馈在 ROS driver 中直接使用标准 `sensor_msgs/Imu` 发布，本接口包不再定义自定义
  IMU 消息。
- `RcStatus.online` 来自 `rc_feedback (0x301)` Byte0；`RcStatus.channels` 保留 Byte1-Byte7 的
  `CH0-CH6` 原始通道值，范围 `-100~100`。接口包不解释每个通道的业务含义。
- `BmsStatus` 是统一的电源系统接口。CAN 协议填充当前可获得的电池字段；缺失字段保持默认值，
  或由串口协议/上层电源管理模块补齐。
- `ChassisStatus` 表示底盘运行状态，包括当前运动模式、模式切换状态、用于显示的底盘状态、
  常用状态标志、障碍状态标志、原始系统状态字和底盘故障码。
- `ChassisStatus.current_motion_mode` 使用 ROS 上层语义：`0` 为默认底盘模式，`1` 为驻车模式。
  该枚举与 CAN 协议中的当前默认模式/驻车模式取值保持一致。
- `CommStatus` 表示 CAN SDK 通讯状态，包括通讯状态、失败原因、传输计数、运动命令超时保护状态、
  底盘 UUID、系统软硬件版本、底盘类型版本和通讯协议版本。
- `CommStatus.system_software_hardware_version` 保留 `system_version_feedback` 的 Byte5 原始值；
  `chassis_type_version` 来自 Byte6，按当前 CAN V10 文档定义的底盘类型原始值发布。
- `index` 是根据 CAN ID 范围计算出的协议索引，不代表具体物理轮位或舵机安装位置。
- 故障字段在固件故障 bit 定义最终确认前，均按协议原始值发布。
- 一次性控制命令使用 ROS service。service response 只表示驱动是否接收并发送请求；
  协议执行结果或 ACK 确认应通过反馈消息观察。
- `SetMotionMode` 仅在 `CommStatus.chassis_type_version` 为 `0x61`、`0x62` 或 `0x63` 时可用。
- `SetTowMode` 发送一次拖车模式控制命令。service response 只表示命令是否已发送。
- `CalibrateServo` 触发固件自动标定流程。请求里不传入标定值，也不会把当前角度设置为零点。

## 推荐发布方式

建议上层驱动按数据类型单独发布 topic，避免使用聚合大消息：

| Topic 建议 | 消息类型 | 说明 |
|---|---|---|
| `/chassis_status` | `ChassisStatus` | 底盘运行状态 |
| `/bms_status` | `BmsStatus` | 电池/BMS 状态 |
| `/drive_motors` | `DriveMotorArray` | 驱动电机实时反馈 |
| `/drive_motor_status` | `DriveMotorStatusArray` | 驱动电机温度和故障 |
| `/servo_motors` | `ServoMotorArray` | 舵机实时反馈 |
| `/servo_motor_status` | `ServoMotorStatusArray` | 舵机电机温度和故障 |
| `/imu` | `sensor_msgs/Imu` | IMU 姿态 |
| `/rc_status` | `RcStatus` | 遥控器通道原始值 |
| `/comm_status` | `CommStatus` | CAN 通讯状态 |
| `/can_frame_stats` | `CanFrameStatsArray` | 各 CAN ID 接收和解析统计 |

`CanFrameStatsArray` 建议低频发布，例如 `1-2 Hz`，主要用于现场调试和诊断。
