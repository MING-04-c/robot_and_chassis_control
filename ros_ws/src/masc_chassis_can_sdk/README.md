# MASC Chassis CAN SDK

MASC 是多机协同（苏州）有限公司的英文名。

MASC Chassis CAN SDK 是 MASC Chassis CAN V10 的 C++ SDK。当前版本提供协议编解码、
Linux SocketCAN 通讯、解析后数据缓存、反馈回调和健康信息。

SDK 返回解析后的结构体，上层应用无需处理 CAN 二进制 payload。正常连接条件是 CAN
transport 已打开，并且已收到协议版本匹配的 `system_version_feedback (0x205)`。

## 当前支持范围

| 项目 | 状态 |
|---|---|
| 协议版本 | MASC Chassis CAN V10 |
| CAN 类型 | CAN 2.0A standard frame，11-bit ID，DLC 按 CAN ID 定义 |
| 字节序 | Little-Endian |
| 单位边界 | SDK 对外使用 `rad/s` 和 `rad`；CAN payload 中 `angular_z` 使用 `deg/s`、`steering_angle` 使用 `deg`，协议层自动转换 |
| Linux | 已实现 SocketCAN 后端 |
| Windows | 预留接口，当前返回 unsupported，等待具体 USB-CAN 厂商 SDK |
| ACK | SDK v0.1 不等待 ACK，由上层应用按业务处理 |
| 自动重连 | SDK v0.1 不主动重连，由上层应用处理 |

## 构建

```bash
mkdir -p ~/masc_ros_ws/src
cd ~/masc_ros_ws/src/masc_chassis_can_driver/masc_chassis_can_sdk
cmake -S . -B /tmp/masc_chassis_can_sdk_build
cmake --build /tmp/masc_chassis_can_sdk_build
ctest --test-dir /tmp/masc_chassis_can_sdk_build --output-on-failure
```

默认会构建：

- `libmasc_chassis_can_sdk.a`
- `masc_chassis_can_sdk_demo`
- 协议层和运行时单元测试

## Linux CAN 配置

SDK 不主动执行 `ip link`。应用启动前需要先配置 CAN 口：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
```

`restart-ms` 是 Linux CAN 控制器的 bus-off 自动恢复配置；SDK 仍然只记录 transport
错误，不做主动重连。

## 运行 Demo

```bash
/tmp/masc_chassis_can_sdk_build/masc_chassis_can_sdk_demo can0 10
```

参数说明：

- 第 1 个参数：CAN 接口名，默认 `can0`。
- 第 2 个参数：运行秒数，默认 `10`。

Demo 行为：

- 打开 CAN 接口。
- 等待 `system_version_feedback (0x205)`，并校验 `protocol_version == 0x0A`。
- 周期发送零目标 `motion_control_cmd (0x100)`。
- 每秒打印一次解析后的运动反馈、系统反馈、BMS 信息、IMU/RC 信息和健康状态。

现场首次运行建议让底盘离地，或确保运动区域安全。Demo 默认只发送零速度/零转角；
在确认运动模式、急停状态、故障状态和场地安全前，不要改成非零运动目标。

## 最小使用示例

```cpp
#include <chrono>
#include <iostream>
#include <thread>

#include "masc_chassis_can_sdk/chassis_sdk.hpp"

int main() {
    masc_chassis_can_sdk::CanConfig config;
    config.m_interface_name = "can0";
    config.m_bitrate = 500000;
    config.m_connect_timeout_ms = 1000;

    masc_chassis_can_sdk::ChassisSdk sdk;
    if (!sdk.Configure(config)) {
        std::cerr << "Configure failed\n";
        return 1;
    }

    sdk.SetMotionFeedbackCallback([](const masc_chassis_can_sdk::MotionFeedback &feedback) {
        std::cout << "vx=" << feedback.m_current_linear_x_mps
                  << " wz=" << feedback.m_current_angular_z_radps << "\n";
    });

    if (!sdk.Connect()) {
        const auto health = sdk.GetHealthSnapshot();
        std::cerr << "Connect failed, transport_errors="
                  << health.m_transport_error_count << "\n";
        return 1;
    }

    masc_chassis_can_sdk::MotionCommand command;
    command.m_target_linear_x_mps = 0.0;
    command.m_target_linear_y_mps = 0.0;
    command.m_target_angular_z_radps = 0.0;
    command.m_target_steering_angle_rad = 0.0;

    for (int i = 0; i < 100; ++i) {
        sdk.SetMotionCommand(command);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    sdk.Disconnect();
    return 0;
}
```

## ChassisSdk API 索引

### 连接与生命周期

| API | 用途 |
|---|---|
| `ChassisSdk()` | 使用当前平台默认 transport 创建 SDK。Linux 下默认使用 SocketCAN |
| `ChassisSdk(std::unique_ptr<ICanTransport>)` | 使用自定义 transport 创建 SDK，常用于测试或厂商 CAN 后端扩展 |
| `~ChassisSdk()` | 停止 SDK 线程并关闭 transport |
| `Configure(config)` | 设置 CAN 接口名、期望协议版本、连接超时和运动指令周期 |
| `Connect()` | 打开 transport，并等待 `system_version_feedback (0x205)` 匹配 |
| `Disconnect()` | 停止收发线程并关闭 transport，可重复调用 |
| `IsConnected()` | 查询 `Connect()` 握手成功后、`Disconnect()` 前的连接状态 |

### 控制命令

| API | 用途 |
|---|---|
| `SetMotionCommand(command)` | 刷新周期发送的 `motion_control_cmd (0x100)` 目标 |
| `SendMotionCommandOnce(command)` | 立即发送一次 `motion_control_cmd (0x100)`，主要给测试/工具使用 |
| `SendTowMode(enable)` | 发送 `tow_mode_control_cmd (0x101)`，进入或退出拖车模式 |
| `SendMotionMode(mode)` | 支持时发送 `motion_mode_switch_cmd (0x110)`，SDK 不等待模式反馈确认 |
| `SendFaultClear(fault_id)` | 发送 `fault_clear_cmd (0x120)`，SDK 不解释清除是否成功 |
| `SendCalibrateServo(servo_id)` | 发送 `servo_calibration_cmd (0x170)`，触发舵机自动标定 |
| `SendRawFrame(frame)` | 发送一帧原始 CAN 帧，主要用于诊断、测试工具和协议联调 |

`motion_mode_switch_cmd (0x110)` 和 `motion_mode_feedback (0x220)` 只适用于
`system_version_feedback (0x205)` 中当前定义的全轮转向/舵轮底盘类型
`0x61`、`0x62`、`0x63`。其他底盘类型调用 `SendMotionMode()` 会返回 `false`。

### 反馈读取

| API | 用途 |
|---|---|
| `GetVersionInfo()` | 读取缓存的 `system_version_feedback (0x205)` |
| `IsMotionModeSwitchSupported()` | 判断当前连接底盘是否支持 `motion_mode_switch_cmd (0x110)` / `motion_mode_feedback (0x220)` |
| `GetMotionFeedback()` | 读取缓存的 `motion_control_feedback (0x200)` |
| `GetSystemFeedback()` | 读取缓存的 `system_info_feedback (0x204)` |
| `GetBmsBasicFeedback()` | 读取缓存的 `bms_basic_feedback (0x206)` |
| `GetBmsFaultFeedback()` | 读取缓存的 `bms_fault_feedback (0x207)` |
| `GetMotionModeFeedback()` | 读取缓存的 `motion_mode_feedback (0x220)` |
| `GetPowerWheelFeedback(index)` | 按协议 index 读取 `power_wheel_feedback (0x251-0x258)` |
| `GetPowerWheelDriverFeedback(index)` | 按协议 index 读取 `power_wheel_driver_feedback (0x261-0x268)` |
| `GetServoFeedback(index)` | 按协议 index 读取 `servo_feedback (0x271-0x278)` |
| `GetServoDriverFeedback(index)` | 按协议 index 读取 `servo_driver_feedback (0x281-0x288)` |
| `GetImuFeedback()` | 读取缓存的 `imu_feedback (0x300)`，姿态角已转换为 rad |
| `GetRcFeedback()` | 读取缓存的 `rc_feedback (0x301)` 在线状态和原始 CH0-CH6 通道值 |

`index` 是从 CAN ID 范围计算出来的协议序号，例如 `power_wheel_feedback (0x251)` 对应
`index = 0`。SDK v0.1 不假设 index 到物理轮位/舵机位置的映射。

### 健康与统计

| API | 用途 |
|---|---|
| `GetHealthSnapshot()` | 读取连接、协议版本匹配、收发错误和运动命令超时状态 |
| `GetFrameStats(can_id)` | 读取指定 CAN ID 的解析计数、丢帧计数、最后接收时间和估算频率 |

### 回调注册

| API | 触发时机 |
|---|---|
| `SetMotionFeedbackCallback(callback)` | 收到并解析 `motion_control_feedback (0x200)` 后触发 |
| `SetSystemFeedbackCallback(callback)` | 收到并解析 `system_info_feedback (0x204)` 后触发 |
| `SetBmsBasicFeedbackCallback(callback)` | 收到并解析 `bms_basic_feedback (0x206)` 后触发 |
| `SetBmsFaultFeedbackCallback(callback)` | 收到并解析 `bms_fault_feedback (0x207)` 后触发 |
| `SetMotionModeFeedbackCallback(callback)` | 收到并解析 `motion_mode_feedback (0x220)` 后触发 |
| `SetPowerWheelFeedbackCallback(callback)` | 收到并解析 `power_wheel_feedback (0x251-0x258)` 后触发 |
| `SetPowerWheelDriverFeedbackCallback(callback)` | 收到并解析 `power_wheel_driver_feedback (0x261-0x268)` 后触发 |
| `SetServoFeedbackCallback(callback)` | 收到并解析 `servo_feedback (0x271-0x278)` 后触发 |
| `SetServoDriverFeedbackCallback(callback)` | 收到并解析 `servo_driver_feedback (0x281-0x288)` 后触发 |
| `SetImuFeedbackCallback(callback)` | 收到并解析 `imu_feedback (0x300)` 后触发 |
| `SetRcFeedbackCallback(callback)` | 收到并解析 `rc_feedback (0x301)` 后触发 |
| `SetRawFrameCallback(callback)` | 收到当前协议未解析的 CAN 帧后触发 |
| `SetHealthCallback(callback)` | SDK 发布健康状态通知时触发 |

## 原始 CAN 帧调试接口

SDK 正常业务接口优先返回解析后的结构体。原始 CAN 帧接口只建议用于诊断、测试工具和协议
扩展联调。

`SendRawFrame(frame)` 会直接发送一帧 CAN 帧。发送前 SDK 会检查：

- CAN ID 必须是 11-bit standard ID。
- DLC 必须符合 classic CAN 范围 `0-8`。

`SetRawFrameCallback(callback)` 会在接收到以下帧时触发：

- 当前 V10 协议未定义的 CAN ID。
- DLC 不符合该 CAN ID 协议定义的帧。
- CAN ID 已定义，但当前 parser 校验失败的帧。

这个 callback 不会替代正常 feedback callback。已成功解析的协议反馈仍然只走对应的 typed
callback 和 getter。

## 命令超时保护

SDK 内部会周期发送最近一次 `SetMotionCommand()` 设置的 `motion_control_cmd (0x100)`。
默认周期为 `20 ms`。

如果上层超过 `m_motion_command_timeout_ms` 没有刷新运动目标，SDK 会自动发送零速度/零转角
的 `motion_control_cmd (0x100)`，并在 `HealthSnapshot::m_motion_command_timeout_active`
中标记该状态。

这只是 SDK 侧保护。固件仍应把 `motion_control_cmd (0x100)` 作为看门狗输入，长时间未收到
有效运动指令时将目标速度归零。

## 回调和轮询

SDK 同时支持回调和轮询：

- 回调：`SetMotionFeedbackCallback()`、`SetSystemFeedbackCallback()` 等。
- 轮询：`GetMotionFeedback()`、`GetSystemFeedback()`、`GetPowerWheelFeedback(index)` 等。

回调在 SDK 接收分发路径中触发，处理函数要尽量轻量，不建议在回调里阻塞或做耗时业务。

## 上层软件职责

SDK v0.1 不做以下业务：

- 不主动重连 CAN。
- 不等待命令 ACK。
- 不自动切换运动模式。
- 不根据故障码做急停决策。
- 不定义未明确的 fault bit 文本。
- 不假设电机/舵机 CAN ID 到物理位置的映射。

这些逻辑建议放在 ROS driver、诊断节点或上层应用中。
