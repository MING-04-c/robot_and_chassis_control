# MASC Chassis CAN SDK 使用说明

本文面向需要直接集成 MASC 底盘 CAN 通讯的 C++ 用户。SDK 可以在 ROS1/ROS2
驱动中使用，也可以在非 ROS 程序中独立使用。

MASC 是多机协同（苏州）有限公司的英文名。

## 功能范围

`masc_chassis_can_sdk` 是 MASC Chassis CAN V10 协议的 C++ SDK，主要提供：

- CAN V10 协议帧打包和解析。
- Linux SocketCAN 通讯后端。
- 解析后的 C++ 结构体缓存和读取接口。
- 反馈回调接口。
- 运动控制指令周期发送和本地超时归零保护。
- 通讯健康状态和 CAN ID 统计信息。
- 原始 CAN 帧诊断接口。

SDK 不依赖 ROS。ROS topic、ROS service、诊断发布和业务安全策略由上层应用实现。

## 当前支持

| 项目 | 说明 |
|---|---|
| 协议版本 | MASC Chassis CAN V10 |
| CAN 类型 | CAN 2.0A standard frame，11-bit ID |
| CAN FD | 不使用 |
| 字节序 | Little-Endian |
| Linux | 支持 SocketCAN |
| Windows | 预留 transport 接口，当前未内置具体 USB-CAN 厂商后端 |
| 自动重连 | SDK 不主动重连，上层应用负责 |
| ACK 等待 | SDK 不等待命令 ACK，上层通过反馈判断结果 |
| ROS 依赖 | 无 |

## 目录结构

```text
masc_chassis_can_sdk/
  include/masc_chassis_can_sdk/
    chassis_sdk.hpp       # SDK 主入口
    types.hpp             # 配置、命令、反馈、健康状态结构体
    can_frame.hpp         # SDK 内部标准 CAN 帧结构
    can_transport.hpp     # CAN transport 抽象接口
    chassis_protocol.hpp  # CAN V10 协议编解码接口
  src/
    chassis_sdk.cpp
    chassis_protocol.cpp
    socketcan_transport.cpp
  examples/
    chassis_can_sdk_demo.cpp
```

正常用户优先包含：

```cpp
#include "masc_chassis_can_sdk/chassis_sdk.hpp"
```

## 构建 SDK

非 ROS 程序可以直接用 CMake 构建 SDK：

```bash
mkdir -p ~/masc_ros_ws/src
cd ~/masc_ros_ws/src/masc_chassis_can_driver/masc_chassis_can_sdk
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

默认会生成：

- `libmasc_chassis_can_sdk.a`
- `masc_chassis_can_sdk_demo`
- SDK 单元测试

## Linux CAN 配置

SDK 不执行 `ip link`，启动应用前需要先配置 CAN 接口：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
```

`restart-ms` 由 Linux CAN 控制器处理 bus-off 自动恢复。SDK 只记录 transport
错误，不会主动重连。

## 运行 Demo

```bash
./build/masc_chassis_can_sdk_demo can0 10
```

参数说明：

- `can0`：CAN 接口名。
- `10`：运行秒数。

Demo 会打开 CAN 接口，等待 `system_version_feedback (0x205)` 协议版本匹配，
周期发送零速度 `motion_control_cmd (0x100)`，并打印解析后的反馈和健康信息。

首次现场运行建议底盘离地，或确保运动区域安全。Demo 默认只发送零速度和零转角。

## 非 ROS 最小示例

```cpp
#include <chrono>
#include <iostream>
#include <thread>

#include "masc_chassis_can_sdk/chassis_sdk.hpp"

int main() {
    masc_chassis_can_sdk::CanConfig config;
    config.m_interface_name = "can0";
    config.m_bitrate = 500000;
    config.m_expected_protocol_version = masc_chassis_can_sdk::kProtocolVersionV10;
    config.m_connect_timeout_ms = 1000;
    config.m_motion_command_period_ms = 20;
    config.m_motion_command_timeout_ms = 100;

    masc_chassis_can_sdk::ChassisSdk sdk;
    if (!sdk.Configure(config)) {
        std::cerr << "SDK configure failed\n";
        return 1;
    }

    sdk.SetMotionFeedbackCallback([](const masc_chassis_can_sdk::MotionFeedback &feedback) {
        std::cout << "vx=" << feedback.m_current_linear_x_mps
                  << " wz=" << feedback.m_current_angular_z_radps << "\n";
    });

    if (!sdk.Connect()) {
        const auto health = sdk.GetHealthSnapshot();
        std::cerr << "SDK connect failed, transport_errors="
                  << health.m_transport_error_count << "\n";
        return 1;
    }

    masc_chassis_can_sdk::MotionCommand command;
    command.m_target_linear_x_mps = 0.0;
    command.m_target_linear_y_mps = 0.0;
    command.m_target_angular_z_radps = 0.0;
    command.m_target_steering_angle_rad = 0.0;

    for (int i = 0; i < 500; ++i) {
        sdk.SetMotionCommand(command);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    sdk.Disconnect();
    return 0;
}
```

`Connect()` 不是只打开 socket 就返回成功。SDK 需要收到并解析
`system_version_feedback (0x205)`，且 `protocol_version` 与配置的
`m_expected_protocol_version` 一致后才认为连接成功。

## 非 ROS CMake 集成示例

如果 SDK 作为源码子目录一起编译：

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_chassis_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(path/to/masc_chassis_can_sdk)

add_executable(my_chassis_app main.cpp)
target_link_libraries(my_chassis_app PRIVATE masc_chassis_can_sdk)
```

如果 SDK 已经安装为 CMake 包：

```cmake
find_package(masc_chassis_can_sdk REQUIRED)

add_executable(my_chassis_app main.cpp)
target_link_libraries(
    my_chassis_app
    PRIVATE masc_chassis_can_sdk::masc_chassis_can_sdk
)
```

## 生命周期

典型使用顺序：

1. 创建 `ChassisSdk`。
2. 填写 `CanConfig`。
3. 调用 `Configure(config)`。
4. 注册需要的回调。
5. 调用 `Connect()`。
6. 通过 `SetMotionCommand()` 刷新运动目标。
7. 通过 getter 或 callback 获取反馈。
8. 程序退出或需要重连时调用 `Disconnect()`。

`Configure()` 不能在 transport 已打开时修改配置。需要切换 CAN 接口或重新配置时，
先调用 `Disconnect()`。

## 控制接口

| API | 说明 |
|---|---|
| `SetMotionCommand(command)` | 刷新周期发送的 `motion_control_cmd (0x100)` 目标 |
| `SendMotionCommandOnce(command)` | 立即发送一次 `motion_control_cmd (0x100)`，主要用于测试工具 |
| `SendTowMode(enable)` | 发送 `tow_mode_control_cmd (0x101)` |
| `SendMotionMode(mode)` | 发送 `motion_mode_switch_cmd (0x110)`，仅支持指定底盘类型 |
| `SendFaultClear(fault_id)` | 发送 `fault_clear_cmd (0x120)` |
| `SendCalibrateServo(servo_id)` | 发送 `servo_calibration_cmd (0x170)`，触发舵机自动校准 |
| `SendRawFrame(frame)` | 发送原始 CAN 帧，主要用于诊断和协议联调 |

`SendMotionMode()` 只在 `system_version_feedback (0x205)` 报告的底盘类型为
`0x61`、`0x62`、`0x63` 时返回成功。SDK 只发送命令，不等待
`motion_mode_feedback (0x220)` 确认。

## 运动命令超时保护

SDK 内部会按 `m_motion_command_period_ms` 周期发送最近一次
`SetMotionCommand()` 设置的 `motion_control_cmd (0x100)`。

如果上层超过 `m_motion_command_timeout_ms` 没有刷新运动命令，SDK 会自动发送零速度
和零转角，并在 `HealthSnapshot::m_motion_command_timeout_active` 中标记该状态。

这只是 SDK 侧保护。固件仍应把 `motion_control_cmd (0x100)` 作为最终看门狗输入。

## 反馈读取

SDK 保存最近一次解析成功的反馈。getter 返回 `std::optional<T>`，未收到对应反馈时为空。

| API | 对应反馈 |
|---|---|
| `GetVersionInfo()` | `system_version_feedback (0x205)` |
| `GetMotionFeedback()` | `motion_control_feedback (0x200)` |
| `GetSystemFeedback()` | `system_info_feedback (0x204)` |
| `GetBmsBasicFeedback()` | `bms_basic_feedback (0x206)` |
| `GetBmsFaultFeedback()` | `bms_fault_feedback (0x207)` |
| `GetMotionModeFeedback()` | `motion_mode_feedback (0x220)` |
| `GetPowerWheelFeedback(index)` | `power_wheel_feedback (0x251-0x258)` |
| `GetPowerWheelDriverFeedback(index)` | `power_wheel_driver_feedback (0x261-0x268)` |
| `GetServoFeedback(index)` | `servo_feedback (0x271-0x278)` |
| `GetServoDriverFeedback(index)` | `servo_driver_feedback (0x281-0x288)` |
| `GetImuFeedback()` | `imu_feedback (0x300)` |
| `GetRcFeedback()` | `rc_feedback (0x301)` |

`index` 是协议序号。例如 `0x251` 对应 `index = 0`，`0x254` 对应 `index = 3`。
SDK 不假设协议序号到物理轮位或舵机位置的映射。

## 回调使用

每类反馈都可以注册回调：

```cpp
sdk.SetSystemFeedbackCallback([](const masc_chassis_can_sdk::SystemFeedback &feedback) {
    if (feedback.m_system_state == masc_chassis_can_sdk::SystemState::Fault) {
        // 上层应用在这里记录或发布故障状态。
    }
});
```

回调在 SDK 接收分发路径中触发，应尽量轻量。不要在回调中执行长时间阻塞操作。
复杂业务建议把数据写入应用自己的队列，再由业务线程处理。

## 健康状态和 CAN 统计

`GetHealthSnapshot()` 返回 SDK 通讯状态：

- socket 是否打开。
- 是否已完成协议版本握手。
- 协议版本是否匹配。
- 运动命令是否处于超时归零保护。
- 连接尝试次数。
- transport、发送、接收错误计数。
- 缓存的版本信息。

`GetFrameStats(can_id)` 返回指定 CAN ID 的统计信息：

- 成功解析次数。
- 解析失败次数。
- 丢弃次数。
- 估算频率。
- 最后一次接收时间。

这些信息只用于诊断和上层决策。SDK 不会因为统计异常自动急停或自动重连。

## 原始 CAN 帧接口

正常业务优先使用 typed API。原始 CAN 帧接口用于测试工具、诊断和协议扩展联调。

```cpp
sdk.SetRawFrameCallback([](const masc_chassis_can_sdk::CanFrame &frame) {
    // 收到未支持 CAN ID、DLC 不匹配或解析失败的帧时触发。
});
```

`SendRawFrame(frame)` 会检查 11-bit standard CAN ID 和 classic CAN DLC 范围。

## 非 ROS 应用的重连建议

SDK 不主动重连。非 ROS 应用可以在业务层做简单状态机：

1. 周期检查 `GetHealthSnapshot()`。
2. 当 transport 错误持续增加或关键反馈超时时，调用 `Disconnect()`。
3. 等待一段退避时间。
4. 重新调用 `Connect()`。
5. 连接成功后重新刷新运动命令。

重连前建议上层先停止下发非零运动目标。

## Windows 和厂商 CAN 设备

SDK 已经抽象 `ICanTransport`。当前默认实现：

- Linux：`SocketCanTransport`。
- 其他平台：`UnsupportedCanTransport`，不会实际收发 CAN。

Windows 或特定 USB-CAN 设备需要基于厂商 SDK 实现新的 `ICanTransport` 后端，例如
Peak、Kvaser、ZLG 等。实现完成后可通过：

```cpp
auto transport = std::make_unique<MyCanTransport>();
masc_chassis_can_sdk::ChassisSdk sdk(std::move(transport));
```

注入到 `ChassisSdk`。

## 上层应用职责

SDK 不处理以下业务：

- 不主动配置 Linux CAN 接口。
- 不主动重连。
- 不等待命令 ACK。
- 不自动切换运动模式。
- 不根据故障码做急停决策。
- 不定义未明确的 fault bit 文本。
- 不假设电机或舵机的物理安装位置。

这些逻辑应由 ROS driver、诊断程序或非 ROS 上层应用完成。
