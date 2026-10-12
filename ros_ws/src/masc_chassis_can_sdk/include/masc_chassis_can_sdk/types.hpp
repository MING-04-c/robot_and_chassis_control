/*
 * Copyright (C) 2026, MASC.
 * @Version: V1.0
 * @Author: owen (keaa@keaa.net)
 * @Date: 2026-05-03 21:37:05
 * @LastEditTime: 2026-05-03 21:52:04
 * @LastEditors: owen (keaa@keaa.net)
 * @Description: Public SDK data models, configuration, decoded feedback, and health types.
 */
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace masc_chassis_can_sdk {

/// Current protocol version byte reported by system_version_feedback (0x205) for CAN V10.
constexpr std::uint8_t kProtocolVersionV10 = 0x0A;
/// Maximum number of indexed wheel/servo devices covered by one CAN ID range.
constexpr std::size_t kMaxIndexedDevices = 8;
/// Number of RC channels in rc_feedback (0x301).
constexpr std::size_t kRcChannelCount = 7;
/// Reserved/unspecified chassis type value from system_version_feedback (0x205) Byte6.
constexpr std::uint8_t kChassisTypeReserved = 0x00;
/// Four-wheel all-wheel-steer / steer-wheel chassis type.
constexpr std::uint8_t kChassisTypeAws4ws = 0x61;
/// Six-wheel all-wheel-steer / steer-wheel chassis type.
constexpr std::uint8_t kChassisTypeAws6ws = 0x62;
/// Eight-wheel all-wheel-steer / steer-wheel chassis type.
constexpr std::uint8_t kChassisTypeAws8ws = 0x63;

/// Returns whether the chassis type supports motion_mode_switch_cmd (0x110) and
/// motion_mode_feedback (0x220).
inline bool IsMotionModeSwitchSupportedChassisType(std::uint8_t chassis_type_version) {
    return chassis_type_version == kChassisTypeAws4ws ||
           chassis_type_version == kChassisTypeAws6ws ||
           chassis_type_version == kChassisTypeAws8ws;
}

/// Motion mode values used by motion_mode_switch_cmd (0x110) and motion_mode_feedback (0x220).
enum class MotionMode : std::uint8_t {
    DefaultChassis = 0x00,
    Park = 0x01,
    Ackermann = 0x02,
    Spin = 0x03,
    Lateral = 0x04,
    Diagonal = 0x05,
};

/// Servo automatic calibration state reported by servo_feedback (0x271-0x278).
enum class ServoCalibrationState : std::uint8_t {
    NotCalibrated = 0x00,
    Calibrating = 0x01,
    Calibrated = 0x02,
};

/// Primary chassis/system state reported by system_info_feedback (0x204) Byte0.
enum class SystemState : std::uint8_t {
    Unknown = 0xFF,
    Estop = 0x00,
    Tow = 0x01,
    Fault = 0x02,
    SelfCheck = 0x03,
    Park = 0x04,
    RemoteControl = 0x05,
    CommControl = 0x06,
};

/// system_info_feedback (0x204) Byte1 status flag: brake active.
constexpr std::uint8_t kSystemStatusFlagBrake = static_cast<std::uint8_t>(1U << 0U);
/// system_info_feedback (0x204) Byte1 status flag: low battery.
constexpr std::uint8_t kSystemStatusFlagLowBattery = static_cast<std::uint8_t>(1U << 1U);
/// system_info_feedback (0x204) Byte1 status flag: excessive tilt.
constexpr std::uint8_t kSystemStatusFlagExcessiveTilt = static_cast<std::uint8_t>(1U << 2U);
/// system_info_feedback (0x204) Byte1 status flag: manual charging.
constexpr std::uint8_t kSystemStatusFlagManualCharging = static_cast<std::uint8_t>(1U << 3U);
/// system_info_feedback (0x204) Byte1 status flag: dock charging.
constexpr std::uint8_t kSystemStatusFlagDockCharging = static_cast<std::uint8_t>(1U << 4U);
/// system_info_feedback (0x204) Byte1 status flag: discharge contactor active.
constexpr std::uint8_t kSystemStatusFlagDischargeContactor =
    static_cast<std::uint8_t>(1U << 5U);
/// system_info_feedback (0x204) Byte1 status flag: precharge contactor active.
constexpr std::uint8_t kSystemStatusFlagPrechargeContactor =
    static_cast<std::uint8_t>(1U << 6U);
/// system_info_feedback (0x204) Byte1 status flag: main contactor active.
constexpr std::uint8_t kSystemStatusFlagMainContactor = static_cast<std::uint8_t>(1U << 7U);

/// system_info_feedback (0x204) Byte2 obstacle flag: front obstacle.
constexpr std::uint8_t kObstacleStatusFlagFront = static_cast<std::uint8_t>(1U << 0U);
/// system_info_feedback (0x204) Byte2 obstacle flag: rear obstacle.
constexpr std::uint8_t kObstacleStatusFlagRear = static_cast<std::uint8_t>(1U << 1U);
/// system_info_feedback (0x204) Byte2 obstacle flag: left-front obstacle.
constexpr std::uint8_t kObstacleStatusFlagLeftFront = static_cast<std::uint8_t>(1U << 2U);
/// system_info_feedback (0x204) Byte2 obstacle flag: left-rear obstacle.
constexpr std::uint8_t kObstacleStatusFlagLeftRear = static_cast<std::uint8_t>(1U << 3U);
/// system_info_feedback (0x204) Byte2 obstacle flag: right-front obstacle.
constexpr std::uint8_t kObstacleStatusFlagRightFront = static_cast<std::uint8_t>(1U << 4U);
/// system_info_feedback (0x204) Byte2 obstacle flag: right-rear obstacle.
constexpr std::uint8_t kObstacleStatusFlagRightRear = static_cast<std::uint8_t>(1U << 5U);

/// SDK configuration.
///
/// The SDK does not configure the OS CAN interface bitrate itself. The bitrate is kept here as
/// desired configuration metadata and for future transport implementations.
struct CanConfig {
    /// CAN interface name, e.g. "can0" on Linux.
    std::string m_interface_name = "can0";
    /// Expected CAN bitrate in bit/s.
    int m_bitrate = 500000;
    /// Required protocol version from system_version_feedback (0x205) before Connect() succeeds.
    std::uint8_t m_expected_protocol_version = kProtocolVersionV10;
    /// How long Connect() waits for a valid system_version_feedback (0x205) frame.
    int m_connect_timeout_ms = 1000;
    /// Period for SDK-owned motion_control_cmd (0x100) sending.
    int m_motion_command_period_ms = 20;
    /// If SetMotionCommand() is not refreshed within this time, SDK sends zero motion.
    int m_motion_command_timeout_ms = 100;
};

/// Outgoing motion_control_cmd (0x100) in SI units.
struct MotionCommand {
    double m_target_linear_x_mps = 0.0;
    double m_target_linear_y_mps = 0.0;
    double m_target_angular_z_radps = 0.0;
    double m_target_steering_angle_rad = 0.0;
};

/// Decoded motion_control_feedback (0x200) in SI units.
struct MotionFeedback {
    double m_current_linear_x_mps = 0.0;
    double m_current_linear_y_mps = 0.0;
    double m_current_angular_z_radps = 0.0;
    double m_current_steering_angle_rad = 0.0;
};

/// Decoded motion_mode_feedback (0x220).
struct MotionModeFeedback {
    MotionMode m_current_motion_mode = MotionMode::DefaultChassis;
    bool m_motion_mode_switching = false;
};

/// Decoded system_info_feedback (0x204) primary state, status flags, obstacle flags, and raw fault
/// code.
///
/// Fault bits are intentionally raw because V10 fault definitions are still reserved/undefined.
struct SystemFeedback {
    SystemState m_system_state = SystemState::Unknown;
    std::uint8_t m_system_state_raw = 0;
    std::uint8_t m_system_status_flags = 0;
    std::uint8_t m_obstacle_status_flags = 0;
    std::uint8_t m_system_status_reserved = 0;
    std::uint32_t m_raw_system_status = 0;
    std::uint32_t m_system_fault = 0;
};

/// Decoded system_version_feedback (0x205) frame.
struct VersionInfo {
    std::array<std::uint8_t, 5> m_uuid{};
    std::uint8_t m_system_software_hardware_version = 0;
    std::uint8_t m_chassis_type_version = 0;
    std::uint8_t m_protocol_version = 0;
};

/// Decoded bms_basic_feedback (0x206).
struct BmsBasicFeedback {
    std::uint8_t m_battery_soc_percent = 0;
    std::uint8_t m_battery_soh_percent = 0;
    double m_battery_voltage_v = 0.0;
    double m_battery_current_a = 0.0;
    double m_battery_full_voltage_v = 0.0;
};

/// Decoded bms_fault_feedback (0x207).
struct BmsFaultFeedback {
    double m_battery_temp_max_deg_c = 0.0;
    double m_battery_temp_min_deg_c = 0.0;
    std::uint32_t m_battery_fault = 0;
};

/// Decoded power_wheel_feedback (0x251-0x258).
///
/// m_index is computed as can_id - 0x251.
struct PowerWheelFeedback {
    std::size_t m_index = 0;
    double m_wheel_speed_mps = 0.0;
    double m_wheel_current_a = 0.0;
    double m_wheel_position_m = 0.0;
};

/// Decoded power_wheel_driver_feedback (0x261-0x268).
///
/// m_index is computed as can_id - 0x261.
struct PowerWheelDriverFeedback {
    std::size_t m_index = 0;
    double m_motor_temperature_deg_c = 0.0;
    double m_driver_temperature_deg_c = 0.0;
    std::uint32_t m_driver_fault = 0;
};

/// Decoded servo_feedback (0x271-0x278).
///
/// m_index is computed as can_id - 0x271.
struct ServoFeedback {
    std::size_t m_index = 0;
    ServoCalibrationState m_calibration_state = ServoCalibrationState::NotCalibrated;
    double m_servo_angle_deg = 0.0;
    double m_servo_current_a = 0.0;
};

/// Decoded servo_driver_feedback (0x281-0x288).
///
/// m_index is computed as can_id - 0x281.
struct ServoDriverFeedback {
    std::size_t m_index = 0;
    double m_motor_temperature_deg_c = 0.0;
    double m_driver_temperature_deg_c = 0.0;
    std::uint32_t m_driver_fault = 0;
};

/// Decoded imu_feedback (0x300).
struct ImuFeedback {
    double m_pitch_rad = 0.0;
    double m_yaw_rad = 0.0;
    double m_roll_rad = 0.0;
    double m_temperature_deg_c = 0.0;
};

/// Decoded rc_feedback (0x301).
struct RcFeedback {
    bool m_online = false;
    std::array<std::int8_t, kRcChannelCount> m_channels{};
};

/// Runtime statistics for one CAN ID.
struct FrameStats {
    std::uint16_t m_can_id = 0;
    std::uint64_t m_rx_count = 0;
    std::uint64_t m_parse_error_count = 0;
    std::uint64_t m_drop_count = 0;
    double m_frequency_hz = 0.0;
    std::optional<std::chrono::steady_clock::time_point> m_last_rx_time;
};

/// Snapshot of SDK transport/protocol health.
///
/// This is passive state for upper-layer decisions. The SDK records errors but does not reconnect
/// automatically.
struct HealthSnapshot {
    bool m_socket_open = false;
    bool m_connected = false;
    bool m_protocol_version_matched = false;
    bool m_motion_command_timeout_active = false;
    std::uint64_t m_connection_attempt_count = 0;
    std::uint64_t m_transport_error_count = 0;
    std::uint64_t m_tx_error_count = 0;
    std::uint64_t m_rx_error_count = 0;
    std::optional<VersionInfo> m_version_info;
};

}  // namespace masc_chassis_can_sdk
