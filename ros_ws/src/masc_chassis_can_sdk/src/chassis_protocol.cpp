/*
 * Copyright (C) 2026, MASC.
 * @Version: V1.0
 * @Author: owen (keaa@keaa.net)
 * @Date: 2026-05-03 21:44:04
 * @LastEditTime: 2026-05-03 21:52:16
 * @LastEditors: owen (keaa@keaa.net)
 * @Description: MASC Chassis CAN V10 little-endian frame packing and parsing implementation.
 */
#include "masc_chassis_can_sdk/chassis_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace masc_chassis_can_sdk::chassis_protocol {
namespace {

constexpr double kVelocityScale = 0.001;
constexpr double kAngularVelocityScaleDegps = 0.01;
constexpr double kSteeringAngleScaleDeg = 0.01;
constexpr double kServoAngleScaleDeg = 0.01;
constexpr double kImuAngleScaleDeg = 0.01;
constexpr double kVoltageScale = 0.1;
constexpr double kCurrentScale = 0.1;
constexpr double kTemperatureScale = 0.1;
constexpr double kPositionScale = 0.001;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

bool HasExpectedHeader(const CanFrame &frame, std::uint16_t expected_id) {
    return frame.m_id == expected_id && HasExpectedDlc(frame);
}

bool IsInRange(std::uint16_t can_id, std::uint16_t begin_id, std::uint16_t end_id) {
    return can_id >= begin_id && can_id <= end_id;
}

std::int16_t ReadI16Le(const CanFrame &frame, std::size_t offset) {
    const auto raw = static_cast<std::uint16_t>(frame.m_data[offset]) |
                     (static_cast<std::uint16_t>(frame.m_data[offset + 1]) << 8U);
    return static_cast<std::int16_t>(raw);
}

std::uint16_t ReadU16Le(const CanFrame &frame, std::size_t offset) {
    return static_cast<std::uint16_t>(frame.m_data[offset]) |
           (static_cast<std::uint16_t>(frame.m_data[offset + 1]) << 8U);
}

std::int32_t ReadI32Le(const CanFrame &frame, std::size_t offset) {
    const auto raw = static_cast<std::uint32_t>(frame.m_data[offset]) |
                     (static_cast<std::uint32_t>(frame.m_data[offset + 1]) << 8U) |
                     (static_cast<std::uint32_t>(frame.m_data[offset + 2]) << 16U) |
                     (static_cast<std::uint32_t>(frame.m_data[offset + 3]) << 24U);
    return static_cast<std::int32_t>(raw);
}

std::uint32_t ReadU32Le(const CanFrame &frame, std::size_t offset) {
    return static_cast<std::uint32_t>(frame.m_data[offset]) |
           (static_cast<std::uint32_t>(frame.m_data[offset + 1]) << 8U) |
           (static_cast<std::uint32_t>(frame.m_data[offset + 2]) << 16U) |
           (static_cast<std::uint32_t>(frame.m_data[offset + 3]) << 24U);
}

std::int8_t ReadI8(const CanFrame &frame, std::size_t offset) {
    const auto raw = frame.m_data[offset];
    if (raw <= 0x7FU) {
        return static_cast<std::int8_t>(raw);
    }
    return static_cast<std::int8_t>(static_cast<int>(raw) - 0x100);
}

bool IsRcChannelValueValid(std::int8_t value) {
    return value >= -100 && value <= 100;
}

std::int16_t ClampToI16(double value) {
    if (std::isnan(value)) {
        return 0;
    }
    const auto min_value = static_cast<double>(std::numeric_limits<std::int16_t>::min());
    const auto max_value = static_cast<double>(std::numeric_limits<std::int16_t>::max());
    return static_cast<std::int16_t>(std::lround(std::clamp(value, min_value, max_value)));
}

void WriteI16Le(CanFrame *frame, std::size_t offset, std::int16_t value) {
    const auto raw = static_cast<std::uint16_t>(value);
    frame->m_data[offset] = static_cast<std::uint8_t>(raw & 0xFFU);
    frame->m_data[offset + 1] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
}

void WriteU16Le(CanFrame *frame, std::size_t offset, std::uint16_t value) {
    frame->m_data[offset] = static_cast<std::uint8_t>(value & 0xFFU);
    frame->m_data[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

std::int16_t ToScaledI16(double value, double scale) {
    return ClampToI16(value / scale);
}

CanFrame MakeEmptyFrame(std::uint16_t can_id) {
    CanFrame frame;
    frame.m_id = can_id;
    frame.m_dlc = GetExpectedDlc(can_id).value_or(kMaxCanDlc);
    frame.m_data.fill(0);
    return frame;
}

std::optional<ServoCalibrationState> ParseServoCalibrationState(std::uint8_t raw) {
    switch (raw) {
        case 0x00:
            return ServoCalibrationState::NotCalibrated;
        case 0x01:
            return ServoCalibrationState::Calibrating;
        case 0x02:
            return ServoCalibrationState::Calibrated;
        default:
            return std::nullopt;
    }
}

std::optional<MotionMode> ParseMotionModeValue(std::uint8_t raw) {
    switch (raw) {
        case 0x00:
            return MotionMode::DefaultChassis;
        case 0x01:
            return MotionMode::Park;
        case 0x02:
            return MotionMode::Ackermann;
        case 0x03:
            return MotionMode::Spin;
        case 0x04:
            return MotionMode::Lateral;
        case 0x05:
            return MotionMode::Diagonal;
        default:
            return std::nullopt;
    }
}

SystemState ParseSystemStateValue(std::uint8_t raw) {
    switch (raw) {
        case 0x00:
            return SystemState::Estop;
        case 0x01:
            return SystemState::Tow;
        case 0x02:
            return SystemState::Fault;
        case 0x03:
            return SystemState::SelfCheck;
        case 0x04:
            return SystemState::Park;
        case 0x05:
            return SystemState::RemoteControl;
        case 0x06:
            return SystemState::CommControl;
        default:
            return SystemState::Unknown;
    }
}

}  // namespace

bool IsSupportedCanId(std::uint16_t can_id) {
    return IsControlCanId(can_id) || IsFeedbackCanId(can_id);
}

bool IsFeedbackCanId(std::uint16_t can_id) {
    return can_id == kMotionFeedbackId || can_id == kSystemFeedbackId ||
           can_id == kVersionFeedbackId || can_id == kBmsBasicFeedbackId ||
           can_id == kBmsFaultFeedbackId || can_id == kMotionModeFeedbackId ||
           can_id == kImuFeedbackId || can_id == kRcFeedbackId ||
           IsInRange(can_id, kPowerWheelFeedbackBeginId, kPowerWheelFeedbackEndId) ||
           IsInRange(can_id, kPowerWheelDriverFeedbackBeginId, kPowerWheelDriverFeedbackEndId) ||
           IsInRange(can_id, kServoFeedbackBeginId, kServoFeedbackEndId) ||
           IsInRange(can_id, kServoDriverFeedbackBeginId, kServoDriverFeedbackEndId);
}

bool IsControlCanId(std::uint16_t can_id) {
    return can_id == kMotionControlCmdId || can_id == kTowModeCmdId ||
           can_id == kModeSwitchCmdId || can_id == kFaultClearCmdId ||
           can_id == kServoCalibrationCmdId;
}

std::optional<std::uint8_t> GetExpectedDlc(std::uint16_t can_id) {
    switch (can_id) {
        case kMotionControlCmdId:
            return kMotionControlCmdDlc;
        case kTowModeCmdId:
            return kTowModeCmdDlc;
        case kModeSwitchCmdId:
            return kModeSwitchCmdDlc;
        case kFaultClearCmdId:
            return kFaultClearCmdDlc;
        case kServoCalibrationCmdId:
            return kServoCalibrationCmdDlc;
        case kMotionFeedbackId:
            return kMotionFeedbackDlc;
        case kSystemFeedbackId:
            return kSystemFeedbackDlc;
        case kVersionFeedbackId:
            return kVersionFeedbackDlc;
        case kBmsBasicFeedbackId:
            return kBmsBasicFeedbackDlc;
        case kBmsFaultFeedbackId:
            return kBmsFaultFeedbackDlc;
        case kMotionModeFeedbackId:
            return kMotionModeFeedbackDlc;
        case kImuFeedbackId:
            return kImuFeedbackDlc;
        case kRcFeedbackId:
            return kRcFeedbackDlc;
        default:
            break;
    }

    if (IsInRange(can_id, kPowerWheelFeedbackBeginId, kPowerWheelFeedbackEndId)) {
        return kPowerWheelFeedbackDlc;
    }
    if (IsInRange(can_id, kPowerWheelDriverFeedbackBeginId, kPowerWheelDriverFeedbackEndId)) {
        return kPowerWheelDriverFeedbackDlc;
    }
    if (IsInRange(can_id, kServoFeedbackBeginId, kServoFeedbackEndId)) {
        return kServoFeedbackDlc;
    }
    if (IsInRange(can_id, kServoDriverFeedbackBeginId, kServoDriverFeedbackEndId)) {
        return kServoDriverFeedbackDlc;
    }
    return std::nullopt;
}

bool HasExpectedDlc(const CanFrame &frame) {
    const auto expected_dlc = GetExpectedDlc(frame.m_id);
    return expected_dlc.has_value() && frame.m_dlc == *expected_dlc;
}

CanFrame PackMotionCommand(const MotionCommand &command) {
    auto frame = MakeEmptyFrame(kMotionControlCmdId);
    WriteI16Le(&frame, 0, ToScaledI16(command.m_target_linear_x_mps, kVelocityScale));
    WriteI16Le(&frame, 2, ToScaledI16(command.m_target_linear_y_mps, kVelocityScale));
    WriteI16Le(&frame, 4,
               ToScaledI16(command.m_target_angular_z_radps * kRadToDeg,
                           kAngularVelocityScaleDegps));
    WriteI16Le(&frame, 6,
               ToScaledI16(command.m_target_steering_angle_rad * kRadToDeg,
                           kSteeringAngleScaleDeg));
    return frame;
}

CanFrame PackTowMode(bool enable) {
    auto frame = MakeEmptyFrame(kTowModeCmdId);
    frame.m_data[0] = enable ? 0x01 : 0x00;
    return frame;
}

CanFrame PackMotionMode(MotionMode mode) {
    auto frame = MakeEmptyFrame(kModeSwitchCmdId);
    frame.m_data[0] = static_cast<std::uint8_t>(mode);
    return frame;
}

CanFrame PackFaultClear(std::uint16_t fault_id) {
    auto frame = MakeEmptyFrame(kFaultClearCmdId);
    WriteU16Le(&frame, 0, fault_id);
    return frame;
}

CanFrame PackCalibrateServo(std::uint16_t servo_id) {
    auto frame = MakeEmptyFrame(kServoCalibrationCmdId);
    WriteU16Le(&frame, 0, servo_id);
    return frame;
}

std::optional<MotionFeedback> ParseMotionFeedback(const CanFrame &frame) {
    if (!HasExpectedHeader(frame, kMotionFeedbackId)) {
        return std::nullopt;
    }

    MotionFeedback feedback;
    feedback.m_current_linear_x_mps = static_cast<double>(ReadI16Le(frame, 0)) * kVelocityScale;
    feedback.m_current_linear_y_mps = static_cast<double>(ReadI16Le(frame, 2)) * kVelocityScale;
    feedback.m_current_angular_z_radps =
        static_cast<double>(ReadI16Le(frame, 4)) * kAngularVelocityScaleDegps * kDegToRad;
    feedback.m_current_steering_angle_rad =
        static_cast<double>(ReadI16Le(frame, 6)) * kSteeringAngleScaleDeg * kDegToRad;
    return feedback;
}

std::optional<SystemFeedback> ParseSystemFeedback(const CanFrame &frame) {
    if (!HasExpectedHeader(frame, kSystemFeedbackId)) {
        return std::nullopt;
    }

    SystemFeedback feedback;
    feedback.m_system_state = ParseSystemStateValue(frame.m_data[0]);
    feedback.m_system_state_raw = frame.m_data[0];
    feedback.m_system_status_flags = frame.m_data[1];
    feedback.m_obstacle_status_flags = frame.m_data[2];
    feedback.m_system_status_reserved = frame.m_data[3];
    feedback.m_raw_system_status = ReadU32Le(frame, 0);
    feedback.m_system_fault = ReadU32Le(frame, 4);
    return feedback;
}

std::optional<VersionInfo> ParseVersionInfo(const CanFrame &frame) {
    if (!HasExpectedHeader(frame, kVersionFeedbackId)) {
        return std::nullopt;
    }

    VersionInfo info;
    for (std::size_t index = 0; index < info.m_uuid.size(); ++index) {
        info.m_uuid[index] = frame.m_data[index];
    }
    info.m_system_software_hardware_version = frame.m_data[5];
    info.m_chassis_type_version = frame.m_data[6];
    info.m_protocol_version = frame.m_data[7];
    return info;
}

std::optional<BmsBasicFeedback> ParseBmsBasicFeedback(const CanFrame &frame) {
    if (!HasExpectedHeader(frame, kBmsBasicFeedbackId)) {
        return std::nullopt;
    }

    BmsBasicFeedback feedback;
    feedback.m_battery_soc_percent = frame.m_data[0];
    feedback.m_battery_soh_percent = frame.m_data[1];
    feedback.m_battery_voltage_v = static_cast<double>(ReadU16Le(frame, 2)) * kVoltageScale;
    feedback.m_battery_current_a = static_cast<double>(ReadI16Le(frame, 4)) * kCurrentScale;
    feedback.m_battery_full_voltage_v = static_cast<double>(ReadU16Le(frame, 6)) * kVoltageScale;
    return feedback;
}

std::optional<BmsFaultFeedback> ParseBmsFaultFeedback(const CanFrame &frame) {
    if (!HasExpectedHeader(frame, kBmsFaultFeedbackId)) {
        return std::nullopt;
    }

    BmsFaultFeedback feedback;
    feedback.m_battery_temp_max_deg_c =
        static_cast<double>(ReadI16Le(frame, 0)) * kTemperatureScale;
    feedback.m_battery_temp_min_deg_c =
        static_cast<double>(ReadI16Le(frame, 2)) * kTemperatureScale;
    feedback.m_battery_fault = ReadU32Le(frame, 4);
    return feedback;
}

std::optional<MotionModeFeedback> ParseMotionModeFeedback(const CanFrame &frame) {
    if (!HasExpectedHeader(frame, kMotionModeFeedbackId)) {
        return std::nullopt;
    }

    const auto mode = ParseMotionModeValue(frame.m_data[0]);
    if (!mode.has_value()) {
        return std::nullopt;
    }
    if (frame.m_data[1] > 0x01) {
        return std::nullopt;
    }

    MotionModeFeedback feedback;
    feedback.m_current_motion_mode = *mode;
    feedback.m_motion_mode_switching = frame.m_data[1] != 0;
    return feedback;
}

std::optional<PowerWheelFeedback> ParsePowerWheelFeedback(const CanFrame &frame) {
    const auto index =
        GetRangeIndex(frame.m_id, kPowerWheelFeedbackBeginId, kPowerWheelFeedbackEndId);
    if (!index.has_value() || !HasExpectedDlc(frame)) {
        return std::nullopt;
    }

    PowerWheelFeedback feedback;
    feedback.m_index = *index;
    feedback.m_wheel_speed_mps = static_cast<double>(ReadI16Le(frame, 0)) * kVelocityScale;
    feedback.m_wheel_current_a = static_cast<double>(ReadI16Le(frame, 2)) * kCurrentScale;
    feedback.m_wheel_position_m = static_cast<double>(ReadI32Le(frame, 4)) * kPositionScale;
    return feedback;
}

std::optional<PowerWheelDriverFeedback> ParsePowerWheelDriverFeedback(const CanFrame &frame) {
    const auto index = GetRangeIndex(frame.m_id, kPowerWheelDriverFeedbackBeginId,
                                     kPowerWheelDriverFeedbackEndId);
    if (!index.has_value() || !HasExpectedDlc(frame)) {
        return std::nullopt;
    }

    PowerWheelDriverFeedback feedback;
    feedback.m_index = *index;
    feedback.m_motor_temperature_deg_c =
        static_cast<double>(ReadI16Le(frame, 0)) * kTemperatureScale;
    feedback.m_driver_temperature_deg_c =
        static_cast<double>(ReadI16Le(frame, 2)) * kTemperatureScale;
    feedback.m_driver_fault = ReadU32Le(frame, 4);
    return feedback;
}

std::optional<ServoFeedback> ParseServoFeedback(const CanFrame &frame) {
    const auto index = GetRangeIndex(frame.m_id, kServoFeedbackBeginId, kServoFeedbackEndId);
    if (!index.has_value() || !HasExpectedDlc(frame)) {
        return std::nullopt;
    }

    const auto state = ParseServoCalibrationState(frame.m_data[0]);
    if (!state.has_value()) {
        return std::nullopt;
    }

    ServoFeedback feedback;
    feedback.m_index = *index;
    feedback.m_calibration_state = *state;
    feedback.m_servo_angle_deg = static_cast<double>(ReadI16Le(frame, 1)) * kServoAngleScaleDeg;
    feedback.m_servo_current_a = static_cast<double>(ReadI16Le(frame, 3)) * kCurrentScale;
    return feedback;
}

std::optional<ServoDriverFeedback> ParseServoDriverFeedback(const CanFrame &frame) {
    const auto index =
        GetRangeIndex(frame.m_id, kServoDriverFeedbackBeginId, kServoDriverFeedbackEndId);
    if (!index.has_value() || !HasExpectedDlc(frame)) {
        return std::nullopt;
    }

    ServoDriverFeedback feedback;
    feedback.m_index = *index;
    feedback.m_motor_temperature_deg_c =
        static_cast<double>(ReadI16Le(frame, 0)) * kTemperatureScale;
    feedback.m_driver_temperature_deg_c =
        static_cast<double>(ReadI16Le(frame, 2)) * kTemperatureScale;
    feedback.m_driver_fault = ReadU32Le(frame, 4);
    return feedback;
}

std::optional<ImuFeedback> ParseImuFeedback(const CanFrame &frame) {
    if (!HasExpectedHeader(frame, kImuFeedbackId)) {
        return std::nullopt;
    }

    ImuFeedback feedback;
    feedback.m_pitch_rad = static_cast<double>(ReadI16Le(frame, 0)) * kImuAngleScaleDeg * kDegToRad;
    feedback.m_yaw_rad = static_cast<double>(ReadI16Le(frame, 2)) * kImuAngleScaleDeg * kDegToRad;
    feedback.m_roll_rad = static_cast<double>(ReadI16Le(frame, 4)) * kImuAngleScaleDeg * kDegToRad;
    feedback.m_temperature_deg_c = static_cast<double>(ReadI16Le(frame, 6)) * kTemperatureScale;
    return feedback;
}

std::optional<RcFeedback> ParseRcFeedback(const CanFrame &frame) {
    if (!HasExpectedHeader(frame, kRcFeedbackId)) {
        return std::nullopt;
    }

    RcFeedback feedback;
    const auto online = frame.m_data[0];
    if (online > 0x01U) {
        return std::nullopt;
    }
    feedback.m_online = online == 0x01U;
    for (std::size_t index = 0; index < feedback.m_channels.size(); ++index) {
        const auto channel = ReadI8(frame, index + 1);
        if (!IsRcChannelValueValid(channel)) {
            return std::nullopt;
        }
        feedback.m_channels[index] = channel;
    }
    return feedback;
}

std::optional<std::size_t> GetRangeIndex(std::uint16_t can_id, std::uint16_t begin_id,
                                         std::uint16_t end_id) {
    if (!IsInRange(can_id, begin_id, end_id)) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(can_id - begin_id);
}

}  // namespace masc_chassis_can_sdk::chassis_protocol
