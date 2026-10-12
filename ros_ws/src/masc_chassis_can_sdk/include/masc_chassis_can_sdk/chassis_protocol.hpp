/*
 * Copyright (C) 2026, MASC.
 * @Version: V1.0
 * @Author: owen (keaa@keaa.net)
 * @Date: 2026-05-03 21:44:04
 * @LastEditTime: 2026-05-03 21:51:34
 * @LastEditors: owen (keaa@keaa.net)
 * @Description: MASC Chassis CAN V10 protocol constants and pack/parse declarations.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "masc_chassis_can_sdk/can_frame.hpp"
#include "masc_chassis_can_sdk/types.hpp"

namespace masc_chassis_can_sdk::chassis_protocol {

/// motion_control_cmd CAN ID.
constexpr std::uint16_t kMotionControlCmdId = 0x100;
/// tow_mode_control_cmd CAN ID.
constexpr std::uint16_t kTowModeCmdId = 0x101;
/// motion_mode_switch_cmd CAN ID.
constexpr std::uint16_t kModeSwitchCmdId = 0x110;
/// fault_clear_cmd CAN ID.
constexpr std::uint16_t kFaultClearCmdId = 0x120;
/// servo_calibration_cmd CAN ID.
constexpr std::uint16_t kServoCalibrationCmdId = 0x170;

/// motion_control_feedback CAN ID.
constexpr std::uint16_t kMotionFeedbackId = 0x200;
/// system_info_feedback CAN ID.
constexpr std::uint16_t kSystemFeedbackId = 0x204;
/// system_version_feedback CAN ID used by Connect() handshake.
constexpr std::uint16_t kVersionFeedbackId = 0x205;
/// bms_basic_feedback CAN ID.
constexpr std::uint16_t kBmsBasicFeedbackId = 0x206;
/// bms_fault_feedback CAN ID.
constexpr std::uint16_t kBmsFaultFeedbackId = 0x207;
/// motion_mode_feedback CAN ID.
constexpr std::uint16_t kMotionModeFeedbackId = 0x220;

/// First power_wheel_feedback CAN ID.
constexpr std::uint16_t kPowerWheelFeedbackBeginId = 0x251;
/// Last power_wheel_feedback CAN ID.
constexpr std::uint16_t kPowerWheelFeedbackEndId = 0x258;
/// First power_wheel_driver_feedback CAN ID.
constexpr std::uint16_t kPowerWheelDriverFeedbackBeginId = 0x261;
/// Last power_wheel_driver_feedback CAN ID.
constexpr std::uint16_t kPowerWheelDriverFeedbackEndId = 0x268;
/// First servo_feedback CAN ID.
constexpr std::uint16_t kServoFeedbackBeginId = 0x271;
/// Last servo_feedback CAN ID.
constexpr std::uint16_t kServoFeedbackEndId = 0x278;
/// First servo_driver_feedback CAN ID.
constexpr std::uint16_t kServoDriverFeedbackBeginId = 0x281;
/// Last servo_driver_feedback CAN ID.
constexpr std::uint16_t kServoDriverFeedbackEndId = 0x288;
/// imu_feedback CAN ID.
constexpr std::uint16_t kImuFeedbackId = 0x300;
/// rc_feedback CAN ID.
constexpr std::uint16_t kRcFeedbackId = 0x301;

/// DLC values from the MASC Chassis CAN V10 source protocol.
constexpr std::uint8_t kMotionControlCmdDlc = 0x08;
constexpr std::uint8_t kTowModeCmdDlc = 0x01;
constexpr std::uint8_t kModeSwitchCmdDlc = 0x01;
constexpr std::uint8_t kFaultClearCmdDlc = 0x02;
constexpr std::uint8_t kServoCalibrationCmdDlc = 0x02;
constexpr std::uint8_t kMotionFeedbackDlc = 0x08;
constexpr std::uint8_t kSystemFeedbackDlc = 0x08;
constexpr std::uint8_t kVersionFeedbackDlc = 0x08;
constexpr std::uint8_t kBmsBasicFeedbackDlc = 0x08;
constexpr std::uint8_t kBmsFaultFeedbackDlc = 0x08;
constexpr std::uint8_t kMotionModeFeedbackDlc = 0x02;
constexpr std::uint8_t kPowerWheelFeedbackDlc = 0x08;
constexpr std::uint8_t kPowerWheelDriverFeedbackDlc = 0x08;
constexpr std::uint8_t kServoFeedbackDlc = 0x05;
constexpr std::uint8_t kServoDriverFeedbackDlc = 0x08;
constexpr std::uint8_t kImuFeedbackDlc = 0x08;
constexpr std::uint8_t kRcFeedbackDlc = 0x08;

/// Returns true if can_id belongs to any V10 control or feedback frame known by the SDK.
bool IsSupportedCanId(std::uint16_t can_id);
/// Returns true if can_id belongs to a V10 feedback frame known by the SDK.
bool IsFeedbackCanId(std::uint16_t can_id);
/// Returns true if can_id belongs to a V10 control frame known by the SDK.
bool IsControlCanId(std::uint16_t can_id);
/// Returns the source protocol DLC for a supported CAN ID.
std::optional<std::uint8_t> GetExpectedDlc(std::uint16_t can_id);
/// Returns true when the frame uses the source protocol DLC for its CAN ID.
bool HasExpectedDlc(const CanFrame &frame);

/// Packs motion_control_cmd (0x100).
CanFrame PackMotionCommand(const MotionCommand &command);
/// Packs tow_mode_control_cmd (0x101). true enters tow mode, false exits tow mode.
CanFrame PackTowMode(bool enable);
/// Packs motion_mode_switch_cmd (0x110).
CanFrame PackMotionMode(MotionMode mode);
/// Packs fault_clear_cmd (0x120).
CanFrame PackFaultClear(std::uint16_t fault_id);
/// Packs servo_calibration_cmd (0x170).
CanFrame PackCalibrateServo(std::uint16_t servo_id);

/// Parses motion_control_feedback (0x200). Returns nullopt on wrong CAN ID, wrong DLC, or invalid
/// data.
std::optional<MotionFeedback> ParseMotionFeedback(const CanFrame &frame);
/// Parses system_info_feedback (0x204).
std::optional<SystemFeedback> ParseSystemFeedback(const CanFrame &frame);
/// Parses system_version_feedback (0x205).
std::optional<VersionInfo> ParseVersionInfo(const CanFrame &frame);
/// Parses bms_basic_feedback (0x206).
std::optional<BmsBasicFeedback> ParseBmsBasicFeedback(const CanFrame &frame);
/// Parses bms_fault_feedback (0x207).
std::optional<BmsFaultFeedback> ParseBmsFaultFeedback(const CanFrame &frame);
/// Parses motion_mode_feedback (0x220).
std::optional<MotionModeFeedback> ParseMotionModeFeedback(const CanFrame &frame);
/// Parses power_wheel_feedback (0x251-0x258) and fills the decoded index.
std::optional<PowerWheelFeedback> ParsePowerWheelFeedback(const CanFrame &frame);
/// Parses power_wheel_driver_feedback (0x261-0x268) and fills the decoded index.
std::optional<PowerWheelDriverFeedback> ParsePowerWheelDriverFeedback(const CanFrame &frame);
/// Parses servo_feedback (0x271-0x278) and fills the decoded index.
std::optional<ServoFeedback> ParseServoFeedback(const CanFrame &frame);
/// Parses servo_driver_feedback (0x281-0x288) and fills the decoded index.
std::optional<ServoDriverFeedback> ParseServoDriverFeedback(const CanFrame &frame);
/// Parses imu_feedback (0x300).
std::optional<ImuFeedback> ParseImuFeedback(const CanFrame &frame);
/// Parses rc_feedback (0x301).
std::optional<RcFeedback> ParseRcFeedback(const CanFrame &frame);

/// Converts a CAN ID in [begin_id, end_id] to a zero-based range index.
std::optional<std::size_t> GetRangeIndex(std::uint16_t can_id, std::uint16_t begin_id,
                                         std::uint16_t end_id);

}  // namespace masc_chassis_can_sdk::chassis_protocol
