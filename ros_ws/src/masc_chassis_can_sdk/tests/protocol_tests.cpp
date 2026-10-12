#include <cassert>
#include <cmath>
#include <cstdint>

#include "masc_chassis_can_sdk/chassis_protocol.hpp"

namespace {

using namespace masc_chassis_can_sdk;

constexpr double kPi = 3.14159265358979323846;

bool Near(double lhs, double rhs) {
    return std::fabs(lhs - rhs) < 1e-4;
}

void TestMotionModeSwitchChassisTypeRange() {
    assert(!IsMotionModeSwitchSupportedChassisType(0x00));
    assert(!IsMotionModeSwitchSupportedChassisType(0x5F));
    assert(!IsMotionModeSwitchSupportedChassisType(0x60));
    assert(IsMotionModeSwitchSupportedChassisType(0x61));
    assert(IsMotionModeSwitchSupportedChassisType(0x62));
    assert(IsMotionModeSwitchSupportedChassisType(0x63));
    assert(!IsMotionModeSwitchSupportedChassisType(0x64));
    assert(!IsMotionModeSwitchSupportedChassisType(0x80));
    assert(!IsMotionModeSwitchSupportedChassisType(0x81));
    assert(!IsMotionModeSwitchSupportedChassisType(0xA0));
}

void TestSupportedDlc() {
    assert(chassis_protocol::IsControlCanId(chassis_protocol::kTowModeCmdId));
    assert(chassis_protocol::IsFeedbackCanId(chassis_protocol::kImuFeedbackId));
    assert(chassis_protocol::IsFeedbackCanId(chassis_protocol::kRcFeedbackId));
    assert(chassis_protocol::GetExpectedDlc(chassis_protocol::kTowModeCmdId) ==
           chassis_protocol::kTowModeCmdDlc);
    assert(chassis_protocol::GetExpectedDlc(chassis_protocol::kImuFeedbackId) ==
           chassis_protocol::kImuFeedbackDlc);
    assert(chassis_protocol::GetExpectedDlc(chassis_protocol::kRcFeedbackId) ==
           chassis_protocol::kRcFeedbackDlc);
}

void TestPackMotionCommand() {
    MotionCommand command;
    command.m_target_linear_x_mps = 0.5;
    command.m_target_linear_y_mps = -0.25;
    command.m_target_angular_z_radps = 0.2;
    command.m_target_steering_angle_rad = 0.1;

    const auto frame = chassis_protocol::PackMotionCommand(command);
    assert(frame.m_id == chassis_protocol::kMotionControlCmdId);
    assert(frame.m_dlc == chassis_protocol::kMotionControlCmdDlc);
    assert(frame.m_data[0] == 0xF4);
    assert(frame.m_data[1] == 0x01);
    assert(frame.m_data[2] == 0x06);
    assert(frame.m_data[3] == 0xFF);
    assert(frame.m_data[4] == 0x7A);
    assert(frame.m_data[5] == 0x04);
    assert(frame.m_data[6] == 0x3D);
    assert(frame.m_data[7] == 0x02);
}

void TestReservedBytesAreZero() {
    const auto mode_frame = chassis_protocol::PackMotionMode(MotionMode::DefaultChassis);
    assert(mode_frame.m_id == chassis_protocol::kModeSwitchCmdId);
    assert(mode_frame.m_dlc == chassis_protocol::kModeSwitchCmdDlc);
    assert(mode_frame.m_data[0] == 0x00);
    for (std::size_t index = 1; index < mode_frame.m_data.size(); ++index) {
        assert(mode_frame.m_data[index] == 0x00);
    }

    const auto park_frame = chassis_protocol::PackMotionMode(MotionMode::Park);
    assert(park_frame.m_id == chassis_protocol::kModeSwitchCmdId);
    assert(park_frame.m_dlc == chassis_protocol::kModeSwitchCmdDlc);
    assert(park_frame.m_data[0] == 0x01);

    const auto fault_frame = chassis_protocol::PackFaultClear(0x0001);
    assert(fault_frame.m_dlc == chassis_protocol::kFaultClearCmdDlc);
    assert(fault_frame.m_data[0] == 0x01);
    assert(fault_frame.m_data[1] == 0x00);
    for (std::size_t index = 2; index < fault_frame.m_data.size(); ++index) {
        assert(fault_frame.m_data[index] == 0x00);
    }

    const auto servo_frame = chassis_protocol::PackCalibrateServo(0x0271);
    assert(servo_frame.m_dlc == chassis_protocol::kServoCalibrationCmdDlc);
    assert(servo_frame.m_data[0] == 0x71);
    assert(servo_frame.m_data[1] == 0x02);
    for (std::size_t index = 2; index < servo_frame.m_data.size(); ++index) {
        assert(servo_frame.m_data[index] == 0x00);
    }

    const auto tow_frame = chassis_protocol::PackTowMode(true);
    assert(tow_frame.m_id == chassis_protocol::kTowModeCmdId);
    assert(tow_frame.m_dlc == chassis_protocol::kTowModeCmdDlc);
    assert(tow_frame.m_data[0] == 0x01);
    for (std::size_t index = 1; index < tow_frame.m_data.size(); ++index) {
        assert(tow_frame.m_data[index] == 0x00);
    }

    const auto exit_tow_frame = chassis_protocol::PackTowMode(false);
    assert(exit_tow_frame.m_data[0] == 0x00);
}

void TestParseVersionInfo() {
    CanFrame frame;
    frame.m_id = chassis_protocol::kVersionFeedbackId;
    frame.m_dlc = kCanDlc;
    frame.m_data = {0x10, 0x11, 0x12, 0x13, 0x14, 0xA1, 0x60, kProtocolVersionV10};

    const auto version = chassis_protocol::ParseVersionInfo(frame);
    assert(version.has_value());
    assert(version->m_uuid[0] == 0x10);
    assert(version->m_system_software_hardware_version == 0xA1);
    assert(version->m_chassis_type_version == 0x60);
    assert(version->m_protocol_version == kProtocolVersionV10);
}

void TestRangeIndex() {
    const auto index =
        chassis_protocol::GetRangeIndex(0x254, chassis_protocol::kPowerWheelFeedbackBeginId,
                                        chassis_protocol::kPowerWheelFeedbackEndId);
    assert(index.has_value());
    assert(*index == 3);
    assert(!chassis_protocol::GetRangeIndex(0x250, chassis_protocol::kPowerWheelFeedbackBeginId,
                                            chassis_protocol::kPowerWheelFeedbackEndId)
                .has_value());
}

void TestParseMotionFeedback() {
    CanFrame frame;
    frame.m_id = chassis_protocol::kMotionFeedbackId;
    frame.m_dlc = kCanDlc;
    frame.m_data = {0xF4, 0x01, 0x00, 0x00, 0x7A, 0x04, 0x3D, 0x02};

    const auto feedback = chassis_protocol::ParseMotionFeedback(frame);
    assert(feedback.has_value());
    assert(Near(feedback->m_current_linear_x_mps, 0.5));
    assert(Near(feedback->m_current_linear_y_mps, 0.0));
    assert(Near(feedback->m_current_angular_z_radps, 0.2));
    assert(Near(feedback->m_current_steering_angle_rad, 0.1));

    frame.m_dlc = 7;
    assert(!chassis_protocol::ParseMotionFeedback(frame).has_value());
}

void TestParseSystemFeedback() {
    CanFrame frame;
    frame.m_id = chassis_protocol::kSystemFeedbackId;
    frame.m_dlc = chassis_protocol::kSystemFeedbackDlc;
    frame.m_data = {0x06, 0x1E, 0xAA, 0x55, 0x78, 0x56, 0x34, 0x12};

    const auto feedback = chassis_protocol::ParseSystemFeedback(frame);
    assert(feedback.has_value());
    assert(feedback->m_system_state == SystemState::CommControl);
    assert(feedback->m_system_state_raw == 0x06);
    assert(feedback->m_system_status_flags == 0x1E);
    assert(feedback->m_obstacle_status_flags == 0xAA);
    assert(feedback->m_system_status_reserved == 0x55);
    assert(feedback->m_raw_system_status == 0x55AA1E06);
    assert(feedback->m_system_fault == 0x12345678);

    frame.m_dlc = 7;
    assert(!chassis_protocol::ParseSystemFeedback(frame).has_value());

    frame.m_dlc = chassis_protocol::kSystemFeedbackDlc;
    frame.m_data[0] = 0x07;
    const auto unknown_state_feedback = chassis_protocol::ParseSystemFeedback(frame);
    assert(unknown_state_feedback.has_value());
    assert(unknown_state_feedback->m_system_state == SystemState::Unknown);
    assert(unknown_state_feedback->m_system_state_raw == 0x07);
    assert(unknown_state_feedback->m_system_status_flags == 0x1E);
    assert(unknown_state_feedback->m_obstacle_status_flags == 0xAA);
    assert(unknown_state_feedback->m_system_fault == 0x12345678);
}

void TestParseServoFeedbackUsesDegree() {
    CanFrame frame;
    frame.m_id = chassis_protocol::kServoFeedbackBeginId;
    frame.m_dlc = chassis_protocol::kServoFeedbackDlc;
    frame.m_data = {0x02, 0xE8, 0x03, 0x7B, 0x00, 0x00, 0x00, 0x00};

    const auto feedback = chassis_protocol::ParseServoFeedback(frame);
    assert(feedback.has_value());
    assert(feedback->m_index == 0);
    assert(feedback->m_calibration_state == ServoCalibrationState::Calibrated);
    assert(Near(feedback->m_servo_angle_deg, 10.0));
    assert(Near(feedback->m_servo_current_a, 12.3));

    frame.m_dlc = kCanDlc;
    assert(!chassis_protocol::ParseServoFeedback(frame).has_value());
}

void TestParseBmsFrames() {
    CanFrame basic_frame;
    basic_frame.m_id = chassis_protocol::kBmsBasicFeedbackId;
    basic_frame.m_dlc = kCanDlc;
    basic_frame.m_data = {80, 95, 0xF4, 0x01, 0xCE, 0xFF, 0x1C, 0x02};

    const auto basic = chassis_protocol::ParseBmsBasicFeedback(basic_frame);
    assert(basic.has_value());
    assert(basic->m_battery_soc_percent == 80);
    assert(basic->m_battery_soh_percent == 95);
    assert(Near(basic->m_battery_voltage_v, 50.0));
    assert(Near(basic->m_battery_current_a, -5.0));
    assert(Near(basic->m_battery_full_voltage_v, 54.0));

    CanFrame fault_frame;
    fault_frame.m_id = chassis_protocol::kBmsFaultFeedbackId;
    fault_frame.m_dlc = kCanDlc;
    fault_frame.m_data = {0xFA, 0x00, 0xE6, 0xFF, 0x78, 0x56, 0x34, 0x12};

    const auto fault = chassis_protocol::ParseBmsFaultFeedback(fault_frame);
    assert(fault.has_value());
    assert(Near(fault->m_battery_temp_max_deg_c, 25.0));
    assert(Near(fault->m_battery_temp_min_deg_c, -2.6));
    assert(fault->m_battery_fault == 0x12345678);
}

void TestIllegalEnum() {
    CanFrame frame;
    frame.m_id = chassis_protocol::kMotionModeFeedbackId;
    frame.m_dlc = chassis_protocol::kMotionModeFeedbackDlc;
    frame.m_data[0] = 0xFF;
    assert(!chassis_protocol::ParseMotionModeFeedback(frame).has_value());
}

void TestParseMotionModeFeedback() {
    CanFrame frame;
    frame.m_id = chassis_protocol::kMotionModeFeedbackId;
    frame.m_dlc = chassis_protocol::kMotionModeFeedbackDlc;

    frame.m_data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    auto feedback = chassis_protocol::ParseMotionModeFeedback(frame);
    assert(feedback.has_value());
    assert(feedback->m_current_motion_mode == MotionMode::DefaultChassis);
    assert(!feedback->m_motion_mode_switching);

    frame.m_data = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    feedback = chassis_protocol::ParseMotionModeFeedback(frame);
    assert(feedback.has_value());
    assert(feedback->m_current_motion_mode == MotionMode::Park);
    assert(feedback->m_motion_mode_switching);
}

void TestParseImuFeedback() {
    CanFrame frame;
    frame.m_id = chassis_protocol::kImuFeedbackId;
    frame.m_dlc = chassis_protocol::kImuFeedbackDlc;
    frame.m_data = {0xE8, 0x03, 0x18, 0xFC, 0xFA, 0x00, 0xFB, 0x00};

    const auto feedback = chassis_protocol::ParseImuFeedback(frame);
    assert(feedback.has_value());
    assert(Near(feedback->m_pitch_rad, 10.0 * kPi / 180.0));
    assert(Near(feedback->m_yaw_rad, -10.0 * kPi / 180.0));
    assert(Near(feedback->m_roll_rad, 2.5 * kPi / 180.0));
    assert(Near(feedback->m_temperature_deg_c, 25.1));

    frame.m_dlc = 7;
    assert(!chassis_protocol::ParseImuFeedback(frame).has_value());
}

void TestParseRcFeedback() {
    CanFrame frame;
    frame.m_id = chassis_protocol::kRcFeedbackId;
    frame.m_dlc = chassis_protocol::kRcFeedbackDlc;
    frame.m_data = {0x01, 0x9C, 0xCE, 0x00, 0x32, 0x64, 0xFF, 0x01};

    const auto feedback = chassis_protocol::ParseRcFeedback(frame);
    assert(feedback.has_value());
    assert(feedback->m_online);
    assert(feedback->m_channels[0] == -100);
    assert(feedback->m_channels[1] == -50);
    assert(feedback->m_channels[2] == 0);
    assert(feedback->m_channels[3] == 50);
    assert(feedback->m_channels[4] == 100);
    assert(feedback->m_channels[5] == -1);
    assert(feedback->m_channels[6] == 1);

    frame.m_dlc = 7;
    assert(!chassis_protocol::ParseRcFeedback(frame).has_value());

    frame.m_dlc = chassis_protocol::kRcFeedbackDlc;
    frame.m_data[0] = 0x02;
    assert(!chassis_protocol::ParseRcFeedback(frame).has_value());

    frame.m_data[0] = 0x00;
    frame.m_data[7] = 0x80;
    assert(!chassis_protocol::ParseRcFeedback(frame).has_value());
}

}  // namespace

int main() {
    TestMotionModeSwitchChassisTypeRange();
    TestSupportedDlc();
    TestPackMotionCommand();
    TestReservedBytesAreZero();
    TestParseVersionInfo();
    TestRangeIndex();
    TestParseMotionFeedback();
    TestParseSystemFeedback();
    TestParseServoFeedbackUsesDegree();
    TestParseBmsFrames();
    TestIllegalEnum();
    TestParseMotionModeFeedback();
    TestParseImuFeedback();
    TestParseRcFeedback();
    return 0;
}
