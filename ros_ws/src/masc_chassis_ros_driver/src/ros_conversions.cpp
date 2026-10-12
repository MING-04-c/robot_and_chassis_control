/*
 * Copyright (C) 2026, MASC.
 * @Description: SDK to ROS message conversion helpers.
 */
#include "masc_chassis_ros_driver/ros_conversions.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace masc_chassis_ros_driver {
namespace {

// 本文件只负责 SDK 结构体 -> ROS 消息的单位和枚举转换。
// CAN 协议解析发生在 can_sdk 中，节点业务判断发生在 chassis_ros_driver_node 中。

constexpr double kPi = 3.14159265358979323846;

bool HasFlag(std::uint8_t flags, std::uint8_t flag) {
    return (flags & flag) != 0U;
}

void FillQuaternionFromRollPitchYaw(double roll, double pitch, double yaw, rosmsg::Imu *msg) {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    msg->orientation.w = cr * cp * cy + sr * sp * sy;
    msg->orientation.x = sr * cp * cy - cr * sp * sy;
    msg->orientation.y = cr * sp * cy + sr * cp * sy;
    msg->orientation.z = cr * cp * sy - sr * sp * cy;
}

}  // namespace

rosmsg::Header MakeHeader(const RosTime &stamp, const std::string &frame_id) {
    rosmsg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id;
    return header;
}

std::optional<masc_chassis_can_sdk::MotionMode> ToSdkMotionMode(std::uint8_t mode) {
    using MotionMode = masc_chassis_can_sdk::MotionMode;
    switch (mode) {
        case rosmsg::ChassisStatus::MOTION_MODE_DEFAULT_CHASSIS:
            return MotionMode::DefaultChassis;
        case rosmsg::ChassisStatus::MOTION_MODE_PARK:
            return MotionMode::Park;
        case rosmsg::ChassisStatus::MOTION_MODE_ACKERMANN:
            return MotionMode::Ackermann;
        case rosmsg::ChassisStatus::MOTION_MODE_SPIN:
            return MotionMode::Spin;
        case rosmsg::ChassisStatus::MOTION_MODE_LATERAL:
            return MotionMode::Lateral;
        case rosmsg::ChassisStatus::MOTION_MODE_DIAGONAL:
            return MotionMode::Diagonal;
        default:
            return std::nullopt;
    }
}

std::uint8_t ToRosMotionMode(masc_chassis_can_sdk::MotionMode mode) {
    switch (mode) {
        case masc_chassis_can_sdk::MotionMode::DefaultChassis:
            return rosmsg::ChassisStatus::MOTION_MODE_DEFAULT_CHASSIS;
        case masc_chassis_can_sdk::MotionMode::Park:
            return rosmsg::ChassisStatus::MOTION_MODE_PARK;
        case masc_chassis_can_sdk::MotionMode::Ackermann:
            return rosmsg::ChassisStatus::MOTION_MODE_ACKERMANN;
        case masc_chassis_can_sdk::MotionMode::Spin:
            return rosmsg::ChassisStatus::MOTION_MODE_SPIN;
        case masc_chassis_can_sdk::MotionMode::Lateral:
            return rosmsg::ChassisStatus::MOTION_MODE_LATERAL;
        case masc_chassis_can_sdk::MotionMode::Diagonal:
            return rosmsg::ChassisStatus::MOTION_MODE_DIAGONAL;
    }
    return rosmsg::ChassisStatus::MOTION_MODE_DEFAULT_CHASSIS;
}

std::uint8_t ToRosServoCalibrationState(masc_chassis_can_sdk::ServoCalibrationState state) {
    switch (state) {
        case masc_chassis_can_sdk::ServoCalibrationState::NotCalibrated:
            return rosmsg::ServoMotor::SERVO_CALIBRATION_NOT_CALIBRATED;
        case masc_chassis_can_sdk::ServoCalibrationState::Calibrating:
            return rosmsg::ServoMotor::SERVO_CALIBRATION_CALIBRATING;
        case masc_chassis_can_sdk::ServoCalibrationState::Calibrated:
            return rosmsg::ServoMotor::SERVO_CALIBRATION_CALIBRATED;
    }
    return rosmsg::ServoMotor::SERVO_CALIBRATION_NOT_CALIBRATED;
}

std::uint8_t ToRosChassisState(masc_chassis_can_sdk::SystemState state) {
    switch (state) {
        case masc_chassis_can_sdk::SystemState::Unknown:
            return rosmsg::ChassisStatus::CHASSIS_STATE_UNKNOWN;
        case masc_chassis_can_sdk::SystemState::Estop:
            return rosmsg::ChassisStatus::CHASSIS_STATE_ESTOP;
        case masc_chassis_can_sdk::SystemState::Tow:
            return rosmsg::ChassisStatus::CHASSIS_STATE_TOW;
        case masc_chassis_can_sdk::SystemState::Fault:
            return rosmsg::ChassisStatus::CHASSIS_STATE_FAULT;
        case masc_chassis_can_sdk::SystemState::SelfCheck:
            return rosmsg::ChassisStatus::CHASSIS_STATE_SELF_CHECK;
        case masc_chassis_can_sdk::SystemState::Park:
            return rosmsg::ChassisStatus::CHASSIS_STATE_PARK;
        case masc_chassis_can_sdk::SystemState::RemoteControl:
            return rosmsg::ChassisStatus::CHASSIS_STATE_REMOTE_CONTROL;
        case masc_chassis_can_sdk::SystemState::CommControl:
            return rosmsg::ChassisStatus::CHASSIS_STATE_COMM_CONTROL;
    }
    return rosmsg::ChassisStatus::CHASSIS_STATE_UNKNOWN;
}

rosmsg::ChassisStatus BuildChassisStatus(
    const RosTime &stamp, const std::optional<masc_chassis_can_sdk::SystemFeedback> &system,
    const std::optional<masc_chassis_can_sdk::MotionModeFeedback> &mode) {
    // system_feedback 提供系统状态/故障/flags，motion_mode_feedback 提供当前运动模式。
    rosmsg::ChassisStatus msg;
    msg.header = MakeHeader(stamp);
    msg.chassis_state = rosmsg::ChassisStatus::CHASSIS_STATE_UNKNOWN;
    msg.current_motion_mode = rosmsg::ChassisStatus::MOTION_MODE_DEFAULT_CHASSIS;

    if (system.has_value()) {
        msg.brake_active =
            HasFlag(system->m_system_status_flags, masc_chassis_can_sdk::kSystemStatusFlagBrake);
        msg.low_battery =
            HasFlag(system->m_system_status_flags, masc_chassis_can_sdk::kSystemStatusFlagLowBattery);
        msg.excessive_tilt = HasFlag(system->m_system_status_flags,
                                     masc_chassis_can_sdk::kSystemStatusFlagExcessiveTilt);
        msg.manual_charging = HasFlag(system->m_system_status_flags,
                                      masc_chassis_can_sdk::kSystemStatusFlagManualCharging);
        msg.dock_charging = HasFlag(system->m_system_status_flags,
                                    masc_chassis_can_sdk::kSystemStatusFlagDockCharging);
        msg.main_power_active =
            HasFlag(system->m_system_status_flags,
                    masc_chassis_can_sdk::kSystemStatusFlagMainContactor);
        msg.obstacle_status_flags = system->m_obstacle_status_flags;
        msg.system_status_flags = system->m_raw_system_status;
        msg.chassis_fault = system->m_system_fault;
        msg.chassis_state = ToRosChassisState(system->m_system_state);
    }
    if (mode.has_value()) {
        msg.current_motion_mode = ToRosMotionMode(mode->m_current_motion_mode);
        msg.motion_mode_switching = mode->m_motion_mode_switching;
    }
    return msg;
}

rosmsg::BmsStatus BuildBmsStatus(
    const RosTime &stamp, bool timed_out,
    const std::optional<masc_chassis_can_sdk::BmsBasicFeedback> &basic,
    const std::optional<masc_chassis_can_sdk::BmsFaultFeedback> &fault) {
    // BMS 协议未提供的字段保持 ROS 消息默认值；超时优先于普通健康状态。
    rosmsg::BmsStatus msg;
    msg.header = MakeHeader(stamp);
    if (basic.has_value()) {
        msg.bat_soc = basic->m_battery_soc_percent;
        msg.bat_soh = basic->m_battery_soh_percent;
        msg.bat_voltage = basic->m_battery_voltage_v;
        msg.bat_current = basic->m_battery_current_a;
        msg.bat_max_voltage = basic->m_battery_full_voltage_v;
    }
    if (fault.has_value()) {
        msg.bat_temp_max = fault->m_battery_temp_max_deg_c;
        msg.bat_temp_min = fault->m_battery_temp_min_deg_c;
    }

    if (timed_out) {
        msg.bms_health = rosmsg::BmsStatus::BMS_HEALTH_TIMEOUT;
    } else if (fault.has_value() && fault->m_battery_fault != 0u) {
        msg.bms_health = rosmsg::BmsStatus::BMS_HEALTH_UNSPEC_FAILURE;
    } else if (basic.has_value()) {
        msg.bms_health = rosmsg::BmsStatus::BMS_HEALTH_GOOD;
    } else {
        msg.bms_health = rosmsg::BmsStatus::BMS_HEALTH_UNKNOWN;
    }
    return msg;
}

rosmsg::DriveMotor BuildDriveMotor(const RosTime &stamp,
                                   const masc_chassis_can_sdk::PowerWheelFeedback &feedback) {
    rosmsg::DriveMotor msg;
    msg.header = MakeHeader(stamp);
    msg.index = static_cast<std::uint8_t>(feedback.m_index);
    msg.speed = feedback.m_wheel_speed_mps;
    msg.current = feedback.m_wheel_current_a;
    msg.position = feedback.m_wheel_position_m;
    return msg;
}

rosmsg::DriveMotorStatus BuildDriveMotorStatus(
    const RosTime &stamp, const masc_chassis_can_sdk::PowerWheelDriverFeedback &feedback) {
    rosmsg::DriveMotorStatus msg;
    msg.header = MakeHeader(stamp);
    msg.index = static_cast<std::uint8_t>(feedback.m_index);
    msg.motor_temperature = feedback.m_motor_temperature_deg_c;
    msg.driver_temperature = feedback.m_driver_temperature_deg_c;
    msg.driver_fault = feedback.m_driver_fault;
    return msg;
}

rosmsg::ServoMotor BuildServoMotor(const RosTime &stamp,
                                   const masc_chassis_can_sdk::ServoFeedback &feedback) {
    rosmsg::ServoMotor msg;
    msg.header = MakeHeader(stamp);
    msg.index = static_cast<std::uint8_t>(feedback.m_index);
    msg.calibration_state = ToRosServoCalibrationState(feedback.m_calibration_state);
    msg.angle = feedback.m_servo_angle_deg * kPi / 180.0;
    msg.current = feedback.m_servo_current_a;
    return msg;
}

rosmsg::ServoMotorStatus BuildServoMotorStatus(
    const RosTime &stamp, const masc_chassis_can_sdk::ServoDriverFeedback &feedback) {
    rosmsg::ServoMotorStatus msg;
    msg.header = MakeHeader(stamp);
    msg.index = static_cast<std::uint8_t>(feedback.m_index);
    msg.motor_temperature = feedback.m_motor_temperature_deg_c;
    msg.driver_temperature = feedback.m_driver_temperature_deg_c;
    msg.driver_fault = feedback.m_driver_fault;
    return msg;
}

rosmsg::Imu BuildImu(const RosTime &stamp, const std::string &frame_id,
                     const masc_chassis_can_sdk::ImuFeedback &feedback) {
    // SDK 中姿态已经是 rad，这里只把 roll/pitch/yaw 转成 ROS 四元数。
    // 当前 CAN 帧没有角速度和线加速度，因此对应 covariance 首项置为 -1 表示未知。
    rosmsg::Imu msg;
    msg.header = MakeHeader(stamp, frame_id);
    FillQuaternionFromRollPitchYaw(feedback.m_roll_rad, feedback.m_pitch_rad, feedback.m_yaw_rad,
                                   &msg);
    msg.angular_velocity_covariance[0] = -1.0;
    msg.linear_acceleration_covariance[0] = -1.0;
    return msg;
}

rosmsg::RcStatus BuildRcStatus(const RosTime &stamp,
                               const masc_chassis_can_sdk::RcFeedback &feedback) {
    rosmsg::RcStatus msg;
    msg.header = MakeHeader(stamp);
    msg.online = feedback.m_online;
    for (std::size_t i = 0; i < feedback.m_channels.size(); ++i) {
        msg.channels[i] = feedback.m_channels[i];
    }
    return msg;
}

}  // namespace masc_chassis_ros_driver
