/*
 * Copyright (C) 2026, MASC.
 * @Description: SDK to ROS message conversion helpers.
 */
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "masc_chassis_can_sdk/types.hpp"
#include "masc_chassis_ros_driver/ros_api.hpp"
#include "masc_chassis_ros_driver/ros_msgs.hpp"

namespace masc_chassis_ros_driver {

// SDK 与 ROS 消息之间的“表示层适配”：枚举映射、单位转换、消息组装均集中在这里。

rosmsg::Header MakeHeader(const RosTime &stamp, const std::string &frame_id = "");

std::optional<masc_chassis_can_sdk::MotionMode> ToSdkMotionMode(std::uint8_t mode);
std::uint8_t ToRosMotionMode(masc_chassis_can_sdk::MotionMode mode);
std::uint8_t ToRosServoCalibrationState(masc_chassis_can_sdk::ServoCalibrationState state);
std::uint8_t ToRosChassisState(masc_chassis_can_sdk::SystemState state);

rosmsg::ChassisStatus BuildChassisStatus(
    const RosTime &stamp, const std::optional<masc_chassis_can_sdk::SystemFeedback> &system,
    const std::optional<masc_chassis_can_sdk::MotionModeFeedback> &mode);

rosmsg::BmsStatus BuildBmsStatus(
    const RosTime &stamp, bool timed_out,
    const std::optional<masc_chassis_can_sdk::BmsBasicFeedback> &basic,
    const std::optional<masc_chassis_can_sdk::BmsFaultFeedback> &fault);

rosmsg::DriveMotor BuildDriveMotor(const RosTime &stamp,
                                   const masc_chassis_can_sdk::PowerWheelFeedback &feedback);
rosmsg::DriveMotorStatus BuildDriveMotorStatus(
    const RosTime &stamp, const masc_chassis_can_sdk::PowerWheelDriverFeedback &feedback);
rosmsg::ServoMotor BuildServoMotor(const RosTime &stamp,
                                   const masc_chassis_can_sdk::ServoFeedback &feedback);
rosmsg::ServoMotorStatus BuildServoMotorStatus(
    const RosTime &stamp, const masc_chassis_can_sdk::ServoDriverFeedback &feedback);
rosmsg::Imu BuildImu(const RosTime &stamp, const std::string &frame_id,
                     const masc_chassis_can_sdk::ImuFeedback &feedback);
rosmsg::RcStatus BuildRcStatus(const RosTime &stamp,
                               const masc_chassis_can_sdk::RcFeedback &feedback);

}  // namespace masc_chassis_ros_driver
