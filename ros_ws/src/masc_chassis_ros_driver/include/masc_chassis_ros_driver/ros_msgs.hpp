/*
 * Copyright (C) 2026, MASC.
 * @Description: ROS1/ROS2 message and service aliases for the chassis ROS driver.
 */
#pragma once

// 将 ROS1/ROS2 的消息类型起统一别名，例如 rosmsg::Twist、rosmsg::Odometry。
// 这样核心驱动无需在每个函数里写两套消息类型名称。
#if defined(MASC_CHASSIS_USE_ROS2)

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "masc_chassis_msgs/msg/bms_status.hpp"
#include "masc_chassis_msgs/msg/can_frame_stats.hpp"
#include "masc_chassis_msgs/msg/can_frame_stats_array.hpp"
#include "masc_chassis_msgs/msg/chassis_status.hpp"
#include "masc_chassis_msgs/msg/comm_status.hpp"
#include "masc_chassis_msgs/msg/drive_motor.hpp"
#include "masc_chassis_msgs/msg/drive_motor_array.hpp"
#include "masc_chassis_msgs/msg/drive_motor_status.hpp"
#include "masc_chassis_msgs/msg/drive_motor_status_array.hpp"
#include "masc_chassis_msgs/msg/rc_status.hpp"
#include "masc_chassis_msgs/msg/servo_motor.hpp"
#include "masc_chassis_msgs/msg/servo_motor_array.hpp"
#include "masc_chassis_msgs/msg/servo_motor_status.hpp"
#include "masc_chassis_msgs/msg/servo_motor_status_array.hpp"
#include "masc_chassis_msgs/srv/calibrate_servo.hpp"
#include "masc_chassis_msgs/srv/clear_fault.hpp"
#include "masc_chassis_msgs/srv/set_motion_mode.hpp"
#include "masc_chassis_msgs/srv/set_tow_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace masc_chassis_ros_driver::rosmsg {
using Header = std_msgs::msg::Header;
using Twist = geometry_msgs::msg::Twist;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Odometry = nav_msgs::msg::Odometry;
using BmsStatus = masc_chassis_msgs::msg::BmsStatus;
using CanFrameStats = masc_chassis_msgs::msg::CanFrameStats;
using CanFrameStatsArray = masc_chassis_msgs::msg::CanFrameStatsArray;
using ChassisStatus = masc_chassis_msgs::msg::ChassisStatus;
using CommStatus = masc_chassis_msgs::msg::CommStatus;
using DriveMotor = masc_chassis_msgs::msg::DriveMotor;
using DriveMotorArray = masc_chassis_msgs::msg::DriveMotorArray;
using DriveMotorStatus = masc_chassis_msgs::msg::DriveMotorStatus;
using DriveMotorStatusArray = masc_chassis_msgs::msg::DriveMotorStatusArray;
using Imu = sensor_msgs::msg::Imu;
using RcStatus = masc_chassis_msgs::msg::RcStatus;
using ServoMotor = masc_chassis_msgs::msg::ServoMotor;
using ServoMotorArray = masc_chassis_msgs::msg::ServoMotorArray;
using ServoMotorStatus = masc_chassis_msgs::msg::ServoMotorStatus;
using ServoMotorStatusArray = masc_chassis_msgs::msg::ServoMotorStatusArray;
using SetMotionMode = masc_chassis_msgs::srv::SetMotionMode;
using SetTowMode = masc_chassis_msgs::srv::SetTowMode;
using ClearFault = masc_chassis_msgs::srv::ClearFault;
using CalibrateServo = masc_chassis_msgs::srv::CalibrateServo;
using Trigger = std_srvs::srv::Trigger;
}  // namespace masc_chassis_ros_driver::rosmsg

#elif defined(MASC_CHASSIS_USE_ROS1)

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <masc_chassis_msgs/BmsStatus.h>
#include <masc_chassis_msgs/CalibrateServo.h>
#include <masc_chassis_msgs/CanFrameStats.h>
#include <masc_chassis_msgs/CanFrameStatsArray.h>
#include <masc_chassis_msgs/ChassisStatus.h>
#include <masc_chassis_msgs/ClearFault.h>
#include <masc_chassis_msgs/CommStatus.h>
#include <masc_chassis_msgs/DriveMotor.h>
#include <masc_chassis_msgs/DriveMotorArray.h>
#include <masc_chassis_msgs/DriveMotorStatus.h>
#include <masc_chassis_msgs/DriveMotorStatusArray.h>
#include <masc_chassis_msgs/RcStatus.h>
#include <masc_chassis_msgs/ServoMotor.h>
#include <masc_chassis_msgs/ServoMotorArray.h>
#include <masc_chassis_msgs/ServoMotorStatus.h>
#include <masc_chassis_msgs/ServoMotorStatusArray.h>
#include <masc_chassis_msgs/SetMotionMode.h>
#include <masc_chassis_msgs/SetTowMode.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <std_srvs/Trigger.h>

namespace masc_chassis_ros_driver::rosmsg {
using Header = std_msgs::Header;
using Twist = geometry_msgs::Twist;
using TwistStamped = geometry_msgs::TwistStamped;
using TransformStamped = geometry_msgs::TransformStamped;
using Odometry = nav_msgs::Odometry;
using BmsStatus = masc_chassis_msgs::BmsStatus;
using CanFrameStats = masc_chassis_msgs::CanFrameStats;
using CanFrameStatsArray = masc_chassis_msgs::CanFrameStatsArray;
using ChassisStatus = masc_chassis_msgs::ChassisStatus;
using CommStatus = masc_chassis_msgs::CommStatus;
using DriveMotor = masc_chassis_msgs::DriveMotor;
using DriveMotorArray = masc_chassis_msgs::DriveMotorArray;
using DriveMotorStatus = masc_chassis_msgs::DriveMotorStatus;
using DriveMotorStatusArray = masc_chassis_msgs::DriveMotorStatusArray;
using Imu = sensor_msgs::Imu;
using RcStatus = masc_chassis_msgs::RcStatus;
using ServoMotor = masc_chassis_msgs::ServoMotor;
using ServoMotorArray = masc_chassis_msgs::ServoMotorArray;
using ServoMotorStatus = masc_chassis_msgs::ServoMotorStatus;
using ServoMotorStatusArray = masc_chassis_msgs::ServoMotorStatusArray;
using SetMotionMode = masc_chassis_msgs::SetMotionMode;
using SetTowMode = masc_chassis_msgs::SetTowMode;
using ClearFault = masc_chassis_msgs::ClearFault;
using CalibrateServo = masc_chassis_msgs::CalibrateServo;
using Trigger = std_srvs::Trigger;
}  // namespace masc_chassis_ros_driver::rosmsg

#else
#error "Must define MASC_CHASSIS_USE_ROS2 or MASC_CHASSIS_USE_ROS1"
#endif
