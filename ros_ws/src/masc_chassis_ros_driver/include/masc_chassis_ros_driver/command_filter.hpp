/*
 * Copyright (C) 2026, MASC.
 * @Description: Velocity command timeout and limit handling.
 */
#pragma once

#include "masc_chassis_can_sdk/types.hpp"
#include "masc_chassis_ros_driver/ros_api.hpp"
#include "masc_chassis_ros_driver/ros_msgs.hpp"

namespace masc_chassis_ros_driver {

struct CommandFilterConfig {
    double m_max_linear_x = 1.5;
    double m_max_linear_y = 1.0;
    double m_max_angular_z = 2.0;
    double m_max_steering_angle = 2.0;
    int m_timeout_ms = 100;
};

// cmd_vel 的安全边界层：检查消息时效，并对线速度/角速度做对称限幅。
// 它不是 CAN 帧过滤器，也不滤除反馈数据。
class CommandFilter {
public:
    void Configure(const CommandFilterConfig &config);

    masc_chassis_can_sdk::MotionCommand BuildCommand(const rosmsg::Twist &twist,
                                                     bool has_command,
                                                     const RosTime &last_command_time,
                                                     const RosTime &now) const;

private:
    static double Clamp(double value, double limit);

    CommandFilterConfig m_config;
};

}  // namespace masc_chassis_ros_driver
