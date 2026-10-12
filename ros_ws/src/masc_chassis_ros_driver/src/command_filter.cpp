/*
 * Copyright (C) 2026, MASC.
 * @Description: Velocity command timeout and limit handling.
 */
#include "masc_chassis_ros_driver/command_filter.hpp"

#include <algorithm>
#include <cmath>

namespace masc_chassis_ros_driver {

void CommandFilter::Configure(const CommandFilterConfig &config) {
    m_config = config;
}

masc_chassis_can_sdk::MotionCommand CommandFilter::BuildCommand(
    const rosmsg::Twist &twist, bool has_command, const RosTime &last_command_time,
    const RosTime &now) const {
    // 默认构造就是全零命令：没有收到命令，或命令已经超时，都进入安全停机状态。
    masc_chassis_can_sdk::MotionCommand command;
    if (!has_command ||
        SecondsBetween(last_command_time, now) * 1000.0 > static_cast<double>(m_config.m_timeout_ms)) {
        return command;
    }

    // 这里只做速度限幅和超时判断，不负责发送 CAN。发送由节点定时器和 SDK 完成。
    command.m_target_linear_x_mps = Clamp(twist.linear.x, m_config.m_max_linear_x);
    command.m_target_linear_y_mps = Clamp(twist.linear.y, m_config.m_max_linear_y);
    command.m_target_angular_z_radps = Clamp(twist.angular.z, m_config.m_max_angular_z);
    // 当前 ROS Twist 没有独立的转向角字段，因此转向角暂时固定为 0。
    command.m_target_steering_angle_rad = 0.0;
    return command;
}

double CommandFilter::Clamp(double value, double limit) {
    const double abs_limit = std::fabs(limit);
    return std::clamp(value, -abs_limit, abs_limit);
}

}  // namespace masc_chassis_ros_driver
