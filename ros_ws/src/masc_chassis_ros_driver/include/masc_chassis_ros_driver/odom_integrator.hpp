/*
 * Copyright (C) 2026, MASC.
 * @Description: Integrates chassis motion feedback into odometry.
 */
#pragma once

#include <string>

#include "masc_chassis_can_sdk/types.hpp"
#include "masc_chassis_ros_driver/ros_api.hpp"
#include "masc_chassis_ros_driver/ros_msgs.hpp"

namespace masc_chassis_ros_driver {

struct OdomIntegratorConfig {
    std::string m_odom_frame_id = "odom";
    std::string m_base_frame_id = "base_link";
    double m_linear_scale = 1.0;
    double m_angular_scale = 1.0;
};

// 将 CAN MotionFeedback 的车体速度积分为 nav_msgs/Odometry，并可生成 odom->base_link TF。
class OdomIntegrator {
public:
    void Configure(const OdomIntegratorConfig &config);
    void Reset();

    rosmsg::Odometry Integrate(const masc_chassis_can_sdk::MotionFeedback &feedback,
                               const RosTime &stamp);
    rosmsg::TransformStamped BuildTransform(const rosmsg::Odometry &odom) const;

private:
    void SetOdomCovariance(rosmsg::Odometry *odom) const;

    OdomIntegratorConfig m_config;
    double m_position_x = 0.0;
    double m_position_y = 0.0;
    double m_theta = 0.0;
    RosTime m_last_stamp;
    bool m_has_last_stamp = false;
};

}  // namespace masc_chassis_ros_driver
