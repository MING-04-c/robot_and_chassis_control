/*
 * Copyright (C) 2026, MASC.
 * @Description: Integrates chassis motion feedback into odometry.
 */
#include "masc_chassis_ros_driver/odom_integrator.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#if defined(MASC_CHASSIS_USE_ROS2)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

namespace masc_chassis_ros_driver {

void OdomIntegrator::Configure(const OdomIntegratorConfig &config) {
    m_config = config;
}

void OdomIntegrator::Reset() {
    m_position_x = 0.0;
    m_position_y = 0.0;
    m_theta = 0.0;
    m_has_last_stamp = false;
}

rosmsg::Odometry OdomIntegrator::Integrate(const masc_chassis_can_sdk::MotionFeedback &feedback,
                                           const RosTime &stamp) {
    // 底盘反馈给出的是车体坐标系速度。这里用上一帧时间间隔做一阶积分，
    // 得到 odom 坐标系中的 x/y/yaw；它是轮速/底盘反馈里程计，不是定位结果。
    const double linear_x = feedback.m_current_linear_x_mps * m_config.m_linear_scale;
    const double linear_y = feedback.m_current_linear_y_mps * m_config.m_linear_scale;
    const double angular_z = feedback.m_current_angular_z_radps * m_config.m_angular_scale;

    if (m_has_last_stamp) {
        const double dt = SecondsBetween(m_last_stamp, stamp);
        // 时间跳变或长时间断帧时不积分，避免一次异常 dt 把位姿直接积分飞掉。
        if (dt > 0.0 && dt < 1.0) {
            const double delta_x = (linear_x * std::cos(m_theta) - linear_y * std::sin(m_theta)) * dt;
            const double delta_y = (linear_x * std::sin(m_theta) + linear_y * std::cos(m_theta)) * dt;
            m_position_x += delta_x;
            m_position_y += delta_y;
            m_theta += angular_z * dt;
        }
    }
    m_last_stamp = stamp;
    m_has_last_stamp = true;

    // 速度同时作为 twist 输出，积分位姿作为 pose 输出。
    rosmsg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = m_config.m_odom_frame_id;
    odom.child_frame_id = m_config.m_base_frame_id;
    odom.pose.pose.position.x = m_position_x;
    odom.pose.pose.position.y = m_position_y;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0.0, 0.0, m_theta);
    odom.pose.pose.orientation = tf2::toMsg(odom_quat);

    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.linear.y = linear_y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = angular_z;
    SetOdomCovariance(&odom);
    return odom;
}

rosmsg::TransformStamped OdomIntegrator::BuildTransform(const rosmsg::Odometry &odom) const {
    rosmsg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = odom.child_frame_id;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom.pose.pose.orientation;
    return transform;
}

void OdomIntegrator::SetOdomCovariance(rosmsg::Odometry *odom) const {
    // 当前协议没有完整的协方差模型，使用“运动中较大、静止时较小”的保守值，
    // 对未提供的 z/roll/pitch 方向设置极大不确定性。
    const bool moving = std::fabs(odom->twist.twist.linear.x) > 1e-6 ||
                        std::fabs(odom->twist.twist.linear.y) > 1e-6 ||
                        std::fabs(odom->twist.twist.angular.z) > 1e-6;
    const double xy_cov = moving ? 1e-3 : 1e-9;
    const double yaw_cov = moving ? 1e-2 : 1e-9;

    odom->pose.covariance[0] = xy_cov;
    odom->pose.covariance[7] = xy_cov;
    odom->pose.covariance[14] = 1e6;
    odom->pose.covariance[21] = 1e6;
    odom->pose.covariance[28] = 1e6;
    odom->pose.covariance[35] = yaw_cov;

    odom->twist.covariance[0] = xy_cov;
    odom->twist.covariance[7] = xy_cov;
    odom->twist.covariance[14] = 1e6;
    odom->twist.covariance[21] = 1e6;
    odom->twist.covariance[28] = 1e6;
    odom->twist.covariance[35] = yaw_cov;
}

}  // namespace masc_chassis_ros_driver
