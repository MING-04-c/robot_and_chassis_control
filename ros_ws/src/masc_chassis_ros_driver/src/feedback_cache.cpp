/*
 * Copyright (C) 2026, MASC.
 * @Description: Thread-safe decoded feedback cache for the chassis ROS driver.
 */
#include "masc_chassis_ros_driver/feedback_cache.hpp"

namespace masc_chassis_ros_driver {

// SDK 的接收线程和 ROS 的发布定时器不是同一执行上下文，因此所有反馈先写入
// 这个线程安全缓存。发布端读取缓存副本，避免直接在 CAN 回调里执行 ROS 发布。

void FeedbackCache::SetMotionFeedback(const masc_chassis_can_sdk::MotionFeedback &feedback,
                                      const RosTime &stamp) {
    std::lock_guard<std::mutex> lock(m_mutex);
    // UpdateStampedCache 同时保存数据、ROS 接收时间和 sequence；sequence 用来避免重复发布。
    UpdateStampedCache(&m_motion_feedback, feedback, stamp);
}

void FeedbackCache::SetSystemFeedback(const masc_chassis_can_sdk::SystemFeedback &feedback,
                                      const RosTime &stamp) {
    std::lock_guard<std::mutex> lock(m_mutex);
    UpdateStampedCache(&m_system_feedback, feedback, stamp);
}

void FeedbackCache::SetMotionModeFeedback(
    const masc_chassis_can_sdk::MotionModeFeedback &feedback, const RosTime &stamp) {
    std::lock_guard<std::mutex> lock(m_mutex);
    UpdateStampedCache(&m_motion_mode_feedback, feedback, stamp);
}

void FeedbackCache::SetBmsBasicFeedback(const masc_chassis_can_sdk::BmsBasicFeedback &feedback,
                                        const RosTime &stamp) {
    std::lock_guard<std::mutex> lock(m_mutex);
    UpdateStampedCache(&m_bms_basic_feedback, feedback, stamp);
}

void FeedbackCache::SetBmsFaultFeedback(const masc_chassis_can_sdk::BmsFaultFeedback &feedback,
                                        const RosTime &stamp) {
    std::lock_guard<std::mutex> lock(m_mutex);
    UpdateStampedCache(&m_bms_fault_feedback, feedback, stamp);
}

void FeedbackCache::SetPowerWheelFeedback(
    const masc_chassis_can_sdk::PowerWheelFeedback &feedback, const RosTime &stamp) {
    if (feedback.m_index >= m_power_wheel_feedback.size()) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    UpdateStampedCache(&m_power_wheel_feedback[feedback.m_index], feedback, stamp);
    m_drive_motors_sequence++;
}

void FeedbackCache::SetPowerWheelDriverFeedback(
    const masc_chassis_can_sdk::PowerWheelDriverFeedback &feedback, const RosTime &stamp) {
    if (feedback.m_index >= m_power_wheel_driver_feedback.size()) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    UpdateStampedCache(&m_power_wheel_driver_feedback[feedback.m_index], feedback, stamp);
    m_drive_motor_status_sequence++;
}

void FeedbackCache::SetServoFeedback(const masc_chassis_can_sdk::ServoFeedback &feedback,
                                     const RosTime &stamp) {
    if (feedback.m_index >= m_servo_feedback.size()) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    UpdateStampedCache(&m_servo_feedback[feedback.m_index], feedback, stamp);
    m_servo_motors_sequence++;
}

void FeedbackCache::SetServoDriverFeedback(
    const masc_chassis_can_sdk::ServoDriverFeedback &feedback, const RosTime &stamp) {
    if (feedback.m_index >= m_servo_driver_feedback.size()) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    UpdateStampedCache(&m_servo_driver_feedback[feedback.m_index], feedback, stamp);
    m_servo_motor_status_sequence++;
}

void FeedbackCache::SetImuFeedback(const masc_chassis_can_sdk::ImuFeedback &feedback,
                                   const RosTime &stamp) {
    std::lock_guard<std::mutex> lock(m_mutex);
    UpdateStampedCache(&m_imu_feedback, feedback, stamp);
}

void FeedbackCache::SetRcFeedback(const masc_chassis_can_sdk::RcFeedback &feedback,
                                  const RosTime &stamp) {
    std::lock_guard<std::mutex> lock(m_mutex);
    UpdateStampedCache(&m_rc_feedback, feedback, stamp);
}

StampedCache<masc_chassis_can_sdk::MotionFeedback> FeedbackCache::GetMotionFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_motion_feedback;
}

StampedCache<masc_chassis_can_sdk::SystemFeedback> FeedbackCache::GetSystemFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_system_feedback;
}

StampedCache<masc_chassis_can_sdk::MotionModeFeedback>
FeedbackCache::GetMotionModeFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_motion_mode_feedback;
}

StampedCache<masc_chassis_can_sdk::BmsBasicFeedback> FeedbackCache::GetBmsBasicFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_bms_basic_feedback;
}

StampedCache<masc_chassis_can_sdk::BmsFaultFeedback> FeedbackCache::GetBmsFaultFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_bms_fault_feedback;
}

StampedCache<masc_chassis_can_sdk::ImuFeedback> FeedbackCache::GetImuFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_imu_feedback;
}

StampedCache<masc_chassis_can_sdk::RcFeedback> FeedbackCache::GetRcFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_rc_feedback;
}

IndexedFeedbackSnapshot<PowerWheelCache> FeedbackCache::GetPowerWheelFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return {m_power_wheel_feedback, m_drive_motors_sequence};
}

IndexedFeedbackSnapshot<PowerWheelDriverCache> FeedbackCache::GetPowerWheelDriverFeedback()
    const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return {m_power_wheel_driver_feedback, m_drive_motor_status_sequence};
}

IndexedFeedbackSnapshot<ServoCache> FeedbackCache::GetServoFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return {m_servo_feedback, m_servo_motors_sequence};
}

IndexedFeedbackSnapshot<ServoDriverCache> FeedbackCache::GetServoDriverFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return {m_servo_driver_feedback, m_servo_motor_status_sequence};
}

}  // namespace masc_chassis_ros_driver
