/*
 * Copyright (C) 2026, MASC.
 * @Description: Thread-safe decoded feedback cache for the chassis ROS driver.
 */
#pragma once

#include <array>
#include <cstdint>
#include <mutex>

#include "masc_chassis_can_sdk/types.hpp"
#include "masc_chassis_ros_driver/ros_api.hpp"

namespace masc_chassis_ros_driver {

// 每条反馈都保存“数据 + 接收时间 + 是否有效 + 更新次数”。
// ROS 定时器用时间判断超时，用 sequence 判断是否有新数据。

template <typename T>
struct StampedCache {
    T m_data;
    RosTime m_stamp;
    bool m_valid = false;
    std::uint64_t m_sequence = 0;
};

using PowerWheelCache =
    std::array<StampedCache<masc_chassis_can_sdk::PowerWheelFeedback>,
               masc_chassis_can_sdk::kMaxIndexedDevices>;
using PowerWheelDriverCache =
    std::array<StampedCache<masc_chassis_can_sdk::PowerWheelDriverFeedback>,
               masc_chassis_can_sdk::kMaxIndexedDevices>;
using ServoCache = std::array<StampedCache<masc_chassis_can_sdk::ServoFeedback>,
                              masc_chassis_can_sdk::kMaxIndexedDevices>;
using ServoDriverCache =
    std::array<StampedCache<masc_chassis_can_sdk::ServoDriverFeedback>,
               masc_chassis_can_sdk::kMaxIndexedDevices>;

template <typename CacheT>
struct IndexedFeedbackSnapshot {
    CacheT m_cache;
    std::uint64_t m_sequence = 0;
};

class FeedbackCache {
public:
    void SetMotionFeedback(const masc_chassis_can_sdk::MotionFeedback &feedback,
                           const RosTime &stamp);
    void SetSystemFeedback(const masc_chassis_can_sdk::SystemFeedback &feedback,
                           const RosTime &stamp);
    void SetMotionModeFeedback(const masc_chassis_can_sdk::MotionModeFeedback &feedback,
                               const RosTime &stamp);
    void SetBmsBasicFeedback(const masc_chassis_can_sdk::BmsBasicFeedback &feedback,
                             const RosTime &stamp);
    void SetBmsFaultFeedback(const masc_chassis_can_sdk::BmsFaultFeedback &feedback,
                             const RosTime &stamp);
    void SetPowerWheelFeedback(const masc_chassis_can_sdk::PowerWheelFeedback &feedback,
                               const RosTime &stamp);
    void SetPowerWheelDriverFeedback(
        const masc_chassis_can_sdk::PowerWheelDriverFeedback &feedback, const RosTime &stamp);
    void SetServoFeedback(const masc_chassis_can_sdk::ServoFeedback &feedback,
                          const RosTime &stamp);
    void SetServoDriverFeedback(const masc_chassis_can_sdk::ServoDriverFeedback &feedback,
                                const RosTime &stamp);
    void SetImuFeedback(const masc_chassis_can_sdk::ImuFeedback &feedback, const RosTime &stamp);
    void SetRcFeedback(const masc_chassis_can_sdk::RcFeedback &feedback, const RosTime &stamp);

    StampedCache<masc_chassis_can_sdk::MotionFeedback> GetMotionFeedback() const;
    StampedCache<masc_chassis_can_sdk::SystemFeedback> GetSystemFeedback() const;
    StampedCache<masc_chassis_can_sdk::MotionModeFeedback> GetMotionModeFeedback() const;
    StampedCache<masc_chassis_can_sdk::BmsBasicFeedback> GetBmsBasicFeedback() const;
    StampedCache<masc_chassis_can_sdk::BmsFaultFeedback> GetBmsFaultFeedback() const;
    StampedCache<masc_chassis_can_sdk::ImuFeedback> GetImuFeedback() const;
    StampedCache<masc_chassis_can_sdk::RcFeedback> GetRcFeedback() const;

    IndexedFeedbackSnapshot<PowerWheelCache> GetPowerWheelFeedback() const;
    IndexedFeedbackSnapshot<PowerWheelDriverCache> GetPowerWheelDriverFeedback() const;
    IndexedFeedbackSnapshot<ServoCache> GetServoFeedback() const;
    IndexedFeedbackSnapshot<ServoDriverCache> GetServoDriverFeedback() const;

private:
    template <typename T>
    static void UpdateStampedCache(StampedCache<T> *cache, const T &feedback,
                                   const RosTime &stamp) {
        cache->m_data = feedback;
        cache->m_stamp = stamp;
        cache->m_valid = true;
        cache->m_sequence++;
    }

    mutable std::mutex m_mutex;
    StampedCache<masc_chassis_can_sdk::MotionFeedback> m_motion_feedback;
    StampedCache<masc_chassis_can_sdk::SystemFeedback> m_system_feedback;
    StampedCache<masc_chassis_can_sdk::MotionModeFeedback> m_motion_mode_feedback;
    StampedCache<masc_chassis_can_sdk::BmsBasicFeedback> m_bms_basic_feedback;
    StampedCache<masc_chassis_can_sdk::BmsFaultFeedback> m_bms_fault_feedback;
    StampedCache<masc_chassis_can_sdk::ImuFeedback> m_imu_feedback;
    StampedCache<masc_chassis_can_sdk::RcFeedback> m_rc_feedback;
    PowerWheelCache m_power_wheel_feedback;
    PowerWheelDriverCache m_power_wheel_driver_feedback;
    ServoCache m_servo_feedback;
    ServoDriverCache m_servo_driver_feedback;
    std::uint64_t m_drive_motors_sequence = 0;
    std::uint64_t m_drive_motor_status_sequence = 0;
    std::uint64_t m_servo_motors_sequence = 0;
    std::uint64_t m_servo_motor_status_sequence = 0;
};

}  // namespace masc_chassis_ros_driver
