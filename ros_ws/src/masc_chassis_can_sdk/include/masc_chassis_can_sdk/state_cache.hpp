/*
 * Copyright (C) 2026, MASC.
 * @Version: V1.0
 * @Author: owen (keaa@keaa.net)
 * @Date: 2026-05-03 21:38:33
 * @LastEditTime: 2026-05-03 21:51:58
 * @LastEditors: owen (keaa@keaa.net)
 * @Description: Internal decoded state cache and health/statistics storage.
 */
#pragma once

#include <array>
#include <optional>
#include <unordered_map>

#include "masc_chassis_can_sdk/types.hpp"

namespace masc_chassis_can_sdk {

/// Internal decoded state cache used by ChassisSdk.
///
/// Public getters return copies from this cache. It is protected by ChassisSdk's mutex at runtime.
struct ChassisStateCache {
    std::optional<MotionFeedback> m_motion_feedback;
    std::optional<SystemFeedback> m_system_feedback;
    std::optional<VersionInfo> m_version_info;
    std::optional<BmsBasicFeedback> m_bms_basic_feedback;
    std::optional<BmsFaultFeedback> m_bms_fault_feedback;
    std::optional<MotionModeFeedback> m_motion_mode_feedback;
    std::optional<ImuFeedback> m_imu_feedback;
    std::optional<RcFeedback> m_rc_feedback;

    std::array<std::optional<PowerWheelFeedback>, kMaxIndexedDevices> m_power_wheels;
    std::array<std::optional<PowerWheelDriverFeedback>, kMaxIndexedDevices> m_power_wheel_drivers;
    std::array<std::optional<ServoFeedback>, kMaxIndexedDevices> m_servos;
    std::array<std::optional<ServoDriverFeedback>, kMaxIndexedDevices> m_servo_drivers;

    std::unordered_map<std::uint16_t, FrameStats> m_frame_stats;
    HealthSnapshot m_health;
};

}  // namespace masc_chassis_can_sdk
