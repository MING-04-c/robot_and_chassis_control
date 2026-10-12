/*
 * Copyright (C) 2026, MASC.
 * @Description: Standstill auto lock policy for the chassis ROS driver.
 */
#pragma once

#include <optional>

#include "masc_chassis_can_sdk/types.hpp"
#include "masc_chassis_ros_driver/ros_api.hpp"

namespace masc_chassis_ros_driver {

struct AutoLockConfig {
    bool m_enabled = true;
    int m_delay_ms = 3000;
    double m_cmd_filter_alpha = 0.2;
    double m_cmd_zero_linear_threshold = 0.01;
    double m_cmd_zero_angular_threshold = 0.01;
    double m_feedback_linear_stop_threshold = 0.02;
    double m_feedback_angular_stop_threshold = 0.02;
    bool m_auto_unlock_on_cmd = true;
};

enum class AutoLockAction {
    None,
    Lock,
    Unlock,
};

// 自动驻车每个命令周期的输入：限幅后的命令、最近反馈、当前模式和系统状态。
struct AutoLockUpdateInput {
    masc_chassis_can_sdk::MotionCommand m_command;
    std::optional<masc_chassis_can_sdk::MotionFeedback> m_motion_feedback;
    std::optional<masc_chassis_can_sdk::MotionModeFeedback> m_motion_mode_feedback;
    bool m_motion_mode_switch_supported = false;
    bool m_auto_lock_allowed = false;
    RosTime m_now;
};

// 自动驻车的输出。m_command 是本周期最终发送的运动命令，mode_request 是可选的一次性切换。
struct AutoLockUpdateResult {
    masc_chassis_can_sdk::MotionCommand m_command;
    std::optional<masc_chassis_can_sdk::MotionMode> m_motion_mode_request;
    AutoLockAction m_action = AutoLockAction::None;
};

class AutoLockController {
public:
    void Configure(const AutoLockConfig &config);
    void Reset();

    AutoLockUpdateResult Update(const AutoLockUpdateInput &input);
    void HandleMotionModeRequestResult(AutoLockAction action, bool sent);

private:
    void UpdateFilteredCommand(const masc_chassis_can_sdk::MotionCommand &command);
    bool IsFilteredCommandZero() const;
    bool IsMotionFeedbackStopped(const masc_chassis_can_sdk::MotionFeedback &feedback) const;
    bool ShouldRetryModeRequest(const RosTime &now) const;
    void SetModeRequest(AutoLockAction action, masc_chassis_can_sdk::MotionMode mode,
                        const RosTime &now, AutoLockUpdateResult *result);

    AutoLockConfig m_config;
    masc_chassis_can_sdk::MotionCommand m_filtered_command;
    bool m_filter_initialized = false;
    std::optional<RosTime> m_stop_since;
    std::optional<RosTime> m_last_mode_request_time;
    bool m_lock_requested = false;
    bool m_unlock_requested = false;
};

}  // namespace masc_chassis_ros_driver
