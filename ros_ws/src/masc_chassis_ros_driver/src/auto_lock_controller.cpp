/*
 * Copyright (C) 2026, MASC.
 * @Description: Standstill auto lock policy for the chassis ROS driver.
 */
#include "masc_chassis_ros_driver/auto_lock_controller.hpp"

#include <algorithm>
#include <cmath>

namespace masc_chassis_ros_driver {
namespace {

constexpr int kModeRetryMs = 1000;
constexpr double kMsPerSecond = 1000.0;
constexpr double kMinFilterAlpha = 0.01;

double LowPass(double previous, double input, double alpha) {
    return previous + alpha * (input - previous);
}

double SnapToZero(double value, double threshold) {
    return std::fabs(value) <= threshold ? 0.0 : value;
}

}  // namespace

void AutoLockController::Configure(const AutoLockConfig &config) {
    m_config = config;
    m_config.m_delay_ms = std::max(0, m_config.m_delay_ms);
    m_config.m_cmd_filter_alpha =
        std::clamp(m_config.m_cmd_filter_alpha, kMinFilterAlpha, 1.0);
    m_config.m_cmd_zero_linear_threshold =
        std::max(0.0, m_config.m_cmd_zero_linear_threshold);
    m_config.m_cmd_zero_angular_threshold =
        std::max(0.0, m_config.m_cmd_zero_angular_threshold);
    m_config.m_feedback_linear_stop_threshold =
        std::max(0.0, m_config.m_feedback_linear_stop_threshold);
    m_config.m_feedback_angular_stop_threshold =
        std::max(0.0, m_config.m_feedback_angular_stop_threshold);
    Reset();
}

void AutoLockController::Reset() {
    m_filtered_command = {};
    m_filter_initialized = false;
    m_stop_since.reset();
    m_last_mode_request_time.reset();
    m_lock_requested = false;
    m_unlock_requested = false;
}

AutoLockUpdateResult AutoLockController::Update(const AutoLockUpdateInput &input) {
    using MotionMode = masc_chassis_can_sdk::MotionMode;

    // 返回值包含两部分：本周期实际要发送的运动命令，以及可选的一次性模式切换请求。
    AutoLockUpdateResult result;
    result.m_command = input.m_command;

    if (!m_config.m_enabled) {
        Reset();
        return result;
    }

    // 低通只用于判断“用户是否已经停止发命令”，不会改变正常运动命令本身。
    UpdateFilteredCommand(input.m_command);
    const bool filtered_command_zero = IsFilteredCommandZero();

    // 只有支持 0x110/0x220 的底盘，并且已经收到模式反馈，自动锁止才有依据。
    if (!input.m_motion_mode_switch_supported || !input.m_motion_mode_feedback.has_value()) {
        m_stop_since.reset();
        m_last_mode_request_time.reset();
        m_lock_requested = false;
        m_unlock_requested = false;
        return result;
    }

    const auto &mode_feedback = *input.m_motion_mode_feedback;
    const bool switching = mode_feedback.m_motion_mode_switching;
    const bool is_default = mode_feedback.m_current_motion_mode == MotionMode::DefaultChassis;
    const bool is_park = mode_feedback.m_current_motion_mode == MotionMode::Park;

    // 解锁请求尚未被反馈确认时，持续发送零速度，防止模式切换过程中突然运动。
    if (m_unlock_requested) {
        if (is_default && !switching) {
            m_unlock_requested = false;
            m_lock_requested = false;
            m_last_mode_request_time.reset();
        } else {
            result.m_command = {};
            if (!filtered_command_zero && m_config.m_auto_unlock_on_cmd) {
                SetModeRequest(AutoLockAction::Unlock, MotionMode::DefaultChassis, input.m_now,
                               &result);
            }
            return result;
        }
    }

    if (switching) {
        m_stop_since.reset();
        if (m_lock_requested || m_unlock_requested) {
            result.m_command = {};
        }
        return result;
    }

    // 已经处于驻车模式时强制发送零命令；收到非零命令可自动请求切回默认模式。
    if (is_park) {
        m_stop_since.reset();
        m_lock_requested = true;
        result.m_command = {};
        if (!filtered_command_zero && m_config.m_auto_unlock_on_cmd) {
            if (!m_unlock_requested) {
                m_last_mode_request_time.reset();
            }
            m_unlock_requested = true;
            SetModeRequest(AutoLockAction::Unlock, MotionMode::DefaultChassis, input.m_now,
                           &result);
        }
        return result;
    }

    // 急停、故障、遥控、拖车等状态禁止自动驻车；只有 Park/CommControl 被允许。
    if (!input.m_auto_lock_allowed) {
        m_stop_since.reset();
        m_last_mode_request_time.reset();
        m_lock_requested = false;
        return result;
    }

    if (!is_default) {
        m_stop_since.reset();
        m_lock_requested = false;
        return result;
    }

    if (!filtered_command_zero) {
        m_stop_since.reset();
        if (m_lock_requested) {
            m_last_mode_request_time.reset();
        }
        m_lock_requested = false;
        return result;
    }

    // 自动锁止必须同时满足：命令归零、实际反馈速度也低于阈值。
    if (!input.m_motion_feedback.has_value() ||
        !IsMotionFeedbackStopped(*input.m_motion_feedback)) {
        m_stop_since.reset();
        return result;
    }

    if (!m_stop_since.has_value()) {
        m_stop_since = input.m_now;
        return result;
    }

    const double stopped_ms = SecondsBetween(*m_stop_since, input.m_now) * kMsPerSecond;
    if (stopped_ms < static_cast<double>(m_config.m_delay_ms)) {
        return result;
    }

    // 命令和反馈都持续停稳 delay_ms 后，请求切换到 Park。请求失败时按 1 秒间隔重试。
    result.m_command = {};
    SetModeRequest(AutoLockAction::Lock, MotionMode::Park, input.m_now, &result);
    return result;
}

void AutoLockController::HandleMotionModeRequestResult(AutoLockAction action, bool sent) {
    if (action == AutoLockAction::Lock && sent) {
        m_lock_requested = true;
    }
}

void AutoLockController::UpdateFilteredCommand(
    const masc_chassis_can_sdk::MotionCommand &command) {
    if (!m_filter_initialized) {
        m_filtered_command = command;
        m_filter_initialized = true;
    } else {
        m_filtered_command.m_target_linear_x_mps =
            LowPass(m_filtered_command.m_target_linear_x_mps, command.m_target_linear_x_mps,
                    m_config.m_cmd_filter_alpha);
        m_filtered_command.m_target_linear_y_mps =
            LowPass(m_filtered_command.m_target_linear_y_mps, command.m_target_linear_y_mps,
                    m_config.m_cmd_filter_alpha);
        m_filtered_command.m_target_angular_z_radps =
            LowPass(m_filtered_command.m_target_angular_z_radps,
                    command.m_target_angular_z_radps, m_config.m_cmd_filter_alpha);
        m_filtered_command.m_target_steering_angle_rad =
            LowPass(m_filtered_command.m_target_steering_angle_rad,
                    command.m_target_steering_angle_rad, m_config.m_cmd_filter_alpha);
    }

    m_filtered_command.m_target_linear_x_mps =
        SnapToZero(m_filtered_command.m_target_linear_x_mps,
                   m_config.m_cmd_zero_linear_threshold);
    m_filtered_command.m_target_linear_y_mps =
        SnapToZero(m_filtered_command.m_target_linear_y_mps,
                   m_config.m_cmd_zero_linear_threshold);
    m_filtered_command.m_target_angular_z_radps =
        SnapToZero(m_filtered_command.m_target_angular_z_radps,
                   m_config.m_cmd_zero_angular_threshold);
    m_filtered_command.m_target_steering_angle_rad =
        SnapToZero(m_filtered_command.m_target_steering_angle_rad,
                   m_config.m_cmd_zero_angular_threshold);
}

bool AutoLockController::IsFilteredCommandZero() const {
    return m_filtered_command.m_target_linear_x_mps == 0.0 &&
           m_filtered_command.m_target_linear_y_mps == 0.0 &&
           m_filtered_command.m_target_angular_z_radps == 0.0 &&
           m_filtered_command.m_target_steering_angle_rad == 0.0;
}

bool AutoLockController::IsMotionFeedbackStopped(
    const masc_chassis_can_sdk::MotionFeedback &feedback) const {
    return std::fabs(feedback.m_current_linear_x_mps) <=
               m_config.m_feedback_linear_stop_threshold &&
           std::fabs(feedback.m_current_linear_y_mps) <=
               m_config.m_feedback_linear_stop_threshold &&
           std::fabs(feedback.m_current_angular_z_radps) <=
               m_config.m_feedback_angular_stop_threshold;
}

bool AutoLockController::ShouldRetryModeRequest(const RosTime &now) const {
    if (!m_last_mode_request_time.has_value()) {
        return true;
    }
    return SecondsBetween(*m_last_mode_request_time, now) * kMsPerSecond >=
           static_cast<double>(kModeRetryMs);
}

void AutoLockController::SetModeRequest(AutoLockAction action,
                                        masc_chassis_can_sdk::MotionMode mode,
                                        const RosTime &now,
                                        AutoLockUpdateResult *result) {
    if (result == nullptr || !ShouldRetryModeRequest(now)) {
        return;
    }
    m_last_mode_request_time = now;
    result->m_motion_mode_request = mode;
    result->m_action = action;
}

}  // namespace masc_chassis_ros_driver
