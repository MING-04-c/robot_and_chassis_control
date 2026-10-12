/*
 * Copyright (C) 2026, MASC.
 * @Version: V1.0
 * @Author: owen (keaa@keaa.net)
 * @Date: 2026-05-03 21:45:24
 * @LastEditTime: 2026-05-03 21:52:23
 * @LastEditors: owen (keaa@keaa.net)
 * @Description: ChassisSdk connection, receive dispatch, cache, callback, and watchdog logic.
 */
#include "masc_chassis_can_sdk/chassis_sdk.hpp"

#include <chrono>
#include <utility>

#include "masc_chassis_can_sdk/chassis_protocol.hpp"
#if defined(__linux__)
#include "masc_chassis_can_sdk/socketcan_transport.hpp"
#else
#include "masc_chassis_can_sdk/unsupported_can_transport.hpp"
#endif

namespace masc_chassis_can_sdk {
namespace {

constexpr int kReceiveTimeoutMs = 20;
constexpr double kFrameFrequencyLowPassAlpha = 0.2;

std::unique_ptr<ICanTransport> MakeDefaultTransport() {
#if defined(__linux__)
    return std::make_unique<SocketCanTransport>();
#else
    return std::make_unique<UnsupportedCanTransport>();
#endif
}

bool IsValidIndex(std::size_t index) {
    return index < kMaxIndexedDevices;
}

}  // namespace

ChassisSdk::ChassisSdk() : ChassisSdk(MakeDefaultTransport()) {}

ChassisSdk::ChassisSdk(std::unique_ptr<ICanTransport> transport)
    : m_transport(std::move(transport)),
      m_last_motion_command_update_time(std::chrono::steady_clock::now()) {}

ChassisSdk::~ChassisSdk() {
    Disconnect();
}

bool ChassisSdk::Configure(const CanConfig &config) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_cache.m_health.m_socket_open) {
        return false;
    }
    m_config = config;
    return true;
}

bool ChassisSdk::Connect() {
    Disconnect();

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cache.m_health.m_connection_attempt_count++;
        m_cache.m_health.m_connected = false;
        m_cache.m_health.m_protocol_version_matched = false;
        m_cache.m_health.m_socket_open = false;
        m_cache.m_version_info.reset();
        m_cache.m_health.m_version_info.reset();
        m_stop_requested = false;
    }

    if (!m_transport || !m_transport->Open(m_config)) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cache.m_health.m_transport_error_count++;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cache.m_health.m_socket_open = true;
        m_receive_thread_running = true;
    }
    m_receive_thread = std::thread(&ChassisSdk::ReceiveLoop, this);

    const auto timeout = std::chrono::milliseconds(m_config.m_connect_timeout_ms);
    std::unique_lock<std::mutex> lock(m_mutex);
    const bool received_version = m_version_cv.wait_for(lock, timeout, [this]() {
        return m_stop_requested || m_cache.m_version_info.has_value();
    });

    if (!received_version || m_stop_requested || !m_cache.m_version_info.has_value()) {
        lock.unlock();
        Disconnect();
        return false;
    }

    const bool version_matched =
        m_cache.m_version_info->m_protocol_version == m_config.m_expected_protocol_version;
    m_cache.m_health.m_protocol_version_matched = version_matched;
    m_cache.m_health.m_connected = version_matched;

    if (!version_matched) {
        lock.unlock();
        Disconnect();
        return false;
    }

    m_last_motion_command_update_time = std::chrono::steady_clock::now();
    m_motion_thread_running = true;
    lock.unlock();

    m_motion_thread = std::thread(&ChassisSdk::MotionCommandLoop, this);
    NotifyHealth();
    return true;
}

void ChassisSdk::Disconnect() {
    StopThreads();
    if (m_transport) {
        m_transport->Close();
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cache.m_health.m_socket_open = false;
        m_cache.m_health.m_connected = false;
        m_cache.m_health.m_protocol_version_matched = false;
    }
    NotifyHealth();
}

bool ChassisSdk::IsConnected() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_cache.m_health.m_connected;
}

bool ChassisSdk::SetMotionCommand(const MotionCommand &command) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_latest_motion_command = command;
    m_last_motion_command_update_time = std::chrono::steady_clock::now();
    m_cache.m_health.m_motion_command_timeout_active = false;
    return true;
}

bool ChassisSdk::SendMotionCommandOnce(const MotionCommand &command) {
    return WriteCanFrame(chassis_protocol::PackMotionCommand(command));
}

bool ChassisSdk::SendTowMode(bool enable) {
    return WriteCanFrame(chassis_protocol::PackTowMode(enable));
}

bool ChassisSdk::SendMotionMode(MotionMode mode) {
    if (!IsMotionModeSwitchSupported()) {
        return false;
    }
    return WriteCanFrame(chassis_protocol::PackMotionMode(mode));
}

bool ChassisSdk::SendFaultClear(std::uint16_t fault_id) {
    return WriteCanFrame(chassis_protocol::PackFaultClear(fault_id));
}

bool ChassisSdk::SendCalibrateServo(std::uint16_t servo_id) {
    return WriteCanFrame(chassis_protocol::PackCalibrateServo(servo_id));
}

bool ChassisSdk::SendRawFrame(const CanFrame &frame) {
    if (!IsStandardCanId(frame.m_id) || !HasValidClassicCanDlc(frame)) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cache.m_health.m_tx_error_count++;
        return false;
    }
    return WriteCanFrame(frame);
}

std::optional<VersionInfo> ChassisSdk::GetVersionInfo() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_cache.m_version_info;
}

bool ChassisSdk::IsMotionModeSwitchSupported() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_cache.m_version_info.has_value()) {
        return false;
    }
    return IsMotionModeSwitchSupportedChassisType(
        m_cache.m_version_info->m_chassis_type_version);
}

std::optional<MotionFeedback> ChassisSdk::GetMotionFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_cache.m_motion_feedback;
}

std::optional<SystemFeedback> ChassisSdk::GetSystemFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_cache.m_system_feedback;
}

std::optional<BmsBasicFeedback> ChassisSdk::GetBmsBasicFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_cache.m_bms_basic_feedback;
}

std::optional<BmsFaultFeedback> ChassisSdk::GetBmsFaultFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_cache.m_bms_fault_feedback;
}

std::optional<MotionModeFeedback> ChassisSdk::GetMotionModeFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_cache.m_motion_mode_feedback;
}

std::optional<PowerWheelFeedback> ChassisSdk::GetPowerWheelFeedback(std::size_t index) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!IsValidIndex(index)) {
        return std::nullopt;
    }
    return m_cache.m_power_wheels[index];
}

std::optional<PowerWheelDriverFeedback> ChassisSdk::GetPowerWheelDriverFeedback(
    std::size_t index) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!IsValidIndex(index)) {
        return std::nullopt;
    }
    return m_cache.m_power_wheel_drivers[index];
}

std::optional<ServoFeedback> ChassisSdk::GetServoFeedback(std::size_t index) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!IsValidIndex(index)) {
        return std::nullopt;
    }
    return m_cache.m_servos[index];
}

std::optional<ServoDriverFeedback> ChassisSdk::GetServoDriverFeedback(std::size_t index) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!IsValidIndex(index)) {
        return std::nullopt;
    }
    return m_cache.m_servo_drivers[index];
}

std::optional<ImuFeedback> ChassisSdk::GetImuFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_cache.m_imu_feedback;
}

std::optional<RcFeedback> ChassisSdk::GetRcFeedback() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_cache.m_rc_feedback;
}

HealthSnapshot ChassisSdk::GetHealthSnapshot() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_cache.m_health;
}

std::optional<FrameStats> ChassisSdk::GetFrameStats(std::uint16_t can_id) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto iter = m_cache.m_frame_stats.find(can_id);
    if (iter == m_cache.m_frame_stats.end()) {
        return std::nullopt;
    }
    return iter->second;
}

void ChassisSdk::SetMotionFeedbackCallback(MotionFeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_motion_feedback_callback = std::move(callback);
}

void ChassisSdk::SetSystemFeedbackCallback(SystemFeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_system_feedback_callback = std::move(callback);
}

void ChassisSdk::SetBmsBasicFeedbackCallback(BmsBasicFeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_bms_basic_feedback_callback = std::move(callback);
}

void ChassisSdk::SetBmsFaultFeedbackCallback(BmsFaultFeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_bms_fault_feedback_callback = std::move(callback);
}

void ChassisSdk::SetMotionModeFeedbackCallback(MotionModeFeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_motion_mode_feedback_callback = std::move(callback);
}

void ChassisSdk::SetPowerWheelFeedbackCallback(PowerWheelFeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_power_wheel_feedback_callback = std::move(callback);
}

void ChassisSdk::SetPowerWheelDriverFeedbackCallback(PowerWheelDriverFeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_power_wheel_driver_feedback_callback = std::move(callback);
}

void ChassisSdk::SetServoFeedbackCallback(ServoFeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_servo_feedback_callback = std::move(callback);
}

void ChassisSdk::SetServoDriverFeedbackCallback(ServoDriverFeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_servo_driver_feedback_callback = std::move(callback);
}

void ChassisSdk::SetImuFeedbackCallback(ImuFeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_imu_feedback_callback = std::move(callback);
}

void ChassisSdk::SetRcFeedbackCallback(RcFeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rc_feedback_callback = std::move(callback);
}

void ChassisSdk::SetRawFrameCallback(RawFrameCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_raw_frame_callback = std::move(callback);
}

void ChassisSdk::SetHealthCallback(HealthCallback callback) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_health_callback = std::move(callback);
}

void ChassisSdk::ReceiveLoop() {
    while (true) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_stop_requested) {
                break;
            }
        }

        CanFrame frame;
        const auto status = m_transport->ReadFrame(&frame, kReceiveTimeoutMs);
        if (status == CanReadStatus::Timeout) {
            continue;
        }
        if (status == CanReadStatus::Error) {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (!m_stop_requested) {
                m_cache.m_health.m_transport_error_count++;
                m_cache.m_health.m_rx_error_count++;
            }
            continue;
        }

        DispatchFrame(frame);
    }
}

void ChassisSdk::MotionCommandLoop() {
    while (true) {
        MotionCommand command;
        bool should_stop = false;

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            should_stop = m_stop_requested || !m_cache.m_health.m_connected;
            const auto now = std::chrono::steady_clock::now();
            const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    now - m_last_motion_command_update_time)
                                    .count();
            if (age_ms > m_config.m_motion_command_timeout_ms) {
                command = MotionCommand{};
                m_cache.m_health.m_motion_command_timeout_active = true;
            } else {
                command = m_latest_motion_command;
            }
        }

        if (should_stop) {
            break;
        }

        WriteCanFrame(chassis_protocol::PackMotionCommand(command));
        std::this_thread::sleep_for(std::chrono::milliseconds(m_config.m_motion_command_period_ms));
    }
}

void ChassisSdk::DispatchFrame(const CanFrame &frame) {
    if (!IsStandardCanId(frame.m_id) || !chassis_protocol::HasExpectedDlc(frame) ||
        !chassis_protocol::IsFeedbackCanId(frame.m_id)) {
        RawFrameCallback raw_callback;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            IncrementDropLocked(frame.m_id);
            raw_callback = m_raw_frame_callback;
        }
        NotifyRawFrame(frame, raw_callback);
        return;
    }

    if (auto feedback = chassis_protocol::ParseMotionFeedback(frame); feedback.has_value()) {
        MotionFeedbackCallback callback;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_cache.m_motion_feedback = feedback;
            UpdateFrameStatsLocked(frame.m_id, true);
            callback = m_motion_feedback_callback;
        }
        if (callback) {
            callback(*feedback);
        }
        return;
    }

    if (auto feedback = chassis_protocol::ParseSystemFeedback(frame); feedback.has_value()) {
        SystemFeedbackCallback callback;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_cache.m_system_feedback = feedback;
            UpdateFrameStatsLocked(frame.m_id, true);
            callback = m_system_feedback_callback;
        }
        if (callback) {
            callback(*feedback);
        }
        return;
    }

    if (auto version = chassis_protocol::ParseVersionInfo(frame); version.has_value()) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_cache.m_version_info = version;
            m_cache.m_health.m_version_info = version;
            m_cache.m_health.m_protocol_version_matched =
                version->m_protocol_version == m_config.m_expected_protocol_version;
            UpdateFrameStatsLocked(frame.m_id, true);
        }
        m_version_cv.notify_all();
        return;
    }

    if (auto feedback = chassis_protocol::ParseBmsBasicFeedback(frame); feedback.has_value()) {
        BmsBasicFeedbackCallback callback;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_cache.m_bms_basic_feedback = feedback;
            UpdateFrameStatsLocked(frame.m_id, true);
            callback = m_bms_basic_feedback_callback;
        }
        if (callback) {
            callback(*feedback);
        }
        return;
    }

    if (auto feedback = chassis_protocol::ParseBmsFaultFeedback(frame); feedback.has_value()) {
        BmsFaultFeedbackCallback callback;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_cache.m_bms_fault_feedback = feedback;
            UpdateFrameStatsLocked(frame.m_id, true);
            callback = m_bms_fault_feedback_callback;
        }
        if (callback) {
            callback(*feedback);
        }
        return;
    }

    if (auto feedback = chassis_protocol::ParseMotionModeFeedback(frame); feedback.has_value()) {
        MotionModeFeedbackCallback callback;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_cache.m_motion_mode_feedback = feedback;
            UpdateFrameStatsLocked(frame.m_id, true);
            callback = m_motion_mode_feedback_callback;
        }
        if (callback) {
            callback(*feedback);
        }
        return;
    }

    if (auto feedback = chassis_protocol::ParsePowerWheelFeedback(frame); feedback.has_value()) {
        PowerWheelFeedbackCallback callback;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (IsValidIndex(feedback->m_index)) {
                m_cache.m_power_wheels[feedback->m_index] = feedback;
            }
            UpdateFrameStatsLocked(frame.m_id, true);
            callback = m_power_wheel_feedback_callback;
        }
        if (callback) {
            callback(*feedback);
        }
        return;
    }

    if (auto feedback = chassis_protocol::ParsePowerWheelDriverFeedback(frame);
        feedback.has_value()) {
        PowerWheelDriverFeedbackCallback callback;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (IsValidIndex(feedback->m_index)) {
                m_cache.m_power_wheel_drivers[feedback->m_index] = feedback;
            }
            UpdateFrameStatsLocked(frame.m_id, true);
            callback = m_power_wheel_driver_feedback_callback;
        }
        if (callback) {
            callback(*feedback);
        }
        return;
    }

    if (auto feedback = chassis_protocol::ParseServoFeedback(frame); feedback.has_value()) {
        ServoFeedbackCallback callback;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (IsValidIndex(feedback->m_index)) {
                m_cache.m_servos[feedback->m_index] = feedback;
            }
            UpdateFrameStatsLocked(frame.m_id, true);
            callback = m_servo_feedback_callback;
        }
        if (callback) {
            callback(*feedback);
        }
        return;
    }

    if (auto feedback = chassis_protocol::ParseServoDriverFeedback(frame); feedback.has_value()) {
        ServoDriverFeedbackCallback callback;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (IsValidIndex(feedback->m_index)) {
                m_cache.m_servo_drivers[feedback->m_index] = feedback;
            }
            UpdateFrameStatsLocked(frame.m_id, true);
            callback = m_servo_driver_feedback_callback;
        }
        if (callback) {
            callback(*feedback);
        }
        return;
    }

    if (auto feedback = chassis_protocol::ParseImuFeedback(frame); feedback.has_value()) {
        ImuFeedbackCallback callback;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_cache.m_imu_feedback = feedback;
            UpdateFrameStatsLocked(frame.m_id, true);
            callback = m_imu_feedback_callback;
        }
        if (callback) {
            callback(*feedback);
        }
        return;
    }

    if (auto feedback = chassis_protocol::ParseRcFeedback(frame); feedback.has_value()) {
        RcFeedbackCallback callback;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_cache.m_rc_feedback = feedback;
            UpdateFrameStatsLocked(frame.m_id, true);
            callback = m_rc_feedback_callback;
        }
        if (callback) {
            callback(*feedback);
        }
        return;
    }

    RawFrameCallback raw_callback;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        UpdateFrameStatsLocked(frame.m_id, false);
        raw_callback = m_raw_frame_callback;
    }
    NotifyRawFrame(frame, raw_callback);
}

bool ChassisSdk::WriteCanFrame(const CanFrame &frame) {
    if (!m_transport || !m_transport->IsOpen()) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cache.m_health.m_tx_error_count++;
        m_cache.m_health.m_transport_error_count++;
        return false;
    }

    if (!m_transport->WriteFrame(frame)) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_cache.m_health.m_tx_error_count++;
        m_cache.m_health.m_transport_error_count++;
        return false;
    }
    return true;
}

void ChassisSdk::NotifyRawFrame(const CanFrame &frame, RawFrameCallback callback) {
    if (callback) {
        callback(frame);
    }
}

void ChassisSdk::UpdateFrameStatsLocked(std::uint16_t can_id, bool parse_success) {
    auto &stats = m_cache.m_frame_stats[can_id];
    stats.m_can_id = can_id;
    const auto now = std::chrono::steady_clock::now();
    if (parse_success) {
        if (stats.m_last_rx_time.has_value()) {
            const auto delta = std::chrono::duration<double>(now - *stats.m_last_rx_time).count();
            if (delta > 0.0) {
                const auto instant_frequency_hz = 1.0 / delta;
                if (stats.m_frequency_hz <= 0.0) {
                    stats.m_frequency_hz = instant_frequency_hz;
                } else {
                    stats.m_frequency_hz =
                        (1.0 - kFrameFrequencyLowPassAlpha) * stats.m_frequency_hz +
                        kFrameFrequencyLowPassAlpha * instant_frequency_hz;
                }
            }
        }
        stats.m_last_rx_time = now;
        stats.m_rx_count++;
    } else {
        stats.m_parse_error_count++;
    }
}

void ChassisSdk::IncrementDropLocked(std::uint16_t can_id) {
    auto &stats = m_cache.m_frame_stats[can_id];
    stats.m_can_id = can_id;
    stats.m_drop_count++;
}

void ChassisSdk::NotifyHealth() {
    HealthCallback callback;
    HealthSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        callback = m_health_callback;
        snapshot = m_cache.m_health;
    }
    if (callback) {
        callback(snapshot);
    }
}

void ChassisSdk::StopThreads() {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stop_requested = true;
    }
    m_version_cv.notify_all();

    if (m_receive_thread.joinable()) {
        m_receive_thread.join();
    }
    if (m_motion_thread.joinable()) {
        m_motion_thread.join();
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_receive_thread_running = false;
    m_motion_thread_running = false;
    m_stop_requested = false;
}

}  // namespace masc_chassis_can_sdk
