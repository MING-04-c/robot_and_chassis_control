/*
 * Copyright (C) 2026, MASC.
 * @Version: V1.0
 * @Author: owen (keaa@keaa.net)
 * @Date: 2026-05-03 21:44:04
 * @LastEditTime: 2026-05-03 21:51:47
 * @LastEditors: owen (keaa@keaa.net)
 * @Description: High-level SDK runtime API for transport, cache, callbacks, and command watchdog.
 */
#pragma once

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "masc_chassis_can_sdk/can_transport.hpp"
#include "masc_chassis_can_sdk/state_cache.hpp"

namespace masc_chassis_can_sdk {

/// High-level SDK runtime for one MASC chassis.
///
/// ChassisSdk owns the CAN transport, receive thread, optional motion_control_cmd (0x100) periodic
/// command thread, decoded state cache, and callbacks. It does not reconnect automatically and does
/// not wait for command ACKs.
class ChassisSdk {
public:
    /// Called after motion_control_feedback (0x200) is decoded and cached.
    using MotionFeedbackCallback = std::function<void(const MotionFeedback &)>;
    /// Called after system_info_feedback (0x204) is decoded and cached.
    using SystemFeedbackCallback = std::function<void(const SystemFeedback &)>;
    /// Called after bms_basic_feedback (0x206) is decoded and cached.
    using BmsBasicFeedbackCallback = std::function<void(const BmsBasicFeedback &)>;
    /// Called after bms_fault_feedback (0x207) is decoded and cached.
    using BmsFaultFeedbackCallback = std::function<void(const BmsFaultFeedback &)>;
    /// Called after motion_mode_feedback (0x220) is decoded and cached.
    using MotionModeFeedbackCallback = std::function<void(const MotionModeFeedback &)>;
    /// Called after one power_wheel_feedback (0x251-0x258) frame is decoded and cached.
    using PowerWheelFeedbackCallback = std::function<void(const PowerWheelFeedback &)>;
    /// Called after one power_wheel_driver_feedback (0x261-0x268) frame is decoded and cached.
    using PowerWheelDriverFeedbackCallback = std::function<void(const PowerWheelDriverFeedback &)>;
    /// Called after one servo_feedback (0x271-0x278) frame is decoded and cached.
    using ServoFeedbackCallback = std::function<void(const ServoFeedback &)>;
    /// Called after one servo_driver_feedback (0x281-0x288) frame is decoded and cached.
    using ServoDriverFeedbackCallback = std::function<void(const ServoDriverFeedback &)>;
    /// Called after imu_feedback (0x300) is decoded and cached.
    using ImuFeedbackCallback = std::function<void(const ImuFeedback &)>;
    /// Called after rc_feedback (0x301) is decoded and cached.
    using RcFeedbackCallback = std::function<void(const RcFeedback &)>;
    /// Called when a received frame is not decoded by the current V10 protocol parser.
    using RawFrameCallback = std::function<void(const CanFrame &)>;
    /// Called when health state changes or SDK emits a health notification.
    using HealthCallback = std::function<void(const HealthSnapshot &)>;

    /// Constructs an SDK with the platform default transport.
    ///
    /// Linux builds use SocketCanTransport. Other platforms use UnsupportedCanTransport until a
    /// vendor CAN backend is added.
    ChassisSdk();
    /// Constructs an SDK with a custom transport, mainly for tests or vendor-specific backends.
    explicit ChassisSdk(std::unique_ptr<ICanTransport> transport);
    /// Stops threads and closes the transport.
    ~ChassisSdk();

    ChassisSdk(const ChassisSdk &) = delete;
    ChassisSdk &operator=(const ChassisSdk &) = delete;

    /// Updates SDK configuration.
    ///
    /// Returns false if called while the SDK transport is open.
    bool Configure(const CanConfig &config);
    /// Opens the CAN transport and waits for a valid system_version_feedback (0x205) frame.
    ///
    /// Connect() returns true only after system_version_feedback.protocol_version matches
    /// CanConfig::m_expected_protocol_version. It does not perform OS-level CAN setup and does not
    /// retry automatically.
    bool Connect();
    /// Stops SDK threads and closes the transport. Safe to call more than once.
    void Disconnect();
    /// Returns true after a successful Connect() handshake and before Disconnect().
    bool IsConnected() const;

    /// Refreshes the latest motion command used by the SDK motion_control_cmd (0x100) sender.
    ///
    /// If this method is not called within CanConfig::m_motion_command_timeout_ms, the SDK sends a
    /// zero motion command while keeping the connection open.
    bool SetMotionCommand(const MotionCommand &command);
    /// Sends one motion_control_cmd (0x100) immediately.
    ///
    /// This helper is useful for tests/tools. Normal runtime control should use SetMotionCommand().
    bool SendMotionCommandOnce(const MotionCommand &command);
    /// Sends one tow_mode_control_cmd (0x101). The SDK does not wait for feedback confirmation.
    bool SendTowMode(bool enable);
    /// Sends one motion_mode_switch_cmd (0x110) when the connected chassis type supports it.
    ///
    /// The SDK does not wait for motion_mode_feedback (0x220) confirmation. Chassis types outside
    /// the supported range reported by system_version_feedback (0x205) are rejected with false.
    bool SendMotionMode(MotionMode mode);
    /// Sends one fault_clear_cmd (0x120). The SDK does not interpret whether clearing succeeded.
    bool SendFaultClear(std::uint16_t fault_id);
    /// Sends one servo_calibration_cmd (0x170) to start automatic servo calibration.
    bool SendCalibrateServo(std::uint16_t servo_id);
    /// Sends one raw standard CAN frame for diagnostics or protocol bring-up tools.
    ///
    /// The frame must use an 11-bit standard CAN ID and a classic CAN DLC in [0, 8]. Normal
    /// application control should prefer the typed command APIs above.
    bool SendRawFrame(const CanFrame &frame);

    /// Returns cached system_version_feedback (0x205) information, if received.
    std::optional<VersionInfo> GetVersionInfo() const;
    /// Returns whether the connected chassis type supports motion_mode_switch_cmd (0x110) and
    /// motion_mode_feedback (0x220).
    bool IsMotionModeSwitchSupported() const;
    /// Returns latest cached motion_control_feedback (0x200).
    std::optional<MotionFeedback> GetMotionFeedback() const;
    /// Returns latest cached system_info_feedback (0x204).
    std::optional<SystemFeedback> GetSystemFeedback() const;
    /// Returns latest cached bms_basic_feedback (0x206).
    std::optional<BmsBasicFeedback> GetBmsBasicFeedback() const;
    /// Returns latest cached bms_fault_feedback (0x207).
    std::optional<BmsFaultFeedback> GetBmsFaultFeedback() const;
    /// Returns latest cached motion_mode_feedback (0x220).
    std::optional<MotionModeFeedback> GetMotionModeFeedback() const;
    /// Returns cached power wheel feedback by zero-based protocol index.
    std::optional<PowerWheelFeedback> GetPowerWheelFeedback(std::size_t index) const;
    /// Returns cached power wheel driver feedback by zero-based protocol index.
    std::optional<PowerWheelDriverFeedback> GetPowerWheelDriverFeedback(std::size_t index) const;
    /// Returns cached servo feedback by zero-based protocol index.
    std::optional<ServoFeedback> GetServoFeedback(std::size_t index) const;
    /// Returns cached servo driver feedback by zero-based protocol index.
    std::optional<ServoDriverFeedback> GetServoDriverFeedback(std::size_t index) const;
    /// Returns latest cached imu_feedback (0x300).
    std::optional<ImuFeedback> GetImuFeedback() const;
    /// Returns latest cached rc_feedback (0x301).
    std::optional<RcFeedback> GetRcFeedback() const;
    /// Returns a copy of current SDK health state.
    HealthSnapshot GetHealthSnapshot() const;
    /// Returns per-CAN-ID statistics, if the SDK has seen or dropped that CAN ID.
    std::optional<FrameStats> GetFrameStats(std::uint16_t can_id) const;

    /// Registers or replaces the motion feedback callback.
    ///
    /// Callbacks run from the SDK receive dispatch path and should return quickly.
    void SetMotionFeedbackCallback(MotionFeedbackCallback callback);
    /// Registers or replaces the system feedback callback.
    void SetSystemFeedbackCallback(SystemFeedbackCallback callback);
    /// Registers or replaces the BMS basic feedback callback.
    void SetBmsBasicFeedbackCallback(BmsBasicFeedbackCallback callback);
    /// Registers or replaces the BMS fault feedback callback.
    void SetBmsFaultFeedbackCallback(BmsFaultFeedbackCallback callback);
    /// Registers or replaces the motion mode feedback callback.
    void SetMotionModeFeedbackCallback(MotionModeFeedbackCallback callback);
    /// Registers or replaces the power wheel feedback callback.
    void SetPowerWheelFeedbackCallback(PowerWheelFeedbackCallback callback);
    /// Registers or replaces the power wheel driver feedback callback.
    void SetPowerWheelDriverFeedbackCallback(PowerWheelDriverFeedbackCallback callback);
    /// Registers or replaces the servo feedback callback.
    void SetServoFeedbackCallback(ServoFeedbackCallback callback);
    /// Registers or replaces the servo driver feedback callback.
    void SetServoDriverFeedbackCallback(ServoDriverFeedbackCallback callback);
    /// Registers or replaces the IMU feedback callback.
    void SetImuFeedbackCallback(ImuFeedbackCallback callback);
    /// Registers or replaces the RC feedback callback.
    void SetRcFeedbackCallback(RcFeedbackCallback callback);
    /// Registers or replaces the raw-frame callback.
    ///
    /// This callback is intended for diagnostics and future protocol extension. It is invoked when
    /// the SDK receives an unsupported CAN ID, unexpected protocol DLC, or a supported frame that
    /// fails parser validation.
    void SetRawFrameCallback(RawFrameCallback callback);
    /// Registers or replaces the health callback.
    void SetHealthCallback(HealthCallback callback);

private:
    void ReceiveLoop();
    void MotionCommandLoop();
    void DispatchFrame(const CanFrame &frame);
    bool WriteCanFrame(const CanFrame &frame);
    void NotifyRawFrame(const CanFrame &frame, RawFrameCallback callback);
    void UpdateFrameStatsLocked(std::uint16_t can_id, bool parse_success);
    void IncrementDropLocked(std::uint16_t can_id);
    void NotifyHealth();
    void StopThreads();

    mutable std::mutex m_mutex;
    std::condition_variable m_version_cv;
    CanConfig m_config;
    std::unique_ptr<ICanTransport> m_transport;
    ChassisStateCache m_cache;
    MotionCommand m_latest_motion_command;
    std::chrono::steady_clock::time_point m_last_motion_command_update_time;

    bool m_stop_requested = false;
    bool m_receive_thread_running = false;
    bool m_motion_thread_running = false;
    std::thread m_receive_thread;
    std::thread m_motion_thread;

    MotionFeedbackCallback m_motion_feedback_callback;
    SystemFeedbackCallback m_system_feedback_callback;
    BmsBasicFeedbackCallback m_bms_basic_feedback_callback;
    BmsFaultFeedbackCallback m_bms_fault_feedback_callback;
    MotionModeFeedbackCallback m_motion_mode_feedback_callback;
    PowerWheelFeedbackCallback m_power_wheel_feedback_callback;
    PowerWheelDriverFeedbackCallback m_power_wheel_driver_feedback_callback;
    ServoFeedbackCallback m_servo_feedback_callback;
    ServoDriverFeedbackCallback m_servo_driver_feedback_callback;
    ImuFeedbackCallback m_imu_feedback_callback;
    RcFeedbackCallback m_rc_feedback_callback;
    RawFrameCallback m_raw_frame_callback;
    HealthCallback m_health_callback;
};

}  // namespace masc_chassis_can_sdk
