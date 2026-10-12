/*
 * Copyright (C) 2026, MASC.
 * @Version: V1.0
 * @Author: owen (keaa@keaa.net)
 * @Date: 2026-05-03 21:44:04
 * @LastEditTime: 2026-05-03 21:51:41
 * @LastEditors: owen (keaa@keaa.net)
 * @Description: Platform-neutral CAN transport interface for SDK runtime.
 */
#pragma once

#include <string>

#include "masc_chassis_can_sdk/can_frame.hpp"
#include "masc_chassis_can_sdk/types.hpp"

namespace masc_chassis_can_sdk {

/// Result of a transport read attempt.
enum class CanReadStatus {
    /// A complete CAN frame was read.
    Frame,
    /// No frame arrived before the requested timeout.
    Timeout,
    /// A transport error occurred. Call GetLastError() for a human-readable reason.
    Error,
};

/// Abstract CAN transport used by ChassisSdk.
///
/// Implementations may wrap Linux SocketCAN, a Windows vendor SDK, or a test/mock transport. The
/// interface is deliberately frame-oriented and does not expose platform-specific CAN types.
class ICanTransport {
public:
    virtual ~ICanTransport() = default;

    /// Opens the transport with the supplied SDK configuration.
    ///
    /// Implementations should only open/bind the interface. They must not perform SDK-level
    /// protocol handshakes or automatic reconnect.
    virtual bool Open(const CanConfig &config) = 0;
    /// Closes the transport. Safe to call more than once.
    virtual void Close() = 0;
    /// Returns true when the underlying transport is currently open.
    virtual bool IsOpen() const = 0;
    /// Reads one frame, waiting up to timeout_ms.
    ///
    /// On CanReadStatus::Frame, frame must contain a complete platform-neutral CanFrame.
    virtual CanReadStatus ReadFrame(CanFrame *frame, int timeout_ms) = 0;
    /// Writes one complete CAN frame.
    virtual bool WriteFrame(const CanFrame &frame) = 0;
    /// Returns the last transport-level error message.
    virtual std::string GetLastError() const = 0;
};

}  // namespace masc_chassis_can_sdk
