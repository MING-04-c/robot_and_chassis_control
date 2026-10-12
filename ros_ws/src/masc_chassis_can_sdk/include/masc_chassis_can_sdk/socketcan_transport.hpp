/*
 * Copyright (C) 2026, MASC.
 * @Version: V1.0
 * @Author: owen (keaa@keaa.net)
 * @Date: 2026-05-03 21:44:04
 * @LastEditTime: 2026-05-03 21:51:52
 * @LastEditors: owen (keaa@keaa.net)
 * @Description: Linux SocketCAN transport backend declaration.
 */
#pragma once

#include <string>

#include "masc_chassis_can_sdk/can_transport.hpp"

namespace masc_chassis_can_sdk {

/// Linux SocketCAN transport backend.
///
/// This backend only opens, reads, writes, and closes an existing CAN network interface. It does
/// not run ip-link commands and does not reconnect automatically.
class SocketCanTransport : public ICanTransport {
public:
    SocketCanTransport() = default;
    ~SocketCanTransport() override;

    bool Open(const CanConfig &config) override;
    void Close() override;
    bool IsOpen() const override;
    CanReadStatus ReadFrame(CanFrame *frame, int timeout_ms) override;
    bool WriteFrame(const CanFrame &frame) override;
    std::string GetLastError() const override;

private:
    int m_socket_fd = -1;
    std::string m_last_error;
};

}  // namespace masc_chassis_can_sdk
