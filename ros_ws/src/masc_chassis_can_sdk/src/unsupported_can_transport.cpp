/*
 * Copyright (C) 2026, MASC.
 * @Version: V1.0
 * @Author: owen (keaa@keaa.net)
 * @Date: 2026-05-03 21:45:24
 * @LastEditTime: 2026-05-03 21:52:37
 * @LastEditors: owen (keaa@keaa.net)
 * @Description: Unsupported transport implementation for unavailable CAN backends.
 */
#include "masc_chassis_can_sdk/unsupported_can_transport.hpp"

namespace masc_chassis_can_sdk {

bool UnsupportedCanTransport::Open(const CanConfig &config) {
    (void)config;
    m_last_error = "CAN transport is unsupported on this platform";
    return false;
}

void UnsupportedCanTransport::Close() {}

bool UnsupportedCanTransport::IsOpen() const {
    return false;
}

CanReadStatus UnsupportedCanTransport::ReadFrame(CanFrame *frame, int timeout_ms) {
    (void)frame;
    (void)timeout_ms;
    m_last_error = "CAN transport is unsupported on this platform";
    return CanReadStatus::Error;
}

bool UnsupportedCanTransport::WriteFrame(const CanFrame &frame) {
    (void)frame;
    m_last_error = "CAN transport is unsupported on this platform";
    return false;
}

std::string UnsupportedCanTransport::GetLastError() const {
    return m_last_error;
}

}  // namespace masc_chassis_can_sdk
