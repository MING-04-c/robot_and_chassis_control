/*
 * Copyright (C) 2026, MASC.
 * @Version: V1.0
 * @Author: owen (keaa@keaa.net)
 * @Date: 2026-05-03 21:44:04
 * @LastEditTime: 2026-05-03 21:52:09
 * @LastEditors: owen (keaa@keaa.net)
 * @Description: Unsupported transport stub for platforms without a CAN backend.
 */
#pragma once

#include <string>

#include "masc_chassis_can_sdk/can_transport.hpp"

namespace masc_chassis_can_sdk {

/// Placeholder transport for platforms without an implemented CAN backend.
///
/// v0.1 uses this for non-Linux builds. It lets the SDK compile while making runtime CAN access
/// fail explicitly until a vendor-specific backend is added.
class UnsupportedCanTransport : public ICanTransport {
public:
    bool Open(const CanConfig &config) override;
    void Close() override;
    bool IsOpen() const override;
    CanReadStatus ReadFrame(CanFrame *frame, int timeout_ms) override;
    bool WriteFrame(const CanFrame &frame) override;
    std::string GetLastError() const override;

private:
    std::string m_last_error = "CAN transport is unsupported on this platform";
};

}  // namespace masc_chassis_can_sdk
