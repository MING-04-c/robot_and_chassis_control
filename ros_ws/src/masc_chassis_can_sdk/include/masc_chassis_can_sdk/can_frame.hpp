/*
 * Copyright (C) 2026, MASC.
 * @Version: V1.0
 * @Author: owen (keaa@keaa.net)
 * @Date: 2026-05-03 21:37:05
 * @LastEditTime: 2026-05-03 21:51:26
 * @LastEditors: owen (keaa@keaa.net)
 * @Description: Classic CAN frame model used by MASC Chassis CAN SDK.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace masc_chassis_can_sdk {

/// Maximum payload storage for a classic CAN frame.
constexpr std::size_t kCanPayloadSize = 8;
/// Maximum DLC for a classic CAN frame.
constexpr std::uint8_t kMaxCanDlc = 8;
/// Backward-compatible alias for the maximum classic CAN DLC.
constexpr std::uint8_t kCanDlc = kMaxCanDlc;
/// Maximum 11-bit standard CAN identifier.
constexpr std::uint16_t kMaxStandardCanId = 0x7FF;

/// Platform-neutral CAN frame used by the SDK.
///
/// This type intentionally avoids SocketCAN or vendor-specific fields so the protocol and runtime
/// layers can stay portable.
struct CanFrame {
    /// 11-bit standard CAN identifier.
    std::uint16_t m_id = 0;
    /// Data length code. Protocol-level code validates the expected DLC for each CAN ID.
    std::uint8_t m_dlc = kMaxCanDlc;
    /// Payload bytes. Bytes beyond m_dlc are local padding and are not transmitted as payload.
    std::array<std::uint8_t, kCanPayloadSize> m_data{};
};

/// Returns true when the CAN ID fits in an 11-bit standard identifier.
inline bool IsStandardCanId(std::uint16_t can_id) {
    return can_id <= kMaxStandardCanId;
}

/// Returns true when the DLC fits a classic CAN data frame.
inline bool HasValidClassicCanDlc(const CanFrame &frame) {
    return frame.m_dlc <= kMaxCanDlc;
}

}  // namespace masc_chassis_can_sdk
