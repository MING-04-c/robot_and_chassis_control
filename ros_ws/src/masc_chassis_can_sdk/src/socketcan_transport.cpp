/*
 * Copyright (C) 2026, MASC.
 * @Version: V1.0
 * @Author: owen (keaa@keaa.net)
 * @Date: 2026-05-03 21:45:24
 * @LastEditTime: 2026-05-03 21:52:29
 * @LastEditors: owen (keaa@keaa.net)
 * @Description: Linux SocketCAN open, read, write, and close implementation.
 */
#include "masc_chassis_can_sdk/socketcan_transport.hpp"

#if defined(__linux__)

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <cstdint>

namespace masc_chassis_can_sdk {
namespace {

std::string ErrnoMessage(const std::string &prefix) {
    return prefix + ": " + std::strerror(errno);
}

}  // namespace

SocketCanTransport::~SocketCanTransport() {
    Close();
}

bool SocketCanTransport::Open(const CanConfig &config) {
    Close();

    m_socket_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socket_fd < 0) {
        m_last_error = ErrnoMessage("socket(PF_CAN) failed");
        return false;
    }

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, config.m_interface_name.c_str(), IFNAMSIZ - 1);
    if (::ioctl(m_socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        m_last_error = ErrnoMessage("ioctl(SIOCGIFINDEX) failed");
        Close();
        return false;
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = ifr.ifr_ifindex;

    if (::bind(m_socket_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
        m_last_error = ErrnoMessage("bind(AF_CAN) failed");
        Close();
        return false;
    }

    m_last_error.clear();
    return true;
}

void SocketCanTransport::Close() {
    if (m_socket_fd >= 0) {
        ::close(m_socket_fd);
        m_socket_fd = -1;
    }
}

bool SocketCanTransport::IsOpen() const {
    return m_socket_fd >= 0;
}

CanReadStatus SocketCanTransport::ReadFrame(CanFrame *frame, int timeout_ms) {
    if (frame == nullptr || m_socket_fd < 0) {
        m_last_error = "ReadFrame called while socket is closed";
        return CanReadStatus::Error;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(m_socket_fd, &read_fds);

    timeval timeout{};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    const int select_result = ::select(m_socket_fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (select_result == 0) {
        return CanReadStatus::Timeout;
    }
    if (select_result < 0) {
        if (errno == EINTR) {
            return CanReadStatus::Timeout;
        }
        m_last_error = ErrnoMessage("select() failed");
        return CanReadStatus::Error;
    }

    can_frame native_frame{};
    const auto read_size = ::read(m_socket_fd, &native_frame, sizeof(native_frame));
    if (read_size != static_cast<ssize_t>(sizeof(native_frame))) {
        m_last_error = ErrnoMessage("read(can_frame) failed");
        return CanReadStatus::Error;
    }

    if ((native_frame.can_id & CAN_EFF_FLAG) != 0U) {
        frame->m_id = kMaxStandardCanId + 1U;
    } else {
        frame->m_id = static_cast<std::uint16_t>(native_frame.can_id & CAN_SFF_MASK);
    }
    frame->m_dlc = native_frame.can_dlc;
    frame->m_data.fill(0);
    for (std::size_t index = 0; index < frame->m_data.size() && index < native_frame.can_dlc; ++index) {
        frame->m_data[index] = native_frame.data[index];
    }
    return CanReadStatus::Frame;
}

bool SocketCanTransport::WriteFrame(const CanFrame &frame) {
    if (m_socket_fd < 0) {
        m_last_error = "WriteFrame called while socket is closed";
        return false;
    }
    if (!IsStandardCanId(frame.m_id) || !HasValidClassicCanDlc(frame)) {
        m_last_error = "invalid CAN frame for SocketCAN write";
        return false;
    }

    can_frame native_frame{};
    native_frame.can_id = frame.m_id;
    native_frame.can_dlc = frame.m_dlc;
    for (std::size_t index = 0; index < frame.m_data.size() && index < frame.m_dlc; ++index) {
        native_frame.data[index] = frame.m_data[index];
    }

    const auto write_size = ::write(m_socket_fd, &native_frame, sizeof(native_frame));
    if (write_size != static_cast<ssize_t>(sizeof(native_frame))) {
        m_last_error = ErrnoMessage("write(can_frame) failed");
        return false;
    }
    return true;
}

std::string SocketCanTransport::GetLastError() const {
    return m_last_error;
}

}  // namespace masc_chassis_can_sdk

#endif  // defined(__linux__)
