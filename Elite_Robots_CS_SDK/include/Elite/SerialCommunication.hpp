// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// SerialCommunication.hpp
// Provides the SerialCommunication class for RS485 communication over TCP.
#ifndef __ELITE__SERIAL_COMMUNICATION_HPP__
#define __ELITE__SERIAL_COMMUNICATION_HPP__

#include <Elite/EliteOptions.hpp>
#include <memory>
#include <string>
#include <mutex>

namespace ELITE {
/**
 * @brief RS485 configuration structure
 *
 */
class SerialConfig {
   public:
    /**
     * @brief Baud rate enumeration
     *
     */
    enum class BaudRate : int {
        BR_2400 = 2400,
        BR_4800 = 4800,
        BR_9600 = 9600,
        BR_19200 = 19200,
        BR_38400 = 38400,
        BR_57600 = 57600,
        BR_115200 = 115200,
        BR_460800 = 460800,
        BR_1000000 = 1000000,
        BR_2000000 = 2000000,
    };

    /**
     * @brief Parity enumeration
     *
     */
    enum class Parity : int {
        NONE = 0,
        ODD = 1,
        EVEN = 2,
    };

    /**
     * @brief Stop bits enumeration
     *
     */
    enum class StopBits : int { ONE = 1, TWO = 2 };

    BaudRate baud_rate = BaudRate::BR_115200;
    Parity parity = Parity::NONE;
    StopBits stop_bits = StopBits::ONE;

    ELITE_EXPORT SerialConfig() = default;
    ELITE_EXPORT ~SerialConfig() = default;
};

/**
 * @brief RS485 communication class.
 *
 * This class provides an interface for RS485 communication over TCP.
 *
 */
class SerialCommunication {
   public:
    /**
     * @brief Construct a new Serial Communication object
     *
     */
    ELITE_EXPORT explicit SerialCommunication() = default;

    /**
     * @brief Destroy the Serial Communication object
     *
     */
    ELITE_EXPORT virtual ~SerialCommunication() = default;

    /**
     * @brief Connect to the RS485 TCP server.
     *
     * @param timeout_ms Timeout in milliseconds.
     * @return true success
     * @return false fail
     */
    ELITE_EXPORT virtual bool connect(int timeout_ms) = 0;

    /**
     * @brief Disconnect from the RS485 TCP server.
     *
     */
    ELITE_EXPORT virtual void disconnect() = 0;

    /**
     * @brief Write data to the RS485 TCP server.
     *
     * @param data data buffer
     * @param size data size
     * @return int success write size, -1 fail
     */
    ELITE_EXPORT virtual int write(const uint8_t* data, size_t size) = 0;

    /**
     * @brief Read data from the RS485 TCP server.
     *
     * @param data data buffer
     * @param size data size
     * @param timeout_ms timeout in milliseconds
     * @return int success read size, -1 fail
     */
    ELITE_EXPORT virtual int read(uint8_t* data, size_t size, int timeout_ms) = 0;

    /**
     * @brief Check if connected to the RS485 TCP server.
     *
     * @return true connected
     * @return false disconnect
     */
    ELITE_EXPORT virtual bool isConnected() = 0;

    /**
     * @brief Get the Socat PID
     * 
     * @return int socat pid 
     */
    ELITE_EXPORT virtual int getSocatPid() const = 0;
};

using SerialCommunicationSharedPtr = std::shared_ptr<SerialCommunication>;

}  // namespace ELITE

#endif
