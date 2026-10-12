// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// EndianUtils.hpp
// Provides utility functions for handling endianness in data conversion.
#ifndef __ELITE__ENDIAN_UTILS_HPP__
#define __ELITE__ENDIAN_UTILS_HPP__

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Elite/EliteOptions.hpp>

namespace ELITE {

class EndianUtils {
   private:
   public:
    EndianUtils() = default;
    virtual ~EndianUtils() = default;

    /**
     * @brief Convert bytes to base type
     *
     * @tparam T Must base type
     * @param message The byte buffer to be converted.
     * @param message_offset The offset of the bytes to be converted in the buffer.
     * @param out_value Output value.
     * @note
     *      1. The message_offset will be change
     *      2. The default data for messages is stored in big-endian byte order.
     */
    template <typename T>
    static void unpack(const std::vector<uint8_t>& message, int& message_offset, T& out_value) {
        unpack<T>(message.begin() + message_offset, out_value);
        message_offset += sizeof(T);
    }

    /**
     * @brief Convert bytes to base type
     *
     * @tparam T Must base type
     * @param message The byte buffer to be converted.
     * @param out_value Output value.
     */
    template <typename T>
    static void unpack(const std::vector<uint8_t>::const_iterator& message, T& out_value) {
        static_assert(std::is_fundamental<T>::value, "must use base type");
        union {
            T value;
            uint8_t bytes[sizeof(T)];
        } msg;
#if ELITE_SDK_COMPILE_STANDARD >= 17
        if constexpr (sizeof(T) == 8) {
            msg.bytes[0] = *(message + 7);
            msg.bytes[1] = *(message + 6);
            msg.bytes[2] = *(message + 5);
            msg.bytes[3] = *(message + 4);
            msg.bytes[4] = *(message + 3);
            msg.bytes[5] = *(message + 2);
            msg.bytes[6] = *(message + 1);
            msg.bytes[7] = *(message);
        } else if constexpr (sizeof(T) == 4) {
            msg.bytes[0] = *(message + 3);
            msg.bytes[1] = *(message + 2);
            msg.bytes[2] = *(message + 1);
            msg.bytes[3] = *(message);
        } else if constexpr (sizeof(T) == 2) {
            msg.bytes[0] = *(message + 1);
            msg.bytes[1] = *(message);
        } else if constexpr (sizeof(T) == 1) {
            out_value = *message;
        }
        out_value = msg.value;
#else
        switch (sizeof(T)) {
            case 8:
                msg.bytes[0] = *(message + 7);
                msg.bytes[1] = *(message + 6);
                msg.bytes[2] = *(message + 5);
                msg.bytes[3] = *(message + 4);
                msg.bytes[4] = *(message + 3);
                msg.bytes[5] = *(message + 2);
                msg.bytes[6] = *(message + 1);
                msg.bytes[7] = *(message);
                break;
            case 4:
                msg.bytes[0] = *(message + 3);
                msg.bytes[1] = *(message + 2);
                msg.bytes[2] = *(message + 1);
                msg.bytes[3] = *(message);
                break;
            case 2:
                msg.bytes[0] = *(message + 1);
                msg.bytes[1] = *(message);
                break;
            case 1:
                out_value = *message;
                return;
            default:
                break;
        }
        out_value = msg.value;
#endif
    }

    /**
     * @brief Convert bytes to array
     *
     * @tparam T The type in array
     * @tparam size Array size
     * @param message The byte buffer to be converted.
     * @param message_offset The offset of the bytes to be converted in the buffer.
     * @param out_value Output value
     * @note
     *      1. The message_offset will be change
     *      2. The default data for messages is stored in big-endian byte order.
     */
    template <typename T, int size>
    static void unpack(const std::vector<uint8_t>& message, int& message_offset, std::array<T, size>& out_value) {
        for (size_t i = 0; i < size; i++) {
            unpack<T>(message, message_offset, out_value[i]);
        }
    }

    /**
     * @brief Pack an value which is base type to bytes
     *
     * @tparam T Must base type
     * @param value Will be converted value
     * @return std::vector<uint8_t> The result in bytes
     * @note The endian of result is different of value
     */
    template <typename T>
    static std::vector<uint8_t> pack(const T value) {
        static_assert(std::is_fundamental<T>::value, "must use base type");
        std::vector<uint8_t> result(sizeof(T));
        union {
            T value;
            uint8_t bytes[sizeof(T)];
        } msg;
        msg.value = value;
        for (size_t i = 0; i < sizeof(T); i++) {
            result[(sizeof(T) - 1) - i] = msg.bytes[i];
        }
        return result;
    }

    /**
     * @brief Pack an array to bytes
     *
     * @tparam T The type in array
     * @tparam size Array size
     * @param value Will be converted array
     * @return std::vector<uint8_t> The result in bytes
     * * @note The endian of result is different of value
     */
    template <typename T, int size>
    static std::vector<uint8_t> pack(const std::array<T, size>& value) {
        std::vector<uint8_t> result;
        std::vector<uint8_t> temp;
        for (size_t i = 0; i < size; i++) {
            temp = pack<T>(value[i]);
            result.insert(result.end(), std::make_move_iterator(temp.begin()), std::make_move_iterator(temp.end()));
        }
        return result;
    }
};

}  // namespace ELITE

#endif