// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// RobotException.hpp
// Provides the RobotException and RobotError classes for handling robot exceptions.
#ifndef __ELITE_ROBOT_EXCEPTION_HPP__
#define __ELITE_ROBOT_EXCEPTION_HPP__

#include <Elite/EliteOptions.hpp>
#include <cstdint>
#include <memory>
#include <string>

#if (ELITE_SDK_COMPILE_STANDARD >= 17)
#include <variant>
#elif (ELITE_SDK_COMPILE_STANDARD == 14)
#include <boost/variant.hpp>
#endif

namespace ELITE {

/**
 * @brief The base class for robot exceptions, representing common exception properties such as type and timestamp.
 *
 */
class RobotException {
   public:
    // Exception Type
    enum class Type : int8_t {
        ROBOT_DISCONNECTED = -1,  // Represents a disconnection from the robot.
        ROBOT_ERROR = 6,          // Represents a robot operation error.
        SCRIPT_RUNTIME = 10       // Represents a runtime error, such as script execution or syntax issues.
    };

    /**
     * @brief Get exception Type
     *
     * @return Type Exception type.
     */
    Type getType() { return type_; }

    /**
     * @brief Get the Timestamp
     *
     * @return uint64_t The timestamp when the exception occurred (in milliseconds).
     */
    uint64_t getTimestamp() { return timestamp_; }

    /**
     * @brief Construct a new Robot Exception object
     *
     * @param type The type of exception (ROBOT_ERROR or SCRIPT_RUNTIME).
     * @param ts Timestamp in milliseconds.
     */
    RobotException(Type type, uint64_t ts) : type_(type), timestamp_(ts) {}

    RobotException() = delete;
    ~RobotException() = default;

   protected:
    Type type_;
    uint64_t timestamp_;
};

/**
 * @brief Represents an error exception on the robot side, including error codes, source modules, severity level, and additional
 * data. It describes errors at the controller or hardware level.
 *
 */
class RobotError : public RobotException {
   public:
    // Error source
    enum class Source : int {
        SAFETY = 99,       // Safety controller
        GUI = 103,         // Teach pendant UI
        CONTROLLER = 104,  // Controller
        RTSI = 105,        // RTSI protocol
        JOINT = 120,       // Joint
        TOOL = 121,        // Tool
        TP = 122,          // Teach pendant
        JOINT_FPGA = 200,  // Joint FPGA
        TOOL_FPGA = 201    // Tool FPGA
    };

    // Error data type
    enum class DataType : uint32_t {
        NONE = 0,      // None data
        UNSIGNED = 1,  // Unsigned int data, uint32_t
        SIGNED = 2,    // Signed int data, int32_t
        FLOAT = 3,     // Float data, float
        HEX = 4,       // Unsigned integer (it is recommended to use hexadecimal representation)
        STRING = 5,    // String
        JOINT = 6      // Joint exception data, int32_t
    };

    // Error level
    enum class Level : int { INFO, WARNING, ERROR, FATAL };

#if (ELITE_SDK_COMPILE_STANDARD >= 17)
    using Data = std::variant<uint32_t, int32_t, float, std::string>;
#elif (ELITE_SDK_COMPILE_STANDARD == 14)
    using Data = boost::variant<uint32_t, int32_t, float, std::string>;
#endif

    /**
     * @brief Get the error code
     *
     * @return int Error code
     */
    int getErrorCode() { return code_; }

    /**
     * @brief Get the error sub-code
     *
     * @return int Error sub-code
     */
    int getSubErrorCode() { return sub_code_; }

    /**
     * @brief Get the error souce
     *
     * @return Source Error souce
     */
    Source getErrorSouce() { return error_source_; }

    /**
     * @brief Get the error level
     *
     * @return Level error level
     */
    Level getErrorLevel() { return error_level_; }

    /**
     * @brief Get the error data type
     *
     * @return DataType Error data type
     */
    DataType getErrorDataType() { return error_data_type_; }

    /**
     * @brief Get the data
     *
     * @return Data
     */
    Data getData() { return data_; };

    /**
     * @brief Construct a new Robot Error object
     *
     * @param ts timestamp
     * @param code Error code
     * @param sc Sub-error code.
     * @param es Error source module.
     * @param el Error level.
     * @param et Error data type.
     * @param data Additional data (e.g., string, integer, float).
     */
    RobotError(uint64_t ts, int code, int sc, Source es, Level el, DataType et, Data data)
        : RobotException(RobotException::Type::ROBOT_ERROR, ts),
          code_(code),
          sub_code_(sc),
          error_source_(es),
          error_level_(el),
          error_data_type_(et),
          data_(data) {}

    RobotError() = delete;
    ~RobotError() = default;

   private:
    int code_;

    int sub_code_;

    Source error_source_;

    Level error_level_;

    DataType error_data_type_;

    Data data_;
};

/**
 * @brief Represents runtime exceptions in the robot, such as syntax or execution errors in scripts. It contains line/column
 * information and error messages.
 *
 */
class RobotRuntimeException : public RobotException {
   private:
    int line_;
    int column_;
    std::string message_;

   public:
    /**
     * @brief Line number of the script where the exception occurred.
     *
     * @return int Line number
     */
    int getLine() { return line_; }

    /**
     * @brief Column number of the script where the exception occurred.
     *
     * @return int Column number
     */
    int getColumn() { return column_; }

    /**
     * @brief Script exception message
     *
     * @return const std::string& message
     */
    const std::string& getMessage() { return message_; }

    /**
     * @brief Construct a new Robot Runtime Exception object
     *
     * @param ts timestamp
     * @param line Line number of the script where the exception occurred.
     * @param column Column number of the script where the exception occurred.
     * @param msg Script exception message
     */
    RobotRuntimeException(uint64_t ts, int line, int column, std::string&& msg)
        : RobotException(RobotException::Type::SCRIPT_RUNTIME, ts), line_(line), column_(column), message_(std::move(msg)) {}

    RobotRuntimeException() = delete;
    ~RobotRuntimeException() = default;
};

using RobotExceptionSharedPtr = std::shared_ptr<RobotException>;
using RobotErrorSharedPtr = std::shared_ptr<RobotError>;
using RobotRuntimeExceptionSharedPtr = std::shared_ptr<RobotRuntimeException>;

}  // namespace ELITE

#endif