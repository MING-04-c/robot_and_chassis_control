// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// Log.hpp
// Provides logging functionality for the Elite Robotics SDK.
#ifndef __ELITE__LOG_HPP__
#define __ELITE__LOG_HPP__

#include <Elite/EliteOptions.hpp>
#include <memory>

#ifndef __REL_FILE__
#define __REL_FILE__ __FILE__
#endif

#define ELITE_LOG_DEBUG(...) ELITE::log(__REL_FILE__, __LINE__, ELITE::LogLevel::ELI_DEBUG, __VA_ARGS__)
#define ELITE_LOG_WARN(...) ELITE::log(__REL_FILE__, __LINE__, ELITE::LogLevel::ELI_WARN, __VA_ARGS__)
#define ELITE_LOG_INFO(...) ELITE::log(__REL_FILE__, __LINE__, ELITE::LogLevel::ELI_INFO, __VA_ARGS__)
#define ELITE_LOG_ERROR(...) ELITE::log(__REL_FILE__, __LINE__, ELITE::LogLevel::ELI_ERROR, __VA_ARGS__)
#define ELITE_LOG_FATAL(...) ELITE::log(__REL_FILE__, __LINE__, ELITE::LogLevel::ELI_FATAL, __VA_ARGS__)

namespace ELITE {

/**
 * @brief The log level
 *
 */
enum class LogLevel { ELI_DEBUG, ELI_INFO, ELI_WARN, ELI_ERROR, ELI_FATAL, ELI_NONE };

/**
 * @brief If you want to change the way you log,
 *  you can inherit from this class and register the instance through the registerLogHandler() function
 *
 */
class LogHandler {
   private:
   public:
    LogHandler() = default;
    virtual ~LogHandler() = default;

    /**
     * @brief Function to log a message
     *
     * @param file The log message comes from this file
     * @param line The log message comes from this line
     * @param loglevel The log level
     * @param log Log message
     */
    ELITE_EXPORT virtual void log(const char* file, int line, LogLevel loglevel, const char* log) = 0;
};

/**
 * @brief Register a new LogHandler object for processing log messages.
 *
 * @param hanlder Pointer to the new object
 */
ELITE_EXPORT void registerLogHandler(std::unique_ptr<LogHandler> hanlder);

/**
 * @brief Unregister current log handler, this will enable default log handler.
 *
 */
ELITE_EXPORT void unregisterLogHandler();

/**
 * @brief Log a message, this is used internally by the macros to unpack the log message.
 *  Should use the macros instead of this function directly.
 *
 * @param file The log message comes from this file
 * @param line The log message comes from this line
 * @param level Level of the log message
 * @param fmt Format string
 */
ELITE_EXPORT void log(const char* file, int line, LogLevel level, const char* fmt, ...);

/**
 * @brief Set log level this will disable messages with lower log level.
 *
 * @param level Desired log level
 */
ELITE_EXPORT void setLogLevel(LogLevel level);

}  // namespace ELITE

#endif