// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// Logger.hpp
// Provides the Logger class for managing log handlers and log levels.
#ifndef __ELITE__LOGGER_HPP__
#define __ELITE__LOGGER_HPP__

#include "DefaultLogHandler.hpp"
#include "Log.hpp"

namespace ELITE {

class Logger {
   private:
    LogLevel level_;
    std::unique_ptr<LogHandler> handler_;

   public:
    Logger() {
        level_ = LogLevel::ELI_INFO;
        handler_.reset(new DefaultLogHandler);
    };

    ~Logger() = default;

    void setLevel(LogLevel level) { level_ = level; }

    void registerHandler(std::unique_ptr<LogHandler>& handler) { handler_ = std::move(handler); }

    void unregisterHandler() { handler_.reset(new DefaultLogHandler); }

    void log(const char* file, int line, LogLevel level, const char* log) {
        if (!handler_) {
            handler_.reset(new DefaultLogHandler());
        }
        handler_->log(file, line, level, log);
    }

    LogLevel getLogLevel() { return level_; }
};

Logger& getLogger();

}  // namespace ELITE

#endif
