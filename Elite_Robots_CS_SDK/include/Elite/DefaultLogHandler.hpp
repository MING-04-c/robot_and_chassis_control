// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
//
// DefaultLogHandler.hpp
// Provides the DefaultLogHandler class for logging messages to the console.
#ifndef __ELITE__DEFATULT_LOG_HPP__
#define __ELITE__DEFATULT_LOG_HPP__

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include "Log.hpp"

namespace ELITE {

class DefaultLogHandler : public LogHandler {
   private:
    std::string getCurrentTimeStringMs() {
        using namespace std::chrono;
        auto now = std::chrono::system_clock::now();
        auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
        std::time_t t = system_clock::to_time_t(now);
        std::tm tm;
#ifdef _WIN32
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(3) << std::setfill('0') << ms.count();
        return oss.str();
    }

   public:
    DefaultLogHandler() = default;
    ~DefaultLogHandler() = default;

    void log(const char* file, int line, LogLevel level, const char* log) {
        std::string time_str = getCurrentTimeStringMs();
        switch (level) {
            case LogLevel::ELI_DEBUG:
                std::cout << "[" << time_str << " DEBUG] " << file << ":" << line << ": " << log << std::endl;
                break;
            case LogLevel::ELI_INFO:
                std::cout << "[" << time_str << " INFO ] " << file << ":" << line << ": " << log << std::endl;
                break;
            case LogLevel::ELI_WARN:
                std::cout << "[" << time_str << " WARN ] " << file << ":" << line << ": " << log << std::endl;
                break;
            case LogLevel::ELI_ERROR:
                std::cout << "[" << time_str << " ERROR] " << file << ":" << line << ": " << log << std::endl;
                break;
            case LogLevel::ELI_FATAL:
                std::cout << "[" << time_str << " FATAL] " << file << ":" << line << ": " << log << std::endl;
                break;
            case LogLevel::ELI_NONE:
                std::cout << "[" << time_str << " NONE ] " << file << ":" << line << ": " << log << std::endl;
                break;
            default:
                break;
        }
    }
};

}  // namespace ELITE

#endif
