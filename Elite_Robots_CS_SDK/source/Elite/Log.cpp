// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "Log.hpp"
#include "Logger.hpp"
#include <cstdarg>


namespace ELITE{


void registerLogHandler(std::unique_ptr<LogHandler> hanlder) {
    getLogger().registerHandler(hanlder);
}

void unregisterLogHandler() {
    getLogger().unregisterHandler();
}

void setLogLevel(LogLevel level) {
    getLogger().setLevel(level);
}

void log(const char* file, int line, LogLevel level, const char* fmt, ...) {
    if (level >= getLogger().getLogLevel()) {
        size_t buffer_size = 4096;
        std::unique_ptr<char[]> buffer;
        buffer.reset(new char[buffer_size]);

        va_list args;
        va_start(args, fmt);
        va_list args_copy;
        va_copy(args_copy, args);

        size_t characters = 1 + std::vsnprintf(buffer.get(), buffer_size, fmt, args);

        if (characters >= buffer_size) {
            buffer_size = characters + 1;
            buffer.reset(new char[buffer_size]);
            std::vsnprintf(buffer.get(), buffer_size, fmt, args_copy);
        }

        va_end(args);
        va_end(args_copy);

        getLogger().log(file, line, level, buffer.get());
    }
}


}