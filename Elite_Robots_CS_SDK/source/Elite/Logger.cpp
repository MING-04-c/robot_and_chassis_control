// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "Logger.hpp"

namespace ELITE{

Logger& getLogger() {
    static Logger* s_logger = new Logger();
    return *s_logger;
}

}
