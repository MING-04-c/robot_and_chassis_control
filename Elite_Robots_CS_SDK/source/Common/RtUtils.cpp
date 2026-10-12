// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include "Common/RtUtils.hpp"
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <vector>
#include "Elite/Log.hpp"

#if defined(__linux) || defined(linux) || defined(__linux__)
#include <pthread.h>
#elif defined(_WIN32) || defined(_WIN64)
#define NOMINMAX
#include <windows.h>
#endif

namespace ELITE {

namespace RT_UTILS {

bool setThreadFiFoScheduling(std::thread::native_handle_type& thread, const int priority) {
#if defined(__linux) || defined(linux) || defined(__linux__)
    struct sched_param params;
    params.sched_priority = priority;
    int ret = pthread_setschedparam(thread, SCHED_FIFO, &params);
    if (ret != 0) {
        switch (ret) {
            case EPERM: {
                ELITE_LOG_WARN(
                    "Your system/user seems not to be setup for FIFO scheduling. We recommend using a lowlatency "
                    "kernel with FIFO scheduling.");
                break;
            }
            default: {
                ELITE_LOG_ERROR("Unsuccessful in setting thread to FIFO scheduling with priority %i. %s", priority, strerror(ret));
            }
        }
        return false;
    }

    // Now verify the change in thread priority
    int policy = 0;
    ret = pthread_getschedparam(thread, &policy, &params);
    if (ret != 0) {
        ELITE_LOG_ERROR("Couldn't retrieve scheduling parameters");
        return false;
    }

    // Check the correct policy was applied
    if (policy != SCHED_FIFO) {
        ELITE_LOG_ERROR("Scheduling is NOT SCHED_FIFO!");
        return false;
    } else {
        ELITE_LOG_INFO("SCHED_FIFO OK, priority %i", params.sched_priority);
        if (params.sched_priority != priority) {
            ELITE_LOG_ERROR("Thread priority is %i instead of the expected %i", params.sched_priority, priority);
            return false;
        }
    }
    return true;
#elif defined(_WIN32) || defined(_WIN64)
    return ::SetThreadPriority(thread, priority);
#endif
}

int getThreadFiFoMaxPriority() {
#if defined(__linux) || defined(linux) || defined(__linux__)
    return sched_get_priority_max(SCHED_FIFO);
#elif defined(_WIN32) || defined(_WIN64)
    DWORD priorityClass = GetPriorityClass(GetCurrentProcess());
    if (priorityClass == REALTIME_PRIORITY_CLASS) {
        return 31;  // real time process
    } else {
        return 15;
    }
#endif
}

bool bindThreadToCpus(std::thread::native_handle_type& thread, const int cpu) {
    const int ncpu = static_cast<int>(std::thread::hardware_concurrency());
    if (cpu < 0 || cpu >= ncpu) {
        ELITE_LOG_ERROR("CPU index %d out of range 0 - %d", cpu, ncpu - 1);
        return false;
    }
#if defined(__linux__)
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(cpu, &mask);

    int ret = pthread_setaffinity_np(thread, sizeof(mask), &mask);
    if (ret != 0) {
        ELITE_LOG_ERROR("pthread_setaffinity_np failed");
        return false;
    }

    return true;
#elif defined(_WIN32) || defined(_WIN64)
    DWORD_PTR mask = 0;
    mask |= (1ULL << cpu);

    HANDLE hThread = static_cast<HANDLE>(thread);
    DWORD_PTR prev = SetThreadAffinityMask(hThread, mask);
    if (!prev) {
        throw std::system_error(static_cast<int>(::GetLastError()), std::system_category(), "SetThreadAffinityMask failed");
    }
    return true;
#else
    throw std::runtime_error("CPU affinity not supported on this platform");
#endif
}

}  // namespace RT_UTILS

}  // namespace ELITE
