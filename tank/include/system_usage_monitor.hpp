#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>

#include "rc_status_types.h"

class SystemUsageMonitor {
public:
    explicit SystemUsageMonitor(std::chrono::milliseconds refresh_interval = std::chrono::milliseconds(1000));

    std::optional<SystemUsageInfo> snapshot();

private:
    struct CpuTimes {
        uint64_t user = 0;
        uint64_t nice = 0;
        uint64_t system = 0;
        uint64_t idle = 0;
        uint64_t iowait = 0;
        uint64_t irq = 0;
        uint64_t softirq = 0;
        uint64_t steal = 0;

        uint64_t total() const {
            return user + nice + system + idle + iowait + irq + softirq + steal;
        }
    };

    std::optional<SystemUsageInfo> read_usage_locked();
    static bool read_cpu_times(CpuTimes& out_times);
    static bool read_memory_percent(int& out_percent);

private:
    const std::chrono::milliseconds refresh_interval_;
    std::mutex mutex_;
    std::optional<SystemUsageInfo> cached_usage_;
    std::chrono::steady_clock::time_point last_refresh_;
    std::optional<CpuTimes> prev_cpu_times_;
};
