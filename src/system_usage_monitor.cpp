#include "system_usage_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>

namespace {

constexpr const char* kProcStatPath = "/proc/stat";
constexpr const char* kProcMeminfoPath = "/proc/meminfo";

} // namespace

SystemUsageMonitor::SystemUsageMonitor(std::chrono::milliseconds refresh_interval)
    : refresh_interval_(refresh_interval),
      last_refresh_(std::chrono::steady_clock::time_point::min()) {}

std::optional<SystemUsageInfo> SystemUsageMonitor::snapshot() {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto now = std::chrono::steady_clock::now();
    if (cached_usage_.has_value() &&
        (now - last_refresh_) < refresh_interval_) {
        return cached_usage_;
    }

    cached_usage_ = read_usage_locked();
    last_refresh_ = now;
    return cached_usage_;
}

std::optional<SystemUsageInfo> SystemUsageMonitor::read_usage_locked() {
    CpuTimes cpu_times;
    int memory_percent = -1;
    if (!read_cpu_times(cpu_times) || !read_memory_percent(memory_percent)) {
        return std::nullopt;
    }

    int cpu_percent = -1;
    if (prev_cpu_times_.has_value()) {
        const uint64_t prev_total = prev_cpu_times_->total();
        const uint64_t curr_total = cpu_times.total();
        const uint64_t total_delta = curr_total > prev_total ? (curr_total - prev_total) : 0;

        const uint64_t prev_idle = prev_cpu_times_->idle + prev_cpu_times_->iowait;
        const uint64_t curr_idle = cpu_times.idle + cpu_times.iowait;
        const uint64_t idle_delta = curr_idle > prev_idle ? (curr_idle - prev_idle) : 0;

        if (total_delta > 0) {
            const double busy_ratio =
                1.0 - (static_cast<double>(idle_delta) / static_cast<double>(total_delta));
            cpu_percent = static_cast<int>(std::lround(std::clamp(busy_ratio, 0.0, 1.0) * 100.0));
        }
    }
    prev_cpu_times_ = cpu_times;

    SystemUsageInfo usage;
    usage.cpu = cpu_percent;
    usage.memory = memory_percent;
    return usage;
}

bool SystemUsageMonitor::read_cpu_times(CpuTimes& out_times) {
    std::ifstream input(kProcStatPath);
    if (!input.is_open()) {
        return false;
    }

    std::string cpu_label;
    input >> cpu_label
          >> out_times.user
          >> out_times.nice
          >> out_times.system
          >> out_times.idle
          >> out_times.iowait
          >> out_times.irq
          >> out_times.softirq
          >> out_times.steal;
    return input.good() && cpu_label == "cpu";
}

bool SystemUsageMonitor::read_memory_percent(int& out_percent) {
    std::ifstream input(kProcMeminfoPath);
    if (!input.is_open()) {
        return false;
    }

    long long mem_total_kb = -1;
    long long mem_available_kb = -1;
    std::string key;
    long long value_kb = 0;
    std::string unit;
    while (input >> key >> value_kb >> unit) {
        if (key == "MemTotal:") {
            mem_total_kb = value_kb;
        } else if (key == "MemAvailable:") {
            mem_available_kb = value_kb;
        }

        if (mem_total_kb > 0 && mem_available_kb >= 0) {
            break;
        }
    }

    if (mem_total_kb <= 0 || mem_available_kb < 0 || mem_available_kb > mem_total_kb) {
        return false;
    }

    const long long used_kb = mem_total_kb - mem_available_kb;
    const double used_ratio = static_cast<double>(used_kb) / static_cast<double>(mem_total_kb);
    out_percent = static_cast<int>(std::lround(std::clamp(used_ratio, 0.0, 1.0) * 100.0));
    return true;
}
