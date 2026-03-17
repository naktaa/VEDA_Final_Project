#include "eis_globals.hpp"

std::atomic<bool> g_running{true};

std::mutex g_mtx;
GstAppSrc* g_rawsrc = nullptr;
GstAppSrc* g_stabsrc = nullptr;

std::atomic<bool> g_imu_ready{false};
GyroBuffer g_gyro_buffer(IMU_STORE_SIZE);
GyroSample g_last_gyro_sample;
std::atomic<double> g_imu_actual_hz{0};

std::atomic<int> g_mode{(int)EisMode::GYRO};
std::atomic<int> g_output_mode{(int)OutputMode::CAM_ONLY};
std::atomic<bool> g_debug_overlay{DEFAULT_DEBUG_OVERLAY};
std::atomic<int> g_log_every_frames{-1};
std::atomic<int> g_ts_pref{(int)TsSourcePref::AUTO};
std::atomic<bool> g_offset_sweep{false};
std::atomic<int> g_gyro_warp_mode{(int)GyroWarpMode::JITTER};
double g_manual_imu_offset_ms = 0.0;
std::atomic<int> g_profile{(int)RunProfile::RUN};

int64_t clock_ns(clockid_t id) {
    struct timespec ts;
    clock_gettime(id, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

int64_t g_time_origin_raw_ns = clock_ns(CLOCK_MONOTONIC_RAW);

double now_ms() {
    return (clock_ns(CLOCK_MONOTONIC_RAW) - g_time_origin_raw_ns) / 1e6;
}
