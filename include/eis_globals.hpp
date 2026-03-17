#pragma once

#include <atomic>
#include <mutex>
#include <gst/app/gstappsrc.h>
#include <time.h>

#include "eis_common.hpp"
#include "gyro_eis.hpp"

// Global run state
extern std::atomic<bool> g_running;

// RTSP appsrc
extern std::mutex g_mtx;
extern GstAppSrc* g_rawsrc;
extern GstAppSrc* g_stabsrc;

// IMU state
extern std::atomic<bool> g_imu_ready;
extern GyroBuffer g_gyro_buffer;
extern GyroSample g_last_gyro_sample;
extern std::atomic<double> g_imu_actual_hz;

// Runtime controls
extern std::atomic<int> g_mode;
extern std::atomic<int> g_output_mode;
extern std::atomic<bool> g_debug_overlay;
extern std::atomic<int> g_log_every_frames;
extern std::atomic<int> g_ts_pref;
extern std::atomic<bool> g_offset_sweep;
extern std::atomic<int> g_gyro_warp_mode;
extern double g_manual_imu_offset_ms;
extern std::atomic<int> g_profile;

// Time helpers
int64_t clock_ns(clockid_t id);
extern int64_t g_time_origin_raw_ns;
double now_ms();
