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
extern std::atomic<bool> g_libcamera_xrgb;

// Gyro tuning (runtime overrides)
extern std::atomic<double> g_gyro_gain_roll;
extern std::atomic<double> g_gyro_gain_pitch;
extern std::atomic<double> g_gyro_gain_yaw;
extern std::atomic<double> g_gyro_max_roll_rad;
extern std::atomic<double> g_gyro_max_pitch_rad;
extern std::atomic<double> g_gyro_max_yaw_rad;
extern std::atomic<double> g_gyro_hp_lpf_alpha;
extern std::atomic<double> g_gyro_hp_gain_roll;
extern std::atomic<double> g_gyro_hp_gain_pitch;
extern std::atomic<double> g_gyro_hp_gain_yaw;
extern std::atomic<double> g_gyro_large_rot_thresh_deg;
extern std::atomic<double> g_gyro_large_rot_gain_scale;

// LK tuning (runtime overrides)
extern std::atomic<bool> g_lk_trans_enable;
extern std::atomic<int> g_lk_max_features;
extern std::atomic<double> g_lk_quality;
extern std::atomic<double> g_lk_min_dist;
extern std::atomic<int> g_lk_trans_min_features;
extern std::atomic<int> g_lk_trans_every_n;
extern std::atomic<double> g_lk_trans_alpha;
extern std::atomic<double> g_lk_trans_max_corr_px;
extern std::atomic<double> g_lk_trans_scale;

// Time helpers
int64_t clock_ns(clockid_t id);
extern int64_t g_time_origin_raw_ns;
double now_ms();
