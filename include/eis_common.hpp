#pragma once

#include <opencv2/core.hpp>
#include <cstdint>

// ======================== Constants ========================

inline constexpr int G_WIDTH = 640;
inline constexpr int G_HEIGHT = 480;
inline constexpr int G_FPS = 20;
inline constexpr int64_t TARGET_FRAME_DURATION_US = 50000; // 20fps
inline constexpr int64_t TARGET_EXPOSURE_US = 20000;       // 20ms
inline constexpr int IMU_TARGET_HZ = 500;
inline constexpr int64_t IMU_TARGET_PERIOD_NS = 1000000000LL / IMU_TARGET_HZ;

// MPU-6050
inline constexpr int IMU_ADDR = 0x68;
inline constexpr double GYRO_SENSITIVITY = 131.0;

// Gyro stabilization defaults.
// Tuned to start from "RC car high-frequency vibration" assumptions:
// - prefer high-pass gyro mode
// - keep yaw correction conservative to avoid fighting intentional turns
inline constexpr double GYRO_GAIN_ROLL = 1.2;
inline constexpr double GYRO_GAIN_PITCH = 1.2;
inline constexpr double GYRO_GAIN_YAW = 0.45;
inline constexpr double GYRO_MAX_ROLL_RAD = 5.0 * CV_PI / 180.0;
inline constexpr double GYRO_MAX_PITCH_RAD = 4.0 * CV_PI / 180.0;
inline constexpr double GYRO_MAX_YAW_RAD = 8.0 * CV_PI / 180.0;
inline constexpr double GYRO_HP_LPF_ALPHA = 0.90;
inline constexpr double GYRO_HP_GAIN_ROLL = 1.0;
inline constexpr double GYRO_HP_GAIN_PITCH = 1.0;
inline constexpr double GYRO_HP_GAIN_YAW = 1.0;
inline constexpr double GYRO_LARGE_ROT_THRESH_DEG = 3.5;
inline constexpr double GYRO_LARGE_ROT_GAIN_SCALE = 0.40;

// LK (offset sweep/debug only)
inline constexpr int LK_MAX_FEATURES = 100;
inline constexpr double LK_QUALITY = 0.01;
inline constexpr double LK_MIN_DIST = 30.0;
inline constexpr int LK_TRANS_MIN_FEATURES = 12;
inline constexpr int LK_TRANS_MIN_INLIERS = 10;
inline constexpr double LK_TRANS_ALPHA = 0.55;
inline constexpr double LK_TRANS_MAX_CORR_PX = 20.0;
inline constexpr int LK_TRANS_EVERY_N = 1; // run LK translation every frame
inline constexpr double LK_TRANS_JERK_LIMIT_PX = 6.0;
inline constexpr double LK_TRANS_CONFIDENCE_GATE = 0.55;

// Crop
inline constexpr double FIXED_CROP_PERCENT = 20.0;

// Test environment: camera is vertically flipped (both axes)
inline constexpr bool FLIP_VERTICAL = true;

// Camera FOV
inline constexpr double HFOV_DEG = 62.2;
inline constexpr double VFOV_DEG = 48.8;

// Libcamera 4-channel packing default (XRGB vs XBGR)
inline constexpr bool DEFAULT_LIBCAMERA_XRGB = false;

// IMU axis mapping (from gyro_axis_plot test)
// Yaw right  -> gy negative
// Pitch up   -> gx negative
// Roll CW    -> gz positive
inline constexpr int IMU_AXIS_ROLL = 2;   // gz
inline constexpr int IMU_AXIS_PITCH = 0;  // gx
inline constexpr int IMU_AXIS_YAW = 1;    // gy
inline constexpr int IMU_SIGN_ROLL = 1;
inline constexpr int IMU_SIGN_PITCH = -1;
inline constexpr int IMU_SIGN_YAW = -1;

inline constexpr int CALIB_SAMPLES = 300;
inline constexpr bool DEFAULT_DEBUG_OVERLAY = false;
inline constexpr int IMU_BUFFER_SIZE = 1500; // target for log
inline constexpr int IMU_STORE_SIZE = 2500;  // ring buffer size

// Offset sweep
inline constexpr double OFFSET_CALIB_DURATION_MS = 8000.0;
inline constexpr double OFFSET_COARSE_RANGE_MS = 50.0;
inline constexpr double OFFSET_COARSE_STEP_MS = 0.5;
inline constexpr double OFFSET_FINE_RANGE_MS = 5.0;
inline constexpr double OFFSET_FINE_STEP_MS = 0.1;

// ======================== Enums ========================

enum class EisMode {
    LK = 0,
    GYRO = 1,
    HYBRID = 2
};

enum class OutputMode {
    BOTH = 0,
    RAW_ONLY = 1,
    CAM_ONLY = 2
};

enum class TsSource {
    SENSOR = 0,
    PTS = 1,
    ARRIVAL = 2
};

enum class TsSourcePref {
    AUTO = 0,
    SENSOR = 1,
    PTS = 2,
    ARRIVAL = 3
};

enum class GyroWarpMode {
    DELTA_DIRECT = 0,
    HIGHPASS = 1
};

enum class RunProfile {
    RUN = 0,
    CALIB = 1,
    DEBUG = 2
};

// ======================== String helpers ========================

inline const char* ts_source_str(TsSource s) {
    switch (s) {
    case TsSource::SENSOR: return "SENSOR";
    case TsSource::PTS: return "PTS";
    default: return "ARRIVAL";
    }
}

inline const char* mode_str(EisMode m) {
    switch (m) {
    case EisMode::LK: return "LK";
    case EisMode::GYRO: return "GYRO";
    default: return "HYBRID";
    }
}

inline const char* output_mode_str(OutputMode m) {
    switch (m) {
    case OutputMode::RAW_ONLY: return "RAW_ONLY";
    case OutputMode::CAM_ONLY: return "CAM_ONLY";
    default: return "BOTH";
    }
}

inline const char* ts_pref_str(TsSourcePref p) {
    switch (p) {
    case TsSourcePref::SENSOR: return "SENSOR";
    case TsSourcePref::PTS: return "PTS";
    case TsSourcePref::ARRIVAL: return "ARRIVAL";
    default: return "AUTO";
    }
}

inline const char* gyro_warp_str(GyroWarpMode m) {
    switch (m) {
    case GyroWarpMode::HIGHPASS: return "HIGHPASS";
    default: return "DELTA_DIRECT";
    }
}

inline const char* profile_str(RunProfile p) {
    switch (p) {
    case RunProfile::CALIB: return "CALIB";
    case RunProfile::DEBUG: return "DEBUG";
    default: return "RUN";
    }
}
