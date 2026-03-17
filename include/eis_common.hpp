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

// Gyro smoothing / clamp (gyro-first path)
inline constexpr double SMOOTH_ALPHA = 0.985;
inline constexpr double ROLL_GAIN = 0.9;
inline constexpr double PITCH_GAIN = 0.8;
inline constexpr double YAW_GAIN = 0.9;
inline constexpr double MAX_YAW_RAD = 10.0 * CV_PI / 180.0;
inline constexpr double MAX_ROLL_RAD = 6.0 * CV_PI / 180.0;
inline constexpr double MAX_PITCH_RAD = 5.0 * CV_PI / 180.0;

// LK (offset sweep/debug only)
inline constexpr int LK_MAX_FEATURES = 200;
inline constexpr double LK_QUALITY = 0.01;
inline constexpr double LK_MIN_DIST = 30.0;
inline constexpr int LK_TRANS_MIN_FEATURES = 12;
inline constexpr double LK_TRANS_ALPHA = 0.90;
inline constexpr double LK_TRANS_MAX_CORR_PX = 30.0;
inline constexpr int LK_TRANS_EVERY_N = 2; // run LK translation every N frames

// Crop
inline constexpr double FIXED_CROP_PERCENT = 20.0;

// Test environment: camera is vertically flipped (both axes)
inline constexpr bool FLIP_VERTICAL = true;

// Camera FOV
inline constexpr double HFOV_DEG = 62.2;
inline constexpr double VFOV_DEG = 48.8;

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
    JITTER = 0,
    DELTA = 1
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
    case GyroWarpMode::DELTA: return "DELTA";
    default: return "JITTER";
    }
}
