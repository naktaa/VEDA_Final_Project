#pragma once

#include <cstdint>

#include <opencv2/core.hpp>

struct CapturedFrame {
    cv::Mat image;
    double frame_time_ms = 0.0;
    int64_t sensor_ts_ns = 0;
    int exposure_us = 0;
    int frame_duration_us = 0;
    uint64_t frame_index = 0;
};

struct ImuSample {
    double sample_time_ms = 0.0;
    cv::Vec3d gyro_rad_s = {0.0, 0.0, 0.0};
    cv::Vec3d raw_counts = {0.0, 0.0, 0.0};
};

struct LkMotionEstimate {
    bool valid = false;
    double dx = 0.0;
    double dy = 0.0;
    double da = 0.0;
    double sx = 1.0;
    double sy = 1.0;
    int features = 0;
    int valid_points = 0;
    int inliers = 0;
    double confidence = 0.0;
};

enum class HybridState {
    STABILIZE = 0,
    TURN_FOLLOW = 1,
    RECOVER = 2,
};

inline const char* hybrid_state_str(HybridState state) {
    switch (state) {
    case HybridState::STABILIZE:
        return "STABILIZE";
    case HybridState::TURN_FOLLOW:
        return "TURN_FOLLOW";
    case HybridState::RECOVER:
        return "RECOVER";
    default:
        return "UNKNOWN";
    }
}
