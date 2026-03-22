#pragma once

#include <opencv2/opencv.hpp>

#include "app_config.hpp"
#include "app_types.hpp"
#include "gyro_eis.hpp"
#include "lk_tracker.hpp"

struct HybridEisDebugInfo {
    HybridState state = HybridState::STABILIZE;
    bool lk_valid = false;
    double lk_confidence = 0.0;
    int lk_features = 0;
    int lk_valid_points = 0;
    int lk_inliers = 0;
    bool gyro_valid = false;
    double yaw_rate_dps = 0.0;
    double crop_required_percent = 0.0;
    double clamp_scale = 1.0;
    double visual_anchor_rad = 0.0;
    cv::Vec3d gyro_delta_raw_rad = {0.0, 0.0, 0.0};
    cv::Vec3d gyro_delta_hp_rad = {0.0, 0.0, 0.0};
    cv::Vec3d applied_rotation_rad = {0.0, 0.0, 0.0};
    double applied_tx = 0.0;
    double applied_ty = 0.0;
};

class HybridEisProcessor {
public:
    HybridEisProcessor(const AppConfig& config, const GyroBuffer* gyro_buffer);

    void update_config(const AppConfig& config);
    void reset();
    bool process(const CapturedFrame& frame, cv::Mat& stabilized, HybridEisDebugInfo* debug = nullptr);

private:
    AppConfig config_;
    const GyroBuffer* gyro_buffer_ = nullptr;
    LkTracker lk_tracker_;
    CameraIntrinsics intrinsics_;
    EISWarpCalculator warp_;
    GyroIntegrator integrator_;

    cv::Mat prev_frame_;
    bool prev_frame_ok_ = false;

    Quaternion prev_phys_;
    bool prev_phys_ok_ = false;
    double prev_target_ms_ = 0.0;
    bool prev_target_ok_ = false;

    cv::Vec3d hp_lp_delta_rad_ = {0.0, 0.0, 0.0};
    bool hp_initialized_ = false;

    double visual_anchor_rad_ = 0.0;
    bool visual_anchor_ok_ = false;
    double smooth_tx_ = 0.0;
    double smooth_ty_ = 0.0;
    bool smooth_translation_ok_ = false;

    HybridState state_ = HybridState::STABILIZE;
    int turn_enter_count_ = 0;
    int turn_exit_count_ = 0;
    int recover_frames_left_ = 0;

    void update_state(double yaw_rate_dps);
};
