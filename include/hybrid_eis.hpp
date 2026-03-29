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
    bool gyro_gate_valid = false;
    double yaw_gate_dps = 0.0;
    int gyro_gate_samples = 0;
    double gyro_target_time_ms = 0.0;
    double gyro_latest_sample_time_ms = 0.0;
    double gyro_latest_lag_ms = 0.0;
    int gyro_range_used = 0;
    double gyro_range_min_ms = 0.0;
    double gyro_range_max_ms = 0.0;
    bool gyro_covers_start = false;
    bool gyro_covers_end = false;
    double crop_required_percent = 0.0;
    double clamp_scale = 1.0;
    double visual_anchor_rad = 0.0;
    cv::Vec3d gyro_delta_raw_rad = {0.0, 0.0, 0.0};
    cv::Vec3d gyro_delta_hp_rad = {0.0, 0.0, 0.0};
    cv::Vec3d applied_rotation_rad = {0.0, 0.0, 0.0};
    double applied_tx = 0.0;
    double applied_ty = 0.0;
    bool rs_active = false;
    int rs_band_used = 0;
    int rs_band_total = 0;
    double rs_crop_required_percent = 0.0;
};

class HybridEisProcessor {
public:
    HybridEisProcessor(const AppConfig& config, const GyroBuffer* gyro_buffer);

    void update_config(const AppConfig& config);
    void reset();
    bool process(const CapturedFrame& frame, cv::Mat& stabilized, HybridEisDebugInfo* debug = nullptr);

private:
    struct KalmanState {
        double x = 0.0;
        double P = 1.0;
        double Q = 0.004;
        double R = 0.5;
        double sum = 0.0;

        void init(double q, double r) {
            x = 0.0;
            P = 1.0;
            Q = q;
            R = r;
            sum = 0.0;
        }

        void update(double measurement) {
            sum += measurement;
            const double x_pred = x;
            const double P_pred = P + Q;
            const double K = P_pred / (P_pred + R);
            x = x_pred + K * (sum - x_pred);
            P = (1.0 - K) * P_pred;
        }

        double diff() const {
            return x - sum;
        }
    };

    AppConfig config_;
    const GyroBuffer* gyro_buffer_ = nullptr;
    LkTracker lk_tracker_;
    CameraIntrinsics intrinsics_;
    EISWarpCalculator warp_;
    GyroIntegrator integrator_;

    cv::Mat prev_frame_;
    bool prev_frame_ok_ = false;
    double prev_frame_time_ms_ = 0.0;
    int64_t prev_sensor_ts_ns_ = 0;
    int prev_exposure_us_ = 0;

    Quaternion prev_phys_;
    bool prev_phys_ok_ = false;
    double prev_target_ms_ = 0.0;
    bool prev_target_ok_ = false;

    cv::Vec3d hp_lp_delta_rad_ = {0.0, 0.0, 0.0};
    bool hp_initialized_ = false;

    KalmanState kf_theta_;
    KalmanState kf_tx_;
    KalmanState kf_ty_;
    int lk_frame_count_ = 0;
    int weak_lk_frames_ = 0;

    HybridState state_ = HybridState::STABILIZE;
    int turn_enter_count_ = 0;
    int turn_exit_count_ = 0;
    int recover_frames_left_ = 0;
    int gate_invalid_frames_ = 0;

    void update_state(bool gate_valid, double yaw_gate_dps);
    void reset_lk_stabilization();
};
