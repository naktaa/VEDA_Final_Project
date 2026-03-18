#pragma once

#include <opencv2/opencv.hpp>
#include <deque>
#include <mutex>
#include <vector>

#include "eis_common.hpp"

struct GyroSample {
    double t_ms = 0.0;            // CLOCK_MONOTONIC_RAW base ms
    cv::Vec3d w_rad = {0, 0, 0};  // rad/s (bias-corrected)
    cv::Vec3d raw = {0, 0, 0};    // raw (LSB) for debug
};

struct GyroRangeInfo {
    int used = 0;
    double min_ts = 0.0;
    double max_ts = 0.0;
    double t0 = 0.0;
    double t1 = 0.0;
};

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    bool valid() const { return fx > 0.0 && fy > 0.0; }
    cv::Mat K() const;
    cv::Mat Kinv() const;
    static CameraIntrinsics from_fov(int w, int h, double hfov_deg, double vfov_deg);
};

struct EISConfig {
    // Common correction gains/clamps for both delta_direct and highpass.
    double gain_roll = 1.0;
    double gain_pitch = 1.0;
    double gain_yaw = 1.0;
    double max_roll_rad = 5.0 * CV_PI / 180.0;
    double max_pitch_rad = 4.0 * CV_PI / 180.0;
    double max_yaw_rad = 8.0 * CV_PI / 180.0;

    // High-pass specific parameters.
    double hp_lpf_alpha = 0.90;
    double hp_gain_roll = 1.0;
    double hp_gain_pitch = 1.0;
    double hp_gain_yaw = 1.0;

    // Protection against aggressive correction on large frame deltas.
    double large_rot_thresh_deg = 3.5;
    double large_rot_gain_scale = 0.40;

    double crop_percent = 20.0;
    bool enable_crop = true;
};

class GyroBuffer {
public:
    explicit GyroBuffer(size_t max_samples = 1500);
    void push(const GyroSample& s);
    bool get_range(double t0_ms, double t1_ms, std::vector<GyroSample>& out, GyroRangeInfo* info = nullptr) const;
    size_t size() const;
private:
    mutable std::mutex mtx_;
    std::deque<GyroSample> buf_;
    size_t max_samples_;
};

struct Quaternion {
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
    static Quaternion identity();
    Quaternion normalized() const;
    void normalize();
    Quaternion conjugate() const;
    Quaternion operator*(const Quaternion& r) const;
    static Quaternion from_omega_dt(const cv::Vec3d& w_rad, double dt_sec);
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, double t);
    cv::Mat toRotationMatrix() const;
};

class GyroIntegrator {
public:
    GyroIntegrator();
    void reset();
    void set_bias(const cv::Vec3d& bias);
    bool integrate_to(double t_ms, const GyroBuffer& buf, Quaternion& out_q, GyroRangeInfo* info = nullptr);
    bool initialized() const { return initialized_; }
private:
    Quaternion q_;
    double last_t_ms_ = 0.0;
    bool initialized_ = false;
    cv::Vec3d bias_ = {0, 0, 0};
};

class EISWarpCalculator {
public:
    explicit EISWarpCalculator(const CameraIntrinsics& K);
    bool valid() const { return valid_; }
    cv::Mat homography_from_quat(const Quaternion& q_corr) const;
    double fx() const { return fx_; }
    double fy() const { return fy_; }
private:
    bool valid_ = false;
    cv::Mat K_;
    cv::Mat Kinv_;
    double fx_ = 0.0;
    double fy_ = 0.0;
};

enum class GyroProcessStatus {
    OK = 0,
    BYPASS_NO_BUFFER = 1,
    BYPASS_INVALID_INTRINSICS = 2,
    BYPASS_NON_MONOTONIC_TS = 3,
    BYPASS_INIT = 4,
    BYPASS_NO_IMU_SAMPLES = 5,
    BYPASS_NO_PREV_FRAME = 6,
};

inline const char* gyro_process_status_str(GyroProcessStatus s) {
    switch (s) {
    case GyroProcessStatus::OK: return "OK";
    case GyroProcessStatus::BYPASS_NO_BUFFER: return "NO_BUFFER";
    case GyroProcessStatus::BYPASS_INVALID_INTRINSICS: return "INVALID_INTRINSICS";
    case GyroProcessStatus::BYPASS_NON_MONOTONIC_TS: return "NON_MONOTONIC_TS";
    case GyroProcessStatus::BYPASS_INIT: return "INIT";
    case GyroProcessStatus::BYPASS_NO_IMU_SAMPLES: return "NO_IMU_SAMPLES";
    default: return "NO_PREV_FRAME";
    }
}

struct GyroEISDebug {
    GyroProcessStatus status = GyroProcessStatus::OK;
    GyroWarpMode mode = GyroWarpMode::DELTA_DIRECT;

    double frame_time_ms = 0.0;
    double target_imu_time_ms = 0.0;
    double prev_target_imu_time_ms = 0.0;

    GyroRangeInfo range;
    Quaternion q_phys;
    Quaternion q_delta;
    Quaternion q_corr;

    cv::Vec3d delta_raw_rad = {0, 0, 0};
    cv::Vec3d delta_lp_rad = {0, 0, 0};
    cv::Vec3d delta_hp_rad = {0, 0, 0};
    cv::Vec3d corr_final_rad = {0, 0, 0};

    double delta_angle_deg = 0.0;
    bool large_rot_detected = false;
    bool gain_scaled = false;
    double gain_scale_applied = 1.0;
    bool clamp_applied = false;

    double pixel_displacement_est = 0.0;
    double required_scale = 1.0;
    double required_crop_percent = 0.0;
};

class GyroEIS {
public:
    GyroEIS(const CameraIntrinsics& intr, const EISConfig& cfg, GyroBuffer* buf);
    void reset();
    void set_config(const EISConfig& cfg);
    void set_warp_mode(GyroWarpMode mode);
    bool process(const cv::Mat& frame, double frame_time_ms, double imu_offset_ms,
                 cv::Mat& out, GyroEISDebug* dbg = nullptr);
private:
    GyroBuffer* buf_ = nullptr;
    EISConfig cfg_;
    GyroWarpMode mode_ = GyroWarpMode::DELTA_DIRECT;
    GyroIntegrator integrator_;
    EISWarpCalculator warp_;

    Quaternion prev_phys_;
    bool prev_phys_ok_ = false;
    cv::Vec3d hp_lp_euler_rad_ = {0, 0, 0};
    bool hp_initialized_ = false;
    double prev_target_ms_ = 0.0;
    bool prev_target_ok_ = false;
};
