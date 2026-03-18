#pragma once

#include <opencv2/opencv.hpp>
#include <deque>
#include <mutex>
#include <vector>

#include "eis_common.hpp"

struct GyroSample {
    double t_ms = 0.0;        // CLOCK_MONOTONIC_RAW 기준 ms
    cv::Vec3d w_rad = {0, 0, 0}; // rad/s (bias 보정 후)
    cv::Vec3d raw = {0, 0, 0};   // raw (LSB) 디버그용
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
    double smooth_alpha = 0.97; // 0~1 (높을수록 더 부드럽게)
    double max_roll_rad = 4.0 * CV_PI / 180.0;
    double max_pitch_rad = 3.0 * CV_PI / 180.0;
    double max_yaw_rad = 8.0 * CV_PI / 180.0;
    double roll_gain = 0.7;
    double pitch_gain = 0.6;
    double yaw_gain = 0.8;
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

class OrientationFilter {
public:
    explicit OrientationFilter(double alpha = 0.97);
    void reset();
    void set_alpha(double alpha);
    Quaternion update(const Quaternion& q_phys);
    bool initialized() const { return initialized_; }
private:
    double alpha_;
    bool initialized_ = false;
    Quaternion q_virtual_;
};

class EISWarpCalculator {
public:
    explicit EISWarpCalculator(const CameraIntrinsics& K);
    bool valid() const { return valid_; }
    cv::Mat homography_from_quat(const Quaternion& q_corr) const;
private:
    bool valid_ = false;
    cv::Mat K_;
    cv::Mat Kinv_;
};

struct GyroEISDebug {
    Quaternion q_phys;
    Quaternion q_virtual;
    Quaternion q_corr;
    GyroRangeInfo range;
    double crop_percent = 0.0;
    cv::Vec3d e_corr_raw = {0,0,0};
    cv::Vec3d e_corr_clamped = {0,0,0};
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
    GyroWarpMode mode_ = GyroWarpMode::JITTER;
    GyroIntegrator integrator_;
    OrientationFilter filter_;
    EISWarpCalculator warp_;
    Quaternion prev_phys_;
    bool prev_phys_ok_ = false;
};
