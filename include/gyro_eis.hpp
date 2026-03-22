#pragma once

#include <deque>
#include <mutex>
#include <vector>

#include <opencv2/core.hpp>

#include "app_config.hpp"
#include "app_types.hpp"

struct GyroRangeInfo {
    int used = 0;
    double min_ts = 0.0;
    double max_ts = 0.0;
    double t0 = 0.0;
    double t1 = 0.0;
    bool covers_start = false;
    bool covers_end = false;
};

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    bool valid() const { return fx > 0.0 && fy > 0.0; }
    cv::Mat K() const;
    cv::Mat Kinv() const;
    static CameraIntrinsics from_fov(int width, int height, double hfov_deg, double vfov_deg);
};

class GyroBuffer {
public:
    explicit GyroBuffer(size_t max_samples = 2500);

    void push(const ImuSample& sample);
    bool get_range(double t0_ms, double t1_ms, std::vector<ImuSample>& out, GyroRangeInfo* info = nullptr) const;
    bool latest(ImuSample& out) const;
    size_t size() const;

private:
    mutable std::mutex mutex_;
    std::deque<ImuSample> buffer_;
    size_t max_samples_ = 0;
};

struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    static Quaternion identity();
    Quaternion normalized() const;
    void normalize();
    Quaternion conjugate() const;
    Quaternion operator*(const Quaternion& rhs) const;
    static Quaternion from_omega_dt(const cv::Vec3d& omega_rad_s, double dt_sec);
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, double t);
    cv::Mat to_rotation_matrix() const;
};

class GyroIntegrator {
public:
    GyroIntegrator();

    void reset();
    void set_bias(const cv::Vec3d& bias_rad_s);
    bool integrate_to(double t_ms, const GyroBuffer& buffer, Quaternion& out_q, GyroRangeInfo* info = nullptr);
    bool initialized() const { return initialized_; }

private:
    Quaternion q_;
    double last_t_ms_ = 0.0;
    bool initialized_ = false;
    cv::Vec3d bias_ = {0.0, 0.0, 0.0};
};

class EISWarpCalculator {
public:
    explicit EISWarpCalculator(const CameraIntrinsics& intrinsics);

    bool valid() const { return valid_; }
    cv::Mat homography_from_quat(const Quaternion& q_corr) const;

private:
    bool valid_ = false;
    cv::Mat K_;
    cv::Mat Kinv_;
};

cv::Vec3d quat_to_euler(const Quaternion& q);
Quaternion quat_from_euler(double roll_rad, double pitch_rad, double yaw_rad);
double quaternion_angle_deg(const Quaternion& q);
cv::Mat translation_homography(double tx, double ty);
cv::Vec3d clamp_rotation_correction(const cv::Vec3d& in_rad, const EisRuntimeConfig& cfg, bool* clamp_applied = nullptr);
