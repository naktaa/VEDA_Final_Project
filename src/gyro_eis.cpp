#include "gyro_eis.hpp"

#include <algorithm>
#include <cmath>

namespace {

static cv::Vec3d quat_to_euler(const Quaternion& q) {
    const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch = 0.0;
    if (std::abs(sinp) >= 1.0) pitch = std::copysign(CV_PI / 2.0, sinp);
    else pitch = std::asin(sinp);

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    return cv::Vec3d(roll, pitch, yaw);
}

static Quaternion quat_from_euler(double roll, double pitch, double yaw) {
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.normalize();
    return q;
}

static double rotation_angle_deg(const Quaternion& q) {
    const double w = std::clamp(std::abs(q.w), 0.0, 1.0);
    return 2.0 * std::acos(w) * 180.0 / CV_PI;
}

static void analyze_homography(const cv::Mat& H,
                               int width,
                               int height,
                               double& out_required_scale,
                               double& out_required_crop,
                               double& out_pixel_disp_est) {
    std::vector<cv::Point2f> corners;
    corners.emplace_back(0.0f, 0.0f);
    corners.emplace_back((float)width, 0.0f);
    corners.emplace_back((float)width, (float)height);
    corners.emplace_back(0.0f, (float)height);

    std::vector<cv::Point2f> warped;
    cv::perspectiveTransform(corners, warped, H);

    double minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
    double max_disp = 0.0;
    for (int i = 0; i < 4; ++i) {
        const auto& p = warped[i];
        minx = std::min(minx, (double)p.x);
        maxx = std::max(maxx, (double)p.x);
        miny = std::min(miny, (double)p.y);
        maxy = std::max(maxy, (double)p.y);

        const cv::Point2f d = p - corners[i];
        max_disp = std::max(max_disp, std::sqrt((double)d.x * d.x + (double)d.y * d.y));
    }

    const double overlap_w = std::min((double)width, maxx) - std::max(0.0, minx);
    const double overlap_h = std::min((double)height, maxy) - std::max(0.0, miny);
    if (overlap_w <= 1.0 || overlap_h <= 1.0) {
        out_required_scale = 10.0;
    } else {
        const double scale_x = (double)width / overlap_w;
        const double scale_y = (double)height / overlap_h;
        out_required_scale = std::max(1.0, std::max(scale_x, scale_y));
    }

    out_required_crop = (1.0 - 1.0 / out_required_scale) * 100.0;
    out_pixel_disp_est = max_disp;
}

static cv::Vec3d clamp_correction(const cv::Vec3d& in,
                                  const EISConfig& cfg,
                                  bool& clamp_applied) {
    cv::Vec3d out = in;

    auto clamp_axis = [&](double v, double lim) {
        if (!std::isfinite(v)) {
            clamp_applied = true;
            return 0.0;
        }
        const double c = std::clamp(v, -lim, lim);
        if (std::abs(c - v) > 1e-12) clamp_applied = true;
        return c;
    };

    out[0] = clamp_axis(out[0], cfg.max_roll_rad);
    out[1] = clamp_axis(out[1], cfg.max_pitch_rad);
    out[2] = clamp_axis(out[2], cfg.max_yaw_rad);
    return out;
}

} // namespace

// ---------------- CameraIntrinsics ----------------

cv::Mat CameraIntrinsics::K() const {
    return (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                     0, fy, cy,
                                     0, 0, 1);
}

cv::Mat CameraIntrinsics::Kinv() const {
    cv::Mat k = K();
    return k.inv();
}

CameraIntrinsics CameraIntrinsics::from_fov(int w, int h, double hfov_deg, double vfov_deg) {
    CameraIntrinsics ci;
    const double hfov = hfov_deg * CV_PI / 180.0;
    const double vfov = vfov_deg * CV_PI / 180.0;
    ci.cx = w * 0.5;
    ci.cy = h * 0.5;
    ci.fx = (w * 0.5) / std::tan(hfov * 0.5);
    ci.fy = (h * 0.5) / std::tan(vfov * 0.5);
    return ci;
}

// ---------------- GyroBuffer ----------------

GyroBuffer::GyroBuffer(size_t max_samples) : max_samples_(max_samples) {}

void GyroBuffer::push(const GyroSample& s) {
    std::lock_guard<std::mutex> lk(mtx_);
    buf_.push_back(s);
    if (buf_.size() > max_samples_) buf_.pop_front();
}

size_t GyroBuffer::size() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return buf_.size();
}

bool GyroBuffer::get_range(double t0_ms, double t1_ms, std::vector<GyroSample>& out, GyroRangeInfo* info) const {
    out.clear();
    if (t1_ms < t0_ms) std::swap(t0_ms, t1_ms);

    std::lock_guard<std::mutex> lk(mtx_);
    if (buf_.empty()) return false;

    int start_idx = -1;
    for (size_t i = 0; i < buf_.size(); ++i) {
        if (buf_[i].t_ms >= t0_ms) {
            start_idx = (int)i;
            break;
        }
    }
    if (start_idx < 0) start_idx = (int)buf_.size() - 1;
    if (start_idx > 0) out.push_back(buf_[start_idx - 1]);

    for (size_t i = (size_t)start_idx; i < buf_.size(); ++i) {
        const auto& s = buf_[i];
        if (s.t_ms > t1_ms) {
            out.push_back(s); // include one sample after t1 for tail integration
            break;
        }
        out.push_back(s);
    }

    if (info) {
        info->t0 = t0_ms;
        info->t1 = t1_ms;
        info->used = (int)out.size();
        if (!out.empty()) {
            info->min_ts = out.front().t_ms;
            info->max_ts = out.back().t_ms;
        } else {
            info->min_ts = info->max_ts = 0.0;
        }
    }
    return !out.empty();
}

// ---------------- Quaternion ----------------

Quaternion Quaternion::identity() { return Quaternion(); }

Quaternion Quaternion::normalized() const {
    Quaternion q = *this;
    q.normalize();
    return q;
}

void Quaternion::normalize() {
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n <= 1e-12) {
        w = 1.0;
        x = y = z = 0.0;
        return;
    }
    w /= n;
    x /= n;
    y /= n;
    z /= n;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion{w, -x, -y, -z};
}

Quaternion Quaternion::operator*(const Quaternion& r) const {
    Quaternion q;
    q.w = w * r.w - x * r.x - y * r.y - z * r.z;
    q.x = w * r.x + x * r.w + y * r.z - z * r.y;
    q.y = w * r.y - x * r.z + y * r.w + z * r.x;
    q.z = w * r.z + x * r.y - y * r.x + z * r.w;
    return q;
}

Quaternion Quaternion::from_omega_dt(const cv::Vec3d& w_rad, double dt_sec) {
    const double wx = w_rad[0], wy = w_rad[1], wz = w_rad[2];
    const double norm = std::sqrt(wx * wx + wy * wy + wz * wz);
    const double ang = norm * dt_sec;

    if (ang < 1e-6) {
        return Quaternion{1.0, 0.5 * wx * dt_sec, 0.5 * wy * dt_sec, 0.5 * wz * dt_sec};
    }

    const double half = ang * 0.5;
    const double s = std::sin(half);
    const double inv = 1.0 / norm;
    return Quaternion{std::cos(half), wx * inv * s, wy * inv * s, wz * inv * s};
}

Quaternion Quaternion::slerp(const Quaternion& a, const Quaternion& b, double t) {
    double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    Quaternion bb = b;
    if (dot < 0.0) {
        dot = -dot;
        bb = Quaternion{-b.w, -b.x, -b.y, -b.z};
    }
    if (dot > 0.9995) {
        Quaternion q{a.w + t * (bb.w - a.w),
                     a.x + t * (bb.x - a.x),
                     a.y + t * (bb.y - a.y),
                     a.z + t * (bb.z - a.z)};
        q.normalize();
        return q;
    }
    const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
    const double s0 = std::sin((1.0 - t) * theta);
    const double s1 = std::sin(t * theta);
    const double s = std::sin(theta);
    Quaternion q;
    q.w = (a.w * s0 + bb.w * s1) / s;
    q.x = (a.x * s0 + bb.x * s1) / s;
    q.y = (a.y * s0 + bb.y * s1) / s;
    q.z = (a.z * s0 + bb.z * s1) / s;
    return q;
}

cv::Mat Quaternion::toRotationMatrix() const {
    const double ww = w * w;
    const double xx = x * x;
    const double yy = y * y;
    const double zz = z * z;
    const double wx = w * x;
    const double wy = w * y;
    const double wz = w * z;
    const double xy = x * y;
    const double xz = x * z;
    const double yz = y * z;

    return (cv::Mat_<double>(3, 3) <<
            ww + xx - yy - zz, 2 * (xy - wz), 2 * (xz + wy),
            2 * (xy + wz), ww - xx + yy - zz, 2 * (yz - wx),
            2 * (xz - wy), 2 * (yz + wx), ww - xx - yy + zz);
}

// ---------------- GyroIntegrator ----------------

GyroIntegrator::GyroIntegrator() { reset(); }

void GyroIntegrator::reset() {
    q_ = Quaternion::identity();
    last_t_ms_ = 0.0;
    initialized_ = false;
}

void GyroIntegrator::set_bias(const cv::Vec3d& bias) {
    bias_ = bias;
}

bool GyroIntegrator::integrate_to(double t_ms, const GyroBuffer& buf, Quaternion& out_q, GyroRangeInfo* info) {
    if (!initialized_) {
        last_t_ms_ = t_ms;
        initialized_ = true;
        out_q = q_;
        if (info) {
            info->used = 0;
            info->t0 = t_ms;
            info->t1 = t_ms;
        }
        return false;
    }

    if (t_ms <= last_t_ms_) {
        out_q = q_;
        if (info) {
            info->used = 0;
            info->t0 = t_ms;
            info->t1 = t_ms;
        }
        return false;
    }

    std::vector<GyroSample> samples;
    GyroRangeInfo tmp;
    if (!buf.get_range(last_t_ms_, t_ms, samples, &tmp)) {
        out_q = q_;
        if (info) *info = tmp;
        return false;
    }

    double t_prev = last_t_ms_;
    cv::Vec3d w_prev = samples.front().w_rad - bias_;
    int used = 0;

    for (const auto& s : samples) {
        const double t = std::clamp(s.t_ms, last_t_ms_, t_ms);
        if (t <= t_prev) {
            w_prev = s.w_rad - bias_;
            continue;
        }
        const double dt = (t - t_prev) / 1000.0;
        q_ = q_ * Quaternion::from_omega_dt(w_prev, dt);
        t_prev = t;
        w_prev = s.w_rad - bias_;
        used++;
    }

    if (t_prev < t_ms) {
        const double dt = (t_ms - t_prev) / 1000.0;
        q_ = q_ * Quaternion::from_omega_dt(w_prev, dt);
    }

    q_.normalize();
    last_t_ms_ = t_ms;
    out_q = q_;
    if (info) {
        *info = tmp;
        info->used = used;
    }
    return true;
}

// ---------------- EISWarpCalculator ----------------

EISWarpCalculator::EISWarpCalculator(const CameraIntrinsics& K) {
    if (K.valid()) {
        K_ = K.K();
        Kinv_ = K.Kinv();
        fx_ = K.fx;
        fy_ = K.fy;
        valid_ = true;
    }
}

cv::Mat EISWarpCalculator::homography_from_quat(const Quaternion& q_corr) const {
    const cv::Mat R = q_corr.toRotationMatrix();
    return K_ * R * Kinv_;
}

// ---------------- GyroEIS ----------------

GyroEIS::GyroEIS(const CameraIntrinsics& intr, const EISConfig& cfg, GyroBuffer* buf)
    : buf_(buf), cfg_(cfg), warp_(intr) {}

void GyroEIS::reset() {
    integrator_.reset();
    prev_phys_ = Quaternion::identity();
    prev_phys_ok_ = false;
    hp_lp_euler_rad_ = cv::Vec3d(0, 0, 0);
    hp_initialized_ = false;
    prev_target_ms_ = 0.0;
    prev_target_ok_ = false;
}

void GyroEIS::set_config(const EISConfig& cfg) {
    cfg_ = cfg;
}

void GyroEIS::set_warp_mode(GyroWarpMode mode) {
    if (mode_ == mode) return;
    mode_ = mode;

    // Force one-frame warm-up after mode change to avoid mixed states.
    prev_phys_ok_ = false;
    hp_lp_euler_rad_ = cv::Vec3d(0, 0, 0);
    hp_initialized_ = false;
}

bool GyroEIS::process(const cv::Mat& frame,
                      double frame_time_ms,
                      double imu_offset_ms,
                      cv::Mat& out,
                      GyroEISDebug* dbg) {
    if (dbg) {
        *dbg = GyroEISDebug{};
        dbg->mode = mode_;
        dbg->frame_time_ms = frame_time_ms;
    }

    if (!buf_) {
        out = frame.clone();
        if (dbg) dbg->status = GyroProcessStatus::BYPASS_NO_BUFFER;
        return false;
    }
    if (!warp_.valid()) {
        out = frame.clone();
        if (dbg) dbg->status = GyroProcessStatus::BYPASS_INVALID_INTRINSICS;
        return false;
    }

    const double target_ms = frame_time_ms + imu_offset_ms;
    if (dbg) {
        dbg->target_imu_time_ms = target_ms;
        if (prev_target_ok_) dbg->prev_target_imu_time_ms = prev_target_ms_;
    }

    if (prev_target_ok_ && target_ms <= prev_target_ms_) {
        out = frame.clone();
        if (dbg) dbg->status = GyroProcessStatus::BYPASS_NON_MONOTONIC_TS;
        return false;
    }

    const bool integrator_was_initialized = integrator_.initialized();

    Quaternion q_phys;
    GyroRangeInfo info;
    const bool ok = integrator_.integrate_to(target_ms, *buf_, q_phys, &info);
    prev_target_ms_ = target_ms;
    prev_target_ok_ = true;

    if (!ok) {
        out = frame.clone();
        prev_phys_ok_ = false;
        hp_initialized_ = false;
        if (dbg) {
            dbg->status = integrator_was_initialized
                ? GyroProcessStatus::BYPASS_NO_IMU_SAMPLES
                : GyroProcessStatus::BYPASS_INIT;
            dbg->q_phys = q_phys;
            dbg->range = info;
        }
        return false;
    }

    if (!prev_phys_ok_) {
        prev_phys_ = q_phys;
        prev_phys_ok_ = true;
        out = frame.clone();
        if (dbg) {
            dbg->status = GyroProcessStatus::BYPASS_NO_PREV_FRAME;
            dbg->q_phys = q_phys;
            dbg->range = info;
        }
        return false;
    }

    Quaternion q_delta = prev_phys_.conjugate() * q_phys;
    q_delta.normalize();

    const cv::Vec3d delta_raw = quat_to_euler(q_delta);
    cv::Vec3d delta_lp = hp_lp_euler_rad_;
    cv::Vec3d delta_hp(0, 0, 0);

    cv::Vec3d corr(0, 0, 0);
    if (mode_ == GyroWarpMode::DELTA_DIRECT) {
        corr[0] = -delta_raw[0] * cfg_.gain_roll;
        corr[1] = -delta_raw[1] * cfg_.gain_pitch;
        corr[2] = -delta_raw[2] * cfg_.gain_yaw;
    } else {
        const double alpha = std::clamp(cfg_.hp_lpf_alpha, 0.0, 1.0);
        if (!hp_initialized_) {
            hp_lp_euler_rad_ = delta_raw;
            hp_initialized_ = true;
        } else {
            hp_lp_euler_rad_ = alpha * hp_lp_euler_rad_ + (1.0 - alpha) * delta_raw;
        }
        delta_lp = hp_lp_euler_rad_;
        delta_hp = delta_raw - delta_lp;

        corr[0] = -delta_hp[0] * cfg_.gain_roll * cfg_.hp_gain_roll;
        corr[1] = -delta_hp[1] * cfg_.gain_pitch * cfg_.hp_gain_pitch;
        corr[2] = -delta_hp[2] * cfg_.gain_yaw * cfg_.hp_gain_yaw;
    }

    const double delta_angle_deg = rotation_angle_deg(q_delta);
    const bool large_rot = delta_angle_deg > cfg_.large_rot_thresh_deg;
    double gain_scale = 1.0;
    if (large_rot) {
        gain_scale = std::clamp(cfg_.large_rot_gain_scale, 0.0, 1.0);
        corr *= gain_scale;
    }

    bool clamp_applied = false;
    corr = clamp_correction(corr, cfg_, clamp_applied);

    const Quaternion q_corr = quat_from_euler(corr[0], corr[1], corr[2]);
    const cv::Mat H = warp_.homography_from_quat(q_corr);

    cv::warpPerspective(frame, out, H, frame.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    if (cfg_.enable_crop && cfg_.crop_percent > 0.0) {
        const int cw = (int)(out.cols * cfg_.crop_percent / 200.0);
        const int ch = (int)(out.rows * cfg_.crop_percent / 200.0);
        cv::Rect roi(cw, ch, out.cols - 2 * cw, out.rows - 2 * ch);
        cv::Mat cropped;
        cv::resize(out(roi), cropped, out.size(), 0, 0, cv::INTER_LINEAR);
        out = cropped;
    }

    prev_phys_ = q_phys;
    prev_phys_ok_ = true;

    if (dbg) {
        dbg->status = GyroProcessStatus::OK;
        dbg->range = info;
        dbg->q_phys = q_phys;
        dbg->q_delta = q_delta;
        dbg->q_corr = q_corr;
        dbg->delta_raw_rad = delta_raw;
        dbg->delta_lp_rad = delta_lp;
        dbg->delta_hp_rad = delta_hp;
        dbg->corr_final_rad = corr;
        dbg->delta_angle_deg = delta_angle_deg;
        dbg->large_rot_detected = large_rot;
        dbg->gain_scaled = large_rot;
        dbg->gain_scale_applied = gain_scale;
        dbg->clamp_applied = clamp_applied;

        analyze_homography(H,
                          frame.cols,
                          frame.rows,
                          dbg->required_scale,
                          dbg->required_crop_percent,
                          dbg->pixel_displacement_est);
    }

    return true;
}
