#include "gyro_eis.hpp"

#include <algorithm>
#include <cmath>

static cv::Vec3d quat_to_euler(const Quaternion& q) {
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch = 0.0;
    if (std::abs(sinp) >= 1.0) pitch = std::copysign(CV_PI / 2.0, sinp);
    else pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return cv::Vec3d(roll, pitch, yaw);
}

static Quaternion quat_from_euler(double roll, double pitch, double yaw) {
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.normalize();
    return q;
}

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
    double hfov = hfov_deg * CV_PI / 180.0;
    double vfov = vfov_deg * CV_PI / 180.0;
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
        if (buf_[i].t_ms >= t0_ms) { start_idx = (int)i; break; }
    }
    if (start_idx < 0) start_idx = (int)buf_.size() - 1;
    if (start_idx > 0) out.push_back(buf_[start_idx - 1]);

    for (size_t i = (size_t)start_idx; i < buf_.size(); ++i) {
        const auto& s = buf_[i];
        if (s.t_ms > t1_ms) {
            out.push_back(s); // include one after t1 for tail integration
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
    double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n <= 1e-12) { w = 1.0; x = y = z = 0.0; return; }
    w /= n; x /= n; y /= n; z /= n;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion{w, -x, -y, -z};
}

Quaternion Quaternion::operator*(const Quaternion& r) const {
    Quaternion q;
    q.w = w*r.w - x*r.x - y*r.y - z*r.z;
    q.x = w*r.x + x*r.w + y*r.z - z*r.y;
    q.y = w*r.y - x*r.z + y*r.w + z*r.x;
    q.z = w*r.z + x*r.y - y*r.x + z*r.w;
    return q;
}

Quaternion Quaternion::from_omega_dt(const cv::Vec3d& w_rad, double dt_sec) {
    double wx = w_rad[0], wy = w_rad[1], wz = w_rad[2];
    double ang = std::sqrt(wx*wx + wy*wy + wz*wz) * dt_sec;
    if (ang < 1e-6) {
        return Quaternion{1.0, 0.5*wx*dt_sec, 0.5*wy*dt_sec, 0.5*wz*dt_sec};
    }
    double half = ang * 0.5;
    double s = std::sin(half);
    double inv = 1.0 / std::sqrt(wx*wx + wy*wy + wz*wz);
    return Quaternion{std::cos(half), wx*inv*s, wy*inv*s, wz*inv*s};
}

Quaternion Quaternion::slerp(const Quaternion& a, const Quaternion& b, double t) {
    double dot = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    Quaternion bb = b;
    if (dot < 0.0) { dot = -dot; bb = Quaternion{-b.w, -b.x, -b.y, -b.z}; }
    if (dot > 0.9995) {
        Quaternion q{a.w + t*(bb.w-a.w), a.x + t*(bb.x-a.x), a.y + t*(bb.y-a.y), a.z + t*(bb.z-a.z)};
        q.normalize();
        return q;
    }
    double theta = std::acos(std::clamp(dot, -1.0, 1.0));
    double s0 = std::sin((1.0 - t) * theta);
    double s1 = std::sin(t * theta);
    double s = std::sin(theta);
    Quaternion q;
    q.w = (a.w*s0 + bb.w*s1) / s;
    q.x = (a.x*s0 + bb.x*s1) / s;
    q.y = (a.y*s0 + bb.y*s1) / s;
    q.z = (a.z*s0 + bb.z*s1) / s;
    return q;
}

cv::Mat Quaternion::toRotationMatrix() const {
    double ww = w*w, xx = x*x, yy = y*y, zz = z*z;
    double wx = w*x, wy = w*y, wz = w*z;
    double xy = x*y, xz = x*z, yz = y*z;
    cv::Mat R = (cv::Mat_<double>(3,3) <<
        ww+xx-yy-zz, 2*(xy-wz),     2*(xz+wy),
        2*(xy+wz),     ww-xx+yy-zz, 2*(yz-wx),
        2*(xz-wy),     2*(yz+wx),   ww-xx-yy+zz);
    return R;
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
        if (info) { info->used = 0; info->t0 = t_ms; info->t1 = t_ms; }
        return false;
    }
    if (t_ms <= last_t_ms_) {
        out_q = q_;
        if (info) { info->used = 0; info->t0 = t_ms; info->t1 = t_ms; }
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
        double t = std::clamp(s.t_ms, last_t_ms_, t_ms);
        if (t <= t_prev) { w_prev = s.w_rad - bias_; continue; }
        double dt = (t - t_prev) / 1000.0;
        q_ = q_ * Quaternion::from_omega_dt(w_prev, dt);
        t_prev = t;
        w_prev = s.w_rad - bias_;
        used++;
    }
    if (t_prev < t_ms) {
        double dt = (t_ms - t_prev) / 1000.0;
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

// ---------------- OrientationFilter ----------------

OrientationFilter::OrientationFilter(double alpha) : alpha_(alpha) {}

void OrientationFilter::reset() { initialized_ = false; q_virtual_ = Quaternion::identity(); }

void OrientationFilter::set_alpha(double alpha) { alpha_ = alpha; }

Quaternion OrientationFilter::update(const Quaternion& q_phys) {
    if (!initialized_) {
        q_virtual_ = q_phys;
        initialized_ = true;
        return q_virtual_;
    }
    double t = 1.0 - alpha_;
    q_virtual_ = Quaternion::slerp(q_virtual_, q_phys, t);
    return q_virtual_;
}

// ---------------- EISWarpCalculator ----------------

EISWarpCalculator::EISWarpCalculator(const CameraIntrinsics& K) {
    if (K.valid()) {
        K_ = K.K();
        Kinv_ = K.Kinv();
        valid_ = true;
    }
}

cv::Mat EISWarpCalculator::homography_from_quat(const Quaternion& q_corr) const {
    cv::Mat R = q_corr.toRotationMatrix();
    return K_ * R * Kinv_;
}

// ---------------- GyroEIS ----------------

GyroEIS::GyroEIS(const CameraIntrinsics& intr, const EISConfig& cfg, GyroBuffer* buf)
    : buf_(buf), cfg_(cfg), filter_(cfg.smooth_alpha), warp_(intr) {}

void GyroEIS::reset() {
    integrator_.reset();
    filter_.reset();
    prev_phys_ = Quaternion::identity();
    prev_phys_ok_ = false;
}

void GyroEIS::set_config(const EISConfig& cfg) {
    cfg_ = cfg;
    filter_.set_alpha(cfg_.smooth_alpha);
}

void GyroEIS::set_warp_mode(GyroWarpMode mode) { mode_ = mode; }

bool GyroEIS::process(const cv::Mat& frame, double frame_time_ms, double imu_offset_ms,
                      cv::Mat& out, GyroEISDebug* dbg) {
    if (!buf_ || !warp_.valid()) {
        out = frame.clone();
        return false;
    }
    double target_ms = frame_time_ms + imu_offset_ms;
    Quaternion q_phys;
    GyroRangeInfo info;
    bool ok = integrator_.integrate_to(target_ms, *buf_, q_phys, &info);
    if (!ok) {
        out = frame.clone();
        if (dbg) { dbg->q_phys = q_phys; dbg->range = info; }
        return false;
    }

    Quaternion q_virtual;
    if (mode_ == GyroWarpMode::DELTA) {
        if (!prev_phys_ok_) {
            prev_phys_ = q_phys;
            prev_phys_ok_ = true;
            out = frame.clone();
            if (dbg) { dbg->q_phys = q_phys; dbg->q_virtual = q_phys; dbg->range = info; }
            return false;
        }
        q_virtual = prev_phys_;
    } else {
        q_virtual = filter_.update(q_phys);
    }

    Quaternion q_corr = q_virtual * q_phys.conjugate();

    // clamp correction (prevent 180deg flip + slow recovery on large turns)
    cv::Vec3d e = quat_to_euler(q_corr);
    e[0] = std::clamp(e[0] * cfg_.roll_gain, -cfg_.max_roll_rad, cfg_.max_roll_rad);
    e[1] = std::clamp(e[1] * cfg_.pitch_gain, -cfg_.max_pitch_rad, cfg_.max_pitch_rad);
    e[2] = std::clamp(e[2] * cfg_.yaw_gain, -cfg_.max_yaw_rad, cfg_.max_yaw_rad);
    q_corr = quat_from_euler(e[0], e[1], e[2]);

    cv::Mat H = warp_.homography_from_quat(q_corr);

    cv::warpPerspective(frame, out, H, frame.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    if (cfg_.enable_crop && cfg_.crop_percent > 0.0) {
        int cw = (int)(out.cols * cfg_.crop_percent / 200.0);
        int ch = (int)(out.rows * cfg_.crop_percent / 200.0);
        cv::Rect roi(cw, ch, out.cols - 2 * cw, out.rows - 2 * ch);
        cv::Mat cropped;
        cv::resize(out(roi), cropped, out.size(), 0, 0, cv::INTER_LINEAR);
        out = cropped;
    }

    prev_phys_ = q_phys;
    prev_phys_ok_ = true;

    if (dbg) {
        dbg->q_phys = q_phys;
        dbg->q_virtual = q_virtual;
        dbg->q_corr = q_corr;
        dbg->range = info;
        dbg->crop_percent = cfg_.enable_crop ? cfg_.crop_percent : 0.0;
    }
    return true;
}
