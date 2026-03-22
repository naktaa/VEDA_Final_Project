#include "gyro_eis.hpp"

#include <algorithm>
#include <cmath>

cv::Mat CameraIntrinsics::K() const {
    return (cv::Mat_<double>(3, 3) << fx, 0.0, cx,
                                      0.0, fy, cy,
                                      0.0, 0.0, 1.0);
}

cv::Mat CameraIntrinsics::Kinv() const {
    return K().inv();
}

CameraIntrinsics CameraIntrinsics::from_fov(int width, int height, double hfov_deg, double vfov_deg) {
    CameraIntrinsics intr;
    const double hfov = hfov_deg * CV_PI / 180.0;
    const double vfov = vfov_deg * CV_PI / 180.0;
    intr.cx = static_cast<double>(width) * 0.5;
    intr.cy = static_cast<double>(height) * 0.5;
    intr.fx = intr.cx / std::tan(hfov * 0.5);
    intr.fy = intr.cy / std::tan(vfov * 0.5);
    return intr;
}

GyroBuffer::GyroBuffer(size_t max_samples)
    : max_samples_(max_samples) {}

void GyroBuffer::push(const ImuSample& sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push_back(sample);
    while (buffer_.size() > max_samples_) {
        buffer_.pop_front();
    }
}

bool GyroBuffer::get_range(double t0_ms, double t1_ms, std::vector<ImuSample>& out, GyroRangeInfo* info) const {
    out.clear();
    if (t1_ms < t0_ms) {
        std::swap(t0_ms, t1_ms);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.empty()) {
        return false;
    }

    int start_index = -1;
    for (size_t i = 0; i < buffer_.size(); ++i) {
        if (buffer_[i].sample_time_ms >= t0_ms) {
            start_index = static_cast<int>(i);
            break;
        }
    }
    if (start_index < 0) {
        start_index = static_cast<int>(buffer_.size()) - 1;
    }
    if (start_index > 0) {
        out.push_back(buffer_[static_cast<size_t>(start_index - 1)]);
    }

    for (size_t i = static_cast<size_t>(start_index); i < buffer_.size(); ++i) {
        const ImuSample& sample = buffer_[i];
        if (sample.sample_time_ms > t1_ms) {
            out.push_back(sample);
            break;
        }
        out.push_back(sample);
    }

    if (info) {
        info->t0 = t0_ms;
        info->t1 = t1_ms;
        info->used = static_cast<int>(out.size());
        if (!out.empty()) {
            info->min_ts = out.front().sample_time_ms;
            info->max_ts = out.back().sample_time_ms;
        }
    }
    return !out.empty();
}

bool GyroBuffer::latest(ImuSample& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.empty()) {
        return false;
    }
    out = buffer_.back();
    return true;
}

size_t GyroBuffer::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
}

Quaternion Quaternion::identity() {
    return {};
}

Quaternion Quaternion::normalized() const {
    Quaternion q = *this;
    q.normalize();
    return q;
}

void Quaternion::normalize() {
    const double norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (norm <= 1e-12) {
        w = 1.0;
        x = y = z = 0.0;
        return;
    }
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion{w, -x, -y, -z};
}

Quaternion Quaternion::operator*(const Quaternion& rhs) const {
    Quaternion q;
    q.w = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
    q.x = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
    q.y = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
    q.z = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;
    return q;
}

Quaternion Quaternion::from_omega_dt(const cv::Vec3d& omega_rad_s, double dt_sec) {
    const double wx = omega_rad_s[0];
    const double wy = omega_rad_s[1];
    const double wz = omega_rad_s[2];
    const double norm = std::sqrt(wx * wx + wy * wy + wz * wz);
    const double angle = norm * dt_sec;

    if (angle < 1e-6) {
        return Quaternion{1.0, 0.5 * wx * dt_sec, 0.5 * wy * dt_sec, 0.5 * wz * dt_sec};
    }

    const double half = angle * 0.5;
    const double scale = std::sin(half) / norm;
    return Quaternion{std::cos(half), wx * scale, wy * scale, wz * scale};
}

Quaternion Quaternion::slerp(const Quaternion& a, const Quaternion& b, double t) {
    double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    Quaternion end = b;
    if (dot < 0.0) {
        dot = -dot;
        end = Quaternion{-b.w, -b.x, -b.y, -b.z};
    }
    if (dot > 0.9995) {
        Quaternion q{a.w + t * (end.w - a.w),
                     a.x + t * (end.x - a.x),
                     a.y + t * (end.y - a.y),
                     a.z + t * (end.z - a.z)};
        q.normalize();
        return q;
    }

    const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
    const double s = std::sin(theta);
    const double s0 = std::sin((1.0 - t) * theta) / s;
    const double s1 = std::sin(t * theta) / s;
    Quaternion q{
        a.w * s0 + end.w * s1,
        a.x * s0 + end.x * s1,
        a.y * s0 + end.y * s1,
        a.z * s0 + end.z * s1,
    };
    q.normalize();
    return q;
}

cv::Mat Quaternion::to_rotation_matrix() const {
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
        ww + xx - yy - zz, 2.0 * (xy - wz),     2.0 * (xz + wy),
        2.0 * (xy + wz),     ww - xx + yy - zz, 2.0 * (yz - wx),
        2.0 * (xz - wy),     2.0 * (yz + wx),   ww - xx - yy + zz);
}

GyroIntegrator::GyroIntegrator() {
    reset();
}

void GyroIntegrator::reset() {
    q_ = Quaternion::identity();
    last_t_ms_ = 0.0;
    initialized_ = false;
}

void GyroIntegrator::set_bias(const cv::Vec3d& bias_rad_s) {
    bias_ = bias_rad_s;
}

bool GyroIntegrator::integrate_to(double t_ms, const GyroBuffer& buffer, Quaternion& out_q, GyroRangeInfo* info) {
    if (!initialized_) {
        last_t_ms_ = t_ms;
        initialized_ = true;
        out_q = q_;
        if (info) {
            info->t0 = t_ms;
            info->t1 = t_ms;
            info->used = 0;
        }
        return false;
    }

    if (t_ms <= last_t_ms_) {
        out_q = q_;
        if (info) {
            info->t0 = t_ms;
            info->t1 = t_ms;
            info->used = 0;
        }
        return false;
    }

    std::vector<ImuSample> samples;
    GyroRangeInfo tmp;
    if (!buffer.get_range(last_t_ms_, t_ms, samples, &tmp)) {
        out_q = q_;
        if (info) {
            *info = tmp;
        }
        return false;
    }

    double t_prev = last_t_ms_;
    cv::Vec3d omega_prev = samples.front().gyro_rad_s - bias_;
    int used = 0;

    for (const ImuSample& sample : samples) {
        const double t = std::clamp(sample.sample_time_ms, last_t_ms_, t_ms);
        if (t <= t_prev) {
            omega_prev = sample.gyro_rad_s - bias_;
            continue;
        }

        const double dt = (t - t_prev) / 1000.0;
        q_ = q_ * Quaternion::from_omega_dt(omega_prev, dt);
        t_prev = t;
        omega_prev = sample.gyro_rad_s - bias_;
        ++used;
    }

    if (t_prev < t_ms) {
        const double dt = (t_ms - t_prev) / 1000.0;
        q_ = q_ * Quaternion::from_omega_dt(omega_prev, dt);
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

EISWarpCalculator::EISWarpCalculator(const CameraIntrinsics& intrinsics) {
    if (intrinsics.valid()) {
        K_ = intrinsics.K();
        Kinv_ = intrinsics.Kinv();
        valid_ = true;
    }
}

cv::Mat EISWarpCalculator::homography_from_quat(const Quaternion& q_corr) const {
    const cv::Mat R = q_corr.to_rotation_matrix();
    return K_ * R * Kinv_;
}

cv::Vec3d quat_to_euler(const Quaternion& q) {
    const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch = 0.0;
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(CV_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    return cv::Vec3d(roll, pitch, yaw);
}

Quaternion quat_from_euler(double roll_rad, double pitch_rad, double yaw_rad) {
    const double cy = std::cos(yaw_rad * 0.5);
    const double sy = std::sin(yaw_rad * 0.5);
    const double cp = std::cos(pitch_rad * 0.5);
    const double sp = std::sin(pitch_rad * 0.5);
    const double cr = std::cos(roll_rad * 0.5);
    const double sr = std::sin(roll_rad * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.normalize();
    return q;
}

double quaternion_angle_deg(const Quaternion& q) {
    const double clamped = std::clamp(std::abs(q.w), 0.0, 1.0);
    return 2.0 * std::acos(clamped) * 180.0 / CV_PI;
}

cv::Mat translation_homography(double tx, double ty) {
    return (cv::Mat_<double>(3, 3) << 1.0, 0.0, tx,
                                      0.0, 1.0, ty,
                                      0.0, 0.0, 1.0);
}

cv::Vec3d clamp_rotation_correction(const cv::Vec3d& in_rad, const EisRuntimeConfig& cfg, bool* clamp_applied) {
    bool clamped = false;
    auto clamp_axis = [&](double value, double limit_deg) {
        const double limit = limit_deg * CV_PI / 180.0;
        if (!std::isfinite(value)) {
            clamped = true;
            return 0.0;
        }
        const double out = std::clamp(value, -limit, limit);
        if (std::abs(out - value) > 1e-12) {
            clamped = true;
        }
        return out;
    };

    cv::Vec3d out;
    out[0] = clamp_axis(in_rad[0], cfg.gyro_max_roll_deg);
    out[1] = clamp_axis(in_rad[1], cfg.gyro_max_pitch_deg);
    out[2] = clamp_axis(in_rad[2], cfg.gyro_max_yaw_deg);
    if (clamp_applied) {
        *clamp_applied = clamped;
    }
    return out;
}
