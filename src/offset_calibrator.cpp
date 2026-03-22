#include "offset_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "imu_reader.hpp"
#include "libcamera_capture.hpp"
#include "lk_tracker.hpp"

namespace {

struct CalibFrame {
    double t_ms = 0.0;
    double lk_da = 0.0;
};

std::string now_timestamp() {
    const std::time_t now = std::time(nullptr);
    std::tm tm {};
    localtime_r(&now, &tm);
    char buffer[64];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm);
    return buffer;
}

bool integrate_axis_delta(const GyroBuffer& buffer,
                          double t0_ms,
                          double t1_ms,
                          int axis,
                          bool use_mapped,
                          int sign,
                          double& out_delta) {
    out_delta = 0.0;
    std::vector<ImuSample> samples;
    if (!buffer.get_range(t0_ms, t1_ms, samples, nullptr) || samples.empty()) {
        return false;
    }

    auto pick_value = [&](const ImuSample& sample) {
        const cv::Vec3d& source = use_mapped ? sample.gyro_rad_s : sample.raw_counts;
        return sign * source[axis];
    };

    double prev_t = t0_ms;
    double prev_w = pick_value(samples.front());
    for (const ImuSample& sample : samples) {
        const double t = std::clamp(sample.sample_time_ms, t0_ms, t1_ms);
        if (t <= prev_t) {
            prev_w = pick_value(sample);
            continue;
        }
        const double dt = (t - prev_t) / 1000.0;
        out_delta += prev_w * dt;
        prev_t = t;
        prev_w = pick_value(sample);
    }
    if (prev_t < t1_ms) {
        out_delta += prev_w * ((t1_ms - prev_t) / 1000.0);
    }
    return true;
}

double correlation(const std::vector<std::pair<double, double>>& values) {
    if (values.size() < 2) {
        return 0.0;
    }
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_yy = 0.0;
    double sum_xy = 0.0;
    const double n = static_cast<double>(values.size());
    for (const auto& value : values) {
        sum_x += value.first;
        sum_y += value.second;
        sum_xx += value.first * value.first;
        sum_yy += value.second * value.second;
        sum_xy += value.first * value.second;
    }
    const double denom = std::sqrt((n * sum_xx - sum_x * sum_x) * (n * sum_yy - sum_y * sum_y));
    if (denom < 1e-9) {
        return 0.0;
    }
    return (n * sum_xy - sum_x * sum_y) / denom;
}

double sweep_offset_ms(const GyroBuffer& buffer,
                       const std::vector<CalibFrame>& frames,
                       double center_ms,
                       double range_ms,
                       double step_ms,
                       double* best_corr_out) {
    double best_offset = center_ms;
    double best_corr = 0.0;
    bool first = true;

    for (double offset = center_ms - range_ms; offset <= center_ms + range_ms + 1e-6; offset += step_ms) {
        std::vector<std::pair<double, double>> pairs;
        pairs.reserve(frames.size());
        for (size_t i = 1; i < frames.size(); ++i) {
            double gyro_delta = 0.0;
            if (!integrate_axis_delta(buffer,
                                      frames[i - 1].t_ms + offset,
                                      frames[i].t_ms + offset,
                                      2,
                                      true,
                                      1,
                                      gyro_delta)) {
                continue;
            }
            pairs.emplace_back(frames[i].lk_da, gyro_delta);
        }
        const double corr = correlation(pairs);
        if (first || std::abs(corr) > std::abs(best_corr)) {
            best_corr = corr;
            best_offset = offset;
            first = false;
        }
    }

    if (best_corr_out) {
        *best_corr_out = best_corr;
    }
    return best_offset;
}

} // namespace

bool OffsetCalibrator::run(AppConfig& config, std::string* error) {
    const int bias_samples = std::max(600, (config.imu.target_hz * config.calib.bias_duration_ms) / 1000);
    const int bias_sleep_us = std::max(1000, 1000000 / std::max(1, config.imu.target_hz));

    cv::Vec3d measured_bias;
    if (!ImuReader::collect_stationary_bias(config.imu, bias_samples, bias_sleep_us, measured_bias, error)) {
        return false;
    }
    config.calib.bias_x = measured_bias[0];
    config.calib.bias_y = measured_bias[1];
    config.calib.bias_z = measured_bias[2];

    std::fprintf(stderr,
                 "[CALIB] stationary bias counts: gx=%.2f gy=%.2f gz=%.2f\n",
                 measured_bias[0],
                 measured_bias[1],
                 measured_bias[2]);
    std::fprintf(stderr,
                 "[CALIB] start moving the camera in yaw for about %.1f seconds.\n",
                 static_cast<double>(config.calib.sweep_duration_ms) / 1000.0);

    ImuReader imu_reader;
    if (!imu_reader.start(config.imu, config.calib, false, error)) {
        return false;
    }

    LibcameraCapture capture;
    if (!capture.init(config.camera, error)) {
        imu_reader.stop();
        return false;
    }

    LkTracker lk_tracker(config.eis);
    std::vector<CalibFrame> frames;
    frames.reserve(300);
    cv::Mat prev_frame;
    bool prev_ok = false;
    double start_ms = -1.0;

    while (true) {
        CapturedFrame frame;
        if (!capture.get_frame(frame, error)) {
            capture.shutdown();
            imu_reader.stop();
            return false;
        }

        if (config.camera.flip) {
            cv::flip(frame.image, frame.image, -1);
        }

        if (start_ms < 0.0) {
            start_ms = frame.frame_time_ms;
        }
        if (prev_ok) {
            const LkMotionEstimate lk = lk_tracker.estimate(prev_frame, frame.image);
            if (lk.valid && lk.confidence >= config.eis.lk_confidence_gate) {
                frames.push_back({frame.frame_time_ms, lk.da});
            }
        }
        prev_frame = frame.image.clone();
        prev_ok = true;

        if ((frame.frame_time_ms - start_ms) >= config.calib.sweep_duration_ms) {
            break;
        }
    }

    capture.shutdown();

    if (frames.size() < 12) {
        imu_reader.stop();
        if (error) *error = "not enough LK calibration samples were collected";
        return false;
    }

    double coarse_corr = 0.0;
    const double coarse_offset = sweep_offset_ms(imu_reader.buffer(),
                                                 frames,
                                                 config.calib.imu_offset_ms,
                                                 config.calib.coarse_range_ms,
                                                 config.calib.coarse_step_ms,
                                                 &coarse_corr);
    double fine_corr = 0.0;
    const double fine_offset = sweep_offset_ms(imu_reader.buffer(),
                                               frames,
                                               coarse_offset,
                                               config.calib.fine_range_ms,
                                               config.calib.fine_step_ms,
                                               &fine_corr);

    std::fprintf(stderr, "[CALIB] coarse offset %.2f ms corr=%.3f\n", coarse_offset, coarse_corr);
    std::fprintf(stderr, "[CALIB] fine offset %.2f ms corr=%.3f\n", fine_offset, fine_corr);

    double best_axis_corr = 0.0;
    int best_axis = -1;
    int best_sign = 1;
    for (int axis = 0; axis < 3; ++axis) {
        for (int sign : {-1, 1}) {
            std::vector<std::pair<double, double>> pairs;
            for (size_t i = 1; i < frames.size(); ++i) {
                double delta = 0.0;
                if (!integrate_axis_delta(imu_reader.buffer(),
                                          frames[i - 1].t_ms + fine_offset,
                                          frames[i].t_ms + fine_offset,
                                          axis,
                                          false,
                                          sign,
                                          delta)) {
                    continue;
                }
                pairs.emplace_back(frames[i].lk_da, delta);
            }
            const double corr = correlation(pairs);
            if (std::abs(corr) > std::abs(best_axis_corr)) {
                best_axis_corr = corr;
                best_axis = axis;
                best_sign = sign;
            }
        }
    }

    std::fprintf(stderr,
                 "[CALIB] yaw mapping hint: axis=%d sign=%d corr=%.3f (current axis_yaw=%d sign_yaw=%d)\n",
                 best_axis,
                 best_sign,
                 best_axis_corr,
                 config.imu.axis_yaw,
                 config.imu.sign_yaw);

    config.calib.imu_offset_ms = fine_offset;
    config.calib.ts_source = "sensor";
    config.calib.last_calibration = now_timestamp();

    imu_reader.stop();
    return true;
}
