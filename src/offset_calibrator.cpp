#include "offset_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "imu_reader.hpp"
#include "libcamera_capture.hpp"
#include "lk_tracker.hpp"
#include "log_utils.hpp"

namespace {

struct CalibFrame {
    uint64_t frame_index = 0;
    double t_ms = 0.0;
    double lk_da = 0.0;
    double lk_confidence = 0.0;
    int lk_features = 0;
    int lk_valid_points = 0;
    int lk_inliers = 0;
};

struct SweepCandidate {
    double offset_ms = 0.0;
    double corr = 0.0;
    size_t pair_count = 0;
};

struct AxisHintCandidate {
    int axis = -1;
    int sign = 1;
    double corr = 0.0;
    size_t pair_count = 0;
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
    GyroRangeInfo info;
    if (!buffer.get_range(t0_ms, t1_ms, samples, &info) ||
        samples.empty() ||
        !info.covers_start ||
        !info.covers_end) {
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
                       double* best_corr_out,
                       std::vector<SweepCandidate>* trace_out = nullptr) {
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
        if (trace_out) {
            trace_out->push_back({offset, corr, pairs.size()});
        }
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
    std::ofstream calib_log;
    std::string calib_log_path;
    if (!log_utils::open_log_file("calib", calib_log, calib_log_path, error)) {
        return false;
    }
    calib_log << std::fixed << std::setprecision(6);
    calib_log << "# type=calibration\n";
    calib_log << "# started_at=" << log_utils::human_timestamp() << "\n";
    calib_log << "# camera_fps=" << config.camera.fps << "\n";
    calib_log << "# imu_target_hz=" << config.imu.target_hz << "\n";
    calib_log << "# imu_use_fifo=" << (config.imu.use_fifo ? 1 : 0) << "\n";
    calib_log << "# imu_int_pin_wpi=" << config.imu.int_pin_wpi << "\n";
    calib_log << "# sweep_duration_ms=" << config.calib.sweep_duration_ms << "\n";
    calib_log << "# coarse_range_ms=" << config.calib.coarse_range_ms << "\n";
    calib_log << "# coarse_step_ms=" << config.calib.coarse_step_ms << "\n";
    calib_log << "# fine_range_ms=" << config.calib.fine_range_ms << "\n";
    calib_log << "# fine_step_ms=" << config.calib.fine_step_ms << "\n";
    std::fprintf(stderr, "[CALIB] log file: %s\n", calib_log_path.c_str());

    const int bias_samples = std::max(600, (config.imu.target_hz * config.calib.bias_duration_ms) / 1000);
    const int bias_sleep_us = std::max(1000, 1000000 / std::max(1, config.imu.target_hz));
    const double retention_ms = std::max(5000.0,
                                         static_cast<double>(config.calib.sweep_duration_ms) +
                                             config.calib.coarse_range_ms * 2.0 +
                                             1000.0);
    const size_t calibration_buffer_samples =
        static_cast<size_t>(std::ceil(retention_ms * static_cast<double>(std::max(1, config.imu.target_hz)) / 1000.0)) +
        128U;

    cv::Vec3d measured_bias;
    if (!ImuReader::collect_stationary_bias(config.imu, bias_samples, bias_sleep_us, measured_bias, error)) {
        return false;
    }
    config.calib.bias_x = measured_bias[0];
    config.calib.bias_y = measured_bias[1];
    config.calib.bias_z = measured_bias[2];

    calib_log << "# section=bias\n";
    calib_log << "bias_x\tbias_y\tbias_z\tbias_samples\tbias_sleep_us\n";
    calib_log << measured_bias[0] << '\t'
              << measured_bias[1] << '\t'
              << measured_bias[2] << '\t'
              << bias_samples << '\t'
              << bias_sleep_us << '\n';

    std::fprintf(stderr,
                 "[CALIB] stationary bias counts: gx=%.2f gy=%.2f gz=%.2f\n",
                 measured_bias[0],
                 measured_bias[1],
                 measured_bias[2]);
    std::fprintf(stderr,
                 "[CALIB] start moving the camera in yaw for about %.1f seconds.\n",
                 static_cast<double>(config.calib.sweep_duration_ms) / 1000.0);

    ImuReader imu_reader(std::max<size_t>(2500, calibration_buffer_samples));
    if (!imu_reader.start(config.imu, config.calib, false, error)) {
        return false;
    }

    std::fprintf(stderr,
                 "[CALIB] imu buffer: %.1f sec (%zu samples)\n",
                 retention_ms / 1000.0,
                 std::max<size_t>(2500, calibration_buffer_samples));
    calib_log << "# section=buffer\n";
    calib_log << "retention_ms\tbuffer_samples\n";
    calib_log << retention_ms << '\t' << std::max<size_t>(2500, calibration_buffer_samples) << '\n';

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
    int total_pairs = 0;

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
            ++total_pairs;
            const LkMotionEstimate lk = lk_tracker.estimate(prev_frame, frame.image);
            if (lk.valid && lk.confidence >= config.eis.lk_confidence_gate) {
                frames.push_back({frame.frame_index,
                                  frame.frame_time_ms,
                                  lk.da,
                                  lk.confidence,
                                  lk.features,
                                  lk.valid_points,
                                  lk.inliers});
            }
        }
        prev_frame = frame.image.clone();
        prev_ok = true;

        if ((frame.frame_time_ms - start_ms) >= config.calib.sweep_duration_ms) {
            break;
        }
    }

    capture.shutdown();

    calib_log << "# section=lk_frames\n";
    calib_log << "frame_index\tt_ms\tlk_da\tlk_confidence\tlk_features\tlk_valid_points\tlk_inliers\n";
    for (const CalibFrame& sample : frames) {
        calib_log << sample.frame_index << '\t'
                  << sample.t_ms << '\t'
                  << sample.lk_da << '\t'
                  << sample.lk_confidence << '\t'
                  << sample.lk_features << '\t'
                  << sample.lk_valid_points << '\t'
                  << sample.lk_inliers << '\n';
    }
    calib_log << "# section=lk_summary\n";
    calib_log << "total_pairs\taccepted_pairs\taccept_ratio\n";
    calib_log << total_pairs << '\t'
              << frames.size() << '\t'
              << (total_pairs > 0 ? static_cast<double>(frames.size()) / static_cast<double>(total_pairs) : 0.0)
              << '\n';

    if (frames.size() < 12) {
        imu_reader.stop();
        calib_log.flush();
        if (error) *error = "not enough LK calibration samples were collected";
        return false;
    }

    double coarse_corr = 0.0;
    std::vector<SweepCandidate> coarse_trace;
    const double coarse_offset = sweep_offset_ms(imu_reader.buffer(),
                                                 frames,
                                                 config.calib.imu_offset_ms,
                                                 config.calib.coarse_range_ms,
                                                 config.calib.coarse_step_ms,
                                                 &coarse_corr,
                                                 &coarse_trace);
    double fine_corr = 0.0;
    std::vector<SweepCandidate> fine_trace;
    const double fine_offset = sweep_offset_ms(imu_reader.buffer(),
                                               frames,
                                               coarse_offset,
                                               config.calib.fine_range_ms,
                                               config.calib.fine_step_ms,
                                               &fine_corr,
                                               &fine_trace);

    calib_log << "# section=sweep_coarse\n";
    calib_log << "offset_ms\tcorr\tpair_count\n";
    for (const SweepCandidate& sample : coarse_trace) {
        calib_log << sample.offset_ms << '\t'
                  << sample.corr << '\t'
                  << sample.pair_count << '\n';
    }
    calib_log << "# section=sweep_fine\n";
    calib_log << "offset_ms\tcorr\tpair_count\n";
    for (const SweepCandidate& sample : fine_trace) {
        calib_log << sample.offset_ms << '\t'
                  << sample.corr << '\t'
                  << sample.pair_count << '\n';
    }

    std::fprintf(stderr, "[CALIB] coarse offset %.2f ms corr=%.3f\n", coarse_offset, coarse_corr);
    std::fprintf(stderr, "[CALIB] fine offset %.2f ms corr=%.3f\n", fine_offset, fine_corr);
    calib_log << "# section=sweep_summary\n";
    calib_log << "coarse_offset_ms\tcoarse_corr\tfine_offset_ms\tfine_corr\n";
    calib_log << coarse_offset << '\t'
              << coarse_corr << '\t'
              << fine_offset << '\t'
              << fine_corr << '\n';

    double best_axis_corr = 0.0;
    int best_axis = -1;
    int best_sign = 1;
    std::vector<AxisHintCandidate> axis_trace;
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
            axis_trace.push_back({axis, sign, corr, pairs.size()});
            if (std::abs(corr) > std::abs(best_axis_corr)) {
                best_axis_corr = corr;
                best_axis = axis;
                best_sign = sign;
            }
        }
    }

    calib_log << "# section=axis_hint\n";
    calib_log << "axis\tsign\tcorr\tpair_count\n";
    for (const AxisHintCandidate& sample : axis_trace) {
        calib_log << sample.axis << '\t'
                  << sample.sign << '\t'
                  << sample.corr << '\t'
                  << sample.pair_count << '\n';
    }
    calib_log << "# section=axis_summary\n";
    calib_log << "best_axis\tbest_sign\tbest_corr\tcurrent_axis_yaw\tcurrent_sign_yaw\n";
    calib_log << best_axis << '\t'
              << best_sign << '\t'
              << best_axis_corr << '\t'
              << config.imu.axis_yaw << '\t'
              << config.imu.sign_yaw << '\n';

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
    calib_log.flush();
    return true;
}
