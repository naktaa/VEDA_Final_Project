#include "hybrid_eis.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <string>
#include <vector>

#include "image_utils.hpp"

namespace {

constexpr double kGateWindowMs = 12.0;
constexpr double kGateStaleMs = 30.0;
constexpr double kGateMaxAbsYawDps = 250.0;
constexpr int kGateInvalidFrameLimit = 3;

struct GateYawResult {
    bool valid = false;
    double yaw_gate_dps = 0.0;
    int samples = 0;
    double latest_sample_time_ms = 0.0;
    double latest_lag_ms = 0.0;
    int range_used = 0;
    double range_min_ms = 0.0;
    double range_max_ms = 0.0;
    bool covers_start = false;
    bool covers_end = false;
};

struct RollingShutterResult {
    bool attempted = false;
    bool applied = false;
    int band_used = 0;
    int band_total = 0;
    double required_crop_percent = 0.0;
};

std::string lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool rolling_shutter_enabled(const EisRuntimeConfig& config) {
    return lower_copy(config.rs_mode) == "gyro_bands";
}

cv::Mat make_identity_homography() {
    return cv::Mat::eye(3, 3, CV_64F);
}

cv::Mat apply_fixed_crop(const cv::Mat& frame, double crop_percent) {
    if (frame.empty() || crop_percent <= 0.0) {
        return frame.clone();
    }

    const int crop_x = static_cast<int>(frame.cols * crop_percent / 200.0);
    const int crop_y = static_cast<int>(frame.rows * crop_percent / 200.0);
    if (crop_x <= 0 && crop_y <= 0) {
        return frame.clone();
    }

    const cv::Rect roi(crop_x,
                       crop_y,
                       frame.cols - crop_x * 2,
                       frame.rows - crop_y * 2);
    if (roi.width <= 0 || roi.height <= 0) {
        return frame.clone();
    }

    cv::Mat cropped;
    cv::resize(frame(roi), cropped, frame.size(), 0.0, 0.0, cv::INTER_LINEAR);
    return cropped;
}

double recover_progress(const AppConfig& config, int frames_left) {
    const int total = std::max(1, config.eis.recover_frames);
    return 1.0 - static_cast<double>(std::clamp(frames_left, 0, total)) / static_cast<double>(total);
}

double median_value(std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    double median = values[mid];
    if ((values.size() % 2) == 0) {
        const auto max_it = std::max_element(values.begin(), values.begin() + mid);
        median = (*max_it + median) * 0.5;
    }
    return median;
}

GateYawResult compute_gate_yaw(const GyroBuffer* gyro_buffer, double target_time_ms) {
    GateYawResult result;
    if (!gyro_buffer || !std::isfinite(target_time_ms)) {
        return result;
    }

    ImuSample latest_sample;
    if (!gyro_buffer->latest(latest_sample)) {
        return result;
    }
    result.latest_sample_time_ms = latest_sample.sample_time_ms;
    result.latest_lag_ms = target_time_ms - latest_sample.sample_time_ms;
    if (!std::isfinite(result.latest_lag_ms) || result.latest_lag_ms > kGateStaleMs) {
        return result;
    }

    std::vector<ImuSample> range_samples;
    GyroRangeInfo range_info;
    const double window_start_ms = target_time_ms - kGateWindowMs;
    if (!gyro_buffer->get_range(window_start_ms, target_time_ms, range_samples, &range_info)) {
        return result;
    }
    result.range_used = range_info.used;
    result.range_min_ms = range_info.min_ts;
    result.range_max_ms = range_info.max_ts;
    result.covers_start = range_info.covers_start;
    result.covers_end = range_info.covers_end;

    std::vector<double> abs_yaw_values;
    abs_yaw_values.reserve(range_samples.size());
    for (const ImuSample& sample : range_samples) {
        if (sample.sample_time_ms < window_start_ms || sample.sample_time_ms > target_time_ms) {
            continue;
        }
        const double yaw_dps = sample.gyro_rad_s[2] * 180.0 / CV_PI;
        if (!std::isfinite(yaw_dps)) {
            continue;
        }
        const double abs_yaw = std::abs(yaw_dps);
        if (abs_yaw > kGateMaxAbsYawDps) {
            continue;
        }
        abs_yaw_values.push_back(abs_yaw);
    }

    result.samples = static_cast<int>(abs_yaw_values.size());
    if (result.samples <= 0) {
        return result;
    }

    result.yaw_gate_dps = median_value(abs_yaw_values);
    result.valid = std::isfinite(result.yaw_gate_dps);
    return result;
}

RollingShutterResult render_rolling_shutter_bands(const cv::Mat& source_frame,
                                                  const cv::Mat& global_homography,
                                                  double source_center_time_ms,
                                                  int64_t source_sensor_ts_ns,
                                                  const GyroBuffer* gyro_buffer,
                                                  const EISWarpCalculator& warp,
                                                  const AppConfig& config,
                                                  cv::Mat& stabilized) {
    RollingShutterResult result;
    result.band_total = std::max(1, std::min(config.eis.rs_band_count, source_frame.rows));
    result.attempted = rolling_shutter_enabled(config.eis);
    if (!result.attempted ||
        source_frame.empty() ||
        result.band_total < 2 ||
        config.eis.rs_readout_time_ms <= 0.0 ||
        source_sensor_ts_ns <= 0 ||
        !gyro_buffer ||
        !warp.valid() ||
        !std::isfinite(source_center_time_ms)) {
        return result;
    }

    cv::Mat assembled(source_frame.size(), source_frame.type());
    cv::Mat warped_band;
    const cv::Mat global_h = to_homography3x3(global_homography);
    const double readout_ms = config.eis.rs_readout_time_ms;

    for (int band = 0; band < result.band_total; ++band) {
        const int y0 = (source_frame.rows * band) / result.band_total;
        const int y1 = (source_frame.rows * (band + 1)) / result.band_total;
        if (y1 <= y0) {
            continue;
        }

        const double band_center_y = (static_cast<double>(y0 + y1) * 0.5) / static_cast<double>(source_frame.rows);
        const double band_time_ms = source_center_time_ms - (readout_ms * 0.5) + (band_center_y * readout_ms);

        cv::Mat total_h = global_h;
        Quaternion q_center_to_band;
        GyroRangeInfo range_info;
        if (integrate_gyro_delta(*gyro_buffer, source_center_time_ms, band_time_ms, q_center_to_band, &range_info)) {
            const cv::Mat rs_h = warp.homography_from_quat(q_center_to_band.conjugate());
            if (!rs_h.empty()) {
                total_h = global_h * rs_h;
                ++result.band_used;
            }
        }

        result.required_crop_percent = std::max(result.required_crop_percent,
                                                compute_required_crop_percent(total_h, source_frame.cols, source_frame.rows));

        cv::warpPerspective(source_frame,
                            warped_band,
                            total_h,
                            source_frame.size(),
                            cv::INTER_LINEAR,
                            cv::BORDER_REPLICATE);
        warped_band.rowRange(y0, y1).copyTo(assembled.rowRange(y0, y1));
    }

    if (result.band_used > (result.band_total / 2)) {
        stabilized = assembled;
        result.applied = true;
    }
    return result;
}

} // namespace

HybridEisProcessor::HybridEisProcessor(const AppConfig& config, const GyroBuffer* gyro_buffer)
    : config_(config),
      gyro_buffer_(gyro_buffer),
      lk_tracker_(config.eis),
      intrinsics_(CameraIntrinsics::from_fov(config.camera.width,
                                             config.camera.height,
                                             config.camera.hfov_deg,
                                             config.camera.vfov_deg)),
      warp_(intrinsics_) {
    reset_lk_stabilization();
}

void HybridEisProcessor::update_config(const AppConfig& config) {
    config_ = config;
    lk_tracker_.update_config(config.eis);
    intrinsics_ = CameraIntrinsics::from_fov(config.camera.width,
                                             config.camera.height,
                                             config.camera.hfov_deg,
                                             config.camera.vfov_deg);
    warp_ = EISWarpCalculator(intrinsics_);
    reset_lk_stabilization();
}

void HybridEisProcessor::reset_lk_stabilization() {
    kf_theta_.init(config_.eis.lk_kalman_q, config_.eis.lk_kalman_r);
    kf_tx_.init(config_.eis.lk_kalman_q, config_.eis.lk_kalman_r);
    kf_ty_.init(config_.eis.lk_kalman_q, config_.eis.lk_kalman_r);
    lk_frame_count_ = 0;
}

void HybridEisProcessor::reset() {
    prev_tracking_frame_.release();
    prev_render_frame_.release();
    prev_frames_ok_ = false;
    prev_frame_time_ms_ = 0.0;
    prev_sensor_ts_ns_ = 0;
    prev_exposure_us_ = 0;
    prev_phys_ = Quaternion::identity();
    prev_phys_ok_ = false;
    prev_target_ms_ = 0.0;
    prev_target_ok_ = false;
    hp_lp_delta_rad_ = cv::Vec3d(0.0, 0.0, 0.0);
    hp_initialized_ = false;
    state_ = HybridState::STABILIZE;
    turn_enter_count_ = 0;
    turn_exit_count_ = 0;
    recover_frames_left_ = 0;
    gate_invalid_frames_ = 0;
    integrator_.reset();
    reset_lk_stabilization();
}

void HybridEisProcessor::update_state(bool gate_valid, double yaw_gate_dps) {
    const double abs_yaw = std::abs(yaw_gate_dps);
    switch (state_) {
    case HybridState::STABILIZE:
        gate_invalid_frames_ = 0;
        if (!gate_valid) {
            turn_enter_count_ = 0;
            break;
        }
        if (abs_yaw >= config_.eis.turn_enter_yaw_rate_dps) {
            ++turn_enter_count_;
            if (turn_enter_count_ >= std::max(1, config_.eis.turn_hold_frames)) {
                state_ = HybridState::TURN_FOLLOW;
                turn_enter_count_ = 0;
                turn_exit_count_ = 0;
                recover_frames_left_ = 0;
                gate_invalid_frames_ = 0;
            }
        } else {
            turn_enter_count_ = 0;
        }
        break;
    case HybridState::TURN_FOLLOW:
        if (!gate_valid) {
            turn_exit_count_ = 0;
            ++gate_invalid_frames_;
            if (gate_invalid_frames_ >= kGateInvalidFrameLimit) {
                state_ = HybridState::RECOVER;
                recover_frames_left_ = std::max(1, config_.eis.recover_frames);
                gate_invalid_frames_ = 0;
            }
            break;
        }
        gate_invalid_frames_ = 0;
        if (abs_yaw <= config_.eis.turn_exit_yaw_rate_dps) {
            ++turn_exit_count_;
            if (turn_exit_count_ >= std::max(1, config_.eis.turn_hold_frames)) {
                state_ = HybridState::RECOVER;
                recover_frames_left_ = std::max(1, config_.eis.recover_frames);
                turn_exit_count_ = 0;
            }
        } else {
            turn_exit_count_ = 0;
        }
        break;
    case HybridState::RECOVER:
        gate_invalid_frames_ = 0;
        if (gate_valid && abs_yaw >= config_.eis.turn_enter_yaw_rate_dps) {
            state_ = HybridState::TURN_FOLLOW;
            turn_enter_count_ = 0;
            turn_exit_count_ = 0;
            recover_frames_left_ = 0;
            break;
        }
        if (recover_frames_left_ > 0) {
            --recover_frames_left_;
        }
        if (recover_frames_left_ <= 0) {
            state_ = HybridState::STABILIZE;
        }
        break;
    }
}

bool HybridEisProcessor::process(const CapturedFrame& frame,
                                 const cv::Mat& tracking_frame,
                                 cv::Mat& stabilized,
                                 HybridEisDebugInfo* debug) {
    if (debug) {
        *debug = HybridEisDebugInfo{};
        debug->state = state_;
    }

    if (frame.image.empty()) {
        stabilized.release();
        return false;
    }

    const cv::Mat current_render = frame.image;
    const cv::Mat current_tracking = tracking_frame.empty() ? frame.image : tracking_frame;
    const double target_time_ms = frame.frame_time_ms + config_.calib.imu_offset_ms;

    ImuSample latest_imu;
    const bool have_latest_imu = gyro_buffer_ && gyro_buffer_->latest(latest_imu);
    const double raw_yaw_rate_dps = have_latest_imu
        ? latest_imu.gyro_rad_s[2] * 180.0 / CV_PI
        : 0.0;
    const GateYawResult gate = compute_gate_yaw(gyro_buffer_, target_time_ms);

    const HybridState prev_state = state_;
    update_state(gate.valid, gate.yaw_gate_dps);
    if (state_ == HybridState::TURN_FOLLOW && prev_state != HybridState::TURN_FOLLOW) {
        reset_lk_stabilization();
    }

    LkMotionEstimate lk;
    if (prev_frames_ok_) {
        lk = lk_tracker_.estimate(prev_tracking_frame_, current_tracking);
    }

    const bool lk_usable = lk.valid;
    const double correction_scale = state_ == HybridState::RECOVER
        ? recover_progress(config_, recover_frames_left_)
        : (state_ == HybridState::TURN_FOLLOW ? 0.0 : 1.0);

    double corr_dx = 0.0;
    double corr_dy = 0.0;
    double corr_da = 0.0;
    double global_required_crop = 0.0;
    cv::Mat global_h = make_identity_homography();
    bool use_global_lk = false;

    if (prev_frames_ok_ && lk_usable && state_ != HybridState::TURN_FOLLOW) {
        kf_theta_.update(lk.da);
        kf_tx_.update(lk.dx);
        kf_ty_.update(lk.dy);

        if (lk_frame_count_ > 0) {
            corr_da = kf_theta_.diff() * correction_scale;
            corr_dx = std::clamp(kf_tx_.diff() * correction_scale,
                                 -config_.eis.lk_translation_max_corr_px,
                                 config_.eis.lk_translation_max_corr_px);
            corr_dy = std::clamp(kf_ty_.diff() * correction_scale,
                                 -config_.eis.lk_translation_max_corr_px,
                                 config_.eis.lk_translation_max_corr_px);
        }
        ++lk_frame_count_;

        const double out_da = lk.da + corr_da;
        const double out_dx = lk.dx + corr_dx;
        const double out_dy = lk.dy + corr_dy;
        const cv::Mat smoothed = (cv::Mat_<double>(2, 3) <<
            lk.sx * std::cos(out_da), lk.sx * -std::sin(out_da), out_dx,
            lk.sy * std::sin(out_da), lk.sy *  std::cos(out_da), out_dy);

        global_h = to_homography3x3(smoothed);
        global_required_crop = compute_required_crop_percent(global_h, current_render.cols, current_render.rows);
        use_global_lk = true;
    } else if (prev_frames_ok_ && !lk_usable) {
        reset_lk_stabilization();
    }

    const cv::Mat* source_frame = &current_render;
    double source_time_ms = frame.frame_time_ms;
    int64_t source_sensor_ts_ns = frame.sensor_ts_ns;
    if (use_global_lk) {
        source_frame = &prev_render_frame_;
        source_time_ms = prev_frame_time_ms_;
        source_sensor_ts_ns = prev_sensor_ts_ns_;
    }

    RollingShutterResult rs_result;
    cv::Mat rs_stabilized;
    if (rolling_shutter_enabled(config_.eis)) {
        rs_result = render_rolling_shutter_bands(*source_frame,
                                                 global_h,
                                                 source_time_ms,
                                                 source_sensor_ts_ns,
                                                 gyro_buffer_,
                                                 warp_,
                                                 config_,
                                                 rs_stabilized);
    }

    double required_crop = 0.0;
    if (rs_result.applied) {
        stabilized = rs_stabilized;
        required_crop = std::max(global_required_crop, rs_result.required_crop_percent);
    } else if (use_global_lk) {
        cv::warpPerspective(*source_frame,
                            stabilized,
                            global_h,
                            current_render.size(),
                            cv::INTER_LINEAR,
                            cv::BORDER_REPLICATE);
        required_crop = global_required_crop;
    } else {
        stabilized = source_frame->clone();
    }

    stabilized = apply_fixed_crop(stabilized, config_.eis.crop_budget_percent);

    if (config_.eis.debug_overlay) {
        const char* rs_label = rolling_shutter_enabled(config_.eis) ? "bands" : "off";
        char line1[256];
        std::snprintf(line1,
                      sizeof(line1),
                      "state=%s feat=%d pts=%d yaw=%+.1f corr=%+.1f/%+.1f/%+.2f rs=%s %d/%d",
                      hybrid_state_str(state_),
                      lk.features,
                      lk.valid_points,
                      gate.yaw_gate_dps,
                      corr_dx,
                      corr_dy,
                      corr_da * 180.0 / CV_PI,
                      rs_label,
                      rs_result.band_used,
                      rs_result.band_total);
        cv::putText(stabilized,
                    line1,
                    cv::Point(10, 28),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.55,
                    cv::Scalar(40, 255, 40),
                    2,
                    cv::LINE_AA);
    }

    prev_tracking_frame_ = current_tracking.clone();
    prev_render_frame_ = current_render.clone();
    prev_frames_ok_ = true;
    prev_frame_time_ms_ = frame.frame_time_ms;
    prev_sensor_ts_ns_ = frame.sensor_ts_ns;
    prev_exposure_us_ = frame.exposure_us;

    if (debug) {
        debug->state = state_;
        debug->lk_valid = lk.valid;
        debug->lk_confidence = lk.confidence;
        debug->lk_features = lk.features;
        debug->lk_valid_points = lk.valid_points;
        debug->lk_inliers = lk.inliers;
        debug->gyro_valid = have_latest_imu;
        debug->yaw_rate_dps = raw_yaw_rate_dps;
        debug->gyro_gate_valid = gate.valid;
        debug->yaw_gate_dps = gate.yaw_gate_dps;
        debug->gyro_gate_samples = gate.samples;
        debug->gyro_target_time_ms = target_time_ms;
        debug->gyro_latest_sample_time_ms = gate.latest_sample_time_ms;
        debug->gyro_latest_lag_ms = gate.latest_lag_ms;
        debug->gyro_range_used = gate.range_used;
        debug->gyro_range_min_ms = gate.range_min_ms;
        debug->gyro_range_max_ms = gate.range_max_ms;
        debug->gyro_covers_start = gate.covers_start;
        debug->gyro_covers_end = gate.covers_end;
        debug->crop_required_percent = required_crop;
        debug->rs_crop_required_percent = rs_result.required_crop_percent;
        debug->rs_active = rs_result.applied;
        debug->rs_band_used = rs_result.band_used;
        debug->rs_band_total = rs_result.band_total;
        debug->clamp_scale = 1.0;
        debug->visual_anchor_rad = corr_da;
        debug->applied_rotation_rad = cv::Vec3d(0.0, 0.0, corr_da);
        debug->applied_tx = corr_dx;
        debug->applied_ty = corr_dy;
    }

    return true;
}
