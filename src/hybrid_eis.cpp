#include "hybrid_eis.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "image_utils.hpp"

namespace {

constexpr int kReferenceMinFeatureCount = 10;
constexpr int kReferenceMinAffinePoints = 6;

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

LkMotionEstimate estimate_reference_style_lk(const cv::Mat& prev_bgr,
                                             const cv::Mat& curr_bgr,
                                             const EisRuntimeConfig& config) {
    LkMotionEstimate result;
    if (prev_bgr.empty() || curr_bgr.empty()) {
        return result;
    }

    cv::Mat prev_gray;
    cv::Mat curr_gray;
    if (prev_bgr.channels() == 3) {
        cv::cvtColor(prev_bgr, prev_gray, cv::COLOR_BGR2GRAY);
    } else {
        prev_gray = prev_bgr;
    }
    if (curr_bgr.channels() == 3) {
        cv::cvtColor(curr_bgr, curr_gray, cv::COLOR_BGR2GRAY);
    } else {
        curr_gray = curr_bgr;
    }

    std::vector<cv::Point2f> features_prev;
    cv::goodFeaturesToTrack(prev_gray,
                            features_prev,
                            config.lk_max_features,
                            config.lk_quality,
                            config.lk_min_dist);
    result.features = static_cast<int>(features_prev.size());
    if (result.features < kReferenceMinFeatureCount) {
        return result;
    }

    std::vector<cv::Point2f> features_curr;
    std::vector<uchar> status;
    std::vector<float> errors;
    cv::calcOpticalFlowPyrLK(prev_gray,
                             curr_gray,
                             features_prev,
                             features_curr,
                             status,
                             errors);

    std::vector<cv::Point2f> good_prev;
    std::vector<cv::Point2f> good_curr;
    good_prev.reserve(features_prev.size());
    good_curr.reserve(features_prev.size());
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            good_prev.push_back(features_prev[i]);
            good_curr.push_back(features_curr[i]);
        }
    }

    result.valid_points = static_cast<int>(good_prev.size());
    result.inliers = result.valid_points;
    result.confidence = result.valid_points > 0 ? 1.0 : 0.0;
    if (result.valid_points < kReferenceMinAffinePoints) {
        return result;
    }

    const cv::Mat affine = cv::estimateAffinePartial2D(good_prev, good_curr);
    if (affine.empty()) {
        return result;
    }

    result.dx = affine.at<double>(0, 2);
    result.dy = affine.at<double>(1, 2);
    result.da = std::atan2(affine.at<double>(1, 0), affine.at<double>(0, 0));
    const double cos_da = std::cos(result.da);
    if (std::abs(cos_da) <= 1e-6) {
        return result;
    }
    result.sx = affine.at<double>(0, 0) / cos_da;
    result.sy = affine.at<double>(1, 1) / cos_da;
    result.valid = true;
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
    prev_frame_.release();
    prev_frame_ok_ = false;
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
    integrator_.reset();
    reset_lk_stabilization();
}

void HybridEisProcessor::update_state(double yaw_rate_dps) {
    const double abs_yaw = std::abs(yaw_rate_dps);
    switch (state_) {
    case HybridState::STABILIZE:
        if (abs_yaw >= config_.eis.turn_enter_yaw_rate_dps) {
            ++turn_enter_count_;
            if (turn_enter_count_ >= std::max(1, config_.eis.turn_hold_frames)) {
                state_ = HybridState::TURN_FOLLOW;
                turn_enter_count_ = 0;
                turn_exit_count_ = 0;
                recover_frames_left_ = 0;
            }
        } else {
            turn_enter_count_ = 0;
        }
        break;
    case HybridState::TURN_FOLLOW:
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
        if (abs_yaw >= config_.eis.turn_enter_yaw_rate_dps) {
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

bool HybridEisProcessor::process(const CapturedFrame& frame, cv::Mat& stabilized, HybridEisDebugInfo* debug) {
    if (debug) {
        *debug = HybridEisDebugInfo{};
        debug->state = state_;
    }

    if (frame.image.empty()) {
        stabilized.release();
        return false;
    }

    const cv::Mat current = frame.image;

    ImuSample latest_imu;
    const bool have_latest_imu = gyro_buffer_ && gyro_buffer_->latest(latest_imu);
    const double yaw_rate_dps = have_latest_imu
        ? latest_imu.gyro_rad_s[2] * 180.0 / CV_PI
        : 0.0;

    const HybridState prev_state = state_;
    update_state(yaw_rate_dps);
    if (state_ == HybridState::TURN_FOLLOW && prev_state != HybridState::TURN_FOLLOW) {
        reset_lk_stabilization();
    }

    LkMotionEstimate lk;
    if (prev_frame_ok_) {
        lk = estimate_reference_style_lk(prev_frame_, current, config_.eis);
    }

    const bool lk_usable = lk.valid;
    const double correction_scale = state_ == HybridState::RECOVER
        ? recover_progress(config_, recover_frames_left_)
        : (state_ == HybridState::TURN_FOLLOW ? 0.0 : 1.0);

    double target_time_ms = frame.frame_time_ms + config_.calib.imu_offset_ms;
    if (debug) {
        debug->state = state_;
        debug->yaw_rate_dps = yaw_rate_dps;
        debug->gyro_valid = have_latest_imu;
        debug->gyro_target_time_ms = target_time_ms;
        debug->gyro_latest_sample_time_ms = have_latest_imu ? latest_imu.sample_time_ms : 0.0;
        debug->gyro_latest_lag_ms = have_latest_imu ? (target_time_ms - latest_imu.sample_time_ms) : 0.0;
    }

    double corr_dx = 0.0;
    double corr_dy = 0.0;
    double corr_da = 0.0;
    double required_crop = 0.0;

    if (!prev_frame_ok_ || state_ == HybridState::TURN_FOLLOW) {
        stabilized = apply_fixed_crop(current, config_.eis.crop_budget_percent);
    } else if (!lk_usable) {
        reset_lk_stabilization();
        stabilized = apply_fixed_crop(current, config_.eis.crop_budget_percent);
    } else {
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

        cv::warpAffine(prev_frame_,
                       stabilized,
                       smoothed,
                       current.size(),
                       cv::INTER_LINEAR,
                       cv::BORDER_REPLICATE);
        required_crop = compute_required_crop_percent(to_homography3x3(smoothed), current.cols, current.rows);
        stabilized = apply_fixed_crop(stabilized, config_.eis.crop_budget_percent);
    }

    if (config_.eis.debug_overlay) {
        char line1[256];
        std::snprintf(line1,
                      sizeof(line1),
                      "state=%s yaw=%+.1fdps lk=%.2f corr x=%+.1f y=%+.1f a=%+.2fdeg",
                      hybrid_state_str(state_),
                      yaw_rate_dps,
                      lk.confidence,
                      corr_dx,
                      corr_dy,
                      corr_da * 180.0 / CV_PI);
        cv::putText(stabilized,
                    line1,
                    cv::Point(10, 28),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.55,
                    cv::Scalar(40, 255, 40),
                    2,
                    cv::LINE_AA);
    }

    prev_frame_ = current.clone();
    prev_frame_ok_ = true;

    if (debug) {
        debug->lk_valid = lk.valid;
        debug->lk_confidence = lk.confidence;
        debug->lk_features = lk.features;
        debug->lk_valid_points = lk.valid_points;
        debug->lk_inliers = lk.inliers;
        debug->crop_required_percent = required_crop;
        debug->clamp_scale = 1.0;
        debug->visual_anchor_rad = corr_da;
        debug->applied_rotation_rad = cv::Vec3d(0.0, 0.0, corr_da);
        debug->applied_tx = corr_dx;
        debug->applied_ty = corr_dy;
    }

    return true;
}
