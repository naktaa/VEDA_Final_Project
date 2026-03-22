#include "hybrid_eis.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "image_utils.hpp"

namespace {

cv::Mat center_rotation_homography(int width, int height, double angle_rad) {
    const cv::Point2f center(width * 0.5F, height * 0.5F);
    const cv::Mat affine = cv::getRotationMatrix2D(center, angle_rad * 180.0 / CV_PI, 1.0);
    return to_homography3x3(affine);
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

} // namespace

HybridEisProcessor::HybridEisProcessor(const AppConfig& config, const GyroBuffer* gyro_buffer)
    : config_(config),
      gyro_buffer_(gyro_buffer),
      lk_tracker_(config.eis),
      intrinsics_(CameraIntrinsics::from_fov(config.camera.width,
                                             config.camera.height,
                                             config.camera.hfov_deg,
                                             config.camera.vfov_deg)),
      warp_(intrinsics_) {}

void HybridEisProcessor::update_config(const AppConfig& config) {
    config_ = config;
    lk_tracker_.update_config(config.eis);
    intrinsics_ = CameraIntrinsics::from_fov(config.camera.width,
                                             config.camera.height,
                                             config.camera.hfov_deg,
                                             config.camera.vfov_deg);
    warp_ = EISWarpCalculator(intrinsics_);
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
    visual_anchor_rad_ = 0.0;
    visual_anchor_ok_ = false;
    smooth_tx_ = 0.0;
    smooth_ty_ = 0.0;
    smooth_translation_ok_ = false;
    state_ = HybridState::STABILIZE;
    turn_enter_count_ = 0;
    turn_exit_count_ = 0;
    recover_frames_left_ = 0;
    integrator_.reset();
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
    update_state(yaw_rate_dps);

    if (debug) {
        debug->state = state_;
        debug->yaw_rate_dps = yaw_rate_dps;
    }

    LkMotionEstimate lk;
    if (prev_frame_ok_) {
        lk = lk_tracker_.estimate(prev_frame_, current);
    }

    cv::Vec3d gyro_delta_raw(0.0, 0.0, 0.0);
    cv::Vec3d gyro_delta_hp(0.0, 0.0, 0.0);
    cv::Vec3d gyro_corr(0.0, 0.0, 0.0);
    bool gyro_valid = false;
    if (gyro_buffer_) {
        const double target_time_ms = frame.frame_time_ms + config_.calib.imu_offset_ms;
        if (!prev_target_ok_ || target_time_ms > prev_target_ms_) {
            Quaternion current_phys;
            GyroRangeInfo info;
            if (integrator_.integrate_to(target_time_ms, *gyro_buffer_, current_phys, &info)) {
                if (prev_phys_ok_) {
                    Quaternion delta = prev_phys_.conjugate() * current_phys;
                    delta.normalize();
                    gyro_delta_raw = quat_to_euler(delta);
                    const double alpha = std::clamp(config_.eis.gyro_hp_lpf_alpha, 0.0, 1.0);
                    if (!hp_initialized_) {
                        hp_lp_delta_rad_ = gyro_delta_raw;
                        hp_initialized_ = true;
                    } else {
                        hp_lp_delta_rad_ = alpha * hp_lp_delta_rad_ + (1.0 - alpha) * gyro_delta_raw;
                    }
                    gyro_delta_hp = gyro_delta_raw - hp_lp_delta_rad_;

                    gyro_corr[0] = -gyro_delta_hp[0] * config_.eis.gyro_gain_roll * config_.eis.gyro_hp_gain_roll;
                    gyro_corr[1] = -gyro_delta_hp[1] * config_.eis.gyro_gain_pitch * config_.eis.gyro_hp_gain_pitch;
                    gyro_corr[2] = -gyro_delta_hp[2] * config_.eis.gyro_gain_yaw * config_.eis.gyro_hp_gain_yaw;

                    const double delta_angle_deg = quaternion_angle_deg(delta);
                    if (delta_angle_deg > config_.eis.gyro_large_rot_thresh_deg) {
                        gyro_corr *= config_.eis.gyro_large_rot_gain_scale;
                    }
                    gyro_corr = clamp_rotation_correction(gyro_corr, config_.eis);
                    gyro_valid = true;
                }
                prev_phys_ = current_phys;
                prev_phys_ok_ = true;
            }
            prev_target_ms_ = target_time_ms;
            prev_target_ok_ = true;
        }
    }

    double translation_scale = 1.0;
    double rotation_scale = 1.0;
    double visual_scale = 1.0;
    if (state_ == HybridState::TURN_FOLLOW) {
        translation_scale = config_.eis.lk_translation_turn_scale;
        rotation_scale = config_.eis.turn_follow_correction_scale;
        visual_scale = 0.0;
    } else if (state_ == HybridState::RECOVER) {
        const double progress = recover_progress(config_, recover_frames_left_);
        translation_scale = config_.eis.lk_translation_turn_scale +
            (1.0 - config_.eis.lk_translation_turn_scale) * progress;
        rotation_scale = config_.eis.turn_follow_correction_scale +
            (1.0 - config_.eis.turn_follow_correction_scale) * progress;
        visual_scale = progress;
    }

    double tx = smooth_tx_;
    double ty = smooth_ty_;
    if (lk.valid && lk.confidence >= config_.eis.lk_confidence_gate) {
        const double measured_tx = -lk.dx * translation_scale;
        const double measured_ty = -lk.dy * translation_scale;
        const double alpha = std::clamp(config_.eis.lk_translation_alpha, 0.0, 1.0);
        if (!smooth_translation_ok_) {
            smooth_tx_ = measured_tx;
            smooth_ty_ = measured_ty;
            smooth_translation_ok_ = true;
        } else {
            smooth_tx_ = alpha * smooth_tx_ + (1.0 - alpha) * measured_tx;
            smooth_ty_ = alpha * smooth_ty_ + (1.0 - alpha) * measured_ty;
        }

        smooth_tx_ = std::clamp(smooth_tx_,
                                -config_.eis.lk_translation_max_corr_px,
                                config_.eis.lk_translation_max_corr_px);
        smooth_ty_ = std::clamp(smooth_ty_,
                                -config_.eis.lk_translation_max_corr_px,
                                config_.eis.lk_translation_max_corr_px);
        tx = smooth_tx_;
        ty = smooth_ty_;

        const double visual_alpha = std::clamp(config_.eis.lk_rotation_anchor_alpha, 0.0, 1.0);
        if (!visual_anchor_ok_) {
            visual_anchor_rad_ = lk.da;
            visual_anchor_ok_ = true;
        } else {
            visual_anchor_rad_ = visual_alpha * visual_anchor_rad_ + (1.0 - visual_alpha) * lk.da;
        }
    } else if (smooth_translation_ok_) {
        smooth_tx_ *= 0.92;
        smooth_ty_ *= 0.92;
        tx = smooth_tx_;
        ty = smooth_ty_;
    }

    double visual_anchor = 0.0;
    if (visual_anchor_ok_ && lk.confidence >= config_.eis.lk_confidence_gate) {
        visual_anchor = -visual_anchor_rad_ * config_.eis.lk_rotation_gain * visual_scale;
    }

    gyro_corr *= rotation_scale;

    cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
    H = translation_homography(tx, ty) * H;
    if (std::abs(visual_anchor) > 1e-8) {
        H = center_rotation_homography(current.cols, current.rows, visual_anchor) * H;
    }
    if (gyro_valid && warp_.valid()) {
        const Quaternion q_corr = quat_from_euler(gyro_corr[0], gyro_corr[1], gyro_corr[2]);
        H = warp_.homography_from_quat(q_corr) * H;
    }

    double clamp_scale = 1.0;
    const double required_crop = compute_required_crop_percent(H, current.cols, current.rows);
    if (required_crop > config_.eis.crop_budget_percent && required_crop > 0.0) {
        clamp_scale = std::clamp(config_.eis.crop_budget_percent / required_crop, 0.0, 1.0);
        tx *= clamp_scale;
        ty *= clamp_scale;
        visual_anchor *= clamp_scale;
        gyro_corr *= clamp_scale;

        H = translation_homography(tx, ty);
        if (std::abs(visual_anchor) > 1e-8) {
            H = center_rotation_homography(current.cols, current.rows, visual_anchor) * H;
        }
        if (gyro_valid && warp_.valid()) {
            const Quaternion q_corr = quat_from_euler(gyro_corr[0], gyro_corr[1], gyro_corr[2]);
            H = warp_.homography_from_quat(q_corr) * H;
        }
    }

    cv::warpPerspective(current,
                        stabilized,
                        H,
                        current.size(),
                        cv::INTER_LINEAR,
                        cv::BORDER_REPLICATE);
    stabilized = apply_fixed_crop(stabilized, config_.eis.crop_budget_percent);

    if (config_.eis.debug_overlay) {
        char line1[256];
        std::snprintf(line1,
                      sizeof(line1),
                      "state=%s yaw=%+.1fdps lk=%.2f tx=%+.1f ty=%+.1f crop=%.1f%%",
                      hybrid_state_str(state_),
                      yaw_rate_dps,
                      lk.confidence,
                      tx,
                      ty,
                      compute_required_crop_percent(H, current.cols, current.rows));
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
        debug->gyro_valid = gyro_valid;
        debug->crop_required_percent = compute_required_crop_percent(H, current.cols, current.rows);
        debug->clamp_scale = clamp_scale;
        debug->visual_anchor_rad = visual_anchor;
        debug->gyro_delta_raw_rad = gyro_delta_raw;
        debug->gyro_delta_hp_rad = gyro_delta_hp;
        debug->applied_rotation_rad = gyro_corr;
        debug->applied_tx = tx;
        debug->applied_ty = ty;
    }
    return gyro_valid || lk.valid;
}
