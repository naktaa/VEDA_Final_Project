#include "lk_tracker.hpp"

#include <algorithm>
#include <vector>

LkTracker::LkTracker(const EisRuntimeConfig& config)
    : config_(config) {}

void LkTracker::update_config(const EisRuntimeConfig& config) {
    config_ = config;
}

LkMotionEstimate LkTracker::estimate(const cv::Mat& prev_bgr, const cv::Mat& curr_bgr) const {
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
                            config_.lk_max_features,
                            config_.lk_quality,
                            config_.lk_min_dist);
    result.features = static_cast<int>(features_prev.size());
    if (result.features < config_.lk_min_features) {
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
                             errors,
                             cv::Size(21, 21),
                             3);

    std::vector<float> valid_errors;
    valid_errors.reserve(status.size());
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            valid_errors.push_back(errors[i]);
        }
    }

    float error_threshold = 1e9F;
    if (!valid_errors.empty()) {
        std::sort(valid_errors.begin(), valid_errors.end());
        error_threshold = valid_errors[valid_errors.size() / 2] * 2.0F;
    }

    std::vector<cv::Point2f> good_prev;
    std::vector<cv::Point2f> good_curr;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i] && errors[i] <= error_threshold) {
            good_prev.push_back(features_prev[i]);
            good_curr.push_back(features_curr[i]);
        }
    }

    result.valid_points = static_cast<int>(good_prev.size());
    if (result.valid_points < config_.lk_min_features) {
        return result;
    }

    cv::Mat inlier_mask;
    cv::Mat affine = cv::estimateAffinePartial2D(good_prev,
                                                 good_curr,
                                                 inlier_mask,
                                                 cv::RANSAC,
                                                 config_.lk_ransac_thresh,
                                                 2000,
                                                 0.99,
                                                 10);
    if (affine.empty()) {
        return result;
    }

    result.inliers = inlier_mask.empty() ? result.valid_points : cv::countNonZero(inlier_mask);
    result.confidence = result.valid_points > 0
        ? static_cast<double>(result.inliers) / static_cast<double>(result.valid_points)
        : 0.0;
    if (result.inliers < config_.lk_min_inliers) {
        return result;
    }

    result.dx = affine.at<double>(0, 2);
    result.dy = affine.at<double>(1, 2);
    result.da = std::atan2(affine.at<double>(1, 0), affine.at<double>(0, 0));
    result.valid = true;
    return result;
}
