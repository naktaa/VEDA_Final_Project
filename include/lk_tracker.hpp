#pragma once

#include <opencv2/opencv.hpp>

#include "app_config.hpp"
#include "app_types.hpp"

class LkTracker {
public:
    explicit LkTracker(const EisRuntimeConfig& config);

    void update_config(const EisRuntimeConfig& config);
    LkMotionEstimate estimate(const cv::Mat& prev_bgr, const cv::Mat& curr_bgr) const;

private:
    EisRuntimeConfig config_;
};
