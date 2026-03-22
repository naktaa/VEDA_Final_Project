#pragma once

#include <opencv2/opencv.hpp>

cv::Mat ensure_bgr(const cv::Mat& input, bool libcamera_xrgb, int target_width, int target_height);
double compute_required_crop_percent(const cv::Mat& homography, int width, int height);
cv::Mat to_homography3x3(const cv::Mat& affine_or_homography);
