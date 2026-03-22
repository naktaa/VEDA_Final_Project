#include "image_utils.hpp"

#include <algorithm>

cv::Mat ensure_bgr(const cv::Mat& input, bool libcamera_xrgb, int target_width, int target_height) {
    if (input.empty()) return {};

    cv::Mat output = input;
    if (output.type() == CV_8UC4) {
        if (libcamera_xrgb) {
            cv::Mat converted(output.rows, output.cols, CV_8UC3);
            const int from_to[] = {3, 0, 2, 1, 1, 2};
            cv::mixChannels(&output, 1, &converted, 1, from_to, 3);
            output = converted;
        } else {
            cv::cvtColor(output, output, cv::COLOR_BGRA2BGR);
        }
    } else if (output.type() == CV_8UC1) {
        cv::cvtColor(output, output, cv::COLOR_GRAY2BGR);
    } else if (output.type() != CV_8UC3) {
        cv::Mat converted;
        output.convertTo(converted, CV_8U);
        if (converted.channels() == 1) {
            cv::cvtColor(converted, converted, cv::COLOR_GRAY2BGR);
        }
        output = converted;
    }

    if (output.cols != target_width || output.rows != target_height) {
        cv::resize(output, output, cv::Size(target_width, target_height), 0.0, 0.0, cv::INTER_LINEAR);
    }
    if (!output.isContinuous()) {
        output = output.clone();
    }
    return output;
}

cv::Mat to_homography3x3(const cv::Mat& affine_or_homography) {
    if (affine_or_homography.empty()) {
        return cv::Mat::eye(3, 3, CV_64F);
    }
    if (affine_or_homography.rows == 3 && affine_or_homography.cols == 3) {
        return affine_or_homography.clone();
    }
    if (affine_or_homography.rows == 2 && affine_or_homography.cols == 3) {
        cv::Mat homography = cv::Mat::eye(3, 3, CV_64F);
        affine_or_homography.copyTo(homography(cv::Rect(0, 0, 3, 2)));
        return homography;
    }
    return cv::Mat::eye(3, 3, CV_64F);
}

double compute_required_crop_percent(const cv::Mat& homography, int width, int height) {
    const cv::Mat H = to_homography3x3(homography);
    std::vector<cv::Point2f> corners;
    corners.emplace_back(0.0F, 0.0F);
    corners.emplace_back(static_cast<float>(width), 0.0F);
    corners.emplace_back(static_cast<float>(width), static_cast<float>(height));
    corners.emplace_back(0.0F, static_cast<float>(height));

    std::vector<cv::Point2f> warped;
    cv::perspectiveTransform(corners, warped, H);

    double minx = 1e9;
    double maxx = -1e9;
    double miny = 1e9;
    double maxy = -1e9;
    for (const auto& point : warped) {
        minx = std::min(minx, static_cast<double>(point.x));
        maxx = std::max(maxx, static_cast<double>(point.x));
        miny = std::min(miny, static_cast<double>(point.y));
        maxy = std::max(maxy, static_cast<double>(point.y));
    }

    const double overlap_w = std::min(static_cast<double>(width), maxx) - std::max(0.0, minx);
    const double overlap_h = std::min(static_cast<double>(height), maxy) - std::max(0.0, miny);
    if (overlap_w <= 1.0 || overlap_h <= 1.0) {
        return 100.0;
    }

    const double scale_x = static_cast<double>(width) / overlap_w;
    const double scale_y = static_cast<double>(height) / overlap_h;
    const double required_scale = std::max(1.0, std::max(scale_x, scale_y));
    return (1.0 - (1.0 / required_scale)) * 100.0;
}
