#include "eis_processor.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

void EisProcessor::Config::load_from_ini(const std::string& path) {
    std::ifstream fs(path);
    if (!fs.is_open()) {
        std::cerr << "[EIS] No config file found: " << path << " (using defaults)\n";
        return;
    }

    std::string line;
    while (std::getline(fs, line)) {
        if (line.empty() || line[0] == '#' || line[0] == '[') continue;
        size_t sep = line.find('=');
        if (sep == std::string::npos) continue;

        std::string key = line.substr(0, sep);
        std::string val = line.substr(sep + 1);
        // Trim
        key.erase(key.find_last_not_of(" \t\r\n") + 1);
        val.erase(0, val.find_first_not_of(" \t\r\n"));

        if      (key == "max_features")    max_features = std::stoi(val);
        else if (key == "feature_quality") feature_quality = std::stod(val);
        else if (key == "feature_min_dist") feature_min_dist = std::stod(val);
        else if (key == "kf_q")            kf_q = std::stod(val);
        else if (key == "kf_r")            kf_r = std::stod(val);
        else if (key == "border_crop")     border_crop = std::stoi(val);
        else if (key == "debug_overlay")   debug_overlay = (val == "true" || val == "1");
    }
    std::cout << "[EIS] Loaded config from " << path << "\n";
}

EisProcessor::EisProcessor() {
    config_.load_from_ini("config_lk_local.ini");
    kf_theta_.init(config_.kf_q, config_.kf_r);
    kf_tx_.init(config_.kf_q, config_.kf_r);
    kf_ty_.init(config_.kf_q, config_.kf_r);
}

void EisProcessor::process(cv::Mat& frame) {
    if (frame.empty()) return;

    cv::rotate(frame, frame, cv::ROTATE_180);
    cv::Mat curr_gray;
    cv::cvtColor(frame, curr_gray, cv::COLOR_BGR2GRAY);

    if (frame_count_ == 0) {
        prev_gray_ = curr_gray.clone();
        frame_count_++;
        return; 
    }

    cv::Mat raw_frame = frame.clone(); // 다음 프레임 비교를 위한 원본 저장용

    std::vector<cv::Point2f> features_prev, features_curr;
    std::vector<uchar> status;
    std::vector<float> err_vec;

    cv::goodFeaturesToTrack(prev_gray_, features_prev, 
                            config_.max_features, config_.feature_quality, config_.feature_min_dist);

    if (features_prev.size() < 10) {
        prev_gray_ = curr_gray.clone();
        prev_frame_ = frame.clone();
        return;
    }

    cv::calcOpticalFlowPyrLK(prev_gray_, curr_gray, features_prev, features_curr, status, err_vec);

    std::vector<cv::Point2f> good_prev, good_curr;
    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            good_prev.push_back(features_prev[i]);
            good_curr.push_back(features_curr[i]);
        }
    }

    if (good_prev.size() < 6) {
        prev_gray_ = curr_gray.clone();
        prev_frame_ = frame.clone();
        return;
    }

    cv::Mat affine = cv::estimateAffinePartial2D(good_prev, good_curr);
    if (affine.empty()) {
        prev_gray_ = curr_gray.clone();
        prev_frame_ = frame.clone();
        return;
    }

    double dx = affine.at<double>(0, 2);
    double dy = affine.at<double>(1, 2);
    double da = atan2(affine.at<double>(1, 0), affine.at<double>(0, 0));
    double sx = affine.at<double>(0, 0) / cos(da);
    double sy = affine.at<double>(1, 1) / cos(da);

    kf_theta_.update(da);
    kf_tx_.update(dx);
    kf_ty_.update(dy);

    double diff_da = 0.0, diff_dx = 0.0, diff_dy = 0.0;
    if (frame_count_ > 1) {
        diff_da = kf_theta_.diff();
        diff_dx = kf_tx_.diff();
        diff_dy = kf_ty_.diff();
    }
    frame_count_++;

    da += diff_da;
    dx += diff_dx;
    dy += diff_dy;

    cv::Mat smoothed = (cv::Mat_<double>(2, 3) <<
        sx * cos(da), sx * -sin(da), dx,
        sy * sin(da), sy *  cos(da), dy);

    cv::Mat stabilized;
    cv::warpAffine(frame, stabilized, smoothed, frame.size());

    // Crop & Resize
    if (config_.border_crop > 0) {
        int vert_border = config_.border_crop * frame.rows / frame.cols;
        cv::Rect roi(config_.border_crop, vert_border, 
                     frame.cols - 2 * config_.border_crop, 
                     frame.rows - 2 * vert_border);
        if (roi.width > 0 && roi.height > 0) {
            stabilized = stabilized(roi);
            cv::resize(stabilized, stabilized, frame.size());
        }
    }

    if (config_.debug_overlay) {
        char buf[128];
        snprintf(buf, sizeof(buf), "EIS: dx:%+5.1f dy:%+5.1f da:%+5.2f", 
                 diff_dx, diff_dy, diff_da * 180.0 / CV_PI);
        cv::putText(stabilized, buf, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }

    frame = stabilized;
    prev_gray_ = curr_gray;
}
