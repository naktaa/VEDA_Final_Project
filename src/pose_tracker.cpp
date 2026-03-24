#include "pose_tracker.hpp"

#include "server_utils.hpp"

#include <cmath>
#include <cstdio>

namespace veda_server {
namespace {

cv::Matx33d ToMatx33d(const cv::Mat& R) {
    return cv::Matx33d(
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
}

} // namespace

PoseTracker::PoseTracker(const ServerConfig& config,
                         const CameraModel& camera_model,
                         SharedHomography& shared)
    : config_(config),
      camera_model_(camera_model),
      shared_(shared),
      detector_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50),
                cv::aruco::DetectorParameters()) {}

bool PoseTracker::ProcessFrame(const cv::Mat& frame, mosquitto* mosq, long long& publish_count) {
    cv::Mat H;
    cv::Matx33d R_world_cam;
    {
        std::lock_guard<std::mutex> lk(shared_.mtx);
        H = shared_.H_img2world.clone();
        R_world_cam = shared_.R_world_cam;
    }

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    detector_.detectMarkers(frame, corners, ids);

    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    if (!ids.empty()) {
        cv::aruco::estimatePoseSingleMarkers(
            corners, config_.marker_size, camera_model_.K, camera_model_.dist, rvecs, tvecs);
    }

    int best_idx = -1;
    MarkerConfig best_cfg;
    double best_area = -1.0;
    int best_priority = 99;

    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] < 21 || ids[i] > 25) {
            continue;
        }
        MarkerConfig cfg;
        if (!GetMarkerConfig(ids[i], config_.cube_size, cfg)) {
            continue;
        }
        const double area = std::abs(cv::contourArea(corners[i]));
        if (cfg.priority < best_priority ||
            (cfg.priority == best_priority && area > best_area)) {
            best_priority = cfg.priority;
            best_area = area;
            best_idx = static_cast<int>(i);
            best_cfg = cfg;
        }
    }

    last_ids_seen_ = ids.size();
    last_best_id_ = (best_idx >= 0) ? ids[best_idx] : -1;
    if (best_idx < 0) {
        return false;
    }

    const auto& marker = corners[best_idx];
    cv::Point2f center(0.0f, 0.0f);
    for (const auto& point : marker) {
        center += point;
    }
    center *= 0.25f;

    cv::Mat pt_img = (cv::Mat_<double>(3, 1) << center.x, center.y, 1.0);
    cv::Mat pt_world = H * pt_img;
    const double x = pt_world.at<double>(0) / pt_world.at<double>(2);
    const double y = pt_world.at<double>(1) / pt_world.at<double>(2);

    cv::Mat R_cam_marker_m;
    cv::Rodrigues(rvecs[best_idx], R_cam_marker_m);
    const cv::Matx33d R_cam_marker = ToMatx33d(R_cam_marker_m);
    const cv::Matx33d R_m_c = best_cfg.R_c_m.t();
    const cv::Matx33d R_cam_cube = R_cam_marker * R_m_c;
    const cv::Matx33d R_world_cube = R_world_cam * R_cam_cube;

    const cv::Vec3d forward = R_world_cube * cv::Vec3d(0.0, 1.0, 0.0);
    const double yaw = NormalizeAngle(-std::atan2(forward[1], forward[0]));
    const long long ts = NowMs();

    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.6f,\"frame\":\"world\",\"ts_ms\":%lld}",
                  x, y, yaw, ts);

    if (PublishJson(mosq, config_.pose_topic, buf, false)) {
        ++publish_count;
        return true;
    }
    return false;
}

size_t PoseTracker::last_ids_seen() const {
    return last_ids_seen_;
}

int PoseTracker::last_best_id() const {
    return last_best_id_;
}

} // namespace veda_server
