#pragma once

#include <atomic>
#include <filesystem>
#include <mutex>
#include <string>

#include <opencv2/core.hpp>

#include "server_defaults.hpp"

namespace veda_server {

struct ServerConfig {
    std::string rtsp_url = std::string(defaults::kRtspUrl);
    std::string mqtt_host = std::string(defaults::kMqttHost);
    int mqtt_port = defaults::kMqttPort;

    std::string pose_topic = std::string(defaults::kPoseTopic);
    std::string homography_topic = std::string(defaults::kHomographyTopic);
    std::string map_topic = std::string(defaults::kMapTopic);

    std::filesystem::path config_dir = "config";
    std::filesystem::path homography_yaml = defaults::kHomographyYaml;
    std::filesystem::path camera_yaml = defaults::kCameraYaml;

    double marker_size = defaults::kMarkerSize;
    double cube_size = defaults::kCubeSize;
    int publish_interval_ms = defaults::kPublishIntervalMs;
};

struct CameraModel {
    cv::Mat K;
    cv::Mat dist;
};

struct MarkerConfig {
    cv::Matx33d R_c_m;
    cv::Vec3d t_c_m;
    int priority = 9;
};

struct SharedHomography {
    std::mutex mtx;
    cv::Mat H_img2world;
    cv::Matx33d R_world_cam = cv::Matx33d::eye();
    cv::Vec3d t_cam_world = cv::Vec3d(0.0, 0.0, 0.0);
    std::atomic<long long> update_count{0};
};

} // namespace veda_server
