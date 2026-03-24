#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include <opencv2/core.hpp>

namespace veda_server {

struct ServerConfig {
    std::string rtsp_url = "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
    std::string mqtt_host = "192.168.100.10";
    int mqtt_port = 1883;

    std::string pose_topic = "wiserisk/p1/pose";
    std::string homography_topic = "wiserisk/map/H_img2world";
    std::string map_topic = "wiserisk/map/graph";

    std::string homography_yaml = "config/H_img2world.yaml";
    std::string camera_yaml = "config/camera.yaml";

    double marker_size = 0.17;
    double cube_size = 0.17;
    int publish_interval_ms = 1000;
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
    std::atomic<long long> update_count{0};
};

} // namespace veda_server
