#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#include "server_types.hpp"

struct mosquitto;

namespace veda_server {

bool LoadHomography(const std::string& path, cv::Mat& H_img2world);
bool LoadCameraModel(const std::string& path, CameraModel& out);
bool EstimateWorldPoseFromHomography(const cv::Mat& H_img2world,
                                     const cv::Mat& K,
                                     cv::Matx33d& R_world_cam,
                                     cv::Vec3d& t_cam_world);
cv::Matx33d EstimateRworldCam(const cv::Mat& H_img2world, const cv::Mat& K);
double NormalizeAngle(double angle);
long long NowMs();
bool OpenRtspCapture(const std::string& rtsp_url, cv::VideoCapture& cap);
bool GetMarkerConfig(int id, double cube_size, MarkerConfig& out);
std::string JsonEscape(const std::string& s);
std::string BuildHomographyPayload(const cv::Mat& H, const std::string& yaml_path);
std::string BuildMapPayload();
bool PublishJson(mosquitto* mosq, const std::string& topic, const std::string& payload, bool retain);

} // namespace veda_server
