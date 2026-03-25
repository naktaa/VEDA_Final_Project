#pragma once

#include <string_view>

namespace veda_server::defaults {

inline constexpr std::string_view kRtspUrl =
    "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
inline constexpr std::string_view kMqttHost = "192.168.100.7";
inline constexpr int kMqttPort = 1883;

inline constexpr std::string_view kPoseTopic = "wiserisk/p1/pose";
inline constexpr std::string_view kHomographyTopic = "wiserisk/map/H_img2world";
inline constexpr std::string_view kMapTopic = "wiserisk/map/graph";

inline constexpr std::string_view kHomographyYaml = "config/H_img2world.yaml";
inline constexpr std::string_view kCameraYaml = "config/camera.yaml";

inline constexpr double kMarkerSize = 0.17;
inline constexpr double kCubeSize = 0.17;
inline constexpr int kPublishIntervalMs = 1000;

} // namespace veda_server::defaults
