#pragma once

#include <atomic>
#include <functional>
#include <string>

class PtzController;

struct MqttConfig {
    std::string host;
    int port = 0;
    int keepalive_sec = 0;
    std::string control_topic;
    std::string goal_topic;
    std::string pose_topic;
    std::string safety_topic;
    std::string status_topic;
    int status_publish_interval_ms = 50;
};

bool run_mqtt_drive_loop(const MqttConfig& cfg,
                         std::atomic<bool>& running,
                         PtzController* ptz_controller = nullptr,
                         const std::function<void(const char*)>& on_manual_override = {});

