#pragma once

#include <atomic>
#include <string>

struct MqttConfig {
    std::string host;
    int port = 0;
    int keepalive_sec = 0;
    std::string topic;
};

bool run_mqtt_drive_loop(const MqttConfig& cfg, std::atomic<bool>& running);

