#pragma once

#include <cstdint>
#include <string>

struct PtzConfig {
    std::string serial_device = "/dev/serial0";
    int serial_baud = 115200;
    int imu_timeout_ms = 300;
};

struct PtzStatus {
    bool servo_ready = false;
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    uint64_t client_timestamp_ms = 0;
    int pan = 1800;
    int tilt = 2400;
    std::string active_source = "hold";
};

class PtzController {
public:
    PtzController() = default;
    ~PtzController();
    PtzController(const PtzController&) = delete;
    PtzController& operator=(const PtzController&) = delete;
    PtzController(PtzController&&) = delete;
    PtzController& operator=(PtzController&&) = delete;

    bool start(const PtzConfig& cfg);
    void stop();

    void handle_mqtt_command(const std::string& command, bool active);
    void handle_imu(float pitch, float roll, float yaw, uint64_t client_timestamp_ms);

    PtzStatus latest_status() const;
    bool servo_ready() const;

private:
    struct Impl;
    Impl* impl_ = nullptr;
};
