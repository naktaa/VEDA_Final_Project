#pragma once

#include <cstdint>
#include <string>

struct PtzConfig {
    std::string i2c_device = "/dev/i2c-1";
    int i2c_address = 0x40;
    int pwm_frequency_hz = 50;
    int pan_channel = 0;
    int tilt_channel = 1;
    float pan_center_deg = 90.0f;
    float pan_left_deg = 180.0f;
    float pan_right_deg = 0.0f;
    float tilt_center_deg = 90.0f;
    float tilt_up_deg = 0.0f;
    float tilt_down_deg = 180.0f;
    int imu_timeout_ms = 300;
};

struct PtzStatus {
    bool servo_ready = false;
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    uint64_t client_timestamp_ms = 0;
    float pan = 90.0f;
    float tilt = 90.0f;
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
