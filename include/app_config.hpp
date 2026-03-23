#pragma once

#include <string>

#include "mqtt_drive.hpp"

struct CameraConfig {
    int width = 640;
    int height = 480;
    int fps = 20;
    int frame_duration_us = 50000;
    int exposure_us = 8000;
    bool flip = true;
    double hfov_deg = 62.2;
    double vfov_deg = 48.8;
    bool libcamera_xrgb = false;
    std::string capture_backend = "libcamera";
};

struct ImuConfig {
    int bus = 1;
    int addr = 0x68;
    int int_pin_wpi = -1;
    int target_hz = 500;
    bool use_fifo = true;
    double gyro_sensitivity = 131.0;

    int axis_roll = 2;
    int axis_pitch = 0;
    int axis_yaw = 1;
    int sign_roll = 1;
    int sign_pitch = -1;
    int sign_yaw = -1;
};

struct EisRuntimeConfig {
    int lk_max_features = 180;
    int lk_min_features = 12;
    int lk_min_inliers = 10;
    double lk_quality = 0.01;
    double lk_min_dist = 24.0;
    double lk_ransac_thresh = 2.5;
    double lk_confidence_gate = 0.55;

    double lk_translation_alpha = 0.55;
    double lk_translation_max_corr_px = 20.0;
    double lk_translation_turn_scale = 0.60;

    double lk_rotation_anchor_alpha = 0.92;
    double lk_rotation_gain = 0.18;

    double gyro_gain_roll = 1.20;
    double gyro_gain_pitch = 1.20;
    double gyro_gain_yaw = 0.45;
    double gyro_max_roll_deg = 5.0;
    double gyro_max_pitch_deg = 4.0;
    double gyro_max_yaw_deg = 8.0;
    double gyro_hp_lpf_alpha = 0.90;
    double gyro_hp_gain_roll = 1.0;
    double gyro_hp_gain_pitch = 1.0;
    double gyro_hp_gain_yaw = 1.0;
    double gyro_large_rot_thresh_deg = 3.5;
    double gyro_large_rot_gain_scale = 0.40;

    double crop_budget_percent = 18.0;
    double turn_enter_yaw_rate_dps = 42.0;
    double turn_exit_yaw_rate_dps = 18.0;
    int turn_hold_frames = 3;
    int recover_frames = 12;
    double turn_follow_correction_scale = 0.25;

    bool debug_overlay = false;
};

struct RtspConfig {
    std::string port = "8555";
    std::string path = "/cam";
    std::string raw_path = "/raw";
    int bitrate = 1500000;
    int iframe_period = 30;
};

struct CalibrationConfig {
    double bias_x = 0.0;
    double bias_y = 0.0;
    double bias_z = 0.0;
    double imu_offset_ms = 0.0;
    std::string ts_source = "sensor";
    std::string last_calibration = "";
    int bias_duration_ms = 2500;
    int sweep_duration_ms = 8000;
    double coarse_range_ms = 50.0;
    double coarse_step_ms = 0.5;
    double fine_range_ms = 5.0;
    double fine_step_ms = 0.1;
};

struct AppConfig {
    CameraConfig camera;
    ImuConfig imu;
    EisRuntimeConfig eis;
    RtspConfig rtsp;
    MqttConfig mqtt{
        "192.168.100.10",
        1883,
        30,
        "wiserisk/rc/control",
    };
    CalibrationConfig calib;
};

std::string make_default_config_text();
bool ensure_local_config_exists(const std::string& template_path,
                                const std::string& local_path,
                                bool* created = nullptr,
                                std::string* error = nullptr);
bool load_app_config(const std::string& path, AppConfig& config, std::string* error = nullptr);
bool write_app_config(const std::string& path, const AppConfig& config, std::string* error = nullptr);
