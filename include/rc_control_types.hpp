#pragma once

#include <filesystem>
#include <string>

#include "rc_defaults.hpp"

struct RcGoal {
    double x = 0.0;
    double y = 0.0;
    std::string frame = "world";
    long long ts_ms = 0;
    bool valid = false;
};

struct RcPose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    std::string frame = "world";
    long long ts_ms = 0;
    bool valid = false;
};

struct RcSafety {
    bool estop = false;
    bool obstacle_stop = false;
    bool planner_fail = false;
};

struct ControlStatus {
    std::string robot_state = "WAIT_INPUT";
    bool reached = false;
    double err_dist_m = 0.0;
    double err_yaw_rad = 0.0;
};

struct RcCommand {
    double speed_mps = 0.0;
    double yaw_rate_rps = 0.0;
};

struct RcControlParams {
    double k_linear = 0.5;
    double k_yaw = 0.8;
    double max_speed_mps = 0.70;
    double max_yaw_rate_rps = 0.5;
    double tolerance_m = 0.10;
    double rotate_yaw_offset_rad = 5.0 * 3.14159265358979323846 / 180.0;
};

struct RcMotorParams {
    double track_width_m = 0.22;
    double wheel_max_speed_mps = 0.70;
    double speed_deadband_mps = 0.003;
    int pwm_min_effective = 110;
    int pwm_max = 220;
};

struct RcTopics {
    std::string goal = std::string(rc_defaults::kGoalTopic);
    std::string pose = std::string(rc_defaults::kPoseTopic);
    std::string safety = std::string(rc_defaults::kSafetyTopic);
    std::string status = std::string(rc_defaults::kStatusTopic);
};

struct RcAppConfig {
    std::string host = std::string(rc_defaults::kBrokerHost);
    int port = rc_defaults::kBrokerPort;
    RcTopics topics;
    int status_publish_interval_ms = rc_defaults::kStatusPublishIntervalMs;
    std::filesystem::path config_dir = "config";
    std::filesystem::path ini_path = "config/rc_control.ini";
    std::filesystem::path template_ini_path = "config/rc_control.template.ini";
    RcControlParams control;
    RcMotorParams motor;
};
