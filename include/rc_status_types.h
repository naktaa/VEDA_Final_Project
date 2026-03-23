#pragma once

#include <optional>
#include <string>
#include <vector>

struct TargetInfo {
    double x = -1.0;
    double y = -1.0;
    double z = -1.0;
};

struct MotorInfo {
    std::string name;
    double torque = 0.0;
};

struct RcStatus {
    std::string type = "rc_status";
    std::string src = "rc";

    bool connected = true;
    std::string mode = "manual";
    std::string mission = "none";

    double battery = -1.0;
    double speed = -1.0;
    double x = -1.0;
    double y = -1.0;
    double heading = -1.0;

    std::optional<std::string> comm_state;
    std::optional<std::string> robot_state;
    std::optional<std::string> data_period;
    std::optional<double> z;
    std::optional<TargetInfo> target;
    std::optional<int> task_daily;
    std::optional<int> task_weekly;
    std::optional<int> task_monthly;
    std::vector<MotorInfo> motors;
};

RcStatus CreateDefaultRcStatus(int publish_interval_ms = 500);
RcStatus CreateDisconnectedRcStatus(int publish_interval_ms = 500);

std::string SerializeRcStatusToJson(const RcStatus& status);
