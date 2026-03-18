#include "rc_status_types.h"

#include <nlohmann/json.hpp>

namespace {
std::string MakeDataPeriodString(int publish_interval_ms) {
    return std::to_string(publish_interval_ms) + "ms";
}

nlohmann::json TargetToJson(const TargetInfo& target) {
    return nlohmann::json{{"x", target.x}, {"y", target.y}, {"z", target.z}};
}

nlohmann::json MotorToJson(const MotorInfo& motor) {
    return nlohmann::json{{"name", motor.name}, {"torque", motor.torque}};
}
} // namespace

RcStatus CreateDefaultRcStatus(int publish_interval_ms) {
    RcStatus status;
    status.type = "rc_status";
    status.src = "rc";

    status.connected = true;
    status.mode = "manual";
    status.mission = "none";

    status.battery = -1.0;
    status.speed = -1.0;
    status.x = -1.0;
    status.y = -1.0;
    status.heading = -1.0;

    status.comm_state = "connected";
    status.robot_state = "idle";
    status.data_period = MakeDataPeriodString(publish_interval_ms);

    return status;
}

RcStatus CreateDisconnectedRcStatus(int publish_interval_ms) {
    RcStatus status = CreateDefaultRcStatus(publish_interval_ms);
    status.connected = false;
    status.comm_state = "disconnected";
    status.robot_state = "offline";
    return status;
}

std::string SerializeRcStatusToJson(const RcStatus& status) {
    nlohmann::json j;

    // Required snapshot fields (always present)
    j["type"] = status.type;
    j["src"] = status.src;
    j["connected"] = status.connected;
    j["mode"] = status.mode;
    j["mission"] = status.mission;
    j["battery"] = status.battery;
    j["speed"] = status.speed;
    j["x"] = status.x;
    j["y"] = status.y;
    j["heading"] = status.heading;

    // Optional fields
    if (status.comm_state.has_value()) {
        j["comm_state"] = status.comm_state.value();
    }
    if (status.robot_state.has_value()) {
        j["robot_state"] = status.robot_state.value();
    }
    if (status.data_period.has_value()) {
        j["data_period"] = status.data_period.value();
    }
    if (status.z.has_value()) {
        j["z"] = status.z.value();
    }
    if (status.target.has_value()) {
        j["target"] = TargetToJson(status.target.value());
    }
    if (status.task_daily.has_value()) {
        j["task_daily"] = status.task_daily.value();
    }
    if (status.task_weekly.has_value()) {
        j["task_weekly"] = status.task_weekly.value();
    }
    if (status.task_monthly.has_value()) {
        j["task_monthly"] = status.task_monthly.value();
    }
    if (!status.motors.empty()) {
        j["motors"] = nlohmann::json::array();
        for (const MotorInfo& motor : status.motors) {
            j["motors"].push_back(MotorToJson(motor));
        }
    }

    return j.dump();
}
