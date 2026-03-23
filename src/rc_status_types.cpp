#include "rc_status_types.h"

#include <nlohmann/json.hpp>

namespace {

std::string make_data_period_string(int publish_interval_ms) {
    return std::to_string(publish_interval_ms) + "ms";
}

nlohmann::json target_to_json(const TargetInfo& target) {
    return nlohmann::json{{"x", target.x}, {"y", target.y}, {"z", target.z}};
}

nlohmann::json motor_to_json(const MotorInfo& motor) {
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
    status.data_period = make_data_period_string(publish_interval_ms);
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
    nlohmann::json payload;
    payload["type"] = status.type;
    payload["src"] = status.src;
    payload["connected"] = status.connected;
    payload["mode"] = status.mode;
    payload["mission"] = status.mission;
    payload["battery"] = status.battery;
    payload["speed"] = status.speed;
    payload["x"] = status.x;
    payload["y"] = status.y;
    payload["heading"] = status.heading;

    if (status.comm_state.has_value()) {
        payload["comm_state"] = status.comm_state.value();
    }
    if (status.robot_state.has_value()) {
        payload["robot_state"] = status.robot_state.value();
    }
    if (status.data_period.has_value()) {
        payload["data_period"] = status.data_period.value();
    }
    if (status.z.has_value()) {
        payload["z"] = status.z.value();
    }
    if (status.target.has_value()) {
        payload["target"] = target_to_json(status.target.value());
    }
    if (status.task_daily.has_value()) {
        payload["task_daily"] = status.task_daily.value();
    }
    if (status.task_weekly.has_value()) {
        payload["task_weekly"] = status.task_weekly.value();
    }
    if (status.task_monthly.has_value()) {
        payload["task_monthly"] = status.task_monthly.value();
    }
    if (!status.motors.empty()) {
        payload["motors"] = nlohmann::json::array();
        for (const MotorInfo& motor : status.motors) {
            payload["motors"].push_back(motor_to_json(motor));
        }
    }

    return payload.dump();
}
