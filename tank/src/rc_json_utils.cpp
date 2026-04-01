#include "rc_json_utils.hpp"

#include <regex>

namespace {

bool ExtractDouble(const std::string& json, const std::string& key, double& out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) {
        return false;
    }
    out = std::stod(m[1].str());
    return true;
}

bool ExtractInt64(const std::string& json, const std::string& key, long long& out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*([0-9]+)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) {
        return false;
    }
    out = std::stoll(m[1].str());
    return true;
}

bool ExtractString(const std::string& json, const std::string& key, std::string& out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) {
        return false;
    }
    out = m[1].str();
    return true;
}

bool ExtractBool(const std::string& json, const std::string& key, bool& out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(true|false)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) {
        return false;
    }
    out = (m[1].str() == "true");
    return true;
}

} // namespace

bool ParseGoalJson(const std::string& payload, RcGoal& out_goal) {
    if (!ExtractDouble(payload, "x", out_goal.x)) {
        return false;
    }
    if (!ExtractDouble(payload, "y", out_goal.y)) {
        return false;
    }
    if (!ExtractString(payload, "frame", out_goal.frame)) {
        return false;
    }
    if (!ExtractInt64(payload, "ts_ms", out_goal.ts_ms)) {
        return false;
    }
    out_goal.valid = true;
    return true;
}

bool ParsePoseJson(const std::string& payload, RcPose& out_pose) {
    if (!ExtractDouble(payload, "x", out_pose.x)) {
        return false;
    }
    if (!ExtractDouble(payload, "y", out_pose.y)) {
        return false;
    }
    if (!ExtractDouble(payload, "yaw", out_pose.yaw)) {
        return false;
    }
    if (!ExtractString(payload, "frame", out_pose.frame)) {
        return false;
    }
    if (!ExtractInt64(payload, "ts_ms", out_pose.ts_ms) &&
        !ExtractInt64(payload, "ts", out_pose.ts_ms)) {
        return false;
    }
    out_pose.valid = true;
    return true;
}

bool ParseSafetyJson(const std::string& payload, RcSafety& out_safety) {
    bool ok = false;
    ok = ExtractBool(payload, "estop", out_safety.estop) || ok;
    ok = ExtractBool(payload, "obstacle_stop", out_safety.obstacle_stop) || ok;
    ok = ExtractBool(payload, "planner_fail", out_safety.planner_fail) || ok;
    return ok;
}
