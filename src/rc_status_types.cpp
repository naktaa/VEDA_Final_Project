#include "rc_status_types.h"

#include <iomanip>
#include <sstream>

namespace {
std::string MakeDataPeriodString(int publish_interval_ms) {
    return std::to_string(publish_interval_ms) + "ms";
}

std::string JsonEscape(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (char c : value) {
        switch (c) {
            case '\\':
                out += "\\\\";
                break;
            case '"':
                out += "\\\"";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += c;
                break;
        }
    }
    return out;
}

void AppendStringField(std::ostringstream& oss, bool& first, const std::string& key, const std::string& value) {
    if (!first) {
        oss << ",";
    }
    first = false;
    oss << "\"" << key << "\":\"" << JsonEscape(value) << "\"";
}

void AppendBoolField(std::ostringstream& oss, bool& first, const std::string& key, bool value) {
    if (!first) {
        oss << ",";
    }
    first = false;
    oss << "\"" << key << "\":" << (value ? "true" : "false");
}

void AppendDoubleField(std::ostringstream& oss, bool& first, const std::string& key, double value) {
    if (!first) {
        oss << ",";
    }
    first = false;
    oss << "\"" << key << "\":" << value;
}

void AppendIntField(std::ostringstream& oss, bool& first, const std::string& key, int value) {
    if (!first) {
        oss << ",";
    }
    first = false;
    oss << "\"" << key << "\":" << value;
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
    std::ostringstream oss;
    oss << std::boolalpha << std::setprecision(15);

    bool first = true;
    oss << "{";

    AppendStringField(oss, first, "type", status.type);
    AppendStringField(oss, first, "src", status.src);
    AppendBoolField(oss, first, "connected", status.connected);
    AppendStringField(oss, first, "mode", status.mode);
    AppendStringField(oss, first, "mission", status.mission);
    AppendDoubleField(oss, first, "battery", status.battery);
    AppendDoubleField(oss, first, "speed", status.speed);
    AppendDoubleField(oss, first, "x", status.x);
    AppendDoubleField(oss, first, "y", status.y);
    AppendDoubleField(oss, first, "heading", status.heading);

    if (status.comm_state.has_value()) {
        AppendStringField(oss, first, "comm_state", status.comm_state.value());
    }
    if (status.robot_state.has_value()) {
        AppendStringField(oss, first, "robot_state", status.robot_state.value());
    }
    if (status.data_period.has_value()) {
        AppendStringField(oss, first, "data_period", status.data_period.value());
    }
    if (status.z.has_value()) {
        AppendDoubleField(oss, first, "z", status.z.value());
    }
    if (status.target.has_value()) {
        if (!first) {
            oss << ",";
        }
        first = false;
        const TargetInfo& target = status.target.value();
        oss << "\"target\":{"
            << "\"x\":" << target.x << ","
            << "\"y\":" << target.y << ","
            << "\"z\":" << target.z
            << "}";
    }
    if (status.task_daily.has_value()) {
        AppendIntField(oss, first, "task_daily", status.task_daily.value());
    }
    if (status.task_weekly.has_value()) {
        AppendIntField(oss, first, "task_weekly", status.task_weekly.value());
    }
    if (status.task_monthly.has_value()) {
        AppendIntField(oss, first, "task_monthly", status.task_monthly.value());
    }
    if (!status.motors.empty()) {
        if (!first) {
            oss << ",";
        }
        first = false;
        oss << "\"motors\":[";
        for (size_t i = 0; i < status.motors.size(); ++i) {
            if (i != 0) {
                oss << ",";
            }
            oss << "{"
                << "\"name\":\"" << JsonEscape(status.motors[i].name) << "\","
                << "\"torque\":" << status.motors[i].torque
                << "}";
        }
        oss << "]";
    }

    oss << "}";
    return oss.str();
}
