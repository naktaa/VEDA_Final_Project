#include "mqtt_drive.hpp"

#include <cstdio>

#include <mosquitto.h>

#include "ptz_control.hpp"
#include "tank_drive.hpp"

namespace {

struct MqttRuntime {
    std::string topic;
    std::string active_drive_cmd;
    PtzController* ptz_controller = nullptr;
};

bool extract_json_string(const std::string& json, const char* key, std::string& out) {
    const std::string k = std::string("\"") + key + "\"";
    const size_t key_pos = json.find(k);
    if (key_pos == std::string::npos) return false;

    const size_t colon = json.find(':', key_pos + k.size());
    if (colon == std::string::npos) return false;
    const size_t q1 = json.find('"', colon + 1);
    if (q1 == std::string::npos) return false;
    const size_t q2 = json.find('"', q1 + 1);
    if (q2 == std::string::npos) return false;

    out.assign(json, q1 + 1, q2 - (q1 + 1));
    return true;
}

bool extract_json_bool(const std::string& json, const char* key, bool& out) {
    const std::string k = std::string("\"") + key + "\"";
    const size_t key_pos = json.find(k);
    if (key_pos == std::string::npos) return false;

    const size_t colon = json.find(':', key_pos + k.size());
    if (colon == std::string::npos) return false;
    const size_t value_pos = json.find_first_not_of(" \t\r\n", colon + 1);
    if (value_pos == std::string::npos) return false;

    if (json.compare(value_pos, 4, "true") == 0) {
        out = true;
        return true;
    }
    if (json.compare(value_pos, 5, "false") == 0) {
        out = false;
        return true;
    }
    return false;
}

bool decode_drive_command(const std::string& cmd, int& left_cmd, int& right_cmd) {
    if (cmd == "forward") {
        left_cmd = 1;
        right_cmd = 1;
        return true;
    }
    if (cmd == "backward") {
        left_cmd = -1;
        right_cmd = -1;
        return true;
    }
    if (cmd == "turn_left") {
        left_cmd = -1;
        right_cmd = 1;
        return true;
    }
    if (cmd == "turn_right") {
        left_cmd = 1;
        right_cmd = -1;
        return true;
    }
    return false;
}

bool is_ptz_command(const std::string& cmd) {
    return cmd == "pan_left" ||
           cmd == "pan_right" ||
           cmd == "tilt_up" ||
           cmd == "tilt_down";
}

void on_connect(struct mosquitto* mosq, void* obj, int rc) {
    if (rc != 0) {
        fprintf(stderr, "[MQTT] connect failed rc=%d\n", rc);
        return;
    }

    auto* rt = static_cast<MqttRuntime*>(obj);
    if (!rt) return;

    fprintf(stderr, "[MQTT] connected\n");
    mosquitto_subscribe(mosq, nullptr, rt->topic.c_str(), 0);
}

void on_message(struct mosquitto*, void* obj, const struct mosquitto_message* msg) {
    if (!obj || !msg || !msg->payload || msg->payloadlen <= 0) return;
    auto* rt = static_cast<MqttRuntime*>(obj);

    const std::string payload((const char*)msg->payload, (size_t)msg->payloadlen);

    std::string type;
    std::string target;
    std::string group;
    std::string command;
    bool active = false;
    if (!extract_json_string(payload, "type", type)) return;
    if (!extract_json_string(payload, "target", target)) return;
    if (!extract_json_string(payload, "group", group)) return;
    if (!extract_json_string(payload, "command", command)) return;
    if (!extract_json_bool(payload, "active", active)) return;
    if (type != "tank_control" || target != "tank") return;

    if (group == "ptz") {
        if (!rt->ptz_controller || !is_ptz_command(command)) return;
        rt->ptz_controller->handle_mqtt_command(command, active);
        fprintf(stderr, "[MQTT] ptz %s: %s\n", active ? "active" : "idle", command.c_str());
        return;
    }

    if (group != "drive") return;

    int left_cmd = 0;
    int right_cmd = 0;
    if (!decode_drive_command(command, left_cmd, right_cmd)) return;

    if (active) {
        tank_drive::command_drive_from(tank_drive::DriveSource::kMqtt, left_cmd, right_cmd);
        rt->active_drive_cmd = command;
        fprintf(stderr, "[MQTT] drive start: %s\n", command.c_str());
        return;
    }

    if (rt->active_drive_cmd == command) {
        tank_drive::stop_from(tank_drive::DriveSource::kMqtt);
        rt->active_drive_cmd.clear();
        fprintf(stderr, "[MQTT] drive stop: %s\n", command.c_str());
    }
}

} // namespace

bool run_mqtt_drive_loop(const MqttConfig& cfg,
                         std::atomic<bool>& running,
                         PtzController* ptz_controller) {
    MqttRuntime runtime;
    runtime.topic = cfg.topic;
    runtime.ptz_controller = ptz_controller;

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("tank_mqtt_drive", true, &runtime);
    if (!mosq) {
        fprintf(stderr, "[MQTT] mosquitto_new failed\n");
        mosquitto_lib_cleanup();
        return false;
    }

    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);

    fprintf(stderr, "[MQTT] host=%s port=%d topic=%s\n", cfg.host.c_str(), cfg.port, cfg.topic.c_str());
    int rc = mosquitto_connect(mosq, cfg.host.c_str(), cfg.port, cfg.keepalive_sec);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[MQTT] connect error: %s\n", mosquitto_strerror(rc));
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return false;
    }

    bool ok = true;
    while (running) {
        rc = mosquitto_loop(mosq, 100, 1);
        if (rc == MOSQ_ERR_CONN_LOST || rc == MOSQ_ERR_NO_CONN) {
            fprintf(stderr, "[MQTT] reconnecting...\n");
            while (running && (rc = mosquitto_reconnect(mosq)) != MOSQ_ERR_SUCCESS) {
                fprintf(stderr, "[MQTT] reconnect failed: %s\n", mosquitto_strerror(rc));
                mosquitto_loop(mosq, 1000, 1);
            }
            continue;
        }
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "[MQTT] loop error: %s\n", mosquitto_strerror(rc));
            ok = false;
            break;
        }
    }

    tank_drive::stop_from(tank_drive::DriveSource::kMqtt);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return ok;
}

