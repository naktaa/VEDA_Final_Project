#include "tank_drive.hpp"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <mosquitto.h>

namespace {

constexpr const char* kDefaultHost = "127.0.0.1";
constexpr int kDefaultPort = 1883;
constexpr int kDefaultKeepAliveSec = 30;
constexpr const char* kDefaultTopic = "wiserisk/rc/control";

std::atomic<bool> g_running{true};
std::string g_active_drive_cmd;

struct MqttConfig {
    std::string topic = kDefaultTopic;
};

void sigint_handler(int) {
    g_running = false;
}

bool extract_json_string(const std::string& json, const char* key, std::string& out) {
    const std::string k = std::string("\"") + key + "\"";
    const size_t key_pos = json.find(k);
    if (key_pos == std::string::npos) return false;

    size_t colon = json.find(':', key_pos + k.size());
    if (colon == std::string::npos) return false;
    size_t q1 = json.find('"', colon + 1);
    if (q1 == std::string::npos) return false;
    size_t q2 = json.find('"', q1 + 1);
    if (q2 == std::string::npos) return false;

    out.assign(json, q1 + 1, q2 - (q1 + 1));
    return true;
}

bool extract_json_bool(const std::string& json, const char* key, bool& out) {
    const std::string k = std::string("\"") + key + "\"";
    const size_t key_pos = json.find(k);
    if (key_pos == std::string::npos) return false;

    size_t colon = json.find(':', key_pos + k.size());
    if (colon == std::string::npos) return false;
    size_t value_pos = json.find_first_not_of(" \t\r\n", colon + 1);
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

void on_connect(struct mosquitto* mosq, void* obj, int rc) {
    if (rc != 0) {
        fprintf(stderr, "[MQTT] connect failed rc=%d\n", rc);
        return;
    }
    fprintf(stderr, "[MQTT] connected\n");
    auto* cfg = static_cast<MqttConfig*>(obj);
    if (!cfg) return;
    mosquitto_subscribe(mosq, nullptr, cfg->topic.c_str(), 0);
}

void on_message(struct mosquitto*, void*, const struct mosquitto_message* msg) {
    if (!msg || !msg->payload || msg->payloadlen <= 0) return;

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
    if (group != "drive") return;

    int left_cmd = 0;
    int right_cmd = 0;
    if (!decode_drive_command(command, left_cmd, right_cmd)) return;

    if (active) {
        tank_drive::command_drive(left_cmd, right_cmd);
        g_active_drive_cmd = command;
        fprintf(stderr, "[MQTT] drive start: %s\n", command.c_str());
        return;
    }

    if (g_active_drive_cmd == command) {
        tank_drive::stop();
        g_active_drive_cmd.clear();
        fprintf(stderr, "[MQTT] drive stop: %s\n", command.c_str());
    }
}

void print_usage(const char* prog) {
    fprintf(stderr, "Usage: %s [--host <addr>] [--port <n>] [--topic <topic>]\n", prog);
}

} // namespace

int main(int argc, char* argv[]) {
    const char* host = kDefaultHost;
    int port = kDefaultPort;
    MqttConfig cfg;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--host") == 0 && i + 1 < argc) {
            host = argv[++i];
        } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--topic") == 0 && i + 1 < argc) {
            cfg.topic = argv[++i];
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "[ERR] Unknown arg: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    if (!tank_drive::init()) return 1;
    tank_drive::set_idle_autostop(false);

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("tank_mqtt_drive", true, &cfg);
    if (!mosq) {
        fprintf(stderr, "[MQTT] mosquitto_new failed\n");
        tank_drive::shutdown();
        mosquitto_lib_cleanup();
        return 1;
    }

    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);

    fprintf(stderr, "[MQTT] host=%s port=%d topic=%s\n", host, port, cfg.topic.c_str());
    int rc = mosquitto_connect(mosq, host, port, kDefaultKeepAliveSec);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[MQTT] connect error: %s\n", mosquitto_strerror(rc));
        mosquitto_destroy(mosq);
        tank_drive::shutdown();
        mosquitto_lib_cleanup();
        return 1;
    }

    while (g_running) {
        rc = mosquitto_loop(mosq, 100, 1);
        if (rc == MOSQ_ERR_CONN_LOST || rc == MOSQ_ERR_NO_CONN) {
            fprintf(stderr, "[MQTT] reconnecting...\n");
            while (g_running && (rc = mosquitto_reconnect(mosq)) != MOSQ_ERR_SUCCESS) {
                fprintf(stderr, "[MQTT] reconnect failed: %s\n", mosquitto_strerror(rc));
                mosquitto_loop(mosq, 1000, 1);
            }
            continue;
        }
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "[MQTT] loop error: %s\n", mosquitto_strerror(rc));
            break;
        }
    }

    tank_drive::stop();
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    tank_drive::shutdown();
    return 0;
}
