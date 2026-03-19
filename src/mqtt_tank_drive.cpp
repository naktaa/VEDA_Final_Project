#include "tank_drive.hpp"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <mosquitto.h>

namespace {

constexpr const char* kDefaultHost = "127.0.0.1";
constexpr int kDefaultPort = 1883;
constexpr int kDefaultKeepAliveSec = 30;
constexpr const char* kDefaultTopic = "wiserisk/rc/control";
constexpr const char* kDefaultRtspPort = "8555";
constexpr const char* kDefaultRtspPath = "/cam";
constexpr const char* kDefaultRtspLaunch =
    "( libcamerasrc ! video/x-raw,width=640,height=480,framerate=20/1 "
    "! videoflip method=vertical-flip "
    "! videoconvert "
    "! video/x-raw,format=I420 "
    "! v4l2h264enc extra-controls=\"controls,video_bitrate=1500000,h264_i_frame_period=30\" "
    "! video/x-h264,level=(string)4,profile=(string)baseline "
    "! rtph264pay name=pay0 pt=96 config-interval=1 )";

std::atomic<bool> g_running{true};
std::string g_active_drive_cmd;

struct MqttConfig {
    std::string topic = kDefaultTopic;
};

struct RtspConfig {
    bool enable = true;
    std::string service = kDefaultRtspPort;
    std::string path = kDefaultRtspPath;
    std::string launch = kDefaultRtspLaunch;
};

struct RtspRuntime {
    GMainLoop* loop = nullptr;
    GstRTSPServer* server = nullptr;
    std::thread loop_thread;
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
    fprintf(stderr, "Usage: %s [options]\n", prog);
    fprintf(stderr, "  --host <addr>           MQTT broker host (default: 127.0.0.1)\n");
    fprintf(stderr, "  --port <n>              MQTT broker port (default: 1883)\n");
    fprintf(stderr, "  --topic <topic>         MQTT topic (default: wiserisk/rc/control)\n");
    fprintf(stderr, "  --no-rtsp               disable RTSP server\n");
    fprintf(stderr, "  --rtsp-port <port>      RTSP port (default: 8555)\n");
    fprintf(stderr, "  --rtsp-path <path>      RTSP mount path (default: /cam)\n");
    fprintf(stderr, "  --rtsp-launch <launch>  GStreamer RTSP factory launch string\n");
}

bool start_rtsp_server(const RtspConfig& cfg, RtspRuntime& rtsp) {
    if (!cfg.enable) return true;

    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }

    rtsp.server = gst_rtsp_server_new();
    if (!rtsp.server) {
        fprintf(stderr, "[RTSP] server create failed\n");
        return false;
    }

    gst_rtsp_server_set_service(rtsp.server, cfg.service.c_str());

    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(rtsp.server);
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, cfg.launch.c_str());
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    gst_rtsp_mount_points_add_factory(mounts, cfg.path.c_str(), factory);
    g_object_unref(mounts);

    if (gst_rtsp_server_attach(rtsp.server, nullptr) == 0) {
        fprintf(stderr, "[RTSP] attach failed\n");
        g_object_unref(rtsp.server);
        rtsp.server = nullptr;
        return false;
    }

    rtsp.loop = g_main_loop_new(nullptr, FALSE);
    if (!rtsp.loop) {
        fprintf(stderr, "[RTSP] main loop create failed\n");
        g_object_unref(rtsp.server);
        rtsp.server = nullptr;
        return false;
    }

    rtsp.loop_thread = std::thread([&rtsp]() {
        g_main_loop_run(rtsp.loop);
    });

    fprintf(stderr, "[RTSP] stream ready: rtsp://<PI_IP>:%s%s\n", cfg.service.c_str(), cfg.path.c_str());
    return true;
}

void stop_rtsp_server(RtspRuntime& rtsp) {
    if (rtsp.loop) {
        g_main_loop_quit(rtsp.loop);
    }
    if (rtsp.loop_thread.joinable()) {
        rtsp.loop_thread.join();
    }
    if (rtsp.loop) {
        g_main_loop_unref(rtsp.loop);
        rtsp.loop = nullptr;
    }
    if (rtsp.server) {
        g_object_unref(rtsp.server);
        rtsp.server = nullptr;
    }
}

} // namespace

int main(int argc, char* argv[]) {
    const char* host = kDefaultHost;
    int port = kDefaultPort;
    MqttConfig cfg;
    RtspConfig rtsp_cfg;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--host") == 0 && i + 1 < argc) {
            host = argv[++i];
        } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--topic") == 0 && i + 1 < argc) {
            cfg.topic = argv[++i];
        } else if (std::strcmp(argv[i], "--no-rtsp") == 0) {
            rtsp_cfg.enable = false;
        } else if (std::strcmp(argv[i], "--rtsp-port") == 0 && i + 1 < argc) {
            rtsp_cfg.service = argv[++i];
        } else if (std::strcmp(argv[i], "--rtsp-path") == 0 && i + 1 < argc) {
            rtsp_cfg.path = argv[++i];
            if (!rtsp_cfg.path.empty() && rtsp_cfg.path[0] != '/') {
                rtsp_cfg.path.insert(rtsp_cfg.path.begin(), '/');
            }
        } else if (std::strcmp(argv[i], "--rtsp-launch") == 0 && i + 1 < argc) {
            rtsp_cfg.launch = argv[++i];
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

    RtspRuntime rtsp_runtime;
    if (!start_rtsp_server(rtsp_cfg, rtsp_runtime)) {
        tank_drive::shutdown();
        return 1;
    }

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("tank_mqtt_drive", true, &cfg);
    if (!mosq) {
        fprintf(stderr, "[MQTT] mosquitto_new failed\n");
        stop_rtsp_server(rtsp_runtime);
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
        stop_rtsp_server(rtsp_runtime);
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
    stop_rtsp_server(rtsp_runtime);
    tank_drive::shutdown();
    return 0;
}
