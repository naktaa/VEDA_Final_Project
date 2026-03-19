#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "mqtt_drive.hpp"
#include "rtsp_stream.hpp"
#include "stream_config.hpp"
#include "tank_drive.hpp"

namespace {

std::atomic<bool> g_running{true};

enum class ParseResult {
    kOk,
    kHelp,
    kError
};

void signal_handler(int) {
    g_running = false;
}

void print_usage(const char* prog) {
    fprintf(stderr, "Usage: %s [options]\n", prog);
    fprintf(stderr, "  --host <addr>           MQTT broker host (default: %s)\n",
            stream_config::DEFAULT_MQTT_HOST);
    fprintf(stderr, "  --port <n>              MQTT broker port (default: %d)\n",
            stream_config::DEFAULT_MQTT_PORT);
    fprintf(stderr, "  --topic <topic>         MQTT topic (default: %s)\n",
            stream_config::DEFAULT_MQTT_TOPIC);
    fprintf(stderr, "  --no-rtsp               disable RTSP server\n");
    fprintf(stderr, "  --rtsp-port <port>      RTSP port (default: %s)\n",
            stream_config::DEFAULT_RTSP_PORT);
    fprintf(stderr, "  --rtsp-path <path>      RTSP mount path (default: %s)\n",
            stream_config::DEFAULT_RTSP_PATH);
    fprintf(stderr, "  --rtsp-launch <launch>  GStreamer RTSP factory launch string\n");
}

ParseResult parse_args(int argc, char* argv[], MqttConfig& mqtt_cfg, RtspConfig& rtsp_cfg) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--host") == 0 && i + 1 < argc) {
            mqtt_cfg.host = argv[++i];
        } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            mqtt_cfg.port = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--topic") == 0 && i + 1 < argc) {
            mqtt_cfg.topic = argv[++i];
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
            return ParseResult::kHelp;
        } else {
            fprintf(stderr, "[ERR] Unknown arg: %s\n", argv[i]);
            print_usage(argv[0]);
            return ParseResult::kError;
        }
    }
    return ParseResult::kOk;
}

} // namespace

int main(int argc, char* argv[]) {
    MqttConfig mqtt_cfg{
        stream_config::DEFAULT_MQTT_HOST,
        stream_config::DEFAULT_MQTT_PORT,
        stream_config::DEFAULT_MQTT_KEEPALIVE_SEC,
        stream_config::DEFAULT_MQTT_TOPIC};

    RtspConfig rtsp_cfg{
        true,
        stream_config::DEFAULT_RTSP_PORT,
        stream_config::DEFAULT_RTSP_PATH,
        stream_config::make_default_rtsp_launch()};

    const ParseResult parse_result = parse_args(argc, argv, mqtt_cfg, rtsp_cfg);
    if (parse_result == ParseResult::kHelp) return 0;
    if (parse_result == ParseResult::kError) return 1;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    if (!tank_drive::init()) return 1;
    tank_drive::set_idle_autostop(false);

    RtspStreamServer rtsp_server;
    if (!rtsp_server.start(rtsp_cfg)) {
        tank_drive::shutdown();
        return 1;
    }

    const bool mqtt_ok = run_mqtt_drive_loop(mqtt_cfg, g_running);

    rtsp_server.stop();
    tank_drive::shutdown();
    return mqtt_ok ? 0 : 1;
}
