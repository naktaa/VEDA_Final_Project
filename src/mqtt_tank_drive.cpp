#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "camera_capture.hpp"
#include "frame_jpeg_cache.hpp"
#include "http_vr_server.hpp"
#include "mqtt_drive.hpp"
#include "ptz_control.hpp"
#include "rtsp_stream.hpp"
#include "stream_config.hpp"
#include "tank_drive.hpp"

#ifndef TANK_SOURCE_DIR
#define TANK_SOURCE_DIR "."
#endif

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
    fprintf(stderr, "  --no-http-vr            disable HTTP/MJPEG/VR web server\n");
    fprintf(stderr, "  --http-port <port>      HTTP port for /web and /stream.mjpg (default: %d)\n",
            stream_config::DEFAULT_HTTP_PORT);
    fprintf(stderr, "  --i2c-dev <path>        I2C device for PCA9685 (default: %s)\n",
            stream_config::DEFAULT_I2C_DEVICE);
    fprintf(stderr, "  --i2c-addr <n>          I2C address for PCA9685 (default: 0x%02X)\n",
            stream_config::DEFAULT_I2C_ADDRESS);
    fprintf(stderr, "  --pan-channel <n>       PCA9685 channel for pan SG90 (default: %d)\n",
            stream_config::DEFAULT_PAN_CHANNEL);
    fprintf(stderr, "  --tilt-channel <n>      PCA9685 channel for tilt SG90 (default: %d)\n",
            stream_config::DEFAULT_TILT_CHANNEL);
}

ParseResult parse_args(int argc,
                       char* argv[],
                       MqttConfig& mqtt_cfg,
                       RtspConfig& rtsp_cfg,
                       HttpVrConfig& http_cfg,
                       PtzConfig& ptz_cfg) {
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
        } else if (std::strcmp(argv[i], "--no-http-vr") == 0) {
            http_cfg.enable = false;
        } else if (std::strcmp(argv[i], "--http-port") == 0 && i + 1 < argc) {
            http_cfg.port = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--i2c-dev") == 0 && i + 1 < argc) {
            ptz_cfg.i2c_device = argv[++i];
        } else if (std::strcmp(argv[i], "--i2c-addr") == 0 && i + 1 < argc) {
            ptz_cfg.i2c_address = std::strtol(argv[++i], nullptr, 0);
        } else if (std::strcmp(argv[i], "--pan-channel") == 0 && i + 1 < argc) {
            ptz_cfg.pan_channel = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--tilt-channel") == 0 && i + 1 < argc) {
            ptz_cfg.tilt_channel = std::atoi(argv[++i]);
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

    HttpVrConfig http_cfg{
        true,
        stream_config::DEFAULT_HTTP_PORT,
        std::string(TANK_SOURCE_DIR) + "/web/tilt_vr"};

    PtzConfig ptz_cfg{
        stream_config::DEFAULT_I2C_DEVICE,
        stream_config::DEFAULT_I2C_ADDRESS,
        stream_config::DEFAULT_PWM_FREQUENCY_HZ,
        stream_config::DEFAULT_PAN_CHANNEL,
        stream_config::DEFAULT_TILT_CHANNEL,
        stream_config::DEFAULT_PAN_CENTER_DEG,
        stream_config::DEFAULT_PAN_LEFT_DEG,
        stream_config::DEFAULT_PAN_RIGHT_DEG,
        stream_config::DEFAULT_TILT_CENTER_DEG,
        stream_config::DEFAULT_TILT_UP_DEG,
        stream_config::DEFAULT_TILT_DOWN_DEG,
        stream_config::DEFAULT_IMU_PRIORITY_TIMEOUT_MS};

    const ParseResult parse_result = parse_args(argc, argv, mqtt_cfg, rtsp_cfg, http_cfg, ptz_cfg);
    if (parse_result == ParseResult::kHelp) return 0;
    if (parse_result == ParseResult::kError) return 1;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    if (!tank_drive::init()) return 1;
    tank_drive::set_idle_autostop(false);

    PtzController ptz_controller;
    ptz_controller.start(ptz_cfg);

    FrameJpegCache frame_cache;
    CameraCapture camera_capture;
    RtspStreamServer rtsp_server;
    HttpVrServer http_server;

    if (rtsp_cfg.enable) {
        if (!rtsp_server.start(rtsp_cfg)) {
            ptz_controller.stop();
            tank_drive::shutdown();
            return 1;
        }
    }

    if (rtsp_cfg.enable || http_cfg.enable) {
        if (!camera_capture.start(g_running,
                                  rtsp_cfg.enable ? &rtsp_server : nullptr,
                                  http_cfg.enable ? &frame_cache : nullptr)) {
            rtsp_server.stop();
            ptz_controller.stop();
            tank_drive::shutdown();
            return 1;
        }
    }

    if (http_cfg.enable) {
        if (!http_server.start(http_cfg, g_running, frame_cache, ptz_controller)) {
            camera_capture.stop();
            rtsp_server.stop();
            ptz_controller.stop();
            tank_drive::shutdown();
            return 1;
        }
    }

    const bool mqtt_ok = run_mqtt_drive_loop(mqtt_cfg, g_running, &ptz_controller);

    g_running = false;
    if (http_cfg.enable) {
        http_server.stop();
    }
    if (rtsp_cfg.enable || http_cfg.enable) {
        camera_capture.stop();
    }
    rtsp_server.stop();
    ptz_controller.stop();
    tank_drive::shutdown();
    return mqtt_ok ? 0 : 1;
}
