#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <mosquitto.h>

#include "camera_capture.hpp"
#include "frame_jpeg_cache.hpp"
#include "http_vr_server.hpp"
#include "mqtt_drive.hpp"
#include "ptz_control.hpp"
#include "rc_status_publisher.h"
#include "rtsp_stream.hpp"
#include "stream_config.hpp"
#include "tank_drive.hpp"
#include "vr_remote_input.hpp"
#include "vr_ui_command_bridge.hpp"

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

const char* drive_source_to_cstr(tank_drive::DriveSource source) {
    switch (source) {
    case tank_drive::DriveSource::kManualKey:
        return "manual";
    case tank_drive::DriveSource::kMqtt:
        return "mqtt";
    case tank_drive::DriveSource::kVrRemote:
        return "controller";
    }
    return "unknown";
}

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
    fprintf(stderr, "  --vr-input-dev <path>   controller evdev path override (default enabled)\n");
    fprintf(stderr, "  --vr-idle-stop-ms <n>   controller auto-stop timeout in ms (default: 180)\n");
    fprintf(stderr, "  --vr-speed-step <n>     controller speed adjustment step (default: 10)\n");
    fprintf(stderr, "  --vr-log-only           print controller commands without motor control\n");
    fprintf(stderr, "  --no-status-pub         disable RC status MQTT publisher\n");
    fprintf(stderr, "  --status-topic <topic>  RC status topic (default: wiserisk/rc/status)\n");
    fprintf(stderr, "  --status-interval-ms <n> RC status publish interval in ms (default: 500)\n");
}

ParseResult parse_args(int argc,
                       char* argv[],
                       MqttConfig& mqtt_cfg,
                       RtspConfig& rtsp_cfg,
                       HttpVrConfig& http_cfg,
                       PtzConfig& ptz_cfg,
                       bool& vr_input_enabled,
                       VrRemoteInputConfig& vr_input_cfg,
                       bool& status_publish_enabled,
                       RcStatusPublisher::Config& status_cfg) {
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
        } else if (std::strcmp(argv[i], "--vr-input-dev") == 0 && i + 1 < argc) {
            vr_input_enabled = true;
            vr_input_cfg.input_device = argv[++i];
        } else if (std::strcmp(argv[i], "--vr-idle-stop-ms") == 0 && i + 1 < argc) {
            vr_input_enabled = true;
            vr_input_cfg.idle_stop_ms = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--vr-speed-step") == 0 && i + 1 < argc) {
            vr_input_enabled = true;
            vr_input_cfg.speed_step = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--vr-log-only") == 0) {
            vr_input_enabled = true;
            vr_input_cfg.log_only = true;
        } else if (std::strcmp(argv[i], "--no-status-pub") == 0) {
            status_publish_enabled = false;
        } else if (std::strcmp(argv[i], "--status-topic") == 0 && i + 1 < argc) {
            status_cfg.topic = argv[++i];
        } else if (std::strcmp(argv[i], "--status-interval-ms") == 0 && i + 1 < argc) {
            status_cfg.publish_interval_ms = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return ParseResult::kHelp;
        } else {
            fprintf(stderr, "[ERR] Unknown arg: %s\n", argv[i]);
            print_usage(argv[0]);
            return ParseResult::kError;
        }
    }

    if (vr_input_cfg.idle_stop_ms < 0) {
        fprintf(stderr, "[VR] idle timeout must be >= 0\n");
        return ParseResult::kError;
    }
    if (vr_input_cfg.speed_step < 1) {
        fprintf(stderr, "[VR] speed step must be >= 1\n");
        return ParseResult::kError;
    }
    if (status_cfg.publish_interval_ms < 1) {
        fprintf(stderr, "[STATUS] publish interval must be >= 1\n");
        return ParseResult::kError;
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

    bool vr_input_enabled = true;
    VrRemoteInputConfig vr_input_cfg;
    VrUiCommandBridge vr_ui_command_bridge;
    bool status_publish_enabled = true;
    RcStatusPublisher::Config status_cfg;

    vr_input_cfg.ui_action_callback = [&](VrUiAction action) {
        switch (action) {
        case VrUiAction::kSessionButton:
            vr_ui_command_bridge.handle_session_button();
            break;
        case VrUiAction::kZeroCalibrate:
            vr_ui_command_bridge.request_zero_calibrate();
            break;
        }
    };

    const ParseResult parse_result = parse_args(argc, argv, mqtt_cfg, rtsp_cfg, http_cfg, ptz_cfg,
                                                vr_input_enabled, vr_input_cfg,
                                                status_publish_enabled, status_cfg);
    if (parse_result == ParseResult::kHelp) return 0;
    if (parse_result == ParseResult::kError) return 1;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    mosquitto_lib_init();

    if (!tank_drive::init()) {
        mosquitto_lib_cleanup();
        return 1;
    }
    tank_drive::set_idle_autostop(false);

    PtzController ptz_controller;
    ptz_controller.start(ptz_cfg);

    FrameJpegCache frame_cache;
    CameraCapture camera_capture;
    RtspStreamServer rtsp_server;
    HttpVrServer http_server;
    std::thread vr_input_thread;
    bool vr_input_thread_started = false;
    std::unique_ptr<RcStatusPublisher> status_publisher;
    std::thread status_publish_thread;
    bool status_publish_thread_started = false;

    if (rtsp_cfg.enable) {
        if (!rtsp_server.start(rtsp_cfg)) {
            ptz_controller.stop();
            tank_drive::shutdown();
            mosquitto_lib_cleanup();
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
            mosquitto_lib_cleanup();
            return 1;
        }
    }

    if (http_cfg.enable) {
        if (!http_server.start(http_cfg, g_running, frame_cache, ptz_controller, vr_ui_command_bridge)) {
            camera_capture.stop();
            rtsp_server.stop();
            ptz_controller.stop();
            tank_drive::shutdown();
            mosquitto_lib_cleanup();
            return 1;
        }
    }

    if (vr_input_enabled) {
        vr_input_thread = std::thread([&]() {
            const bool ok = run_vr_remote_input_loop(vr_input_cfg, g_running);
            if (!ok && g_running.load()) {
                fprintf(stderr, "[VR] controller loop exited; MQTT/HTTP control remains active\n");
                fflush(stderr);
            }
        });
        vr_input_thread_started = true;
    }

    if (status_publish_enabled) {
        status_cfg.broker_host = mqtt_cfg.host;
        status_cfg.broker_port = mqtt_cfg.port;
        status_cfg.keepalive_sec = mqtt_cfg.keepalive_sec;
        status_cfg.client_id = "tank_status_publisher";

        status_publisher = std::make_unique<RcStatusPublisher>(status_cfg);
        status_publisher->setStatusProvider([&](RcStatus& status) {
            const auto drive_status = tank_drive::get_status_snapshot();
            const auto ptz_status = ptz_controller.latest_status();

            status.mode = drive_source_to_cstr(drive_status.active_source);
            status.mission = ptz_status.mode;
            status.speed = drive_status.moving ? static_cast<double>(drive_status.pwm) : 0.0;
            status.battery = -1.0;
            status.x = -1.0;
            status.y = -1.0;
            status.heading = -1.0;
            status.robot_state = drive_status.motor_ready
                ? (drive_status.moving ? "moving" : "idle")
                : "motor_unavailable";
            status.data_period = std::to_string(status_cfg.publish_interval_ms) + "ms";
        });

        if (status_publisher->start()) {
            status_publish_thread = std::thread([&]() {
                while (g_running.load()) {
                    status_publisher->spinOnce();
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            });
            status_publish_thread_started = true;
        } else {
            fprintf(stderr, "[STATUS] publisher start failed; continuing without status publish\n");
            fflush(stderr);
            status_publisher.reset();
        }
    }

    const bool mqtt_ok = run_mqtt_drive_loop(mqtt_cfg, g_running, &ptz_controller);

    g_running = false;
    if (vr_input_thread_started && vr_input_thread.joinable()) {
        vr_input_thread.join();
    }
    if (status_publish_thread_started && status_publish_thread.joinable()) {
        status_publish_thread.join();
    }
    if (status_publisher) {
        status_publisher->stop();
    }
    if (http_cfg.enable) {
        http_server.stop();
    }
    if (rtsp_cfg.enable || http_cfg.enable) {
        camera_capture.stop();
    }
    rtsp_server.stop();
    ptz_controller.stop();
    tank_drive::shutdown();
    mosquitto_lib_cleanup();
    return mqtt_ok ? 0 : 1;
}
