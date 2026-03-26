#include <atomic>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include <mosquitto.h>
#include <opencv2/imgproc.hpp>

#include "app_config.hpp"
#include "frame_jpeg_cache.hpp"
#include "gst_camera_capture.hpp"
#include "http_vr_server.hpp"
#include "libcamera_capture.hpp"
#include "mqtt_drive.hpp"
#include "ptz_control.hpp"
#include "rtsp_server.hpp"
#include "tank_drive.hpp"
#include "vr_remote_input.hpp"

namespace {

std::atomic<bool>* g_running_ptr = nullptr;

void handle_signal(int) {
    if (g_running_ptr) {
        g_running_ptr->store(false);
    }
}

constexpr const char* kTemplateConfigPath = "config_template.ini";
constexpr const char* kLocalConfigPath = "config_local.ini";

bool use_gstreamer_capture(const CameraConfig& config) {
    std::string backend = config.capture_backend;
    std::transform(backend.begin(), backend.end(), backend.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return backend == "gst" || backend == "gstreamer";
}

} // namespace

int main() {
    bool created = false;
    std::string error;
    if (!ensure_local_config_exists(kTemplateConfigPath, kLocalConfigPath, &created, &error)) {
        std::fprintf(stderr, "[RAW] config setup failed: %s\n", error.c_str());
        return 1;
    }
    if (created) {
        std::fprintf(stderr, "[RAW] created %s from template. Review it and run again.\n", kLocalConfigPath);
        return 1;
    }

    AppConfig config;
    if (!load_app_config(kLocalConfigPath, config, &error)) {
        std::fprintf(stderr, "[RAW] config load failed: %s\n", error.c_str());
        return 1;
    }

    std::atomic<bool> running{true};
    g_running_ptr = &running;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    mosquitto_lib_init();

    if (!tank_drive::init()) {
        mosquitto_lib_cleanup();
        return 1;
    }
    tank_drive::set_manual_speed(config.manual_drive.default_pwm);
    tank_drive::set_idle_autostop(false);

    RtspServer rtsp_server;
    if (!rtsp_server.start(config.camera, config.rtsp, false, &error)) {
        std::fprintf(stderr, "[RAW] RTSP start failed: %s\n", error.c_str());
        tank_drive::shutdown();
        mosquitto_lib_cleanup();
        return 1;
    }

    const bool gst_capture_enabled = use_gstreamer_capture(config.camera);
    LibcameraCapture libcamera_capture;
    GstCameraCapture gst_capture;
    auto init_capture = [&](std::string* init_error) {
        return gst_capture_enabled
            ? gst_capture.init(config.camera, init_error)
            : libcamera_capture.init(config.camera, init_error);
    };
    auto get_frame = [&](CapturedFrame& frame, std::string* frame_error) {
        return gst_capture_enabled
            ? gst_capture.get_frame(frame, frame_error)
            : libcamera_capture.get_frame(frame, frame_error);
    };
    auto shutdown_capture = [&]() {
        if (gst_capture_enabled) {
            gst_capture.shutdown();
        } else {
            libcamera_capture.shutdown();
        }
    };

    if (!init_capture(&error)) {
        std::fprintf(stderr, "[RAW] camera start failed: %s\n", error.c_str());
        rtsp_server.stop();
        tank_drive::shutdown();
        mosquitto_lib_cleanup();
        return 1;
    }
    std::fprintf(stderr, "[RAW] capture backend: %s\n", gst_capture_enabled ? "gstreamer" : "libcamera");

    PtzController ptz_controller;
    ptz_controller.start(config.ptz);

    FrameJpegCache frame_cache;
    HttpVrServer http_server;
    if (config.http.enable) {
        if (!http_server.start(config.http, running, frame_cache, ptz_controller)) {
            std::fprintf(stderr, "[RAW] HTTP tiltVR start failed on port %d\n", config.http.port);
            ptz_controller.stop();
            shutdown_capture();
            rtsp_server.stop();
            tank_drive::shutdown();
            mosquitto_lib_cleanup();
            return 1;
        }
    }

    std::thread vr_input_thread;
    bool vr_input_thread_started = false;
    if (config.controller.enabled) {
        VrRemoteInputConfig vr_input_config;
        vr_input_config.input_device = config.controller.input_device;
        vr_input_config.device_name_hint = config.controller.device_name_hint;
        vr_input_config.idle_stop_ms = config.controller.idle_stop_ms;
        vr_input_config.speed_step = config.controller.speed_step;
        vr_input_config.log_only = config.controller.log_only;
        vr_input_thread = std::thread([&, vr_input_config]() {
            const bool ok = run_vr_remote_input_loop(vr_input_config, running);
            if (!ok && running.load()) {
                std::fprintf(stderr, "[VR] controller loop unavailable; Qt control remains active\n");
            }
        });
        vr_input_thread_started = true;
    }

    std::thread tick_thread([&]() {
        while (running.load()) {
            tank_drive::tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });

    std::thread frame_thread([&]() {
        while (running.load()) {
            CapturedFrame frame;
            std::string frame_error;
            if (!get_frame(frame, &frame_error)) {
                if (running.load()) {
                    std::fprintf(stderr, "[RAW] frame pull failed: %s\n", frame_error.c_str());
                }
                running = false;
                break;
            }

            if (config.camera.flip) {
                cv::flip(frame.image, frame.image, -1);
            }

            if (config.http.enable && frame_cache.has_consumers()) {
                const cv::Mat mjpeg_source = frame.image.isContinuous() ? frame.image : frame.image.clone();
                frame_cache.update_bgr_frame(mjpeg_source.data,
                                             mjpeg_source.total() * mjpeg_source.elemSize(),
                                             mjpeg_source.cols,
                                             mjpeg_source.rows);
            }

            if (!rtsp_server.push_stabilized(frame, frame.image)) {
                // No client attached is normal; avoid log spam.
            }
        }
    });

    std::fprintf(stderr,
                 "[RAW] runtime ready: RTSP %s%s, MQTT %s:%d topic=%s\n",
                 config.rtsp.port.c_str(),
                 config.rtsp.path.c_str(),
                 config.mqtt.host.c_str(),
                 config.mqtt.port,
                 config.mqtt.control_topic.c_str());

    const bool mqtt_ok = run_mqtt_drive_loop(config.mqtt, running, &ptz_controller);
    if (!mqtt_ok) {
        std::fprintf(stderr, "[RAW] Qt control loop ended with error\n");
    }
    running = false;

    shutdown_capture();
    http_server.stop();
    ptz_controller.stop();

    if (frame_thread.joinable()) {
        frame_thread.join();
    }
    if (vr_input_thread_started && vr_input_thread.joinable()) {
        vr_input_thread.join();
    }
    if (tick_thread.joinable()) {
        tick_thread.join();
    }

    rtsp_server.stop();
    tank_drive::shutdown();
    mosquitto_lib_cleanup();
    return mqtt_ok ? 0 : 1;
}
