#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include <opencv2/imgproc.hpp>

#include "app_config.hpp"
#include "libcamera_capture.hpp"
#include "mqtt_drive.hpp"
#include "rtsp_server.hpp"
#include "tank_drive.hpp"

namespace {

std::atomic<bool>* g_running_ptr = nullptr;

void handle_signal(int) {
    if (g_running_ptr) {
        g_running_ptr->store(false);
    }
}

constexpr const char* kTemplateConfigPath = "config_template.ini";
constexpr const char* kLocalConfigPath = "config_local.ini";

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

    tank_drive::init();
    tank_drive::set_idle_autostop(true, 200);

    RtspServer rtsp_server;
    if (!rtsp_server.start(config.camera, config.rtsp, false, &error)) {
        std::fprintf(stderr, "[RAW] RTSP start failed: %s\n", error.c_str());
        tank_drive::shutdown();
        return 1;
    }

    LibcameraCapture capture;
    if (!capture.init(config.camera, &error)) {
        std::fprintf(stderr, "[RAW] camera start failed: %s\n", error.c_str());
        rtsp_server.stop();
        tank_drive::shutdown();
        return 1;
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
            if (!capture.get_frame(frame, &frame_error)) {
                if (running.load()) {
                    std::fprintf(stderr, "[RAW] frame pull failed: %s\n", frame_error.c_str());
                }
                running = false;
                break;
            }

            if (config.camera.flip) {
                cv::flip(frame.image, frame.image, -1);
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
                 config.mqtt.topic.c_str());

    const bool mqtt_ok = run_mqtt_drive_loop(config.mqtt, running);
    if (!mqtt_ok) {
        std::fprintf(stderr, "[RAW] MQTT loop ended with error\n");
    }
    running = false;

    capture.shutdown();

    if (frame_thread.joinable()) {
        frame_thread.join();
    }
    if (tick_thread.joinable()) {
        tick_thread.join();
    }

    rtsp_server.stop();
    tank_drive::shutdown();
    return mqtt_ok ? 0 : 1;
}
