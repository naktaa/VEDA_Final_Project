#include <atomic>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <string>
#include <thread>

#include <opencv2/imgproc.hpp>

#include "app_config.hpp"
#include "gst_camera_capture.hpp"
#include "hybrid_eis.hpp"
#include "libcamera_capture.hpp"
#include "log_utils.hpp"
#include "mqtt_drive.hpp"
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
        std::fprintf(stderr, "[LKONLY] config setup failed: %s\n", error.c_str());
        return 1;
    }
    if (created) {
        std::fprintf(stderr, "[LKONLY] created %s from template. Review it and run again.\n", kLocalConfigPath);
        return 1;
    }

    AppConfig config;
    if (!load_app_config(kLocalConfigPath, config, &error)) {
        std::fprintf(stderr, "[LKONLY] config load failed: %s\n", error.c_str());
        return 1;
    }

    std::ofstream runtime_log;
    std::string runtime_log_path;
    if (!log_utils::open_log_file("main_lkonly_runtime", runtime_log, runtime_log_path, &error)) {
        std::fprintf(stderr, "[LKONLY] log setup failed: %s\n", error.c_str());
        return 1;
    }
    runtime_log << std::fixed << std::setprecision(6);
    runtime_log << "# type=main_lkonly_runtime\n";
    runtime_log << "# started_at=" << log_utils::human_timestamp() << "\n";
    runtime_log << "# rtsp_path=" << config.rtsp.port << config.rtsp.path << "\n";
    runtime_log << "# camera_fps=" << config.camera.fps << "\n";
    runtime_log << "# gyro_disabled=1\n";
    runtime_log << "frame_index\tframe_time_ms\tsensor_ts_ns\texposure_us\tframe_duration_us\t"
                   "gyro_disabled\tstate\tlk_valid\tlk_confidence\tlk_features\tlk_valid_points\tlk_inliers\t"
                   "gyro_valid\tyaw_rate_dps\tcrop_required_percent\tclamp_scale\tvisual_anchor_rad\t"
                   "applied_rot_roll\tapplied_rot_pitch\tapplied_rot_yaw\t"
                   "applied_tx\tapplied_ty\n";
    runtime_log.flush();
    std::fprintf(stderr, "[LKONLY] runtime log: %s\n", runtime_log_path.c_str());

    std::atomic<bool> running{true};
    g_running_ptr = &running;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    tank_drive::init();
    tank_drive::set_idle_autostop(false);

    RtspServer rtsp_server;
    if (!rtsp_server.start(config.camera, config.rtsp, false, &error)) {
        std::fprintf(stderr, "[LKONLY] RTSP start failed: %s\n", error.c_str());
        tank_drive::shutdown();
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
        std::fprintf(stderr, "[LKONLY] camera start failed: %s\n", error.c_str());
        rtsp_server.stop();
        tank_drive::shutdown();
        return 1;
    }
    std::fprintf(stderr, "[LKONLY] capture backend: %s\n", gst_capture_enabled ? "gstreamer" : "libcamera");
    std::fprintf(stderr, "[LKONLY] gyro path disabled; comparing LK-only stabilization\n");

    HybridEisProcessor processor(config, nullptr);

    VrRemoteInputConfig vr_input_config;
    std::thread vr_input_thread([&]() {
        const bool ok = run_vr_remote_input_loop(vr_input_config, running);
        if (!ok && running.load()) {
            std::fprintf(stderr, "[VR] controller loop unavailable; MQTT control remains active\n");
        }
    });

    std::thread tick_thread([&]() {
        while (running.load()) {
            tank_drive::tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });

    std::thread frame_thread([&]() {
        uint64_t logged_frames = 0;
        while (running.load()) {
            CapturedFrame frame;
            std::string frame_error;
            if (!get_frame(frame, &frame_error)) {
                if (running.load()) {
                    std::fprintf(stderr, "[LKONLY] frame pull failed: %s\n", frame_error.c_str());
                }
                running = false;
                break;
            }

            if (config.camera.flip) {
                cv::flip(frame.image, frame.image, -1);
            }

            cv::Mat stabilized;
            HybridEisDebugInfo debug;
            processor.process(frame, stabilized, &debug);
            if (stabilized.empty()) {
                stabilized = frame.image.clone();
            }

            runtime_log
                << frame.frame_index << '\t'
                << frame.frame_time_ms << '\t'
                << frame.sensor_ts_ns << '\t'
                << frame.exposure_us << '\t'
                << frame.frame_duration_us << '\t'
                << 1 << '\t'
                << hybrid_state_str(debug.state) << '\t'
                << (debug.lk_valid ? 1 : 0) << '\t'
                << debug.lk_confidence << '\t'
                << debug.lk_features << '\t'
                << debug.lk_valid_points << '\t'
                << debug.lk_inliers << '\t'
                << 0 << '\t'
                << 0.0 << '\t'
                << debug.crop_required_percent << '\t'
                << debug.clamp_scale << '\t'
                << debug.visual_anchor_rad << '\t'
                << debug.applied_rotation_rad[0] << '\t'
                << debug.applied_rotation_rad[1] << '\t'
                << debug.applied_rotation_rad[2] << '\t'
                << debug.applied_tx << '\t'
                << debug.applied_ty << '\n';
            ++logged_frames;
            if ((logged_frames % 30U) == 0U) {
                runtime_log.flush();
            }

            if (!rtsp_server.push_stabilized(frame, stabilized)) {
                // Same as main: no client attached is normal.
            }
        }
        runtime_log.flush();
    });

    std::fprintf(stderr,
                 "[LKONLY] runtime ready: RTSP %s%s, MQTT %s:%d topic=%s\n",
                 config.rtsp.port.c_str(),
                 config.rtsp.path.c_str(),
                 config.mqtt.host.c_str(),
                 config.mqtt.port,
                 config.mqtt.topic.c_str());

    const bool mqtt_ok = run_mqtt_drive_loop(config.mqtt, running);
    if (!mqtt_ok) {
        std::fprintf(stderr, "[LKONLY] MQTT loop ended with error\n");
    }
    running = false;

    shutdown_capture();

    if (frame_thread.joinable()) {
        frame_thread.join();
    }
    if (vr_input_thread.joinable()) {
        vr_input_thread.join();
    }
    if (tick_thread.joinable()) {
        tick_thread.join();
    }

    rtsp_server.stop();
    tank_drive::shutdown();
    return mqtt_ok ? 0 : 1;
}
