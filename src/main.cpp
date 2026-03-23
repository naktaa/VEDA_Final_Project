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
#include "imu_reader.hpp"
#include "libcamera_capture.hpp"
#include "log_utils.hpp"
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
        std::fprintf(stderr, "[MAIN] config setup failed: %s\n", error.c_str());
        return 1;
    }
    if (created) {
        std::fprintf(stderr, "[MAIN] created %s from template. Review it and run again.\n", kLocalConfigPath);
        return 1;
    }

    AppConfig config;
    if (!load_app_config(kLocalConfigPath, config, &error)) {
        std::fprintf(stderr, "[MAIN] config load failed: %s\n", error.c_str());
        return 1;
    }

    std::ofstream runtime_log;
    std::string runtime_log_path;
    if (!log_utils::open_log_file("main_runtime", runtime_log, runtime_log_path, &error)) {
        std::fprintf(stderr, "[MAIN] log setup failed: %s\n", error.c_str());
        return 1;
    }
    runtime_log << std::fixed << std::setprecision(6);
    runtime_log << "# type=main_runtime\n";
    runtime_log << "# started_at=" << log_utils::human_timestamp() << "\n";
    runtime_log << "# rtsp_path=" << config.rtsp.port << config.rtsp.path << "\n";
    runtime_log << "# camera_fps=" << config.camera.fps << "\n";
    runtime_log << "# imu_target_hz=" << config.imu.target_hz << "\n";
    runtime_log << "# imu_use_fifo=" << (config.imu.use_fifo ? 1 : 0) << "\n";
    runtime_log << "# imu_int_pin_wpi=" << config.imu.int_pin_wpi << "\n";
    runtime_log << "# calib_imu_offset_ms=" << config.calib.imu_offset_ms << "\n";
    runtime_log << "frame_index\tframe_time_ms\tsensor_ts_ns\texposure_us\tframe_duration_us\t"
                   "imu_offset_ms\timu_actual_hz\timu_sample_time_ms\t"
                   "imu_raw_x\timu_raw_y\timu_raw_z\t"
                   "imu_roll_rad_s\timu_pitch_rad_s\timu_yaw_rad_s\t"
                   "state\tlk_valid\tlk_confidence\tlk_features\tlk_valid_points\tlk_inliers\t"
                   "gyro_valid\tyaw_rate_dps\tcrop_required_percent\tclamp_scale\tvisual_anchor_rad\t"
                   "gyro_delta_raw_roll\tgyro_delta_raw_pitch\tgyro_delta_raw_yaw\t"
                   "gyro_delta_hp_roll\tgyro_delta_hp_pitch\tgyro_delta_hp_yaw\t"
                   "applied_rot_roll\tapplied_rot_pitch\tapplied_rot_yaw\t"
                   "applied_tx\tapplied_ty\n";
    runtime_log.flush();
    std::fprintf(stderr, "[MAIN] runtime log: %s\n", runtime_log_path.c_str());

    std::atomic<bool> running{true};
    g_running_ptr = &running;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    tank_drive::init();
    tank_drive::set_idle_autostop(true, 200);

    RtspServer rtsp_server;
    if (!rtsp_server.start(config.camera, config.rtsp, false, &error)) {
        std::fprintf(stderr, "[MAIN] RTSP start failed: %s\n", error.c_str());
        tank_drive::shutdown();
        return 1;
    }

    ImuReader imu_reader;
    if (!imu_reader.start(config.imu, config.calib, true, &error)) {
        std::fprintf(stderr, "[MAIN] IMU start failed: %s\n", error.c_str());
        rtsp_server.stop();
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
        std::fprintf(stderr, "[MAIN] camera start failed: %s\n", error.c_str());
        imu_reader.stop();
        rtsp_server.stop();
        tank_drive::shutdown();
        return 1;
    }
    std::fprintf(stderr, "[MAIN] capture backend: %s\n", gst_capture_enabled ? "gstreamer" : "libcamera");

    HybridEisProcessor processor(config, &imu_reader.buffer());

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
                    std::fprintf(stderr, "[MAIN] frame pull failed: %s\n", frame_error.c_str());
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

            ImuSample latest_imu;
            const bool have_imu = imu_reader.latest_sample(latest_imu);
            runtime_log
                << frame.frame_index << '\t'
                << frame.frame_time_ms << '\t'
                << frame.sensor_ts_ns << '\t'
                << frame.exposure_us << '\t'
                << frame.frame_duration_us << '\t'
                << config.calib.imu_offset_ms << '\t'
                << imu_reader.actual_hz() << '\t'
                << (have_imu ? latest_imu.sample_time_ms : -1.0) << '\t'
                << (have_imu ? latest_imu.raw_counts[0] : 0.0) << '\t'
                << (have_imu ? latest_imu.raw_counts[1] : 0.0) << '\t'
                << (have_imu ? latest_imu.raw_counts[2] : 0.0) << '\t'
                << (have_imu ? latest_imu.gyro_rad_s[0] : 0.0) << '\t'
                << (have_imu ? latest_imu.gyro_rad_s[1] : 0.0) << '\t'
                << (have_imu ? latest_imu.gyro_rad_s[2] : 0.0) << '\t'
                << hybrid_state_str(debug.state) << '\t'
                << (debug.lk_valid ? 1 : 0) << '\t'
                << debug.lk_confidence << '\t'
                << debug.lk_features << '\t'
                << debug.lk_valid_points << '\t'
                << debug.lk_inliers << '\t'
                << (debug.gyro_valid ? 1 : 0) << '\t'
                << debug.yaw_rate_dps << '\t'
                << debug.crop_required_percent << '\t'
                << debug.clamp_scale << '\t'
                << debug.visual_anchor_rad << '\t'
                << debug.gyro_delta_raw_rad[0] << '\t'
                << debug.gyro_delta_raw_rad[1] << '\t'
                << debug.gyro_delta_raw_rad[2] << '\t'
                << debug.gyro_delta_hp_rad[0] << '\t'
                << debug.gyro_delta_hp_rad[1] << '\t'
                << debug.gyro_delta_hp_rad[2] << '\t'
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
                // Same as above.
            }
        }
        runtime_log.flush();
    });

    std::fprintf(stderr,
                 "[MAIN] runtime ready: RTSP %s%s, MQTT %s:%d topic=%s\n",
                 config.rtsp.port.c_str(),
                 config.rtsp.path.c_str(),
                 config.mqtt.host.c_str(),
                 config.mqtt.port,
                 config.mqtt.topic.c_str());

    const bool mqtt_ok = run_mqtt_drive_loop(config.mqtt, running);
    if (!mqtt_ok) {
        std::fprintf(stderr, "[MAIN] MQTT loop ended with error\n");
    }
    running = false;

    shutdown_capture();

    if (frame_thread.joinable()) {
        frame_thread.join();
    }
    if (tick_thread.joinable()) {
        tick_thread.join();
    }

    imu_reader.stop();
    rtsp_server.stop();
    tank_drive::shutdown();
    return mqtt_ok ? 0 : 1;
}
