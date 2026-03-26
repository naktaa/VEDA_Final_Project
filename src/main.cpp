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

#include <mosquitto.h>
#include <opencv2/imgproc.hpp>

#include "auto_controller.hpp"
#include "app_config.hpp"
#include "gst_camera_capture.hpp"
#include "hybrid_eis.hpp"
#include "imu_reader.hpp"
#include "libcamera_capture.hpp"
#include "log_utils.hpp"
#include "mqtt_drive.hpp"
#include "rc_status_publisher.h"
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
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

bool use_gstreamer_capture(const CameraConfig& config) {
    std::string backend = config.capture_backend;
    std::transform(backend.begin(), backend.end(), backend.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return backend == "gst" || backend == "gstreamer";
}

double average_pwm(const tank_drive::DriveStatusSnapshot& drive) {
    return (static_cast<double>(drive.left_pwm) + static_cast<double>(drive.right_pwm)) * 0.5;
}

bool manual_override_active(const tank_drive::DriveStatusSnapshot& drive) {
    return drive.active_source == tank_drive::DriveSource::kQt ||
           drive.active_source == tank_drive::DriveSource::kController ||
           (drive.active_source == tank_drive::DriveSource::kManualKey && drive.moving);
}

void fill_runtime_status(RcStatus& status,
                         const tank_drive::DriveStatusSnapshot& drive,
                         const AutoStatusSnapshot& auto_state,
                         int publish_interval_ms) {
    status.data_period = std::to_string(publish_interval_ms) + "ms";
    status.speed = 0.0;
    status.mode = "idle";
    status.mission = "none";
    status.robot_state = drive.motor_ready ? "IDLE" : "motor_unavailable";

    if (auto_state.pose.valid) {
        status.x = auto_state.pose.x;
        status.y = auto_state.pose.y;
        status.heading = auto_state.pose.yaw * kRadToDeg;
    } else {
        status.x = -1.0;
        status.y = -1.0;
        status.heading = -1.0;
    }

    if (auto_state.goal.valid) {
        TargetInfo target;
        target.x = auto_state.goal.x;
        target.y = auto_state.goal.y;
        status.target = target;
    } else {
        status.target.reset();
    }

    if (manual_override_active(drive)) {
        status.mode = "manual";
        status.speed = drive.moving ? average_pwm(drive) : 0.0;
        if (drive.active_source == tank_drive::DriveSource::kQt) {
            status.mission = "qt_drive";
            status.robot_state = drive.moving ? "QT_OVERRIDE" : "QT_IDLE";
        } else if (drive.active_source == tank_drive::DriveSource::kController) {
            status.mission = "controller_drive";
            status.robot_state = drive.moving ? "CONTROLLER_OVERRIDE" : "CONTROLLER_IDLE";
        } else {
            status.mission = "keyboard_drive";
            status.robot_state = drive.moving ? "KEYBOARD_DRIVE" : "KEYBOARD_IDLE";
        }
        return;
    }

    if (auto_state.goal.valid || auto_state.pose.valid) {
        status.mode = auto_state.goal.valid ? "auto" : "idle";
        status.mission = auto_state.goal.valid ? "goal_tracking" : "none";
        status.speed = std::max(0.0, auto_state.command.speed_cmps);
        status.robot_state = auto_state.control_status.robot_state;
    }
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
    runtime_log << "# rtsp_raw_path=" << config.rtsp.port << config.rtsp.raw_path << "\n";
    runtime_log << "# camera_fps=" << config.camera.fps << "\n";
    runtime_log << "# imu_target_hz=" << config.imu.target_hz << "\n";
    runtime_log << "# imu_use_fifo=" << (config.imu.use_fifo ? 1 : 0) << "\n";
    runtime_log << "# imu_int_gpio_chip=" << config.imu.int_gpio_chip << "\n";
    runtime_log << "# imu_int_line_offset=" << config.imu.int_line_offset << "\n";
    runtime_log << "# imu_int_pin_wpi=" << config.imu.int_pin_wpi << "\n";
    runtime_log << "# calib_imu_offset_ms=" << config.calib.imu_offset_ms << "\n";
    runtime_log << "frame_index\tframe_time_ms\tsensor_ts_ns\texposure_us\tframe_duration_us\t"
                   "imu_offset_ms\timu_actual_hz\timu_sample_time_ms\t"
                   "gyro_target_time_ms\tgyro_latest_sample_time_ms\tgyro_latest_lag_ms\t"
                   "gyro_range_used\tgyro_range_min_ms\tgyro_range_max_ms\t"
                   "gyro_covers_start\tgyro_covers_end\t"
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

    mosquitto_lib_init();

    if (!tank_drive::init()) {
        mosquitto_lib_cleanup();
        return 1;
    }
    tank_drive::set_manual_speed(config.manual_drive.default_pwm);
    tank_drive::set_idle_autostop(false);

    RtspServer rtsp_server;
    if (!rtsp_server.start(config.camera, config.rtsp, true, &error)) {
        std::fprintf(stderr, "[MAIN] RTSP start failed: %s\n", error.c_str());
        tank_drive::shutdown();
        mosquitto_lib_cleanup();
        return 1;
    }

    ImuReader imu_reader;
    if (!imu_reader.start(config.imu, config.calib, true, &error)) {
        std::fprintf(stderr, "[MAIN] IMU start failed: %s\n", error.c_str());
        rtsp_server.stop();
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
        std::fprintf(stderr, "[MAIN] camera start failed: %s\n", error.c_str());
        imu_reader.stop();
        rtsp_server.stop();
        tank_drive::shutdown();
        mosquitto_lib_cleanup();
        return 1;
    }
    std::fprintf(stderr, "[MAIN] capture backend: %s\n", gst_capture_enabled ? "gstreamer" : "libcamera");

    HybridEisProcessor processor(config, &imu_reader.buffer());

    AutoController auto_controller(config);
    if (!auto_controller.start(&error)) {
        std::fprintf(stderr, "[MAIN] auto controller start failed: %s\n", error.c_str());
        shutdown_capture();
        imu_reader.stop();
        rtsp_server.stop();
        tank_drive::shutdown();
        mosquitto_lib_cleanup();
        return 1;
    }

    std::thread auto_thread([&]() {
        auto_controller.run(&running);
    });

    RcStatusPublisher::Config status_config;
    status_config.broker_host = config.mqtt.host;
    status_config.broker_port = config.mqtt.port;
    status_config.topic = config.mqtt.status_topic;
    status_config.client_id = "tank_merge_status";
    status_config.publish_interval_ms = config.mqtt.status_publish_interval_ms;
    status_config.keepalive_sec = config.mqtt.keepalive_sec;
    status_config.qos = 1;
    status_config.retain = true;
    RcStatusPublisher status_publisher(status_config);
    status_publisher.setStatusProvider([&](RcStatus& status) {
        fill_runtime_status(
            status,
            tank_drive::get_status_snapshot(),
            auto_controller.snapshot(),
            config.mqtt.status_publish_interval_ms);
    });

    bool status_thread_started = false;
    std::thread status_thread;
    if (status_publisher.start()) {
        status_thread = std::thread([&]() {
            while (running.load()) {
                status_publisher.spinOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        });
        status_thread_started = true;
    } else {
        std::fprintf(stderr, "[MAIN] status publisher disabled due to startup failure\n");
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
                std::fprintf(stderr, "[VR] controller loop unavailable; Qt/auto control remains active\n");
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
                << debug.gyro_target_time_ms << '\t'
                << debug.gyro_latest_sample_time_ms << '\t'
                << debug.gyro_latest_lag_ms << '\t'
                << debug.gyro_range_used << '\t'
                << debug.gyro_range_min_ms << '\t'
                << debug.gyro_range_max_ms << '\t'
                << (debug.gyro_covers_start ? 1 : 0) << '\t'
                << (debug.gyro_covers_end ? 1 : 0) << '\t'
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

            rtsp_server.push_raw(frame, frame.image);
            if (!rtsp_server.push_stabilized(frame, stabilized)) {
                // Same as above.
            }
        }
        runtime_log.flush();
    });

    std::fprintf(stderr,
                 "[MAIN] runtime ready: RTSP %s%s (stab), %s%s (raw), MQTT %s:%d topic=%s\n",
                 config.rtsp.port.c_str(),
                 config.rtsp.path.c_str(),
                 config.rtsp.port.c_str(),
                 config.rtsp.raw_path.c_str(),
                 config.mqtt.host.c_str(),
                 config.mqtt.port,
                 config.mqtt.control_topic.c_str());

    const bool mqtt_ok = run_mqtt_drive_loop(config.mqtt, running);
    if (!mqtt_ok) {
        std::fprintf(stderr, "[MAIN] Qt control loop ended with error\n");
    }
    running = false;

    shutdown_capture();

    if (frame_thread.joinable()) {
        frame_thread.join();
    }
    if (vr_input_thread_started && vr_input_thread.joinable()) {
        vr_input_thread.join();
    }
    if (tick_thread.joinable()) {
        tick_thread.join();
    }
    if (auto_thread.joinable()) {
        auto_thread.join();
    }
    if (status_thread_started && status_thread.joinable()) {
        status_thread.join();
    }

    status_publisher.stop();
    auto_controller.stop();
    imu_reader.stop();
    rtsp_server.stop();
    tank_drive::shutdown();
    mosquitto_lib_cleanup();
    return mqtt_ok ? 0 : 1;
}
