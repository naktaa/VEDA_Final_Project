#include <atomic>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <string>
#include <thread>
#include <vector>

#include <mosquitto.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "auto_controller.hpp"
#include "app_config.hpp"
#include "frame_jpeg_cache.hpp"
#include "gst_camera_capture.hpp"
#include "http_vr_server.hpp"
#include "libcamera_capture.hpp"
#include "log_utils.hpp"
#include "mqtt_drive.hpp"
#include "ptz_control.hpp"
#include "rc_status_publisher.h"
#include "rtsp_server.hpp"
#include "system_usage_monitor.hpp"
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

constexpr int kMaxFeatures = 200;
constexpr double kFeatureQuality = 0.01;
constexpr double kFeatureMinDist = 30.0;
constexpr int kBorderCropPx = 30;
constexpr bool kDebugOverlay = true;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

struct KalmanState {
    double x = 0.0;
    double p = 1.0;
    double q = 0.0;
    double r = 0.0;
    double sum = 0.0;
};

void kalman_init(KalmanState& kf, double q, double r) {
    kf.x = 0.0;
    kf.p = 1.0;
    kf.q = q;
    kf.r = r;
    kf.sum = 0.0;
}

void kalman_update(KalmanState& kf, double measurement) {
    kf.sum += measurement;
    const double x_pred = kf.x;
    const double p_pred = kf.p + kf.q;
    const double k = p_pred / (p_pred + kf.r);
    kf.x = x_pred + k * (kf.sum - x_pred);
    kf.p = (1.0 - k) * p_pred;
}

double kalman_diff(const KalmanState& kf) {
    return kf.x - kf.sum;
}

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

const char* drive_source_name(tank_drive::DriveSource source) {
    switch (source) {
    case tank_drive::DriveSource::kQt:
        return "qt";
    case tank_drive::DriveSource::kController:
        return "controller";
    case tank_drive::DriveSource::kManualKey:
        return "manual_key";
    case tank_drive::DriveSource::kAuto:
        return "auto";
    }
    return "unknown";
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
    runtime_log << "# rtsp_stab_path=" << config.rtsp.port << config.rtsp.path << "\n";
    runtime_log << "# rtsp_raw_path=" << config.rtsp.port << config.rtsp.raw_path << "\n";
    runtime_log << "# camera_fps=" << config.camera.fps << "\n";
    runtime_log << "# lk_mode=reference_tae\n";
    runtime_log << "frame_index\tframe_time_ms\tsensor_ts_ns\texposure_us\tframe_duration_us\t"
                   "features\tvalid_points\tlk_ok\traw_dx\traw_dy\traw_da\t"
                   "corr_dx\tcorr_dy\tcorr_da\tcrop_x_px\tcrop_y_px\n";
    runtime_log.flush();
    std::fprintf(stderr, "[LKONLY] runtime log: %s\n", runtime_log_path.c_str());

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
        std::fprintf(stderr, "[LKONLY] RTSP start failed: %s\n", error.c_str());
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
        std::fprintf(stderr, "[LKONLY] camera start failed: %s\n", error.c_str());
        rtsp_server.stop();
        tank_drive::shutdown();
        mosquitto_lib_cleanup();
        return 1;
    }
    std::fprintf(stderr, "[LKONLY] capture backend: %s\n", gst_capture_enabled ? "gstreamer" : "libcamera");
    std::fprintf(stderr, "[LKONLY] using reference_tae style LK + Kalman stabilization\n");

    AutoController auto_controller(config);
    if (!auto_controller.start(&error)) {
        std::fprintf(stderr, "[LKONLY] auto controller start failed: %s\n", error.c_str());
        shutdown_capture();
        rtsp_server.stop();
        tank_drive::shutdown();
        mosquitto_lib_cleanup();
        return 1;
    }

    PtzController ptz_controller;
    ptz_controller.start(config.ptz);

    FrameJpegCache frame_cache;
    HttpVrServer http_server;
    if (config.http.enable) {
        if (!http_server.start(config.http, running, frame_cache, ptz_controller)) {
            std::fprintf(stderr, "[LKONLY] tiltVR web start failed\n");
            ptz_controller.stop();
            auto_controller.stop();
            shutdown_capture();
            rtsp_server.stop();
            tank_drive::shutdown();
            mosquitto_lib_cleanup();
            return 1;
        }
    }

    std::thread auto_thread([&]() {
        auto_controller.run(&running);
    });

    RcStatusPublisher::Config status_config;
    status_config.broker_host = config.mqtt.host;
    status_config.broker_port = config.mqtt.port;
    status_config.topic = config.mqtt.status_topic;
    status_config.client_id = "tank_merge_lkonly_status";
    status_config.publish_interval_ms = config.mqtt.status_publish_interval_ms;
    status_config.keepalive_sec = config.mqtt.keepalive_sec;
    status_config.qos = 1;
    status_config.retain = true;
    RcStatusPublisher status_publisher(status_config);
    SystemUsageMonitor system_usage_monitor;
    status_publisher.setStatusProvider([&](RcStatus& status) {
        fill_runtime_status(
            status,
            tank_drive::get_status_snapshot(),
            auto_controller.snapshot(),
            config.mqtt.status_publish_interval_ms);
        status.system_usage = system_usage_monitor.snapshot();
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
        std::fprintf(stderr, "[LKONLY] status publisher disabled due to startup failure\n");
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
            const bool ok = run_vr_remote_input_loop(vr_input_config,
                                                     running,
                                                     [&](const char* reason) {
                                                         auto_controller.cancel_goal(reason);
                                                     });
            if (!ok && running.load()) {
                std::fprintf(stderr, "[VR] controller loop unavailable; Qt/auto control remains active\n");
            }
        });
        vr_input_thread_started = true;
    }

    std::thread tick_thread([&]() {
        while (running.load()) {
            tank_drive::tick();
            const tank_drive::DriveStatusSnapshot drive = tank_drive::get_status_snapshot();
            if (manual_override_active(drive) && auto_controller.snapshot().goal.valid) {
                auto_controller.cancel_goal(drive_source_name(drive.active_source));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });

    std::thread frame_thread([&]() {
        KalmanState kf_theta;
        KalmanState kf_tx;
        KalmanState kf_ty;
        kalman_init(kf_theta, config.eis.lk_kalman_q, config.eis.lk_kalman_r);
        kalman_init(kf_tx, config.eis.lk_kalman_q, config.eis.lk_kalman_r);
        kalman_init(kf_ty, config.eis.lk_kalman_q, config.eis.lk_kalman_r);

        cv::Mat prev_gray;
        cv::Mat prev_frame;
        int frame_count = 0;
        uint64_t logged_frames = 0;
        const int crop_x = kBorderCropPx;
        const int crop_y = std::max(1, kBorderCropPx * config.camera.height / std::max(1, config.camera.width));

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

            if (config.http.enable && frame_cache.has_consumers()) {
                const cv::Mat mjpeg_source = frame.image.isContinuous() ? frame.image : frame.image.clone();
                frame_cache.update_bgr_frame(mjpeg_source.data,
                                             mjpeg_source.total() * mjpeg_source.elemSize(),
                                             mjpeg_source.cols,
                                             mjpeg_source.rows);
            }

            cv::Mat stabilized = frame.image.clone();
            cv::Mat curr_gray;
            cv::cvtColor(frame.image, curr_gray, cv::COLOR_BGR2GRAY);

            size_t feature_count = 0;
            size_t valid_points = 0;
            bool lk_ok = false;
            double raw_dx = 0.0;
            double raw_dy = 0.0;
            double raw_da = 0.0;
            double corr_dx = 0.0;
            double corr_dy = 0.0;
            double corr_da = 0.0;
            size_t overlay_points = 0;

            if (frame_count == 0) {
                prev_gray = curr_gray.clone();
                prev_frame = frame.image.clone();
                frame_count++;
            } else {
                std::vector<cv::Point2f> features_prev;
                std::vector<cv::Point2f> features_curr;
                std::vector<uchar> status;
                std::vector<float> err_vec;

                cv::goodFeaturesToTrack(prev_gray,
                                        features_prev,
                                        kMaxFeatures,
                                        kFeatureQuality,
                                        kFeatureMinDist);
                feature_count = features_prev.size();

                if (features_prev.size() >= 10U) {
                    cv::calcOpticalFlowPyrLK(prev_gray,
                                             curr_gray,
                                             features_prev,
                                             features_curr,
                                             status,
                                             err_vec);

                    std::vector<cv::Point2f> good_prev;
                    std::vector<cv::Point2f> good_curr;
                    good_prev.reserve(features_prev.size());
                    good_curr.reserve(features_prev.size());
                    for (size_t i = 0; i < status.size(); ++i) {
                        if (status[i]) {
                            good_prev.push_back(features_prev[i]);
                            good_curr.push_back(features_curr[i]);
                        }
                    }
                    valid_points = good_prev.size();

                    if (good_prev.size() >= 6U) {
                        cv::Mat affine = cv::estimateAffinePartial2D(good_prev, good_curr);
                        if (!affine.empty()) {
                            raw_dx = affine.at<double>(0, 2);
                            raw_dy = affine.at<double>(1, 2);
                            raw_da = std::atan2(affine.at<double>(1, 0), affine.at<double>(0, 0));
                            const double cos_da = std::cos(raw_da);
                            if (std::abs(cos_da) > 1e-6) {
                                const double sx = affine.at<double>(0, 0) / cos_da;
                                const double sy = affine.at<double>(1, 1) / cos_da;

                                lk_ok = true;
                                overlay_points = good_prev.size();
                                kalman_update(kf_theta, raw_da);
                                kalman_update(kf_tx, raw_dx);
                                kalman_update(kf_ty, raw_dy);

                                if (frame_count > 1) {
                                    corr_da = kalman_diff(kf_theta);
                                    corr_dx = kalman_diff(kf_tx);
                                    corr_dy = kalman_diff(kf_ty);
                                }

                                const double out_da = raw_da + corr_da;
                                const double out_dx = raw_dx + corr_dx;
                                const double out_dy = raw_dy + corr_dy;

                                cv::Mat smoothed = (cv::Mat_<double>(2, 3) <<
                                    sx * std::cos(out_da), sx * -std::sin(out_da), out_dx,
                                    sy * std::sin(out_da), sy *  std::cos(out_da), out_dy);

                                cv::warpAffine(prev_frame, stabilized, smoothed, frame.image.size());

                                const cv::Rect roi(crop_x,
                                                   crop_y,
                                                   stabilized.cols - 2 * crop_x,
                                                   stabilized.rows - 2 * crop_y);
                                if (roi.width > 0 && roi.height > 0) {
                                    cv::resize(stabilized(roi), stabilized, frame.image.size(), 0.0, 0.0, cv::INTER_LINEAR);
                                }

                                if (kDebugOverlay) {
                                    char buf[128];
                                    std::snprintf(buf,
                                                  sizeof(buf),
                                                  "dx:%+5.1f dy:%+5.1f da:%+5.2f deg",
                                                  corr_dx,
                                                  corr_dy,
                                                  corr_da * 180.0 / CV_PI);
                                    cv::putText(stabilized,
                                                buf,
                                                cv::Point(8, 18),
                                                cv::FONT_HERSHEY_SIMPLEX,
                                                0.45,
                                                cv::Scalar(0, 255, 0),
                                                1,
                                                cv::LINE_AA);

                                    char buf2[128];
                                    std::snprintf(buf2,
                                                  sizeof(buf2),
                                                  "features: %zu  Q:%.4f R:%.1f",
                                                  overlay_points,
                                                  config.eis.lk_kalman_q,
                                                  config.eis.lk_kalman_r);
                                    cv::putText(stabilized,
                                                buf2,
                                                cv::Point(8, 36),
                                                cv::FONT_HERSHEY_SIMPLEX,
                                                0.4,
                                                cv::Scalar(0, 255, 255),
                                                1,
                                                cv::LINE_AA);
                                }
                            }
                        }
                    }
                }

                prev_gray = curr_gray.clone();
                prev_frame = frame.image.clone();
                frame_count++;
            }

            runtime_log
                << frame.frame_index << '\t'
                << frame.frame_time_ms << '\t'
                << frame.sensor_ts_ns << '\t'
                << frame.exposure_us << '\t'
                << frame.frame_duration_us << '\t'
                << feature_count << '\t'
                << valid_points << '\t'
                << (lk_ok ? 1 : 0) << '\t'
                << raw_dx << '\t'
                << raw_dy << '\t'
                << raw_da << '\t'
                << corr_dx << '\t'
                << corr_dy << '\t'
                << corr_da << '\t'
                << crop_x << '\t'
                << crop_y << '\n';
            ++logged_frames;
            if ((logged_frames % 30U) == 0U) {
                runtime_log.flush();
            }

            rtsp_server.push_raw(frame, frame.image);
            if (!rtsp_server.push_stabilized(frame, stabilized)) {
                // No client attached is normal.
            }
        }
        runtime_log.flush();
    });

    std::fprintf(stderr,
                 "[LKONLY] runtime ready: RTSP %s%s (stab), %s%s (raw), MQTT %s:%d topic=%s\n",
                 config.rtsp.port.c_str(),
                 config.rtsp.path.c_str(),
                 config.rtsp.port.c_str(),
                 config.rtsp.raw_path.c_str(),
                 config.mqtt.host.c_str(),
                 config.mqtt.port,
                 config.mqtt.control_topic.c_str());

    const bool mqtt_ok = run_mqtt_drive_loop(config.mqtt,
                                             running,
                                             &ptz_controller,
                                             [&](const char* reason) {
                                                 auto_controller.cancel_goal(reason);
                                             });
    if (!mqtt_ok) {
        std::fprintf(stderr, "[LKONLY] Qt control loop ended with error\n");
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
    if (auto_thread.joinable()) {
        auto_thread.join();
    }
    if (status_thread_started && status_thread.joinable()) {
        status_thread.join();
    }

    status_publisher.stop();
    auto_controller.stop();
    rtsp_server.stop();
    tank_drive::shutdown();
    mosquitto_lib_cleanup();
    return mqtt_ok ? 0 : 1;
}
