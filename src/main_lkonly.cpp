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

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "app_config.hpp"
#include "gst_camera_capture.hpp"
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

constexpr int kMaxFeatures = 200;
constexpr double kFeatureQuality = 0.01;
constexpr double kFeatureMinDist = 30.0;
constexpr double kKalmanQ = 0.004;
constexpr double kKalmanR = 0.5;
constexpr int kBorderCropPx = 30;
constexpr bool kDebugOverlay = true;

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

    tank_drive::init();
    tank_drive::set_idle_autostop(false);

    RtspServer rtsp_server;
    if (!rtsp_server.start(config.camera, config.rtsp, true, &error)) {
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
    std::fprintf(stderr, "[LKONLY] using reference_tae style LK + Kalman stabilization\n");

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
        KalmanState kf_theta;
        KalmanState kf_tx;
        KalmanState kf_ty;
        kalman_init(kf_theta, kKalmanQ, kKalmanR);
        kalman_init(kf_tx, kKalmanQ, kKalmanR);
        kalman_init(kf_ty, kKalmanQ, kKalmanR);

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
                                                  kKalmanQ,
                                                  kKalmanR);
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
