#include "server_app.hpp"

#include "homography_publisher.hpp"
#include "pose_tracker.hpp"
#include "server_utils.hpp"

#include <mosquitto.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <thread>

namespace veda_server {
namespace {

std::atomic<bool> g_run{true};

void OnSignal(int) {
    g_run = false;
}

class ServerApp {
public:
    explicit ServerApp(const ServerConfig& config)
        : config_(config),
          homography_publisher_(config_, camera_model_, shared_),
          pose_tracker_(config_, camera_model_, shared_) {}

    int Run() {
        if (!LoadCameraModel(config_.camera_yaml, camera_model_)) {
            return 1;
        }

        cv::Mat H_img2world;
        if (!LoadHomography(config_.homography_yaml, H_img2world)) {
            return 2;
        }

        {
            std::lock_guard<std::mutex> lk(shared_.mtx);
            shared_.H_img2world = H_img2world.clone();
            shared_.R_world_cam = EstimateRworldCam(H_img2world, camera_model_.K);
        }

        if (!OpenRtspCapture(config_.rtsp_url, cap_)) {
            std::cerr << "[ERR] RTSP open failed\n";
            return 3;
        }

        mosq_ = mosquitto_new("server_main", true, nullptr);
        if (!mosq_) {
            std::cerr << "[ERR] mosquitto_new failed\n";
            return 4;
        }

        mosquitto_reconnect_delay_set(mosq_, 1, 10, true);
        const int connect_rc = mosquitto_connect(mosq_, config_.mqtt_host.c_str(), config_.mqtt_port, 60);
        if (connect_rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "[ERR] mosquitto_connect failed rc=" << connect_rc
                      << " (" << mosquitto_strerror(connect_rc) << ")\n";
            return 5;
        }

        auto next_publish_at = std::chrono::steady_clock::now();
        std::cout << "[OK] server_main started\n";
        std::cout << "[INFO] pose_topic=" << config_.pose_topic
                  << " homography_topic=" << config_.homography_topic
                  << " map_topic=" << config_.map_topic << "\n";

        while (g_run.load()) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_publish_at) {
                if (!homography_publisher_.RefreshAndPublish(mosq_)) {
                    std::cerr << "[WARN] homography refresh/publish failed\n";
                }
                next_publish_at = now + std::chrono::milliseconds(config_.publish_interval_ms);
            }

            cv::Mat frame;
            if (!cap_.read(frame) || frame.empty()) {
                ++read_fail_count_;
                if (read_fail_count_ % 20 == 0) {
                    std::cerr << "[WARN] frame read failed x" << read_fail_count_ << "\n";
                    cap_.release();
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    if (!OpenRtspCapture(config_.rtsp_url, cap_)) {
                        std::cerr << "[WARN] RTSP reconnect failed\n";
                    }
                }
                mosquitto_loop(mosq_, 0, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                continue;
            }

            read_fail_count_ = 0;
            ++frame_count_;
            pose_tracker_.ProcessFrame(frame, mosq_, publish_count_);
            mosquitto_loop(mosq_, 0, 1);
            LogStatsIfNeeded();
        }

        return 0;
    }

    ~ServerApp() {
        if (mosq_) {
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
            mosq_ = nullptr;
        }
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

private:
    void LogStatsIfNeeded() {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_stat_ts_).count();
        if (elapsed_ms < 1000) {
            return;
        }

        const double fps = (elapsed_ms > 0) ? (frame_count_ * 1000.0 / elapsed_ms) : 0.0;
        std::cout << "[STAT] fps=" << fps
                  << " ids_seen=" << pose_tracker_.last_ids_seen()
                  << " best_id=" << pose_tracker_.last_best_id()
                  << " pose_pub_total=" << publish_count_
                  << " H_updates=" << shared_.update_count.load() << "\n";
        frame_count_ = 0;
        last_stat_ts_ = now;
    }

    ServerConfig config_;
    CameraModel camera_model_;
    SharedHomography shared_;
    HomographyPublisher homography_publisher_;
    PoseTracker pose_tracker_;
    cv::VideoCapture cap_;
    mosquitto* mosq_ = nullptr;

    int read_fail_count_ = 0;
    long long frame_count_ = 0;
    long long publish_count_ = 0;
    std::chrono::steady_clock::time_point last_stat_ts_ = std::chrono::steady_clock::now();
};

} // namespace

void PrintServerUsage(const char* exe) {
    std::cout
        << "사용법: " << exe
        << " <rtsp_url> [mqtt_host] [mqtt_port] [pose_topic] [homography_yaml] [camera_yaml] "
        << "[marker_size] [cube_size] [homography_topic] [map_topic] [publish_interval_ms]\n";
}

bool ParseServerConfig(int argc, char** argv, ServerConfig& config, std::string& error) {
    if (argc < 2) {
        error = "RTSP URL이 필요합니다.";
        return false;
    }

    try {
        config.rtsp_url = argv[1];
        if (argc > 2) config.mqtt_host = argv[2];
        if (argc > 3) config.mqtt_port = std::stoi(argv[3]);
        if (argc > 4) config.pose_topic = argv[4];
        if (argc > 5) config.homography_yaml = argv[5];
        if (argc > 6) config.camera_yaml = argv[6];
        if (argc > 7) config.marker_size = std::stod(argv[7]);
        if (argc > 8) config.cube_size = std::stod(argv[8]);
        if (argc > 9) config.homography_topic = argv[9];
        if (argc > 10) config.map_topic = argv[10];
        if (argc > 11) config.publish_interval_ms = std::stoi(argv[11]);
    } catch (const std::exception& ex) {
        error = std::string("인자 파싱 실패: ") + ex.what();
        return false;
    }

    if (config.publish_interval_ms <= 0) {
        error = "publish_interval_ms는 1 이상이어야 합니다.";
        return false;
    }
    return true;
}

int RunServerApp(const ServerConfig& config) {
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    mosquitto_lib_init();
    const int rc = ServerApp(config).Run();
    mosquitto_lib_cleanup();
    return rc;
}

} // namespace veda_server
