#include "server_app.hpp"

#include "homography_publisher.hpp"
#include "pose_tracker.hpp"
#include "server_utils.hpp"

#include <mosquitto.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <filesystem>
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
                if (!LoadCameraModel(config_.camera_yaml.string(), camera_model_)) {
                    return 1;
                }

                cv::Mat H_img2world;
                if (!LoadHomography(config_.homography_yaml.string(), H_img2world)) {
                    return 2;
                }

                {
                    std::lock_guard<std::mutex> lk(shared_.mtx);
                    shared_.H_img2world = H_img2world.clone();
                    EstimateWorldPoseFromHomography(
                        H_img2world, camera_model_.K, shared_.R_world_cam, shared_.t_cam_world);
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
        std::cout << "사용법: " << exe << "\n";
        std::cout << "빌드 후 build 디렉터리에서 ./main 으로 실행합니다.\n";
    }

    bool ParseServerConfig(int argc, char** argv, ServerConfig& config, std::string& error) {
        if (argc > 1) {
            (void)config;
            error = "CLI 인자는 사용하지 않습니다. 기본값과 config yaml 경로를 사용합니다.";
            return false;
        }
        return true;
    }

    void ResolveServerPaths(const char* exe, ServerConfig& config) {
        namespace fs = std::filesystem;

        fs::path exe_path = fs::absolute(exe);
        if (!exe_path.has_parent_path()) {
            exe_path = fs::current_path() / exe_path;
        }

        const fs::path build_dir = exe_path.parent_path();
        const fs::path repo_dir = build_dir.parent_path();

        config.config_dir = repo_dir / "config";
        config.homography_yaml = config.config_dir / "H_img2world.yaml";
        config.camera_yaml = config.config_dir / "camera.yaml";
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
