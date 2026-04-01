#include "server_app.hpp"

#include "homography_publisher.hpp"
#include "pose_tracker.hpp"
#include "server_utils.hpp"

#include <mosquitto.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cctype>
#include <exception>
#include <filesystem>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace veda_server {
    namespace {

        std::atomic<bool> g_run{true};
        constexpr const char* kRcGoalTopic = "wiserisk/rc/goal";
        constexpr double kLineCrossingGoalX = 44.253;
        constexpr double kLineCrossingGoalY = 522.607;

        void OnSignal(int) {
            g_run = false;
        }

        std::filesystem::path resolve_config_path(const std::filesystem::path& config_dir,
                                                  const std::string& raw_value,
                                                  const std::filesystem::path& fallback) {
            if (raw_value.empty()) {
                return fallback;
            }

            const std::filesystem::path candidate(raw_value);
            if (candidate.is_absolute()) {
                return candidate;
            }

            return (config_dir / candidate).lexically_normal();
        }

        template <typename T>
        void read_if_present(const cv::FileStorage& fs, const char* key, T& value) {
            const cv::FileNode node = fs[key];
            if (!node.empty()) {
                node >> value;
            }
        }

        bool LoadServerSettings(ServerConfig& config, std::string& error) {
            namespace fs = std::filesystem;

            fs::path settings_path;
            for (const fs::path& candidate : {config.settings_yaml, config.config_dir / "server.yaml.example"}) {
                if (fs::exists(candidate)) {
                    settings_path = candidate;
                    break;
                }
            }

            const fs::path default_camera_yaml =
                config.config_dir / fs::path(defaults::kCameraYaml).filename();
            const fs::path default_homography_yaml =
                config.config_dir / fs::path(defaults::kHomographyYaml).filename();

            if (settings_path.empty()) {
                config.camera_yaml = default_camera_yaml;
                config.homography_yaml = default_homography_yaml;
                return true;
            }

            cv::FileStorage fs_settings(settings_path.string(), cv::FileStorage::READ);
            if (!fs_settings.isOpened()) {
                error = "failed to open server settings: " + settings_path.string();
                return false;
            }

            read_if_present(fs_settings, "rtsp_url", config.rtsp_url);
            read_if_present(fs_settings, "mqtt_host", config.mqtt_host);
            read_if_present(fs_settings, "mqtt_port", config.mqtt_port);
            read_if_present(fs_settings, "pose_topic", config.pose_topic);
            read_if_present(fs_settings, "homography_topic", config.homography_topic);
            read_if_present(fs_settings, "map_topic", config.map_topic);
            read_if_present(fs_settings, "marker_size", config.marker_size);
            read_if_present(fs_settings, "cube_size", config.cube_size);
            read_if_present(fs_settings, "publish_interval_ms", config.publish_interval_ms);

            std::string camera_yaml = default_camera_yaml.filename().string();
            std::string homography_yaml = default_homography_yaml.filename().string();
            read_if_present(fs_settings, "camera_yaml", camera_yaml);
            read_if_present(fs_settings, "homography_yaml", homography_yaml);

            config.camera_yaml = resolve_config_path(config.config_dir, camera_yaml, default_camera_yaml);
            config.homography_yaml = resolve_config_path(config.config_dir, homography_yaml, default_homography_yaml);
            return true;
        }

        struct MetadataEvent {
            std::string topic_full;
            std::string topic;
            std::string utc;
            std::string rule;
            std::string action;
            std::string object_id;
            std::string state_str;
        };

        struct SimpleItem {
            std::string name;
            std::string value;
        };

        std::string trim(const std::string& s) {
            const size_t begin = s.find_first_not_of(" \t\r\n");
            const size_t end = s.find_last_not_of(" \t\r\n");
            if (begin == std::string::npos) return "";
            return s.substr(begin, end - begin + 1);
        }

        bool find_between(const std::string& src,
                          const std::string& left,
                          const std::string& right,
                          std::string& out,
                          size_t start_pos = 0) {
            size_t p1 = src.find(left, start_pos);
            if (p1 == std::string::npos) return false;
            p1 += left.size();
            const size_t p2 = src.find(right, p1);
            if (p2 == std::string::npos) return false;
            out = src.substr(p1, p2 - p1);
            return true;
        }

        bool find_attr_value(const std::string& src,
                             const std::string& key,
                             std::string& out,
                             size_t start_pos = 0) {
            size_t p = src.find(key, start_pos);
            if (p == std::string::npos) return false;
            p += key.size();
            const size_t q = src.find('"', p);
            if (q == std::string::npos) return false;
            out = src.substr(p, q - p);
            return true;
        }

        std::vector<SimpleItem> extract_simple_items(const std::string& xml) {
            static const std::regex re("Name=\"([^\"]+)\"\\s+Value=\"([^\"]*)\"");
            std::vector<SimpleItem> out;
            for (std::sregex_iterator it(xml.begin(), xml.end(), re), end; it != end; ++it) {
                out.push_back({(*it)[1].str(), (*it)[2].str()});
            }
            return out;
        }

        std::string normalize_key(const std::string& value) {
            std::string out;
            out.reserve(value.size());
            for (unsigned char ch : value) {
                if (std::isalnum(ch)) out.push_back(static_cast<char>(std::tolower(ch)));
            }
            return out;
        }

        std::string extract_simpleitem_value(const std::vector<SimpleItem>& items,
                                             const std::string& wanted_name) {
            const std::string wanted = normalize_key(wanted_name);
            for (const auto& item : items) {
                if (normalize_key(item.name) == wanted) return item.value;
            }
            return "";
        }

        std::string extract_topic_full(const std::string& xml) {
            std::string out;
            if (!find_between(xml, "<wsnt:Topic", "</wsnt:Topic>", out)) return "";
            const size_t gt = out.find('>');
            if (gt == std::string::npos) return "";
            return trim(out.substr(gt + 1));
        }

        std::string last_token(const std::string& topic_full) {
            const size_t slash = topic_full.find_last_of('/');
            std::string tail = (slash == std::string::npos) ? topic_full : topic_full.substr(slash + 1);
            const size_t colon = tail.find_last_of(':');
            if (colon != std::string::npos) tail = tail.substr(colon + 1);
            return tail;
        }

        bool parse_metadata_xml(const std::string& xml, MetadataEvent& out) {
            out.topic_full = extract_topic_full(xml);
            if (out.topic_full.empty()) return false;

            const std::vector<SimpleItem> items = extract_simple_items(xml);
            find_attr_value(xml, "UtcTime=\"", out.utc);
            out.rule = extract_simpleitem_value(items, "RuleName");
            out.state_str = extract_simpleitem_value(items, "State");
            out.object_id = extract_simpleitem_value(items, "ObjectId");
            out.action = extract_simpleitem_value(items, "Action");
            out.topic = last_token(out.topic_full);
            return true;
        }

        bool is_line_crossing_event(const MetadataEvent& ev) {
            return ev.topic == "LineCrossing" || ev.rule == "LineCrossing";
        }

        std::string build_rc_goal_payload() {
            std::ostringstream j;
            j << "{"
              << "\"x\":" << kLineCrossingGoalX << ","
              << "\"y\":" << kLineCrossingGoalY << ","
              << "\"frame\":\"world\","
              << "\"ts_ms\":" << std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count()
              << "}";
            return j.str();
        }

        class MetadataGoalMonitor {
        public:
            explicit MetadataGoalMonitor(const ServerConfig& config)
                : config_(config) {}

            ~MetadataGoalMonitor() {
                Stop();
            }

            void Start() {
                if (thread_.joinable()) return;
                running_ = true;
                thread_ = std::thread([this] { Run(); });
            }

            void Stop() {
                running_ = false;
                if (thread_.joinable()) thread_.join();
            }

        private:
            void Run() {
                mosquitto* mosq = mosquitto_new("server_linecrossing", true, nullptr);
                if (!mosq) {
                    std::cerr << "[WARN] metadata monitor mosquitto_new failed\n";
                    return;
                }

                const int connect_rc = mosquitto_connect(mosq, config_.mqtt_host.c_str(), config_.mqtt_port, 60);
                if (connect_rc != MOSQ_ERR_SUCCESS) {
                    std::cerr << "[WARN] metadata monitor mosquitto_connect failed rc=" << connect_rc
                              << " (" << mosquitto_strerror(connect_rc) << ")\n";
                    mosquitto_destroy(mosq);
                    return;
                }

                const std::string cmd =
                    "ffmpeg -loglevel error -rtsp_transport tcp "
                    "-i \"" + config_.rtsp_url + "\" "
                    "-map 0:1 -c copy -f data -";

                FILE* fp = popen(cmd.c_str(), "r");
                if (!fp) {
                    std::cerr << "[WARN] metadata monitor popen(ffmpeg) failed\n";
                    mosquitto_disconnect(mosq);
                    mosquitto_destroy(mosq);
                    return;
                }

                std::string buf;
                buf.reserve(1024 * 1024);
                char readbuf[8192];
                while (running_.load() && g_run.load()) {
                    const size_t n = std::fread(readbuf, 1, sizeof(readbuf), fp);
                    if (n == 0) break;
                    buf.append(readbuf, n);

                    while (running_.load() && g_run.load()) {
                        const size_t p1 = buf.find("<?xml");
                        if (p1 == std::string::npos) break;
                        const size_t p2 = buf.find("<?xml", p1 + 5);
                        if (p2 == std::string::npos) break;

                        const std::string one = buf.substr(p1, p2 - p1);
                        buf.erase(0, p2);

                        MetadataEvent ev{};
                        if (!parse_metadata_xml(one, ev)) continue;
                        if (!is_line_crossing_event(ev)) continue;
                        if (ev.state_str != "true") continue;

                        const auto now = std::chrono::steady_clock::now();
                        if (last_publish_at_.time_since_epoch().count() != 0 &&
                            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_publish_at_).count() < 2000) {
                            continue;
                        }

                        const std::string payload = build_rc_goal_payload();
                        const int pub_rc = mosquitto_publish(
                            mosq, nullptr, kRcGoalTopic,
                            static_cast<int>(payload.size()), payload.c_str(),
                            1, false);
                        mosquitto_loop(mosq, 0, 1);
                        if (pub_rc == MOSQ_ERR_SUCCESS) {
                            last_publish_at_ = now;
                            std::cerr << "[LINECROSS] rc goal published objectId=" << ev.object_id
                                      << " action=" << ev.action
                                      << " utc=" << ev.utc
                                      << " payload=" << payload << "\n";
                        } else {
                            std::cerr << "[WARN] linecross rc goal publish failed rc=" << pub_rc
                                      << " (" << mosquitto_strerror(pub_rc) << ")\n";
                        }
                    }

                    if (buf.size() > 8 * 1024 * 1024) {
                        buf.erase(0, buf.size() - 1024 * 1024);
                    }
                }

                pclose(fp);
                mosquitto_disconnect(mosq);
                mosquitto_destroy(mosq);
            }

            ServerConfig config_;
            std::atomic<bool> running_{false};
            std::thread thread_;
            std::chrono::steady_clock::time_point last_publish_at_{};
        };

        class ServerApp {
        public:
            explicit ServerApp(const ServerConfig& config)
                : config_(config),
                  homography_publisher_(config_, camera_model_, shared_),
                  pose_tracker_(config_, camera_model_, shared_),
                  metadata_goal_monitor_(config_) {}

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
                metadata_goal_monitor_.Start();

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
                metadata_goal_monitor_.Stop();
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
            MetadataGoalMonitor metadata_goal_monitor_;
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
        std::cout << "통합 빌드에서는 build/server/main 에서 실행합니다.\n";
    }

    bool ParseServerConfig(int argc, char** argv, ServerConfig& config, std::string& error) {
        if (argc > 1) {
            error = "CLI 인자는 지원하지 않습니다. config/server.yaml 기준으로 설정을 읽습니다.";
            return false;
        }

        return LoadServerSettings(config, error);
    }

    void ResolveServerPaths(const char* exe, ServerConfig& config) {
        namespace fs = std::filesystem;

        fs::path exe_path = fs::absolute(exe);
        if (!exe_path.has_parent_path()) {
            exe_path = fs::current_path() / exe_path;
        }

        config.runtime_dir = exe_path.parent_path();
        config.config_dir = config.runtime_dir / "config";
        config.settings_yaml = config.config_dir / "server.yaml";
        config.homography_yaml = config.config_dir / fs::path(defaults::kHomographyYaml).filename();
        config.camera_yaml = config.config_dir / fs::path(defaults::kCameraYaml).filename();
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
