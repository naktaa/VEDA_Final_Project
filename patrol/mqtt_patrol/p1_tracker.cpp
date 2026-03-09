// ./rc_control_node 192.168.100.10 1883 wiserisk/rc/goal wiserisk/p1/pose wiserisk/rc/safety wiserisk/rc/status
// g++ -O2 -std=c++17 p1_tracker.cpp -o p1_tracker $(pkg-config --cflags --libs opencv4) -lmosquitto

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cctype>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <mosquitto.h>

namespace {
double normalizeAngle(double a) {
    constexpr double PI = 3.14159265358979323846;
    while (a > PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

double blendAngle(double base, double target, double alpha) {
    return normalizeAngle(base + alpha * normalizeAngle(target - base));
}

long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

long long nowSec() {
    return std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

bool loadHomography(const std::string& path, cv::Mat& H_img2world) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["H_img2world"] >> H_img2world;
    fs.release();

    return (!H_img2world.empty() && H_img2world.rows == 3 && H_img2world.cols == 3);
}

cv::Point2f applyHomography(const cv::Mat& H, const cv::Point2f& p) {
    std::vector<cv::Point2f> src{p}, dst;
    cv::perspectiveTransform(src, dst, H);
    return dst[0];
}

double calcYawWorld(const cv::Mat& H_img2world,
                    const cv::Point2f& centerPx,
                    const std::vector<cv::Point2f>& markerCorners) {
    cv::Point2f dir = markerCorners[1] - markerCorners[0];
    const float norm = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (norm < 1e-6f) return 0.0;
    dir *= (1.0f / norm);

    const cv::Point2f headPx = centerPx + dir * 30.0f;
    const cv::Point2f w0 = applyHomography(H_img2world, centerPx);
    const cv::Point2f w1 = applyHomography(H_img2world, headPx);
    return std::atan2((double)(w1.y - w0.y), (double)(w1.x - w0.x));
}

double markerAreaPx2(const std::vector<cv::Point2f>& markerCorners) {
    if (markerCorners.size() != 4) return 0.0;
    return std::fabs(cv::contourArea(markerCorners));
}

bool mqttPublishJson(mosquitto* mosq,
                     const std::string& topic,
                     const std::string& json) {
    const int rc = mosquitto_publish(
        mosq,
        nullptr,
        topic.c_str(),
        static_cast<int>(json.size()),
        json.c_str(),
        1,
        false);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_publish failed rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ") topic=" << topic << "\n";
    }
    return (rc == MOSQ_ERR_SUCCESS);
}

bool extractDoubleByKey(const std::string& payload, const std::string& key, double& out) {
    const std::string needle = "\"" + key + "\"";
    const size_t keyPos = payload.find(needle);
    if (keyPos == std::string::npos) return false;
    const size_t colonPos = payload.find(':', keyPos + needle.size());
    if (colonPos == std::string::npos) return false;

    size_t p = colonPos + 1;
    while (p < payload.size() && std::isspace(static_cast<unsigned char>(payload[p]))) ++p;
    if (p >= payload.size()) return false;

    char* endPtr = nullptr;
    const double v = std::strtod(payload.c_str() + p, &endPtr);
    if (endPtr == payload.c_str() + p) return false;
    out = v;
    return true;
}

bool extractInt64ByKey(const std::string& payload, const std::string& key, long long& out) {
    const std::string needle = "\"" + key + "\"";
    const size_t keyPos = payload.find(needle);
    if (keyPos == std::string::npos) return false;
    const size_t colonPos = payload.find(':', keyPos + needle.size());
    if (colonPos == std::string::npos) return false;

    size_t p = colonPos + 1;
    while (p < payload.size() && std::isspace(static_cast<unsigned char>(payload[p]))) ++p;
    if (p >= payload.size()) return false;

    char* endPtr = nullptr;
    const long long v = std::strtoll(payload.c_str() + p, &endPtr, 10);
    if (endPtr == payload.c_str() + p) return false;
    out = v;
    return true;
}

bool extractStringByKey(const std::string& payload, const std::string& key, std::string& out) {
    const std::string needle = "\"" + key + "\"";
    const size_t keyPos = payload.find(needle);
    if (keyPos == std::string::npos) return false;
    const size_t colonPos = payload.find(':', keyPos + needle.size());
    if (colonPos == std::string::npos) return false;

    const size_t quote1 = payload.find('"', colonPos + 1);
    if (quote1 == std::string::npos) return false;
    const size_t quote2 = payload.find('"', quote1 + 1);
    if (quote2 == std::string::npos) return false;

    out = payload.substr(quote1 + 1, quote2 - quote1 - 1);
    return true;
}

struct TrackerState {
    std::string status_topic;
    std::string init_pose_topic;
    std::string cmd_feedback_topic;

    bool initialized = false;
    double pose_x = 0.0;
    double pose_y = 0.0;
    double pose_yaw = 0.0;
    long long init_ts_ms = 0;

    std::string status_mode = "WAIT_INPUT";

    bool has_cmd = false;
    double cmd_speed_mps = 0.0;
    double cmd_yaw_rate_rps = 0.0;
    long long cmd_ts_ms = 0;

    long long last_vision_ts_ms = 0;
    double pose_quality = 0.0;
    std::string pose_source = "manual_dr";
};

bool parseInitPoseJson(const std::string& payload,
                       double& x,
                       double& y,
                       double& yaw,
                       long long& ts_ms,
                       std::string& frame) {
    if (!extractDoubleByKey(payload, "x", x)) return false;
    if (!extractDoubleByKey(payload, "y", y)) return false;
    if (!extractDoubleByKey(payload, "yaw", yaw)) return false;
    if (!extractStringByKey(payload, "frame", frame)) return false;
    if (!extractInt64ByKey(payload, "ts_ms", ts_ms) &&
        !extractInt64ByKey(payload, "ts", ts_ms)) {
        ts_ms = nowMs();
    }
    return true;
}

bool parseCmdFeedbackJson(const std::string& payload,
                          double& speed_mps,
                          double& yaw_rate_rps,
                          long long& ts_ms,
                          std::string& mode) {
    if (!extractDoubleByKey(payload, "speed_mps", speed_mps)) return false;
    if (!extractDoubleByKey(payload, "yaw_rate_rps", yaw_rate_rps)) yaw_rate_rps = 0.0;
    if (!extractInt64ByKey(payload, "ts_ms", ts_ms) && !extractInt64ByKey(payload, "ts", ts_ms)) {
        ts_ms = nowMs();
    }
    if (!extractStringByKey(payload, "mode", mode)) {
        mode = "";
    }
    return true;
}

void onMqttMessage(struct mosquitto*, void* userdata, const struct mosquitto_message* msg) {
    if (!userdata || !msg || !msg->topic || !msg->payload || msg->payloadlen <= 0) return;

    auto* st = static_cast<TrackerState*>(userdata);
    const std::string payload(static_cast<const char*>(msg->payload), msg->payloadlen);
    const std::string topic(msg->topic);

    if (topic == st->status_topic) {
        std::string mode;
        if (extractStringByKey(payload, "mode", mode)) {
            st->status_mode = mode;
        }
        return;
    }

    if (topic == st->init_pose_topic) {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        long long ts_ms = 0;
        std::string frame;
        if (!parseInitPoseJson(payload, x, y, yaw, ts_ms, frame) || frame != "world") {
            std::cerr << "[WARN] invalid init_pose payload: " << payload << "\n";
            return;
        }
        st->pose_x = x;
        st->pose_y = y;
        st->pose_yaw = normalizeAngle(yaw);
        st->init_ts_ms = ts_ms;
        st->initialized = true;
        st->pose_quality = 1.0;
        st->pose_source = "manual_dr";
        std::cout << std::fixed << std::setprecision(3)
                  << "[INIT_POSE] x=" << st->pose_x
                  << " y=" << st->pose_y
                  << " yaw=" << st->pose_yaw
                  << " ts_ms=" << st->init_ts_ms << "\n";
        return;
    }

    if (topic == st->cmd_feedback_topic) {
        double speed_mps = 0.0;
        double yaw_rate_rps = 0.0;
        long long ts_ms = 0;
        std::string mode;
        if (!parseCmdFeedbackJson(payload, speed_mps, yaw_rate_rps, ts_ms, mode)) {
            std::cerr << "[WARN] invalid cmd_feedback payload: " << payload << "\n";
            return;
        }

        constexpr double kAlpha = 0.35;
        if (!st->has_cmd) {
            st->cmd_speed_mps = speed_mps;
        } else {
            st->cmd_speed_mps = kAlpha * speed_mps + (1.0 - kAlpha) * st->cmd_speed_mps;
        }
        st->cmd_yaw_rate_rps = yaw_rate_rps;
        st->cmd_ts_ms = ts_ms;
        st->has_cmd = true;
    }
}

double computePoseQuality(bool initialized,
                          bool has_cmd,
                          long long cmd_age_ms,
                          long long vision_age_ms,
                          std::string& out_source,
                          long long& out_age_ms) {
    if (!initialized) {
        out_source = "manual_dr";
        out_age_ms = 99999;
        return 0.0;
    }

    double q = 0.2;
    if (!has_cmd) {
        q = 0.25;
    } else if (cmd_age_ms <= 200) {
        q = 0.80;
    } else if (cmd_age_ms <= 500) {
        q = 0.60;
    } else if (cmd_age_ms <= 1000) {
        q = 0.35;
    } else {
        q = 0.18;
    }

    if (vision_age_ms >= 0 && vision_age_ms <= 1200) {
        out_source = "vision_fused";
        if (vision_age_ms <= 300) {
            q = std::max(q, 0.95);
        } else if (vision_age_ms <= 1000) {
            q = std::max(q, 0.75);
        } else {
            q = std::max(q, 0.55);
        }
        out_age_ms = std::min(cmd_age_ms, vision_age_ms);
    } else {
        out_source = "manual_dr";
        out_age_ms = cmd_age_ms;
    }

    if (out_age_ms < 0) out_age_ms = 0;
    if (q < 0.0) q = 0.0;
    if (q > 1.0) q = 1.0;
    return q;
}
} // namespace

int main(int argc, char** argv) {
    const std::string rtspUrl =
        "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
    const std::string homographyYaml = "H_img2world.yaml";
    const int targetMarkerId = 0;
    const std::string mqttHost = "192.168.100.10";
    const int mqttPort = 1883;
    const std::string poseTopic = "wiserisk/p1/pose";
    const std::string wallTopic = "wiserisk/map/walls";
    const std::string statusTopic = "wiserisk/rc/status";
    const std::string initPoseTopic = "wiserisk/p1/init_pose";
    const std::string cmdFeedbackTopic = "wiserisk/rc/cmd_feedback";

    constexpr int kPublishHz = 10;
    constexpr int kMpuAddr = 0x68;
    constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
    constexpr long long kCmdStaleMs = 500;
    constexpr double kVisionMinAreaPx2 = 800.0;
    constexpr double kVisionBlendPos = 0.20;
    constexpr double kVisionBlendYaw = 0.15;
    constexpr double kVisionYawOffset = -3.101;

    cv::Mat H_img2world;
    if (!loadHomography(homographyYaml, H_img2world)) {
        std::cerr << "[ERR] failed to load homography: " << homographyYaml << "\n";
        std::cerr << "      expected key: H_img2world\n";
        return 1;
    }

    cv::VideoCapture cap;
    cap.open(rtspUrl);
    if (!cap.isOpened()) {
        std::cerr << "[ERR] RTSP open failed\n";
        return 2;
    }

    const auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params;
    cv::aruco::ArucoDetector detector(dict, params);

    mosquitto_lib_init();
    TrackerState tracker_state{};
    tracker_state.status_topic = statusTopic;
    tracker_state.init_pose_topic = initPoseTopic;
    tracker_state.cmd_feedback_topic = cmdFeedbackTopic;

    mosquitto* mosq = mosquitto_new("p1_tracker", true, &tracker_state);
    if (!mosq) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        return 3;
    }
    mosquitto_message_callback_set(mosq, onMqttMessage);

    if (mosquitto_connect(mosq, mqttHost.c_str(), mqttPort, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_connect failed\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 4;
    }

    auto subscribeOrWarn = [&](const std::string& topic) {
        const int rc = mosquitto_subscribe(mosq, nullptr, topic.c_str(), 1);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "[WARN] subscribe failed rc=" << rc
                      << " (" << mosquitto_strerror(rc) << ") topic=" << topic << "\n";
        } else {
            std::cout << "[OK] subscribed topic: " << topic << "\n";
        }
    };
    subscribeOrWarn(statusTopic);
    subscribeOrWarn(initPoseTopic);
    subscribeOrWarn(cmdFeedbackTopic);

    auto writeReg = [](int fd, uint8_t reg, uint8_t val) -> bool {
        uint8_t b[2] = {reg, val};
        return (write(fd, b, 2) == 2);
    };
    auto read16 = [](int fd, uint8_t regH, int16_t& out) -> bool {
        if (write(fd, &regH, 1) != 1) return false;
        uint8_t b[2] = {0, 0};
        if (read(fd, b, 2) != 2) return false;
        out = static_cast<int16_t>((b[0] << 8) | b[1]);
        return true;
    };

    int i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0 || ioctl(i2c_fd, I2C_SLAVE, kMpuAddr) < 0) {
        std::cerr << "[ERR] failed to open MPU6050 on /dev/i2c-1 addr=0x68\n";
        if (i2c_fd >= 0) close(i2c_fd);
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 5;
    }

    if (!writeReg(i2c_fd, 0x6B, 0x00) || !writeReg(i2c_fd, 0x1B, 0x00) || !writeReg(i2c_fd, 0x1A, 0x03)) {
        std::cerr << "[ERR] failed to init MPU6050 registers\n";
        close(i2c_fd);
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 6;
    }

    double gyro_bias = 0.0;
    int bias_n = 0;
    const auto bias_t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - bias_t0).count() < 1.5) {
        int16_t raw = 0;
        if (read16(i2c_fd, 0x47, raw)) {
            const double dps = static_cast<double>(raw) / 131.0;
            gyro_bias += dps * kDeg2Rad;
            bias_n++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (bias_n > 0) gyro_bias /= static_cast<double>(bias_n);
    std::cout << std::fixed << std::setprecision(6)
              << "[OK] MPU6050 gyro bias_z(rad/s)=" << gyro_bias << "\n";

    std::array<bool, 14> wallPublished{};
    auto last_tick = std::chrono::steady_clock::now();
    auto last_wait_log = std::chrono::steady_clock::now() - std::chrono::seconds(2);

    while (true) {
        mosquitto_loop(mosq, 0, 1);

        cv::Mat frame;
        const bool frame_ok = cap.read(frame) && !frame.empty();

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        if (frame_ok) {
            detector.detectMarkers(frame, corners, ids);

            for (size_t i = 0; i < ids.size(); i++) {
                cv::Point2f c(0, 0);
                for (const auto& p : corners[i]) c += p;
                c *= (1.0f / 4.0f);
                const cv::Point2f w = applyHomography(H_img2world, c);

                const int markerId = ids[i];
                if (markerId >= 10 && markerId <= 13 && !wallPublished[markerId]) {
                    wallPublished[markerId] = true;
                    const long long ts_ms = nowMs();
                    char wbuf[192];
                    std::snprintf(wbuf, sizeof(wbuf),
                                  "{\"id\":%d,\"x\":%.3f,\"y\":%.3f,\"frame\":\"world\",\"ts_ms\":%lld}",
                                  markerId, w.x, w.y, ts_ms);
                    mqttPublishJson(mosq, wallTopic, wbuf);
                    std::cout << std::fixed << std::setprecision(3)
                              << "[WALL] id=" << markerId
                              << " x=" << w.x
                              << " y=" << w.y << "\n";
                }

                if (markerId == targetMarkerId && tracker_state.initialized) {
                    const double area = markerAreaPx2(corners[i]);
                    if (area >= kVisionMinAreaPx2) {
                        const double yaw_raw = calcYawWorld(H_img2world, c, corners[i]);
                        const double yaw_world = normalizeAngle(yaw_raw - kVisionYawOffset);
                        tracker_state.pose_x = (1.0 - kVisionBlendPos) * tracker_state.pose_x +
                                               kVisionBlendPos * static_cast<double>(w.x);
                        tracker_state.pose_y = (1.0 - kVisionBlendPos) * tracker_state.pose_y +
                                               kVisionBlendPos * static_cast<double>(w.y);
                        tracker_state.pose_yaw = blendAngle(tracker_state.pose_yaw, yaw_world, kVisionBlendYaw);
                        tracker_state.last_vision_ts_ms = nowMs();
                    }
                }
            }
        }

        if (!tracker_state.initialized) {
            const auto now_steady = std::chrono::steady_clock::now();
            if (now_steady - last_wait_log >= std::chrono::seconds(1)) {
                std::cout << "[WAIT] waiting init_pose on " << initPoseTopic << "\n";
                last_wait_log = now_steady;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / kPublishHz));
            continue;
        }

        const auto now_steady = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now_steady - last_tick).count();
        last_tick = now_steady;
        if (dt < 0.0) dt = 0.0;
        if (dt > 0.2) dt = 0.2;

        int16_t raw_gyro_z = 0;
        double gyro_z_rps = 0.0;
        if (read16(i2c_fd, 0x47, raw_gyro_z)) {
            const double dps = static_cast<double>(raw_gyro_z) / 131.0;
            gyro_z_rps = dps * kDeg2Rad - gyro_bias;
        }
        tracker_state.pose_yaw = normalizeAngle(tracker_state.pose_yaw + gyro_z_rps * dt);

        const long long now_ms = nowMs();
        const long long cmd_age_ms = tracker_state.has_cmd ? (now_ms - tracker_state.cmd_ts_ms) : 99999;
        double v_signed = tracker_state.has_cmd ? tracker_state.cmd_speed_mps : 0.0;
        if (!tracker_state.has_cmd || cmd_age_ms > kCmdStaleMs) {
            v_signed = 0.0;
        }

        tracker_state.pose_x += v_signed * dt * std::cos(tracker_state.pose_yaw);
        tracker_state.pose_y += v_signed * dt * std::sin(tracker_state.pose_yaw);

        const long long vision_age_ms =
            (tracker_state.last_vision_ts_ms > 0) ? (now_ms - tracker_state.last_vision_ts_ms) : -1;
        long long pose_age_ms = 0;
        std::string pose_source;
        const double pose_quality = computePoseQuality(
            tracker_state.initialized,
            tracker_state.has_cmd,
            cmd_age_ms,
            vision_age_ms,
            pose_source,
            pose_age_ms);

        tracker_state.pose_quality = pose_quality;
        tracker_state.pose_source = pose_source;

        const long long ts = nowSec();
        const long long ts_ms = now_ms;

        std::cout << std::fixed << std::setprecision(3)
                  << "[POSE] x=" << tracker_state.pose_x
                  << " y=" << tracker_state.pose_y
                  << " yaw=" << tracker_state.pose_yaw
                  << " gyro_z=" << gyro_z_rps
                  << " v=" << v_signed
                  << " quality=" << tracker_state.pose_quality
                  << " source=" << tracker_state.pose_source
                  << " cmd_age_ms=" << cmd_age_ms
                  << " status_mode=" << tracker_state.status_mode
                  << "\n";

        char buf[384];
        std::snprintf(buf, sizeof(buf),
                      "{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.6f,\"frame\":\"world\",\"ts\":%lld,\"ts_ms\":%lld,\"quality\":%.3f,\"source\":\"%s\",\"age_ms\":%lld}",
                      tracker_state.pose_x,
                      tracker_state.pose_y,
                      tracker_state.pose_yaw,
                      ts,
                      ts_ms,
                      tracker_state.pose_quality,
                      tracker_state.pose_source.c_str(),
                      pose_age_ms);
        mqttPublishJson(mosq, poseTopic, buf);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / kPublishHz));
        mosquitto_loop(mosq, 0, 1);
    }

    close(i2c_fd);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
