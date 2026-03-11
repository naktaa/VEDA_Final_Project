// ./rc_control_node 192.168.100.10 1883 wiserisk/rc/goal wiserisk/p1/pose wiserisk/rc/safety wiserisk/rc/status
// g++ -O2 -std=c++17 p1_tracker.cpp -o p1_tracker $(pkg-config --cflags --libs opencv4) -lmosquitto

#include <iostream>
#include <iomanip>
#include <array>
#include <algorithm>
#include <chrono>
#include <string>
#include <cmath>
#include <thread>
#include <cstdlib>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <mosquitto.h>

namespace {
double normalizeAngle(double a)
{
    constexpr double PI = 3.14159265358979323846;
    while (a > PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

static double blendAngle(double a, double b, double alpha)
{
    const double s = std::sin(a) * (1.0 - alpha) + std::sin(b) * alpha;
    const double c = std::cos(a) * (1.0 - alpha) + std::cos(b) * alpha;
    return std::atan2(s, c);
}
} // namespace

// ---------- utils ----------
static bool loadHomography(const std::string& path, cv::Mat& H_img2world)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["H_img2world"] >> H_img2world;
    fs.release();

    return (!H_img2world.empty() && H_img2world.rows == 3 && H_img2world.cols == 3);
}

static cv::Point2f applyHomography(const cv::Mat& H, const cv::Point2f& p)
{
    std::vector<cv::Point2f> src{p}, dst;
    cv::perspectiveTransform(src, dst, H);
    return dst[0];
}

static double calcYawWorld(const cv::Mat& H_img2world,
                           const cv::Point2f& centerPx,
                           const std::vector<cv::Point2f>& markerCorners)
{
    // ArUco corners order: 0(top-left), 1(top-right), 2(bottom-right), 3(bottom-left)
    // Marker local +X direction in image
    cv::Point2f dir = markerCorners[1] - markerCorners[0];
    const float norm = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (norm < 1e-6f) return 0.0;
    dir *= (1.0f / norm);

    const cv::Point2f headPx = centerPx + dir * 30.0f;
    const cv::Point2f w0 = applyHomography(H_img2world, centerPx);
    const cv::Point2f w1 = applyHomography(H_img2world, headPx);
    return std::atan2((double)(w1.y - w0.y), (double)(w1.x - w0.x));
}

static long long nowSec()
{
    return std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

static bool mqttPublishJson(mosquitto* mosq,
                            const std::string& topic,
                            const std::string& json,
                            bool retain = false)
{
    int rc = mosquitto_publish(
        mosq,
        nullptr,
        topic.c_str(),
        (int)json.size(),
        json.c_str(),
        1,
        retain);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_publish failed rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ") topic=" << topic << "\n";
    }
    return (rc == MOSQ_ERR_SUCCESS);
}

static bool extractDoubleByKey(const std::string& payload, const std::string& key, double& out)
{
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

struct TrackerState {
    std::string status_topic;
    enum class MotionMode {
        TRACKING,
        REV_TRACKING,
        HOLD
    };
    MotionMode motion_mode = MotionMode::TRACKING;
    std::chrono::steady_clock::time_point last_status_ts = std::chrono::steady_clock::now();
};

struct FilterState {
    bool initialized = false;
    bool seen_once = false;
    std::chrono::steady_clock::time_point last_tick = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_marker_seen = std::chrono::steady_clock::now();
    double marker_x = 0.0;
    double marker_y = 0.0;
    double marker_yaw = 0.0;
    double marker_vx = 0.0;
    double marker_vy = 0.0;
    double marker_conf = 0.0;
};

static void onMqttMessage(struct mosquitto*, void* userdata, const struct mosquitto_message* msg)
{
    if (!userdata || !msg || !msg->topic || !msg->payload || msg->payloadlen <= 0) return;
    auto* st = static_cast<TrackerState*>(userdata);
    const std::string payload(static_cast<const char*>(msg->payload), msg->payloadlen);
    const std::string topic(msg->topic);

    if (topic != st->status_topic) return;

    // rc_control_node status json example: {"mode":"REV_TRACKING",...}
    if (payload.find("\"mode\":\"REV_TRACKING\"") != std::string::npos ||
        payload.find("\"mode\":\"REV_WIGGLE_TURN\"") != std::string::npos ||
        payload.find("\"mode\":\"REV_WIGGLE_PULSE\"") != std::string::npos) {
        st->motion_mode = TrackerState::MotionMode::REV_TRACKING;
    } else if (payload.find("\"mode\":\"TRACKING\"") != std::string::npos ||
               payload.find("\"mode\":\"US_TURN_LEFT\"") != std::string::npos ||
               payload.find("\"mode\":\"US_TURN_RIGHT\"") != std::string::npos) {
        st->motion_mode = TrackerState::MotionMode::TRACKING;
    } else if (payload.find("\"mode\":\"REACHED\"") != std::string::npos ||
               payload.find("\"mode\":\"SAFE_STOP\"") != std::string::npos ||
               payload.find("\"mode\":\"WAIT_INPUT\"") != std::string::npos) {
        st->motion_mode = TrackerState::MotionMode::HOLD;
    }
    st->last_status_ts = std::chrono::steady_clock::now();
}

static bool allWallMarkersSeen(const std::array<bool, 14>& wallSeen)
{
    for (int id = 10; id <= 13; ++id) {
        if (!wallSeen[id]) return false;
    }
    return true;
}

// ---------- main ----------
int main(int argc, char** argv)
{
    const std::string rtspUrl =
        "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
    const std::string homographyYaml = "H_img2world.yaml";
    const int targetMarkerId = 0;
    const std::string mqttHost = "192.168.100.10";
    const int mqttPort = 1883;
    const std::string mqttTopic = "wiserisk/p1/pose";
    const std::string wallTopic = "wiserisk/map/walls";
    const std::string statusTopic = "wiserisk/rc/status";
    constexpr double kSpeedMps = 0.6;      // dead-reckoning speed (match rc_control_node setting)
    constexpr int kPublishHz = 10;         // publish rate after init
    constexpr double kLostRecoverHoldSec = 6.0;
    constexpr double kLostFullSlowdownSec = 12.0;
    constexpr double kMaxPoseJumpM = 2.2;
    constexpr int kMpuAddr = 0x68;
    constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

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

    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params;
    cv::aruco::ArucoDetector detector(dict, params);

    mosquitto_lib_init();
    TrackerState tracker_state{};
    tracker_state.status_topic = statusTopic;
    tracker_state.motion_mode = TrackerState::MotionMode::TRACKING;
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
    const int subRc = mosquitto_subscribe(mosq, nullptr, statusTopic.c_str(), 1);
    if (subRc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[WARN] status subscribe failed rc=" << subRc
                  << " (" << mosquitto_strerror(subRc) << ") topic=" << statusTopic << "\n";
    } else {
        std::cout << "[OK] subscribed status topic: " << statusTopic << "\n";
    }

    std::cout << "[OK] start tracking (init once by ArUco id=0, then dead-reckoning)\n";
    std::cout << "[OK] build source: " << __FILE__ << "\n";

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
    FilterState filter{};
    std::array<bool, 14> wallPublished{};
    std::array<bool, 14> wallSeen{};
    bool wallAllSeenLogged = false;
    auto last_wait_log = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    auto last_pose_pub = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    double pose_x = 0.0;
    double pose_y = 0.0;
    double pose_yaw = 0.0;

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            const auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_wait_log).count() >= 1.0) {
                std::cerr << "[INIT] waiting marker for init while frame empty\n";
                last_wait_log = now;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / kPublishHz));
            mosquitto_loop(mosq, 0, 1);
            continue;
        }

        const auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - filter.last_tick).count();
        filter.last_tick = now;
        if (dt < 0.0) dt = 0.0;
        if (dt > 0.2) dt = 0.2;

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        detector.detectMarkers(frame, corners, ids);

        bool marker_detected = false;
        double det_x = 0.0;
        double det_y = 0.0;
        double det_yaw = 0.0;
        double det_alpha = 0.25;

        for (size_t i = 0; i < ids.size(); i++) {
            cv::Point2f c(0, 0);
            for (const auto& p : corners[i]) c += p;
            c *= (1.0f / 4.0f);
            cv::Point2f w = applyHomography(H_img2world, c);

            const int markerId = ids[i];
            if (markerId >= 10 && markerId <= 13 && !wallPublished[markerId]) {
                wallPublished[markerId] = true;
                wallSeen[markerId] = true;
                const long long ts_ms = nowSec() * 1000;
                char wbuf[192];
                std::snprintf(wbuf, sizeof(wbuf),
                              "{\"id\":%d,\"x\":%.3f,\"y\":%.3f,\"frame\":\"world\",\"ts_ms\":%lld}",
                              markerId, w.x, w.y, ts_ms);
                mqttPublishJson(mosq, wallTopic, wbuf, true);
                std::cout << std::fixed << std::setprecision(3)
                          << "[WALL] id=" << markerId
                          << " x=" << w.x
                          << " y=" << w.y << "\n";
                if (!wallAllSeenLogged && allWallMarkersSeen(wallSeen)) {
                    wallAllSeenLogged = true;
                    std::cout << "[WALL] seen=true ids=10,11,12,13" << "\n";
                }
            }

            if (markerId >= 10 && markerId <= 13 && !wallSeen[markerId]) {
                wallSeen[markerId] = true;
                std::cout << std::fixed << std::setprecision(3)
                          << "[WALL_SEEN] id=" << markerId
                          << " seen=true"
                          << "\n";
                if (!wallAllSeenLogged && allWallMarkersSeen(wallSeen)) {
                    wallAllSeenLogged = true;
                    std::cout << "[WALL] seen=true ids=10,11,12,13" << "\n";
                }
            }

            if (markerId != targetMarkerId) continue;
            const double area = cv::contourArea(corners[i]);
            if (area < 5.0) continue;

            double yaw_raw = calcYawWorld(H_img2world, c, corners[i]);
            constexpr double YAW_X_PLUS_REF = 3.194;
            det_yaw = normalizeAngle(yaw_raw - YAW_X_PLUS_REF);
            det_x = w.x;
            det_y = w.y;
            det_alpha = std::clamp(0.15 + area * 0.0008, 0.2, 0.8);
            marker_detected = true;
            break;
        }

        if (!filter.initialized) {
            if (marker_detected) {
                pose_x = det_x;
                pose_y = det_y;
                pose_yaw = det_yaw;
                filter.initialized = true;
                filter.seen_once = true;
                filter.last_marker_seen = now;
                filter.marker_x = det_x;
                filter.marker_y = det_y;
                filter.marker_yaw = det_yaw;
                filter.marker_conf = 1.0;
                std::cout << std::fixed << std::setprecision(3)
                          << "[INIT] id=0 x=" << pose_x
                          << " y=" << pose_y
                          << " yaw=" << pose_yaw << "\n";
            } else {
                const auto now_wait = std::chrono::steady_clock::now();
                if (std::chrono::duration<double>(now_wait - last_wait_log).count() >= 1.0) {
                    std::cout << std::fixed << std::setprecision(1)
                              << "[INIT] waiting ArUco id=0... "
                              << "seen10=" << wallSeen[10]
                              << ",seen11=" << wallSeen[11]
                              << ",seen12=" << wallSeen[12]
                              << ",seen13=" << wallSeen[13] << "\n";
                    last_wait_log = now_wait;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / kPublishHz));
            mosquitto_loop(mosq, 0, 1);
            continue;
        }

        if (marker_detected) {
            const double jump = std::hypot(det_x - pose_x, det_y - pose_y);
            double alpha = det_alpha;
            if (jump > kMaxPoseJumpM) alpha *= 0.2;
            if (jump > (kMaxPoseJumpM * 1.5)) alpha = 0.05;

            pose_x = pose_x * (1.0 - alpha) + det_x * alpha;
            pose_y = pose_y * (1.0 - alpha) + det_y * alpha;
            pose_yaw = blendAngle(pose_yaw, det_yaw, alpha);

            const double marker_dt = std::chrono::duration<double>(now - filter.last_marker_seen).count();
            if (marker_dt > 1e-3 && filter.seen_once) {
                const double vx = (det_x - filter.marker_x) / marker_dt;
                const double vy = (det_y - filter.marker_y) / marker_dt;
                filter.marker_vx = 0.7 * filter.marker_vx + 0.3 * vx;
                filter.marker_vy = 0.7 * filter.marker_vy + 0.3 * vy;
            }
            filter.last_marker_seen = now;
            filter.marker_x = det_x;
            filter.marker_y = det_y;
            filter.marker_yaw = det_yaw;
            filter.marker_conf = std::min(1.0, filter.marker_conf + 0.2);
            filter.seen_once = true;
        } else {
            filter.marker_conf = std::max(0.0, filter.marker_conf - 0.02);
        }

        if (marker_detected) {
            const auto now_pub = std::chrono::steady_clock::now();
            if (now_pub - last_pose_pub >= std::chrono::seconds(1)) {
                const long long ts = nowSec();
                const long long ts_ms = ts * 1000;
                char buf[320];
                std::snprintf(buf, sizeof(buf),
                              "{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.6f,\"frame\":\"world\",\"ts\":%lld,\"ts_ms\":%lld,\"marker_conf\":%.3f}",
                              pose_x, pose_y, pose_yaw, ts, ts_ms, filter.marker_conf);
                mqttPublishJson(mosq, mqttTopic, buf);
                std::cout << std::fixed << std::setprecision(3)
                          << "[POSE] x=" << pose_x
                          << " y=" << pose_y
                          << " yaw=" << pose_yaw
                          << " conf=" << filter.marker_conf << "\n";
                last_pose_pub = now_pub;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / kPublishHz));

        mosquitto_loop(mosq, 0, 1);
    }

    close(i2c_fd);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
