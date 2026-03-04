// ./rc_control_node 192.168.100.10 1883 wiserisk/rc/goal wiserisk/p1/pose wiserisk/rc/safety wiserisk/rc/status
// g++ -O2 -std=c++17 p1_tracker.cpp -o p1_tracker $(pkg-config --cflags --libs opencv4) -lmosquitto

#include <iostream>
#include <iomanip>
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
                            const std::string& json)
{
    int rc = mosquitto_publish(
        mosq,
        nullptr,
        topic.c_str(),
        (int)json.size(),
        json.c_str(),
        1,
        false);
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
};

static void onMqttMessage(struct mosquitto*, void* userdata, const struct mosquitto_message* msg)
{
    if (!userdata || !msg || !msg->topic || !msg->payload || msg->payloadlen <= 0) return;
    auto* st = static_cast<TrackerState*>(userdata);
    const std::string payload(static_cast<const char*>(msg->payload), msg->payloadlen);
    const std::string topic(msg->topic);

    if (topic != st->status_topic) return;

    // rc_control_node status json example: {"mode":"REV_TRACKING",...}
    if (payload.find("\"mode\":\"REV_TRACKING\"") != std::string::npos) {
        st->motion_mode = TrackerState::MotionMode::REV_TRACKING;
    } else if (payload.find("\"mode\":\"TRACKING\"") != std::string::npos) {
        st->motion_mode = TrackerState::MotionMode::TRACKING;
    } else if (payload.find("\"mode\":\"REACHED\"") != std::string::npos ||
               payload.find("\"mode\":\"SAFE_STOP\"") != std::string::npos ||
               payload.find("\"mode\":\"WAIT_INPUT\"") != std::string::npos) {
        st->motion_mode = TrackerState::MotionMode::HOLD;
    }
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
    const std::string statusTopic = "wiserisk/rc/status";
    constexpr double kSpeedMps = 1.0;      // dead-reckoning speed (match rc_control_node setting)
    constexpr int kPublishHz = 10;         // publish rate after init
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

    bool initialized = false;
    double pose_x = 0.0;
    double pose_y = 0.0;
    double pose_yaw = 0.0;
    auto last_tick = std::chrono::steady_clock::now();

    while (true) {
        if (!initialized) {
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) {
                std::cerr << "[WARN] empty frame\n";
                continue;
            }

            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids;
            detector.detectMarkers(frame, corners, ids);

            for (size_t i = 0; i < ids.size(); i++) {
                if (ids[i] != targetMarkerId) continue;

                cv::Point2f c(0, 0);
                for (const auto& p : corners[i]) c += p;
                c *= (1.0f / 4.0f);

                cv::Point2f w = applyHomography(H_img2world, c);
                double yaw_raw = calcYawWorld(H_img2world, c, corners[i]);
                constexpr double YAW_X_PLUS_REF = -3.101;
                double yaw = normalizeAngle(yaw_raw - YAW_X_PLUS_REF);

                pose_x = w.x;
                pose_y = w.y;
                pose_yaw = yaw;
                initialized = true;
                last_tick = std::chrono::steady_clock::now();

                std::cout << std::fixed << std::setprecision(3)
                          << "[INIT] id=0 x=" << pose_x
                          << " y=" << pose_y
                          << " yaw=" << pose_yaw << "\n";
                break;
            }
            if (!initialized) {
                std::cout << "[INIT] waiting ArUco id=0...\n";
            }
        } else {
            const auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_tick).count();
            last_tick = now;
            if (dt < 0.0) dt = 0.0;
            if (dt > 0.2) dt = 0.2;

            int16_t raw_gyro_z = 0;
            double gyro_z_rps = 0.0;
            if (read16(i2c_fd, 0x47, raw_gyro_z)) {
                const double dps = static_cast<double>(raw_gyro_z) / 131.0;
                gyro_z_rps = dps * kDeg2Rad - gyro_bias;
            }
            pose_yaw = normalizeAngle(pose_yaw + gyro_z_rps * dt);

            double v_signed = 0.0;
            if (tracker_state.motion_mode == TrackerState::MotionMode::TRACKING) {
                v_signed = kSpeedMps;
            } else if (tracker_state.motion_mode == TrackerState::MotionMode::REV_TRACKING) {
                v_signed = -kSpeedMps;
            }
            pose_x += v_signed * dt * std::cos(pose_yaw);
            pose_y += v_signed * dt * std::sin(pose_yaw);

            const long long ts = nowSec();
            const long long ts_ms = ts * 1000;

            std::cout << std::fixed << std::setprecision(3)
                      << "[DR] x=" << pose_x
                      << " y=" << pose_y
                      << " yaw=" << pose_yaw
                      << " gyro_z=" << gyro_z_rps
                      << " v=" << v_signed
                      << " mode=" << (tracker_state.motion_mode == TrackerState::MotionMode::TRACKING ? "TRACKING"
                                        : tracker_state.motion_mode == TrackerState::MotionMode::REV_TRACKING ? "REV_TRACKING"
                                                                                                              : "HOLD")
                      << "\n";

            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.6f,\"frame\":\"world\",\"ts\":%lld,\"ts_ms\":%lld}",
                          pose_x, pose_y, pose_yaw, ts, ts_ms);

            mqttPublishJson(mosq, mqttTopic, buf);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / kPublishHz));
        }

        mosquitto_loop(mosq, 0, 1);
    }

    close(i2c_fd);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
