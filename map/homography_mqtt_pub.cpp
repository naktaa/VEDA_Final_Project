#include <mosquitto.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

static std::atomic<bool> g_run{true};

static void onSignal(int) {
    g_run = false;
}

static bool loadHomography(const std::string& yamlPath, cv::Mat& H_img2world) {
    cv::FileStorage fs(yamlPath, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["H_img2world"] >> H_img2world;
    fs.release();

    if (H_img2world.empty() || H_img2world.rows != 3 || H_img2world.cols != 3) return false;
    if (H_img2world.type() != CV_64F) H_img2world.convertTo(H_img2world, CV_64F);
    return true;
}

static long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

static std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default: out += c; break;
        }
    }
    return out;
}

static std::string buildHomographyPayload(const cv::Mat& H, const std::string& yamlPath) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(16);
    oss << "{";
    oss << "\"type\":\"homography\",";
    oss << "\"key\":\"H_img2world\",";
    oss << "\"yaml\":\"" << jsonEscape(yamlPath) << "\",";
    oss << "\"rows\":3,\"cols\":3,";
    oss << "\"data\":[";
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (r != 0 || c != 0) oss << ",";
            oss << H.at<double>(r, c);
        }
    }
    oss << "],";
    oss << "\"ts_ms\":" << nowMs();
    oss << "}";
    return oss.str();
}

static bool publishJson(mosquitto* mosq, const std::string& topic, const std::string& payload, bool retain = true) {
    const int rc = mosquitto_publish(
        mosq,
        nullptr,
        topic.c_str(),
        static_cast<int>(payload.size()),
        payload.c_str(),
        1,
        retain);
    return (rc == MOSQ_ERR_SUCCESS);
}

int main(int argc, char** argv) {
    const std::string yamlPath = (argc > 1) ? argv[1] : "H_img2world.yaml";
    const std::string mqttHost = (argc > 2) ? argv[2] : "192.168.100.10";
    const int mqttPort = (argc > 3) ? std::stoi(argv[3]) : 1883;
    const std::string homographyTopic = (argc > 4) ? argv[4] : "wiserisk/map/H_img2world";
    const int intervalMs = (argc > 5) ? std::stoi(argv[5]) : 1000;
    const bool once = (argc > 6) ? (std::stoi(argv[6]) != 0) : true;

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("homography_map_pub", true, nullptr);
    if (!mosq) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        mosquitto_lib_cleanup();
        return 1;
    }

    if (mosquitto_connect(mosq, mqttHost.c_str(), mqttPort, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_connect failed: " << mqttHost << ":" << mqttPort << "\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 2;
    }

    std::cout << "[INFO] homographyTopic=" << homographyTopic
              << " intervalMs=" << intervalMs << " once=" << (once ? 1 : 0) << "\n";

    std::string lastHomographyPayload;

    while (g_run) {
        cv::Mat H;
        if (!loadHomography(yamlPath, H)) {
            std::cerr << "[WARN] failed to load yaml: " << yamlPath << "\n";
            if (once) {
                mosquitto_disconnect(mosq);
                mosquitto_destroy(mosq);
                mosquitto_lib_cleanup();
                return 3;
            }
        } else {
            const std::string hPayload = buildHomographyPayload(H, yamlPath);
            if (hPayload != lastHomographyPayload) {
                if (publishJson(mosq, homographyTopic, hPayload, true)) {
                    std::cout << "[PUB][H] " << hPayload << "\n";
                    lastHomographyPayload = hPayload;
                } else {
                    std::cerr << "[WARN] homography publish failed\n";
                    mosquitto_reconnect(mosq);
                }
            }

            if (once) break;
        }

        mosquitto_loop(mosq, 0, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
    }

    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
