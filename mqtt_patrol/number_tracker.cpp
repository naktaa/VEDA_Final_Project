// ./rc_control_node 192.168.100.10 1883 wiserisk/rc/goal wiserisk/p1/pose wiserisk/rc/safety wiserisk/rc/status
/*
g++ -O2 -std=c++17 p1_tracker.cpp -o p1_tracker $(pkg-config --cflags --libs opencv4) -lmosquitto
./p1_tracker
*/

#include <iostream>
#include <iomanip>
#include <array>
#include <chrono>
#include <string>
#include <cmath>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <mosquitto.h>

// ---------- utils ----------
static bool loadHomography(const std::string& path, cv::Mat& H_img2world)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["H_img2world"] >> H_img2world;   // 우리가 저장할 키 이름
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

    const cv::Point2f headPx = centerPx + dir * 30.0f; // 30 px ahead to estimate heading
    const cv::Point2f w0 = applyHomography(H_img2world, centerPx);
    const cv::Point2f w1 = applyHomography(H_img2world, headPx);
    return std::atan2((double)(w1.y - w0.y), (double)(w1.x - w0.x));
}

static long long nowSec()
{
    return std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::system_clock::now().time_since_epoch()
           ).count();
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
        1,  // qos
        false
    );
    return (rc == MOSQ_ERR_SUCCESS);
}

// ---------- main ----------
int main(int argc, char** argv)
{
    // ---- args (간단 버전) ----
    const std::string rtspUrl =
        "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp"; // user 제공
    const std::string homographyYaml = "H_img2world.yaml";              // 아래에서 만들 파일
    const int targetMarkerId = 0;                                       // P1에 붙일 ArUco ID
    const std::string mqttHost = "192.168.100.10";
    const int mqttPort = 1883;
    const std::string mqttTopic = "wiserisk/p1/pose";                   // Qt가 구독할 토픽

    // ---- load H (image->world) ----
    cv::Mat H_img2world;
    if (!loadHomography(homographyYaml, H_img2world)) {
        std::cerr << "[ERR] failed to load homography: " << homographyYaml << "\n";
        std::cerr << "      expected key: H_img2world\n";
        return 1;
    }

    // ---- open RTSP ----
    cv::VideoCapture cap;
    cap.open(rtspUrl); // (조건부) 여기서 안 열리면 아래 'GStreamer 파이프라인' 방식으로 바꿔야 함

    if (!cap.isOpened()) {
        std::cerr << "[ERR] RTSP open failed\n";
        return 2;
    }

    // ---- aruco setup ----
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params;
    cv::aruco::ArucoDetector detector(dict, params);

    // ---- mqtt ----
    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("p1_tracker", true, nullptr);
    if (!mosq) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        return 3;
    }

    if (mosquitto_connect(mosq, mqttHost.c_str(), mqttPort, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_connect failed\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 4;
    }

    std::cout << "[OK] start tracking\n";
    std::array<bool, 14> printedOnce{};
    auto lastDetectLog = std::chrono::steady_clock::now();

    // ---- loop ----
    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "[WARN] empty frame\n";
            continue;
        }

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        detector.detectMarkers(frame, corners, ids);

        // 1초에 1번, 현재 검출된 marker ID 목록 디버그 출력
        auto now = std::chrono::steady_clock::now();
        if (now - lastDetectLog >= std::chrono::seconds(1)) {
            std::ostringstream oss;
            oss << "[DETECT] ids=";
            if (ids.empty()) {
                oss << "(none)";
            } else {
                for (size_t k = 0; k < ids.size(); ++k) {
                    if (k) oss << ",";
                    oss << ids[k];
                }
            }
            std::cout << oss.str() << "\n";
            lastDetectLog = now;
        }

        // targetMarkerId만 publish + id 10/11/12/13 좌표는 각 1회만 출력
        for (size_t i = 0; i < ids.size(); i++) {
            // corners[i] = 4개 점. 중심점 계산
            cv::Point2f c(0, 0);
            for (const auto& p : corners[i]) c += p;
            c *= (1.0f / 4.0f);

            // pixel -> world
            cv::Point2f w = applyHomography(H_img2world, c);

            const int markerId = ids[i];
            if (markerId >= 10 && markerId <= 13 && !printedOnce[markerId]) {
                printedOnce[markerId] = true;
                std::cout << std::fixed << std::setprecision(3)
                          << "[MARKER_ONCE] id=" << markerId
                          << " x=" << w.x
                          << " y=" << w.y << "\n";
            }

            if (markerId != targetMarkerId) continue;

            double yaw = calcYawWorld(H_img2world, c, corners[i]);

            // timestamp(sec): Qt 권장 예시와 같은 epoch seconds
            const long long ts = nowSec();

            std::cout << std::fixed << std::setprecision(3)
                      << "[ARUCO] id=" << markerId
                      << " x=" << w.x
                      << " y=" << w.y
                      << " yaw=" << yaw
                      << " ts=" << ts << "\n";

            // Qt Pose JSON:
            // {"x":0.92,"y":1.40,"yaw":1.57,"frame":"world","ts":1234567890}
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.6f,\"frame\":\"world\",\"ts\":%lld}",
                          w.x, w.y, yaw, ts);

            mqttPublishJson(mosq, mqttTopic, buf);
        }

        // mosquitto network loop
        mosquitto_loop(mosq, 0, 1);

        // ESC 종료(로컬 디버깅용) — headless면 제거
        // if (cv::waitKey(1) == 27) break;
    }

    // ---- cleanup ----
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
