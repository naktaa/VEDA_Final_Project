#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <mosquitto.h>

#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>

// ---------- 간단 JSON 만들기 ----------
static std::string make_json(const std::string& id, double x, double y, double yaw_deg, double ts_sec)
{
    std::ostringstream oss;
    oss << "{"
        << "\"id\":\"" << id << "\","
        << "\"x\":" << x << ","
        << "\"y\":" << y << ","
        << "\"yaw\":" << yaw_deg << ","
        << "\"t\":" << ts_sec
        << "}";
    return oss.str();
}

// ---------- Homography: 3x3 ----------
struct H3 {
    double a[3][3];
};

static bool load_homography_yaml(const std::string& path, H3& out)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    cv::Mat H;
    fs["H_i2w"] >> H; // YAML에 H_i2w 키로 저장한다고 가정
    if (H.empty() || H.rows != 3 || H.cols != 3) return false;

    H.convertTo(H, CV_64F);
    for (int r=0;r<3;r++)
        for (int c=0;c<3;c++)
            out.a[r][c] = H.at<double>(r,c);
    return true;
}

static cv::Point2d img_to_world(const H3& H, double u, double v)
{
    double X = H.a[0][0]*u + H.a[0][1]*v + H.a[0][2];
    double Y = H.a[1][0]*u + H.a[1][1]*v + H.a[1][2];
    double W = H.a[2][0]*u + H.a[2][1]*v + H.a[2][2];
    if (std::abs(W) < 1e-9) W = 1e-9;
    return { X/W, Y/W };
}

// ---------- EMA 필터 ----------
struct EMA {
    bool inited=false;
    double alpha=0.3;
    double x=0, y=0, yaw=0;

    void update(double nx, double ny, double nyaw) {
        if (!inited) { x=nx; y=ny; yaw=nyaw; inited=true; return; }
        x = alpha*nx + (1.0-alpha)*x;
        y = alpha*ny + (1.0-alpha)*y;
        yaw = alpha*nyaw + (1.0-alpha)*yaw;
    }
};

static double now_sec()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

int main(int argc, char** argv)
{
    // ---- 설정(필요하면 argv로 바꿔도 됨) ----
    const std::string rtsp_url   = "rtsp://admin:PASS@192.168.100.22/profile1/media.smp"; // 너 환경으로 교체
    const std::string broker_ip  = "192.168.100.10"; // central server broker IP
    const int broker_port        = 1883;
    const std::string topic_state= "wiserisk/rc/state";

    const int target_marker_id   = 0;        // P1에 붙인 ArUco ID
    const std::string robot_id   = "P1";     // Qt에서 표시할 ID

    const std::string homography_yaml = "./H.yaml"; // H_i2w 저장파일

    // ---- Homography 로드 ----
    H3 H{};
    if (!load_homography_yaml(homography_yaml, H)) {
        std::cerr << "[ERR] Homography YAML load failed: " << homography_yaml << "\n";
        std::cerr << "YAML must contain key: H_i2w (3x3)\n";
        return 1;
    }

    // ---- MQTT 초기화 ----
    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new(nullptr, true, nullptr);
    if (!mosq) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        return 1;
    }
    if (mosquitto_connect(mosq, broker_ip.c_str(), broker_port, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_connect failed\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    // ---- RTSP 캡처 ----
    // RPi에서 안정적으로는 GStreamer 파이프라인을 쓰는 게 좋긴 한데,
    // 환경마다 플러그인이 달라서 일단 OpenCV 기본으로 열고,
    // 필요하면 아래 주석의 gst 문자열로 교체하는 방식 추천.
    cv::VideoCapture cap(rtsp_url);
    // 예) cap.open("rtspsrc location=... latency=100 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink", cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        std::cerr << "[ERR] RTSP open failed\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    // ---- ArUco 설정 ----
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params;
    // Detection tuning for 170mm markers at 1024x768
    params.adaptiveThreshWinSizeMin = 3;
    params.adaptiveThreshWinSizeMax = 53;
    params.adaptiveThreshWinSizeStep = 4;
    params.minMarkerPerimeterRate = 0.02;
    params.maxMarkerPerimeterRate = 4.0;
    params.polygonalApproxAccuracyRate = 0.03;
    params.minCornerDistanceRate = 0.05;
    params.minDistanceToBorder = 3;
    params.minMarkerDistanceRate = 0.05;
    params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    params.cornerRefinementWinSize = 5;
    params.cornerRefinementMaxIterations = 50;
    params.cornerRefinementMinAccuracy = 0.05;
    params.perspectiveRemovePixelPerCell = 8;
    params.perspectiveRemoveIgnoredMarginPerCell = 0.13;
    params.maxErroneousBitsInBorderRate = 0.35;
    params.errorCorrectionRate = 0.6;
    cv::aruco::ArucoDetector detector(dict, params);

    EMA ema;
    ema.alpha = 0.25; // 튐 줄이기

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector.detectMarkers(frame, corners, ids);

        // target id 찾기
        int idx = -1;
        for (int i=0;i<(int)ids.size();++i) {
            if (ids[i] == target_marker_id) { idx=i; break; }
        }
        if (idx < 0) {
            // 못 찾으면 publish 안 함(또는 마지막 값 유지 정책도 가능)
            mosquitto_loop(mosq, 0, 1);
            continue;
        }

        const auto& c = corners[idx]; // 4 corners
        // 중심 픽셀
        double u=0, v=0;
        for (auto& p : c) { u += p.x; v += p.y; }
        u *= 0.25; v *= 0.25;

        // yaw(화면 기준): corner0->corner1 벡터
        double dx = (double)c[1].x - (double)c[0].x;
        double dy = (double)c[1].y - (double)c[0].y;
        double yaw_deg = std::atan2(dy, dx) * 180.0 / CV_PI;

        // 픽셀 -> 월드(m)
        auto w = img_to_world(H, u, v);

        // EMA로 안정화
        ema.update(w.x, w.y, yaw_deg);

        // MQTT publish
        const double t = now_sec();
        const std::string payload = make_json(robot_id, ema.x, ema.y, ema.yaw, t);

        int rc = mosquitto_publish(mosq, nullptr, topic_state.c_str(),
                                  (int)payload.size(), payload.c_str(),
                                  0, false);
        if (rc != MOSQ_ERR_SUCCESS) {
            // 재연결 시도
            mosquitto_reconnect(mosq);
        }

        mosquitto_loop(mosq, 0, 1);

        // 너무 빠르면 네트워크/CPU 부담 → 10~20Hz면 충분
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // never reached
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
