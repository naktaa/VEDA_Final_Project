#include <iostream>
#include <chrono>
#include <string>
#include <cmath>
#include <thread>
#include <cstdlib>
#include <vector>
#include <mutex>
#include <atomic>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <mosquitto.h>

// ---------- utils ----------
struct CameraModel {
    cv::Mat K;
    cv::Mat dist;
};

static bool loadHomography(const std::string& path, cv::Mat& H_img2world)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ERR] loadHomography: cannot open " << path << "\n";
        return false;
    }
    fs["H_img2world"] >> H_img2world;
    fs.release();

    if (H_img2world.empty() || H_img2world.rows != 3 || H_img2world.cols != 3) {
        std::cerr << "[ERR] loadHomography: H_img2world invalid\n";
        return false;
    }

    std::cout << "[INFO] H_img2world loaded from " << path << ":\n";
    for (int r = 0; r < 3; r++) {
        std::cout << "  [";
        for (int c = 0; c < 3; c++)
            std::cout << " " << H_img2world.at<double>(r, c);
        std::cout << " ]\n";
    }
    return true;
}

static bool loadCameraModel(const std::string& path, CameraModel& out)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    cv::Mat K, dist;
    if (!fs["camera_matrix"].isNone()) fs["camera_matrix"] >> K;
    if (!fs["K"].isNone()) fs["K"] >> K;
    if (!fs["dist_coeffs"].isNone()) fs["dist_coeffs"] >> dist;
    if (!fs["distortion_coefficients"].isNone()) fs["distortion_coefficients"] >> dist;
    fs.release();

    if (K.empty() || K.rows != 3 || K.cols != 3) return false;
    if (dist.empty()) dist = cv::Mat::zeros(1, 5, CV_64F);
    if (K.type() != CV_64F) K.convertTo(K, CV_64F);
    if (dist.type() != CV_64F) dist.convertTo(dist, CV_64F);

    out.K = K;
    out.dist = dist;
    return true;
}

// ★ H_img2world에서 R_world_cam 추정 (yaw 계산용)
static cv::Matx33d estimateRworldCam(const cv::Mat& H_img2world, const cv::Mat& K)
{
    cv::Mat H_world2img = H_img2world.inv();
    cv::Mat K_inv = K.inv();

    cv::Mat h1 = H_world2img.col(0);
    cv::Mat h2 = H_world2img.col(1);

    cv::Mat r1m = K_inv * h1;
    cv::Mat r2m = K_inv * h2;

    double norm1 = cv::norm(r1m);
    if (norm1 < 1e-9) return cv::Matx33d::eye();

    r1m /= norm1;
    r2m /= norm1;

    cv::Vec3d r1(r1m.at<double>(0), r1m.at<double>(1), r1m.at<double>(2));
    cv::Vec3d r2(r2m.at<double>(0), r2m.at<double>(1), r2m.at<double>(2));
    cv::Vec3d r3 = r1.cross(r2);

    cv::Mat R_cam_world = (cv::Mat_<double>(3,3) <<
        r1[0], r2[0], r3[0],
        r1[1], r2[1], r3[1],
        r1[2], r2[2], r3[2]);

    cv::SVD svd(R_cam_world);
    cv::Mat R_ortho = svd.u * svd.vt;
    if (cv::determinant(R_ortho) < 0) R_ortho *= -1.0;

    cv::Matx33d Rcw(
        R_ortho.at<double>(0,0), R_ortho.at<double>(0,1), R_ortho.at<double>(0,2),
        R_ortho.at<double>(1,0), R_ortho.at<double>(1,1), R_ortho.at<double>(1,2),
        R_ortho.at<double>(2,0), R_ortho.at<double>(2,1), R_ortho.at<double>(2,2));

    return Rcw.t(); // R_world_cam
}

struct MarkerConfig {
    cv::Matx33d R_c_m;
    cv::Vec3d t_c_m;
    int priority = 9;
};

static bool getMarkerConfig(int id, double cubeSize, MarkerConfig& out)
{
    const double h = cubeSize * 0.5;
    switch (id) {
        case 21: // top
            out.R_c_m = cv::Matx33d(-1,0,0, 0,-1,0, 0,0,1);
            out.t_c_m = cv::Vec3d(0.0, 0.0, h);
            out.priority = 0;
            return true;
        case 24: // front
            out.R_c_m = cv::Matx33d(1,0,0, 0,0,1, 0,-1,0);
            out.t_c_m = cv::Vec3d(0.0, h, 0.0);
            out.priority = 3;
            return true;
        case 23: // right
            out.R_c_m = cv::Matx33d(0,0,1, -1,0,0, 0,-1,0);
            out.t_c_m = cv::Vec3d(h, 0.0, 0.0);
            out.priority = 1;
            return true;
        case 25: // left
            out.R_c_m = cv::Matx33d(0,0,-1, 1,0,0, 0,-1,0);
            out.t_c_m = cv::Vec3d(-h, 0.0, 0.0);
            out.priority = 2;
            return true;
        case 22: // back
            out.R_c_m = cv::Matx33d(-1,0,0, 0,0,-1, 0,-1,0);
            out.t_c_m = cv::Vec3d(0.0, -h, 0.0);
            out.priority = 4;
            return true;
        default:
            return false;
    }
}

static cv::Matx33d toMatx33d(const cv::Mat& R)
{
    return cv::Matx33d(
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
}

static double normalizeAngle(double a)
{
    constexpr double PI = 3.14159265358979323846;
    while (a >  PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

static long long nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch()
           ).count();
}

static bool mqttPublishJson(mosquitto* mosq,
                            const std::string& topic,
                            const std::string& json)
{
    int rc = mosquitto_publish(mosq, nullptr, topic.c_str(),
                               (int)json.size(), json.c_str(), 1, false);
    if (rc != MOSQ_ERR_SUCCESS)
        std::cerr << "[WARN] mqtt publish failed rc=" << rc << "\n";
    return (rc == MOSQ_ERR_SUCCESS);
}

static bool openRtspCapture(const std::string& rtspUrl, cv::VideoCapture& cap)
{
    const std::string gst =
        "rtspsrc location=" + rtspUrl +
        " protocols=tcp latency=50 drop-on-latency=true do-rtsp-keep-alive=true tcp-timeout=5000000 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! queue leaky=downstream max-size-buffers=1 ! "
        "appsink sync=false max-buffers=1 drop=true";

    if (cap.open(gst, cv::CAP_GSTREAMER)) return true;

    setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
           "rtsp_transport;tcp|fflags;nobuffer|max_delay;500000|stimeout;5000000", 1);
    if (cap.open(rtspUrl, cv::CAP_FFMPEG)) {
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
        return true;
    }
    return false;
}

// ---------- shared homography ----------
struct SharedHomography {
    std::mutex mtx;
    cv::Mat H_img2world;
    cv::Matx33d R_world_cam;
    std::atomic<long long> updateCount{0};
};

struct HomographyContext {
    SharedHomography* shared = nullptr;
    const CameraModel* cam   = nullptr;
};

static bool parseHomographyJson(const std::string& json, cv::Mat& H_out)
{
    if (json.find("H_img2world") == std::string::npos) return false;

    const size_t dataPos = json.find("\"data\"");
    if (dataPos == std::string::npos) return false;

    const size_t lb = json.find('[', dataPos);
    const size_t rb = json.find(']', lb == std::string::npos ? dataPos : lb);
    if (lb == std::string::npos || rb == std::string::npos || rb <= lb) return false;

    std::string nums = json.substr(lb + 1, rb - lb - 1);
    for (char& c : nums) if (c == ',') c = ' ';

    std::stringstream ss(nums);
    std::vector<double> v;
    double x = 0.0;
    while (ss >> x) v.push_back(x);
    if (v.size() != 9) return false;

    H_out = (cv::Mat_<double>(3,3) <<
        v[0], v[1], v[2],
        v[3], v[4], v[5],
        v[6], v[7], v[8]);
    return true;
}

static void on_message_cb(struct mosquitto*,
                          void* userdata,
                          const struct mosquitto_message* msg)
{
    if (!userdata || !msg || !msg->topic || !msg->payload || msg->payloadlen <= 0) return;
    const std::string topic(msg->topic);
    if (topic != "wiserisk/map/H_img2world") return;

    const std::string payload((const char*)msg->payload, (size_t)msg->payloadlen);
    cv::Mat newH;
    if (!parseHomographyJson(payload, newH)) {
        std::cerr << "[WARN] homography payload parse failed\n";
        return;
    }

    auto* ctx = static_cast<HomographyContext*>(userdata);
    if (!ctx || !ctx->shared || !ctx->cam) return;

    cv::Matx33d Rwc = estimateRworldCam(newH, ctx->cam->K);

    {
        std::lock_guard<std::mutex> lk(ctx->shared->mtx);
        ctx->shared->H_img2world = newH.clone();
        ctx->shared->R_world_cam = Rwc;
        ctx->shared->updateCount.fetch_add(1);
    }
    std::cout << "[INFO] H_img2world updated from MQTT\n";
}

// ---------- main ----------
int main(int argc, char** argv)
{
    const std::string rtspUrl = (argc > 1)
        ? argv[1]
        : "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
    const int targetMarkerMinId = 21;
    const int targetMarkerMaxId = 25;
    const std::string mqttHost       = (argc > 2) ? argv[2] : "192.168.100.7";
    const int         mqttPort       = (argc > 3) ? std::stoi(argv[3]) : 1883;
    const std::string mqttTopic      = (argc > 4) ? argv[4] : "wiserisk/p1/pose";
    const std::string homographyYaml = (argc > 5) ? argv[5] : "config/H_img2world.yaml";
    const std::string cameraYaml     = (argc > 6) ? argv[6] : "config/camera.yaml";
    const double markerSize = (argc > 7) ? std::stod(argv[7]) : 0.17;
    const double cubeSize   = (argc > 8) ? std::stod(argv[8]) : 0.17;

    CameraModel cam;
    if (!loadCameraModel(cameraYaml, cam)) {
        std::cerr << "[ERR] failed to load camera model: " << cameraYaml << "\n";
        return 1;
    }

    cv::Mat H_img2world;
    if (!loadHomography(homographyYaml, H_img2world)) {
        std::cerr << "[ERR] failed to load homography: " << homographyYaml << "\n";
        return 2;
    }

    SharedHomography shared;
    shared.H_img2world = H_img2world.clone();
    shared.R_world_cam = estimateRworldCam(H_img2world, cam.K);
    std::cout << "[INFO] R_world_cam estimated from H_img2world\n";

    cv::VideoCapture cap;
    if (!openRtspCapture(rtspUrl, cap)) {
        std::cerr << "[ERR] RTSP open failed\n";
        return 3;
    }

    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params;
    cv::aruco::ArucoDetector detector(dict, params);

    mosquitto_lib_init();
    HomographyContext ctx;
    ctx.shared = &shared;
    ctx.cam    = &cam;

    mosquitto* mosq = mosquitto_new(nullptr, true, &ctx);
    if (!mosq) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        return 4;
    }

    mosquitto_reconnect_delay_set(mosq, 1, 10, true);
    mosquitto_message_callback_set(mosq, on_message_cb);

    if (mosquitto_connect(mosq, mqttHost.c_str(), mqttPort, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_connect failed\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 5;
    }

    mosquitto_subscribe(mosq, nullptr, "wiserisk/map/H_img2world", 1);
    mosquitto_loop_start(mosq);

    std::cout << "[OK] start tracking  pub=" << mqttTopic << "\n";

    int readFailCount  = 0;
    long long frameCount   = 0;
    long long publishCount = 0;
    auto lastStatTs = std::chrono::steady_clock::now();

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            readFailCount++;
            if (readFailCount % 20 == 0) {
                std::cerr << "[WARN] frame read failed x" << readFailCount << "\n";
                cap.release();
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                if (!openRtspCapture(rtspUrl, cap))
                    std::cerr << "[WARN] RTSP reconnect failed\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }

        readFailCount = 0;
        frameCount++;

        cv::Mat H;
        cv::Matx33d R_world_cam;
        {
            std::lock_guard<std::mutex> lk(shared.mtx);
            H           = shared.H_img2world.clone();
            R_world_cam = shared.R_world_cam;
        }

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        detector.detectMarkers(frame, corners, ids);

        std::vector<cv::Vec3d> rvecs, tvecs;
        if (!ids.empty())
            cv::aruco::estimatePoseSingleMarkers(corners, markerSize, cam.K, cam.dist, rvecs, tvecs);

        // best marker 선택
        int bestIdx      = -1;
        MarkerConfig bestCfg;
        double bestArea  = -1.0;
        int bestPriority = 99;

        for (size_t i = 0; i < ids.size(); i++) {
            if (ids[i] < targetMarkerMinId || ids[i] > targetMarkerMaxId) continue;
            MarkerConfig cfg;
            if (!getMarkerConfig(ids[i], cubeSize, cfg)) continue;
            const double area = std::abs(cv::contourArea(corners[i]));
            if (cfg.priority < bestPriority ||
               (cfg.priority == bestPriority && area > bestArea)) {
                bestPriority = cfg.priority;
                bestArea     = area;
                bestIdx      = (int)i;
                bestCfg      = cfg;
            }
        }

        if (bestIdx >= 0) {
            // ★ x, y: 마커 픽셀 중심 → H_img2world 직접 변환
            const auto& c = corners[bestIdx];
            cv::Point2f center(0.f, 0.f);
            for (auto& p : c) center += p;
            center *= 0.25f;

            cv::Mat pt_img   = (cv::Mat_<double>(3,1) << center.x, center.y, 1.0);
            cv::Mat pt_world = H * pt_img;
            const double x = pt_world.at<double>(0) / pt_world.at<double>(2);
            const double y = pt_world.at<double>(1) / pt_world.at<double>(2);

            // ★ yaw: ArUco pose → R_cam_cube → R_world_cube → forward 벡터
            cv::Mat R_cam_marker_m;
            cv::Rodrigues(rvecs[bestIdx], R_cam_marker_m);
            cv::Matx33d R_cam_marker  = toMatx33d(R_cam_marker_m);
            const cv::Matx33d R_m_c   = bestCfg.R_c_m.t();
            const cv::Matx33d R_cam_cube   = R_cam_marker * R_m_c;
            const cv::Matx33d R_world_cube = R_world_cam * R_cam_cube;

            const cv::Vec3d forward = R_world_cube * cv::Vec3d(0.0, 1.0, 0.0);
            double yaw = normalizeAngle(-std::atan2(forward[1], forward[0]));
            const long long ts = nowMs();

            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.6f,\"frame\":\"world\",\"ts_ms\":%lld}",
                          x, y, yaw, ts);

            if (mqttPublishJson(mosq, mqttTopic, buf))
                publishCount++;
        }

        const auto nowSteady = std::chrono::steady_clock::now();
        const auto elapsedMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(nowSteady - lastStatTs).count();
        if (elapsedMs >= 1000) {
            const double fps = (elapsedMs > 0) ? (frameCount * 1000.0 / elapsedMs) : 0.0;
            std::cout << "[STAT] fps=" << fps
                      << "  ids_seen=" << ids.size()
                      << "  best_id=" << (bestIdx >= 0 ? ids[bestIdx] : -1)
                      << "  pub_total=" << publishCount
                      << "  H_updates=" << shared.updateCount.load()
                      << "\n";
            frameCount = 0;
            lastStatTs = nowSteady;
        }
    }

    mosquitto_loop_stop(mosq, true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
