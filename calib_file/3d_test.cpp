/*
컴파일 : g++ -std=c++17 3d_test.cpp -o 3d_test $(pkg-config --cflags --libs opencv4)

실행 : ./3d_test "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp" config/H_img2world.yaml config/camera.yaml 0.17 0.17

*/

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

struct CameraModel {
    cv::Mat K;
    cv::Mat dist;
};

struct MarkerConfig {
    cv::Matx33d R_c_m;
    cv::Vec3d t_c_m;
    int priority = 9;
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

static cv::Matx33d estimateRworldCam(const cv::Mat& H_img2world, const cv::Mat& K)
{
    cv::Mat H_world2img = H_img2world.inv();
    cv::Mat K_inv = K.inv();

    cv::Mat h1 = H_world2img.col(0);
    cv::Mat h2 = H_world2img.col(1);

    cv::Mat r1m = K_inv * h1;
    cv::Mat r2m = K_inv * h2;

    const double norm1 = cv::norm(r1m);
    if (norm1 < 1e-9) return cv::Matx33d::eye();

    r1m /= norm1;
    r2m /= norm1;

    cv::Vec3d r1(r1m.at<double>(0), r1m.at<double>(1), r1m.at<double>(2));
    cv::Vec3d r2(r2m.at<double>(0), r2m.at<double>(1), r2m.at<double>(2));
    cv::Vec3d r3 = r1.cross(r2);

    cv::Mat R_cam_world = (cv::Mat_<double>(3, 3) <<
        r1[0], r2[0], r3[0],
        r1[1], r2[1], r3[1],
        r1[2], r2[2], r3[2]);

    cv::SVD svd(R_cam_world);
    cv::Mat R_ortho = svd.u * svd.vt;
    if (cv::determinant(R_ortho) < 0) R_ortho *= -1.0;

    cv::Matx33d Rcw(
        R_ortho.at<double>(0, 0), R_ortho.at<double>(0, 1), R_ortho.at<double>(0, 2),
        R_ortho.at<double>(1, 0), R_ortho.at<double>(1, 1), R_ortho.at<double>(1, 2),
        R_ortho.at<double>(2, 0), R_ortho.at<double>(2, 1), R_ortho.at<double>(2, 2));

    return Rcw.t();
}

static bool getMarkerConfig(int id, double cubeSize, MarkerConfig& out)
{
    const double h = cubeSize * 0.5;
    switch (id) {
        case 21:
            out.R_c_m = cv::Matx33d(-1, 0, 0, 0, -1, 0, 0, 0, 1);
            out.t_c_m = cv::Vec3d(0.0, 0.0, h);
            out.priority = 0;
            return true;
        case 24:
            out.R_c_m = cv::Matx33d(0, 1, 0, 0, 0, 1, 1, 0, 0);
            out.t_c_m = cv::Vec3d(0.0, h, 0.0);
            out.priority = 1;
            return true;
        case 23:
            out.R_c_m = cv::Matx33d(0, 0, 1, -1, 0, 0, 0, -1, 0);
            out.t_c_m = cv::Vec3d(h, 0.0, 0.0);
            out.priority = 2;
            return true;
        case 25:
            out.R_c_m = cv::Matx33d(0, 0, -1, 1, 0, 0, 0, -1, 0);
            out.t_c_m = cv::Vec3d(-h, 0.0, 0.0);
            out.priority = 3;
            return true;
        case 22:
            out.R_c_m = cv::Matx33d(0, -1, 0, 0, 0, -1, 1, 0, 0);
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
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
}

static double normalizeAngle(double a)
{
    constexpr double PI = 3.14159265358979323846;
    while (a > PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
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

int main(int argc, char** argv)
{
    const std::string rtspUrl = (argc > 1)
        ? argv[1]
        : "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
    const std::string homographyYaml = (argc > 2)
        ? argv[2]
        : "config/H_img2world.yaml";
    const std::string cameraYaml = (argc > 3)
        ? argv[3]
        : "config/camera.yaml";
    const double markerSize = (argc > 4) ? std::stod(argv[4]) : 0.17;
    const double cubeSize = (argc > 5) ? std::stod(argv[5]) : 0.17;

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

    const cv::Matx33d R_world_cam = estimateRworldCam(H_img2world, cam.K);

    cv::VideoCapture cap;
    if (!openRtspCapture(rtspUrl, cap)) {
        std::cerr << "[ERR] RTSP open failed\n";
        return 3;
    }

    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters params;
    cv::aruco::ArucoDetector detector(dict, params);

    std::cout << "[OK] yaw test started\n";
    std::cout << "[INFO] Press Ctrl+C to stop\n";

    auto lastPrintTs = std::chrono::steady_clock::now();

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "[WARN] frame read failed\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        detector.detectMarkers(frame, corners, ids);

        std::vector<cv::Vec3d> rvecs, tvecs;
        if (!ids.empty()) {
            cv::aruco::estimatePoseSingleMarkers(corners, markerSize, cam.K, cam.dist, rvecs, tvecs);
        }

        std::map<int, double> yawById;
        for (size_t i = 0; i < ids.size(); ++i) {
            const int id = ids[i];
            if (id < 21 || id > 25) continue;

            MarkerConfig cfg;
            if (!getMarkerConfig(id, cubeSize, cfg)) continue;

            cv::Mat R_cam_marker_m;
            cv::Rodrigues(rvecs[i], R_cam_marker_m);
            const cv::Matx33d R_cam_marker = toMatx33d(R_cam_marker_m);
            const cv::Matx33d R_m_c = cfg.R_c_m.t();
            const cv::Matx33d R_cam_cube = R_cam_marker * R_m_c;
            const cv::Matx33d R_world_cube = R_world_cam * R_cam_cube;
            const cv::Vec3d forward = R_world_cube * cv::Vec3d(0.0, 1.0, 0.0);
            double yaw = normalizeAngle(-std::atan2(forward[1], forward[0]));
            if (id == 22 || id == 24) {
                yaw = normalizeAngle(yaw + CV_PI);
            }
            yawById[id] = yaw;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPrintTs).count();
        if (elapsedMs >= 500) {
            std::cout << "[YAW]";
            for (int id = 21; id <= 25; ++id) {
                const auto it = yawById.find(id);
                if (it != yawById.end()) {
                    std::cout << " id=" << id << ":" << it->second;
                } else {
                    std::cout << " id=" << id << ":---";
                }
            }
            std::cout << "\n";
            lastPrintTs = now;
        }
    }

    return 0;
}
