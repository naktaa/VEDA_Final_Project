#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

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
    cv::Point2f dir = markerCorners[1] - markerCorners[0];
    const float norm = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (norm < 1e-6f) return 0.0;
    dir *= (1.0f / norm);

    const cv::Point2f headPx = centerPx + dir * 30.0f;
    const cv::Point2f w0 = applyHomography(H_img2world, centerPx);
    const cv::Point2f w1 = applyHomography(H_img2world, headPx);
    return std::atan2((double)(w1.y - w0.y), (double)(w1.x - w0.x));
}

int main()
{
    const std::string rtspUrl =
        "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
    const std::string homographyYaml = "H_img2world.yaml";
    const int targetMarkerId = 0;

    cv::Mat H_img2world;
    if (!loadHomography(homographyYaml, H_img2world)) {
        std::cerr << "[ERR] failed to load homography: " << homographyYaml << "\n";
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

    std::cout << "[OK] reading ArUco id=0 yaw\n";
    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "[WARN] empty frame\n";
            continue;
        }

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        detector.detectMarkers(frame, corners, ids);

        bool found = false;
        double yaw = 0.0;
        for (size_t i = 0; i < ids.size(); ++i) {
            if (ids[i] != targetMarkerId) continue;

            cv::Point2f c(0, 0);
            for (const auto& p : corners[i]) c += p;
            c *= 0.25f;

            yaw = calcYawWorld(H_img2world, c, corners[i]);
            found = true;
            break;
        }

        if (found) {
            std::cout << std::fixed << std::setprecision(6)
                      << "[ARUCO_YAW] id=0 yaw=" << yaw << "\n";
        } else {
            std::cout << "[ARUCO_YAW] id=0 not found\n";
        }
    }
}

