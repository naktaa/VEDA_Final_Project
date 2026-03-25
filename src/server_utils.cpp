#include "server_utils.hpp"

#include <mosquitto.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <utility>
#include <vector>

namespace veda_server {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct WorldPoint {
    double x;
    double y;
};

} // namespace

bool LoadHomography(const std::string& path, cv::Mat& H_img2world) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ERR] homography yaml open failed: " << path << "\n";
        return false;
    }

    fs["H_img2world"] >> H_img2world;
    fs.release();

    if (H_img2world.empty() || H_img2world.rows != 3 || H_img2world.cols != 3) {
        std::cerr << "[ERR] H_img2world invalid in: " << path << "\n";
        return false;
    }
    if (H_img2world.type() != CV_64F) {
        H_img2world.convertTo(H_img2world, CV_64F);
    }
    return true;
}

bool LoadCameraModel(const std::string& path, CameraModel& out) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[ERR] camera yaml open failed: " << path << "\n";
        return false;
    }

    cv::Mat K;
    cv::Mat dist;
    if (!fs["camera_matrix"].isNone()) {
        fs["camera_matrix"] >> K;
    }
    if (!fs["K"].isNone()) {
        fs["K"] >> K;
    }
    if (!fs["dist_coeffs"].isNone()) {
        fs["dist_coeffs"] >> dist;
    }
    if (!fs["distortion_coefficients"].isNone()) {
        fs["distortion_coefficients"] >> dist;
    }
    fs.release();

    if (K.empty() || K.rows != 3 || K.cols != 3) {
        std::cerr << "[ERR] camera_matrix missing or invalid in: " << path << "\n";
        return false;
    }

    if (dist.empty()) {
        dist = cv::Mat::zeros(1, 5, CV_64F);
    }
    if (K.type() != CV_64F) {
        K.convertTo(K, CV_64F);
    }
    if (dist.type() != CV_64F) {
        dist.convertTo(dist, CV_64F);
    }

    out.K = K;
    out.dist = dist;
    return true;
}

bool EstimateWorldPoseFromHomography(const cv::Mat& H_img2world,
                                     const cv::Mat& K,
                                     cv::Matx33d& R_world_cam,
                                     cv::Vec3d& t_cam_world) {
    cv::Mat H_world2img = H_img2world.inv();
    cv::Mat K_inv = K.inv();

    cv::Mat h1 = H_world2img.col(0);
    cv::Mat h2 = H_world2img.col(1);
    cv::Mat h3 = H_world2img.col(2);

    cv::Mat r1m = K_inv * h1;
    cv::Mat r2m = K_inv * h2;
    cv::Mat tm = K_inv * h3;

    const double norm1 = cv::norm(r1m);
    const double norm2 = cv::norm(r2m);
    if (norm1 < 1e-9 || norm2 < 1e-9) {
        R_world_cam = cv::Matx33d::eye();
        t_cam_world = cv::Vec3d(0.0, 0.0, 0.0);
        return false;
    }

    const double scale = 2.0 / (norm1 + norm2);
    r1m *= scale;
    r2m *= scale;
    tm *= scale;

    cv::Vec3d r1(r1m.at<double>(0), r1m.at<double>(1), r1m.at<double>(2));
    cv::Vec3d r2(r2m.at<double>(0), r2m.at<double>(1), r2m.at<double>(2));
    cv::Vec3d r3 = r1.cross(r2);

    cv::Mat R_cam_world = (cv::Mat_<double>(3, 3) <<
        r1[0], r2[0], r3[0],
        r1[1], r2[1], r3[1],
        r1[2], r2[2], r3[2]);

    cv::SVD svd(R_cam_world);
    cv::Mat R_ortho = svd.u * svd.vt;
    if (cv::determinant(R_ortho) < 0.0) {
        R_ortho.col(2) *= -1.0;
    }

    cv::Matx33d Rcw(
        R_ortho.at<double>(0, 0), R_ortho.at<double>(0, 1), R_ortho.at<double>(0, 2),
        R_ortho.at<double>(1, 0), R_ortho.at<double>(1, 1), R_ortho.at<double>(1, 2),
        R_ortho.at<double>(2, 0), R_ortho.at<double>(2, 1), R_ortho.at<double>(2, 2));

    R_world_cam = Rcw.t();
    t_cam_world = cv::Vec3d(tm.at<double>(0), tm.at<double>(1), tm.at<double>(2));
    return true;
}

cv::Matx33d EstimateRworldCam(const cv::Mat& H_img2world, const cv::Mat& K) {
    cv::Matx33d R_world_cam = cv::Matx33d::eye();
    cv::Vec3d t_cam_world;
    EstimateWorldPoseFromHomography(H_img2world, K, R_world_cam, t_cam_world);
    return R_world_cam;
}

double NormalizeAngle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

long long NowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

bool OpenRtspCapture(const std::string& rtsp_url, cv::VideoCapture& cap) {
    const std::string gst =
        "rtspsrc location=" + rtsp_url +
        " protocols=tcp latency=50 drop-on-latency=true do-rtsp-keep-alive=true tcp-timeout=5000000 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! queue leaky=downstream max-size-buffers=1 ! "
        "appsink sync=false max-buffers=1 drop=true";

    if (cap.open(gst, cv::CAP_GSTREAMER)) {
        return true;
    }

    setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
           "rtsp_transport;tcp|fflags;nobuffer|max_delay;500000|stimeout;5000000", 1);
    if (cap.open(rtsp_url, cv::CAP_FFMPEG)) {
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
        return true;
    }

    return false;
}

bool GetMarkerConfig(int id, double cube_size, MarkerConfig& out) {
    const double h = cube_size * 0.5;
    switch (id) {
        case 21:
            out.R_c_m = cv::Matx33d(-1, 0, 0, 0, -1, 0, 0, 0, 1);
            out.t_c_m = cv::Vec3d(0.0, 0.0, h);
            out.priority = 0;
            return true;
        case 24:
            out.R_c_m = cv::Matx33d(1, 0, 0, 0, 0, 1, 0, -1, 0);
            out.t_c_m = cv::Vec3d(0.0, h, 0.0);
            out.priority = 3;
            return true;
        case 23:
            out.R_c_m = cv::Matx33d(0, 0, 1, -1, 0, 0, 0, -1, 0);
            out.t_c_m = cv::Vec3d(h, 0.0, 0.0);
            out.priority = 1;
            return true;
        case 25:
            out.R_c_m = cv::Matx33d(0, 0, -1, 1, 0, 0, 0, -1, 0);
            out.t_c_m = cv::Vec3d(-h, 0.0, 0.0);
            out.priority = 2;
            return true;
        case 22:
            out.R_c_m = cv::Matx33d(-1, 0, 0, 0, 0, -1, 0, -1, 0);
            out.t_c_m = cv::Vec3d(0.0, -h, 0.0);
            out.priority = 4;
            return true;
        default:
            return false;
    }
}

std::string JsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '\\':
                out += "\\\\";
                break;
            case '"':
                out += "\\\"";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += c;
                break;
        }
    }
    return out;
}

std::string BuildHomographyPayload(const cv::Mat& H, const std::string& yaml_path) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(16);
    oss << "{";
    oss << "\"type\":\"homography\",";
    oss << "\"key\":\"H_img2world\",";
    oss << "\"yaml\":\"" << JsonEscape(yaml_path) << "\",";
    oss << "\"rows\":3,\"cols\":3,";
    oss << "\"data\":[";
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (r != 0 || c != 0) {
                oss << ",";
            }
            oss << H.at<double>(r, c);
        }
    }
    oss << "],";
    oss << "\"ts_ms\":" << NowMs();
    oss << "}";
    return oss.str();
}

std::string BuildMapPayload() {
    const std::map<int, WorldPoint> id2world = {
        {10, {0.01, 3.30}},
        {11, {0.95, 3.30}},
        {12, {0.01, 0.01}},
        {13, {0.95, 0.01}},
    };
    const std::vector<std::pair<int, int>> edges = {
        {10, 11},
        {11, 13},
        {13, 12},
        {12, 10},
    };
    const std::vector<int> polyline = {10, 11, 13, 12, 10};

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "{";

    oss << "\"nodes\":[";
    bool first = true;
    for (const auto& [id, point] : id2world) {
        if (!first) {
            oss << ",";
        }
        first = false;
        oss << "{\"id\":\"" << id << "\",\"x\":" << point.x << ",\"y\":" << point.y << "}";
    }
    oss << "],";

    oss << "\"edges\":[";
    for (size_t i = 0; i < edges.size(); ++i) {
        if (i != 0) {
            oss << ",";
        }
        oss << "{\"from\":\"" << edges[i].first << "\",\"to\":\"" << edges[i].second << "\"}";
    }
    oss << "],";

    oss << "\"polyline\":[";
    for (size_t i = 0; i < polyline.size(); ++i) {
        if (i != 0) {
            oss << ",";
        }
        const auto it = id2world.find(polyline[i]);
        oss << "{\"x\":" << it->second.x << ",\"y\":" << it->second.y << "}";
    }
    oss << "]";

    oss << "}";
    return oss.str();
}

bool PublishJson(mosquitto* mosq,
                 const std::string& topic,
                 const std::string& payload,
                 bool retain) {
    const int rc = mosquitto_publish(
        mosq,
        nullptr,
        topic.c_str(),
        static_cast<int>(payload.size()),
        payload.c_str(),
        1,
        retain);

    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[WARN] mqtt publish failed topic=" << topic
                  << " rc=" << rc << " (" << mosquitto_strerror(rc) << ")\n";
        return false;
    }
    return true;
}

} // namespace veda_server
