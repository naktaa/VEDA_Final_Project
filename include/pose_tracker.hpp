#pragma once

#include <cstddef>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "server_types.hpp"

struct mosquitto;

namespace veda_server {

class PoseTracker {
public:
    PoseTracker(const ServerConfig& config,
                const CameraModel& camera_model,
                SharedHomography& shared);

    bool ProcessFrame(const cv::Mat& frame, mosquitto* mosq, long long& publish_count);
    size_t last_ids_seen() const;
    int last_best_id() const;

private:
    const ServerConfig& config_;
    const CameraModel& camera_model_;
    SharedHomography& shared_;
    cv::aruco::ArucoDetector detector_;
    size_t last_ids_seen_ = 0;
    int last_best_id_ = -1;
};

} // namespace veda_server
