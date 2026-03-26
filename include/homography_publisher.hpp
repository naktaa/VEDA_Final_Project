#pragma once

#include "server_types.hpp"
#include "server_utils.hpp"

#include <mosquitto.h>
#include <opencv2/opencv.hpp>

#include <string>

namespace veda_server
{

    class HomographyPublisher
    {
    public:
        explicit HomographyPublisher(const ServerConfig &config,
                                     const CameraModel &camera_model,
                                     SharedHomography &shared);

        bool RefreshAndPublish(mosquitto *mosq);

    private:
        const ServerConfig &config_;
        const CameraModel &camera_model_;
        SharedHomography &shared_;

        // yaml은 최초 1회만 읽고 이후엔 캐시 사용
        bool h_loaded_ = false;
        cv::Mat cached_H_;

        std::string last_homography_payload_;
        std::string last_map_payload_;
    };

} // namespace veda_server