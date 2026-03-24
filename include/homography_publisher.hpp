#pragma once

#include <string>

#include "server_types.hpp"

struct mosquitto;

namespace veda_server {

class HomographyPublisher {
public:
    HomographyPublisher(const ServerConfig& config,
                        const CameraModel& camera_model,
                        SharedHomography& shared);

    bool RefreshAndPublish(mosquitto* mosq);

private:
    const ServerConfig& config_;
    const CameraModel& camera_model_;
    SharedHomography& shared_;
    std::string last_homography_payload_;
    std::string last_map_payload_;
};

} // namespace veda_server
