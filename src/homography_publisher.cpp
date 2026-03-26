#include "homography_publisher.hpp"

#include "server_utils.hpp"

#include <iostream>

namespace veda_server
{

    HomographyPublisher::HomographyPublisher(const ServerConfig &config,
                                             const CameraModel &camera_model,
                                             SharedHomography &shared)
        : config_(config),
          camera_model_(camera_model),
          shared_(shared) {}

    bool HomographyPublisher::RefreshAndPublish(mosquitto *mosq)
    {
        // yaml은 최초 1회만 읽음 — 이후엔 cached_H_ 재사용
        if (!h_loaded_)
        {
            if (!LoadHomography(config_.homography_yaml.string(), cached_H_))
            {
                std::cerr << "[ERR] HomographyPublisher: yaml load failed: "
                          << config_.homography_yaml.string() << "\n";
                return false;
            }

            cv::Matx33d R_world_cam = cv::Matx33d::eye();
            cv::Vec3d t_cam_world;
            EstimateWorldPoseFromHomography(
                cached_H_, camera_model_.K, R_world_cam, t_cam_world);

            {
                std::lock_guard<std::mutex> lk(shared_.mtx);
                shared_.H_img2world = cached_H_.clone();
                shared_.R_world_cam = R_world_cam;
                shared_.t_cam_world = t_cam_world;
                shared_.update_count.fetch_add(1);
            }

            h_loaded_ = true;
            std::cout << "[INFO] HomographyPublisher: H loaded from yaml (1회만 로드)\n";
        }

        // 이후엔 캐시된 H로 MQTT publish만 수행
        const std::string homography_payload =
            BuildHomographyPayload(cached_H_, config_.homography_yaml.string());

        if (homography_payload != last_homography_payload_)
        {
            if (PublishJson(mosq, config_.homography_topic, homography_payload, true))
            {
                last_homography_payload_ = homography_payload;
            }
        }

        const std::string map_payload = BuildMapPayload();
        if (map_payload != last_map_payload_)
        {
            if (PublishJson(mosq, config_.map_topic, map_payload, true))
            {
                last_map_payload_ = map_payload;
            }
        }

        return true;
    }

} // namespace veda_server