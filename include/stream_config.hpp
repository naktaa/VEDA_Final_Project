#pragma once

#include <string>

namespace stream_config {

    inline constexpr const char* DEFAULT_MQTT_HOST = "192.168.100.10";
    inline constexpr int DEFAULT_MQTT_PORT = 1883;
    inline constexpr int DEFAULT_MQTT_KEEPALIVE_SEC = 30;
    inline constexpr const char* DEFAULT_MQTT_TOPIC = "wiserisk/rc/control";

    inline constexpr const char* DEFAULT_RTSP_PORT = "8555";
    inline constexpr const char* DEFAULT_RTSP_PATH = "/cam";
    inline constexpr int DEFAULT_WIDTH = 640;
    inline constexpr int DEFAULT_HEIGHT = 480;
    inline constexpr int DEFAULT_FPS = 20;
    inline constexpr int DEFAULT_BITRATE = 1500000;
    inline constexpr int DEFAULT_IFRAME_PERIOD = 30;
    inline constexpr bool DEFAULT_FLIP_VERTICAL = true;
    inline constexpr bool DEFAULT_FLIP_HORIZONTAL = true;

    inline std::string make_default_rtsp_launch() {
        std::string launch =
            "( libcamerasrc ! video/x-raw,width=" + std::to_string(DEFAULT_WIDTH) +
            ",height=" + std::to_string(DEFAULT_HEIGHT) +
            ",framerate=" + std::to_string(DEFAULT_FPS) + "/1 ";

        if (DEFAULT_FLIP_VERTICAL) {
            launch += "! videoflip method=vertical-flip ";
        }
        if (DEFAULT_FLIP_HORIZONTAL) {
            launch += "! videoflip method=horizontal-flip ";
        }

        launch +=
            "! videoconvert "
            "! video/x-raw,format=I420 "
            "! v4l2h264enc extra-controls=\"controls,video_bitrate=" +
            std::to_string(DEFAULT_BITRATE) +
            ",h264_i_frame_period=" + std::to_string(DEFAULT_IFRAME_PERIOD) + "\" "
                                                                              "! video/x-h264,level=(string)4,profile=(string)baseline "
                                                                              "! rtph264pay name=pay0 pt=96 config-interval=1 )";
        return launch;
    }

} // namespace stream_config
