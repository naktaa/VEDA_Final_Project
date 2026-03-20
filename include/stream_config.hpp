#pragma once

#include <string>

namespace stream_config {

    inline constexpr const char* DEFAULT_MQTT_HOST = "192.168.100.10";
    inline constexpr int DEFAULT_MQTT_PORT = 1883;
    inline constexpr int DEFAULT_MQTT_KEEPALIVE_SEC = 30;
    inline constexpr const char* DEFAULT_MQTT_TOPIC = "wiserisk/rc/control";

    inline constexpr const char* DEFAULT_RTSP_PORT = "8555";
    inline constexpr const char* DEFAULT_RTSP_PATH = "/cam";
    inline constexpr const char* RTSP_APPSRC_NAME = "stabsrc";
    inline constexpr const char* CAPTURE_APPSINK_NAME = "appsink";
    inline constexpr int DEFAULT_WIDTH = 640;
    inline constexpr int DEFAULT_HEIGHT = 480;
    inline constexpr int DEFAULT_FPS = 20;
    inline constexpr int DEFAULT_BITRATE = 1500000;
    inline constexpr int DEFAULT_IFRAME_PERIOD = 5;
    inline constexpr bool DEFAULT_FLIP_VERTICAL = true;
    inline constexpr bool DEFAULT_FLIP_HORIZONTAL = true;

    // NV12 frame size: Y plane (w*h) + UV interleaved plane (w*h/2)
    inline constexpr std::size_t nv12_frame_bytes(int w, int h) {
        return (std::size_t)w * (std::size_t)h * 3U / 2U;
    }

    inline std::string make_default_rtsp_launch() {
        std::string launch =
            "( appsrc name=" + std::string(RTSP_APPSRC_NAME) +
            " is-live=true format=time do-timestamp=false block=false "
            "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ";

        if (DEFAULT_FLIP_VERTICAL && DEFAULT_FLIP_HORIZONTAL) {
            launch += "! videoflip method=rotate-180 ";
        }
        else if (DEFAULT_FLIP_VERTICAL) {
            launch += "! videoflip method=vertical-flip ";
        }
        else if (DEFAULT_FLIP_HORIZONTAL) {
            launch += "! videoflip method=horizontal-flip ";
        }

        launch +=
            "! v4l2h264enc extra-controls=\"controls,video_bitrate=" +
            std::to_string(DEFAULT_BITRATE) +
            ",h264_i_frame_period=" + std::to_string(DEFAULT_IFRAME_PERIOD) + "\" "
                                                                              "! video/x-h264,level=(string)4,profile=(string)main "
                                                                              "! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
                                                                              "! rtph264pay name=pay0 pt=96 config-interval=1 )";
        return launch;
    }

    inline std::string make_default_capture_launch() {
        return "libcamerasrc "
               "! video/x-raw,format=NV12,width=" +
               std::to_string(DEFAULT_WIDTH) +
               ",height=" + std::to_string(DEFAULT_HEIGHT) +
               ",framerate=" + std::to_string(DEFAULT_FPS) + "/1 "
                                                             "! appsink name=" +
               std::string(CAPTURE_APPSINK_NAME) +
               " drop=true max-buffers=1 sync=false";
    }

} // namespace stream_config
