#pragma once

#include <mutex>
#include <string>
#include <thread>

#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>

#include <opencv2/opencv.hpp>

#include "app_config.hpp"
#include "app_types.hpp"

class RtspServer {
public:
    struct StreamSlot {
        std::mutex mutex;
        GstAppSrc* appsrc = nullptr;
        bool has_origin = false;
        double origin_frame_time_ms = 0.0;
    };

    RtspServer() = default;
    ~RtspServer();

    bool start(const CameraConfig& camera_config,
               const RtspConfig& rtsp_config,
               bool enable_raw_stream = false,
               std::string* error = nullptr);
    void stop();

    bool push_stabilized(const CapturedFrame& frame, const cv::Mat& image);
    bool push_raw(const CapturedFrame& frame, const cv::Mat& image);

private:
    static void on_media_configure(GstRTSPMediaFactory* factory, GstRTSPMedia* media, gpointer user_data);
    static void set_appsrc_caps(GstAppSrc* appsrc, int width, int height, int fps);
    static bool push_frame(StreamSlot& slot,
                           const CapturedFrame& frame,
                           const cv::Mat& image,
                           int width,
                           int height,
                           int fps);

    CameraConfig camera_config_;
    RtspConfig rtsp_config_;

    GMainLoop* main_loop_ = nullptr;
    GstRTSPServer* server_ = nullptr;
    std::thread loop_thread_;
    bool started_ = false;

    StreamSlot raw_slot_;
    StreamSlot stabilized_slot_;
};
