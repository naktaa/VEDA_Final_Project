#pragma once

#include <string>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include "app_config.hpp"
#include "app_types.hpp"

class GstCameraCapture {
public:
    GstCameraCapture() = default;
    ~GstCameraCapture();

    bool init(const CameraConfig& config, std::string* error = nullptr);
    void shutdown();
    bool get_frame(CapturedFrame& out, std::string* error = nullptr);

private:
    CameraConfig config_;
    GstElement* pipeline_ = nullptr;
    GstElement* sink_ = nullptr;
    GstAppSink* appsink_ = nullptr;
    bool started_ = false;
    bool pts_origin_ready_ = false;
    uint64_t frame_index_ = 0;
    int64_t first_pts_ns_ = 0;
    int64_t pts_base_raw_ns_ = 0;
};
