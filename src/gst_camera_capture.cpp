#include "gst_camera_capture.hpp"

#include <cstdio>
#include <string>

#include <gst/video/video.h>

#include <opencv2/opencv.hpp>

#include "timebase.hpp"

namespace {

std::string build_capture_pipeline(const CameraConfig& config) {
    return "libcamerasrc ! video/x-raw,format=RGBx,width=" + std::to_string(config.width) +
           ",height=" + std::to_string(config.height) +
           ",framerate=" + std::to_string(config.fps) + "/1 "
           "! videoconvert ! video/x-raw,format=BGR "
           "! appsink name=gstcam_sink drop=true max-buffers=1 sync=false emit-signals=false";
}

} // namespace

GstCameraCapture::~GstCameraCapture() {
    shutdown();
}

bool GstCameraCapture::init(const CameraConfig& config, std::string* error) {
    shutdown();

    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }

    config_ = config;
    frame_index_ = 0;
    pts_origin_ready_ = false;
    first_pts_ns_ = 0;
    pts_base_raw_ns_ = 0;

    GError* gst_error = nullptr;
    const std::string pipeline_desc = build_capture_pipeline(config_);
    pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &gst_error);
    if (!pipeline_) {
        if (error) {
            *error = gst_error ? gst_error->message : "failed to create GStreamer capture pipeline";
        }
        if (gst_error) {
            g_error_free(gst_error);
        }
        return false;
    }
    if (gst_error) {
        std::fprintf(stderr, "[GSTCAP] pipeline warning: %s\n", gst_error->message);
        g_error_free(gst_error);
    }

    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "gstcam_sink");
    if (!sink_) {
        if (error) {
            *error = "appsink not found in GStreamer capture pipeline";
        }
        shutdown();
        return false;
    }

    appsink_ = GST_APP_SINK(sink_);
    gst_app_sink_set_emit_signals(appsink_, FALSE);
    gst_app_sink_set_drop(appsink_, TRUE);
    gst_app_sink_set_max_buffers(appsink_, 1);
    g_object_set(G_OBJECT(appsink_),
                 "enable-last-sample", FALSE,
                 "wait-on-eos", FALSE,
                 "sync", FALSE,
                 nullptr);

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        if (error) {
            *error = "failed to start GStreamer capture pipeline";
        }
        shutdown();
        return false;
    }

    std::fprintf(stderr,
                 "[GSTCAP] pipeline ready %dx%d@%d\n",
                 config_.width,
                 config_.height,
                 config_.fps);
    started_ = true;
    return true;
}

void GstCameraCapture::shutdown() {
    started_ = false;

    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (sink_) {
        gst_object_unref(sink_);
        sink_ = nullptr;
        appsink_ = nullptr;
    }
    if (pipeline_) {
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}

bool GstCameraCapture::get_frame(CapturedFrame& out, std::string* error) {
    out = CapturedFrame{};
    if (!started_ || !appsink_) {
        if (error) {
            *error = "GStreamer capture is not started";
        }
        return false;
    }

    GstSample* sample = gst_app_sink_try_pull_sample(appsink_, 200000000);
    if (!sample) {
        if (error) {
            *error = "timed out waiting for GStreamer capture sample";
        }
        return false;
    }

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    if (!buffer || !caps) {
        gst_sample_unref(sample);
        if (error) {
            *error = "capture sample missing buffer or caps";
        }
        return false;
    }

    GstVideoInfo info;
    const bool info_ok = gst_video_info_from_caps(&info, caps);
    const int width = info_ok ? static_cast<int>(info.width) : config_.width;
    const int height = info_ok ? static_cast<int>(info.height) : config_.height;
    const int stride = info_ok && info.stride[0] > 0 ? static_cast<int>(info.stride[0]) : width * 3;

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        gst_sample_unref(sample);
        if (error) {
            *error = "failed to map capture sample buffer";
        }
        return false;
    }

    cv::Mat image(height, width, CV_8UC3, map.data, stride);
    out.image = image.clone();

    const GstClockTime pts = GST_BUFFER_PTS(buffer);
    if (pts != GST_CLOCK_TIME_NONE) {
        if (!pts_origin_ready_) {
            pts_origin_ready_ = true;
            first_pts_ns_ = static_cast<int64_t>(pts);
            pts_base_raw_ns_ = clock_ns(CLOCK_MONOTONIC_RAW);
        }
        const int64_t relative_pts_ns = static_cast<int64_t>(pts) - first_pts_ns_;
        out.frame_time_ms =
            static_cast<double>(pts_base_raw_ns_ + relative_pts_ns - monotonic_raw_origin_ns()) / 1e6;
        out.sensor_ts_ns = static_cast<int64_t>(pts);
    } else {
        out.frame_time_ms = now_ms();
        out.sensor_ts_ns = 0;
    }

    out.exposure_us = config_.exposure_us;
    out.frame_duration_us = config_.frame_duration_us;
    out.frame_index = frame_index_++;

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return !out.image.empty();
}
