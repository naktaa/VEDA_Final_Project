#include "camera_capture.hpp"

#include <cstring>
#include <cstdio>
#include <cstdint>
#include <thread>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include "rtsp_stream.hpp"
#include "stream_config.hpp"

namespace {

bool copy_nv12_frame(const GstVideoFrame& frame, int width, int height,
                     std::vector<unsigned char>& packed) {
    if (GST_VIDEO_FRAME_N_PLANES(&frame) < 2) return false;

    const std::size_t expected_bytes = stream_config::nv12_frame_bytes(width, height);
    packed.resize(expected_bytes);

    const unsigned char* y_src = GST_VIDEO_FRAME_PLANE_DATA(&frame, 0);
    const unsigned char* uv_src = GST_VIDEO_FRAME_PLANE_DATA(&frame, 1);
    const int y_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    const int uv_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 1);
    if (!y_src || !uv_src || y_stride < width || uv_stride < width) return false;

    unsigned char* y_dst = packed.data();
    unsigned char* uv_dst = packed.data() + (std::size_t)width * (std::size_t)height;

    for (int y = 0; y < height; ++y) {
        std::memcpy(y_dst + (std::size_t)y * (std::size_t)width,
                    y_src + (std::size_t)y * (std::size_t)y_stride,
                    (std::size_t)width);
    }

    const int chroma_rows = height / 2;
    for (int y = 0; y < chroma_rows; ++y) {
        std::memcpy(uv_dst + (std::size_t)y * (std::size_t)width,
                    uv_src + (std::size_t)y * (std::size_t)uv_stride,
                    (std::size_t)width);
    }

    return true;
}

GstClockTime normalize_pts(GstClockTime source_pts,
                           GstClockTime duration,
                           bool& have_pts_base,
                           GstClockTime& pts_base,
                           GstClockTime& next_pts) {
    GstClockTime pts = next_pts;

    if (GST_CLOCK_TIME_IS_VALID(source_pts)) {
        if (!have_pts_base) {
            have_pts_base = true;
            pts_base = source_pts;
            pts = 0;
        } else if (source_pts >= pts_base) {
            pts = source_pts - pts_base;
        }
    }

    if (pts < next_pts) {
        pts = next_pts;
    }

    next_pts = pts + duration;
    return pts;
}

} // namespace

struct CameraCapture::Impl {
    std::atomic<bool>* app_running = nullptr;
    std::atomic<bool> running{false};
    RtspStreamServer* rtsp_server = nullptr;

    GstElement* pipeline = nullptr;
    GstElement* sink = nullptr;
    GstAppSink* appsink = nullptr;
    std::thread worker;
};

CameraCapture::~CameraCapture() {
    stop();
}

bool CameraCapture::start(std::atomic<bool>& app_running, RtspStreamServer& rtsp_server) {
    if (impl_) return true;

    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }

    auto* impl = new Impl();
    impl->app_running = &app_running;
    impl->rtsp_server = &rtsp_server;

    GError* err = nullptr;
    const std::string cap_pipe = stream_config::make_default_capture_launch();
    impl->pipeline = gst_parse_launch(cap_pipe.c_str(), &err);
    if (!impl->pipeline) {
        fprintf(stderr, "[CAP] capture pipeline create failed\n");
        if (err) g_error_free(err);
        delete impl;
        return false;
    }
    if (err) {
        fprintf(stderr, "[CAP] warning: %s\n", err->message);
        g_error_free(err);
    }

    impl->sink = gst_bin_get_by_name(GST_BIN(impl->pipeline), stream_config::CAPTURE_APPSINK_NAME);
    if (!impl->sink) {
        fprintf(stderr, "[CAP] appsink not found\n");
        gst_object_unref(impl->pipeline);
        delete impl;
        return false;
    }

    impl->appsink = GST_APP_SINK(impl->sink);
    gst_app_sink_set_emit_signals(impl->appsink, FALSE);
    gst_app_sink_set_drop(impl->appsink, TRUE);
    gst_app_sink_set_max_buffers(impl->appsink, 1);

    if (gst_element_set_state(impl->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "[CAP] capture pipeline start failed\n");
        gst_object_unref(impl->sink);
        gst_object_unref(impl->pipeline);
        delete impl;
        return false;
    }

    impl->running = true;
    impl->worker = std::thread([impl]() {
        const GstClockTime default_duration =
            gst_util_uint64_scale_int(1, GST_SECOND, stream_config::DEFAULT_FPS);
        std::vector<unsigned char> packed;
        bool have_pts_base = false;
        GstClockTime pts_base = GST_CLOCK_TIME_NONE;
        GstClockTime next_pts = 0;

        while (impl->running && *(impl->app_running)) {
            GstSample* sample = gst_app_sink_try_pull_sample(impl->appsink, 100000000);
            if (!sample) continue;

            GstBuffer* buffer = gst_sample_get_buffer(sample);
            GstCaps* caps = gst_sample_get_caps(sample);
            if (!buffer || !caps) {
                gst_sample_unref(sample);
                continue;
            }

            GstVideoInfo info;
            if (!gst_video_info_from_caps(&info, caps) ||
                info.finfo->format != GST_VIDEO_FORMAT_NV12) {
                gst_sample_unref(sample);
                continue;
            }

            if ((int)info.width != stream_config::DEFAULT_WIDTH ||
                (int)info.height != stream_config::DEFAULT_HEIGHT) {
                gst_sample_unref(sample);
                continue;
            }

            GstVideoFrame frame;
            if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) {
                gst_sample_unref(sample);
                continue;
            }

            const GstClockTime source_duration = GST_BUFFER_DURATION(buffer);
            const GstClockTime duration =
                GST_CLOCK_TIME_IS_VALID(source_duration) && source_duration > 0
                    ? source_duration
                    : default_duration;
            const GstClockTime stream_pts =
                normalize_pts(GST_BUFFER_PTS(buffer), duration, have_pts_base, pts_base, next_pts);

            if (copy_nv12_frame(frame, (int)info.width, (int)info.height, packed)) {
                impl->rtsp_server->push_frame(packed.data(), packed.size(), stream_pts, duration);
            }

            gst_video_frame_unmap(&frame);
            gst_sample_unref(sample);
        }
    });

    impl_ = impl;
    fprintf(stderr, "[CAP] capture thread started (NV12 appsink -> appsrc)\n");
    return true;
}

void CameraCapture::stop() {
    if (!impl_) return;

    impl_->running = false;
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }

    if (impl_->pipeline) {
        gst_element_set_state(impl_->pipeline, GST_STATE_NULL);
    }
    if (impl_->sink) {
        gst_object_unref(impl_->sink);
        impl_->sink = nullptr;
    }
    if (impl_->pipeline) {
        gst_object_unref(impl_->pipeline);
        impl_->pipeline = nullptr;
    }

    delete impl_;
    impl_ = nullptr;
}
