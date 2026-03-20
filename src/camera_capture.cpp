#include "camera_capture.hpp"

#include <cstring>
#include <cstdio>
#include <cstdint>
#include <thread>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include "rtsp_stream.hpp"
#include "stream_config.hpp"

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
        const std::size_t expected_bytes =
            stream_config::nv12_frame_bytes(stream_config::DEFAULT_WIDTH,
                                            stream_config::DEFAULT_HEIGHT);

        while (impl->running && *(impl->app_running)) {
            GstSample* sample = gst_app_sink_try_pull_sample(impl->appsink, 100000000);
            if (!sample) continue;

            GstBuffer* buffer = gst_sample_get_buffer(sample);
            if (!buffer) {
                gst_sample_unref(sample);
                continue;
            }

            // Preserve original camera PTS for encoder rate-control
            // and future gyro<->frame timestamp synchronization.
            const uint64_t pts = GST_BUFFER_PTS(buffer);
            const uint64_t duration = GST_BUFFER_DURATION(buffer);

            GstMapInfo map;
            if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                gst_sample_unref(sample);
                continue;
            }

            // NV12 is contiguous (Y plane + UV interleaved), no stride
            // repacking needed when the buffer matches expected size.
            const std::size_t frame_bytes =
                (map.size <= expected_bytes) ? map.size : expected_bytes;

            impl->rtsp_server->push_frame(map.data, frame_bytes, pts, duration);

            gst_buffer_unmap(buffer, &map);
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
