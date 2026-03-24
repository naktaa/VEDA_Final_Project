#include "camera_capture.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include <opencv2/opencv.hpp>

#include "frame_jpeg_cache.hpp"
#include "rtsp_stream.hpp"
#include "stream_config.hpp"

struct CameraCapture::Impl {
    std::atomic<bool>* app_running = nullptr;
    std::atomic<bool> running{false};
    RtspStreamServer* rtsp_server = nullptr;
    FrameJpegCache* frame_cache = nullptr;

    GstElement* pipeline = nullptr;
    GstElement* sink = nullptr;
    GstAppSink* appsink = nullptr;
    std::thread worker;
    bool use_synthetic = false;

    void cleanup_pipeline() {
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
        }
        if (sink) {
            gst_object_unref(sink);
            sink = nullptr;
            appsink = nullptr;
        }
        if (pipeline) {
            gst_object_unref(pipeline);
            pipeline = nullptr;
        }
    }

    bool init_capture_pipeline() {
        GError* err = nullptr;
        const std::string cap_pipe = stream_config::make_default_capture_launch();
        pipeline = gst_parse_launch(cap_pipe.c_str(), &err);
        if (!pipeline) {
            std::fprintf(stderr, "[CAP] capture pipeline create failed\n");
            if (err) {
                g_error_free(err);
            }
            return false;
        }
        if (err) {
            std::fprintf(stderr, "[CAP] warning: %s\n", err->message);
            g_error_free(err);
        }

        sink = gst_bin_get_by_name(GST_BIN(pipeline), stream_config::CAPTURE_APPSINK_NAME);
        if (!sink) {
            std::fprintf(stderr, "[CAP] appsink not found\n");
            cleanup_pipeline();
            return false;
        }

        appsink = GST_APP_SINK(sink);
        gst_app_sink_set_emit_signals(appsink, FALSE);
        gst_app_sink_set_drop(appsink, TRUE);
        gst_app_sink_set_max_buffers(appsink, 1);

        if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            std::fprintf(stderr, "[CAP] capture pipeline start failed\n");
            cleanup_pipeline();
            return false;
        }
        return true;
    }

    void publish_frame(const unsigned char* data, std::size_t bytes, int width, int height) {
        if (rtsp_server) {
            rtsp_server->push_bgr_frame(data, bytes);
        }
        if (frame_cache && frame_cache->has_consumers()) {
            frame_cache->update_bgr_frame(data, bytes, width, height);
        }
    }

    static cv::Mat make_test_pattern(int width, int height, int frame_index) {
        cv::Mat img(height, width, CV_8UC3, cv::Scalar(25, 25, 25));
        const int band_width = 120;
        const int x = (frame_index * 8) % std::max(1, width - band_width);

        cv::rectangle(img,
                      cv::Rect(x, 30, band_width, 80),
                      cv::Scalar(0, 210, 255),
                      cv::FILLED);
        cv::putText(img,
                    "NO CAMERA - TEST PATTERN",
                    cv::Point(24, height - 30),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(255, 255, 255),
                    2);
        return img;
    }
};

CameraCapture::~CameraCapture() {
    stop();
}

bool CameraCapture::start(std::atomic<bool>& app_running,
                          RtspStreamServer* rtsp_server,
                          FrameJpegCache* frame_cache) {
    if (impl_) return true;
    if (!rtsp_server && !frame_cache) return true;

    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }

    auto* impl = new Impl();
    impl->app_running = &app_running;
    impl->rtsp_server = rtsp_server;
    impl->frame_cache = frame_cache;

    if (!impl->init_capture_pipeline()) {
        impl->use_synthetic = true;
        std::fprintf(stderr, "[CAP] using synthetic fallback frames\n");
    }

    impl->running = true;
    impl->worker = std::thread([impl]() {
        std::vector<unsigned char> packed;
        int pull_failures = 0;
        int synthetic_frame = 0;

        while (impl->running && *(impl->app_running)) {
            if (impl->use_synthetic) {
                cv::Mat fallback = Impl::make_test_pattern(stream_config::DEFAULT_WIDTH,
                                                           stream_config::DEFAULT_HEIGHT,
                                                           synthetic_frame++);
                impl->publish_frame(fallback.data,
                                    static_cast<std::size_t>(fallback.total() * fallback.elemSize()),
                                    fallback.cols,
                                    fallback.rows);
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(1000 / std::max(1, stream_config::DEFAULT_FPS)));
                continue;
            }

            GstSample* sample = gst_app_sink_try_pull_sample(impl->appsink, 100000000);
            if (!sample) {
                ++pull_failures;
                if (pull_failures > 30) {
                    std::fprintf(stderr,
                                 "[CAP] frame pull failed repeatedly; switching to synthetic fallback\n");
                    impl->cleanup_pipeline();
                    impl->use_synthetic = true;
                }
                continue;
            }
            pull_failures = 0;

            GstBuffer* buffer = gst_sample_get_buffer(sample);
            GstCaps* caps = gst_sample_get_caps(sample);
            if (!buffer || !caps) {
                gst_sample_unref(sample);
                continue;
            }

            GstVideoInfo info;
            const bool info_ok = gst_video_info_from_caps(&info, caps);
            const int w = info_ok ? (int)info.width : stream_config::DEFAULT_WIDTH;
            const int h = info_ok ? (int)info.height : stream_config::DEFAULT_HEIGHT;
            const int stride = info_ok ? (int)info.stride[0] : (w * 3);

            GstMapInfo map;
            if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                gst_sample_unref(sample);
                continue;
            }

            const std::size_t row_bytes = (std::size_t)w * 3U;
            const std::size_t total_bytes = row_bytes * (std::size_t)h;
            const unsigned char* src_ptr = map.data;

            if ((std::size_t)stride != row_bytes) {
                packed.resize(total_bytes);
                for (int y = 0; y < h; ++y) {
                    const unsigned char* row_src = map.data + (std::size_t)y * (std::size_t)stride;
                    unsigned char* row_dst = packed.data() + (std::size_t)y * row_bytes;
                    std::memcpy(row_dst, row_src, row_bytes);
                }
                src_ptr = packed.data();
            }

            impl->publish_frame(src_ptr, total_bytes, w, h);

            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
        }
    });

    impl_ = impl;
    std::fprintf(stderr, "[CAP] capture thread started\n");
    return true;
}

void CameraCapture::stop() {
    if (!impl_) return;

    impl_->running = false;
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }

    impl_->cleanup_pipeline();

    delete impl_;
    impl_ = nullptr;
}
