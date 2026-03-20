#include "rtsp_stream.hpp"

#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

#include "stream_config.hpp"

struct RtspStreamServerImpl {
    GMainLoop* loop = nullptr;
    GstRTSPServer* server = nullptr;
    std::thread loop_thread;
    bool started = false;

    std::mutex appsrc_mtx;
    GstAppSrc* cam_appsrc = nullptr;
};

namespace {

void set_appsrc_caps(GstAppSrc* appsrc) {
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
                                        "format", G_TYPE_STRING, "NV12",
                                        "width", G_TYPE_INT, stream_config::DEFAULT_WIDTH,
                                        "height", G_TYPE_INT, stream_config::DEFAULT_HEIGHT,
                                        "framerate", GST_TYPE_FRACTION, stream_config::DEFAULT_FPS, 1, nullptr);
    gst_app_src_set_caps(appsrc, caps);
    gst_caps_unref(caps);
    g_object_set(G_OBJECT(appsrc),
                 "is-live", TRUE,
                 "format", GST_FORMAT_TIME,
                 "do-timestamp", FALSE,
                 "block", FALSE,
                 nullptr);
}

void on_media_configure(GstRTSPMediaFactory*, GstRTSPMedia* media, gpointer user_data) {
    auto* impl = static_cast<RtspStreamServerImpl*>(user_data);
    if (!impl) return;

    GstElement* element = gst_rtsp_media_get_element(media);
    GstElement* app = gst_bin_get_by_name_recurse_up(
        GST_BIN(element), stream_config::RTSP_APPSRC_NAME);
    gst_object_unref(element);
    if (!app) {
        g_printerr("[RTSP] Failed to get appsrc: %s\n", stream_config::RTSP_APPSRC_NAME);
        return;
    }

    GstAppSrc* appsrc = GST_APP_SRC(app);
    set_appsrc_caps(appsrc);

    g_object_ref(appsrc);
    {
        std::lock_guard<std::mutex> lk(impl->appsrc_mtx);
        if (impl->cam_appsrc) {
            g_object_unref(impl->cam_appsrc);
        }
        impl->cam_appsrc = appsrc;
    }

    gst_object_unref(app);
    g_print("[RTSP] /cam connected\n");
}

} // namespace

RtspStreamServer::~RtspStreamServer() {
    stop();
}

bool RtspStreamServer::start(const RtspConfig& cfg) {
    if (!cfg.enable) return true;
    if (impl_ && impl_->started) return true;

    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }

    auto* impl = new RtspStreamServerImpl();
    impl->server = gst_rtsp_server_new();
    if (!impl->server) {
        fprintf(stderr, "[RTSP] server create failed\n");
        delete impl;
        return false;
    }

    gst_rtsp_server_set_service(impl->server, cfg.service.c_str());

    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(impl->server);
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, cfg.launch.c_str());
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    gst_rtsp_media_factory_set_suspend_mode(factory, GST_RTSP_SUSPEND_MODE_NONE);
    g_signal_connect(factory, "media-configure", (GCallback)on_media_configure, impl);
    gst_rtsp_mount_points_add_factory(mounts, cfg.path.c_str(), factory);
    g_object_unref(mounts);

    if (gst_rtsp_server_attach(impl->server, nullptr) == 0) {
        fprintf(stderr, "[RTSP] attach failed\n");
        g_object_unref(impl->server);
        delete impl;
        return false;
    }

    impl->loop = g_main_loop_new(nullptr, FALSE);
    if (!impl->loop) {
        fprintf(stderr, "[RTSP] main loop create failed\n");
        g_object_unref(impl->server);
        delete impl;
        return false;
    }

    impl->loop_thread = std::thread([impl]() {
        g_main_loop_run(impl->loop);
    });

    impl->started = true;
    impl_ = impl;

    fprintf(stderr, "[RTSP] stream ready: rtsp://<PI_IP>:%s%s\n", cfg.service.c_str(), cfg.path.c_str());
    return true;
}

bool RtspStreamServer::push_frame(const unsigned char* data, std::size_t bytes,
                                  uint64_t pts, uint64_t duration) {
    if (!data || bytes == 0 || !impl_) return false;

    GstAppSrc* appsrc = nullptr;
    {
        std::lock_guard<std::mutex> lk(impl_->appsrc_mtx);
        if (!impl_->cam_appsrc) return false;
        appsrc = impl_->cam_appsrc;
        g_object_ref(appsrc);
    }

    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, bytes, nullptr);
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        gst_buffer_unref(buffer);
        g_object_unref(appsrc);
        return false;
    }
    std::memcpy(map.data, data, bytes);
    gst_buffer_unmap(buffer, &map);

    // Preserve original camera timestamp for encoder rate-control
    // and future gyro↔frame synchronization.
    GST_BUFFER_PTS(buffer) = static_cast<GstClockTime>(pts);
    GST_BUFFER_DURATION(buffer) = static_cast<GstClockTime>(duration);

    const GstFlowReturn ret = gst_app_src_push_buffer(appsrc, buffer);
    g_object_unref(appsrc);
    return ret == GST_FLOW_OK;
}

void RtspStreamServer::stop() {
    if (!impl_) return;

    {
        std::lock_guard<std::mutex> lk(impl_->appsrc_mtx);
        if (impl_->cam_appsrc) {
            g_object_unref(impl_->cam_appsrc);
            impl_->cam_appsrc = nullptr;
        }
    }

    if (impl_->loop) {
        g_main_loop_quit(impl_->loop);
    }
    if (impl_->loop_thread.joinable()) {
        impl_->loop_thread.join();
    }
    if (impl_->loop) {
        g_main_loop_unref(impl_->loop);
        impl_->loop = nullptr;
    }
    if (impl_->server) {
        g_object_unref(impl_->server);
        impl_->server = nullptr;
    }

    delete impl_;
    impl_ = nullptr;
}
