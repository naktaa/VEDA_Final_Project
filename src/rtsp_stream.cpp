#include "rtsp_stream.hpp"

#include <cstdio>
#include <thread>

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

struct RtspStreamServer::Impl {
    GMainLoop* loop = nullptr;
    GstRTSPServer* server = nullptr;
    std::thread loop_thread;
    bool started = false;
};

RtspStreamServer::~RtspStreamServer() {
    stop();
}

bool RtspStreamServer::start(const RtspConfig& cfg) {
    if (!cfg.enable) return true;
    if (impl_ && impl_->started) return true;

    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }

    auto* impl = new Impl();
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

void RtspStreamServer::stop() {
    if (!impl_) return;

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
