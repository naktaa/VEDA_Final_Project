#include "rtsp_server.hpp"

#include <algorithm>
#include <cstring>
#include <string>

#include <gst/gst.h>
#include <gst/video/video.h>

#include "image_utils.hpp"

namespace {

struct MediaConfigureContext {
    RtspServer::StreamSlot* slot = nullptr;
    int width = 0;
    int height = 0;
    int fps = 0;
    const char* appsrc_name = nullptr;
};

std::string make_launch(const char* appsrc_name, const RtspConfig& config) {
    return "( appsrc name=" + std::string(appsrc_name) +
           " is-live=true format=time block=false do-timestamp=false "
           "! videoconvert "
           "! video/x-raw,format=I420 "
           "! v4l2h264enc extra-controls=\"controls,video_bitrate=" + std::to_string(config.bitrate) +
           ",h264_i_frame_period=" + std::to_string(config.iframe_period) +
           "\" "
           "! video/x-h264,level=(string)4,profile=(string)baseline "
           "! rtph264pay name=pay0 pt=96 config-interval=1 )";
}

} // namespace

RtspServer::~RtspServer() {
    stop();
}

bool RtspServer::start(const CameraConfig& camera_config, const RtspConfig& rtsp_config, std::string* error) {
    stop();

    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }

    camera_config_ = camera_config;
    rtsp_config_ = rtsp_config;

    server_ = gst_rtsp_server_new();
    if (!server_) {
        if (error) *error = "failed to create RTSP server";
        return false;
    }
    gst_rtsp_server_set_service(server_, rtsp_config_.port.c_str());

    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server_);

    MediaConfigureContext* stab_context = new MediaConfigureContext{
        &stabilized_slot_,
        camera_config_.width,
        camera_config_.height,
        camera_config_.fps,
        "stabsrc",
    };
    GstRTSPMediaFactory* stab_factory = gst_rtsp_media_factory_new();
    const std::string stab_launch = make_launch(stab_context->appsrc_name, rtsp_config_);
    gst_rtsp_media_factory_set_launch(stab_factory, stab_launch.c_str());
    gst_rtsp_media_factory_set_shared(stab_factory, TRUE);
    gst_rtsp_media_factory_set_suspend_mode(stab_factory, GST_RTSP_SUSPEND_MODE_NONE);
    g_signal_connect(stab_factory, "media-configure", GCallback(on_media_configure), stab_context);
    gst_rtsp_mount_points_add_factory(mounts, rtsp_config_.path.c_str(), stab_factory);

    MediaConfigureContext* raw_context = new MediaConfigureContext{
        &raw_slot_,
        camera_config_.width,
        camera_config_.height,
        camera_config_.fps,
        "rawsrc",
    };
    GstRTSPMediaFactory* raw_factory = gst_rtsp_media_factory_new();
    const std::string raw_launch = make_launch(raw_context->appsrc_name, rtsp_config_);
    gst_rtsp_media_factory_set_launch(raw_factory, raw_launch.c_str());
    gst_rtsp_media_factory_set_shared(raw_factory, TRUE);
    gst_rtsp_media_factory_set_suspend_mode(raw_factory, GST_RTSP_SUSPEND_MODE_NONE);
    g_signal_connect(raw_factory, "media-configure", GCallback(on_media_configure), raw_context);
    gst_rtsp_mount_points_add_factory(mounts, rtsp_config_.raw_path.c_str(), raw_factory);

    g_object_unref(mounts);

    if (gst_rtsp_server_attach(server_, nullptr) == 0) {
        if (error) *error = "failed to attach RTSP server";
        g_object_unref(server_);
        server_ = nullptr;
        return false;
    }

    main_loop_ = g_main_loop_new(nullptr, FALSE);
    if (!main_loop_) {
        if (error) *error = "failed to create RTSP main loop";
        g_object_unref(server_);
        server_ = nullptr;
        return false;
    }

    loop_thread_ = std::thread([this]() {
        g_main_loop_run(main_loop_);
    });

    started_ = true;
    return true;
}

void RtspServer::stop() {
    auto cleanup_slot = [](StreamSlot& slot) {
        std::lock_guard<std::mutex> lock(slot.mutex);
        if (slot.appsrc) {
            g_object_unref(slot.appsrc);
            slot.appsrc = nullptr;
        }
        slot.has_origin = false;
        slot.origin_frame_time_ms = 0.0;
    };

    cleanup_slot(raw_slot_);
    cleanup_slot(stabilized_slot_);

    if (main_loop_) {
        g_main_loop_quit(main_loop_);
    }
    if (loop_thread_.joinable()) {
        loop_thread_.join();
    }
    if (main_loop_) {
        g_main_loop_unref(main_loop_);
        main_loop_ = nullptr;
    }
    if (server_) {
        g_object_unref(server_);
        server_ = nullptr;
    }
    started_ = false;
}

bool RtspServer::push_stabilized(const CapturedFrame& frame, const cv::Mat& image) {
    return push_frame(stabilized_slot_, frame, image, camera_config_.width, camera_config_.height, camera_config_.fps);
}

bool RtspServer::push_raw(const CapturedFrame& frame, const cv::Mat& image) {
    return push_frame(raw_slot_, frame, image, camera_config_.width, camera_config_.height, camera_config_.fps);
}

void RtspServer::on_media_configure(GstRTSPMediaFactory*, GstRTSPMedia* media, gpointer user_data) {
    auto* context = static_cast<MediaConfigureContext*>(user_data);
    if (!context || !context->slot) {
        return;
    }

    GstElement* element = gst_rtsp_media_get_element(media);
    GstElement* app = gst_bin_get_by_name_recurse_up(GST_BIN(element), context->appsrc_name);
    gst_object_unref(element);
    if (!app) {
        return;
    }

    GstAppSrc* appsrc = GST_APP_SRC(app);
    set_appsrc_caps(appsrc, context->width, context->height, context->fps);

    g_object_ref(appsrc);
    {
        std::lock_guard<std::mutex> lock(context->slot->mutex);
        if (context->slot->appsrc) {
            g_object_unref(context->slot->appsrc);
        }
        context->slot->appsrc = appsrc;
        context->slot->has_origin = false;
        context->slot->origin_frame_time_ms = 0.0;
    }
    gst_object_unref(app);
}

void RtspServer::set_appsrc_caps(GstAppSrc* appsrc, int width, int height, int fps) {
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
                                        "format", G_TYPE_STRING, "BGR",
                                        "width", G_TYPE_INT, width,
                                        "height", G_TYPE_INT, height,
                                        "framerate", GST_TYPE_FRACTION, fps, 1,
                                        nullptr);
    gst_app_src_set_caps(appsrc, caps);
    gst_caps_unref(caps);

    g_object_set(G_OBJECT(appsrc),
                 "is-live", TRUE,
                 "format", GST_FORMAT_TIME,
                 "block", FALSE,
                 "do-timestamp", FALSE,
                 nullptr);
}

bool RtspServer::push_frame(StreamSlot& slot,
                            const CapturedFrame& frame,
                            const cv::Mat& image,
                            int width,
                            int height,
                            int fps) {
    GstAppSrc* appsrc = nullptr;
    double origin_ms = 0.0;
    {
        std::lock_guard<std::mutex> lock(slot.mutex);
        if (!slot.appsrc) {
            return false;
        }
        if (!slot.has_origin) {
            slot.origin_frame_time_ms = frame.frame_time_ms;
            slot.has_origin = true;
        }
        origin_ms = slot.origin_frame_time_ms;
        appsrc = slot.appsrc;
        g_object_ref(appsrc);
    }

    cv::Mat bgr = ensure_bgr(image, false, width, height);
    if (bgr.empty()) {
        g_object_unref(appsrc);
        return false;
    }

    const size_t bytes = bgr.total() * bgr.elemSize();
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, bytes, nullptr);
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        gst_buffer_unref(buffer);
        g_object_unref(appsrc);
        return false;
    }
    std::memcpy(map.data, bgr.data, bytes);
    gst_buffer_unmap(buffer, &map);

    const guint64 pts_ns = static_cast<guint64>(std::max(0.0, frame.frame_time_ms - origin_ms) * 1e6);
    const guint64 duration_ns = frame.frame_duration_us > 0
        ? static_cast<guint64>(frame.frame_duration_us) * 1000ULL
        : static_cast<guint64>(1000000000.0 / std::max(1, fps));
    GST_BUFFER_PTS(buffer) = pts_ns;
    GST_BUFFER_DTS(buffer) = pts_ns;
    GST_BUFFER_DURATION(buffer) = duration_ns;

    const GstFlowReturn result = gst_app_src_push_buffer(appsrc, buffer);
    g_object_unref(appsrc);
    return result == GST_FLOW_OK;
}
