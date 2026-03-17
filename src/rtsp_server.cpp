#include "rtsp_server.hpp"

#include <gst/video/video.h>
#include <cstring>
#include <mutex>
#include <string>

#include "eis_common.hpp"
#include "eis_globals.hpp"

static cv::Mat ensureBGR(const cv::Mat& in) {
    if (in.empty()) return cv::Mat();
    cv::Mat out = in;
    if (out.type() == CV_8UC4)
        cv::cvtColor(out, out, cv::COLOR_BGRA2BGR);
    else if (out.type() == CV_8UC1)
        cv::cvtColor(out, out, cv::COLOR_GRAY2BGR);
    else if (out.type() != CV_8UC3) {
        cv::Mat tmp;
        out.convertTo(tmp, CV_8U);
        if (tmp.channels() == 1)
            cv::cvtColor(tmp, out, cv::COLOR_GRAY2BGR);
        else
            out = tmp;
    }
    if (out.cols != G_WIDTH || out.rows != G_HEIGHT)
        cv::resize(out, out, cv::Size(G_WIDTH, G_HEIGHT), 0, 0, cv::INTER_LINEAR);
    if (!out.isContinuous()) out = out.clone();
    return out;
}

void set_appsrc_caps(GstAppSrc* appsrc) {
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
                                        "format", G_TYPE_STRING, "BGR",
                                        "width", G_TYPE_INT, G_WIDTH,
                                        "height", G_TYPE_INT, G_HEIGHT,
                                        "framerate", GST_TYPE_FRACTION, G_FPS, 1, nullptr);
    gst_app_src_set_caps(appsrc, caps);
    gst_caps_unref(caps);
    g_object_set(G_OBJECT(appsrc),
                 "is-live", TRUE, "format", GST_FORMAT_TIME,
                 "do-timestamp", TRUE, "block", FALSE, nullptr);
}

void on_media_configure(GstRTSPMediaFactory*, GstRTSPMedia* media, gpointer user_data) {
    const char* which = static_cast<const char*>(user_data);
    GstElement* element = gst_rtsp_media_get_element(media);
    const char* name = (strcmp(which, "raw") == 0) ? "rawsrc" : "stabsrc";
    GstElement* app = gst_bin_get_by_name_recurse_up(GST_BIN(element), name);
    gst_object_unref(element);
    if (!app) {
        g_printerr("Failed to get appsrc: %s\n", name);
        return;
    }
    GstAppSrc* appsrc = GST_APP_SRC(app);
    set_appsrc_caps(appsrc);
    {
        std::lock_guard<std::mutex> lk(g_mtx);
        if (strcmp(which, "raw") == 0) {
            g_rawsrc = appsrc;
            g_print("[RTSP] /raw connected\n");
        } else {
            g_stabsrc = appsrc;
            g_print("[RTSP] /cam connected\n");
        }
    }
}

GstRTSPMediaFactory* make_factory(const char* appsrc_name) {
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
    std::string launch =
        "( appsrc name=" + std::string(appsrc_name) + " is-live=true format=time do-timestamp=true block=false "
                                                      "! videoconvert "
                                                      "! video/x-raw,format=I420 "
                                                      "! v4l2h264enc extra-controls=\"controls,video_bitrate=1500000,h264_i_frame_period=30\" "
                                                      "! video/x-h264,level=(string)4,profile=(string)baseline "
                                                      "! rtph264pay name=pay0 pt=96 config-interval=1 )";
    gst_rtsp_media_factory_set_launch(factory, launch.c_str());
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    gst_rtsp_media_factory_set_suspend_mode(factory, GST_RTSP_SUSPEND_MODE_NONE);
    return factory;
}

bool push_bgr(GstAppSrc* appsrc, const cv::Mat& frame, guint64, const char* tag) {
    if (!appsrc) return false;
    cv::Mat bgr = ensureBGR(frame);
    if (bgr.empty()) return false;
    const size_t bytes = bgr.total() * bgr.elemSize();
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, bytes, nullptr);
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        gst_buffer_unref(buffer);
        return false;
    }
    memcpy(map.data, bgr.data, bytes);
    gst_buffer_unmap(buffer, &map);
    GstFlowReturn ret = gst_app_src_push_buffer(appsrc, buffer);
    if (ret != GST_FLOW_OK) {
        g_printerr("[push:%s] failed\n", tag);
        return false;
    }
    return true;
}
