#include "web_rtc_stream.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/sdp/gstsdpmessage.h>
#include <gst/webrtc/webrtc.h>

namespace {

using Clock = std::chrono::steady_clock;

constexpr auto kIceGatherTimeout = std::chrono::seconds(3);

struct SessionState {
    GstElement* pipeline = nullptr;
    GstAppSrc* appsrc = nullptr;
    GstElement* webrtc = nullptr;
};

void cleanup_session(SessionState& session) {
    if (session.pipeline) {
        gst_element_set_state(session.pipeline, GST_STATE_NULL);
    }
    if (session.appsrc) {
        gst_object_unref(session.appsrc);
        session.appsrc = nullptr;
    }
    if (session.webrtc) {
        gst_object_unref(session.webrtc);
        session.webrtc = nullptr;
    }
    if (session.pipeline) {
        gst_object_unref(session.pipeline);
        session.pipeline = nullptr;
    }
}

void set_appsrc_caps(GstAppSrc* appsrc, const WebRtcConfig& cfg) {
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
                                        "format", G_TYPE_STRING, "BGR",
                                        "width", G_TYPE_INT, cfg.width,
                                        "height", G_TYPE_INT, cfg.height,
                                        "framerate", GST_TYPE_FRACTION, cfg.fps, 1,
                                        nullptr);
    gst_app_src_set_caps(appsrc, caps);
    gst_caps_unref(caps);
    g_object_set(G_OBJECT(appsrc),
                 "is-live", TRUE,
                 "format", GST_FORMAT_TIME,
                 "do-timestamp", TRUE,
                 "block", FALSE,
                 nullptr);
}

std::string make_pipeline_launch(const WebRtcConfig& cfg) {
    std::ostringstream launch;
    launch
        << "webrtcbin name=webrtc bundle-policy=max-bundle latency=0 "
        << "appsrc name=webrtcsrc is-live=true format=time do-timestamp=true block=false "
        << "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        << "! videoconvert "
        << "! video/x-raw,format=I420,width=" << cfg.width
        << ",height=" << cfg.height
        << ",framerate=" << cfg.fps << "/1 "
        << "! v4l2h264enc extra-controls=\"controls,video_bitrate=" << cfg.bitrate_bps
        << ",h264_i_frame_period=" << cfg.fps << "\" "
        << "! video/x-h264,profile=baseline "
        << "! h264parse "
        << "! rtph264pay pt=96 config-interval=-1 aggregate-mode=zero-latency "
        << "! application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000 "
        << "! queue "
        << "! webrtc.";
    return launch.str();
}

bool wait_for_promise(GstPromise* promise, std::string& error, const char* label) {
    const GstPromiseResult result = gst_promise_wait(promise);
    if (result == GST_PROMISE_RESULT_REPLIED) {
        return true;
    }
    std::ostringstream oss;
    oss << label << " promise failed (" << static_cast<int>(result) << ")";
    error = oss.str();
    return false;
}

bool parse_offer_sdp(const std::string& offer_sdp,
                     GstWebRTCSessionDescription** out_offer,
                     std::string& error) {
    GstSDPMessage* sdp = nullptr;
    if (gst_sdp_message_new(&sdp) != GST_SDP_OK) {
        error = "failed to allocate SDP message";
        return false;
    }

    const guint8* buffer = reinterpret_cast<const guint8*>(offer_sdp.data());
    const guint size = static_cast<guint>(offer_sdp.size());
    if (gst_sdp_message_parse_buffer(buffer, size, sdp) != GST_SDP_OK) {
        gst_sdp_message_free(sdp);
        error = "failed to parse browser offer SDP";
        return false;
    }

    *out_offer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp);
    return true;
}

bool wait_for_ice_complete(GstElement* webrtc, std::string& error) {
    const auto deadline = Clock::now() + kIceGatherTimeout;
    while (Clock::now() < deadline) {
        GstWebRTCICEGatheringState state = GST_WEBRTC_ICE_GATHERING_STATE_NEW;
        g_object_get(G_OBJECT(webrtc), "ice-gathering-state", &state, nullptr);
        if (state == GST_WEBRTC_ICE_GATHERING_STATE_COMPLETE) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    error = "ICE gathering timed out";
    return false;
}

bool session_description_to_text(GstElement* webrtc, std::string& sdp_text, std::string& error) {
    GstWebRTCSessionDescription* local_desc = nullptr;
    g_object_get(G_OBJECT(webrtc), "local-description", &local_desc, nullptr);
    if (!local_desc || !local_desc->sdp) {
        if (local_desc) {
            gst_webrtc_session_description_free(local_desc);
        }
        error = "local WebRTC description unavailable";
        return false;
    }

    gchar* text = gst_sdp_message_as_text(local_desc->sdp);
    if (!text) {
        gst_webrtc_session_description_free(local_desc);
        error = "failed to serialize local SDP";
        return false;
    }

    sdp_text = text;
    g_free(text);
    gst_webrtc_session_description_free(local_desc);
    return true;
}

bool create_answer(GstElement* webrtc, std::string& error) {
    GstPromise* answer_promise = gst_promise_new();
    g_signal_emit_by_name(webrtc, "create-answer", nullptr, answer_promise);
    if (!wait_for_promise(answer_promise, error, "create-answer")) {
        gst_promise_unref(answer_promise);
        return false;
    }

    const GstStructure* reply = gst_promise_get_reply(answer_promise);
    GstWebRTCSessionDescription* answer = nullptr;
    gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, nullptr);
    gst_promise_unref(answer_promise);

    if (!answer) {
        error = "webrtcbin did not return an SDP answer";
        return false;
    }

    GstPromise* local_promise = gst_promise_new();
    g_signal_emit_by_name(webrtc, "set-local-description", answer, local_promise);
    const bool ok = wait_for_promise(local_promise, error, "set-local-description");
    gst_promise_unref(local_promise);
    gst_webrtc_session_description_free(answer);
    return ok;
}

bool set_remote_description(GstElement* webrtc,
                            GstWebRTCSessionDescription* offer,
                            std::string& error) {
    GstPromise* promise = gst_promise_new();
    g_signal_emit_by_name(webrtc, "set-remote-description", offer, promise);
    const bool ok = wait_for_promise(promise, error, "set-remote-description");
    gst_promise_unref(promise);
    return ok;
}

} // namespace

struct WebRtcStreamServer::Impl {
    WebRtcConfig config;
    mutable std::mutex mutex;
    bool running = false;
    SessionState session;
};

WebRtcStreamServer::~WebRtcStreamServer() {
    stop();
}

bool WebRtcStreamServer::start(const WebRtcConfig& cfg) {
    if (impl_) {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->config = cfg;
        impl_->running = cfg.enable;
        return true;
    }

    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }

    auto* impl = new Impl();
    impl->config = cfg;
    impl->running = cfg.enable;
    impl_ = impl;
    return true;
}

void WebRtcStreamServer::stop() {
    if (!impl_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cleanup_session(impl_->session);
        impl_->running = false;
    }

    delete impl_;
    impl_ = nullptr;
}

bool WebRtcStreamServer::create_session(const std::string& offer_sdp,
                                        std::string& answer_sdp,
                                        std::string& error) {
    if (!impl_) {
        error = "webrtc server is not initialized";
        return false;
    }

    WebRtcConfig config;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!impl_->running) {
            error = "webrtc server is disabled";
            return false;
        }
        config = impl_->config;
    }

    GError* gst_error = nullptr;
    SessionState next_session;
    const std::string launch = make_pipeline_launch(config);
    next_session.pipeline = gst_parse_launch(launch.c_str(), &gst_error);
    if (!next_session.pipeline) {
        error = gst_error ? gst_error->message : "failed to create webrtc pipeline";
        if (gst_error) {
            g_error_free(gst_error);
        }
        return false;
    }
    if (gst_error) {
        std::fprintf(stderr, "[WEBRTC] pipeline warning: %s\n", gst_error->message);
        g_error_free(gst_error);
    }

    std::fprintf(stderr, "[WEBRTC] launch: %s\n", launch.c_str());

    GstElement* appsrc_element = gst_bin_get_by_name(GST_BIN(next_session.pipeline), "webrtcsrc");
    GstElement* webrtc_element = gst_bin_get_by_name(GST_BIN(next_session.pipeline), "webrtc");
    if (!appsrc_element || !webrtc_element) {
        if (appsrc_element) {
            gst_object_unref(appsrc_element);
        }
        if (webrtc_element) {
            gst_object_unref(webrtc_element);
        }
        cleanup_session(next_session);
        error = "webrtc pipeline missing appsrc/webrtcbin";
        return false;
    }

    next_session.appsrc = GST_APP_SRC(appsrc_element);
    next_session.webrtc = webrtc_element;
    set_appsrc_caps(next_session.appsrc, config);

    const GstStateChangeReturn state_rc =
        gst_element_set_state(next_session.pipeline, GST_STATE_PLAYING);
    if (state_rc == GST_STATE_CHANGE_FAILURE) {
        cleanup_session(next_session);
        error = "failed to start webrtc pipeline";
        return false;
    }

    GstWebRTCSessionDescription* offer = nullptr;
    if (!parse_offer_sdp(offer_sdp, &offer, error)) {
        cleanup_session(next_session);
        return false;
    }

    const bool ok =
        set_remote_description(next_session.webrtc, offer, error) &&
        create_answer(next_session.webrtc, error) &&
        wait_for_ice_complete(next_session.webrtc, error) &&
        session_description_to_text(next_session.webrtc, answer_sdp, error);
    gst_webrtc_session_description_free(offer);

    if (!ok) {
        cleanup_session(next_session);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cleanup_session(impl_->session);
        impl_->session = next_session;
    }

    std::fprintf(stderr, "[WEBRTC] session established\n");
    return true;
}

void WebRtcStreamServer::close_session() {
    if (!impl_) {
        return;
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    cleanup_session(impl_->session);
}

bool WebRtcStreamServer::push_bgr_frame(const unsigned char* data,
                                        std::size_t bytes,
                                        int width,
                                        int height) {
    if (!impl_ || !data || bytes == 0) {
        return false;
    }

    GstAppSrc* appsrc = nullptr;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!impl_->running || !impl_->session.appsrc) {
            return false;
        }
        appsrc = impl_->session.appsrc;
        g_object_ref(appsrc);
    }

    const std::size_t expected =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3U;
    if (bytes < expected) {
        g_object_unref(appsrc);
        return false;
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

    const GstFlowReturn ret = gst_app_src_push_buffer(appsrc, buffer);
    g_object_unref(appsrc);
    return ret == GST_FLOW_OK;
}

bool WebRtcStreamServer::has_active_session() const {
    if (!impl_) {
        return false;
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->session.appsrc != nullptr;
}
