#pragma once

#include <cstddef>
#include <string>

struct WebRtcConfig {
    bool enable = true;
    int width = 640;
    int height = 480;
    int fps = 20;
    int bitrate_bps = 1200000;
};

class WebRtcStreamServer {
public:
    WebRtcStreamServer() = default;
    ~WebRtcStreamServer();
    WebRtcStreamServer(const WebRtcStreamServer&) = delete;
    WebRtcStreamServer& operator=(const WebRtcStreamServer&) = delete;
    WebRtcStreamServer(WebRtcStreamServer&&) = delete;
    WebRtcStreamServer& operator=(WebRtcStreamServer&&) = delete;

    bool start(const WebRtcConfig& cfg);
    void stop();

    bool create_session(const std::string& offer_sdp, std::string& answer_sdp, std::string& error);
    void close_session();

    bool push_bgr_frame(const unsigned char* data, std::size_t bytes, int width, int height);
    bool has_active_session() const;

private:
    struct Impl;
    Impl* impl_ = nullptr;
};
