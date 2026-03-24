#pragma once

#include <atomic>
#include <string>

class FrameJpegCache;
class PtzController;
class WebRtcStreamServer;
class VrUiCommandBridge;

struct HttpVrConfig {
    bool enable = true;
    int port = 8000;
    std::string web_root;
};

class HttpVrServer {
public:
    HttpVrServer() = default;
    ~HttpVrServer();
    HttpVrServer(const HttpVrServer&) = delete;
    HttpVrServer& operator=(const HttpVrServer&) = delete;
    HttpVrServer(HttpVrServer&&) = delete;
    HttpVrServer& operator=(HttpVrServer&&) = delete;

    bool start(const HttpVrConfig& cfg,
               std::atomic<bool>& app_running,
               FrameJpegCache& frame_cache,
               PtzController& ptz_controller,
               WebRtcStreamServer& web_rtc_server,
               VrUiCommandBridge& vr_ui_command_bridge);
    void stop();

private:
    struct Impl;
    Impl* impl_ = nullptr;
};
