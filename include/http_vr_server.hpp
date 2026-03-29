#pragma once

#include <atomic>
#include <functional>
#include <string>

class FrameJpegCache;
class PtzController;

struct HttpVrConfig {
    bool enable = true;
    int port = 8000;
    std::string web_root;
};

struct HttpVrOverlayState {
    bool valid = false;
    bool stale = false;
    double x = 0.0;
    double y = 0.0;
    double yaw_rad = 0.0;
    std::string frame = "world";
    long long ts_ms = 0;
};

class HttpVrServer {
public:
    using OverlayStateProvider = std::function<HttpVrOverlayState()>;

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
               OverlayStateProvider overlay_state_provider = {});
    void stop();

private:
    struct Impl;
    Impl* impl_ = nullptr;
};
