#pragma once

#include <atomic>
#include <string>

class FrameJpegCache;
class PtzController;

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
               FrameJpegCache& raw_frame_cache,
               PtzController& ptz_controller,
               FrameJpegCache* display_frame_cache = nullptr);
    void stop();

private:
    struct Impl;
    Impl* impl_ = nullptr;
};
