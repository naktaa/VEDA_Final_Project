#pragma once

#include <atomic>
#include <functional>
#include <string>

class FrameJpegCache;
class PtzController;

struct HttpVrConfig {
    bool enable = true;
    int port = 8000;
    std::string web_root = "../web/tilt_vr";
    bool https_enable = false;
    int https_port = 8443;
    std::string tls_cert_file = "../certs/tiltvrsvr-cert.pem";
    std::string tls_key_file = "../certs/tiltvrsvr-key.pem";
    bool redirect_http_to_https = true;
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
