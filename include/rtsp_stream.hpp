#pragma once

#include <string>

struct RtspConfig {
    bool enable = true;
    std::string service;
    std::string path;
    std::string launch;
};

class RtspStreamServer {
public:
    RtspStreamServer() = default;
    ~RtspStreamServer();
    RtspStreamServer(const RtspStreamServer&) = delete;
    RtspStreamServer& operator=(const RtspStreamServer&) = delete;
    RtspStreamServer(RtspStreamServer&&) = delete;
    RtspStreamServer& operator=(RtspStreamServer&&) = delete;

    bool start(const RtspConfig& cfg);
    void stop();

private:
    struct Impl;
    Impl* impl_ = nullptr;
};
