#pragma once

#include <cstddef>
#include <string>

struct RtspConfig {
    bool enable = true;
    std::string service;
    std::string path;
    std::string launch;
};

struct RtspStreamServerImpl;

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
    bool push_bgr_frame(const unsigned char* data, std::size_t bytes);

private:
    RtspStreamServerImpl* impl_ = nullptr;
};
