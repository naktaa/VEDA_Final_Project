#pragma once

#include <cstddef>
#include <cstdint>
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

    /// Push a raw frame (NV12). pts/duration in nanoseconds (GST_CLOCK_TIME_NONE = auto).
    bool push_frame(const unsigned char* data, std::size_t bytes,
                    uint64_t pts, uint64_t duration);

private:
    RtspStreamServerImpl* impl_ = nullptr;
};
