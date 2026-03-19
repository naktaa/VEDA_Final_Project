#pragma once

#include <atomic>

class RtspStreamServer;

class CameraCapture {
public:
    CameraCapture() = default;
    ~CameraCapture();
    CameraCapture(const CameraCapture&) = delete;
    CameraCapture& operator=(const CameraCapture&) = delete;
    CameraCapture(CameraCapture&&) = delete;
    CameraCapture& operator=(CameraCapture&&) = delete;

    bool start(std::atomic<bool>& app_running, RtspStreamServer& rtsp_server);
    void stop();

private:
    struct Impl;
    Impl* impl_ = nullptr;
};

