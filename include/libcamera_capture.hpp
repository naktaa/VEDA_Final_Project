#pragma once

#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <vector>

#include <libcamera/base/span.h>
#include <libcamera/control_ids.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/formats.h>
#include <libcamera/libcamera.h>

#include "app_config.hpp"
#include "app_types.hpp"

class LibcameraCapture {
public:
    LibcameraCapture() = default;
    ~LibcameraCapture();

    bool init(const CameraConfig& config, std::string* error = nullptr);
    void shutdown();
    bool get_frame(CapturedFrame& out, std::string* error = nullptr);

private:
    struct MappedBuffer {
        std::vector<void*> data;
        std::vector<size_t> lengths;
    };

    bool map_buffer(const libcamera::FrameBuffer* buffer, std::string* error);
    void request_complete(libcamera::Request* request);

    CameraConfig config_;
    std::unique_ptr<libcamera::CameraManager> manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> camera_config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream* stream_ = nullptr;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::map<const libcamera::FrameBuffer*, MappedBuffer> mapped_;

    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<libcamera::Request*> ready_queue_;

    bool started_ = false;
    bool sensor_map_ready_ = false;
    int64_t sensor_ts_first_ns_ = 0;
    int64_t sensor_raw_base_ns_ = 0;
    uint64_t frame_index_ = 0;
};
