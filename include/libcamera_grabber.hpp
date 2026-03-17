#pragma once

#include <opencv2/opencv.hpp>

#include <condition_variable>
#include <deque>
#include <map>
#include <memory>

#include <libcamera/libcamera.h>
#include <libcamera/control_ids.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/formats.h>
#include <libcamera/base/span.h>

#include "eis_common.hpp"

class LibcameraGrabber {
public:
    bool init();
    void shutdown();
    bool get_frame(cv::Mat &out, double &out_time_ms, TsSource &out_src,
                   int64_t &out_sensor_ts_ns, int &out_exp_us, uint64_t &out_index);
    ~LibcameraGrabber();

private:
    struct MappedBuffer {
        std::vector<void *> data;
        std::vector<size_t> lengths;
    };

    bool map_buffer(const libcamera::FrameBuffer *buffer);
    void request_complete(libcamera::Request *request);

    std::unique_ptr<libcamera::CameraManager> cm_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream *stream_ = nullptr;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::map<const libcamera::FrameBuffer *, MappedBuffer> mapped_;

    std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<libcamera::Request *> queue_;

    bool started_ = false;
    bool sensor_map_ready_ = false;
    int64_t sensor_ts_first_ns_ = 0;
    int64_t sensor_raw_base_ns_ = 0;
    uint64_t frame_index_ = 0;
};

