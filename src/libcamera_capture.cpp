#include "libcamera_capture.hpp"

#include <cstdio>
#include <sys/mman.h>

#include "image_utils.hpp"
#include "timebase.hpp"

namespace {

libcamera::PixelFormat select_pixel_format() {
    // Prefer an unambiguous 3-channel format. libcamera may still negotiate a
    // different format after validate(), so get_frame() converts based on the
    // actual stream configuration.
    return libcamera::formats::BGR888;
}

const char* pixel_format_name(const libcamera::PixelFormat& pixel_format) {
    if (pixel_format == libcamera::formats::BGR888) {
        return "BGR888";
    }
    if (pixel_format == libcamera::formats::RGB888) {
        return "RGB888";
    }
    if (pixel_format == libcamera::formats::XRGB8888) {
        return "XRGB8888";
    }
    if (pixel_format == libcamera::formats::XBGR8888) {
        return "XBGR8888";
    }
    return "UNKNOWN";
}

bool is_supported_pixel_format(const libcamera::PixelFormat& pixel_format) {
    return pixel_format == libcamera::formats::BGR888 ||
           pixel_format == libcamera::formats::RGB888 ||
           pixel_format == libcamera::formats::XRGB8888 ||
           pixel_format == libcamera::formats::XBGR8888;
}

cv::Mat resize_if_needed(cv::Mat image, int target_width, int target_height) {
    if (image.empty()) {
        return {};
    }
    if (image.cols != target_width || image.rows != target_height) {
        cv::resize(image,
                   image,
                   cv::Size(target_width, target_height),
                   0.0,
                   0.0,
                   cv::INTER_LINEAR);
    }
    if (!image.isContinuous()) {
        image = image.clone();
    }
    return image;
}

cv::Mat convert_xrgb_to_bgr(const cv::Mat& input) {
    cv::Mat converted(input.rows, input.cols, CV_8UC3);
    const int from_to[] = {3, 0, 2, 1, 1, 2};
    cv::mixChannels(&input, 1, &converted, 1, from_to, 3);
    return converted;
}

cv::Mat convert_xbgr_to_bgr(const cv::Mat& input) {
    cv::Mat converted(input.rows, input.cols, CV_8UC3);
    const int from_to[] = {1, 0, 2, 1, 3, 2};
    cv::mixChannels(&input, 1, &converted, 1, from_to, 3);
    return converted;
}

cv::Mat convert_libcamera_to_bgr(const cv::Mat& input,
                                 const libcamera::PixelFormat& pixel_format,
                                 bool xrgb_fallback,
                                 int target_width,
                                 int target_height) {
    if (input.empty()) {
        return {};
    }

    cv::Mat output;
    if (pixel_format == libcamera::formats::BGR888) {
        output = input.clone();
    } else if (pixel_format == libcamera::formats::RGB888) {
        cv::cvtColor(input, output, cv::COLOR_RGB2BGR);
    } else if (pixel_format == libcamera::formats::XRGB8888) {
        output = convert_xrgb_to_bgr(input);
    } else if (pixel_format == libcamera::formats::XBGR8888) {
        output = convert_xbgr_to_bgr(input);
    } else if (input.type() == CV_8UC4) {
        if (xrgb_fallback) {
            output = convert_xrgb_to_bgr(input);
        } else {
            cv::cvtColor(input, output, cv::COLOR_BGRA2BGR);
        }
    } else {
        output = ensure_bgr(input, false, target_width, target_height);
    }

    return resize_if_needed(output, target_width, target_height);
}

} // namespace

LibcameraCapture::~LibcameraCapture() {
    shutdown();
}

bool LibcameraCapture::init(const CameraConfig& config, std::string* error) {
    config_ = config;

    manager_ = std::make_unique<libcamera::CameraManager>();
    if (manager_->start() != 0) {
        if (error) *error = "CameraManager start failed";
        return false;
    }
    if (manager_->cameras().empty()) {
        if (error) *error = "No cameras found";
        return false;
    }

    camera_ = manager_->cameras()[0];
    if (camera_->acquire() != 0) {
        if (error) *error = "Camera acquire failed";
        return false;
    }

    camera_config_ = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if (!camera_config_) {
        if (error) *error = "Failed to generate camera configuration";
        return false;
    }

    libcamera::StreamConfiguration& stream_config = camera_config_->at(0);
    stream_config.size.width = static_cast<unsigned int>(config_.width);
    stream_config.size.height = static_cast<unsigned int>(config_.height);
    stream_config.pixelFormat = select_pixel_format();
    stream_config.bufferCount = 4;

    if (camera_config_->validate() == libcamera::CameraConfiguration::Invalid) {
        if (error) *error = "Camera configuration invalid";
        return false;
    }
    if (camera_->configure(camera_config_.get()) != 0) {
        if (error) *error = "Camera configure failed";
        return false;
    }

    const libcamera::StreamConfiguration& active_stream_config = camera_config_->at(0);
    if (!is_supported_pixel_format(active_stream_config.pixelFormat)) {
        if (error) {
            *error = "unsupported libcamera pixel format after configure: " +
                     std::string(pixel_format_name(active_stream_config.pixelFormat));
        }
        return false;
    }
    std::fprintf(stderr,
                 "[CAP] libcamera format=%s %ux%u stride=%u requested=%s\n",
                 pixel_format_name(active_stream_config.pixelFormat),
                 active_stream_config.size.width,
                 active_stream_config.size.height,
                 active_stream_config.stride,
                 pixel_format_name(select_pixel_format()));

    stream_ = stream_config.stream();
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    if (allocator_->allocate(stream_) < 0) {
        if (error) *error = "Frame buffer allocation failed";
        return false;
    }

    for (const auto& buffer : allocator_->buffers(stream_)) {
        if (!map_buffer(buffer.get(), error)) {
            return false;
        }
    }

    for (const auto& buffer : allocator_->buffers(stream_)) {
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request) {
            if (error) *error = "Failed to create libcamera request";
            return false;
        }
        if (request->addBuffer(stream_, buffer.get()) != 0) {
            if (error) *error = "Failed to attach frame buffer to request";
            return false;
        }
        requests_.push_back(std::move(request));
    }

    camera_->requestCompleted.connect(this, &LibcameraCapture::request_complete);

    libcamera::ControlList controls(camera_->controls());
    const int64_t frame_limits[2] = {
        static_cast<int64_t>(config_.frame_duration_us),
        static_cast<int64_t>(config_.frame_duration_us),
    };
    controls.set(libcamera::controls::FrameDurationLimits,
                 libcamera::Span<const int64_t, 2>(frame_limits));
    controls.set(libcamera::controls::AeEnable, false);
    controls.set(libcamera::controls::ExposureTime, config_.exposure_us);

    if (camera_->start(&controls) != 0) {
        if (error) *error = "Camera start failed";
        return false;
    }

    for (auto& request : requests_) {
        if (camera_->queueRequest(request.get()) < 0) {
            if (error) *error = "Failed to queue libcamera request";
            return false;
        }
    }

    started_ = true;
    return true;
}

void LibcameraCapture::shutdown() {
    if (started_ && camera_) {
        camera_->stop();
        camera_->requestCompleted.disconnect(this, &LibcameraCapture::request_complete);
    }
    started_ = false;
    condition_.notify_all();

    for (auto& entry : mapped_) {
        for (size_t i = 0; i < entry.second.data.size(); ++i) {
            munmap(entry.second.data[i], entry.second.lengths[i]);
        }
    }
    mapped_.clear();

    if (camera_) {
        camera_->release();
    }
    allocator_.reset();
    camera_config_.reset();
    camera_.reset();
    if (manager_) {
        manager_->stop();
    }
    manager_.reset();
}

bool LibcameraCapture::get_frame(CapturedFrame& out, std::string* error) {
    out = CapturedFrame{};
    if (!started_) {
        if (error) *error = "Capture is not started";
        return false;
    }

    libcamera::Request* request = nullptr;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [&] { return !ready_queue_.empty() || !started_; });
        if (!started_) {
            if (error) *error = "Capture stopped";
            return false;
        }
        request = ready_queue_.front();
        ready_queue_.pop_front();
    }

    const auto& buffers = request->buffers();
    auto buffer_it = buffers.find(stream_);
    if (buffer_it == buffers.end()) {
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        if (error) *error = "Request buffer lookup failed";
        return false;
    }

    libcamera::FrameBuffer* buffer = buffer_it->second;
    auto mapped_it = mapped_.find(buffer);
    if (mapped_it == mapped_.end()) {
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        if (error) *error = "Mapped buffer lookup failed";
        return false;
    }

    const libcamera::StreamConfiguration& stream_config = camera_config_->at(0);
    if (!is_supported_pixel_format(stream_config.pixelFormat)) {
        if (error) {
            *error = "unsupported libcamera pixel format at runtime: " +
                     std::string(pixel_format_name(stream_config.pixelFormat));
        }
        return false;
    }
    auto* data = static_cast<uint8_t*>(mapped_it->second.data[0]);
    const size_t stride = stream_config.stride;

    const int cv_type =
        (stream_config.pixelFormat == libcamera::formats::BGR888 ||
         stream_config.pixelFormat == libcamera::formats::RGB888)
            ? CV_8UC3
            : CV_8UC4;
    cv::Mat raw(static_cast<int>(stream_config.size.height),
                static_cast<int>(stream_config.size.width),
                cv_type,
                data,
                stride);
    out.image = convert_libcamera_to_bgr(raw,
                                         stream_config.pixelFormat,
                                         config_.libcamera_xrgb,
                                         config_.width,
                                         config_.height);

    const libcamera::ControlList& meta = request->metadata();
    int exposure_us = config_.exposure_us;
    int64_t sensor_ts_ns = 0;
    if (meta.contains(libcamera::controls::SensorTimestamp.id())) {
        if (const auto value = meta.get(libcamera::controls::SensorTimestamp)) {
            sensor_ts_ns = *value;
        }
    }
    if (meta.contains(libcamera::controls::ExposureTime.id())) {
        if (const auto value = meta.get(libcamera::controls::ExposureTime)) {
            exposure_us = *value;
        }
    }

    double frame_time_ms = now_ms();
    if (sensor_ts_ns > 0) {
        if (!sensor_map_ready_) {
            sensor_map_ready_ = true;
            sensor_ts_first_ns_ = sensor_ts_ns;
            sensor_raw_base_ns_ = clock_ns(CLOCK_MONOTONIC_RAW);
        }
        const int64_t relative_ns = sensor_ts_ns - sensor_ts_first_ns_;
        const int64_t exposure_ns = static_cast<int64_t>(exposure_us) * 1000LL;
        frame_time_ms = static_cast<double>(sensor_raw_base_ns_ + relative_ns + (exposure_ns / 2) - monotonic_raw_origin_ns()) / 1e6;
    }

    out.frame_time_ms = frame_time_ms;
    out.sensor_ts_ns = sensor_ts_ns;
    out.exposure_us = exposure_us;
    out.frame_duration_us = config_.frame_duration_us;
    out.frame_index = frame_index_++;

    request->reuse(libcamera::Request::ReuseBuffers);
    camera_->queueRequest(request);
    return !out.image.empty();
}

bool LibcameraCapture::map_buffer(const libcamera::FrameBuffer* buffer, std::string* error) {
    MappedBuffer mapped_buffer;
    for (const libcamera::FrameBuffer::Plane& plane : buffer->planes()) {
        void* mem = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), 0);
        if (mem == MAP_FAILED) {
            if (error) *error = "mmap failed for libcamera frame buffer";
            return false;
        }
        mapped_buffer.data.push_back(mem);
        mapped_buffer.lengths.push_back(plane.length);
    }
    mapped_[buffer] = mapped_buffer;
    return true;
}

void LibcameraCapture::request_complete(libcamera::Request* request) {
    if (!request || request->status() == libcamera::Request::RequestCancelled) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    ready_queue_.push_back(request);
    condition_.notify_one();
}
