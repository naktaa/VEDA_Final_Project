#include "libcamera_grabber.hpp"

#include <cstdio>

#include "eis_globals.hpp"

bool LibcameraGrabber::init() {
    cm_ = std::make_unique<libcamera::CameraManager>();
    if (cm_->start() != 0) {
        fprintf(stderr, "[LC] CameraManager start failed\n");
        return false;
    }
    if (cm_->cameras().empty()) {
        fprintf(stderr, "[LC] No cameras found\n");
        return false;
    }

    camera_ = cm_->cameras()[0];
    if (camera_->acquire() != 0) {
        fprintf(stderr, "[LC] Camera acquire failed\n");
        return false;
    }

    config_ = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if (!config_) {
        fprintf(stderr, "[LC] Failed to generate configuration\n");
        return false;
    }

    libcamera::StreamConfiguration &cfg = config_->at(0);
    cfg.size.width = G_WIDTH;
    cfg.size.height = G_HEIGHT;
    cfg.pixelFormat = libcamera::formats::XBGR8888;
    cfg.bufferCount = 4;

    if (config_->validate() == libcamera::CameraConfiguration::Invalid) {
        fprintf(stderr, "[LC] Configuration invalid\n");
        return false;
    }

    if (camera_->configure(config_.get()) != 0) {
        fprintf(stderr, "[LC] Camera configure failed\n");
        return false;
    }

    stream_ = cfg.stream();
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    if (allocator_->allocate(stream_) < 0) {
        fprintf(stderr, "[LC] Buffer allocation failed\n");
        return false;
    }

    for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator_->buffers(stream_)) {
        if (!map_buffer(buffer.get())) {
            fprintf(stderr, "[LC] Buffer mmap failed\n");
            return false;
        }
    }

    for (const std::unique_ptr<libcamera::FrameBuffer> &buffer : allocator_->buffers(stream_)) {
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (!request) {
            fprintf(stderr, "[LC] Create request failed\n");
            return false;
        }
        if (request->addBuffer(stream_, buffer.get()) != 0) {
            fprintf(stderr, "[LC] Add buffer to request failed\n");
            return false;
        }
        requests_.push_back(std::move(request));
    }

    camera_->requestCompleted.connect(this, &LibcameraGrabber::request_complete);

    libcamera::ControlList ctrls(camera_->controls());
    {
        int64_t limits[2] = {TARGET_FRAME_DURATION_US, TARGET_FRAME_DURATION_US};
        ctrls.set(libcamera::controls::FrameDurationLimits,
                  libcamera::Span<const int64_t, 2>(limits));
        ctrls.set(libcamera::controls::AeEnable, false);
        ctrls.set(libcamera::controls::ExposureTime, (int)TARGET_EXPOSURE_US);
    }

    if (camera_->start(&ctrls) != 0) {
        fprintf(stderr, "[LC] Camera start failed\n");
        return false;
    }
    fprintf(stderr, "[LC] Controls set: FrameDuration=%lldus Exposure=%lldus AE=off\n",
            (long long)TARGET_FRAME_DURATION_US, (long long)TARGET_EXPOSURE_US);

    for (auto &req : requests_) {
        if (camera_->queueRequest(req.get()) < 0) {
            fprintf(stderr, "[LC] Queue request failed\n");
            return false;
        }
    }

    started_ = true;
    fprintf(stderr, "[LC] libcamera capture started\n");
    return true;
}

void LibcameraGrabber::shutdown() {
    if (!started_) return;
    started_ = false;
    if (camera_) {
        camera_->stop();
        camera_->requestCompleted.disconnect(this, &LibcameraGrabber::request_complete);
        camera_->release();
    }
    for (auto &kv : mapped_) {
        for (size_t i = 0; i < kv.second.data.size(); ++i) {
            munmap(kv.second.data[i], kv.second.lengths[i]);
        }
    }
    mapped_.clear();
    allocator_.reset();
    config_.reset();
    camera_.reset();
    if (cm_) cm_->stop();
    cm_.reset();
}

bool LibcameraGrabber::get_frame(cv::Mat &out, double &out_time_ms, TsSource &out_src,
                                 int64_t &out_sensor_ts_ns, int &out_exp_us, uint64_t &out_index) {
    out_src = TsSource::ARRIVAL;
    out_time_ms = now_ms();
    out_sensor_ts_ns = 0;
    out_exp_us = 0;

    libcamera::Request *request = nullptr;
    {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [&] { return !queue_.empty() || !g_running; });
        if (!g_running) return false;
        request = queue_.front();
        queue_.pop_front();
    }

    const auto &buffers = request->buffers();
    auto it = buffers.find(stream_);
    if (it == buffers.end()) {
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        return false;
    }
    libcamera::FrameBuffer *buffer = it->second;
    auto map_it = mapped_.find(buffer);
    if (map_it == mapped_.end()) {
        request->reuse(libcamera::Request::ReuseBuffers);
        camera_->queueRequest(request);
        return false;
    }

    const libcamera::StreamConfiguration &cfg = config_->at(0);
    uint8_t *data = static_cast<uint8_t *>(map_it->second.data[0]);
    size_t stride = cfg.stride;

    cv::Mat frame((int)cfg.size.height, (int)cfg.size.width, CV_8UC4, data, stride);
    out = frame.clone();

    const libcamera::ControlList &meta = request->metadata();
    if (meta.contains(libcamera::controls::SensorTimestamp.id())) {
        auto ts = meta.get(libcamera::controls::SensorTimestamp);
        if (ts) out_sensor_ts_ns = *ts;
    }
    if (meta.contains(libcamera::controls::ExposureTime.id())) {
        auto exp = meta.get(libcamera::controls::ExposureTime);
        if (exp) out_exp_us = *exp;
    }

    if (out_sensor_ts_ns > 0) {
        if (!sensor_map_ready_) {
            sensor_map_ready_ = true;
            sensor_ts_first_ns_ = out_sensor_ts_ns;
            sensor_raw_base_ns_ = clock_ns(CLOCK_MONOTONIC_RAW);
        }
        int64_t rel_ns = out_sensor_ts_ns - sensor_ts_first_ns_;
        int64_t exposure_ns = (int64_t)out_exp_us * 1000LL;
        out_time_ms = (sensor_raw_base_ns_ + rel_ns + exposure_ns / 2 - g_time_origin_raw_ns) / 1e6;
        out_src = TsSource::SENSOR;
    }

    out_index = frame_index_++;

    request->reuse(libcamera::Request::ReuseBuffers);
    camera_->queueRequest(request);
    return !out.empty();
}

LibcameraGrabber::~LibcameraGrabber() { shutdown(); }

bool LibcameraGrabber::map_buffer(const libcamera::FrameBuffer *buffer) {
    MappedBuffer mb;
    for (const libcamera::FrameBuffer::Plane &plane : buffer->planes()) {
        void *mem = mmap(nullptr, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), 0);
        if (mem == MAP_FAILED) {
            return false;
        }
        mb.data.push_back(mem);
        mb.lengths.push_back(plane.length);
    }
    mapped_[buffer] = mb;
    return true;
}

void LibcameraGrabber::request_complete(libcamera::Request *request) {
    if (request->status() == libcamera::Request::RequestCancelled) return;
    std::lock_guard<std::mutex> lk(mtx_);
    queue_.push_back(request);
    cv_.notify_one();
}

