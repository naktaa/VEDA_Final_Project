#include "frame_jpeg_cache.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/opencv.hpp>

#include "stream_config.hpp"

void FrameJpegCache::update_bgr_frame(const unsigned char* data,
                                      std::size_t bytes,
                                      int width,
                                      int height) {
    if (!data || width <= 0 || height <= 0) {
        return;
    }

    const std::size_t expected_bytes = static_cast<std::size_t>(width) *
                                       static_cast<std::size_t>(height) * 3U;
    if (bytes < expected_bytes) {
        return;
    }

    // OpenCV Mat expects a mutable pointer, but we only read from the capture buffer here.
    cv::Mat frame(height, width, CV_8UC3, const_cast<unsigned char*>(data));
    const cv::Mat contiguous_source = frame.isContinuous() ? frame : frame.clone();

    cv::Mat mjpeg_source;
    cv::rotate(contiguous_source, mjpeg_source, cv::ROTATE_180);

    cv::Mat encoded_source = mjpeg_source;
    const float width_scale =
        static_cast<float>(stream_config::DEFAULT_MJPEG_WIDTH) /
        static_cast<float>(encoded_source.cols);
    const float height_scale =
        static_cast<float>(stream_config::DEFAULT_MJPEG_HEIGHT) /
        static_cast<float>(encoded_source.rows);
    const float scale = std::min(1.0f, std::min(width_scale, height_scale));
    if (scale < 1.0f) {
        const int target_width = std::max(1, static_cast<int>(std::lround(encoded_source.cols * scale)));
        const int target_height = std::max(1, static_cast<int>(std::lround(encoded_source.rows * scale)));
        cv::resize(mjpeg_source,
                   encoded_source,
                   cv::Size(target_width, target_height),
                   0.0,
                   0.0,
                   cv::INTER_AREA);
    }

    std::vector<int> encode_params = {
        cv::IMWRITE_JPEG_QUALITY,
        stream_config::DEFAULT_MJPEG_QUALITY,
    };
    std::vector<unsigned char> encoded;
    if (!cv::imencode(".jpg", encoded_source, encoded, encode_params)) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_jpeg_ = std::move(encoded);
    ++frame_counter_;
}

bool FrameJpegCache::latest_jpeg(std::vector<unsigned char>& out, uint64_t* frame_counter) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_jpeg_.empty()) {
        return false;
    }

    out = latest_jpeg_;
    if (frame_counter) {
        *frame_counter = frame_counter_;
    }
    return true;
}

void FrameJpegCache::add_consumer() {
    consumer_count_.fetch_add(1);
}

void FrameJpegCache::remove_consumer() {
    int current = consumer_count_.load();
    while (current > 0) {
        if (consumer_count_.compare_exchange_weak(current, current - 1)) {
            return;
        }
    }
    consumer_count_.store(0);
}

bool FrameJpegCache::has_consumers() const {
    return consumer_count_.load() > 0;
}
