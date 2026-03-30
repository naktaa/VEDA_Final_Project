#pragma once

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <vector>

class FrameJpegCache {
public:
    void update_bgr_frame(const unsigned char* data,
                          std::size_t bytes,
                          int width,
                          int height,
                          bool swap_rb = false);
    bool latest_jpeg(std::vector<unsigned char>& out, uint64_t* frame_counter = nullptr) const;
    void add_consumer();
    void remove_consumer();
    bool has_consumers() const;

private:
    std::atomic<int> consumer_count_{0};
    mutable std::mutex mutex_;
    std::vector<unsigned char> latest_jpeg_;
    uint64_t frame_counter_ = 0;
};
