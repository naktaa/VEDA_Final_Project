#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

#include "app_config.hpp"
#include "app_types.hpp"
#include "gpio_line.hpp"
#include "gyro_eis.hpp"

class ImuReader {
public:
    explicit ImuReader(size_t max_samples = 2500);
    ~ImuReader();

    bool start(const ImuConfig& imu_config,
               const CalibrationConfig& calib_config,
               bool refine_bias,
               std::string* error = nullptr);
    void stop();

    bool ready() const { return ready_.load(); }
    double actual_hz() const { return actual_hz_.load(); }
    const GyroBuffer& buffer() const { return buffer_; }
    GyroBuffer& buffer() { return buffer_; }
    bool latest_sample(ImuSample& out) const;
    cv::Vec3d bias_counts() const;

    static bool collect_stationary_bias(const ImuConfig& config,
                                        int sample_count,
                                        int sleep_us,
                                        cv::Vec3d& out_bias_counts,
                                        std::string* error = nullptr);

private:
    bool open_device(std::string* error);
    bool configure_device(std::string* error);
    bool read_raw_rates(cv::Vec3d& raw_counts);
    bool read_fifo_samples(std::vector<cv::Vec3d>& raw_samples);
    bool warmup_refine_bias();
    ImuSample make_sample(const cv::Vec3d& raw_counts, double sample_time_ms) const;
    void run();

    ImuConfig config_;
    int fd_ = -1;
    bool use_interrupt_ = false;
    int resolved_int_line_offset_ = -1;
    std::string resolved_int_gpio_chip_;

    std::atomic<bool> running_{false};
    std::atomic<bool> ready_{false};
    std::atomic<double> actual_hz_{0.0};
    std::thread worker_;
    GpioEdgeLine irq_line_;

    mutable std::mutex sample_mutex_;
    ImuSample latest_sample_;
    cv::Vec3d bias_counts_ = {0.0, 0.0, 0.0};
    GyroBuffer buffer_;
};
