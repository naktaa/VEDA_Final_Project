#include "imu_reader.hpp"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <wiringPi.h>

#include "timebase.hpp"

namespace {

constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_FIFO_EN = 0x23;
constexpr uint8_t REG_INT_PIN_CFG = 0x37;
constexpr uint8_t REG_INT_ENABLE = 0x38;
constexpr uint8_t REG_INT_STATUS = 0x3A;
constexpr uint8_t REG_GYRO_XOUT_H = 0x43;
constexpr uint8_t REG_USER_CTRL = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_FIFO_COUNTH = 0x72;
constexpr uint8_t REG_FIFO_R_W = 0x74;

constexpr uint8_t USERCTRL_FIFO_EN = 0x40;
constexpr uint8_t USERCTRL_FIFO_RESET = 0x04;
constexpr uint8_t INTPINCFG_LATCH_INT_EN = 0x20;

int16_t to_i16(uint8_t high, uint8_t low) {
    return static_cast<int16_t>((high << 8) | low);
}

double pick_axis(int axis, double x, double y, double z) {
    switch (axis) {
    case 0: return x;
    case 1: return y;
    case 2: return z;
    default: return z;
    }
}

bool i2c_write_byte(int fd, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    return write(fd, buffer, 2) == 2;
}

bool i2c_read_bytes(int fd, uint8_t reg, uint8_t* data, int length) {
    if (write(fd, &reg, 1) != 1) {
        return false;
    }
    return read(fd, data, length) == length;
}

std::string i2c_device_path(int bus) {
    return "/dev/i2c-" + std::to_string(bus);
}

} // namespace

ImuReader::ImuReader(size_t max_samples)
    : buffer_(max_samples) {}

ImuReader::~ImuReader() {
    stop();
}

bool ImuReader::start(const ImuConfig& imu_config,
                      const CalibrationConfig& calib_config,
                      bool refine_bias,
                      std::string* error) {
    stop();

    config_ = imu_config;
    bias_counts_ = cv::Vec3d(calib_config.bias_x, calib_config.bias_y, calib_config.bias_z);

    if (!open_device(error)) {
        return false;
    }
    if (!configure_device(error)) {
        close(fd_);
        fd_ = -1;
        return false;
    }

    use_interrupt_ = config_.use_fifo && config_.int_pin_wpi >= 0;
    if (use_interrupt_) {
        if (wiringPiSetup() == -1) {
            if (error) *error = "wiringPiSetup failed for IMU interrupt";
            close(fd_);
            fd_ = -1;
            return false;
        }
        pinMode(config_.int_pin_wpi, INPUT);
        pullUpDnControl(config_.int_pin_wpi, PUD_DOWN);
    }

    if (refine_bias && !warmup_refine_bias()) {
        if (error) *error = "IMU warm-up bias refinement failed";
        close(fd_);
        fd_ = -1;
        return false;
    }

    running_ = true;
    ready_ = true;
    worker_ = std::thread(&ImuReader::run, this);
    return true;
}

void ImuReader::stop() {
    running_ = false;
    ready_ = false;
    if (worker_.joinable()) {
        worker_.join();
    }
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

bool ImuReader::latest_sample(ImuSample& out) const {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    if (!ready_.load() && latest_sample_.sample_time_ms <= 0.0) {
        return false;
    }
    out = latest_sample_;
    return out.sample_time_ms > 0.0;
}

cv::Vec3d ImuReader::bias_counts() const {
    return bias_counts_;
}

bool ImuReader::collect_stationary_bias(const ImuConfig& config,
                                        int sample_count,
                                        int sleep_us,
                                        cv::Vec3d& out_bias_counts,
                                        std::string* error) {
    out_bias_counts = cv::Vec3d(0.0, 0.0, 0.0);

    const std::string path = i2c_device_path(config.bus);
    const int fd = open(path.c_str(), O_RDWR);
    if (fd < 0) {
        if (error) *error = "failed to open " + path + ": " + std::strerror(errno);
        return false;
    }
    if (ioctl(fd, I2C_SLAVE, config.addr) < 0) {
        if (error) *error = "failed to set I2C slave address";
        close(fd);
        return false;
    }

    auto cleanup = [&]() {
        close(fd);
    };

    if (!i2c_write_byte(fd, REG_PWR_MGMT_1, 0x00) ||
        !i2c_write_byte(fd, REG_CONFIG, 0x03) ||
        !i2c_write_byte(fd, REG_SMPLRT_DIV, static_cast<uint8_t>(std::max(0, (1000 / std::max(1, config.target_hz)) - 1))) ||
        !i2c_write_byte(fd, REG_GYRO_CONFIG, 0x00)) {
        if (error) *error = "failed to configure MPU-6050 for bias capture";
        cleanup();
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int valid = 0;
    for (int i = 0; i < sample_count; ++i) {
        uint8_t bytes[6];
        if (i2c_read_bytes(fd, REG_GYRO_XOUT_H, bytes, 6)) {
            out_bias_counts[0] += static_cast<double>(to_i16(bytes[0], bytes[1]));
            out_bias_counts[1] += static_cast<double>(to_i16(bytes[2], bytes[3]));
            out_bias_counts[2] += static_cast<double>(to_i16(bytes[4], bytes[5]));
            ++valid;
        }
        usleep(sleep_us);
    }

    cleanup();
    if (valid <= 0) {
        if (error) *error = "no valid gyro samples collected during bias capture";
        return false;
    }

    out_bias_counts *= (1.0 / static_cast<double>(valid));
    return true;
}

bool ImuReader::open_device(std::string* error) {
    const std::string path = i2c_device_path(config_.bus);
    fd_ = open(path.c_str(), O_RDWR);
    if (fd_ < 0) {
        if (error) *error = "failed to open " + path + ": " + std::strerror(errno);
        return false;
    }
    if (ioctl(fd_, I2C_SLAVE, config_.addr) < 0) {
        if (error) *error = "failed to set IMU I2C address";
        close(fd_);
        fd_ = -1;
        return false;
    }
    return true;
}

bool ImuReader::configure_device(std::string* error) {
    const int sample_div = std::max(0, (1000 / std::max(1, config_.target_hz)) - 1);
    if (!i2c_write_byte(fd_, REG_PWR_MGMT_1, 0x00) ||
        !i2c_write_byte(fd_, REG_CONFIG, 0x03) ||
        !i2c_write_byte(fd_, REG_SMPLRT_DIV, static_cast<uint8_t>(sample_div)) ||
        !i2c_write_byte(fd_, REG_GYRO_CONFIG, 0x00)) {
        if (error) *error = "failed to configure gyro sample registers";
        return false;
    }

    if (config_.use_fifo) {
        if (!i2c_write_byte(fd_, REG_USER_CTRL, USERCTRL_FIFO_RESET) ||
            !i2c_write_byte(fd_, REG_USER_CTRL, USERCTRL_FIFO_EN) ||
            !i2c_write_byte(fd_, REG_FIFO_EN, 0x70) ||
            !i2c_write_byte(fd_, REG_INT_PIN_CFG, config_.int_pin_wpi >= 0 ? INTPINCFG_LATCH_INT_EN : 0x00) ||
            !i2c_write_byte(fd_, REG_INT_ENABLE, config_.int_pin_wpi >= 0 ? 0x01 : 0x00)) {
            if (error) *error = "failed to configure FIFO/interrupt mode";
            return false;
        }
    } else {
        if (!i2c_write_byte(fd_, REG_USER_CTRL, 0x00) ||
            !i2c_write_byte(fd_, REG_FIFO_EN, 0x00) ||
            !i2c_write_byte(fd_, REG_INT_PIN_CFG, 0x00) ||
            !i2c_write_byte(fd_, REG_INT_ENABLE, 0x00)) {
            if (error) *error = "failed to configure polling mode";
            return false;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

bool ImuReader::read_raw_rates(cv::Vec3d& raw_counts) {
    uint8_t bytes[6];
    if (!i2c_read_bytes(fd_, REG_GYRO_XOUT_H, bytes, 6)) {
        return false;
    }
    raw_counts[0] = static_cast<double>(to_i16(bytes[0], bytes[1]));
    raw_counts[1] = static_cast<double>(to_i16(bytes[2], bytes[3]));
    raw_counts[2] = static_cast<double>(to_i16(bytes[4], bytes[5]));
    return true;
}

bool ImuReader::read_fifo_samples(std::vector<cv::Vec3d>& raw_samples) {
    raw_samples.clear();

    uint8_t count_bytes[2];
    if (!i2c_read_bytes(fd_, REG_FIFO_COUNTH, count_bytes, 2)) {
        return false;
    }
    int fifo_count = (static_cast<int>(count_bytes[0]) << 8) | static_cast<int>(count_bytes[1]);
    if (fifo_count <= 0) {
        return false;
    }

    if (fifo_count >= 1024) {
        i2c_write_byte(fd_, REG_USER_CTRL, USERCTRL_FIFO_RESET);
        i2c_write_byte(fd_, REG_USER_CTRL, USERCTRL_FIFO_EN);
        return false;
    }

    fifo_count -= fifo_count % 6;
    if (fifo_count <= 0) {
        return false;
    }

    std::vector<uint8_t> bytes(static_cast<size_t>(fifo_count));
    constexpr int kReadChunkBytes = 240;
    for (int offset = 0; offset < fifo_count; offset += kReadChunkBytes) {
        const int chunk_bytes = std::min(kReadChunkBytes, fifo_count - offset);
        if (!i2c_read_bytes(fd_, REG_FIFO_R_W, bytes.data() + offset, chunk_bytes)) {
            return false;
        }
    }

    raw_samples.reserve(static_cast<size_t>(fifo_count / 6));
    for (int i = 0; i + 5 < fifo_count; i += 6) {
        raw_samples.emplace_back(
            static_cast<double>(to_i16(bytes[static_cast<size_t>(i + 0)], bytes[static_cast<size_t>(i + 1)])),
            static_cast<double>(to_i16(bytes[static_cast<size_t>(i + 2)], bytes[static_cast<size_t>(i + 3)])),
            static_cast<double>(to_i16(bytes[static_cast<size_t>(i + 4)], bytes[static_cast<size_t>(i + 5)]))
        );
    }
    return !raw_samples.empty();
}

bool ImuReader::warmup_refine_bias() {
    cv::Vec3d measured;
    std::string error;
    if (!collect_stationary_bias(config_, 180, 2000, measured, &error)) {
        std::fprintf(stderr, "[IMU] bias warm-up failed: %s\n", error.c_str());
        return false;
    }

    const bool seeded = std::abs(bias_counts_[0]) > 1e-6 ||
                        std::abs(bias_counts_[1]) > 1e-6 ||
                        std::abs(bias_counts_[2]) > 1e-6;
    if (seeded) {
        bias_counts_ = bias_counts_ * 0.75 + measured * 0.25;
    } else {
        bias_counts_ = measured;
    }
    std::fprintf(stderr, "[IMU] bias counts warm-up: gx=%.2f gy=%.2f gz=%.2f\n",
                 bias_counts_[0], bias_counts_[1], bias_counts_[2]);
    return true;
}

ImuSample ImuReader::make_sample(const cv::Vec3d& raw_counts, double sample_time_ms) const {
    const cv::Vec3d corrected = raw_counts - bias_counts_;
    const cv::Vec3d rad_s = (corrected / config_.gyro_sensitivity) * (CV_PI / 180.0);

    ImuSample sample;
    sample.sample_time_ms = sample_time_ms;
    sample.raw_counts = raw_counts;
    sample.gyro_rad_s = cv::Vec3d(
        config_.sign_roll * pick_axis(config_.axis_roll, rad_s[0], rad_s[1], rad_s[2]),
        config_.sign_pitch * pick_axis(config_.axis_pitch, rad_s[0], rad_s[1], rad_s[2]),
        config_.sign_yaw * pick_axis(config_.axis_yaw, rad_s[0], rad_s[1], rad_s[2]));
    return sample;
}

void ImuReader::run() {
    const double period_ms = 1000.0 / std::max(1, config_.target_hz);
    int64_t last_time_ns = clock_ns(CLOCK_MONOTONIC_RAW);

    while (running_) {
        std::vector<cv::Vec3d> fifo_samples;
        bool have_samples = false;
        int64_t event_ns = 0;

        if (use_interrupt_) {
            const int wait_rc = waitForInterrupt(config_.int_pin_wpi, 200);
            if (wait_rc < 0) {
                std::fprintf(stderr, "[IMU] waitForInterrupt failed\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            if (wait_rc == 0) {
                continue;
            }
            event_ns = clock_ns(CLOCK_MONOTONIC_RAW);
            uint8_t status = 0;
            i2c_read_bytes(fd_, REG_INT_STATUS, &status, 1);
            have_samples = read_fifo_samples(fifo_samples);
        } else if (config_.use_fifo) {
            std::this_thread::sleep_for(std::chrono::milliseconds(std::max(1, 1000 / std::max(1, config_.target_hz / 4))));
            event_ns = clock_ns(CLOCK_MONOTONIC_RAW);
            have_samples = read_fifo_samples(fifo_samples);
        } else {
            cv::Vec3d raw_counts;
            event_ns = clock_ns(CLOCK_MONOTONIC_RAW);
            have_samples = read_raw_rates(raw_counts);
            if (have_samples) {
                fifo_samples.push_back(raw_counts);
            }

            const int64_t elapsed_ns = clock_ns(CLOCK_MONOTONIC_RAW) - event_ns;
            const int64_t target_period_ns = static_cast<int64_t>(1000000000.0 / std::max(1, config_.target_hz));
            const int64_t sleep_ns = target_period_ns - elapsed_ns;
            if (sleep_ns > 0) {
                const auto sleep_duration = std::chrono::nanoseconds(sleep_ns);
                std::this_thread::sleep_for(sleep_duration);
            }
        }

        if (!have_samples || fifo_samples.empty()) {
            continue;
        }

        const double end_time_ms = static_cast<double>(event_ns - monotonic_raw_origin_ns()) / 1e6;
        for (size_t i = 0; i < fifo_samples.size(); ++i) {
            const size_t reverse_index = fifo_samples.size() - 1 - i;
            const double sample_time_ms = end_time_ms - (static_cast<double>(i) * period_ms);
            ImuSample sample = make_sample(fifo_samples[reverse_index], sample_time_ms);
            buffer_.push(sample);
            {
                std::lock_guard<std::mutex> lock(sample_mutex_);
                latest_sample_ = sample;
            }
        }

        const int64_t now_ns = clock_ns(CLOCK_MONOTONIC_RAW);
        const double dt_sec = static_cast<double>(now_ns - last_time_ns) / 1e9;
        last_time_ns = now_ns;
        if (dt_sec > 0.0 && dt_sec < 1.0) {
            const double hz = static_cast<double>(fifo_samples.size()) / dt_sec;
            const double previous = actual_hz_.load();
            actual_hz_ = previous <= 0.0 ? hz : (previous * 0.9 + hz * 0.1);
        }
    }
}
