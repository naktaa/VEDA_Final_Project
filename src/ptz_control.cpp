#include "ptz_control.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

namespace {

constexpr float kImuAlpha = 0.2f;
constexpr float kImuMaxDeg = 45.0f;
constexpr int kPanId = 1;
constexpr int kTiltId = 2;
constexpr int kPanCenter = 1800;
constexpr int kPanLeft = 2650;
constexpr int kPanRight = 1150;
constexpr int kTiltCenter = 2400;
constexpr int kTiltUp = 2900;
constexpr int kTiltDown = 2200;
constexpr int kUpdateHz = 50;
constexpr int kUpdateMs = 1000 / kUpdateHz;
constexpr int kMaxStepPerTick = 20;

speed_t to_baud_constant(int baud) {
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    default:
        return B115200;
    }
}

float clampf(float value, float lo, float hi) {
    return std::max(lo, std::min(value, hi));
}

int lerp_int(int a, int b, float t) {
    return static_cast<int>(std::lround(static_cast<float>(a) + (b - a) * t));
}

int step_toward(int current, int target, int max_step) {
    if (current < target) {
        return std::min(current + max_step, target);
    }
    return std::max(current - max_step, target);
}

class ServoSerial {
public:
    ServoSerial() = default;

    bool open_port(const std::string& device, int baud) {
        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            std::fprintf(stderr,
                         "[PTZ] Failed to open serial %s: %s\n",
                         device.c_str(),
                         std::strerror(errno));
            return false;
        }

        struct termios tty {};
        if (tcgetattr(fd_, &tty) != 0) {
            std::fprintf(stderr, "[PTZ] tcgetattr failed: %s\n", std::strerror(errno));
            close(fd_);
            fd_ = -1;
            return false;
        }

        cfsetospeed(&tty, to_baud_constant(baud));
        cfsetispeed(&tty, to_baud_constant(baud));

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::fprintf(stderr, "[PTZ] tcsetattr failed: %s\n", std::strerror(errno));
            close(fd_);
            fd_ = -1;
            return false;
        }

        std::fprintf(stderr, "[PTZ] serial ready: %s @ %d\n", device.c_str(), baud);
        return true;
    }

    ~ServoSerial() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    static int clamp_pan(int value) {
        return std::max(600, std::min(3600, value));
    }

    static int clamp_tilt(int value) {
        return std::max(1300, std::min(4095, value));
    }

    bool control_double(uint8_t index1, int angle1, uint8_t index2, int angle2) {
        if (fd_ < 0) {
            return false;
        }

        angle1 = clamp_pan(angle1);
        angle2 = clamp_tilt(angle2);

        const uint8_t packet[] = {
            0xFF, 0xFF, 0xFE, 0x0E, 0x83, 0x2A, 0x04,
            index1,
            static_cast<uint8_t>((angle1 >> 8) & 0xFF),
            static_cast<uint8_t>(angle1 & 0xFF),
            0x00, 0x0A,
            index2,
            static_cast<uint8_t>((angle2 >> 8) & 0xFF),
            static_cast<uint8_t>(angle2 & 0xFF),
            0x00, 0x0A,
            static_cast<uint8_t>(~(0xFE + 0x0E + 0x83 + 0x2A + 0x04 +
                                   index1 +
                                   static_cast<uint8_t>((angle1 >> 8) & 0xFF) +
                                   static_cast<uint8_t>(angle1 & 0xFF) +
                                   0x00 + 0x0A +
                                   index2 +
                                   static_cast<uint8_t>((angle2 >> 8) & 0xFF) +
                                   static_cast<uint8_t>(angle2 & 0xFF) +
                                   0x00 + 0x0A))
        };

        const ssize_t written = write(fd_, packet, sizeof(packet));
        tcdrain(fd_);
        if (written != static_cast<ssize_t>(sizeof(packet))) {
            std::fprintf(stderr, "[PTZ] serial write failed: %s\n", std::strerror(errno));
            return false;
        }
        return true;
    }

private:
    int fd_ = -1;
};

int map_pan_from_roll(float roll) {
    const float clamped = clampf(roll, -kImuMaxDeg, kImuMaxDeg);
    if (clamped >= 0.0f) {
        const float t = clamped / kImuMaxDeg;
        return lerp_int(kPanCenter, kPanRight, t);
    }
    const float t = (-clamped) / kImuMaxDeg;
    return lerp_int(kPanCenter, kPanLeft, t);
}

int map_tilt_from_pitch(float pitch) {
    const float clamped = clampf(pitch, -kImuMaxDeg, kImuMaxDeg);
    if (clamped >= 0.0f) {
        const float t = clamped / kImuMaxDeg;
        return lerp_int(kTiltCenter, kTiltDown, t);
    }
    const float t = (-clamped) / kImuMaxDeg;
    return lerp_int(kTiltCenter, kTiltUp, t);
}

} // namespace

struct PtzController::Impl {
    PtzConfig config;
    std::unique_ptr<ServoSerial> servo;
    std::atomic<bool> running{false};
    std::atomic<bool> servo_ready{false};
    std::thread worker;

    mutable std::mutex mutex;
    float filtered_pitch = 0.0f;
    float filtered_roll = 0.0f;
    float last_pitch = 0.0f;
    float last_roll = 0.0f;
    float last_yaw = 0.0f;
    uint64_t last_client_timestamp_ms = 0;
    int current_pan = kPanCenter;
    int current_tilt = kTiltCenter;
    int imu_target_pan = kPanCenter;
    int imu_target_tilt = kTiltCenter;
    bool pan_left_active = false;
    bool pan_right_active = false;
    bool tilt_up_active = false;
    bool tilt_down_active = false;
    std::string active_source = "hold";
    std::chrono::steady_clock::time_point last_imu_arrival{};

    void run() {
        int current_pan_local = kPanCenter;
        int current_tilt_local = kTiltCenter;
        auto next_tick = std::chrono::steady_clock::now();

        while (running.load()) {
            int desired_pan = current_pan_local;
            int desired_tilt = current_tilt_local;
            std::string source = "hold";

            {
                std::lock_guard<std::mutex> lock(mutex);
                const auto now = std::chrono::steady_clock::now();
                const bool imu_recent =
                    last_imu_arrival.time_since_epoch().count() != 0 &&
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_imu_arrival).count() <=
                        config.imu_timeout_ms;

                if (imu_recent) {
                    desired_pan = imu_target_pan;
                    desired_tilt = imu_target_tilt;
                    source = "imu";
                } else {
                    const bool pan_manual = pan_left_active != pan_right_active;
                    const bool tilt_manual = tilt_up_active != tilt_down_active;

                    if (pan_left_active && !pan_right_active) {
                        desired_pan = ServoSerial::clamp_pan(current_pan_local + kMaxStepPerTick);
                        source = "mqtt";
                    } else if (pan_right_active && !pan_left_active) {
                        desired_pan = ServoSerial::clamp_pan(current_pan_local - kMaxStepPerTick);
                        source = "mqtt";
                    }

                    if (tilt_up_active && !tilt_down_active) {
                        desired_tilt = ServoSerial::clamp_tilt(current_tilt_local + kMaxStepPerTick);
                        source = "mqtt";
                    } else if (tilt_down_active && !tilt_up_active) {
                        desired_tilt = ServoSerial::clamp_tilt(current_tilt_local - kMaxStepPerTick);
                        source = "mqtt";
                    }

                    if (!pan_manual && !tilt_manual) {
                        desired_pan = current_pan_local;
                        desired_tilt = current_tilt_local;
                        source = "hold";
                    }
                }
            }

            current_pan_local = step_toward(current_pan_local, desired_pan, kMaxStepPerTick);
            current_tilt_local = step_toward(current_tilt_local, desired_tilt, kMaxStepPerTick);

            if (servo_ready.load() && servo) {
                servo->control_double(kPanId, current_pan_local, kTiltId, current_tilt_local);
            }

            {
                std::lock_guard<std::mutex> lock(mutex);
                current_pan = current_pan_local;
                current_tilt = current_tilt_local;
                active_source = source;
            }

            next_tick += std::chrono::milliseconds(kUpdateMs);
            std::this_thread::sleep_until(next_tick);
        }
    }
};

PtzController::~PtzController() {
    stop();
}

bool PtzController::start(const PtzConfig& cfg) {
    if (impl_) {
        return true;
    }

    auto* impl = new Impl();
    impl->config = cfg;
    impl->servo = std::make_unique<ServoSerial>();

    impl->running.store(true);
    if (!impl->servo->open_port(cfg.serial_device, cfg.serial_baud)) {
        std::fprintf(stderr, "[PTZ] serial servo init failed; PTZ motion disabled.\n");
        impl->servo_ready.store(false);
    } else {
        impl->servo_ready.store(true);
        impl->servo->control_double(kPanId, kPanCenter, kTiltId, kTiltCenter);
    }

    impl->worker = std::thread([impl]() {
        impl->run();
    });

    impl_ = impl;
    return true;
}

void PtzController::stop() {
    if (!impl_) {
        return;
    }

    impl_->running.store(false);
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }

    if (impl_->servo_ready.load() && impl_->servo) {
        impl_->servo->control_double(kPanId, kPanCenter, kTiltId, kTiltCenter);
    }

    delete impl_;
    impl_ = nullptr;
}

void PtzController::handle_mqtt_command(const std::string& command, bool active) {
    if (!impl_) {
        return;
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (command == "pan_left") {
        impl_->pan_left_active = active;
    } else if (command == "pan_right") {
        impl_->pan_right_active = active;
    } else if (command == "tilt_up") {
        impl_->tilt_up_active = active;
    } else if (command == "tilt_down") {
        impl_->tilt_down_active = active;
    }
}

void PtzController::handle_imu(float pitch, float roll, float yaw, uint64_t client_timestamp_ms) {
    if (!impl_) {
        return;
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->last_pitch = pitch;
    impl_->last_roll = roll;
    impl_->last_yaw = yaw;
    impl_->last_client_timestamp_ms = client_timestamp_ms;

    // Current phone mount uses roll -> tilt, pitch -> pan.
    impl_->filtered_pitch = (1.0f - kImuAlpha) * impl_->filtered_pitch + kImuAlpha * roll;
    impl_->filtered_roll = (1.0f - kImuAlpha) * impl_->filtered_roll + kImuAlpha * pitch;
    impl_->imu_target_pan = map_pan_from_roll(impl_->filtered_roll);
    impl_->imu_target_tilt = map_tilt_from_pitch(impl_->filtered_pitch);
    impl_->last_imu_arrival = std::chrono::steady_clock::now();
}

PtzStatus PtzController::latest_status() const {
    PtzStatus status;
    if (!impl_) {
        return status;
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    status.servo_ready = impl_->servo_ready.load();
    status.pitch = impl_->last_pitch;
    status.roll = impl_->last_roll;
    status.yaw = impl_->last_yaw;
    status.client_timestamp_ms = impl_->last_client_timestamp_ms;
    status.pan = impl_->current_pan;
    status.tilt = impl_->current_tilt;
    status.active_source = impl_->active_source;
    return status;
}

bool PtzController::servo_ready() const {
    return impl_ && impl_->servo_ready.load();
}
