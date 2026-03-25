#include "ptz_control.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {

constexpr float kImuAlpha = 0.28f;
constexpr float kImuMaxDeg = 45.0f;
constexpr int kUpdateHz = 50;
constexpr int kUpdateMs = 1000 / kUpdateHz;
constexpr float kMaxStepPerTickDeg = 4.5f;
constexpr float kYawPanGain = 1.1f;
constexpr float kPanSensitivityScale = 0.8f;
constexpr float kTiltSensitivityScale = 0.9f;
constexpr float kVrPanLimitDeg = 60.0f;
constexpr float kVrTiltLimitDeg = 50.0f;
constexpr float kYawFilterTauSec = 0.12f;
constexpr float kDefaultImuDtSec = 0.02f;
constexpr float kMinImuDtSec = 0.005f;
constexpr float kMaxImuDtSec = 0.1f;
constexpr float kYawMaxDeltaPerSampleDeg = 18.0f;
constexpr float kPitchMaxDeltaPerSampleDeg = 20.0f;
constexpr float kRollMaxDeltaPerSampleDeg = 20.0f;
constexpr float kRejectPitchDeltaDeg = 85.0f;
constexpr float kRejectRollDeltaDeg = 85.0f;
constexpr float kRejectYawDeltaDeg = 100.0f;

float clampf(float value, float lo, float hi) {
    return std::max(lo, std::min(value, hi));
}

float lerpf(float a, float b, float t) {
    return a + (b - a) * t;
}

float step_toward(float current, float target, float max_step) {
    if (current < target) {
        return std::min(current + max_step, target);
    }
    return std::max(current - max_step, target);
}

float map_axis_from_imu(float value, float center, float negative_limit, float positive_limit) {
    const float clamped = clampf(value, -kImuMaxDeg, kImuMaxDeg);
    if (clamped >= 0.0f) {
        return lerpf(center, positive_limit, clamped / kImuMaxDeg);
    }
    return lerpf(center, negative_limit, (-clamped) / kImuMaxDeg);
}

float clamp_around_center(float value, float center, float max_delta) {
    return clampf(value, center - max_delta, center + max_delta);
}

float wrap_angle_180(float angle_deg) {
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float guard_delta(float current, float previous, float max_delta) {
    return previous + clampf(current - previous, -max_delta, max_delta);
}

const char* mode_to_cstr(PtzMode mode) {
    return mode == PtzMode::kVr ? "vr" : "manual";
}

class Pca9685Driver {
public:
    Pca9685Driver() = default;

    ~Pca9685Driver() {
        close_device();
    }

    bool open_device(const std::string& device, int address, int pwm_frequency_hz) {
        fd_ = open(device.c_str(), O_RDWR);
        if (fd_ < 0) {
            std::fprintf(stderr,
                         "[PTZ] Failed to open I2C device %s: %s\n",
                         device.c_str(),
                         std::strerror(errno));
            return false;
        }

        if (ioctl(fd_, I2C_SLAVE, address) < 0) {
            std::fprintf(stderr,
                         "[PTZ] Failed to set I2C addr 0x%02X: %s\n",
                         address,
                         std::strerror(errno));
            close_device();
            return false;
        }

        if (!write_reg(kMode1, 0x00)) {
            close_device();
            return false;
        }
        usleep(10000);

        if (!set_pwm_frequency(static_cast<double>(pwm_frequency_hz))) {
            close_device();
            return false;
        }

        std::fprintf(stderr,
                     "[PTZ] PCA9685 ready: %s addr=0x%02X freq=%dHz\n",
                     device.c_str(),
                     address,
                     pwm_frequency_hz);
        return true;
    }

    bool set_pan_tilt(int pan_channel, int tilt_channel, float pan_deg, float tilt_deg) {
        return set_servo_angle(pan_channel, pan_deg) &&
               set_servo_angle(tilt_channel, tilt_deg);
    }

private:
    static constexpr uint8_t kMode1 = 0x00;
    static constexpr uint8_t kPrescale = 0xFE;
    static constexpr uint8_t kLed0OnL = 0x06;
    static constexpr uint8_t kLed0OnH = 0x07;
    static constexpr uint8_t kLed0OffL = 0x08;
    static constexpr uint8_t kLed0OffH = 0x09;
    static constexpr double kServoMinUs = 500.0;
    static constexpr double kServoMaxUs = 2500.0;
    static constexpr double kFrameUs = 20000.0;
    static constexpr double kPwmResolution = 4096.0;

    int fd_ = -1;

    void close_device() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    bool write_reg(uint8_t reg, uint8_t value) {
        const uint8_t buf[2] = {reg, value};
        if (write(fd_, buf, sizeof(buf)) != static_cast<ssize_t>(sizeof(buf))) {
            std::fprintf(stderr, "[PTZ] I2C write reg 0x%02X failed: %s\n", reg, std::strerror(errno));
            return false;
        }
        return true;
    }

    bool read_reg(uint8_t reg, uint8_t& value) {
        if (write(fd_, &reg, 1) != 1) {
            std::fprintf(stderr, "[PTZ] I2C address write failed: %s\n", std::strerror(errno));
            return false;
        }
        if (read(fd_, &value, 1) != 1) {
            std::fprintf(stderr, "[PTZ] I2C read reg 0x%02X failed: %s\n", reg, std::strerror(errno));
            return false;
        }
        return true;
    }

    bool set_pwm_frequency(double freq_hz) {
        double prescale_value = 25000000.0 / kPwmResolution / freq_hz - 1.0;
        const auto prescale = static_cast<uint8_t>(std::floor(prescale_value + 0.5));

        uint8_t old_mode = 0;
        if (!read_reg(kMode1, old_mode)) {
            return false;
        }

        const uint8_t sleep_mode = static_cast<uint8_t>((old_mode & 0x7F) | 0x10);
        if (!write_reg(kMode1, sleep_mode)) {
            return false;
        }
        if (!write_reg(kPrescale, prescale)) {
            return false;
        }
        if (!write_reg(kMode1, old_mode)) {
            return false;
        }
        usleep(5000);
        if (!write_reg(kMode1, static_cast<uint8_t>(old_mode | 0xA1))) {
            return false;
        }
        usleep(5000);
        return true;
    }

    bool set_pwm(int channel, int on, int off) {
        if (channel < 0 || channel > 15) {
            return false;
        }
        return write_reg(static_cast<uint8_t>(kLed0OnL + 4 * channel), static_cast<uint8_t>(on & 0xFF)) &&
               write_reg(static_cast<uint8_t>(kLed0OnH + 4 * channel), static_cast<uint8_t>((on >> 8) & 0x0F)) &&
               write_reg(static_cast<uint8_t>(kLed0OffL + 4 * channel), static_cast<uint8_t>(off & 0xFF)) &&
               write_reg(static_cast<uint8_t>(kLed0OffH + 4 * channel), static_cast<uint8_t>((off >> 8) & 0x0F));
    }

    bool set_servo_angle(int channel, float angle_deg) {
        const double clamped = clampf(angle_deg, 0.0f, 180.0f);
        const double pulse_us =
            kServoMinUs + (clamped / 180.0) * (kServoMaxUs - kServoMinUs);
        int ticks = static_cast<int>(std::lround((pulse_us / kFrameUs) * kPwmResolution));
        ticks = std::max(0, std::min(4095, ticks));
        return set_pwm(channel, 0, ticks);
    }
};

} // namespace

struct PtzController::Impl {
    PtzConfig config;
    std::unique_ptr<Pca9685Driver> servo;
    std::atomic<bool> running{false};
    std::atomic<bool> servo_ready{false};
    std::thread worker;

    mutable std::mutex mutex;
    float filtered_pitch = 0.0f;
    float filtered_roll = 0.0f;
    float filtered_yaw = 0.0f;
    float yaw_unwrapped = 0.0f;
    float last_raw_yaw = 0.0f;
    bool yaw_initialized = false;
    float last_raw_pitch = 0.0f;
    float last_raw_roll = 0.0f;
    bool pitch_roll_initialized = false;
    float last_pitch = 0.0f;
    float last_roll = 0.0f;
    float last_yaw = 0.0f;
    uint64_t last_client_timestamp_ms = 0;
    float current_pan = 90.0f;
    float current_tilt = 90.0f;
    float imu_target_pan = 90.0f;
    float imu_target_tilt = 90.0f;
    bool pan_left_active = false;
    bool pan_right_active = false;
    bool tilt_up_active = false;
    bool tilt_down_active = false;
    PtzMode mode = PtzMode::kManual;
    std::string active_source = "hold";
    std::chrono::steady_clock::time_point last_imu_arrival{};

    void run() {
        float current_pan_local = config.pan_center_deg;
        float current_tilt_local = config.tilt_center_deg;
        auto next_tick = std::chrono::steady_clock::now();

        while (running.load()) {
            float desired_pan = current_pan_local;
            float desired_tilt = current_tilt_local;
            std::string source = "hold";

            {
                std::lock_guard<std::mutex> lock(mutex);
                const auto now = std::chrono::steady_clock::now();
                const bool imu_recent =
                    last_imu_arrival.time_since_epoch().count() != 0 &&
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_imu_arrival).count() <=
                        config.imu_timeout_ms;

                if (mode == PtzMode::kVr && imu_recent) {
                    desired_pan = imu_target_pan;
                    desired_tilt = imu_target_tilt;
                    source = "imu";
                } else if (mode == PtzMode::kVr && !imu_recent) {
                    mode = PtzMode::kManual;
                    source = "manual";
                    std::fprintf(stderr, "[PTZ] IMU timeout; fallback to manual mode.\n");
                }

                if (mode == PtzMode::kManual) {
                    const bool pan_manual = pan_left_active != pan_right_active;
                    const bool tilt_manual = tilt_up_active != tilt_down_active;

                    if (pan_left_active && !pan_right_active) {
                        desired_pan = config.pan_left_deg;
                        source = "mqtt";
                    } else if (pan_right_active && !pan_left_active) {
                        desired_pan = config.pan_right_deg;
                        source = "mqtt";
                    }

                    if (tilt_up_active && !tilt_down_active) {
                        desired_tilt = config.tilt_up_deg;
                        source = "mqtt";
                    } else if (tilt_down_active && !tilt_up_active) {
                        desired_tilt = config.tilt_down_deg;
                        source = "mqtt";
                    }

                    if (!pan_manual && !tilt_manual) {
                        desired_pan = current_pan_local;
                        desired_tilt = current_tilt_local;
                        source = "hold";
                    }
                }
            }

            current_pan_local = step_toward(current_pan_local, desired_pan, kMaxStepPerTickDeg);
            current_tilt_local = step_toward(current_tilt_local, desired_tilt, kMaxStepPerTickDeg);

            if (servo_ready.load() && servo) {
                servo->set_pan_tilt(config.pan_channel,
                                    config.tilt_channel,
                                    current_pan_local,
                                    current_tilt_local);
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
    impl->servo = std::make_unique<Pca9685Driver>();
    impl->current_pan = cfg.pan_center_deg;
    impl->current_tilt = cfg.tilt_center_deg;
    impl->imu_target_pan = cfg.pan_center_deg;
    impl->imu_target_tilt = cfg.tilt_center_deg;

    impl->running.store(true);
    if (!impl->servo->open_device(cfg.i2c_device, cfg.i2c_address, cfg.pwm_frequency_hz)) {
        std::fprintf(stderr, "[PTZ] SG90 init failed; PTZ motion disabled.\n");
        impl->servo_ready.store(false);
    } else {
        impl->servo_ready.store(true);
        impl->servo->set_pan_tilt(cfg.pan_channel,
                                  cfg.tilt_channel,
                                  cfg.pan_center_deg,
                                  cfg.tilt_center_deg);
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
        impl_->servo->set_pan_tilt(impl_->config.pan_channel,
                                   impl_->config.tilt_channel,
                                   impl_->config.pan_center_deg,
                                   impl_->config.tilt_center_deg);
    }

    delete impl_;
    impl_ = nullptr;
}

bool PtzController::set_mode(PtzMode mode) {
    if (!impl_) {
        return false;
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->mode = mode;
    impl_->pan_left_active = false;
    impl_->pan_right_active = false;
    impl_->tilt_up_active = false;
    impl_->tilt_down_active = false;
    if (mode == PtzMode::kManual) {
        impl_->last_imu_arrival = {};
        impl_->filtered_pitch = 0.0f;
        impl_->filtered_roll = 0.0f;
        impl_->filtered_yaw = 0.0f;
        impl_->yaw_unwrapped = 0.0f;
        impl_->last_raw_yaw = 0.0f;
        impl_->yaw_initialized = false;
        impl_->last_raw_pitch = 0.0f;
        impl_->last_raw_roll = 0.0f;
        impl_->pitch_roll_initialized = false;
        impl_->imu_target_pan = impl_->current_pan;
        impl_->imu_target_tilt = impl_->current_tilt;
    } else {
        impl_->last_imu_arrival = std::chrono::steady_clock::now();
        impl_->filtered_pitch = 0.0f;
        impl_->filtered_roll = 0.0f;
        impl_->filtered_yaw = 0.0f;
        impl_->yaw_unwrapped = 0.0f;
        impl_->last_raw_yaw = 0.0f;
        impl_->yaw_initialized = false;
        impl_->last_raw_pitch = 0.0f;
        impl_->last_raw_roll = 0.0f;
        impl_->pitch_roll_initialized = false;
    }
    std::fprintf(stderr, "[PTZ] mode -> %s\n", mode_to_cstr(mode));
    return true;
}

PtzMode PtzController::mode() const {
    if (!impl_) {
        return PtzMode::kManual;
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->mode;
}

void PtzController::handle_mqtt_command(const std::string& command, bool active) {
    if (!impl_) {
        return;
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->mode != PtzMode::kManual) {
        return;
    }
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
    if (impl_->mode != PtzMode::kVr) {
        return;
    }
    if (impl_->pitch_roll_initialized && impl_->yaw_initialized) {
        const float pitch_jump = std::fabs(wrap_angle_180(pitch - impl_->last_raw_pitch));
        const float roll_jump = std::fabs(wrap_angle_180(roll - impl_->last_raw_roll));
        const float yaw_jump = std::fabs(wrap_angle_180(yaw - impl_->last_raw_yaw));
        if (pitch_jump > kRejectPitchDeltaDeg ||
            roll_jump > kRejectRollDeltaDeg ||
            yaw_jump > kRejectYawDeltaDeg) {
            return;
        }
    }

    impl_->last_pitch = pitch;
    impl_->last_roll = roll;
    impl_->last_yaw = yaw;
    impl_->last_client_timestamp_ms = client_timestamp_ms;

    const auto now = std::chrono::steady_clock::now();
    float imu_dt_sec = kDefaultImuDtSec;
    if (impl_->last_imu_arrival.time_since_epoch().count() != 0) {
        imu_dt_sec =
            std::chrono::duration_cast<std::chrono::duration<float>>(now - impl_->last_imu_arrival).count();
        imu_dt_sec = clampf(imu_dt_sec, kMinImuDtSec, kMaxImuDtSec);
    }
    const float yaw_alpha = imu_dt_sec / (kYawFilterTauSec + imu_dt_sec);

    if (!impl_->yaw_initialized) {
        impl_->last_raw_yaw = yaw;
        impl_->yaw_unwrapped = yaw;
        impl_->filtered_yaw = yaw;
        impl_->yaw_initialized = true;
    } else {
        const float raw_delta = wrap_angle_180(yaw - impl_->last_raw_yaw);
        const float guarded_delta =
            clampf(raw_delta, -kYawMaxDeltaPerSampleDeg, kYawMaxDeltaPerSampleDeg);
        impl_->yaw_unwrapped += guarded_delta;
        impl_->last_raw_yaw = yaw;
        impl_->filtered_yaw += yaw_alpha * (impl_->yaw_unwrapped - impl_->filtered_yaw);
    }

    float guarded_pitch = pitch;
    float guarded_roll = roll;
    if (!impl_->pitch_roll_initialized) {
        impl_->last_raw_pitch = pitch;
        impl_->last_raw_roll = roll;
        impl_->pitch_roll_initialized = true;
    } else {
        guarded_pitch = guard_delta(pitch, impl_->last_raw_pitch, kPitchMaxDeltaPerSampleDeg);
        guarded_roll = guard_delta(roll, impl_->last_raw_roll, kRollMaxDeltaPerSampleDeg);
        impl_->last_raw_pitch = guarded_pitch;
        impl_->last_raw_roll = guarded_roll;
    }

    // Current phone mount uses roll -> tilt, pitch -> pan.
    impl_->filtered_pitch = (1.0f - kImuAlpha) * impl_->filtered_pitch + kImuAlpha * guarded_roll;
    impl_->filtered_roll = (1.0f - kImuAlpha) * impl_->filtered_roll + kImuAlpha * guarded_pitch;
    const float pitch_pan = map_axis_from_imu(impl_->filtered_roll * kPanSensitivityScale,
                                              impl_->config.pan_center_deg,
                                              impl_->config.pan_right_deg,
                                              impl_->config.pan_left_deg);
    const float yaw_pan = map_axis_from_imu(impl_->filtered_yaw * kYawPanGain * kPanSensitivityScale,
                                            impl_->config.pan_center_deg,
                                            impl_->config.pan_right_deg,
                                            impl_->config.pan_left_deg);
    const float blended_pan = (0.45f * pitch_pan) + (0.55f * yaw_pan);
    impl_->imu_target_pan = clamp_around_center(blended_pan,
                                                impl_->config.pan_center_deg,
                                                kVrPanLimitDeg);

    const float tilt_target = map_axis_from_imu(impl_->filtered_pitch * kTiltSensitivityScale,
                                                impl_->config.tilt_center_deg,
                                                impl_->config.tilt_up_deg,
                                                impl_->config.tilt_down_deg);
    impl_->imu_target_tilt = clamp_around_center(tilt_target,
                                                 impl_->config.tilt_center_deg,
                                                 kVrTiltLimitDeg);
    impl_->last_imu_arrival = now;
}

PtzStatus PtzController::latest_status() const {
    PtzStatus status;
    if (!impl_) {
        return status;
    }

    std::lock_guard<std::mutex> lock(impl_->mutex);
    status.servo_ready = impl_->servo_ready.load();
    status.mode = mode_to_cstr(impl_->mode);
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
