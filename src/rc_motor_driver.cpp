#include "rc_motor_driver.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#if __has_include(<wiringPi.h>)
#include <wiringPi.h>
#define RC_HAS_WIRINGPI 1
#else
#define RC_HAS_WIRINGPI 0
#endif

namespace {

constexpr int kStop = 0;
constexpr int kForward = 1;
constexpr int kBackward = 2;

constexpr int kLeftIn1 = 28;
constexpr int kLeftIn2 = 27;
constexpr int kLeftEn = 1;
constexpr int kRightIn1 = 25;
constexpr int kRightIn2 = 23;
constexpr int kRightEn = 24;
constexpr int kPwmHwRange = 1024;

int Pwm255ToDuty(int pwm_255) {
    const int p = std::max(0, std::min(255, pwm_255));
    return (p * kPwmHwRange) / 255;
}

} // namespace

void RcMotorDriver::configure(const RcMotorParams& params) {
    params_ = params;
}

void RcMotorDriver::setup() {
#if !RC_HAS_WIRINGPI
    std::cerr << "[WARN] wiringPi.h not found. motor output disabled.\n";
    ready_ = false;
    return;
#else
    if (wiringPiSetup() == -1) {
        std::cerr << "[ERR] wiringPiSetup failed. motor output disabled.\n";
        ready_ = false;
        return;
    }

    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(32);
    pwmSetRange(kPwmHwRange);

    auto setup_one = [](int en, int in1, int in2) -> bool {
        pinMode(en, PWM_OUTPUT);
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        pwmWrite(en, 0);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        return true;
    };

    const bool left_ok = setup_one(kLeftEn, kLeftIn1, kLeftIn2);
    const bool right_ok = setup_one(kRightEn, kRightIn1, kRightIn2);
    ready_ = left_ok && right_ok;

    if (!ready_) {
        std::cerr << "[ERR] motor pin setup failed.\n";
        return;
    }

    stopAll();
    std::cout << "[OK] motor driver ready (HW PWM, L_EN=1, R_EN=24)\n";
#endif
}

void RcMotorDriver::stopAll() const {
    if (!ready_) {
        return;
    }
    setMotorControl(kLeftEn, kLeftIn1, kLeftIn2, 0, kStop);
    setMotorControl(kRightEn, kRightIn1, kRightIn2, 0, kStop);
}

void RcMotorDriver::sendCommand(const RcCommand& cmd) const {
    if (!ready_) {
        return;
    }

    double v_left = cmd.speed_mps - (cmd.yaw_rate_rps * params_.track_width_m * 0.5);
    double v_right = cmd.speed_mps + (cmd.yaw_rate_rps * params_.track_width_m * 0.5);

    if (std::fabs(v_left) < params_.speed_deadband_mps) {
        v_left = 0.0;
    }
    if (std::fabs(v_right) < params_.speed_deadband_mps) {
        v_right = 0.0;
    }

    const int left_dir = (v_left > 0.0) ? kForward : (v_left < 0.0 ? kBackward : kStop);
    const int right_dir = (v_right > 0.0) ? kForward : (v_right < 0.0 ? kBackward : kStop);

    const int left_pwm = speedToPwm(v_left);
    const int right_pwm = speedToPwm(v_right);

    setMotorControl(kLeftEn, kLeftIn1, kLeftIn2, left_pwm, left_dir);
    setMotorControl(kRightEn, kRightIn1, kRightIn2, right_pwm, right_dir);
}

bool RcMotorDriver::ready() const {
    return ready_;
}

void RcMotorDriver::setMotorControl(int en, int in1, int in2, int speed_pwm, int dir) const {
#if !RC_HAS_WIRINGPI
    (void)en;
    (void)in1;
    (void)in2;
    (void)speed_pwm;
    (void)dir;
    return;
#else
    const int pwm = std::max(0, std::min(params_.pwm_max, speed_pwm));
    pwmWrite(en, Pwm255ToDuty(pwm));

    if (dir == kForward) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else if (dir == kBackward) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        pwmWrite(en, 0);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
#endif
}

int RcMotorDriver::speedToPwm(double speed_mps) const {
    const double abs_speed = std::fabs(speed_mps);
    if (abs_speed < params_.speed_deadband_mps) {
        return 0;
    }

    const double ratio = std::min(1.0, abs_speed / std::max(0.001, params_.wheel_max_speed_mps));
    int pwm = static_cast<int>(std::lround(ratio * params_.pwm_max));
    if (pwm > 0) {
        pwm = std::max(pwm, params_.pwm_min_effective);
    }
    return std::min(pwm, params_.pwm_max);
}
