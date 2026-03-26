#pragma once

#include "rc_control_types.hpp"

class RcMotorDriver {
public:
    RcMotorDriver() = default;

    void configure(const RcMotorParams& params);
    void setup();
    void stopAll() const;
    void sendCommand(const RcCommand& cmd) const;
    bool ready() const;

private:
    void setMotorControl(int en, int in1, int in2, int speed_pwm, int dir) const;
    int speedToPwm(double speed_cmps) const;

private:
    bool ready_ = false;
    RcMotorParams params_;
};
