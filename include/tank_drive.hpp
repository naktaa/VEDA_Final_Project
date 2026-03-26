#pragma once

#include "rc_control_types.hpp"

namespace tank_drive {
    enum class DriveSource {
        kManualKey,
        kQt,
        kController,
        kAuto,
    };

    struct DriveStatusSnapshot {
        bool motor_ready = false;
        bool moving = false;
        int manual_pwm = 0;
        int left_cmd = 0;
        int right_cmd = 0;
        int left_pwm = 0;
        int right_pwm = 0;
        DriveSource active_source = DriveSource::kManualKey;
    };

    bool init();
    void shutdown();
    bool handle_key(char ch);
    void command_drive(int left_cmd, int right_cmd);
    void command_drive_from(DriveSource source, int left_cmd, int right_cmd);
    void command_auto(const RcCommand& cmd, const RcMotorParams& params);
    void stop();
    void stop_from(DriveSource source);
    int adjust_speed(int delta);
    void set_manual_speed(int pwm);
    DriveStatusSnapshot get_status_snapshot();
    void set_idle_autostop(bool enabled, int timeout_ms = 200);
    void tick();
}

