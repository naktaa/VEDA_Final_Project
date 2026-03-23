#pragma once

namespace tank_drive {
    enum class DriveSource {
        kManualKey,
        kMqtt,
        kVrRemote,
    };

    struct DriveStatusSnapshot {
        bool motor_ready = false;
        int pwm = 0;
        int left_cmd = 0;
        int right_cmd = 0;
        bool moving = false;
        DriveSource active_source = DriveSource::kManualKey;
    };

    bool init();
    void shutdown();
    bool handle_key(char ch);
    void command_drive(int left_cmd, int right_cmd);
    void command_drive_from(DriveSource source, int left_cmd, int right_cmd);
    void stop();
    void stop_from(DriveSource source);
    int adjust_speed(int delta);
    DriveStatusSnapshot get_status_snapshot();
    void set_idle_autostop(bool enabled, int timeout_ms = 200);
    void tick();
}

