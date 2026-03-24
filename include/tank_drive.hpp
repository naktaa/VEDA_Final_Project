#pragma once

namespace tank_drive {
    enum class DriveSource {
        kManualKey,
        kMqtt,
        kVrRemote,
    };

    bool init();
    void shutdown();
    bool handle_key(char ch);
    void command_drive(int left_cmd, int right_cmd);
    void command_drive_from(DriveSource source, int left_cmd, int right_cmd);
    void stop();
    void stop_from(DriveSource source);
    int adjust_speed(int delta);
    void set_idle_autostop(bool enabled, int timeout_ms = 200);
    void tick();
}

