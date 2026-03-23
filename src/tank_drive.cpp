#include "tank_drive.hpp"

#include <chrono>
#include <cstdio>
#include <algorithm>
#include <array>
#include <mutex>

#include <wiringPi.h>

namespace tank_drive {

namespace {
    using Clock = std::chrono::steady_clock;

    constexpr int STOP = 0;
    constexpr int FORWARD = 1;
    constexpr int BACKWARD = 2;

    // wiringPi pin numbers (same as legacy tank+eis.cpp)
    //   L_EN -> GPIO18 (wPi 1,  physical 12) PWM0
    //   R_EN -> GPIO19 (wPi 24, physical 35) PWM1
    constexpr int L_IN1 = 28; // GPIO20, physical 38
    constexpr int L_IN2 = 27; // GPIO16, physical 36
    constexpr int L_EN  = 1;  // GPIO18, physical 12 (HW PWM0)

    constexpr int R_IN1 = 25; // GPIO26, physical 37
    constexpr int R_IN2 = 23; // GPIO13, physical 33
    constexpr int R_EN  = 24; // GPIO19, physical 35 (HW PWM1)

    bool g_motor_ready = false;
    int g_pwm = 200;
    int g_left_cmd = 0;
    int g_right_cmd = 0;
    int g_last_print_l = 999;
    int g_last_print_r = 999;
    int g_last_print_pwm = -1;
    std::chrono::steady_clock::time_point g_last_motion;
    int g_motion_hold_timeout_ms = 200;
    bool g_idle_autostop_enabled = true;
    DriveSource g_active_source = DriveSource::kManualKey;
    std::mutex g_state_mutex;

    struct SourceDriveState {
        int left_cmd = 0;
        int right_cmd = 0;
        bool active = false;
        Clock::time_point last_update{};
    };

    std::array<SourceDriveState, 3> g_source_states{};

    int clampPwm(int v) { return std::max(0, std::min(255, v)); }

    int normalizeCmd(int v) {
        return (v > 0) ? 1 : (v < 0 ? -1 : 0);
    }

    std::size_t sourceIndex(DriveSource source) {
        switch (source) {
        case DriveSource::kManualKey:
            return 0;
        case DriveSource::kMqtt:
            return 1;
        case DriveSource::kVrRemote:
            return 2;
        }
        return 0;
    }

    int toHwPwmDuty(int pwm_255) {
        constexpr int HW_RANGE = 1024;
        const int p = clampPwm(pwm_255);
        return (p * HW_RANGE) / 255;
    }

    void setMotorControl(int en, int in1, int in2, int pwm, int dir) {
        if (!g_motor_ready) return;
        pwm = clampPwm(pwm);
        pwmWrite(en, toHwPwmDuty(pwm));

        if (dir == FORWARD) {
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
        } else if (dir == BACKWARD) {
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
        } else {
            pwmWrite(en, 0);
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
        }
    }

    void applyDrive(int left_cmd, int right_cmd, int pwm) {
        const int ldir = (left_cmd > 0) ? FORWARD : (left_cmd < 0 ? BACKWARD : STOP);
        const int rdir = (right_cmd > 0) ? FORWARD : (right_cmd < 0 ? BACKWARD : STOP);
        setMotorControl(L_EN, L_IN1, L_IN2, (ldir == STOP) ? 0 : pwm, ldir);
        setMotorControl(R_EN, R_IN1, R_IN2, (rdir == STOP) ? 0 : pwm, rdir);
    }

    void stopAll() {
        if (!g_motor_ready) return;
        setMotorControl(L_EN, L_IN1, L_IN2, 0, STOP);
        setMotorControl(R_EN, R_IN1, R_IN2, 0, STOP);
    }

    void logStateIfChanged() {
        if (g_left_cmd != g_last_print_l || g_right_cmd != g_last_print_r || g_pwm != g_last_print_pwm) {
            fprintf(stderr, "[TANK] L=%d R=%d PWM=%d\n", g_left_cmd, g_right_cmd, g_pwm);
            fflush(stderr);
            g_last_print_l = g_left_cmd;
            g_last_print_r = g_right_cmd;
            g_last_print_pwm = g_pwm;
        }
    }

    void printHelp() {
        fprintf(stderr, "\n=== Tank Manual Drive ===\n");
        fprintf(stderr, "W/S : forward/backward\n");
        fprintf(stderr, "A/D : rotate left/right (tank spin)\n");
        fprintf(stderr, "Q/E : pivot left/right\n");
        fprintf(stderr, "Space/X : stop\n");
        fprintf(stderr, "+/- : speed up/down\n");
        fprintf(stderr, "H : help\n");
        fprintf(stderr, "=========================\n\n");
    }

    void applySelectedDriveLocked() {
        std::size_t best_index = g_source_states.size();
        if (g_source_states[sourceIndex(g_active_source)].active) {
            best_index = sourceIndex(g_active_source);
        } else {
            for (std::size_t i = 0; i < g_source_states.size(); ++i) {
                if (!g_source_states[i].active) continue;
                if (best_index == g_source_states.size() ||
                    g_source_states[best_index].last_update < g_source_states[i].last_update) {
                    best_index = i;
                }
            }
        }

        if (best_index == g_source_states.size()) {
            g_left_cmd = 0;
            g_right_cmd = 0;
        } else {
            g_active_source = static_cast<DriveSource>(best_index);
            g_left_cmd = g_source_states[best_index].left_cmd;
            g_right_cmd = g_source_states[best_index].right_cmd;
        }

        g_last_motion = Clock::now();
        applyDrive(g_left_cmd, g_right_cmd, g_pwm);
        logStateIfChanged();
    }
} // namespace

bool init() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (wiringPiSetup() == -1) {
        fprintf(stderr, "[TANK] wiringPiSetup failed; motor control disabled.\n");
        g_motor_ready = false;
        return false;
    }

    pinMode(L_EN, PWM_OUTPUT);
    pinMode(R_EN, PWM_OUTPUT);
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);

    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);

    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(32);
    pwmSetRange(1024);

    g_motor_ready = true;
    g_active_source = DriveSource::kManualKey;
    g_source_states = {};
    g_last_motion = Clock::now();
    stopAll();
    fprintf(stderr, "[TANK] motor control ready (PWM=%d)\n", g_pwm);
    return true;
}

void shutdown() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_source_states = {};
    stopAll();
    g_motor_ready = false;
}

bool handle_key(char ch) {
    int left_cmd = 0;
    int right_cmd = 0;

    switch (ch) {
    case 'w':
    case 'W':
        left_cmd = 1;
        right_cmd = 1;
        break;
    case 's':
    case 'S':
        left_cmd = -1;
        right_cmd = -1;
        break;
    case 'a':
    case 'A':
        left_cmd = -1;
        right_cmd = 1;
        break;
    case 'd':
    case 'D':
        left_cmd = 1;
        right_cmd = -1;
        break;
    case 'q':
    case 'Q':
        left_cmd = 0;
        right_cmd = 1;
        break;
    case 'e':
    case 'E':
        left_cmd = 1;
        right_cmd = 0;
        break;
    case 'x':
    case 'X':
    case ' ':
        stop_from(DriveSource::kManualKey);
        return true;
    case '+':
    case '=':
        adjust_speed(10);
        return true;
    case '-':
    case '_':
        adjust_speed(-10);
        return true;
    case 'h':
    case 'H':
        printHelp();
        return true;
    default:
        return false;
    }

    command_drive_from(DriveSource::kManualKey, left_cmd, right_cmd);
    return true;
}

void command_drive(int left_cmd, int right_cmd) {
    command_drive_from(DriveSource::kManualKey, left_cmd, right_cmd);
}

void command_drive_from(DriveSource source, int left_cmd, int right_cmd) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (!g_motor_ready) return;

    SourceDriveState& state = g_source_states[sourceIndex(source)];
    state.left_cmd = normalizeCmd(left_cmd);
    state.right_cmd = normalizeCmd(right_cmd);
    state.active = (state.left_cmd != 0 || state.right_cmd != 0);
    state.last_update = Clock::now();
    if (state.active) {
        g_active_source = source;
    }
    applySelectedDriveLocked();
}

void stop() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    for (SourceDriveState& state : g_source_states) {
        state.left_cmd = 0;
        state.right_cmd = 0;
        state.active = false;
        state.last_update = Clock::now();
    }
    if (!g_motor_ready) return;
    applySelectedDriveLocked();
}

void stop_from(DriveSource source) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    SourceDriveState& state = g_source_states[sourceIndex(source)];
    state.left_cmd = 0;
    state.right_cmd = 0;
    state.active = false;
    state.last_update = Clock::now();
    if (!g_motor_ready) return;
    applySelectedDriveLocked();
}

int adjust_speed(int delta) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_pwm = clampPwm(g_pwm + delta);
    g_last_motion = Clock::now();
    fprintf(stderr, "[TANK] PWM=%d\n", g_pwm);
    fflush(stderr);

    if (g_motor_ready) {
        applyDrive(g_left_cmd, g_right_cmd, g_pwm);
        logStateIfChanged();
    }
    return g_pwm;
}

void set_idle_autostop(bool enabled, int timeout_ms) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_idle_autostop_enabled = enabled;
    g_motion_hold_timeout_ms = std::max(0, timeout_ms);
}

void tick() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (!g_motor_ready) return;
    if (!g_idle_autostop_enabled) return;
    const auto now = Clock::now();
    const auto idle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_motion).count();
    if ((g_left_cmd != 0 || g_right_cmd != 0) && idle_ms > g_motion_hold_timeout_ms) {
        SourceDriveState& state = g_source_states[sourceIndex(g_active_source)];
        state.left_cmd = 0;
        state.right_cmd = 0;
        state.active = false;
        state.last_update = now;
        applySelectedDriveLocked();
        fprintf(stderr, "[TANK] auto-stop (idle)\n");
        fflush(stderr);
    }
}

} // namespace tank_drive
