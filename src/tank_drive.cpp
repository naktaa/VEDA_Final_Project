#include "tank_drive.hpp"

#include <chrono>
#include <cstdio>
#include <algorithm>

#include <wiringPi.h>

namespace tank_drive {

namespace {
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

    int clampPwm(int v) { return std::max(0, std::min(255, v)); }

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
} // namespace

bool init() {
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
    g_last_motion = std::chrono::steady_clock::now();
    stopAll();
    fprintf(stderr, "[TANK] motor control ready (PWM=%d)\n", g_pwm);
    return true;
}

void shutdown() {
    stopAll();
    g_motor_ready = false;
}

bool handle_key(char ch) {
    if (!g_motor_ready) return false;

    bool consumed = true;
    bool apply_after = true;
    switch (ch) {
    case 'w':
    case 'W':
        g_left_cmd = 1;
        g_right_cmd = 1;
        break;
    case 's':
    case 'S':
        g_left_cmd = -1;
        g_right_cmd = -1;
        break;
    case 'a':
    case 'A':
        g_left_cmd = -1;
        g_right_cmd = 1;
        break;
    case 'd':
    case 'D':
        g_left_cmd = 1;
        g_right_cmd = -1;
        break;
    case 'q':
    case 'Q':
        g_left_cmd = 0;
        g_right_cmd = 1;
        break;
    case 'e':
    case 'E':
        g_left_cmd = 1;
        g_right_cmd = 0;
        break;
    case 'x':
    case 'X':
    case ' ':
        g_left_cmd = 0;
        g_right_cmd = 0;
        break;
    case '+':
    case '=':
        adjust_speed(10);
        apply_after = false;
        break;
    case '-':
    case '_':
        adjust_speed(-10);
        apply_after = false;
        break;
    case 'h':
    case 'H':
        printHelp();
        apply_after = false;
        break;
    default:
        consumed = false;
        break;
    }

    if (consumed && apply_after) {
        g_last_motion = std::chrono::steady_clock::now();
        applyDrive(g_left_cmd, g_right_cmd, g_pwm);
        logStateIfChanged();
    }
    return consumed;
}

void command_drive(int left_cmd, int right_cmd) {
    if (!g_motor_ready) return;
    g_left_cmd = (left_cmd > 0) ? 1 : (left_cmd < 0 ? -1 : 0);
    g_right_cmd = (right_cmd > 0) ? 1 : (right_cmd < 0 ? -1 : 0);
    g_last_motion = std::chrono::steady_clock::now();
    applyDrive(g_left_cmd, g_right_cmd, g_pwm);
    logStateIfChanged();
}

void stop() {
    command_drive(0, 0);
}

int adjust_speed(int delta) {
    g_pwm = clampPwm(g_pwm + delta);
    g_last_motion = std::chrono::steady_clock::now();
    fprintf(stderr, "[TANK] PWM=%d\n", g_pwm);
    fflush(stderr);

    if (g_motor_ready) {
        applyDrive(g_left_cmd, g_right_cmd, g_pwm);
        logStateIfChanged();
    }
    return g_pwm;
}

void set_idle_autostop(bool enabled, int timeout_ms) {
    g_idle_autostop_enabled = enabled;
    g_motion_hold_timeout_ms = std::max(0, timeout_ms);
}

void tick() {
    if (!g_motor_ready) return;
    if (!g_idle_autostop_enabled) return;
    const auto now = std::chrono::steady_clock::now();
    const auto idle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_motion).count();
    if ((g_left_cmd != 0 || g_right_cmd != 0) && idle_ms > g_motion_hold_timeout_ms) {
        g_left_cmd = 0;
        g_right_cmd = 0;
        applyDrive(g_left_cmd, g_right_cmd, g_pwm);
        fprintf(stderr, "[TANK] auto-stop (idle)\n");
        fflush(stderr);
        g_last_print_l = g_left_cmd;
        g_last_print_r = g_right_cmd;
    }
}

} // namespace tank_drive
