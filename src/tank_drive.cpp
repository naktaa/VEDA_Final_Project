#include "tank_drive.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>

#include <wiringPi.h>

namespace tank_drive {

namespace {

using Clock = std::chrono::steady_clock;

constexpr int STOP = 0;
constexpr int FORWARD = 1;
constexpr int BACKWARD = 2;

// wiringPi pin numbers
constexpr int L_IN1 = 28;
constexpr int L_IN2 = 27;
constexpr int L_EN = 1;

constexpr int R_IN1 = 25;
constexpr int R_IN2 = 23;
constexpr int R_EN = 24;

bool g_motor_ready = false;
int g_manual_pwm = 200;
int g_left_cmd = 0;
int g_right_cmd = 0;
int g_left_pwm = 0;
int g_right_pwm = 0;
int g_last_print_l = 999;
int g_last_print_r = 999;
int g_last_print_lpwm = -1;
int g_last_print_rpwm = -1;
Clock::time_point g_last_motion;
int g_motion_hold_timeout_ms = 200;
bool g_idle_autostop_enabled = true;
DriveSource g_active_source = DriveSource::kManualKey;
std::mutex g_state_mutex;

struct SourceDriveState {
    int left_cmd = 0;
    int right_cmd = 0;
    int left_pwm = 0;
    int right_pwm = 0;
    bool active = false;
    Clock::time_point last_update{};
};

std::array<SourceDriveState, 4> g_source_states{};

int clamp_pwm(int v) {
    return std::max(0, std::min(255, v));
}

int normalize_cmd(int v) {
    return (v > 0) ? 1 : (v < 0 ? -1 : 0);
}

bool uses_manual_pwm(DriveSource source) {
    return source == DriveSource::kManualKey ||
           source == DriveSource::kQt ||
           source == DriveSource::kController;
}

std::size_t source_index(DriveSource source) {
    switch (source) {
    case DriveSource::kManualKey:
        return 0;
    case DriveSource::kQt:
        return 1;
    case DriveSource::kController:
        return 2;
    case DriveSource::kAuto:
        return 3;
    }
    return 0;
}

DriveSource index_source(std::size_t index) {
    switch (index) {
    case 0:
        return DriveSource::kManualKey;
    case 1:
        return DriveSource::kQt;
    case 2:
        return DriveSource::kController;
    case 3:
        return DriveSource::kAuto;
    default:
        return DriveSource::kManualKey;
    }
}

int source_priority(DriveSource source) {
    switch (source) {
    case DriveSource::kQt:
    case DriveSource::kController:
        return 2;
    case DriveSource::kAuto:
        return 1;
    case DriveSource::kManualKey:
    default:
        return 0;
    }
}

int to_hw_pwm_duty(int pwm_255) {
    constexpr int kHwRange = 1024;
    return (clamp_pwm(pwm_255) * kHwRange) / 255;
}

void set_motor_control(int en, int in1, int in2, int pwm, int dir) {
    if (!g_motor_ready) {
        return;
    }

    pwm = clamp_pwm(pwm);
    pwmWrite(en, to_hw_pwm_duty(pwm));

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

void apply_drive(int left_cmd, int right_cmd, int left_pwm, int right_pwm) {
    const int left_dir = (left_cmd > 0) ? FORWARD : (left_cmd < 0 ? BACKWARD : STOP);
    const int right_dir = (right_cmd > 0) ? FORWARD : (right_cmd < 0 ? BACKWARD : STOP);
    set_motor_control(L_EN, L_IN1, L_IN2, (left_dir == STOP) ? 0 : left_pwm, left_dir);
    set_motor_control(R_EN, R_IN1, R_IN2, (right_dir == STOP) ? 0 : right_pwm, right_dir);
}

void stop_all() {
    apply_drive(0, 0, 0, 0);
}

void log_state_if_changed() {
    if (g_left_cmd != g_last_print_l ||
        g_right_cmd != g_last_print_r ||
        g_left_pwm != g_last_print_lpwm ||
        g_right_pwm != g_last_print_rpwm) {
        std::fprintf(stderr,
                     "[TANK] source=%d L=%d(%d) R=%d(%d) manual_pwm=%d\n",
                     static_cast<int>(g_active_source),
                     g_left_cmd,
                     g_left_pwm,
                     g_right_cmd,
                     g_right_pwm,
                     g_manual_pwm);
        std::fflush(stderr);
        g_last_print_l = g_left_cmd;
        g_last_print_r = g_right_cmd;
        g_last_print_lpwm = g_left_pwm;
        g_last_print_rpwm = g_right_pwm;
    }
}

void print_help() {
    std::fprintf(stderr, "\n=== Tank Manual Drive ===\n");
    std::fprintf(stderr, "W/S : forward/backward\n");
    std::fprintf(stderr, "A/D : rotate left/right\n");
    std::fprintf(stderr, "Q/E : pivot left/right\n");
    std::fprintf(stderr, "Space/X : stop\n");
    std::fprintf(stderr, "+/- : speed up/down\n");
    std::fprintf(stderr, "H : help\n");
    std::fprintf(stderr, "=========================\n\n");
}

void refresh_manual_pwm_locked() {
    for (std::size_t i = 0; i < g_source_states.size(); ++i) {
        DriveSource source = index_source(i);
        if (!uses_manual_pwm(source)) {
            continue;
        }

        SourceDriveState& state = g_source_states[i];
        if (!state.active) {
            continue;
        }
        state.left_pwm = (state.left_cmd != 0) ? g_manual_pwm : 0;
        state.right_pwm = (state.right_cmd != 0) ? g_manual_pwm : 0;
        state.last_update = Clock::now();
    }
}

void set_state(SourceDriveState& state,
               int left_cmd,
               int right_cmd,
               int left_pwm,
               int right_pwm,
               Clock::time_point stamp) {
    state.left_cmd = normalize_cmd(left_cmd);
    state.right_cmd = normalize_cmd(right_cmd);
    state.left_pwm = (state.left_cmd != 0) ? clamp_pwm(left_pwm) : 0;
    state.right_pwm = (state.right_cmd != 0) ? clamp_pwm(right_pwm) : 0;
    state.active = (state.left_cmd != 0 || state.right_cmd != 0) &&
                   (state.left_pwm > 0 || state.right_pwm > 0);
    state.last_update = stamp;
}

int speed_to_pwm(double speed_cmps, const RcMotorParams& params) {
    const double abs_speed = std::fabs(speed_cmps);
    if (abs_speed < params.speed_deadband_cmps) {
        return 0;
    }

    const double ratio = std::min(1.0, abs_speed / std::max(0.1, params.wheel_max_speed_cmps));
    int pwm = static_cast<int>(std::lround(ratio * params.pwm_max));
    if (pwm > 0) {
        pwm = std::max(pwm, params.pwm_min_effective);
    }
    return std::min(pwm, params.pwm_max);
}

void apply_selected_drive_locked() {
    std::size_t best_index = g_source_states.size();
    int best_priority = -1;
    Clock::time_point best_time = Clock::time_point::min();

    for (std::size_t i = 0; i < g_source_states.size(); ++i) {
        const SourceDriveState& state = g_source_states[i];
        if (!state.active) {
            continue;
        }

        const DriveSource source = index_source(i);
        const int priority = source_priority(source);
        if (priority > best_priority ||
            (priority == best_priority && state.last_update >= best_time)) {
            best_priority = priority;
            best_time = state.last_update;
            best_index = i;
        }
    }

    if (best_index == g_source_states.size()) {
        g_active_source = DriveSource::kManualKey;
        g_left_cmd = 0;
        g_right_cmd = 0;
        g_left_pwm = 0;
        g_right_pwm = 0;
    } else {
        const SourceDriveState& selected = g_source_states[best_index];
        g_active_source = index_source(best_index);
        g_left_cmd = selected.left_cmd;
        g_right_cmd = selected.right_cmd;
        g_left_pwm = selected.left_pwm;
        g_right_pwm = selected.right_pwm;
    }

    g_last_motion = Clock::now();
    apply_drive(g_left_cmd, g_right_cmd, g_left_pwm, g_right_pwm);
    log_state_if_changed();
}

} // namespace

bool init() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (wiringPiSetup() == -1) {
        std::fprintf(stderr, "[TANK] wiringPiSetup failed; motor control disabled.\n");
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
    g_left_cmd = 0;
    g_right_cmd = 0;
    g_left_pwm = 0;
    g_right_pwm = 0;
    stop_all();
    std::fprintf(stderr, "[TANK] motor control ready (manual PWM=%d)\n", g_manual_pwm);
    return true;
}

void shutdown() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_source_states = {};
    g_left_cmd = 0;
    g_right_cmd = 0;
    g_left_pwm = 0;
    g_right_pwm = 0;
    stop_all();
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
        print_help();
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
    SourceDriveState& state = g_source_states[source_index(source)];
    const Clock::time_point now = Clock::now();
    set_state(state,
              left_cmd,
              right_cmd,
              (left_cmd != 0) ? g_manual_pwm : 0,
              (right_cmd != 0) ? g_manual_pwm : 0,
              now);
    apply_selected_drive_locked();
}

void command_auto(const RcCommand& cmd, const RcMotorParams& params) {
    double left_speed = cmd.speed_cmps - (cmd.yaw_rate_rps * params.track_width_cm * 0.5);
    double right_speed = cmd.speed_cmps + (cmd.yaw_rate_rps * params.track_width_cm * 0.5);

    if (std::fabs(left_speed) < params.speed_deadband_cmps) {
        left_speed = 0.0;
    }
    if (std::fabs(right_speed) < params.speed_deadband_cmps) {
        right_speed = 0.0;
    }

    const int left_cmd = (left_speed > 0.0) ? 1 : (left_speed < 0.0 ? -1 : 0);
    const int right_cmd = (right_speed > 0.0) ? 1 : (right_speed < 0.0 ? -1 : 0);
    const int left_pwm = speed_to_pwm(left_speed, params);
    const int right_pwm = speed_to_pwm(right_speed, params);

    std::lock_guard<std::mutex> lock(g_state_mutex);
    SourceDriveState& state = g_source_states[source_index(DriveSource::kAuto)];
    set_state(state, left_cmd, right_cmd, left_pwm, right_pwm, Clock::now());
    apply_selected_drive_locked();
}

void stop() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    for (SourceDriveState& state : g_source_states) {
        set_state(state, 0, 0, 0, 0, Clock::now());
    }
    apply_selected_drive_locked();
}

void stop_from(DriveSource source) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    SourceDriveState& state = g_source_states[source_index(source)];
    set_state(state, 0, 0, 0, 0, Clock::now());
    apply_selected_drive_locked();
}

int adjust_speed(int delta) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_manual_pwm = clamp_pwm(g_manual_pwm + delta);
    refresh_manual_pwm_locked();
    std::fprintf(stderr, "[TANK] manual PWM=%d\n", g_manual_pwm);
    std::fflush(stderr);
    apply_selected_drive_locked();
    return g_manual_pwm;
}

void set_manual_speed(int pwm) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_manual_pwm = clamp_pwm(pwm);
    refresh_manual_pwm_locked();
    apply_selected_drive_locked();
}

DriveStatusSnapshot get_status_snapshot() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    DriveStatusSnapshot snapshot;
    snapshot.motor_ready = g_motor_ready;
    snapshot.moving = (g_left_cmd != 0 || g_right_cmd != 0);
    snapshot.manual_pwm = g_manual_pwm;
    snapshot.left_cmd = g_left_cmd;
    snapshot.right_cmd = g_right_cmd;
    snapshot.left_pwm = g_left_pwm;
    snapshot.right_pwm = g_right_pwm;
    snapshot.active_source = g_active_source;
    return snapshot;
}

void set_idle_autostop(bool enabled, int timeout_ms) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_idle_autostop_enabled = enabled;
    g_motion_hold_timeout_ms = std::max(0, timeout_ms);
}

void tick() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (!g_idle_autostop_enabled) {
        return;
    }

    const auto idle_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - g_last_motion).count();
    if ((g_left_cmd != 0 || g_right_cmd != 0) && idle_ms > g_motion_hold_timeout_ms) {
        SourceDriveState& active_state = g_source_states[source_index(g_active_source)];
        set_state(active_state, 0, 0, 0, 0, Clock::now());
        apply_selected_drive_locked();
    }
}

} // namespace tank_drive
