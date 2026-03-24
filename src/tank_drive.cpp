#include "tank_drive.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "gpio_line.hpp"

namespace tank_drive {

namespace {

using Clock = std::chrono::steady_clock;

constexpr int STOP = 0;
constexpr int FORWARD = 1;
constexpr int BACKWARD = 2;

constexpr const char* kGpioChipPath = "/dev/gpiochip0";
constexpr const char* kPwmClassPath = "/sys/class/pwm";
constexpr int kPwmPeriodNs = 1706667; // ~586Hz, close to legacy wiringPi hardware PWM setup

// BCM GPIO offsets on gpiochip0.
constexpr unsigned int L_IN1 = 20;
constexpr unsigned int L_IN2 = 16;
constexpr unsigned int R_IN1 = 26;
constexpr unsigned int R_IN2 = 13;

constexpr int L_PWM_CHANNEL = 0; // GPIO18
constexpr int R_PWM_CHANNEL = 1; // GPIO19

bool g_motor_ready = false;
int g_pwm = 200;
int g_left_cmd = 0;
int g_right_cmd = 0;
int g_last_print_l = 999;
int g_last_print_r = 999;
int g_last_print_pwm = -1;
Clock::time_point g_last_motion;
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
GpioOutputLine g_left_in1;
GpioOutputLine g_left_in2;
GpioOutputLine g_right_in1;
GpioOutputLine g_right_in2;

struct PwmChannel {
    std::string chip_dir;
    int channel = -1;

    bool valid() const { return !chip_dir.empty() && channel >= 0; }
    std::string channel_dir() const { return chip_dir + "/pwm" + std::to_string(channel); }
    std::string path(const std::string& leaf) const { return channel_dir() + "/" + leaf; }
};

PwmChannel g_left_pwm;
PwmChannel g_right_pwm;

int clamp_pwm(int v) {
    return std::max(0, std::min(255, v));
}

int normalize_cmd(int v) {
    return (v > 0) ? 1 : (v < 0 ? -1 : 0);
}

std::size_t source_index(DriveSource source) {
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

int to_hw_pwm_duty(int pwm_255) {
    const int p = clamp_pwm(pwm_255);
    return (kPwmPeriodNs * p) / 255;
}

bool write_text_file(const std::string& path, const std::string& value) {
    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }
    output << value;
    return output.good();
}

bool wait_for_path(const std::filesystem::path& path, int retries = 50) {
    for (int i = 0; i < retries; ++i) {
        if (std::filesystem::exists(path)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return std::filesystem::exists(path);
}

std::optional<std::string> discover_pwm_chip_dir(int required_channels) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::exists(kPwmClassPath, ec)) {
        return std::nullopt;
    }

    for (const fs::directory_entry& entry : fs::directory_iterator(kPwmClassPath, ec)) {
        if (ec || !entry.is_directory()) {
            continue;
        }
        const std::string name = entry.path().filename().string();
        if (name.rfind("pwmchip", 0) != 0) {
            continue;
        }

        std::ifstream input(entry.path() / "npwm");
        int npwm = 0;
        if (!(input >> npwm)) {
            continue;
        }
        if (npwm > required_channels) {
            return entry.path().string();
        }
    }
    return std::nullopt;
}

bool export_pwm_channel(PwmChannel& channel, std::string* error) {
    namespace fs = std::filesystem;
    const fs::path pwm_dir = channel.channel_dir();
    if (!fs::exists(pwm_dir)) {
        if (!write_text_file(channel.chip_dir + "/export", std::to_string(channel.channel))) {
            if (error) {
                *error = "failed to export PWM channel " + std::to_string(channel.channel) +
                         " under " + channel.chip_dir;
            }
            return false;
        }
        if (!wait_for_path(pwm_dir)) {
            if (error) {
                *error = "PWM channel path did not appear: " + pwm_dir.string();
            }
            return false;
        }
    }
    return true;
}

bool configure_pwm_channel(PwmChannel& channel, std::string* error) {
    if (!export_pwm_channel(channel, error)) {
        return false;
    }

    write_text_file(channel.path("enable"), "0");
    if (!write_text_file(channel.path("period"), std::to_string(kPwmPeriodNs))) {
        if (error) *error = "failed to set PWM period for " + channel.channel_dir();
        return false;
    }
    if (!write_text_file(channel.path("duty_cycle"), "0")) {
        if (error) *error = "failed to set PWM duty_cycle for " + channel.channel_dir();
        return false;
    }
    if (!write_text_file(channel.path("enable"), "1")) {
        if (error) *error = "failed to enable PWM channel at " + channel.channel_dir();
        return false;
    }
    return true;
}

bool set_pwm_duty(const PwmChannel& channel, int pwm_255, std::string* error) {
    if (!channel.valid()) {
        if (error) *error = "PWM channel is not initialized";
        return false;
    }
    if (!write_text_file(channel.path("duty_cycle"), std::to_string(to_hw_pwm_duty(pwm_255)))) {
        if (error) *error = "failed to update PWM duty_cycle for " + channel.channel_dir();
        return false;
    }
    return true;
}

void shutdown_pwm_channel(const PwmChannel& channel) {
    if (!channel.valid()) {
        return;
    }
    write_text_file(channel.path("duty_cycle"), "0");
    write_text_file(channel.path("enable"), "0");
}

bool log_and_disable_motor(const std::string& message) {
    std::fprintf(stderr, "[TANK] %s; motor control disabled.\n", message.c_str());
    std::fflush(stderr);
    g_motor_ready = false;
    return false;
}

bool set_direction(GpioOutputLine& line, int value) {
    std::string error;
    if (line.set_value(value, &error)) {
        return true;
    }
    return log_and_disable_motor(error);
}

bool set_motor_control(const PwmChannel& enable_channel,
                       GpioOutputLine& in1,
                       GpioOutputLine& in2,
                       int pwm,
                       int dir) {
    if (!g_motor_ready) return false;
    pwm = clamp_pwm(pwm);
    std::string error;
    if (!set_pwm_duty(enable_channel, pwm, &error)) {
        return log_and_disable_motor(error);
    }

    if (dir == FORWARD) {
        return set_direction(in1, 0) && set_direction(in2, 1);
    }
    if (dir == BACKWARD) {
        return set_direction(in1, 1) && set_direction(in2, 0);
    }

    if (!set_pwm_duty(enable_channel, 0, &error)) {
        return log_and_disable_motor(error);
    }
    return set_direction(in1, 0) && set_direction(in2, 0);
}

bool init_motor_outputs(std::string* error) {
    if (!g_left_in1.open(kGpioChipPath, L_IN1, "tank_left_in1", 0, error) ||
        !g_left_in2.open(kGpioChipPath, L_IN2, "tank_left_in2", 0, error) ||
        !g_right_in1.open(kGpioChipPath, R_IN1, "tank_right_in1", 0, error) ||
        !g_right_in2.open(kGpioChipPath, R_IN2, "tank_right_in2", 0, error)) {
        return false;
    }

    const std::optional<std::string> pwm_chip_dir = discover_pwm_chip_dir(R_PWM_CHANNEL);
    if (!pwm_chip_dir) {
        if (error) {
            *error = "usable pwmchip not found under /sys/class/pwm";
        }
        return false;
    }

    g_left_pwm = PwmChannel{*pwm_chip_dir, L_PWM_CHANNEL};
    g_right_pwm = PwmChannel{*pwm_chip_dir, R_PWM_CHANNEL};
    if (!configure_pwm_channel(g_left_pwm, error) || !configure_pwm_channel(g_right_pwm, error)) {
        return false;
    }
    return true;
}

void close_motor_outputs() {
    shutdown_pwm_channel(g_left_pwm);
    shutdown_pwm_channel(g_right_pwm);
    g_left_pwm = {};
    g_right_pwm = {};
    g_left_in1.close();
    g_left_in2.close();
    g_right_in1.close();
    g_right_in2.close();
}

void apply_drive(int left_cmd, int right_cmd, int pwm) {
    const int ldir = (left_cmd > 0) ? FORWARD : (left_cmd < 0 ? BACKWARD : STOP);
    const int rdir = (right_cmd > 0) ? FORWARD : (right_cmd < 0 ? BACKWARD : STOP);
    set_motor_control(g_left_pwm, g_left_in1, g_left_in2, (ldir == STOP) ? 0 : pwm, ldir);
    set_motor_control(g_right_pwm, g_right_in1, g_right_in2, (rdir == STOP) ? 0 : pwm, rdir);
}

void stop_all() {
    if (!g_motor_ready) return;
    set_motor_control(L_EN, L_IN1, L_IN2, 0, STOP);
    set_motor_control(R_EN, R_IN1, R_IN2, 0, STOP);
}

void log_state_if_changed() {
    if (g_left_cmd != g_last_print_l || g_right_cmd != g_last_print_r || g_pwm != g_last_print_pwm) {
        std::fprintf(stderr, "[TANK] L=%d R=%d PWM=%d\n", g_left_cmd, g_right_cmd, g_pwm);
        std::fflush(stderr);
        g_last_print_l = g_left_cmd;
        g_last_print_r = g_right_cmd;
        g_last_print_pwm = g_pwm;
    }
}

void print_help() {
    std::fprintf(stderr, "\n=== Tank Manual Drive ===\n");
    std::fprintf(stderr, "W/S : forward/backward\n");
    std::fprintf(stderr, "A/D : rotate left/right (tank spin)\n");
    std::fprintf(stderr, "Q/E : pivot left/right\n");
    std::fprintf(stderr, "Space/X : stop\n");
    std::fprintf(stderr, "+/- : speed up/down\n");
    std::fprintf(stderr, "H : help\n");
    std::fprintf(stderr, "=========================\n\n");
}

void apply_selected_drive_locked() {
    std::size_t best_index = g_source_states.size();
    if (g_source_states[source_index(g_active_source)].active) {
        best_index = source_index(g_active_source);
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
        g_active_source = DriveSource::kManualKey;
        g_left_cmd = 0;
        g_right_cmd = 0;
    } else {
        g_active_source = static_cast<DriveSource>(best_index);
        g_left_cmd = g_source_states[best_index].left_cmd;
        g_right_cmd = g_source_states[best_index].right_cmd;
    }

    g_last_motion = Clock::now();
    apply_drive(g_left_cmd, g_right_cmd, g_pwm);
    log_state_if_changed();
}

} // namespace

bool init() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    std::string error;
    if (!init_motor_outputs(&error)) {
        close_motor_outputs();
        std::fprintf(stderr, "[TANK] %s; motor control disabled.\n", error.c_str());
        g_motor_ready = false;
        return false;
    }

    g_motor_ready = true;
    g_active_source = DriveSource::kManualKey;
    g_source_states = {};
    g_last_motion = Clock::now();
    stop_all();
    std::fprintf(stderr, "[TANK] motor control ready (PWM=%d)\n", g_pwm);
    return true;
}

void shutdown() {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_source_states = {};
    stop_all();
    g_motor_ready = false;
    close_motor_outputs();
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
    if (!g_motor_ready) return;

    SourceDriveState& state = g_source_states[source_index(source)];
    state.left_cmd = normalize_cmd(left_cmd);
    state.right_cmd = normalize_cmd(right_cmd);
    state.active = (state.left_cmd != 0 || state.right_cmd != 0);
    state.last_update = Clock::now();
    if (state.active) {
        g_active_source = source;
    }
    apply_selected_drive_locked();
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
    apply_selected_drive_locked();
}

void stop_from(DriveSource source) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    SourceDriveState& state = g_source_states[source_index(source)];
    state.left_cmd = 0;
    state.right_cmd = 0;
    state.active = false;
    state.last_update = Clock::now();
    if (!g_motor_ready) return;
    apply_selected_drive_locked();
}

int adjust_speed(int delta) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_pwm = clamp_pwm(g_pwm + delta);
    g_last_motion = Clock::now();
    std::fprintf(stderr, "[TANK] PWM=%d\n", g_pwm);
    std::fflush(stderr);

    if (g_motor_ready) {
        apply_drive(g_left_cmd, g_right_cmd, g_pwm);
        log_state_if_changed();
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
        SourceDriveState& state = g_source_states[source_index(g_active_source)];
        state.left_cmd = 0;
        state.right_cmd = 0;
        state.active = false;
        state.last_update = now;
        apply_selected_drive_locked();
        std::fprintf(stderr, "[TANK] auto-stop (idle)\n");
        std::fflush(stderr);
    }
}

} // namespace tank_drive
