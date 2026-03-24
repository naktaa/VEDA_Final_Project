#include "vr_remote_input.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <unistd.h>

#include "tank_drive.hpp"

namespace {

using Clock = std::chrono::steady_clock;

constexpr int kDefaultSpeed = 200;
constexpr int kPollTimeoutMs = 10;
constexpr int kUiButtonDebounceMs = 350;

int clamp_speed(int value) {
    return std::max(0, std::min(255, value));
}

const char* side_button_action_name(SideButtonAction action) {
    switch (action) {
    case SideButtonAction::kEstop:
        return "estop";
    case SideButtonAction::kModeToggle:
        return "mode_toggle";
    default:
        return "unknown";
    }
}

const char* rel_code_name(unsigned short code) {
    switch (code) {
    case REL_X:
        return "REL_X";
    case REL_Y:
        return "REL_Y";
    default:
        return "REL_UNKNOWN";
    }
}

const char* key_code_name(unsigned short code) {
    switch (code) {
    case BTN_LEFT:
        return "BTN_LEFT";
    case BTN_SIDE:
        return "BTN_SIDE";
    case KEY_VOLUMEUP:
        return "KEY_VOLUMEUP";
    case KEY_VOLUMEDOWN:
        return "KEY_VOLUMEDOWN";
    default:
        return "KEY_UNKNOWN";
    }
}

class VrTankDispatcher {
public:
    explicit VrTankDispatcher(const VrRemoteInputConfig& config)
        : config_(config),
          last_input_(Clock::now()) {}

    void handle_event(const input_event& ev) {
        switch (ev.type) {
        case EV_REL:
            handle_rel(ev);
            break;
        case EV_KEY:
            handle_key(ev);
            break;
        default:
            break;
        }
    }

    void tick() {
        if ((left_cmd_ != 0 || right_cmd_ != 0) && idle_expired()) {
            fprintf(stderr, "[VR] auto-stop (idle %dms)\n", config_.idle_stop_ms);
            fflush(stderr);
            stop_drive("idle-stop");
        }
    }

    void stop_for_failure(const std::string& reason) {
        fprintf(stderr, "[VR] %s\n", reason.c_str());
        fflush(stderr);
        stop_drive("failure-stop");
    }

    void stop_for_exit() {
        stop_drive("shutdown");
    }

private:
    void dispatch_ui_action(VrUiAction action, const char* action_name) {
        if (!config_.ui_action_callback) {
            fprintf(stderr, "[VR] ui action ignored: %s (no callback)\n", action_name);
            fflush(stderr);
            return;
        }
        fprintf(stderr, "[VR] ui action -> %s\n", action_name);
        fflush(stderr);
        config_.ui_action_callback(action);
    }

    void handle_rel(const input_event& ev) {
        if (ev.code == REL_Y && ev.value != 0) {
            if (ev.value < 0) {
                drive_command(1, 1, ev, "forward");
            } else {
                drive_command(-1, -1, ev, "backward");
            }
            return;
        }

        if (ev.code == REL_X && ev.value != 0) {
            if (ev.value < 0) {
                drive_command(-1, 1, ev, "rotate_left");
            } else {
                drive_command(1, -1, ev, "rotate_right");
            }
        }
    }

    void handle_key(const input_event& ev) {
        if (ev.code == BTN_LEFT && ev.value == 1) {
            update_last_input();
            if (!should_accept_ui_button(last_zero_button_at_)) {
                return;
            }
            fprintf(stderr, "[VR] EV_KEY %s value=%d\n", key_code_name(ev.code), ev.value);
            fflush(stderr);
            dispatch_ui_action(VrUiAction::kZeroCalibrate, "zero_calibrate");
            return;
        }

        if (ev.code == BTN_SIDE && (ev.value == 0 || ev.value == 1)) {
            update_last_input();
            fprintf(stderr, "[VR] EV_KEY %s value=%d\n", key_code_name(ev.code), ev.value);
            fflush(stderr);
            if (ev.value == 1) {
                if (!should_accept_ui_button(last_session_button_at_)) {
                    return;
                }
                dispatch_ui_action(VrUiAction::kSessionButton, "session_button");
            }
            return;
        }

        if (ev.code == KEY_VOLUMEUP && ev.value == 1) {
            update_last_input();
            fprintf(stderr, "[VR] EV_KEY %s value=%d -> speed_up\n", key_code_name(ev.code), ev.value);
            fflush(stderr);
            adjust_speed(config_.speed_step);
            return;
        }

        if (ev.code == KEY_VOLUMEDOWN && ev.value == 1) {
            update_last_input();
            fprintf(stderr, "[VR] EV_KEY %s value=%d -> speed_down\n", key_code_name(ev.code), ev.value);
            fflush(stderr);
            adjust_speed(-config_.speed_step);
        }
    }

    void drive_command(int left_cmd, int right_cmd, const input_event& ev, const char* action) {
        update_last_input();
        fprintf(stderr, "[VR] EV_REL %s value=%d -> %s\n", rel_code_name(ev.code), ev.value, action);
        fflush(stderr);

        left_cmd_ = normalize_cmd(left_cmd);
        right_cmd_ = normalize_cmd(right_cmd);

        if (config_.log_only) {
            fprintf(stderr, "[VR] dispatch log-only L=%d R=%d speed=%d\n",
                    left_cmd_, right_cmd_, current_speed_);
            fflush(stderr);
            return;
        }

        tank_drive::command_drive_from(tank_drive::DriveSource::kVrRemote, left_cmd_, right_cmd_);
    }

    void stop_drive(const char* reason) {
        left_cmd_ = 0;
        right_cmd_ = 0;

        if (config_.log_only) {
            fprintf(stderr, "[VR] %s -> stop (log-only)\n", reason);
            fflush(stderr);
            return;
        }

        tank_drive::stop_from(tank_drive::DriveSource::kVrRemote);
    }

    void adjust_speed(int delta) {
        if (config_.log_only) {
            current_speed_ = clamp_speed(current_speed_ + delta);
        } else {
            current_speed_ = tank_drive::adjust_speed(delta);
        }

        fprintf(stderr, "[VR] speed=%d\n", current_speed_);
        fflush(stderr);
    }

    bool idle_expired() const {
        const auto now = Clock::now();
        const auto idle_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_input_).count();
        return idle_ms > config_.idle_stop_ms;
    }

    void update_last_input() {
        last_input_ = Clock::now();
    }

    bool should_accept_ui_button(Clock::time_point& last_pressed_at) {
        const auto now = Clock::now();
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pressed_at).count();
        if (last_pressed_at.time_since_epoch().count() != 0 && elapsed_ms < kUiButtonDebounceMs) {
            return false;
        }
        last_pressed_at = now;
        return true;
    }

    static int normalize_cmd(int value) {
        return (value > 0) ? 1 : (value < 0 ? -1 : 0);
    }

    VrRemoteInputConfig config_;
    int current_speed_ = kDefaultSpeed;
    int left_cmd_ = 0;
    int right_cmd_ = 0;
    Clock::time_point last_input_;
    Clock::time_point last_session_button_at_{};
    Clock::time_point last_zero_button_at_{};
};

bool read_events(int fd, VrTankDispatcher& dispatcher) {
    while (true) {
        input_event ev{};
        const ssize_t n = read(fd, &ev, sizeof(ev));
        if (n == static_cast<ssize_t>(sizeof(ev))) {
            dispatcher.handle_event(ev);
            continue;
        }

        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return true;
        }

        if (n < 0 && errno == EINTR) {
            continue;
        }

        if (n < 0) {
            dispatcher.stop_for_failure(std::string("input read failed: ") + std::strerror(errno));
            return false;
        }

        if (n == 0) {
            dispatcher.stop_for_failure("input device disconnected (read=0)");
            return false;
        }

        dispatcher.stop_for_failure("input device short read");
        return false;
    }
}

} // namespace

bool run_vr_remote_input_loop(const VrRemoteInputConfig& config, std::atomic<bool>& running) {
    const int fd = open(config.input_device.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "[VR] failed to open %s: %s\n",
                config.input_device.c_str(), std::strerror(errno));
        return false;
    }

    fprintf(stderr,
            "[VR] input=%s mode=%s idle_stop_ms=%d speed_step=%d side_button=%s ui_buttons=BTN_SIDE(session),BTN_LEFT(zero)\n",
            config.input_device.c_str(),
            config.log_only ? "log-only" : "drive",
            config.idle_stop_ms,
            config.speed_step,
            side_button_action_name(config.side_button_action));
    fflush(stderr);

    VrTankDispatcher dispatcher(config);

    while (running.load()) {
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        const int rc = poll(&pfd, 1, kPollTimeoutMs);
        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            dispatcher.stop_for_failure(std::string("poll failed: ") + std::strerror(errno));
            close(fd);
            return false;
        }

        if ((pfd.revents & POLLERR) != 0 || (pfd.revents & POLLHUP) != 0 || (pfd.revents & POLLNVAL) != 0) {
            dispatcher.stop_for_failure("input device poll error/hangup");
            close(fd);
            return false;
        }

        if ((pfd.revents & POLLIN) != 0) {
            if (!read_events(fd, dispatcher)) {
                close(fd);
                return false;
            }
        }

        dispatcher.tick();
    }

    dispatcher.stop_for_exit();
    close(fd);
    return true;
}
