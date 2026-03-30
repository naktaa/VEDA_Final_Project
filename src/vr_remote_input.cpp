#include "vr_remote_input.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "tank_drive.hpp"

namespace {

using Clock = std::chrono::steady_clock;

constexpr int kDefaultSpeed = 200;
constexpr int kPollTimeoutMs = 10;
constexpr std::size_t kInputNameBufferSize = 256;

int clamp_speed(int value) {
    return std::max(0, std::min(255, value));
}

const char* side_button_action_name(SideButtonAction action) {
    switch (action) {
    case SideButtonAction::kEstop:
        return "estop";
    case SideButtonAction::kModeToggle:
        return "ptz_vr";
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

std::string lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool read_input_device_name(int fd, std::string& out_name) {
    char name[kInputNameBufferSize] {};
    if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) {
        return false;
    }
    out_name = name;
    return true;
}

bool input_name_matches(const std::string& actual_name, const std::string& name_hint) {
    if (name_hint.empty()) {
        return false;
    }
    const std::string actual_lower = lower_copy(actual_name);
    const std::string hint_lower = lower_copy(name_hint);
    return actual_lower == hint_lower || actual_lower.find(hint_lower) != std::string::npos;
}

bool discover_input_device_by_name(const std::string& name_hint,
                                   std::string& out_path,
                                   std::string& out_name,
                                   std::string* error) {
    namespace fs = std::filesystem;

    std::vector<fs::path> event_paths;
    std::error_code ec;
    for (const fs::directory_entry& entry : fs::directory_iterator("/dev/input", ec)) {
        if (ec) {
            break;
        }
        if (!entry.is_character_file(ec) && !entry.is_other(ec)) {
            continue;
        }
        const std::string filename = entry.path().filename().string();
        if (filename.rfind("event", 0) == 0) {
            event_paths.push_back(entry.path());
        }
    }

    std::sort(event_paths.begin(), event_paths.end());
    for (const fs::path& path : event_paths) {
        const int fd = open(path.string().c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            continue;
        }

        std::string actual_name;
        const bool name_ok = read_input_device_name(fd, actual_name);
        close(fd);
        if (!name_ok) {
            continue;
        }
        if (!input_name_matches(actual_name, name_hint)) {
            continue;
        }

        out_path = path.string();
        out_name = actual_name;
        return true;
    }

    if (error) {
        *error = "failed to find input device matching name hint '" + name_hint + "'";
    }
    return false;
}

bool resolve_input_device(const VrRemoteInputConfig& config,
                          std::string& out_path,
                          std::string& out_name,
                          std::string* error) {
    if (!config.device_name_hint.empty() &&
        discover_input_device_by_name(config.device_name_hint, out_path, out_name, error)) {
        return true;
    }

    if (config.input_device.empty()) {
        if (error && error->empty()) {
            *error = "no VR input device configured";
        }
        return false;
    }

    const int fd = open(config.input_device.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        if (error) {
            *error = std::string("failed to open ") + config.input_device + ": " + std::strerror(errno);
        }
        return false;
    }

    out_path = config.input_device;
    read_input_device_name(fd, out_name);
    close(fd);
    return true;
}

class VrTankDispatcher {
public:
    VrTankDispatcher(const VrRemoteInputConfig& config,
                     std::function<void(const char*)> on_manual_override,
                     std::function<bool()> on_side_button,
                     std::function<bool()> on_left_button)
        : config_(config),
          on_manual_override_(std::move(on_manual_override)),
          on_side_button_(std::move(on_side_button)),
          on_left_button_(std::move(on_left_button)),
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
            std::fprintf(stderr, "[VR] auto-stop (idle %dms)\n", config_.idle_stop_ms);
            std::fflush(stderr);
            stop_drive("idle-stop");
        }
    }

    void stop_for_failure(const std::string& reason) {
        std::fprintf(stderr, "[VR] %s\n", reason.c_str());
        std::fflush(stderr);
        stop_drive("failure-stop");
    }

    void stop_for_exit() {
        stop_drive("shutdown");
    }

private:
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
            const bool handled = on_left_button_ && on_left_button_();
            std::fprintf(stderr,
                         "[VR] EV_KEY %s value=%d -> %s\n",
                         key_code_name(ev.code),
                         ev.value,
                         handled ? "zero_calibrate" : "zero_calibrate unavailable");
            std::fflush(stderr);
            return;
        }

        if (ev.code == BTN_SIDE && (ev.value == 0 || ev.value == 1)) {
            update_last_input();
            std::fprintf(stderr, "[VR] EV_KEY %s value=%d\n", key_code_name(ev.code), ev.value);
            std::fflush(stderr);
            handle_side_button(ev.value == 1);
            return;
        }

        if (ev.code == KEY_VOLUMEUP && ev.value == 1) {
            update_last_input();
            std::fprintf(stderr, "[VR] EV_KEY %s value=%d -> speed_up\n", key_code_name(ev.code), ev.value);
            std::fflush(stderr);
            adjust_speed(config_.speed_step);
            return;
        }

        if (ev.code == KEY_VOLUMEDOWN && ev.value == 1) {
            update_last_input();
            std::fprintf(stderr, "[VR] EV_KEY %s value=%d -> speed_down\n", key_code_name(ev.code), ev.value);
            std::fflush(stderr);
            adjust_speed(-config_.speed_step);
        }
    }

    void handle_side_button(bool pressed) {
        switch (config_.side_button_action) {
        case SideButtonAction::kEstop:
            if (pressed) {
                estop_active_ = true;
                notify_manual_override();
                std::fprintf(stderr, "[VR] side button -> estop active\n");
                std::fflush(stderr);
                stop_drive("estop");
            } else {
                estop_active_ = false;
                std::fprintf(stderr, "[VR] side button -> estop released\n");
                std::fflush(stderr);
            }
            return;
        case SideButtonAction::kModeToggle:
            if (pressed) {
                const bool handled = on_side_button_ && on_side_button_();
                std::fprintf(stderr,
                             "[VR] side button -> %s\n",
                             handled ? "vr session toggle" : "vr session unavailable");
                std::fflush(stderr);
            }
            return;
        }
    }

    void drive_command(int left_cmd, int right_cmd, const input_event& ev, const char* action) {
        update_last_input();
        notify_manual_override();
        std::fprintf(stderr, "[VR] EV_REL %s value=%d -> %s\n", rel_code_name(ev.code), ev.value, action);
        std::fflush(stderr);

        if (estop_active_) {
            std::fprintf(stderr, "[VR] ignored while estop is active\n");
            std::fflush(stderr);
            return;
        }

        left_cmd_ = normalize_cmd(left_cmd);
        right_cmd_ = normalize_cmd(right_cmd);

        if (config_.log_only) {
            std::fprintf(stderr, "[VR] dispatch log-only L=%d R=%d speed=%d\n",
                         left_cmd_, right_cmd_, current_speed_);
            std::fflush(stderr);
            return;
        }

        tank_drive::command_drive_from(tank_drive::DriveSource::kController, left_cmd_, right_cmd_);
    }

    void stop_drive(const char* reason) {
        left_cmd_ = 0;
        right_cmd_ = 0;

        if (config_.log_only) {
            std::fprintf(stderr, "[VR] %s -> stop (log-only)\n", reason);
            std::fflush(stderr);
            return;
        }

        tank_drive::stop_from(tank_drive::DriveSource::kController);
    }

    void adjust_speed(int delta) {
        if (config_.log_only) {
            current_speed_ = clamp_speed(current_speed_ + delta);
        } else {
            current_speed_ = tank_drive::adjust_speed(delta);
        }

        std::fprintf(stderr, "[VR] speed=%d\n", current_speed_);
        std::fflush(stderr);
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

    void notify_manual_override() {
        if (on_manual_override_) {
            on_manual_override_("controller");
        }
    }

    static int normalize_cmd(int value) {
        return (value > 0) ? 1 : (value < 0 ? -1 : 0);
    }

    VrRemoteInputConfig config_;
    std::function<void(const char*)> on_manual_override_;
    std::function<bool()> on_side_button_;
    std::function<bool()> on_left_button_;
    int current_speed_ = kDefaultSpeed;
    int left_cmd_ = 0;
    int right_cmd_ = 0;
    bool estop_active_ = false;
    Clock::time_point last_input_;
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

bool run_vr_remote_input_loop(const VrRemoteInputConfig& config,
                              std::atomic<bool>& running,
                              const std::function<void(const char*)>& on_manual_override,
                              const std::function<bool()>& on_side_button,
                              const std::function<bool()>& on_left_button) {
    std::string resolved_path;
    std::string resolved_name;
    std::string resolve_error;
    if (!resolve_input_device(config, resolved_path, resolved_name, &resolve_error)) {
        std::fprintf(stderr, "[VR] %s\n", resolve_error.c_str());
        return false;
    }

    const int fd = open(resolved_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::fprintf(stderr, "[VR] failed to open %s: %s\n",
                     resolved_path.c_str(), std::strerror(errno));
        return false;
    }

    std::fprintf(stderr,
                 "[VR] input=%s name=%s mode=%s idle_stop_ms=%d speed_step=%d side_button=%s\n",
                 resolved_path.c_str(),
                 resolved_name.empty() ? "unknown" : resolved_name.c_str(),
                 config.log_only ? "log-only" : "drive",
                 config.idle_stop_ms,
                 config.speed_step,
                 side_button_action_name(config.side_button_action));
    std::fflush(stderr);

    VrTankDispatcher dispatcher(config, on_manual_override, on_side_button, on_left_button);

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
