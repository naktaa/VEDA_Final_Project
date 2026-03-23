#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "tank_drive.hpp"
#include "vr_remote_input.hpp"

namespace {

std::atomic<bool> g_running{true};

enum class ParseResult {
    kOk,
    kHelp,
    kError,
};

void signal_handler(int) {
    g_running = false;
}

void print_usage(const char* prog) {
    fprintf(stderr, "Usage: %s [options]\n", prog);
    fprintf(stderr, "  --input-dev <path>     evdev path (default: /dev/input/event4)\n");
    fprintf(stderr, "  --idle-stop-ms <n>     auto-stop timeout in ms (default: 180)\n");
    fprintf(stderr, "  --speed-step <n>       speed adjustment step (default: 10)\n");
    fprintf(stderr, "  --log-only             print mapped commands without motor control\n");
    fprintf(stderr, "  --help, -h             show this help\n");
}

ParseResult parse_args(int argc, char* argv[], VrRemoteInputConfig& config) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--input-dev") == 0 && i + 1 < argc) {
            config.input_device = argv[++i];
        } else if (std::strcmp(argv[i], "--idle-stop-ms") == 0 && i + 1 < argc) {
            config.idle_stop_ms = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--speed-step") == 0 && i + 1 < argc) {
            config.speed_step = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--log-only") == 0) {
            config.log_only = true;
        } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return ParseResult::kHelp;
        } else {
            fprintf(stderr, "[VR] unknown arg: %s\n", argv[i]);
            print_usage(argv[0]);
            return ParseResult::kError;
        }
    }

    if (config.idle_stop_ms < 0) {
        fprintf(stderr, "[VR] idle timeout must be >= 0\n");
        return ParseResult::kError;
    }

    if (config.speed_step < 1) {
        fprintf(stderr, "[VR] speed step must be >= 1\n");
        return ParseResult::kError;
    }

    return ParseResult::kOk;
}

} // namespace

int main(int argc, char* argv[]) {
    VrRemoteInputConfig config;

    const ParseResult parse_result = parse_args(argc, argv, config);
    if (parse_result == ParseResult::kHelp) return 0;
    if (parse_result == ParseResult::kError) return 1;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    bool motor_ready = false;
    if (!config.log_only) {
        if (!tank_drive::init()) {
            return 1;
        }
        motor_ready = true;
        tank_drive::set_idle_autostop(true, config.idle_stop_ms);
    }

    const bool ok = run_vr_remote_input_loop(config, g_running);

    if (motor_ready) {
        tank_drive::stop();
        tank_drive::shutdown();
    }

    return ok ? 0 : 1;
}
