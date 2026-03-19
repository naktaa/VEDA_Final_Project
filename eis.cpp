#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <string>
#include <termios.h>
#include <thread>

#include <linux/input.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <wiringPi.h>

static std::atomic<bool> g_running{true};

static void request_shutdown() {
    g_running.store(false);
}

static void sigint_handler(int) {
    request_shutdown();
}

namespace manual_drive {
    constexpr int STOP = 0;
    constexpr int FORWARD = 1;
    constexpr int BACKWARD = 2;

    constexpr int BASE_SPEED_DEFAULT = 255;
    constexpr int BASE_SPEED_STEP = 10;
    constexpr int LOOP_SLEEP_MS = 5;
    constexpr int LOG_PERIOD_MS = 200;
    constexpr int STDIN_HOLD_MS_DEFAULT = 300;

    // 0.0 ~ 1.0 axis update rates (per second)
    constexpr double THROTTLE_ACCEL_RATE = 1.20;
    constexpr double THROTTLE_RELEASE_DECEL_RATE = 1.60;
    constexpr double BRAKE_DECEL_RATE = 4.50;
    constexpr double STEER_RATE = 3.50;
    constexpr int ROTATE_PWM = 180;

    // Mixing / PWM tuning
    constexpr double MIX_DEADZONE = 0.05;
    constexpr int MIN_EFFECTIVE_PWM = 105;
    constexpr bool INVERT_LEFT_MOTOR = false;
    constexpr bool INVERT_RIGHT_MOTOR = true;

    // wiringPi pin numbers (hardware PWM)
    //   L_EN -> GPIO18 (wPi 1,  physical 12) PWM0
    //   R_EN -> GPIO19 (wPi 24, physical 35) PWM1
    constexpr int L_IN1 = 28; // GPIO20, physical 38
    constexpr int L_IN2 = 27; // GPIO16, physical 36
    constexpr int L_EN = 1;   // GPIO18, physical 12 (HW PWM0)

    constexpr int R_IN1 = 25; // GPIO26, physical 37
    constexpr int R_IN2 = 23; // GPIO13, physical 33
    constexpr int R_EN = 24;  // GPIO19, physical 35 (HW PWM1)

    termios g_old_tio{};
    bool g_term_ready = false;
    bool g_motor_ready = false;

    struct DriveCommand {
        double target_throttle = 0.0; // -1.0 ~ 1.0
        double target_steer = 0.0;    // -1.0 ~ 1.0
        bool brake = false;
        bool estop = false;
    };

    struct DriveState {
        double current_throttle = 0.0; // -1.0 ~ 1.0
        double current_steer = 0.0;    // -1.0 ~ 1.0
        int left_pwm = 0;
        int right_pwm = 0;
        int left_dir = STOP;
        int right_dir = STOP;
    };

    struct DriveInputFlags {
        bool throttle_fwd = false;
        bool throttle_rev = false;
        bool steer_left = false;
        bool steer_right = false;
        bool brake = false;
        bool estop = false;
    };

    struct DriveRuntime {
        DriveCommand cmd{};
        DriveState state{};
        int base_speed = BASE_SPEED_DEFAULT;
        std::chrono::steady_clock::time_point last_tick = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point last_log = std::chrono::steady_clock::now();
    };

    struct StdinHoldState {
        int throttle_sign = 0; // -1, 0, +1
        int steer_sign = 0;    // -1, 0, +1
        bool brake = false;
        std::chrono::steady_clock::time_point last_throttle = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point last_steer = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point last_brake = std::chrono::steady_clock::now();
    };

    int clampPwm(int v) {
        return std::max(0, std::min(255, v));
    }

    double clampUnit(double v) {
        return std::max(-1.0, std::min(1.0, v));
    }

    double approach(double current, double target, double max_delta) {
        if (max_delta <= 0.0) return current;
        if (current < target) return std::min(current + max_delta, target);
        if (current > target) return std::max(current - max_delta, target);
        return current;
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
        }
        else if (dir == BACKWARD) {
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
        }
        else {
            pwmWrite(en, 0);
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
        }
    }

    void stopAll() {
        if (!g_motor_ready) return;
        setMotorControl(L_EN, L_IN1, L_IN2, 0, STOP);
        setMotorControl(R_EN, R_IN1, R_IN2, 0, STOP);
    }

    const char* dirName(int dir) {
        switch (dir) {
        case FORWARD:
            return "F";
        case BACKWARD:
            return "B";
        default:
            return "S";
        }
    }

    void applyDriveOutput(const DriveState& state) {
        auto mapDir = [](int dir, bool invert) {
            if (!invert) return dir;
            if (dir == FORWARD) return BACKWARD;
            if (dir == BACKWARD) return FORWARD;
            return STOP;
        };

        setMotorControl(L_EN, L_IN1, L_IN2, state.left_pwm, mapDir(state.left_dir, INVERT_LEFT_MOTOR));
        setMotorControl(R_EN, R_IN1, R_IN2, state.right_pwm, mapDir(state.right_dir, INVERT_RIGHT_MOTOR));
    }

    void setMotorFromMix(double mix, int base_speed, int& out_pwm, int& out_dir) {
        const double mag = std::abs(mix);
        if (base_speed <= 0 || mag < MIX_DEADZONE) {
            out_pwm = 0;
            out_dir = STOP;
            return;
        }

        // Deadzone + remap:
        // [deadzone, 1.0] -> [min_effective_pwm, base_speed]
        const int min_pwm = std::min(clampPwm(MIN_EFFECTIVE_PWM), clampPwm(base_speed));
        const double norm =
            (mag - MIX_DEADZONE) / std::max(1e-6, (1.0 - MIX_DEADZONE));
        const double t = std::max(0.0, std::min(1.0, norm));
        int pwm = static_cast<int>(std::lround(
            static_cast<double>(min_pwm) + t * static_cast<double>(base_speed - min_pwm)));
        pwm = clampPwm(pwm);
        out_pwm = pwm;
        out_dir = (mix >= 0.0) ? FORWARD : BACKWARD;
    }

    void computeTankOutput(DriveState& state, int base_speed) {
        const double throttle = clampUnit(state.current_throttle);
        double left_mix = throttle;
        double right_mix = throttle;

        const double max_mag = std::max(std::abs(left_mix), std::abs(right_mix));
        if (max_mag > 1.0) {
            left_mix /= max_mag;
            right_mix /= max_mag;
        }

        setMotorFromMix(clampUnit(left_mix), base_speed, state.left_pwm, state.left_dir);
        setMotorFromMix(clampUnit(right_mix), base_speed, state.right_pwm, state.right_dir);
    }

    void updateDriveState(const DriveCommand& cmd, DriveState& state, int base_speed, double dt_sec) {
        dt_sec = std::max(0.0, std::min(dt_sec, 0.1));

        if (cmd.estop) {
            state.current_throttle = 0.0;
            state.current_steer = 0.0;
            state.left_pwm = 0;
            state.right_pwm = 0;
            state.left_dir = STOP;
            state.right_dir = STOP;
            return;
        }

        // Brake is safety-priority over everything else.
        if (cmd.brake) {
            state.current_throttle =
                approach(state.current_throttle, 0.0, BRAKE_DECEL_RATE * dt_sec);
            state.current_steer = 0.0;
            computeTankOutput(state, base_speed);
            return;
        }

        // A/D pivot has priority over throttle logic: immediate strong rotate.
        const int rotate_dir = (cmd.target_steer > 0.0) ? 1 : ((cmd.target_steer < 0.0) ? -1 : 0);
        if (rotate_dir != 0) {
            state.current_throttle = 0.0;
            state.current_steer = static_cast<double>(rotate_dir);
            const int rpwm = clampPwm(ROTATE_PWM);
            if (rotate_dir < 0) { // left pivot
                state.left_dir = BACKWARD;
                state.right_dir = FORWARD;
            }
            else { // right pivot
                state.left_dir = FORWARD;
                state.right_dir = BACKWARD;
            }
            state.left_pwm = rpwm;
            state.right_pwm = rpwm;
            return;
        }

        const double target_throttle = clampUnit(cmd.brake ? 0.0 : cmd.target_throttle);

        double throttle_rate = THROTTLE_RELEASE_DECEL_RATE;
        if (std::abs(target_throttle) > 1e-6 && state.current_throttle * target_throttle >= 0.0 &&
                 std::abs(target_throttle) > std::abs(state.current_throttle)) {
            throttle_rate = THROTTLE_ACCEL_RATE;
        }

        state.current_throttle =
            approach(state.current_throttle, target_throttle, throttle_rate * dt_sec);
        state.current_steer = 0.0;

        computeTankOutput(state, base_speed);
    }

    void commandFromInput(const DriveInputFlags& input, DriveCommand& out_cmd) {
        const int throttle_axis = (input.throttle_fwd ? 1 : 0) - (input.throttle_rev ? 1 : 0);
        const int steer_axis = (input.steer_right ? 1 : 0) - (input.steer_left ? 1 : 0);

        out_cmd.target_throttle = static_cast<double>(throttle_axis);
        out_cmd.target_steer = static_cast<double>(steer_axis);
        out_cmd.brake = input.brake;
        out_cmd.estop = input.estop;
    }

    void logDriveState(const DriveRuntime& rt) {
        fprintf(stderr,
                "[MANUAL] tgt(T=%.2f,S=%.2f) cur(T=%.2f,S=%.2f) "
                "L(%s,%d) R(%s,%d) base=%d brake=%d estop=%d\n",
                rt.cmd.target_throttle, rt.cmd.target_steer, rt.state.current_throttle,
                rt.state.current_steer, dirName(rt.state.left_dir), rt.state.left_pwm,
                dirName(rt.state.right_dir), rt.state.right_pwm, rt.base_speed,
                rt.cmd.brake ? 1 : 0, rt.cmd.estop ? 1 : 0);
        fflush(stderr);
    }

    void tickDrive(DriveRuntime& rt, const DriveInputFlags& input, bool force_log = false) {
        commandFromInput(input, rt.cmd);

        const auto now = std::chrono::steady_clock::now();
        const double dt_sec = std::chrono::duration<double>(now - rt.last_tick).count();
        rt.last_tick = now;

        updateDriveState(rt.cmd, rt.state, rt.base_speed, dt_sec);
        applyDriveOutput(rt.state);

        const auto since_log =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - rt.last_log).count();
        if (force_log || since_log >= LOG_PERIOD_MS) {
            logDriveState(rt);
            rt.last_log = now;
        }
    }

    void adjustBaseSpeed(DriveRuntime& rt, int delta) {
        rt.base_speed = clampPwm(rt.base_speed + delta);
        fprintf(stderr, "[MANUAL] base speed=%d\n", rt.base_speed);
        fflush(stderr);
    }

    void cleanupTerminal() {
        if (g_term_ready) {
            tcsetattr(STDIN_FILENO, TCSANOW, &g_old_tio);
            g_term_ready = false;
        }
    }

    bool setupTerminalRaw() {
        if (!isatty(STDIN_FILENO)) {
            fprintf(stderr, "[MANUAL] stdin is not a TTY; manual drive disabled.\n");
            return false;
        }
        if (tcgetattr(STDIN_FILENO, &g_old_tio) != 0) return false;
        termios new_tio = g_old_tio;
        new_tio.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
        new_tio.c_cc[VMIN] = 0;
        new_tio.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &new_tio) != 0) return false;
        g_term_ready = true;
        return true;
    }

    bool setupMotor() {
        if (wiringPiSetup() == -1) {
            fprintf(stderr, "[MANUAL] wiringPiSetup failed; motor control disabled.\n");
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

        // base 19.2MHz / clock(32) / range(1024) ~= 586Hz
        pwmSetMode(PWM_MODE_MS);
        pwmSetClock(32);
        pwmSetRange(1024);

        g_motor_ready = true;
        stopAll();
        return true;
    }

    void printHelp() {
        fprintf(stderr, "\n=== Tank Manual Drive ===\n");
        fprintf(stderr, "W/S: target throttle +1 / -1 (hold to keep)\n");
        fprintf(stderr, "A/D (also Q/E): pivot turn left/right (in-place)\n");
        fprintf(stderr, "Space: brake (fast decel), X: emergency stop\n");
        fprintf(stderr, "+/-: base speed up/down (0~255)\n");
        fprintf(stderr, "H: help, ESC: quit\n");
        fprintf(stderr, "=========================\n\n");
    }

    void applyStdinHoldToInput(StdinHoldState& hold, DriveInputFlags& input,
                               std::chrono::steady_clock::time_point now,
                               int hold_timeout_ms, bool estop_pulse) {
        const auto throttle_idle =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - hold.last_throttle)
                .count();
        const auto steer_idle =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - hold.last_steer).count();
        const auto brake_idle =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - hold.last_brake).count();

        if (throttle_idle > hold_timeout_ms) hold.throttle_sign = 0;
        if (steer_idle > hold_timeout_ms) hold.steer_sign = 0;
        if (brake_idle > hold_timeout_ms) hold.brake = false;

        input = DriveInputFlags{};
        input.throttle_fwd = (hold.throttle_sign > 0);
        input.throttle_rev = (hold.throttle_sign < 0);
        input.steer_left = (hold.steer_sign < 0);
        input.steer_right = (hold.steer_sign > 0);
        input.brake = hold.brake;
        input.estop = estop_pulse;
    }

    bool run_evdev(const char* dev_path) {
        int fd = open(dev_path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            perror("[MANUAL] evdev open");
            return false;
        }

        fprintf(stderr, "[MANUAL] evdev input: %s\n", dev_path);

        DriveRuntime rt{};
        DriveInputFlags input{};

        stopAll();
        printHelp();
        fprintf(stderr, "[MANUAL] start base speed=%d (evdev)\n", rt.base_speed);

        while (g_running.load()) {
            struct pollfd pfd;
            pfd.fd = fd;
            pfd.events = POLLIN;
            pfd.revents = 0;

            const int pr = poll(&pfd, 1, 10);
            if (pr > 0 && (pfd.revents & POLLIN)) {
                struct input_event ev;
                ssize_t n = read(fd, &ev, sizeof(ev));
                while (n == sizeof(ev)) {
                    if (ev.type == EV_KEY) {
                        const bool pressed = (ev.value != 0); // 1=press, 2=repeat, 0=release
                        const int code = ev.code;

                        if (pressed && code == KEY_ESC) {
                            request_shutdown();
                            break;
                        }

                        switch (code) {
                        case KEY_W:
                        case KEY_UP:
                            input.throttle_fwd = pressed;
                            break;
                        case KEY_S:
                        case KEY_DOWN:
                            input.throttle_rev = pressed;
                            break;
                        case KEY_A:
                        case KEY_LEFT:
                        case KEY_Q:
                            input.steer_left = pressed;
                            break;
                        case KEY_D:
                        case KEY_RIGHT:
                        case KEY_E:
                            input.steer_right = pressed;
                            break;
                        case KEY_SPACE:
                            input.brake = pressed;
                            break;
                        case KEY_X:
                            input.estop = pressed;
                            break;
                        default:
                            if (!pressed) break;
                            if (code == KEY_EQUAL || code == KEY_KPPLUS) {
                                adjustBaseSpeed(rt, BASE_SPEED_STEP);
                            }
                            else if (code == KEY_MINUS || code == KEY_KPMINUS) {
                                adjustBaseSpeed(rt, -BASE_SPEED_STEP);
                            }
                            else if (code == KEY_H) {
                                printHelp();
                            }
                            break;
                        }
                    }
                    n = read(fd, &ev, sizeof(ev));
                }
            }

            tickDrive(rt, input);
        }

        stopAll();
        fprintf(stderr, "[MANUAL] stopped (evdev)\n");
        close(fd);
        return true;
    }

    std::string find_kbd_device() {
        const char* dirs[] = {"/dev/input/by-id", "/dev/input/by-path"};
        for (const char* dir_path : dirs) {
            DIR* dir = opendir(dir_path);
            if (!dir) continue;
            while (true) {
                struct dirent* ent = readdir(dir);
                if (!ent) break;
                const std::string name = ent->d_name ? ent->d_name : "";
                if (name.find("event-kbd") == std::string::npos) continue;
                const std::string full = std::string(dir_path) + "/" + name;
                closedir(dir);
                return full;
            }
            closedir(dir);
        }
        return {};
    }

    void run() {
        setupMotor();

        const char* evdev_env_path = std::getenv("MANUAL_EVDEV_PATH");
        if (evdev_env_path && evdev_env_path[0] != '\0') {
            if (run_evdev(evdev_env_path)) return;
            fprintf(stderr, "[MANUAL] MANUAL_EVDEV_PATH open failed, fallback to stdin.\n");
        }

        bool try_evdev = false;
        if (const char* env = std::getenv("MANUAL_USE_EVDEV")) {
            try_evdev = (std::atoi(env) != 0);
        }
        if (try_evdev) {
            const std::string dev_path = find_kbd_device();
            if (!dev_path.empty()) {
                if (run_evdev(dev_path.c_str())) return;
            }
            fprintf(stderr, "[MANUAL] evdev keyboard not found, fallback to stdin.\n");
        }

        if (!setupTerminalRaw()) {
            stopAll();
            return;
        }

        DriveRuntime rt{};
        StdinHoldState hold{};
        DriveInputFlags input{};
        bool estop_pulse = false;

        int motion_hold_timeout_ms = STDIN_HOLD_MS_DEFAULT;
        if (const char* env = std::getenv("MANUAL_HOLD_MS")) {
            const int v = std::atoi(env);
            if (v >= 20 && v <= 1000) motion_hold_timeout_ms = v;
        }

        stopAll();
        printHelp();
        fprintf(stderr, "[MANUAL] stdin control\n");
        fprintf(stderr, "[MANUAL] start base speed=%d\n", rt.base_speed);

        while (g_running.load()) {
            char ch = 0;
            const ssize_t n = read(STDIN_FILENO, &ch, 1);
            if (n > 0) {
                if (ch >= 'A' && ch <= 'Z') ch = static_cast<char>(ch - 'A' + 'a');

                if (ch == 27) { // ESC or arrow key sequence
                    char seq1 = 0;
                    char seq2 = 0;
                    const ssize_t n1 = read(STDIN_FILENO, &seq1, 1);
                    if (n1 == 1 && seq1 == '[') {
                        const ssize_t n2 = read(STDIN_FILENO, &seq2, 1);
                        if (n2 == 1) {
                            switch (seq2) {
                            case 'A':
                                ch = 'w';
                                break;
                            case 'B':
                                ch = 's';
                                break;
                            case 'C':
                                ch = 'd';
                                break;
                            case 'D':
                                ch = 'a';
                                break;
                            default:
                                ch = 27;
                                break;
                            }
                        }
                    }
                    if (ch == 27) {
                        request_shutdown();
                        break;
                    }
                }

                const auto key_time = std::chrono::steady_clock::now();
                switch (ch) {
                case 'w':
                    hold.throttle_sign = 1;
                    hold.last_throttle = key_time;
                    break;
                case 's':
                    hold.throttle_sign = -1;
                    hold.last_throttle = key_time;
                    break;
                case 'a':
                case 'q':
                    hold.steer_sign = -1;
                    hold.last_steer = key_time;
                    break;
                case 'd':
                case 'e':
                    hold.steer_sign = 1;
                    hold.last_steer = key_time;
                    break;
                case ' ':
                    hold.brake = true;
                    hold.last_brake = key_time;
                    break;
                case 'x':
                    hold.throttle_sign = 0;
                    hold.steer_sign = 0;
                    hold.brake = true;
                    hold.last_brake = key_time;
                    estop_pulse = true;
                    break;
                case '+':
                case '=':
                    adjustBaseSpeed(rt, BASE_SPEED_STEP);
                    break;
                case '-':
                case '_':
                    adjustBaseSpeed(rt, -BASE_SPEED_STEP);
                    break;
                case 'h':
                    printHelp();
                    break;
                default:
                    break;
                }
            }

            const auto now = std::chrono::steady_clock::now();
            applyStdinHoldToInput(hold, input, now, motion_hold_timeout_ms, estop_pulse);
            estop_pulse = false;
            tickDrive(rt, input);

            std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_SLEEP_MS));
        }

        stopAll();
        cleanupTerminal();
        fprintf(stderr, "[MANUAL] stopped\n");
    }
} // namespace manual_drive

int main() {
    std::signal(SIGINT, sigint_handler);
    fprintf(stderr, "[MAIN] Tank-only manual drive start\n");
    manual_drive::run();
    fprintf(stderr, "[MAIN] Tank-only manual drive exit\n");
    return 0;
}
