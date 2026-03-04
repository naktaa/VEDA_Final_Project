#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include <wiringPi.h>

namespace {
constexpr int STOP = 0;
constexpr int FORWARD = 1;
constexpr int BACKWARD = 2;

// wiringPi 핀 번호 (하드웨어 PWM 전환 대비 핀맵)
// EN: PWM 가능 GPIO 사용
//   L_EN -> GPIO18(wPi 1,  physical 12)
//   R_EN -> GPIO19(wPi 24, physical 35)
constexpr int L_IN1 = 28; // GPIO20, physical 38
constexpr int L_IN2 = 27; // GPIO16, physical 36
constexpr int L_EN = 1;   // GPIO18, physical 12

constexpr int R_IN1 = 25; // GPIO26, physical 37
constexpr int R_IN2 = 23; // GPIO13, physical 33
constexpr int R_EN = 24;  // GPIO19, physical 35

std::atomic<bool> g_run{true};
termios g_old_tio{};
bool g_term_ready = false;

void onSignal(int) {
    g_run = false;
}

int clampPwm(int v) {
    return std::max(0, std::min(255, v));
}

int toHwPwmDuty(int pwm_255) {
    constexpr int HW_RANGE = 1024;
    const int p = clampPwm(pwm_255);
    return (p * HW_RANGE) / 255;
}

void setMotorControl(int en, int in1, int in2, int pwm, int dir) {
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

void stopAll() {
    setMotorControl(L_EN, L_IN1, L_IN2, 0, STOP);
    setMotorControl(R_EN, R_IN1, R_IN2, 0, STOP);
}

void applyDrive(int left_cmd, int right_cmd, int pwm) {
    const int ldir = (left_cmd > 0) ? FORWARD : (left_cmd < 0 ? BACKWARD : STOP);
    const int rdir = (right_cmd > 0) ? FORWARD : (right_cmd < 0 ? BACKWARD : STOP);

    setMotorControl(L_EN, L_IN1, L_IN2, (ldir == STOP) ? 0 : pwm, ldir);
    setMotorControl(R_EN, R_IN1, R_IN2, (rdir == STOP) ? 0 : pwm, rdir);
}

void cleanupTerminal() {
    if (g_term_ready) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_old_tio);
        g_term_ready = false;
    }
}

bool setupTerminalRaw() {
    if (tcgetattr(STDIN_FILENO, &g_old_tio) != 0) {
        return false;
    }
    termios new_tio = g_old_tio;
    new_tio.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_tio) != 0) {
        return false;
    }
    g_term_ready = true;
    return true;
}

void printHelp() {
    std::printf("\n=== Keyboard Manual Drive ===\n");
    std::printf("W: 전진, S: 후진, A: 좌회전(제자리), D: 우회전(제자리)\n");
    std::printf("Q: 좌로 곡선 전진, E: 우로 곡선 전진\n");
    std::printf("Space/X: 정지\n");
    std::printf("+: 속도 증가, -: 속도 감소\n");
    std::printf("H: 도움말, ESC: 종료\n");
    std::printf("=============================\n\n");
}
} // namespace

int main() {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    if (wiringPiSetup() == -1) {
        std::printf("[ERR] wiringPiSetup failed\n");
        return 1;
    }

    pinMode(L_EN, PWM_OUTPUT);
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_EN, PWM_OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);

    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);

    // 하드웨어 PWM 전역 설정(양 채널 공통)
    // base 19.2MHz / clock(32) / range(1024) ~= 586Hz
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(32);
    pwmSetRange(1024);

    if (!setupTerminalRaw()) {
        std::printf("[ERR] terminal raw mode setup failed\n");
        return 1;
    }
    std::atexit(cleanupTerminal);

    int pwm = 255;
    int left_cmd = 0;
    int right_cmd = 0;
    auto last_motion_key_time = std::chrono::steady_clock::now();
    constexpr int motion_hold_timeout_ms = 120;

    stopAll();
    printHelp();
    std::printf("[INFO] 시작 PWM=%d\n", pwm);

    while (g_run) {
        char ch = 0;
        const ssize_t n = read(STDIN_FILENO, &ch, 1);
        if (n > 0) {
            if (ch >= 'A' && ch <= 'Z') {
                ch = static_cast<char>(ch - 'A' + 'a');
            }

            if (ch == 27) { // ESC
                g_run = false;
                break;
            }

            switch (ch) {
                case 'w':
                    pwm = 255;
                    left_cmd = 1; right_cmd = 1;
                    last_motion_key_time = std::chrono::steady_clock::now();
                    break;
                case 's':
                    pwm = 255;
                    left_cmd = -1; right_cmd = -1;
                    last_motion_key_time = std::chrono::steady_clock::now();
                    break;
                case 'a':
                    pwm = 255;
                    left_cmd = -1; right_cmd = 1;
                    last_motion_key_time = std::chrono::steady_clock::now();
                    break;
                case 'd':
                    pwm = 255;
                    left_cmd = 1; right_cmd = -1;
                    last_motion_key_time = std::chrono::steady_clock::now();
                    break;
                case 'q':
                    pwm = 255;
                    left_cmd = 0; right_cmd = 1;
                    last_motion_key_time = std::chrono::steady_clock::now();
                    break;
                case 'e':
                    pwm = 255;
                    left_cmd = 1; right_cmd = 0;
                    last_motion_key_time = std::chrono::steady_clock::now();
                    break;
                case 'x':
                case ' ': left_cmd = 0; right_cmd = 0; break;
                case '+':
                case '=':
                    pwm = clampPwm(pwm + 10);
                    std::printf("[PWM] %d\n", pwm);
                    break;
                case '-':
                case '_':
                    pwm = clampPwm(pwm - 10);
                    std::printf("[PWM] %d\n", pwm);
                    break;
                case 'h':
                    printHelp();
                    break;
                default:
                    break;
            }

            applyDrive(left_cmd, right_cmd, pwm);
            std::printf("[STATE] L=%d R=%d PWM=%d\n", left_cmd, right_cmd, pwm);
            std::fflush(stdout);
        }

        const auto now = std::chrono::steady_clock::now();
        const auto idle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_motion_key_time).count();
        if ((left_cmd != 0 || right_cmd != 0) && idle_ms > motion_hold_timeout_ms) {
            left_cmd = 0;
            right_cmd = 0;
            applyDrive(left_cmd, right_cmd, pwm);
            std::printf("[STATE] auto-stop (key released)\n");
            std::fflush(stdout);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    stopAll();
    cleanupTerminal();
    std::printf("\n[INFO] manual drive 종료\n");
    return 0;
}
