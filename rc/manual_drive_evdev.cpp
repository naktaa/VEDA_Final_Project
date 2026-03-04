// manual_drive_evdev.cpp : USB 키보드 동글을 직접 evdev로 읽는 수동주행
//
// 빌드:
//   g++ -O2 -std=c++17 manual_drive_evdev.cpp -o manual_drive_evdev -lwiringPi
//
// 실행 (root 또는 input 그룹 필요):
//   sudo ./manual_drive_evdev
//   sudo EIS_INPUT_EVENT=/dev/input/event3 ./manual_drive_evdev   (디바이스 지정)
//
// 키보드 자동 탐색: /dev/input/by-id/ 또는 /dev/input/by-path/ 에서
//   이름에 "event-kbd"가 포함된 디바이스를 자동으로 찾습니다.
//
// 조작:
//   W/↑: 전진, S/↓: 후진, A/←: 좌회전(제자리), D/→: 우회전(제자리)
//   Q: 좌 피봇, E: 우 피봇
//   Space/X: 정지
//   +/-: 속도 증감
//   H: 도움말, ESC: 종료
//
// 키를 떼면 즉시 정지 (stdin 방식의 타임아웃 대기 없음)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <linux/input.h>

#include <softPwm.h>
#include <wiringPi.h>

namespace {

// ======================== 상수 ========================

constexpr int STOP     = 0;
constexpr int FORWARD  = 1;
constexpr int BACKWARD = 2;

// wiringPi 핀 번호 (patrol_track / rc_control_node 과 동일)
constexpr int L_IN1 = 28;
constexpr int L_IN2 = 27;
constexpr int L_EN  = 29;

constexpr int R_IN1 = 25;
constexpr int R_IN2 = 24;
constexpr int R_EN  = 23;

std::atomic<bool> g_run{true};

// ======================== 유틸 ========================

void onSignal(int) {
    g_run = false;
}

int clampPwm(int v) {
    return std::max(0, std::min(255, v));
}

// ======================== 모터 제어 ========================

void setMotorControl(int en, int in1, int in2, int pwm, int dir) {
    pwm = clampPwm(pwm);
    softPwmWrite(en, pwm);

    if (dir == FORWARD) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else if (dir == BACKWARD) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        softPwmWrite(en, 0);
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

// ======================== 키보드 디바이스 탐색 ========================

std::string findKbdDevice() {
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

// ======================== 도움말 ========================

void printHelp() {
    std::printf("\n=== Manual Drive (evdev — USB 키보드 직접 입력) ===\n");
    std::printf("W/↑: 전진, S/↓: 후진\n");
    std::printf("A/←: 좌회전(제자리), D/→: 우회전(제자리)\n");
    std::printf("Q: 좌 피봇 전진, E: 우 피봇 전진\n");
    std::printf("Space/X: 정지\n");
    std::printf("+/-: 속도 증감\n");
    std::printf("H: 도움말, ESC: 종료\n");
    std::printf("키를 떼면 즉시 정지\n");
    std::printf("=================================================\n\n");
}

} // namespace

// ======================== main ========================

int main() {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // ---- wiringPi + 모터 초기화 ----
    if (wiringPiSetup() == -1) {
        std::printf("[ERR] wiringPiSetup 실패\n");
        return 1;
    }

    auto setupPin = [](int en, int in1, int in2) -> bool {
        pinMode(en,  OUTPUT);
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        return (softPwmCreate(en, 0, 255) == 0);
    };

    if (!setupPin(L_EN, L_IN1, L_IN2) || !setupPin(R_EN, R_IN1, R_IN2)) {
        std::printf("[ERR] softPwmCreate 실패 (EN 점퍼 제거 확인)\n");
        return 1;
    }

    stopAll();

    // ---- evdev 키보드 디바이스 열기 ----
    std::string dev_path;

    const char* env_dev = std::getenv("EIS_INPUT_EVENT");
    if (env_dev && env_dev[0] != '\0') {
        dev_path = env_dev;
    } else {
        dev_path = findKbdDevice();
    }

    if (dev_path.empty()) {
        std::printf("[ERR] USB 키보드 디바이스를 찾을 수 없습니다.\n");
        std::printf("      /dev/input/by-id/ 에 event-kbd 디바이스가 있는지 확인하세요.\n");
        std::printf("      또는 EIS_INPUT_EVENT 환경변수로 직접 지정하세요.\n");
        std::printf("      예: sudo EIS_INPUT_EVENT=/dev/input/event3 ./manual_drive_evdev\n");
        return 1;
    }

    int evfd = open(dev_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (evfd < 0) {
        std::perror("[ERR] evdev open");
        std::printf("      경로: %s\n", dev_path.c_str());
        std::printf("      sudo 로 실행하거나 input 그룹에 사용자를 추가하세요.\n");
        return 1;
    }

    // 디바이스 이름 출력
    char dev_name[256] = "Unknown";
    ioctl(evfd, EVIOCGNAME(sizeof(dev_name)), dev_name);
    std::printf("[INFO] 키보드 디바이스: %s\n", dev_path.c_str());
    std::printf("[INFO] 디바이스 이름: %s\n", dev_name);

    // ---- 주행 루프 ----
    int pwm = 255;
    int left_cmd  = 0;
    int right_cmd = 0;
    int active_key = 0; // 현재 눌려있는 이동 키

    auto applyAction = [&](int action_key) {
        switch (action_key) {
            case KEY_W: case KEY_UP:
                pwm = 255; left_cmd = 1;  right_cmd = 1;  break;
            case KEY_S: case KEY_DOWN:
                pwm = 255; left_cmd = -1; right_cmd = -1; break;
            case KEY_A: case KEY_LEFT:
                pwm = 255; left_cmd = -1; right_cmd = 1;  break;
            case KEY_D: case KEY_RIGHT:
                pwm = 255; left_cmd = 1;  right_cmd = -1; break;
            case KEY_Q:
                pwm = 255; left_cmd = 0;  right_cmd = 1;  break;
            case KEY_E:
                pwm = 255; left_cmd = 1;  right_cmd = 0;  break;
            default:     // STOP
                left_cmd = 0; right_cmd = 0; break;
        }
        applyDrive(left_cmd, right_cmd, pwm);
    };

    printHelp();
    std::printf("[INFO] 시작 PWM=%d\n", pwm);

    while (g_run) {
        struct pollfd pfd;
        pfd.fd      = evfd;
        pfd.events  = POLLIN;
        pfd.revents = 0;

        // 10ms 타임아웃 — CPU 부하 최소화
        const int pr = poll(&pfd, 1, 10);
        if (pr <= 0) continue;
        if (!(pfd.revents & POLLIN)) continue;

        struct input_event ev;
        ssize_t n = read(evfd, &ev, sizeof(ev));

        while (n == sizeof(ev)) {
            if (ev.type == EV_KEY) {
                const bool pressed = (ev.value != 0); // 1=press, 2=repeat
                const int  code    = ev.code;

                if (pressed) {
                    if (code == KEY_ESC) {
                        g_run = false;
                        break;
                    }

                    if (code == KEY_SPACE || code == KEY_X) {
                        active_key = 0;
                        applyAction(0);
                        std::printf("[STATE] 정지\n");
                    } else if (code == KEY_EQUAL || code == KEY_KPPLUS) {
                        pwm = clampPwm(pwm + 10);
                        std::printf("[PWM] %d\n", pwm);
                    } else if (code == KEY_MINUS || code == KEY_KPMINUS) {
                        pwm = clampPwm(pwm - 10);
                        std::printf("[PWM] %d\n", pwm);
                    } else if (code == KEY_H) {
                        printHelp();
                    } else {
                        // 이동 키 눌림
                        active_key = code;
                        applyAction(active_key);
                        std::printf("[STATE] L=%d R=%d PWM=%d\n",
                                    left_cmd, right_cmd, pwm);
                    }
                } else {
                    // 키를 떼면 → 해당 키가 active_key였으면 즉시 정지
                    if (code == active_key) {
                        active_key = 0;
                        applyAction(0);
                        std::printf("[STATE] 정지 (key released)\n");
                    }
                }
            }
            n = read(evfd, &ev, sizeof(ev));
        }

        std::fflush(stdout);
    }

    stopAll();
    close(evfd);
    std::printf("\n[INFO] manual_drive_evdev 종료\n");
    return 0;
}
