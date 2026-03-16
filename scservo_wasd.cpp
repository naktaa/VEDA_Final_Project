// scservo_wasd.cpp
// WASD keyboard control for SCServo bus servos using SCServo_Linux library.
// Build with CMake (see CMakeLists.txt) or g++ directly.

#include <iostream>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include "SCServo.h"

// ===== CONFIG (edit here) =====
static const char* DEV = "/dev/serial0";
static const int BAUD = 115200;

static const int PAN_ID = 1;
static const int TILT_ID = 2;

static const int PAN_CENTER  = 1800;
static const int TILT_CENTER = 2400;

static const int PAN_MIN  = 1000;
static const int PAN_MAX  = 3200;
static const int TILT_MIN = 1600;
static const int TILT_MAX = 3400;

static const int STEP = 50;

// SCSCL params
static const int SCSCL_TIME  = 0;
static const int SCSCL_SPEED = 1500;

// SMS_STS params
static const int STS_SPEED = 2400;
static const int STS_ACC   = 50;
// ===============================

#ifndef USE_SMS_STS
#define USE_SMS_STS 1
#endif

static int clampValue(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

class TerminalRawMode {
public:
    TerminalRawMode() {
        if (tcgetattr(STDIN_FILENO, &old_) == -1) {
            throw std::runtime_error("tcgetattr failed");
        }
        raw_ = old_;
        raw_.c_lflag &= ~(ICANON | ECHO);
        raw_.c_cc[VMIN]  = 0;
        raw_.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw_) == -1) {
            throw std::runtime_error("tcsetattr failed");
        }
    }
    ~TerminalRawMode() { tcsetattr(STDIN_FILENO, TCSANOW, &old_); }
private:
    termios old_{};
    termios raw_{};
};

static char getKeyNonBlocking() {
    fd_set set;
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    int rv = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv);
    if (rv > 0 && FD_ISSET(STDIN_FILENO, &set)) {
        char ch = 0;
        if (read(STDIN_FILENO, &ch, 1) == 1) return ch;
    }
    return 0;
}

int main() {
#if USE_SMS_STS
    SMS_STS servo;
#else
    SCSCL servo;
#endif

    if (!servo.begin(BAUD, DEV)) {
        std::cerr << "Failed to open " << DEV << "\n";
        return 1;
    }

    TerminalRawMode rawMode;

    servo.EnableTorque(PAN_ID, 1);
    servo.EnableTorque(TILT_ID, 1);

    int pan = PAN_CENTER;
    int tilt = TILT_CENTER;

    std::cout << "=== WASD PAN/TILT ===\n";
#if USE_SMS_STS
    std::cout << "Mode: SMS_STS\n";
#else
    std::cout << "Mode: SCSCL\n";
#endif
    std::cout << "W/S tilt, A/D pan, SPACE center, P print, Q quit\n";

    auto sendPan = [&](int v) {
#if USE_SMS_STS
        servo.WritePosEx(PAN_ID, v, STS_SPEED, STS_ACC);
#else
        servo.WritePos(PAN_ID, v, SCSCL_TIME, SCSCL_SPEED);
#endif
    };
    auto sendTilt = [&](int v) {
#if USE_SMS_STS
        servo.WritePosEx(TILT_ID, v, STS_SPEED, STS_ACC);
#else
        servo.WritePos(TILT_ID, v, SCSCL_TIME, SCSCL_SPEED);
#endif
    };

    sendPan(pan);
    sendTilt(tilt);

    while (true) {
        char key = getKeyNonBlocking();
        if (!key) continue;

        bool movedPan = false;
        bool movedTilt = false;

        if (key == 'a' || key == 'A') { pan = clampValue(pan + STEP, PAN_MIN, PAN_MAX); movedPan = true; }
        else if (key == 'd' || key == 'D') { pan = clampValue(pan - STEP, PAN_MIN, PAN_MAX); movedPan = true; }
        else if (key == 'w' || key == 'W') { tilt = clampValue(tilt + STEP, TILT_MIN, TILT_MAX); movedTilt = true; }
        else if (key == 's' || key == 'S') { tilt = clampValue(tilt - STEP, TILT_MIN, TILT_MAX); movedTilt = true; }
        else if (key == ' ') { pan = PAN_CENTER; tilt = TILT_CENTER; movedPan = movedTilt = true; }
        else if (key == 'p' || key == 'P') {
            std::cout << "\nPAN=" << pan << " TILT=" << tilt << "\n";
        }
        else if (key == 'q' || key == 'Q') {
            std::cout << "\nQuit\n";
            break;
        }

        if (movedPan) sendPan(pan);
        if (movedTilt) sendTilt(tilt);
        if (movedPan || movedTilt) {
            std::cout << "\rPAN=" << pan << "  TILT=" << tilt << "      " << std::flush;
        }
        usleep(30000);
    }

    servo.end();
    return 0;
}
