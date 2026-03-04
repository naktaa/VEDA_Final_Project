#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>

#include <softPwm.h>
#include <wiringPi.h>

namespace {
constexpr int STOP = 0;
constexpr int FORWARD = 1;
constexpr int BACKWARD = 2;

// wiringPi 핀 (patrol_track.cpp / rc/manual_drive.cpp와 동일)
constexpr int L_IN1 = 28;
constexpr int L_IN2 = 27;
constexpr int L_EN = 29;

constexpr int R_IN1 = 25;
constexpr int R_IN2 = 24;
constexpr int R_EN = 23;

std::atomic<bool> g_run{true};

void onSignal(int) {
    g_run = false;
}

void setMotorControl(int en, int in1, int in2, int pwm, int dir) {
    pwm = std::max(0, std::min(255, pwm));
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

bool waitWithAbort(int ms) {
    int elapsed = 0;
    while (elapsed < ms) {
        if (!g_run.load()) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        elapsed += 20;
    }
    return true;
}
} // namespace

int main() {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    if (wiringPiSetup() == -1) {
        std::printf("[ERR] wiringPiSetup failed\n");
        return 1;
    }

    pinMode(L_EN, OUTPUT);
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);

    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);

    if (softPwmCreate(L_EN, 0, 255) != 0 || softPwmCreate(R_EN, 0, 255) != 0) {
        std::printf("[ERR] softPwmCreate failed (EN 점퍼 제거 확인)\n");
        return 1;
    }

    const int pwm = 180;
    const int run_ms = 1500;
    const int gap_ms = 700;

    std::printf("=== Motor test start ===\n");
    std::printf("Ctrl+C to stop immediately.\n");

    std::printf("[1/8] Left FORWARD\n");
    setMotorControl(L_EN, L_IN1, L_IN2, pwm, FORWARD);
    if (!waitWithAbort(run_ms)) goto cleanup;
    stopAll(); if (!waitWithAbort(gap_ms)) goto cleanup;

    std::printf("[2/8] Left BACKWARD\n");
    setMotorControl(L_EN, L_IN1, L_IN2, pwm, BACKWARD);
    if (!waitWithAbort(run_ms)) goto cleanup;
    stopAll(); if (!waitWithAbort(gap_ms)) goto cleanup;

    std::printf("[3/8] Right FORWARD\n");
    setMotorControl(R_EN, R_IN1, R_IN2, pwm, FORWARD);
    if (!waitWithAbort(run_ms)) goto cleanup;
    stopAll(); if (!waitWithAbort(gap_ms)) goto cleanup;

    std::printf("[4/8] Right BACKWARD\n");
    setMotorControl(R_EN, R_IN1, R_IN2, pwm, BACKWARD);
    if (!waitWithAbort(run_ms)) goto cleanup;
    stopAll(); if (!waitWithAbort(gap_ms)) goto cleanup;

    std::printf("[5/8] Both FORWARD\n");
    setMotorControl(L_EN, L_IN1, L_IN2, pwm, FORWARD);
    setMotorControl(R_EN, R_IN1, R_IN2, pwm, FORWARD);
    if (!waitWithAbort(run_ms)) goto cleanup;
    stopAll(); if (!waitWithAbort(gap_ms)) goto cleanup;

    std::printf("[6/8] Both BACKWARD\n");
    setMotorControl(L_EN, L_IN1, L_IN2, pwm, BACKWARD);
    setMotorControl(R_EN, R_IN1, R_IN2, pwm, BACKWARD);
    if (!waitWithAbort(run_ms)) goto cleanup;
    stopAll(); if (!waitWithAbort(gap_ms)) goto cleanup;

    std::printf("[7/8] Spin LEFT (L back, R forward)\n");
    setMotorControl(L_EN, L_IN1, L_IN2, pwm, BACKWARD);
    setMotorControl(R_EN, R_IN1, R_IN2, pwm, FORWARD);
    if (!waitWithAbort(run_ms)) goto cleanup;
    stopAll(); if (!waitWithAbort(gap_ms)) goto cleanup;

    std::printf("[8/8] Spin RIGHT (L forward, R back)\n");
    setMotorControl(L_EN, L_IN1, L_IN2, pwm, FORWARD);
    setMotorControl(R_EN, R_IN1, R_IN2, pwm, BACKWARD);
    if (!waitWithAbort(run_ms)) goto cleanup;

cleanup:
    stopAll();
    std::printf("=== Motor test end ===\n");
    return 0;
}
