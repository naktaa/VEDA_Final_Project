// patrol_track.cpp : 궤도차량(탱크) 자율주행
// 차동 구동(Differential Drive) — 서보 제거, 좌우 독립 모터 제어
//
// g++ -O2 -std=c++17 patrol_track.cpp -o patrol_track -lwiringPi
// sudo ./patrol_track
//
// 하드웨어:
//   MPU-6050 (I2C)     → yaw 각도 추정
//   HC-SR04 (초음파)   → 전방 장애물 감지
//   L298N × 2          → 좌/우 트랙 독립 제어
//
// 핀 배치 (wiringPi 번호):
//   왼쪽 트랙:  L_EN=29  L_IN1=28  L_IN2=27
//   오른쪽 트랙: R_EN=23  R_IN1=25  R_IN2=24
//   초음파:     TRIG=4   ECHO=5

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <vector>

using namespace std;

// ======================== 상수 ========================

static constexpr double PI       = 3.14159265358979323846;
static constexpr uint8_t MPU_ADDR = 0x68;

#define STOP     0
#define FORWARD  1
#define BACKWARD 2

// 초음파
#define TRIG_PIN  4
#define ECHO_PIN  5

// 왼쪽 트랙 (L298N #1)
#define L_IN1  28
#define L_IN2  27
#define L_EN   29

// 오른쪽 트랙 (L298N #2)
#define R_IN1  25
#define R_IN2  24
#define R_EN   23

// 주행 파라미터
static const double NOMINAL_SPEED_MS  = 0.18;  // 추정 직진 속도 (m/s), 실측 후 조정
static const int    BASE_PWM          = 100;    // 기본 PWM (0~255)
static const int    SPIN_PWM          = 80;     // 제자리 회전 PWM
static const double DIST_TOL         = 0.10;   // waypoint 도착 판정 (m)
static const double STOP_CM          = 25.0;   // 장애물 정지 거리 (cm)
static const double SPIN_THRESHOLD   = 25.0 * PI / 180.0;  // 제자리 회전 시작 각도 (rad)
static const double Kp_steer         = 1.2;    // 조향 P게인
static const double MAX_STEER        = 30.0 * PI / 180.0;  // 최대 조향 각도 (rad)
static const double X_OVERSHOOT      = 0.20;   // x 오버슈트 판정 (m)
static const double LOOP_HZ          = 200.0;  // 제어 루프 주파수

// ======================== 전역 ========================

static int fd;  // I2C 파일 디스크립터

struct Pt { double x, y; };

// ======================== 유틸 ========================

static double wrapAngle(double a) {
    while (a >  PI) a -= 2 * PI;
    while (a < -PI) a += 2 * PI;
    return a;
}

// ======================== I2C / MPU-6050 ========================

static void wreg(uint8_t reg, uint8_t val) {
    uint8_t b[2] = {reg, val};
    write(fd, b, 2);
}

static int16_t r16(uint8_t regH) {
    uint8_t r = regH;
    write(fd, &r, 1);
    uint8_t b[2];
    read(fd, b, 2);
    return (int16_t)((b[0] << 8) | b[1]);
}

// ======================== 모터 제어 ========================

static void setPinConfig(int en, int in1, int in2) {
    pinMode(en,  OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    if (softPwmCreate(en, 0, 255) != 0)
        printf("[WARN] softPwmCreate 실패 (en=%d)\n", en);
}

static void setMotorControl(int en, int in1, int in2, int speed, int dir) {
    speed = max(0, min(255, speed));
    softPwmWrite(en, speed);
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

// 좌우 동시 정지
static void stopAll() {
    setMotorControl(L_EN, L_IN1, L_IN2, 0, STOP);
    setMotorControl(R_EN, R_IN1, R_IN2, 0, STOP);
}

// 차동 구동: steer -1.0(좌) ~ 0(직진) ~ +1.0(우)
static void setDifferential(int base_speed, double steer) {
    steer = max(-1.0, min(1.0, steer));

    int lspeed = (int)(base_speed * (1.0 - steer));  // 우회전이면 왼쪽 빠름
    int rspeed = (int)(base_speed * (1.0 + steer));  // 우회전이면 오른쪽 느림

    lspeed = max(-255, min(255, lspeed));
    rspeed = max(-255, min(255, rspeed));

    int ldir = (lspeed >= 0) ? FORWARD : BACKWARD;
    int rdir = (rspeed >= 0) ? FORWARD : BACKWARD;

    setMotorControl(L_EN, L_IN1, L_IN2, abs(lspeed), ldir);
    setMotorControl(R_EN, R_IN1, R_IN2, abs(rspeed), rdir);
}

// 제자리 회전: err_th > 0 → 좌회전(반시계), err_th < 0 → 우회전(시계)
static void spinTurn(double err_th) {
    if (err_th > 0) {
        // 좌회전: 왼쪽 후진, 오른쪽 전진
        setMotorControl(L_EN, L_IN1, L_IN2, SPIN_PWM, BACKWARD);
        setMotorControl(R_EN, R_IN1, R_IN2, SPIN_PWM, FORWARD);
    } else {
        // 우회전: 왼쪽 전진, 오른쪽 후진
        setMotorControl(L_EN, L_IN1, L_IN2, SPIN_PWM, FORWARD);
        setMotorControl(R_EN, R_IN1, R_IN2, SPIN_PWM, BACKWARD);
    }
}

// ======================== 초음파 ========================

static void setupUltrasonic() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
    delay(50);
}

static double readDistanceCm(int timeout_us = 30000) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    auto t0 = chrono::high_resolution_clock::now();
    while (digitalRead(ECHO_PIN) == LOW) {
        if (chrono::duration_cast<chrono::microseconds>(
                chrono::high_resolution_clock::now() - t0).count() > timeout_us)
            return -1.0;
    }
    auto t1 = chrono::high_resolution_clock::now();
    while (digitalRead(ECHO_PIN) == HIGH) {
        if (chrono::duration_cast<chrono::microseconds>(
                chrono::high_resolution_clock::now() - t1).count() > timeout_us)
            return -1.0;
    }
    auto t2 = chrono::high_resolution_clock::now();
    long echo_us = chrono::duration_cast<chrono::microseconds>(t2 - t1).count();
    return echo_us / 58.0;
}

// 3회 측정 후 유효값 최솟값 반환
static double readDistanceCmFiltered() {
    double d[3];
    for (int i = 0; i < 3; i++) {
        d[i] = readDistanceCm();
        delay(10);
    }
    double best = 1e9;
    for (int i = 0; i < 3; i++)
        if (d[i] > 0) best = min(best, d[i]);
    return (best > 1e8) ? -1.0 : best;
}

// ======================== main ========================

int main() {
    // wiringPi 초기화
    if (wiringPiSetup() == -1) {
        printf("[ERR] wiringPiSetup 실패\n");
        return 1;
    }

    // 모터 핀 설정 (좌/우 독립)
    setPinConfig(L_EN, L_IN1, L_IN2);
    setPinConfig(R_EN, R_IN1, R_IN2);
    setupUltrasonic();

    // I2C open + MPU-6050 초기화
    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) { perror("[ERR] i2c open"); return 1; }
    ioctl(fd, I2C_SLAVE, MPU_ADDR);

    wreg(0x6B, 0x00);  // PWR_MGMT_1: 슬립 해제
    wreg(0x1B, 0x00);  // GYRO_CONFIG: ±250dps
    wreg(0x1A, 0x03);  // DLPF: 노이즈 감소

    // ── bias 캘리브레이션 (2초) ──
    printf("[IMU] Calibrating bias (2s)...\n");
    double bias = 0.0;
    int N = 0;
    auto t0 = chrono::steady_clock::now();
    while (chrono::duration<double>(chrono::steady_clock::now() - t0).count() < 2.0) {
        int16_t raw = r16(0x47);  // GYRO_ZOUT_H
        bias += (raw / 131.0) * (PI / 180.0);
        N++;
        this_thread::sleep_for(chrono::milliseconds(5));
    }
    bias /= (N > 0 ? N : 1);
    printf("[IMU] bias=%.6f rad/s  (N=%d)\n", bias, N);

    // ── Waypoint 경로 정의 ──
    vector<Pt> path = {
        {1.0,  0.0},
        {1.0,  0.5},
        {1.5,  0.5},
        {1.5,  1.0},
        {-3.0, 1.0},
        {-3.0, 0.0},
        {0.0,  0.0}
    };

    int    wp      = 0;
    double tx      = path[0].x;
    double ty      = path[0].y;
    double prev_tx = 0.0;
    double prev_ty = 0.0;

    // 상태 변수
    double x  = 0.0, y  = 0.0, th = 0.0;
    int drive_dir = STOP;

    auto last      = chrono::steady_clock::now();
    auto lastPrint = last;

    const double dt_target = 1.0 / LOOP_HZ;

    printf("[START] 경로 주행 시작. waypoints=%d\n", (int)path.size());

    // ======================== 메인 루프 ========================
    while (true) {
        auto now = chrono::steady_clock::now();
        double dt = chrono::duration<double>(now - last).count();
        last = now;
        if (dt <= 0.0) dt = 0.001;
        if (dt > 0.1)  dt = 0.1;

        // ── 1) Gyro 적분 → yaw(th) ──
        int16_t raw = r16(0x47);
        double omega = (raw / 131.0) * (PI / 180.0) - bias;  // rad/s
        th = wrapAngle(th + omega * dt);

        // ── 2) 초음파 장애물 감지 ──
        double dist_cm = readDistanceCmFiltered();
        if (dist_cm > 0 && dist_cm < STOP_CM) {
            stopAll();
            if (chrono::duration<double>(now - lastPrint).count() >= 0.5) {
                printf("[OBST] dist=%.1f cm → STOP\n", dist_cm);
                lastPrint = now;
            }
            this_thread::sleep_for(chrono::milliseconds(50));
            continue;
        }

        // ── 3) Dead-reckoning 위치 추정 ──
        double ds = NOMINAL_SPEED_MS * dt;
        if (drive_dir == BACKWARD) ds = -ds;
        if (drive_dir == STOP)     ds = 0.0;
        x += ds * cos(th);
        y += ds * sin(th);

        // ── 4) Waypoint 오차 ──
        double dx   = tx - x;
        double dy   = ty - y;
        double dist = hypot(dx, dy);

        // ── 5) Waypoint 도착 처리 ──
        if (dist < DIST_TOL) {
            wp++;
            if (wp >= (int)path.size()) {
                stopAll();
                printf("[DONE] 경로 완료! x=%.2f y=%.2f th=%.1fdeg\n",
                       x, y, th * 180.0 / PI);
                break;
            }
            prev_tx = tx;
            prev_ty = ty;
            tx = path[wp].x;
            ty = path[wp].y;
            printf("[WP] → waypoint %d (%.2f, %.2f)\n", wp, tx, ty);
            stopAll();
            drive_dir = STOP;
            this_thread::sleep_for(chrono::milliseconds(200));
            continue;
        }

        // ── 6) X 오버슈트 복구 ──
        double dirx = tx - prev_tx;
        bool overshoot_x = false;
        if (dirx >= 0 && (x - tx) >  X_OVERSHOOT) overshoot_x = true;
        if (dirx <  0 && (x - tx) < -X_OVERSHOOT) overshoot_x = true;

        if (overshoot_x) {
            printf("[OVERSHOOT] x=%.2f tx=%.2f → 후진 복구\n", x, tx);
            // 후진 (양쪽 동시)
            setMotorControl(L_EN, L_IN1, L_IN2, 90, BACKWARD);
            setMotorControl(R_EN, R_IN1, R_IN2, 90, BACKWARD);
            drive_dir = BACKWARD;
            this_thread::sleep_for(chrono::milliseconds(800));
            stopAll();
            drive_dir = STOP;
            this_thread::sleep_for(chrono::milliseconds(150));
            continue;
        }

        // ── 7) 조향 제어 (차동 구동) ──
        double target_th = atan2(dy, dx);
        double err_th    = wrapAngle(target_th - th);

        if (fabs(err_th) > SPIN_THRESHOLD) {
            // 각도 오차가 크면 제자리 회전으로 방향 먼저 맞춤
            spinTurn(err_th);
            drive_dir = STOP;  // 제자리 회전 중 위치 적분 안 함
        } else {
            // 방향 맞으면 전진 + 미세 조향
            double steer = Kp_steer * err_th / MAX_STEER;
            steer = max(-1.0, min(1.0, steer));
            setDifferential(BASE_PWM, steer);
            drive_dir = FORWARD;
        }

        // ── 8) 디버그 출력 (1초마다) ──
        if (chrono::duration<double>(now - lastPrint).count() >= 1.0) {
            printf("[wp=%d/%d] x=%.2f y=%.2f th=%+.1fdeg | "
                   "tx=%.2f ty=%.2f dist=%.2fm | "
                   "err_th=%+.1fdeg | ultra=%.1fcm\n",
                   wp, (int)path.size(),
                   x, y, th * 180.0 / PI,
                   tx, ty, dist,
                   err_th * 180.0 / PI,
                   dist_cm);
            lastPrint = now;
        }

        this_thread::sleep_for(chrono::duration<double>(dt_target));
    }

    close(fd);
    return 0;
}
