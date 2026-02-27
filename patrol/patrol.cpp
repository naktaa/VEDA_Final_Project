// mpu6050_min.cpp : 최소 버전 (MPU-6050 gyroZ -> theta -> x,y)
// g++ -O2 -std=c++17 patrol.cpp -o patrol -lwiringPi
// sudo ./patrol

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <chrono>
#include <thread>
#include <fstream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <vector>

using namespace std;
// using namespace cv;
void sysfsWrite(string path, string value);


#define STOP 0
#define FORWARD 1
#define BACKWARD 2

#define TRIG_PIN  4   
#define ECHO_PIN  5   
#define IN1 28
#define IN2 27
#define ENA 29 // PWM enable
static constexpr double PI = 3.14159265358979323846;
static constexpr uint8_t MPU_ADDR = 0x68;

static int fd;

// [주의] Pi 5는 보통 pwmchip2를 사용합니다. ls /sys/class/pwm으로 확인하세요.
const string CHIP_PATH = "/sys/class/pwm/pwmchip0/";
const string PWM_PATH = CHIP_PATH + "pwm0/"; // BCM 18은 보통 pwm2 또는 pwm0

static void setPinConfig(int EN, int INA, int INB)
{
    pinMode(EN, OUTPUT);
    pinMode(INA, OUTPUT);
    pinMode(INB, OUTPUT);

    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);

    // soft PWM range 0~255
    if (softPwmCreate(EN, 0, 255) != 0)
    {
        printf("softPwmCreate 실패\n");
    }
}

static void setMotorControl(int EN, int INA, int INB, int speed, int stat)
{
    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    // speed (PWM on ENA)
    softPwmWrite(EN, speed);

    // direction
    if (stat == FORWARD)
    {
        digitalWrite(INA, LOW);
        digitalWrite(INB, HIGH);
    }
    else if (stat == BACKWARD)
    {
        digitalWrite(INA, HIGH);
        digitalWrite(INB, LOW);
    }
    else
    { // STOP (coast)
        softPwmWrite(EN, 0);
        digitalWrite(INA, LOW);
        digitalWrite(INB, LOW);
    }
}

// ======= Ultrasonic functions =======
static void setupUltrasonic()
{
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
    delay(50); // 센서 안정화
}

static void setServoUs(int us)
{
    if (us < 750) us = 750;
    if (us > 1750) us = 1750;
    sysfsWrite(PWM_PATH + "duty_cycle", to_string((long)us * 1000));
}

// return: distance in cm, <0이면 실패(타임아웃)
static double readDistanceCm(int timeout_us = 30000)
{
    // TRIG: 10us 펄스
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // ECHO가 HIGH 되는 시간 대기 (타임아웃)
    auto t_start_wait = chrono::high_resolution_clock::now();
    while (digitalRead(ECHO_PIN) == LOW)
    {
        auto dt = chrono::duration_cast<chrono::microseconds>(
                      chrono::high_resolution_clock::now() - t_start_wait)
                      .count();
        if (dt > timeout_us) return -1.0;
    }

    // HIGH 펄스 폭 측정
    auto t_echo_start = chrono::high_resolution_clock::now();
    while (digitalRead(ECHO_PIN) == HIGH)
    {
        auto dt = chrono::duration_cast<chrono::microseconds>(
                      chrono::high_resolution_clock::now() - t_echo_start)
                      .count();
        if (dt > timeout_us) return -1.0; // 너무 멀거나 에러
    }
    auto t_echo_end = chrono::high_resolution_clock::now();

    long echo_us = chrono::duration_cast<chrono::microseconds>(t_echo_end - t_echo_start).count();

    // 거리(cm) = echo_us / 58.0 (통상 근사식)
    double dist_cm = echo_us / 58.0;
    return dist_cm;
}

// 간단 필터: 3번 읽어서 유효값 중 최소(또는 평균)로 튐 완화
static double readDistanceCmFiltered()
{
    double d1 = readDistanceCm();
    delay(10);
    double d2 = readDistanceCm();
    delay(10);
    double d3 = readDistanceCm();

    double best = 1e9;
    if (d1 > 0) best = min(best, d1);
    if (d2 > 0) best = min(best, d2);
    if (d3 > 0) best = min(best, d3);

    if (best > 1e8) return -1.0;
    return best;
}
// ====================================

static double wrapAngle(double a){
    while(a > PI) a -= 2*PI;
    while(a < -PI) a += 2*PI;
    return a;
}
void sysfsWrite(string path, string value)
{
    ofstream ofs(path);
    if (ofs.is_open())
    {
        ofs << value;
        ofs.close();
    }
    else
    {
        cerr << "Error writing to: " << path << endl;
    }
}

void setupPWM()
{
    // 1. 채널 활성화 (이미 되어있을 수 있음)
    sysfsWrite(CHIP_PATH + "export", "0"); // BCM 18번에 대응하는 채널 번호
    usleep(200000);

    // 2. 주기 설정 (20ms = 20,000,000ns)
    sysfsWrite(PWM_PATH + "period", "20000000");

    // 3. 초기 중앙 위치 (1250us = 1,250,000ns)
    sysfsWrite(PWM_PATH + "duty_cycle", "1250000");

    // 4. 출력 시작
    sysfsWrite(PWM_PATH + "enable", "1");
    cout << "Hardware PWM started (20ms period, 1250us center)" << endl;
}

static void wreg(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = {reg, val};
    write(fd, b, 2);
}

static int16_t r16(uint8_t regH)
{
    uint8_t r = regH;
    write(fd, &r, 1);
    uint8_t b[2];
    read(fd, b, 2);
    return (int16_t)((b[0] << 8) | b[1]);
}

struct Pt { double x, y; };

int main()
{
    int flag = 0;
    setupPWM();
    // 1) I2C open + select slave
    fd = open("/dev/i2c-1", O_RDWR);
    ioctl(fd, I2C_SLAVE, MPU_ADDR);

    // 2) MPU wake + gyro ±250dps (sens=131 LSB/(deg/s))
    wreg(0x6B, 0x00); // PWR_MGMT_1 = 0
    // 이 레지스터의 6번 비트를 0으로 만들어줘야 센서가 깨어나서 일을 시작합니다.
    wreg(0x1B, 0x00); // GYRO_CONFIG FS=±250
    // +-250은 1초에 최대 250도까지 회전하는 걸 측정
    wreg(0x1A, 0x03); // DLPF 옵션(노이즈 줄이기)
    // Digital Low Pass Filter
    // 이 옵션을 켜면 급격한 변화를 부드럽게 깎아서 데이터가 튀는 걸 막아줍니다.
    double v = 0.2; // m/s (speed=150 캘리브레이션 값)
    const double hz = 200.0;
    const double dt_target = 1.0 / hz;
    if (wiringPiSetup() == -1)
    {
        printf("wiringPiSetup 실패\n");
        return 1;
    }
    setPinConfig(ENA, IN1, IN2);
    setupUltrasonic();
    // 3) bias calibration (2s)
    double bias = 0.0;
    int N = 0;
    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < 2.0)
    {
        int16_t raw = r16(0x47);         // GYRO_ZOUT_H
        double dps = raw / 131.0;        // deg/s
        double rad = dps * (PI / 180.0); // rad/s
        bias += rad;
        N++;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    bias /= (N > 0 ? N : 1);

    // 4) integrate
    double x = 0, y = 0, th = 0;
    auto last = std::chrono::steady_clock::now();
    auto lastPrint = last;

    int turn_state = 0;
    double turn_start_th = 0.0;
    // ===== Path (waypoints) =====


vector<Pt> path = {
    {1.0, 0.0},
    {1.0, 0.5},
    {1.5, 0.5},
    {1.5, 1.0},
    {-3.0, 1.0},
    {-3.0, 0.0},
    {0.0, 0.0}
};

int wp = 0;                 // 현재 목표 waypoint 인덱스
double tx = path[0].x;
double ty = path[0].y;
double prev_tx = 0;
double prev_ty = 0;

const double DIST_TOL = 0.10; // waypoint 도착 판정(10cm)
int drive_dir=STOP;
    while (true)
{
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last).count();
    last = now;

    if (dt <= 0.0) dt = 0.001;
    if (dt > 0.1)  dt = 0.1;

    // ===== 1) Gyro integrate (yaw) =====
    int16_t raw = r16(0x47);
    double omega = (raw / 131.0) * (PI / 180.0) - bias;  // rad/s
    th = wrapAngle(th + omega * dt);

    // ===== 2) Ultrasonic (stop only) =====
    double dist_cm = readDistanceCmFiltered();
    const double STOP_CM = 25.0;
    if (dist_cm > 0 && dist_cm < STOP_CM)
    {
        setMotorControl(ENA, IN1, IN2, 0, STOP);
        sysfsWrite(PWM_PATH + "duty_cycle", "1250000"); // 조향 중앙

        if (std::chrono::duration<double>(now - lastPrint).count() >= 0.2) {
            printf("[OBST] dist=%.1f cm -> STOP\n", dist_cm);
            lastPrint = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue; // 장애물 사라질 때까지 대기
    }

    // ===== 3) Dead-reckoning position integrate (nominal v) =====
    
    double ds = v * dt;
    if (drive_dir == BACKWARD) ds = -ds;
    x += ds * cos(th);
    y += ds * sin(th);

    // ===== 4) Waypoint error =====
    
    double dx = tx - x;
    double dy = ty - y;
    double dist = hypot(dx, dy);

    // ===== 5) Waypoint 도착 처리 =====
    if (dist < DIST_TOL)
    {
        wp++;
        if (wp >= (int)path.size())
        {
            setMotorControl(ENA, IN1, IN2, 0, STOP);
            sysfsWrite(PWM_PATH + "duty_cycle", "1250000");
            printf("PATH DONE! x=%.2f y=%.2f th=%.1fdeg\n", x, y, th * 180.0 / PI);
            break;
        }

        // 다음 목표로 갱신 (prev_tx/prev_ty 업데이트!)
        prev_tx = tx;
        prev_ty = ty;
        tx = path[wp].x;
        ty = path[wp].y;

        setMotorControl(ENA, IN1, IN2, 0, STOP);
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        continue;
    }

    // ===== 6) X overshoot 처리: 0.2m 지나치면 중앙 + BACKWARD =====
    const double X_OVERSHOOT = 0.20;  // 0.2m
    const int RECOV_SPEED = 90;       // 후진 PWM
    const int RECOV_MS = 1000;         // 후진 시간

    double dirx = tx - prev_tx;       // +면 x증가 구간, -면 x감소 구간
    bool overshoot_x = false;

    if (dirx >= 0 && (x - tx) >  X_OVERSHOOT) overshoot_x = true;
    if (dirx <  0 && (x - tx) < -X_OVERSHOOT) overshoot_x = true;

    if (overshoot_x)
    {
        sysfsWrite(PWM_PATH + "duty_cycle", "1250000");     // 조향 중앙
        setMotorControl(ENA, IN1, IN2, RECOV_SPEED, BACKWARD);
        std::this_thread::sleep_for(std::chrono::milliseconds(RECOV_MS));
         drive_dir = BACKWARD;2, 0, STOP);
        std::this_thread::sleep_for
        setMotorControl(ENA, IN1, IN(std::chrono::milliseconds(RECOV_MS));
        continue;
    }

    // ===== 7) Steering control toward waypoint =====
    double target_th = atan2(dy, dx);
    double err_th = wrapAngle(target_th - th);

    const double Kp = 1.2;
    const double maxSteer = 25.0 * PI / 180.0;

    double steer = Kp * err_th;
    if (steer >  maxSteer) steer =  maxSteer;
    if (steer < -maxSteer) steer = -maxSteer;

    int us = 1250 + (int)(steer / maxSteer * 500);
    if (us < 750)  us = 750;
    if (us > 1750) us = 1750;
    sysfsWrite(PWM_PATH + "duty_cycle", to_string((long)us * 1000));

    // ===== 8) Motor forward =====
    setMotorControl(ENA, IN1, IN2, 100, FORWARD);
    drive_dir = FORWARD;

    // ===== 9) Debug print =====
    if (std::chrono::duration<double>(now - lastPrint).count() >= 1.0)
    {
        printf("[wp=%d/%d] x=%.2f y=%.2f th=%.1fdeg | tx=%.2f ty=%.2f | dist=%.2f | us=%d | ultra=%.1fcm\n",
               wp, (int)path.size(),
               x, y, th * 180.0 / PI,
               tx, ty,
               dist, us, dist_cm);
        lastPrint = now;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(dt_target));
}
}