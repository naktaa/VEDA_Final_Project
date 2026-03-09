//다익스트라 알고리즘으로 벽(장애물) 라인 피해서 가는 코드
// g++ -O2 -std=c++17 daik_tank.cpp -o daik -lwiringPi

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
#include <queue>
#include <algorithm>
#include <limits>

using namespace std;

#define STOP 0
#define FORWARD 1
#define BACKWARD 2

// ===== 모터 핀 (네 기존 코드) =====
#define IN1 23
#define IN2 24
#define ENA 25
#define IN3 27
#define IN4 28
#define ENB 29

static constexpr double PI = 3.14159265358979323846;
static constexpr uint8_t MPU_ADDR = 0x68;
static int fd;

// ===================== 기존 모터/자이로 함수들 =====================
static void setPinConfig(int EN, int INA, int INB)
{
    pinMode(EN, OUTPUT);
    pinMode(INA, OUTPUT);
    pinMode(INB, OUTPUT);

    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);

    if (softPwmCreate(EN, 0, 255) != 0)
        printf("softPwmCreate 실패\n");
}

static void setMotorControl(int EN, int INA, int INB, int speed, int stat)
{
    if (speed < 0)
        speed = 0;
    if (speed > 255)
        speed = 255;

    softPwmWrite(EN, speed);

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
    {
        softPwmWrite(EN, 0);
        digitalWrite(INA, LOW);
        digitalWrite(INB, LOW);
    }
}

static void tankStop()
{
    setMotorControl(ENA, IN1, IN2, 0, STOP);
    setMotorControl(ENB, IN3, IN4, 0, STOP);
}

static void tankForward(int speed)
{
    setMotorControl(ENA, IN1, IN2, speed, FORWARD);
    setMotorControl(ENB, IN3, IN4, speed, FORWARD); // 필요시 반대로
}

static void tankSpinLeft(int speed)
{
    setMotorControl(ENA, IN1, IN2, speed, BACKWARD);
    setMotorControl(ENB, IN3, IN4, speed, FORWARD);
}

static void tankSpinRight(int speed)
{
    setMotorControl(ENA, IN1, IN2, speed, FORWARD);
    setMotorControl(ENB, IN3, IN4, speed, BACKWARD);
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

static double wrapAngle(double a)
{
    while (a > PI)
        a -= 2.0 * PI;
    while (a < -PI)
        a += 2.0 * PI;
    return a;
}

// ===================== Dijkstra용 구조체/함수 =====================
struct Cell
{
    int x, y;
};

struct Node
{
    int x, y;
    double cost;
    bool operator>(const Node &o) const { return cost > o.cost; }
};

static const double INF = 1e18;

// grid[y][x] : 0 free, 1 obstacle
bool dijkstraGrid(const vector<vector<int>> &grid, Cell s, Cell g, vector<Cell> &path)
{
    int H = (int)grid.size();
    if (H == 0)
        return false;
    int W = (int)grid[0].size();

    auto inRange = [&](int x, int y)
    {
        return x >= 0 && x < W && y >= 0 && y < H;
    };

    if (!inRange(s.x, s.y) || !inRange(g.x, g.y))
        return false;
    if (grid[s.y][s.x] == 1 || grid[g.y][g.x] == 1)
        return false;

    vector<vector<double>> dist(H, vector<double>(W, INF));
    vector<vector<Cell>> parent(H, vector<Cell>(W, {-1, -1}));

    priority_queue<Node, vector<Node>, greater<Node>> pq;
    dist[s.y][s.x] = 0.0;
    pq.push({s.x, s.y, 0.0});

    // 4방향
    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    while (!pq.empty())
    {
        Node cur = pq.top();
        pq.pop();
        if (cur.cost > dist[cur.y][cur.x])
            continue;
        if (cur.x == g.x && cur.y == g.y)
            break;

        for (int k = 0; k < 4; ++k)
        {
            int nx = cur.x + dx[k];
            int ny = cur.y + dy[k];
            if (!inRange(nx, ny))
                continue;
            if (grid[ny][nx] == 1)
                continue;

            double nd = cur.cost + 1.0;
            if (nd < dist[ny][nx])
            {
                dist[ny][nx] = nd;
                parent[ny][nx] = {cur.x, cur.y};
                pq.push({nx, ny, nd});
            }
        }
    }

    if (dist[g.y][g.x] >= INF)
        return false;

    path.clear();
    Cell cur = g;
    while (!(cur.x == s.x && cur.y == s.y))
    {
        path.push_back(cur);
        cur = parent[cur.y][cur.x];
        if (cur.x < 0)
            return false;
    }
    path.push_back(s);
    reverse(path.begin(), path.end());
    return true;
}

// ===================== Grid <-> World 변환 =====================
struct GridMapInfo
{
    double origin_x; // grid(0,0)의 world 좌표 (m)
    double origin_y;
    double resolution; // m/cell
    int width;
    int height;
};

Cell worldToGrid(double x, double y, const GridMapInfo &m)
{
    int gx = (int)floor((x - m.origin_x) / m.resolution);
    int gy = (int)floor((y - m.origin_y) / m.resolution);
    return {gx, gy};
}

pair<double, double> gridToWorldCenter(int gx, int gy, const GridMapInfo &m)
{
    double x = m.origin_x + (gx + 0.5) * m.resolution;
    double y = m.origin_y + (gy + 0.5) * m.resolution;
    return {x, y};
}

// ===================== 경로 단순화 (방향 바뀌는 점만 waypoint) =====================
vector<Cell> compressPath(const vector<Cell> &path)
{
    vector<Cell> out;
    if (path.empty())
        return out;
    if (path.size() <= 2)
        return path;

    out.push_back(path.front());

    int prev_dx = path[1].x - path[0].x;
    int prev_dy = path[1].y - path[0].y;

    for (size_t i = 1; i + 1 < path.size(); ++i)
    {
        int dx = path[i + 1].x - path[i].x;
        int dy = path[i + 1].y - path[i].y;

        if (dx != prev_dx || dy != prev_dy)
        {
            out.push_back(path[i]); // 방향 바뀌는 점만 추가
        }

        prev_dx = dx;
        prev_dy = dy;
    }

    out.push_back(path.back());
    return out;
}

// ===================== 메인 =====================
int main()
{
    // ---- GPIO 초기화 ----
    if (wiringPiSetup() == -1)
    {
        printf("wiringPiSetup 실패\n");
        return 1;
    }
    setPinConfig(ENA, IN1, IN2);
    setPinConfig(ENB, IN3, IN4);

    // ---- I2C / MPU6050 초기화 ----
    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0)
    {
        perror("i2c open");
        return 1;
    }
    if (ioctl(fd, I2C_SLAVE, MPU_ADDR) < 0)
    {
        perror("ioctl I2C_SLAVE");
        close(fd);
        return 1;
    }

    wreg(0x6B, 0x00); // wake
    wreg(0x1B, 0x00); // gyro ±250dps
    wreg(0x1A, 0x03); // DLPF

    // ---- gyro bias calibration (2초) ----
    double bias = 0.0;
    int N = 0;
    auto t0 = chrono::steady_clock::now();
    while (chrono::duration<double>(chrono::steady_clock::now() - t0).count() < 2.0)
    {
        int16_t raw = r16(0x47); // GYRO_ZOUT_H
        double dps = raw / 131.0;
        double rad = dps * (PI / 180.0);
        bias += rad;
        N++;
        this_thread::sleep_for(chrono::milliseconds(2));
    }
    bias /= (N > 0 ? N : 1);
    printf("Gyro bias = %.6f rad/s\n", bias);

    // ==========================================================
    // 1) 하드코딩 grid map 만들기 (CCTV 대신)
    // ==========================================================
    GridMapInfo mapInfo;
    mapInfo.origin_x = -1.0;
    mapInfo.origin_y = 0.0;
    mapInfo.resolution = 0.10; // 10cm / cell
    mapInfo.width = 90;
    mapInfo.height = 60;

    vector<vector<int>> grid(mapInfo.height, vector<int>(mapInfo.width, 0));

    // 예시 장애물: 가운데 세로 벽 (일부만 막기)
    // x=5 열, y=2~8 막기
    // for (int y = 2; y <= 8; ++y)
    // {
    //     grid[y][0] = 1;
    //     grid[y][1] = 1;
    // }

    // ==========================================================
    // 장애물 벽: (x=0.0~1.0m, y=1.0m)  <-- 가로벽
    // ==========================================================
    // int y_wall = (int)floor((1.0 - mapInfo.origin_y) / mapInfo.resolution);
    // int x_start = (int)floor((0.0 - mapInfo.origin_x) / mapInfo.resolution);
    // int x_end = (int)floor((1.0 - mapInfo.origin_x) / mapInfo.resolution);

    // for (int x = x_start; x <= x_end; ++x)
    // {
    //     if (x >= 0 && x < mapInfo.width && y_wall >= 0 && y_wall < mapInfo.height)
    //     {
    //         grid[y_wall][x] = 1;
    //     }
    // }
    // ==========================================================
    // U자 벽 (오른쪽 개방)
    //   아래 가로벽: (0~5, 2)
    //   왼쪽 세로벽: (0, 2~3)
    //   위 가로벽:   (0~5, 3)
    //   오른쪽 (5,2~3)은 일부러 비움 (통로)
    // ==========================================================

    // 해상도/원점 기준으로 월드좌표(m) -> grid index 변환용 람다
    auto wxToGx = [&](double wx)
    {
        return (int)floor((wx - mapInfo.origin_x) / mapInfo.resolution);
    };
    auto wyToGy = [&](double wy)
    {
        return (int)floor((wy - mapInfo.origin_y) / mapInfo.resolution);
    };

    // 안전하게 grid 칠하는 함수
    auto setObstacle = [&](int gx, int gy)
    {
        if (gx >= 0 && gx < mapInfo.width && gy >= 0 && gy < mapInfo.height)
        {
            grid[gy][gx] = 1;
        }
    };

    // --------------------------
    // 1) 아래 가로벽: (x=0~5, y=2)
    // --------------------------
    {
        int gy = wyToGy(2.0);
        int gx0 = wxToGx(0.0);
        int gx1 = wxToGx(5.0);

        for (int gx = gx0; gx <= gx1; ++gx)
        {
            setObstacle(gx, gy);
        }
    }

    // --------------------------
    // 2) 왼쪽 세로벽: (x=0, y=2~3)
    // --------------------------
    {
        int gx = wxToGx(0.0);
        int gy0 = wyToGy(2.0);
        int gy1 = wyToGy(3.0);

        for (int gy = gy0; gy <= gy1; ++gy)
        {
            setObstacle(gx, gy);
        }
    }

    // --------------------------
    // 3) 위 가로벽: (x=0~5, y=3)
    // --------------------------
    {
        int gy = wyToGy(3.0);
        int gx0 = wxToGx(0.0);
        int gx1 = wxToGx(5.0);

        for (int gx = gx0; gx <= gx1; ++gx)
        {
            setObstacle(gx, gy);
        }
    }
    // 시작/목표 (월드 좌표, m)
    // 네 오도메트리 초기값 x=0,y=0 기준과 맞추려고 약간 안쪽 셀 중심으로 잡음
    double x = 0.5, y = 2.5, th = 0.0; // 현재 추정 상태
    double goal_x = 6.0, goal_y = 1.5; // 최종 목표점 (m)

    // grid 좌표로 변환
    Cell s = worldToGrid(x, y, mapInfo);
    Cell g = worldToGrid(goal_x, goal_y, mapInfo);

    // 다익스트라 경로 계산
    vector<Cell> path;
    bool ok = dijkstraGrid(grid, s, g, path);
    if (!ok)
    {
        printf("Dijkstra 실패: 경로 없음\n");
        tankStop();
        close(fd);
        return 1;
    }

    // 경로 압축 (방향 바뀌는 점만)
    vector<Cell> path2 = compressPath(path);

    // waypoint(월드좌표)로 변환
    vector<pair<double, double>> waypoints;
    for (auto &c : path2)
    {
        auto wp = gridToWorldCenter(c.x, c.y, mapInfo);
        waypoints.push_back(wp);
    }

    // 경로 출력 (디버그)
    printf("=== Grid Path (%zu cells) ===\n", path.size());
    for (auto &c : path)
    {
        printf("(%d,%d) ", c.x, c.y);
    }
    printf("\n");

    printf("=== Waypoints (%zu) ===\n", waypoints.size());
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
        printf("[%zu] (%.2f, %.2f)\n", i, waypoints[i].first, waypoints[i].second);
    }

    // ==========================================================
    // 2) waypoint 추종 주행 (네 기존 state machine 재사용)
    // ==========================================================
    const int TURN_SPEED = 55;
    const int FWD_SPEED = 70;

    const double TH_ROTATE_START = 10.0 * PI / 180.0; // 이 이상이면 회전 모드
    const double TH_GO_ALLOW = 5.0 * PI / 180.0;      // 이 이하면 직진 허용
    const double WP_TOL = 0.08;                       // waypoint 도착 판정 (8cm)

    // 단순 오도메트리용 직진 속도 가정 (m/s) - 실측으로 튜닝 필요
    const double v_forward = 0.16;

    enum NavState
    {
        ROTATE_TO_TARGET,
        GO_STRAIGHT,
        ARRIVED
    };
    NavState navState = ROTATE_TO_TARGET;

    size_t wp_idx = 0;
    auto last = chrono::steady_clock::now();
    auto lastPrint = last;

    while (true)
    {
        auto now = chrono::steady_clock::now();
        double dt = chrono::duration<double>(now - last).count();
        last = now;

        if (dt <= 0.0)
            dt = 0.001;
        if (dt > 0.1)
            dt = 0.1;

        // 자이로로 heading 적분
        int16_t raw = r16(0x47);
        double omega = (raw / 131.0) * (PI / 180.0) - bias;
        th = wrapAngle(th + omega * dt);

        // waypoint 끝났으면 종료
        if (wp_idx >= waypoints.size())
        {
            navState = ARRIVED;
        }

        double tx = 0.0, ty = 0.0;
        double dx = 0.0, dy = 0.0, dist = 0.0;
        double target_th = 0.0, err_th = 0.0;

        if (navState != ARRIVED)
        {
            tx = waypoints[wp_idx].first;
            ty = waypoints[wp_idx].second;

            dx = tx - x;
            dy = ty - y;
            dist = hypot(dx, dy);

            // waypoint 도착 시 다음 waypoint로
            if (dist < WP_TOL)
            {
                wp_idx++;
                tankStop();
                this_thread::sleep_for(chrono::milliseconds(100)); // 잠깐 안정화

                if (wp_idx >= waypoints.size())
                {
                    navState = ARRIVED;
                }
                else
                {
                    navState = ROTATE_TO_TARGET;
                }
                continue;
            }

            target_th = atan2(dy, dx);
            err_th = wrapAngle(target_th - th);

            // 상태 전이
            if (navState == GO_STRAIGHT)
            {
                if (fabs(err_th) > TH_ROTATE_START)
                    navState = ROTATE_TO_TARGET;
            }
            else
            { // ROTATE_TO_TARGET
                if (fabs(err_th) < TH_GO_ALLOW)
                    navState = GO_STRAIGHT;
                else
                    navState = ROTATE_TO_TARGET;
            }
        }

        // 제어 출력
        if (navState == ARRIVED)
        {
            tankStop();
            printf("ARRIVED ALL WAYPOINTS: x=%.2f y=%.2f th=%.1f deg\n",
                   x, y, th * 180.0 / PI);
            break;
        }
        else if (navState == ROTATE_TO_TARGET)
        {
            if (err_th > 0.0)
                tankSpinLeft(TURN_SPEED);
            else
                tankSpinRight(TURN_SPEED);
        }
        else
        { // GO_STRAIGHT
            tankForward(FWD_SPEED);

            // 단순 오도메트리 (직진일 때만)
            double ds = v_forward * dt;
            x += ds * cos(th);
            y += ds * sin(th);
        }

        if (chrono::duration<double>(now - lastPrint).count() >= 0.5)
        {
            printf("[NAV] wp=%zu/%zu | x=%.2f y=%.2f th=%.1fdeg | "
                   "tx=%.2f ty=%.2f | dist=%.2f err=%.1fdeg state=%d\n",
                   wp_idx, waypoints.size(),
                   x, y, th * 180.0 / PI,
                   tx, ty,
                   dist,
                   err_th * 180.0 / PI,
                   (int)navState);
            lastPrint = now;
        }

        this_thread::sleep_for(chrono::milliseconds(20));
    }

    tankStop();
    close(fd);
    return 0;
}