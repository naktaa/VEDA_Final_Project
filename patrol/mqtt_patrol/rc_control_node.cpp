/*
g++ -O2 -std=c++17 rc_control_node.cpp -o rc_control_node -lmosquitto -lwiringPi -lpthread
sudo ./rc_control_node 192.168.100.10 1883 wiserisk/rc/goal wiserisk/p1/pose wiserisk/rc/safety wiserisk/rc/status
*/

#include "rc_control_node.h"
#include <fstream>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <thread>

#include <wiringPi.h>
#include <softPwm.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

namespace {
std::atomic<bool> g_sig_run{true};
auto g_last_diag_log = std::chrono::steady_clock::now();

void onSignal(int) {
    g_sig_run = false;
}

bool extractDouble(const std::string& json, const std::string& key, double& out) {
    const std::regex re("\"" + key + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = std::stod(m[1].str());
    return true;
}

bool extractInt64(const std::string& json, const std::string& key, long long& out) {
    const std::regex re("\"" + key + "\"\\s*:\\s*([0-9]+)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = std::stoll(m[1].str());
    return true;
}

bool extractString(const std::string& json, const std::string& key, std::string& out) {
    const std::regex re("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = m[1].str();
    return true;
}

bool extractBool(const std::string& json, const std::string& key, bool& out) {
    const std::regex re("\"" + key + "\"\\s*:\\s*(true|false)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = (m[1].str() == "true");
    return true;
}

static long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

struct KeyboardGuard {
    bool enabled = false;
    struct termios old_tio{};
    int old_flags = -1;

    bool setup() {
        if (!isatty(STDIN_FILENO)) return false;
        if (tcgetattr(STDIN_FILENO, &old_tio) != 0) return false;

        struct termios new_tio = old_tio;
        new_tio.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
        new_tio.c_cc[VMIN] = 0;
        new_tio.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &new_tio) != 0) return false;

        old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (old_flags < 0) return false;
        if (fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK) != 0) return false;

        enabled = true;
        return true;
    }

    int readChar() const {
        unsigned char ch = 0;
        const ssize_t n = ::read(STDIN_FILENO, &ch, 1);
        return (n == 1) ? static_cast<int>(ch) : -1;
    }

    ~KeyboardGuard() {
        if (!enabled) return;
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        if (old_flags >= 0) fcntl(STDIN_FILENO, F_SETFL, old_flags);
    }
};



// ====== HW (Pi) ======
#define STOP 0
#define FORWARD 1
#define BACKWARD 2

// wiringPi 핀 번호 (너 patrol.cpp 기준)
static constexpr int IN1 = 28;
static constexpr int IN2 = 27;
static constexpr int ENA = 29;

// sysfs PWM (너 patrol.cpp 기준)
const std::string CHIP_PATH = "/sys/class/pwm/pwmchip0/";
const std::string PWM_PATH  = CHIP_PATH + "pwm0/";

static void sysfsWrite(const std::string& path, const std::string& value)
{
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        std::cerr << "[ERR] write: " << path << "\n";
        return;
    }
    ofs << value;
    ofs.close();
}

static void setupServoPwmSysfs()
{
    // export 이미 되어있으면 "busy"가 나와도 무시 가능
    sysfsWrite(CHIP_PATH + "export", "0");
    usleep(200000);

    sysfsWrite(PWM_PATH + "period", "20000000");     // 20ms
    sysfsWrite(PWM_PATH + "enable", "1");
    sysfsWrite(PWM_PATH + "duty_cycle", "1250000");  // center
}

static void setServoUs(int us)
{
    if (us < 900) us = 900;
    if (us > 1600) us = 1600;
    long ns = (long)us * 1000;
    sysfsWrite(PWM_PATH + "duty_cycle", std::to_string(ns));
}

static void setupMotorPins()
{
    if (wiringPiSetup() == -1) {
        std::cerr << "[ERR] wiringPiSetup failed\n";
        std::exit(1);
    }

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    if (softPwmCreate(ENA, 0, 255) != 0) {
        std::cerr << "[ERR] softPwmCreate failed\n";
        std::exit(1);
    }
}

static void setMotorControl(int speed, int stat)
{
    if (speed < 0) speed = 0;
    if (speed > 255) speed = 255;

    if (stat == STOP) {
        softPwmWrite(ENA, 0);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        return;
    }

    softPwmWrite(ENA, speed);
    if (stat == FORWARD) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    } else { // BACKWARD
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
}

static void hardStop()
{
    setMotorControl(0, STOP);
    setServoUs(1250);
}

} // namespace

RcControlNode::RcControlNode(const std::string& broker_host,
                             int broker_port,
                             const std::string& topic_goal,
                             const std::string& topic_pose,
                             const std::string& topic_safety,
                             const std::string& topic_status)
    : host_(broker_host),
      port_(broker_port),
      topic_goal_(topic_goal),
      topic_pose_(topic_pose),
      topic_safety_(topic_safety),
      topic_status_(topic_status) {}

RcControlNode::~RcControlNode() {
    stop();
}

bool RcControlNode::start() {

    // HW init first
    setupMotorPins();
    setupServoPwmSysfs();
    hardStop();
    mosquitto_lib_init();

    mosq_ = mosquitto_new("rc_control_node", true, this);
    if (!mosq_) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        mosquitto_lib_cleanup();
        return false;
    }

    mosquitto_connect_callback_set(mosq_, &RcControlNode::onConnectStatic);
    mosquitto_message_callback_set(mosq_, &RcControlNode::onMessageStatic);
    mosquitto_reconnect_delay_set(mosq_, 1, 10, true);

    const int rc = mosquitto_connect(mosq_, host_.c_str(), port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_connect failed rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ")\n";
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mosquitto_lib_cleanup();
        return false;
    }

    if (mosquitto_loop_start(mosq_) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_loop_start failed\n";
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mosquitto_lib_cleanup();
        return false;
    }

    running_ = true;
    std::cout << "[OK] RC control node started. broker=" << host_ << ":" << port_ << "\n";
    return true;
}

void RcControlNode::run() {
    KeyboardGuard kb;
    const bool keyboard_ok = kb.setup();
    if (keyboard_ok) {
        std::cout << "[KEY] press SPACE to set midpoint goal of id10(6,4) and id11(6,5): (6,4.5), press q to quit\n";
    }

    while (running_ && g_sig_run) {
        if (keyboard_ok) {
            const int ch = kb.readChar();
            if (ch == ' ') {
                RcGoal g;
                g.x = 6.0;
                g.y = 4.5; // midpoint between id10(6,4) and id11(6,5)
                g.frame = "world";
                g.ts_ms = nowMs();
                g.valid = true;
                {
                    std::lock_guard<std::mutex> lk(data_mtx_);
                    goal_ = g;
                    reached_hold_ = false; // new manual goal releases reached latch
                }
                std::cout << "[MANUAL_GOAL] id10-11 midpoint"
                          << " x=" << g.x << " y=" << g.y << "\n";
            } else if (ch == 'q' || ch == 'Q') {
                g_sig_run = false;
            }
        }

        controlStep();
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz
    }
    hardStop();
}

void RcControlNode::stop() {
    if (!running_) return;
    running_ = false;
    hardStop();
    if (mosq_) {
        mosquitto_loop_stop(mosq_, true);
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
    }
    mosquitto_lib_cleanup();
}



void RcControlNode::setControlParams(double k_linear,
                                     double k_yaw,
                                     double max_speed_mps,
                                     double max_yaw_rate_rps,
                                     double tolerance_m) {
    k_linear_ = k_linear;
    k_yaw_ = k_yaw;
    max_speed_mps_ = max_speed_mps;
    max_yaw_rate_rps_ = max_yaw_rate_rps;
    tolerance_m_ = tolerance_m;
}

void RcControlNode::onConnectStatic(struct mosquitto* mosq, void* obj, int rc) {
    if (!obj) return;
    static_cast<RcControlNode*>(obj)->onConnect(rc);
}

void RcControlNode::onMessageStatic(struct mosquitto*, void* obj, const struct mosquitto_message* msg) {
    if (!obj) return;
    static_cast<RcControlNode*>(obj)->onMessage(msg);
}

void RcControlNode::onConnect(int rc) {
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] MQTT connect callback rc=" << rc << "\n";
        return;
    }
    static bool subscribed_log_printed = false;
    const int rc1 = mosquitto_subscribe(mosq_, nullptr, topic_goal_.c_str(), 1);
    const int rc2 = mosquitto_subscribe(mosq_, nullptr, topic_pose_.c_str(), 1);
    const int rc3 = mosquitto_subscribe(mosq_, nullptr, topic_safety_.c_str(), 1);
    if (!subscribed_log_printed) {
        subscribed_log_printed = true;
        std::cout << "[OK] subscribed: " << topic_goal_ << ", " << topic_pose_ << ", " << topic_safety_
                  << " (rc=" << rc1 << "," << rc2 << "," << rc3 << ")\n";
    }
}

void RcControlNode::onMessage(const struct mosquitto_message* msg) {
    if (!msg || !msg->topic || !msg->payload || msg->payloadlen <= 0) return;
    const std::string topic(msg->topic);
    const std::string payload(static_cast<const char*>(msg->payload), msg->payloadlen);

    std::lock_guard<std::mutex> lk(data_mtx_);
    if (topic == topic_goal_) {
        RcGoal g;
        if (parseGoalJson(payload, g) && g.frame == "world") {
            goal_ = g;
            reached_hold_ = false; // any new goal input releases reached latch
            std::cout << "[GOAL] x=" << g.x << " y=" << g.y << " ts_ms=" << g.ts_ms << "\n";
        } else {
            std::cerr << "[WARN] invalid goal payload: " << payload << "\n";
        }
    } else if (topic == topic_pose_) {
        RcPose p;
        if (parsePoseJson(payload, p) && p.frame == "world") {
            pose_ = p;
        }
    } else if (topic == topic_safety_) {
        RcSafety s;
        if (parseSafetyJson(payload, s)) {
            safety_ = s;
        }
    }
}

void RcControlNode::controlStep() {
    RcGoal goal;
    RcPose pose;
    RcSafety safety;
    bool reached_hold = false;
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        goal = goal_;
        pose = pose_;
        safety = safety_;
        reached_hold = reached_hold_;
    }

    RcStatus status;
    if (!goal.valid || !pose.valid) {
        const auto now = std::chrono::steady_clock::now();
        if (now - g_last_diag_log >= std::chrono::seconds(1)) {
            std::cout << "[DIAG] WAIT_INPUT goal.valid=" << goal.valid
                      << " pose.valid=" << pose.valid << "\n";
            g_last_diag_log = now;
        }
        status.mode = "WAIT_INPUT";
        publishStatus(status);
        hardStop();
        return;
    }

    if (safety.estop || safety.obstacle_stop || safety.planner_fail) {
        const auto now = std::chrono::steady_clock::now();
        if (now - g_last_diag_log >= std::chrono::seconds(1)) {
            std::cout << "[DIAG] SAFE_STOP estop=" << safety.estop
                      << " obstacle_stop=" << safety.obstacle_stop
                      << " planner_fail=" << safety.planner_fail << "\n";
            g_last_diag_log = now;
        }
        status.mode = "SAFE_STOP";
        status.reached = false;
         hardStop();
        const RcCommand stop_cmd{0.0, 0.0};
        sendCommandToRc(stop_cmd, computeServoUs(stop_cmd));
        publishStatus(status);
        return;
    }

    if (reached_hold) {
        status.mode = "REACHED";
        status.reached = true;
        status.err_dist = std::hypot(goal.x - pose.x, goal.y - pose.y);
        status.err_yaw = 0.0;
        hardStop();
        const RcCommand stop_cmd{0.0, 0.0};
        sendCommandToRc(stop_cmd, computeServoUs(stop_cmd));
        publishStatus(status);
        return;
    }

    RcCommand cmd = computeCommand(pose, goal, status);
    const int servo_us = computeServoUs(cmd);
    if (status.mode == "REACHED") {
        std::lock_guard<std::mutex> lk(data_mtx_);
        reached_hold_ = true;
    }
    {
        const auto now = std::chrono::steady_clock::now();
        if (now - g_last_diag_log >= std::chrono::seconds(1)) {
            std::cout << std::fixed << std::setprecision(3)
                      << "[DIAG] mode=" << status.mode
                      << " pose=(" << pose.x << "," << pose.y << "," << pose.yaw << ")"
                      << " goal=(" << goal.x << "," << goal.y << ")"
                      << " dist=" << status.err_dist
                      << " err_yaw=" << status.err_yaw
                      << " cmd_v=" << cmd.speed_mps
                      << " cmd_w=" << cmd.yaw_rate_rps
                      << " servo_us=" << servo_us << "\n";
            g_last_diag_log = now;
        }
    }
    sendCommandToRc(cmd, servo_us);
    publishStatus(status);
}

RcCommand RcControlNode::computeCommand(const RcPose& pose, const RcGoal& goal, RcStatus& out_status) const {
    constexpr double PI = 3.14159265358979323846;
    constexpr double REVERSE_SWITCH_RAD = PI / 2.0; // 90 deg
    constexpr double REVERSE_SPEED_SCALE = 0.6;     // reverse safety scale
    constexpr double SLOWDOWN_DIST_M = 0.5;
    constexpr double SLOWDOWN_SCALE = 0.5;

    const double dx = goal.x - pose.x;
    const double dy = goal.y - pose.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    const double target_heading = std::atan2(dy, dx);
    constexpr double YAW_FRAME_OFFSET_RAD = 0.0; // tracker yaw is already aligned to +x=0
    const double pose_yaw_ctrl = normalizeAngle(pose.yaw + YAW_FRAME_OFFSET_RAD);
    const double err_yaw_fwd = normalizeAngle(target_heading - pose_yaw_ctrl);
    const bool use_reverse = (std::fabs(err_yaw_fwd) > REVERSE_SWITCH_RAD);
    const double err_yaw = use_reverse ? normalizeAngle(err_yaw_fwd + PI) : err_yaw_fwd;

    RcCommand cmd{};
    out_status.mode = use_reverse ? "REV_TRACKING" : "TRACKING";
    out_status.err_dist = dist;
    out_status.err_yaw = err_yaw;

    if (dist <= tolerance_m_) {
        out_status.mode = "REACHED";
        out_status.reached = true;
        return cmd;
    }

    const double speed_abs = clamp(k_linear_ * dist, 0.0, max_speed_mps_);
    double speed_cmd = use_reverse ? (-speed_abs * REVERSE_SPEED_SCALE) : speed_abs;
    if (dist <= SLOWDOWN_DIST_M) {
        speed_cmd *= SLOWDOWN_SCALE;
    }
    cmd.speed_mps = speed_cmd;
    cmd.yaw_rate_rps = clamp(k_yaw_ * err_yaw, -max_yaw_rate_rps_, max_yaw_rate_rps_);
    return cmd;
}

bool RcControlNode::publishStatus(const RcStatus& status) {
    if (!mosq_) return false;
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "{\"mode\":\"%s\",\"reached\":%s,\"err_dist\":%.3f,\"err_yaw\":%.3f,\"battery\":%.2f}",
                  status.mode.c_str(),
                  status.reached ? "true" : "false",
                  status.err_dist,
                  status.err_yaw,
                  status.battery);

    const int rc = mosquitto_publish(mosq_,
                                     nullptr,
                                     topic_status_.c_str(),
                                     static_cast<int>(std::strlen(buf)),
                                     buf,
                                     1,
                                     false);
    return (rc == MOSQ_ERR_SUCCESS);
}

void RcControlNode::sendCommandToRc(const RcCommand& cmd, int servo_us) const {
    // TODO(team): replace this with real motor/steering transport (UART/CAN/PWM/etc).
    // 1) 작은 속도는 그냥 정지(떨림 방지)
    const double speed_abs = std::fabs(cmd.speed_mps);
    if (speed_abs < 0.05) {
        hardStop();
        return;
    }

    // 2) speed(m/s) -> PWM(0~255)
    int pwm = (int)std::round(speed_abs / max_speed_mps_ * 255.0);
    if (pwm < 70) pwm = 70;     // 최소 구동 (차에 맞게 튜닝)
    if (pwm > 180) pwm = 180;   // 상한(안전)

    // 3) yaw_rate(rad/s) -> servo us(900~1600)
    const bool reverse = (cmd.speed_mps < 0.0);
    setServoUs(servo_us);
    setMotorControl(pwm, reverse ? BACKWARD : FORWARD);

    // 디버그 필요하면:
    // std::cout << "[HW] pwm=" << pwm << " us=" << servo_us << "\n";
}

int RcControlNode::computeServoUs(const RcCommand& cmd) const {
    constexpr int SERVO_CENTER_US = 1250;
    constexpr double STEER_HW_SIGN = 1.0;  // positive yaw -> right steering on this platform
    constexpr double STEER_GAIN = 0.6;     // reduce aggressive steering
    constexpr int SERVO_MIN_US = 900;
    constexpr int SERVO_MAX_US = 1600;

    if (max_yaw_rate_rps_ <= 0.0) return SERVO_CENTER_US;

    // Kinematic reverse compensation, then hardware sign/scale.
    const bool reverse = (cmd.speed_mps < 0.0);
    const double steer_body = reverse ? -cmd.yaw_rate_rps : cmd.yaw_rate_rps;
    const double steer_cmd = STEER_HW_SIGN * STEER_GAIN * steer_body;
    int us = SERVO_CENTER_US + (int)std::round(steer_cmd / max_yaw_rate_rps_ * 500.0);
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    return us;
}

double RcControlNode::normalizeAngle(double rad)
{ 
    constexpr double PI = 3.14159265358979323846; 
    while (rad > PI) rad -= 2.0 * PI; 
    while (rad < -PI) rad += 2.0 * PI; 
    return rad; 
}

double RcControlNode::clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

bool RcControlNode::parseGoalJson(const std::string& payload, RcGoal& out_goal) {
    if (!extractDouble(payload, "x", out_goal.x)) return false;
    if (!extractDouble(payload, "y", out_goal.y)) return false;
    if (!extractString(payload, "frame", out_goal.frame)) return false;
    if (!extractInt64(payload, "ts_ms", out_goal.ts_ms) &&
        !extractInt64(payload, "ts", out_goal.ts_ms)) {
        return false;
    }
    out_goal.valid = true;
    return true;
}

bool RcControlNode::parsePoseJson(const std::string& payload, RcPose& out_pose) {
    if (!extractDouble(payload, "x", out_pose.x)) return false;
    if (!extractDouble(payload, "y", out_pose.y)) return false;
    if (!extractDouble(payload, "yaw", out_pose.yaw)) return false;
    if (!extractString(payload, "frame", out_pose.frame)) return false;
    if (!extractInt64(payload, "ts_ms", out_pose.ts_ms) &&
        !extractInt64(payload, "ts", out_pose.ts_ms)) {
        return false;
    }
    out_pose.valid = true;
    return true;
}

bool RcControlNode::parseSafetyJson(const std::string& payload, RcSafety& out_safety) {
    bool ok = false;
    ok = extractBool(payload, "estop", out_safety.estop) || ok;
    ok = extractBool(payload, "obstacle_stop", out_safety.obstacle_stop) || ok;
    ok = extractBool(payload, "planner_fail", out_safety.planner_fail) || ok;
    return ok;
}

int main(int argc, char** argv) {
    const std::string host = (argc > 1) ? argv[1] : "192.168.100.10";
    const int port = (argc > 2) ? std::stoi(argv[2]) : 1883;
    const std::string topic_goal = (argc > 3) ? argv[3] : "wiserisk/rc/goal";
    const std::string topic_pose = (argc > 4) ? argv[4] : "wiserisk/p1/pose";
    const std::string topic_safety = (argc > 5) ? argv[5] : "wiserisk/rc/safety";
    const std::string topic_status = (argc > 6) ? argv[6] : "wiserisk/rc/status";

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    RcControlNode node(host, port, topic_goal, topic_pose, topic_safety, topic_status);
    node.setControlParams(0.8, 1.2, 1.5, 1.5, 0.15);

    if (!node.start()) return 1;
    node.run();
    node.stop();
    return 0;
}
