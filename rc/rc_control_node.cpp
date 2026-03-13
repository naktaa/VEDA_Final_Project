/*
실행 :sudo /home/pi/VEDA_Final_Project/rc/rc_control_node 192.168.100.10 1883 wiserisk/rc/goal wiserisk/p1/pose wiserisk/rc/safety wiserisk/rc/status
서버 : /home/pi/VEDA_Final_Project/build/p1_tracker "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp" 192.168.100.10 1883 wiserisk/p1/pose /home/pi/VEDA_Final_Project/config/H_img2world.yaml /home/pi/VEDA_Final_Project/config/camera.yaml 0.17 0.17 0 0 0
컴파일 : g++ -std=c++17 /home/pi/VEDA_Final_Project/rc/rc_control_node.cpp \
-I/home/pi/VEDA_Final_Project/rc \
-lmosquitto -lpthread -lwiringPi \
-o /home/pi/VEDA_Final_Project/rc/rc_control_node
*/

#include "rc_control_node.h"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <regex>
#include <sstream>
#include <thread>
#include <algorithm>

#if __has_include(<wiringPi.h>)
#include <wiringPi.h>
#define RC_HAS_WIRINGPI 1
#else
#define RC_HAS_WIRINGPI 0
#endif

namespace {
std::atomic<bool> g_sig_run{true};
std::chrono::steady_clock::time_point g_last_pose_log = std::chrono::steady_clock::time_point::min();

constexpr int STOP     = 0;
constexpr int FORWARD  = 1;
constexpr int BACKWARD = 2;
constexpr int POSE_TIMEOUT_MS = 800;
constexpr double CM_TO_M = 0.01;

// wiringPi pin numbers
constexpr int L_IN1 = 28; // GPIO20, physical 38
constexpr int L_IN2 = 27; // GPIO16, physical 36
constexpr int L_EN  = 1;  // GPIO18, physical 12

constexpr int R_IN1 = 25; // GPIO26, physical 37
constexpr int R_IN2 = 23; // GPIO13, physical 33
constexpr int R_EN  = 24; // GPIO19, physical 35

constexpr int PWM_HW_RANGE = 1024;

int pwm255ToDuty(int pwm_255) {
    const int p = std::max(0, std::min(255, pwm_255));
    return (p * PWM_HW_RANGE) / 255;
}

void onSignal(int) {
    g_sig_run = false;
}

bool extractDouble(const std::string& json, const std::string& key, double& out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = std::stod(m[1].str());
    return true;
}

bool extractInt64(const std::string& json, const std::string& key, long long& out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*([0-9]+)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = std::stoll(m[1].str());
    return true;
}

bool extractString(const std::string& json, const std::string& key, std::string& out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = m[1].str();
    return true;
}

bool extractBool(const std::string& json, const std::string& key, bool& out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(true|false)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = (m[1].str() == "true");
    return true;
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
      topic_status_(topic_status),
      last_pose_rx_(std::chrono::steady_clock::time_point::min()) {}

RcControlNode::~RcControlNode() {
    stop();
}

bool RcControlNode::start() {
    setupMotorDriver();

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
    while (running_ && g_sig_run) {
        controlStep();
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz
    }
}

void RcControlNode::stop() {
    if (!running_) return;
    running_ = false;
    stopAllMotors();

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
    k_linear_        = k_linear;
    k_yaw_           = k_yaw;
    max_speed_mps_   = max_speed_mps;
    max_yaw_rate_rps_ = max_yaw_rate_rps;
    tolerance_m_     = tolerance_m;
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
    const int rc1 = mosquitto_subscribe(mosq_, nullptr, topic_goal_.c_str(),   1);
    const int rc2 = mosquitto_subscribe(mosq_, nullptr, topic_pose_.c_str(),   1);
    const int rc3 = mosquitto_subscribe(mosq_, nullptr, topic_safety_.c_str(), 1);
    std::cout << "[OK] subscribed: " << topic_goal_ << ", " << topic_pose_ << ", " << topic_safety_
              << " (rc=" << rc1 << "," << rc2 << "," << rc3 << ")\n";
}

void RcControlNode::onMessage(const struct mosquitto_message* msg) {
    if (!msg || !msg->topic || !msg->payload || msg->payloadlen <= 0) return;
    const std::string topic(msg->topic);
    const std::string payload(static_cast<const char*>(msg->payload), msg->payloadlen);

    // 어떤 topic이든 수신되면 일단 출력 (진단용, 1초마다)
    {
        static auto last_rx_log = std::chrono::steady_clock::time_point::min();
        const auto now_rx = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now_rx - last_rx_log).count() >= 1000) {
            std::cout << "[MQTT RX] topic=" << topic
                      << " len=" << msg->payloadlen
                      << " payload=" << payload.substr(0, 80) << "\n";
            last_rx_log = now_rx;
        }
    }

    std::lock_guard<std::mutex> lk(data_mtx_);
    if (topic == topic_goal_) {
        RcGoal g;
        if (parseGoalJson(payload, g) && g.frame == "world") {
            goal_ = g;
            const double dx      = g.x - pose_.x;
            const double dy      = g.y - pose_.y;
            const double dist    = std::sqrt(dx * dx + dy * dy);
            const double t_head  = std::atan2(dy, dx);
            const double err_yaw = normalizeAngle(t_head - pose_.yaw);
            std::cout << "[GOAL] goal=(" << g.x << ", " << g.y << ")"
                      << "  cur=(" << pose_.x << ", " << pose_.y << ")"
                      << "  yaw=" << pose_.yaw
                      << "  dist=" << dist
                      << "  err_yaw=" << err_yaw
                      << "  pose_valid=" << pose_.valid
                      << "  ts_ms=" << g.ts_ms << "\n";
        } else {
            std::cerr << "[WARN] goal parse failed or frame!=world. payload=" << payload << "\n";
        }
    } else if (topic == topic_pose_) {
        RcPose p;
        if (parsePoseJson(payload, p) && p.frame == "world") {
            pose_ = p;
            last_pose_rx_ = std::chrono::steady_clock::now();
            static int pose_cnt = 0;
            if (++pose_cnt % 10 == 0) {
                std::cout << "[POSE] x=" << p.x << " y=" << p.y
                          << " yaw=" << p.yaw
                          << " ts_ms=" << p.ts_ms << "\n";
            }
        } else {
            std::cerr << "[WARN] pose parse failed or frame!=world. payload=" << payload << "\n";
        }
    } else if (topic == topic_safety_) {
        RcSafety s;
        if (parseSafetyJson(payload, s)) {
            safety_ = s;
        }
    } else {
        std::cerr << "[WARN] unknown topic: " << topic << "\n";
    }
}

void RcControlNode::controlStep() {
    RcGoal  goal;
    RcPose  pose;
    RcSafety safety;
    std::chrono::steady_clock::time_point last_pose_rx;
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        goal         = goal_;
        pose         = pose_;
        safety       = safety_;
        last_pose_rx = last_pose_rx_;
    }

    // 1초마다 한 번만 로그 출력
    static auto last_status_log = std::chrono::steady_clock::time_point::min();
    const auto now_step = std::chrono::steady_clock::now();
    const bool do_log = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now_step - last_status_log).count() >= 1000;
    if (do_log) last_status_log = now_step;

    RcStatus status;
    if (!goal.valid || !pose.valid) {
        status.mode = "WAIT_INPUT";
        if (do_log)
            std::cout << "[STATUS] WAIT_INPUT  goal_valid=" << goal.valid
                      << "  pose_valid=" << pose.valid << "\n";
        sendCommandToRc({0.0, 0.0});
        publishStatus(status);
        return;
    }

    const auto now    = std::chrono::steady_clock::now();
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - last_pose_rx).count();
    if (age_ms > POSE_TIMEOUT_MS) {
        status.mode    = "POSE_TIMEOUT";
        status.reached = false;
        if (do_log)
            std::cout << "[STATUS] POSE_TIMEOUT  age_ms=" << age_ms << "\n";
        sendCommandToRc({0.0, 0.0});
        publishStatus(status);
        return;
    }

    if (safety.estop || safety.obstacle_stop || safety.planner_fail) {
        status.mode    = "SAFE_STOP";
        status.reached = false;
        if (do_log)
            std::cout << "[STATUS] SAFE_STOP  estop=" << safety.estop
                      << "  obs=" << safety.obstacle_stop
                      << "  plan=" << safety.planner_fail << "\n";
        sendCommandToRc({0.0, 0.0});
        publishStatus(status);
        return;
    }

    RcCommand cmd = computeCommand(pose, goal, status);
    sendCommandToRc(cmd);
    if (do_log) {
        std::cout << "[POSE] x=" << pose.x << " y=" << pose.y
                  << " yaw=" << pose.yaw << "\n";
        std::cout << "[CMD] mode=" << status.mode
                  << "  v=" << cmd.speed_mps
                  << "  w=" << cmd.yaw_rate_rps
                  << "  dist=" << status.err_dist
                  << "  err_yaw=" << status.err_yaw << "\n";
    }
    publishStatus(status);
}

RcCommand RcControlNode::computeCommand(const RcPose& pose,
                                         const RcGoal& goal,
                                         RcStatus& out_status) const {
    constexpr double PI = 3.14159265358979323846;
    constexpr double ROTATE_IN_PLACE_TH = 25.0 * PI / 180.0;

    const double dx   = goal.x - pose.x;
    const double dy   = goal.y - pose.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    const double target_heading = std::atan2(dy, dx);
    const double err_yaw        = normalizeAngle(target_heading - pose.yaw);

    RcCommand cmd{};
    out_status.mode     = "TRACKING";
    out_status.err_dist = dist;
    out_status.err_yaw  = err_yaw;

    if (dist <= tolerance_m_) {
        out_status.mode    = "REACHED";
        out_status.reached = true;
        return cmd;
    }

    const double abs_err = std::fabs(err_yaw);
    if (abs_err > ROTATE_IN_PLACE_TH) {
        // 각도 오차 25도 초과 -> 제자리 회전만
        cmd.speed_mps    = 0.0;
        cmd.yaw_rate_rps = clamp(k_yaw_ * err_yaw, -max_yaw_rate_rps_, max_yaw_rate_rps_);
        return cmd;
    }

    // 각도 오차 작으면 전진 + 미세 조향
    const double heading_scale = std::max(0.2, std::cos(abs_err));
    cmd.speed_mps    = clamp(k_linear_ * dist * heading_scale, 0.0, max_speed_mps_);
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

    const int rc = mosquitto_publish(mosq_, nullptr, topic_status_.c_str(),
                                     static_cast<int>(std::strlen(buf)), buf, 1, false);
    return (rc == MOSQ_ERR_SUCCESS);
}

void RcControlNode::sendCommandToRc(const RcCommand& cmd) const {
    if (!motor_ready_) return;

    double v_left  = cmd.speed_mps - (cmd.yaw_rate_rps * track_width_m_ * 0.5);
    double v_right = cmd.speed_mps + (cmd.yaw_rate_rps * track_width_m_ * 0.5);

    if (std::fabs(v_left)  < speed_deadband_mps_) v_left  = 0.0;
    if (std::fabs(v_right) < speed_deadband_mps_) v_right = 0.0;

    const int left_dir  = (v_left  > 0.0) ? FORWARD  : (v_left  < 0.0 ? BACKWARD : STOP);
    const int right_dir = (v_right > 0.0) ? FORWARD  : (v_right < 0.0 ? BACKWARD : STOP);

    const int left_pwm  = speedToPwm(v_left);
    const int right_pwm = speedToPwm(v_right);

    setMotorControl(L_EN, L_IN1, L_IN2, left_pwm,  left_dir);
    setMotorControl(R_EN, R_IN1, R_IN2, right_pwm, right_dir);
}

void RcControlNode::setupMotorDriver() {
#if !RC_HAS_WIRINGPI
    std::cerr << "[WARN] wiringPi.h not found. motor output disabled.\n";
    motor_ready_ = false;
    return;
#else
    if (wiringPiSetup() == -1) {
        std::cerr << "[ERR] wiringPiSetup failed. motor output disabled.\n";
        motor_ready_ = false;
        return;
    }

    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(32);
    pwmSetRange(PWM_HW_RANGE);

    auto setupOne = [](int en, int in1, int in2) -> bool {
        pinMode(en,  PWM_OUTPUT);
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        pwmWrite(en, 0);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        return true;
    };

    const bool left_ok  = setupOne(L_EN, L_IN1, L_IN2);
    const bool right_ok = setupOne(R_EN, R_IN1, R_IN2);
    motor_ready_ = left_ok && right_ok;

    if (!motor_ready_) {
        std::cerr << "[ERR] motor pin setup failed.\n";
        return;
    }
    stopAllMotors();
    std::cout << "[OK] motor driver ready (HW PWM, L_EN=1, R_EN=24)\n";
#endif
}

void RcControlNode::stopAllMotors() const {
    if (!motor_ready_) return;
    setMotorControl(L_EN, L_IN1, L_IN2, 0, STOP);
    setMotorControl(R_EN, R_IN1, R_IN2, 0, STOP);
}

void RcControlNode::setMotorControl(int en, int in1, int in2, int speed_pwm, int dir) const {
#if !RC_HAS_WIRINGPI
    (void)en; (void)in1; (void)in2; (void)speed_pwm; (void)dir;
    return;
#else
    const int pwm = std::max(0, std::min(pwm_max_, speed_pwm));
    pwmWrite(en, pwm255ToDuty(pwm));

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
#endif
}

int RcControlNode::speedToPwm(double speed_mps) const {
    const double a = std::fabs(speed_mps);
    if (a < speed_deadband_mps_) return 0;

    const double ratio = std::min(1.0, a / std::max(0.01, wheel_max_speed_mps_));
    int pwm = static_cast<int>(std::lround(ratio * pwm_max_));
    if (pwm > 0) pwm = std::max(pwm, pwm_min_effective_);
    return std::min(pwm, pwm_max_);
}

double RcControlNode::normalizeAngle(double rad) {
    constexpr double PI = 3.14159265358979323846;
    while (rad >  PI) rad -= 2.0 * PI;
    while (rad < -PI) rad += 2.0 * PI;
    return rad;
}

double RcControlNode::clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

bool RcControlNode::parseGoalJson(const std::string& payload, RcGoal& out_goal) {
    if (!extractDouble(payload, "x",     out_goal.x))     return false;
    if (!extractDouble(payload, "y",     out_goal.y))     return false;
    if (!extractString(payload, "frame", out_goal.frame)) return false;
    if (!extractInt64 (payload, "ts_ms", out_goal.ts_ms)) return false;
    out_goal.valid = true;
    return true;
}

bool RcControlNode::parsePoseJson(const std::string& payload, RcPose& out_pose) {
    if (!extractDouble(payload, "x",   out_pose.x))   return false;
    if (!extractDouble(payload, "y",   out_pose.y))   return false;
    if (!extractDouble(payload, "yaw", out_pose.yaw)) return false;
    if (!extractString(payload, "frame", out_pose.frame)) return false;
    if (!extractInt64(payload, "ts_ms", out_pose.ts_ms) &&
        !extractInt64(payload, "ts",    out_pose.ts_ms)) {
        return false;
    }
    out_pose.valid = true;
    return true;
}

bool RcControlNode::parseSafetyJson(const std::string& payload, RcSafety& out_safety) {
    bool ok = false;
    ok = extractBool(payload, "estop",         out_safety.estop)         || ok;
    ok = extractBool(payload, "obstacle_stop", out_safety.obstacle_stop) || ok;
    ok = extractBool(payload, "planner_fail",  out_safety.planner_fail)  || ok;
    return ok;
}

int main(int argc, char** argv) {
    const std::string host        = (argc > 1) ? argv[1] : "192.168.100.10";
    const int         port        = (argc > 2) ? std::stoi(argv[2]) : 1883;
    const std::string topic_goal  = (argc > 3) ? argv[3] : "wiserisk/rc/goal";
    const std::string topic_pose  = (argc > 4) ? argv[4] : "wiserisk/p1/pose";
    const std::string topic_safety = (argc > 5) ? argv[5] : "wiserisk/rc/safety";
    const std::string topic_status = (argc > 6) ? argv[6] : "wiserisk/rc/status";

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    RcControlNode node(host, port, topic_goal, topic_pose, topic_safety, topic_status);
    node.setControlParams(0.8, 2.5, 0.4, 3.0, 0.25);

    if (!node.start()) return 1;
    node.run();
    node.stop();
    return 0;
}