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

constexpr int STOP = 0;
constexpr int FORWARD = 1;
constexpr int BACKWARD = 2;

// wiringPi pin numbers
constexpr int L_IN1 = 28; // GPIO20, physical 38
constexpr int L_IN2 = 27; // GPIO16, physical 36
constexpr int L_EN  = 1;  // GPIO18, physical 12 (HW PWM0)

constexpr int R_IN1 = 25; // GPIO26, physical 37
constexpr int R_IN2 = 23; // GPIO13, physical 33
constexpr int R_EN  = 24; // GPIO19, physical 35 (HW PWM1)

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

bool extractInt(const std::string& json, const std::string& key, int& out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = std::stoi(m[1].str());
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

bool nearlyEqual(double a, double b, double eps = 1e-6) {
    return std::fabs(a - b) <= eps;
}

bool sameGoal(const RcGoal& a, const RcGoal& b) {
    return a.ts_ms == b.ts_ms &&
           nearlyEqual(a.x, b.x) &&
           nearlyEqual(a.y, b.y) &&
           a.frame == b.frame;
}

bool samePose(const RcPose& a, const RcPose& b) {
    return a.ts_ms == b.ts_ms &&
           nearlyEqual(a.x, b.x) &&
           nearlyEqual(a.y, b.y) &&
           nearlyEqual(a.yaw, b.yaw) &&
           a.frame == b.frame;
}

bool isZeroCommand(const RcCommand& cmd) {
    return std::fabs(cmd.speed_mps) < 1e-4 && std::fabs(cmd.yaw_rate_rps) < 1e-4;
}

bool commandChanged(const RcCommand& a, const RcCommand& b) {
    return !nearlyEqual(a.speed_mps, b.speed_mps, 1e-3) ||
           !nearlyEqual(a.yaw_rate_rps, b.yaw_rate_rps, 1e-3);
}

bool statusChanged(const RcStatus& a, const RcStatus& b) {
    if (a.mode != b.mode) return true;
    if (a.reached != b.reached) return true;
    if (!nearlyEqual(a.err_dist, b.err_dist, 1e-3)) return true;
    if (!nearlyEqual(a.err_yaw, b.err_yaw, 1e-3)) return true;
    if (!nearlyEqual(a.battery, b.battery, 1e-3)) return true;
    return false;
}

RcCommand g_last_cmd_logged{};
bool g_has_last_cmd_logged = false;
std::chrono::steady_clock::time_point g_last_cmd_log_tp = std::chrono::steady_clock::time_point::min();
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
    k_linear_ = k_linear;
    k_yaw_ = k_yaw;
    max_speed_mps_ = max_speed_mps;
    max_yaw_rate_rps_ = max_yaw_rate_rps;
    tolerance_m_ = tolerance_m;
}

void RcControlNode::setDistanceScale(double scale) {
    distance_scale_ = (scale > 0.0) ? scale : 1.0;
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
    const int rc1 = mosquitto_subscribe(mosq_, nullptr, topic_goal_.c_str(), 1);
    const int rc2 = mosquitto_subscribe(mosq_, nullptr, topic_pose_.c_str(), 1);
    const int rc2b = mosquitto_subscribe(mosq_, nullptr, "wiserisk/+/pose", 1);
    const int rc3 = mosquitto_subscribe(mosq_, nullptr, topic_safety_.c_str(), 1);
    std::cout << "[OK] subscribed: " << topic_goal_ << ", " << topic_pose_ << ", " << topic_safety_
              << " + wiserisk/+/pose"
              << " (rc=" << rc1 << "," << rc2 << "," << rc2b << "," << rc3 << ")\n";
}

void RcControlNode::onMessage(const struct mosquitto_message* msg) {
    if (!msg || !msg->topic || !msg->payload || msg->payloadlen <= 0) return;
    const std::string topic(msg->topic);
    const std::string payload(static_cast<const char*>(msg->payload), msg->payloadlen);

    std::lock_guard<std::mutex> lk(data_mtx_);
    if (topic == topic_goal_) {
        RcGoal g;
        if (parseGoalJson(payload, g) && g.frame == "world") {
            if (has_pending_goal_ && g.ts_ms < pending_goal_.ts_ms) return;
            if (has_pending_goal_ && sameGoal(g, pending_goal_)) return;
            pending_goal_ = g;
            has_pending_goal_ = true;
            std::cout << "[GOAL] x=" << g.x << " y=" << g.y << " ts_ms=" << g.ts_ms << "\n";
        } else {
            std::cerr << "[WARN] invalid goal payload: " << payload << "\n";
        }
    } else if (topic == topic_pose_ ||
               (topic.size() >= 5 && topic.compare(topic.size() - 5, 5, "/pose") == 0)) {
        RcPose p;
        if (parsePoseJson(payload, p) && p.frame == "world") {
            if (pose_.valid && p.ts_ms < pose_.ts_ms) return;
            if (pose_.valid && samePose(p, pose_)) return;
            pose_ = p;
            last_pose_rx_ = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(last_pose_rx_ - g_last_pose_log).count() >= 500) {
                std::cout << "[POSE] x=" << p.x << " y=" << p.y
                          << " yaw=" << p.yaw << " frame=" << p.frame
                          << " id=" << p.marker_id
                          << " ts_ms=" << p.ts_ms << "\n";
                g_last_pose_log = last_pose_rx_;
            }
        } else {
            std::cerr << "[WARN] invalid pose payload: " << payload << "\n";
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
    std::chrono::steady_clock::time_point last_pose_rx;
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        if (has_pending_goal_) {
            goal_ = pending_goal_;
            has_pending_goal_ = false;
        }
        goal = goal_;
        pose = pose_;
        safety = safety_;
        last_pose_rx = last_pose_rx_;
    }

    RcStatus status{};
    status.reached = false;
    status.err_dist = 0.0;
    status.err_yaw = 0.0;
    status.battery = 0.0;

    if (!goal.valid || !pose.valid) {
        status.mode = "WAIT_INPUT";
        static int wait_log_div = 0;
        if ((++wait_log_div % 20) == 0) {
            std::cout << "[WAIT_INPUT] goal.valid=" << goal.valid
                      << " pose.valid=" << pose.valid << "\n";
        }
        sendCommandToRc({0.0, 0.0});
        publishStatus(status);
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pose_rx).count();
    if (age_ms > pose_timeout_ms_) {
        status.mode = "POSE_TIMEOUT";
        status.reached = false;
        sendCommandToRc({0.0, 0.0});
        publishStatus(status);
        return;
    }

    if (safety.estop || safety.obstacle_stop || safety.planner_fail) {
        status.mode = "SAFE_STOP";
        status.reached = false;
        sendCommandToRc({0.0, 0.0});
        publishStatus(status);
        return;
    }

    RcCommand cmd = computeCommand(pose, goal, status);
    sendCommandToRc(cmd);
    publishStatus(status);
}

RcCommand RcControlNode::computeCommand(const RcPose& pose, const RcGoal& goal, RcStatus& out_status) const {
    constexpr double ROTATE_IN_PLACE_TH = 35.0 * 3.14159265358979323846 / 180.0;
    const double dx = goal.x - pose.x;
    const double dy = goal.y - pose.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double dist_m = dist * distance_scale_;

    const double target_heading = std::atan2(dy, dx);
    const double err_yaw = normalizeAngle(target_heading - pose.yaw);

    RcCommand cmd{};
    out_status.mode = "TRACKING";
    out_status.err_dist = dist_m;
    out_status.err_yaw = err_yaw;

    if (dist_m <= tolerance_m_) {
        out_status.mode = "REACHED";
        out_status.reached = true;
        return cmd;
    }

    const double abs_err = std::fabs(err_yaw);
    if (abs_err > ROTATE_IN_PLACE_TH) {
        cmd.speed_mps = 0.0;
        cmd.yaw_rate_rps = clamp(k_yaw_ * err_yaw, -max_yaw_rate_rps_, max_yaw_rate_rps_);
        return cmd;
    }
    const double heading_scale = std::max(0.2, std::cos(abs_err));
    const double speed_limit = std::min(max_speed_mps_, std::max(0.10, dist_m * 0.6));
    cmd.speed_mps = clamp(k_linear_ * dist_m * heading_scale, 0.0, speed_limit);
    cmd.yaw_rate_rps = clamp(k_yaw_ * err_yaw, -max_yaw_rate_rps_, max_yaw_rate_rps_);
    return cmd;
}

bool RcControlNode::publishStatus(const RcStatus& status) {
    if (!mosq_) return false;

    const auto now = std::chrono::steady_clock::now();
    const bool changed = !has_last_status_ || statusChanged(status, last_status_);
    const bool periodic =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_status_pub_tp_).count() >= 1000;
    if (!changed && !periodic) return true;

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
    if (rc == MOSQ_ERR_SUCCESS) {
        last_status_ = status;
        has_last_status_ = true;
        last_status_pub_tp_ = now;
    }
    return (rc == MOSQ_ERR_SUCCESS);
}

void RcControlNode::sendCommandToRc(const RcCommand& cmd) const {
    const auto now = std::chrono::steady_clock::now();
    const bool cmd_zero = isZeroCommand(cmd);
    const bool changed = !g_has_last_cmd_logged || commandChanged(cmd, g_last_cmd_logged);
    const bool periodic = !cmd_zero &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_cmd_log_tp).count() >= 1000;

    if (!motor_ready_) {
        if (changed || periodic) {
            std::cout << "[CMD] speed=" << cmd.speed_mps << " m/s, yaw_rate=" << cmd.yaw_rate_rps
                      << " rad/s (motor driver not ready)\n";
            g_last_cmd_logged = cmd;
            g_has_last_cmd_logged = true;
            g_last_cmd_log_tp = now;
        }
        return;
    }

    double v_left = cmd.speed_mps - (cmd.yaw_rate_rps * track_width_m_ * 0.5);
    double v_right = cmd.speed_mps + (cmd.yaw_rate_rps * track_width_m_ * 0.5);

    if (std::fabs(v_left) < speed_deadband_mps_) v_left = 0.0;
    if (std::fabs(v_right) < speed_deadband_mps_) v_right = 0.0;

    const int left_dir = (v_left > 0.0) ? FORWARD : (v_left < 0.0 ? BACKWARD : STOP);
    const int right_dir = (v_right > 0.0) ? FORWARD : (v_right < 0.0 ? BACKWARD : STOP);

    const int left_pwm = speedToPwm(v_left);
    const int right_pwm = speedToPwm(v_right);

    setMotorControl(L_EN, L_IN1, L_IN2, left_pwm, left_dir);
    setMotorControl(R_EN, R_IN1, R_IN2, right_pwm, right_dir);

    if (changed || periodic) {
        std::cout << "[CMD] v=" << cmd.speed_mps << " w=" << cmd.yaw_rate_rps
                  << " | L(v,pwm,dir)=" << v_left << "," << left_pwm << "," << left_dir
                  << " R(v,pwm,dir)=" << v_right << "," << right_pwm << "," << right_dir << "\n";
        g_last_cmd_logged = cmd;
        g_has_last_cmd_logged = true;
        g_last_cmd_log_tp = now;
    }
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
        pinMode(en, PWM_OUTPUT);
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        pwmWrite(en, 0);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        return true;
    };

    const bool left_ok = setupOne(L_EN, L_IN1, L_IN2);
    const bool right_ok = setupOne(R_EN, R_IN1, R_IN2);
    motor_ready_ = left_ok && right_ok;

    if (!motor_ready_) return;
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
    if (!extractInt64(payload, "ts_ms", out_goal.ts_ms)) return false;
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
    out_pose.marker_id = -1;
    const bool has_id =
        extractInt(payload, "id", out_pose.marker_id) ||
        extractInt(payload, "marker_id", out_pose.marker_id) ||
        extractInt(payload, "aruco_id", out_pose.marker_id);
    if (has_id) {
        if (out_pose.marker_id < 26 || out_pose.marker_id > 30) return false;
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
    // stop within 10 cm of goal
    node.setControlParams(0.8, 0.8, 0.8, 1.5, 0.10);
    // pose/goal are in centimeters -> convert to meters for control
    node.setDistanceScale(0.01);

    if (!node.start()) return 1;
    node.run();
    node.stop();
    return 0;
}
