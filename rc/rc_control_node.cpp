#include "rc_control_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <thread>

#if __has_include(<wiringPi.h>)
#include <wiringPi.h>
#define RC_HAS_WIRINGPI 1
#else
#define RC_HAS_WIRINGPI 0
#endif

namespace {
std::atomic<bool> g_sig_run{true};

constexpr int STOP = 0;
constexpr int FORWARD = 1;
constexpr int BACKWARD = 2;
constexpr int POSE_TIMEOUT_MS = 800;
constexpr int CONTROL_STEP_MS = 50;
constexpr int MQTT_QOS = 1;
constexpr bool MQTT_RETAIN = true;
constexpr double CM_TO_M = 0.01;
constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;

// wiringPi pin numbers
constexpr int L_IN1 = 28; // GPIO20, physical 38
constexpr int L_IN2 = 27; // GPIO16, physical 36
constexpr int L_EN = 1;   // GPIO18, physical 12

constexpr int R_IN1 = 25; // GPIO26, physical 37
constexpr int R_IN2 = 23; // GPIO13, physical 33
constexpr int R_EN = 24;  // GPIO19, physical 35

constexpr int PWM_HW_RANGE = 1024;

std::string trimCopy(const std::string& s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

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
    if (!std::regex_search(json, m, re) || m.size() < 2) {
        return false;
    }
    out = std::stod(m[1].str());
    return true;
}

bool extractInt64(const std::string& json, const std::string& key, long long& out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*([0-9]+)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) {
        return false;
    }
    out = std::stoll(m[1].str());
    return true;
}

bool extractString(const std::string& json, const std::string& key, std::string& out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) {
        return false;
    }
    out = m[1].str();
    return true;
}

bool extractBool(const std::string& json, const std::string& key, bool& out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(true|false)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) {
        return false;
    }
    out = (m[1].str() == "true");
    return true;
}

double radToDeg(double rad) {
    return rad * RAD_TO_DEG;
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
    mosquitto_disconnect_callback_set(mosq_, &RcControlNode::onDisconnectStatic);
    mosquitto_message_callback_set(mosq_, &RcControlNode::onMessageStatic);
    mosquitto_reconnect_delay_set(mosq_, 1, 10, true);

    RcStatus lwt_status = CreateDisconnectedRcStatus(status_publish_interval_ms_);
    lwt_status.mode = "idle";
    lwt_status.mission = "none";
    lwt_status.target = TargetInfo{};
    const std::string lwt_payload = SerializeRcStatusToJson(lwt_status);
    const int will_rc = mosquitto_will_set(mosq_,
                                           topic_status_.c_str(),
                                           static_cast<int>(lwt_payload.size()),
                                           lwt_payload.c_str(),
                                           MQTT_QOS,
                                           MQTT_RETAIN);
    if (will_rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_will_set failed rc=" << will_rc
                  << " (" << mosquitto_strerror(will_rc) << ")\n";
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mosquitto_lib_cleanup();
        return false;
    }

    const int rc = mosquitto_connect(mosq_, host_.c_str(), port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_connect failed rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ")\n";
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mosquitto_lib_cleanup();
        return false;
    }

    const int loop_rc = mosquitto_loop_start(mosq_);
    if (loop_rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_loop_start failed rc=" << loop_rc
                  << " (" << mosquitto_strerror(loop_rc) << ")\n";
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mosquitto_lib_cleanup();
        return false;
    }

    running_ = true;
    std::cout << "[OK] RC control node started. broker=" << host_ << ":" << port_
              << " status_topic=" << topic_status_ << "\n";
    return true;
}

void RcControlNode::run() {
    while (running_.load() && g_sig_run.load()) {
        controlStep();
        std::this_thread::sleep_for(std::chrono::milliseconds(CONTROL_STEP_MS));
    }
}

void RcControlNode::stop() {
    if (!running_.load() && !mosq_) {
        return;
    }

    running_ = false;
    stopAllMotors();

    if (mosq_) {
        if (mqtt_connected_.load()) {
            publishDisconnectedStatus();
        }
        mosquitto_loop_stop(mosq_, true);
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
    }

    mqtt_connected_ = false;
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

void RcControlNode::setMotorParams(double track_width_m,
                                   double wheel_max_speed_mps,
                                   double speed_deadband_mps,
                                   int pwm_min_effective,
                                   int pwm_max) {
    track_width_m_ = track_width_m;
    wheel_max_speed_mps_ = wheel_max_speed_mps;
    speed_deadband_mps_ = speed_deadband_mps;
    pwm_min_effective_ = pwm_min_effective;
    pwm_max_ = pwm_max;
}

bool RcControlNode::loadParamsFromIni(const std::string& ini_path) {
    std::ifstream fin(ini_path);
    if (!fin.is_open()) {
        std::cerr << "[WARN] failed to open ini: " << ini_path << "\n";
        return false;
    }

    double k_linear = k_linear_;
    double k_yaw = k_yaw_;
    double max_speed_mps = max_speed_mps_;
    double max_yaw_rate_rps = max_yaw_rate_rps_;
    double tolerance_m = tolerance_m_;
    double track_width_m = track_width_m_;
    double wheel_max_speed_mps = wheel_max_speed_mps_;
    double speed_deadband_mps = speed_deadband_mps_;
    int pwm_min_effective = pwm_min_effective_;
    int pwm_max = pwm_max_;
    bool used_legacy_cm_keys = false;

    std::string line;
    std::string section;
    int line_no = 0;

    while (std::getline(fin, line)) {
        ++line_no;
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';') {
            continue;
        }

        if (trimmed.front() == '[' && trimmed.back() == ']') {
            section = trimCopy(trimmed.substr(1, trimmed.size() - 2));
            continue;
        }

        const auto eq = trimmed.find('=');
        if (eq == std::string::npos) {
            std::cerr << "[WARN] invalid ini line " << line_no << ": " << trimmed << "\n";
            continue;
        }

        const std::string key = trimCopy(trimmed.substr(0, eq));
        const std::string value = trimCopy(trimmed.substr(eq + 1));
        const std::string scoped_key = section.empty() ? key : section + "." + key;

        try {
            if (scoped_key == "control.k_linear") {
                k_linear = std::stod(value);
            } else if (scoped_key == "control.k_yaw") {
                k_yaw = std::stod(value);
            } else if (scoped_key == "control.max_speed_mps") {
                max_speed_mps = std::stod(value);
            } else if (scoped_key == "control.max_speed_cmps") {
                max_speed_mps = std::stod(value) * CM_TO_M;
                used_legacy_cm_keys = true;
            } else if (scoped_key == "control.max_yaw_rate_rps") {
                max_yaw_rate_rps = std::stod(value);
            } else if (scoped_key == "control.tolerance_m") {
                tolerance_m = std::stod(value);
            } else if (scoped_key == "control.tolerance_cm") {
                tolerance_m = std::stod(value) * CM_TO_M;
                used_legacy_cm_keys = true;
            } else if (scoped_key == "motor.track_width_m") {
                track_width_m = std::stod(value);
            } else if (scoped_key == "motor.track_width_cm") {
                track_width_m = std::stod(value) * CM_TO_M;
                used_legacy_cm_keys = true;
            } else if (scoped_key == "motor.wheel_max_speed_mps") {
                wheel_max_speed_mps = std::stod(value);
            } else if (scoped_key == "motor.wheel_max_speed_cmps") {
                wheel_max_speed_mps = std::stod(value) * CM_TO_M;
                used_legacy_cm_keys = true;
            } else if (scoped_key == "motor.speed_deadband_mps") {
                speed_deadband_mps = std::stod(value);
            } else if (scoped_key == "motor.speed_deadband_cmps") {
                speed_deadband_mps = std::stod(value) * CM_TO_M;
                used_legacy_cm_keys = true;
            } else if (scoped_key == "motor.pwm_min_effective") {
                pwm_min_effective = std::stoi(value);
            } else if (scoped_key == "motor.pwm_max") {
                pwm_max = std::stoi(value);
            } else {
                std::cerr << "[WARN] unknown ini key: " << scoped_key << "\n";
            }
        } catch (const std::exception&) {
            std::cerr << "[WARN] invalid ini value for " << scoped_key
                      << " at line " << line_no << ": " << value << "\n";
        }
    }

    setControlParams(k_linear, k_yaw, max_speed_mps, max_yaw_rate_rps, tolerance_m);
    setMotorParams(track_width_m, wheel_max_speed_mps, speed_deadband_mps, pwm_min_effective, pwm_max);

    if (used_legacy_cm_keys) {
        std::cerr << "[WARN] legacy *_cm* RC config keys detected; converted to meter-based values\n";
    }

    std::cout << "[OK] loaded ini params from " << ini_path
              << " | control=(" << k_linear_ << ", " << k_yaw_ << ", " << max_speed_mps_
              << ", " << max_yaw_rate_rps_ << ", " << tolerance_m_ << ")"
              << " motor=(" << track_width_m_ << ", " << wheel_max_speed_mps_
              << ", " << speed_deadband_mps_ << ", " << pwm_min_effective_
              << ", " << pwm_max_ << ")\n";
    return true;
}

void RcControlNode::onConnectStatic(struct mosquitto*, void* obj, int rc) {
    if (!obj) {
        return;
    }
    static_cast<RcControlNode*>(obj)->onConnect(rc);
}

void RcControlNode::onDisconnectStatic(struct mosquitto*, void* obj, int rc) {
    if (!obj) {
        return;
    }
    static_cast<RcControlNode*>(obj)->onDisconnect(rc);
}

void RcControlNode::onMessageStatic(struct mosquitto*, void* obj, const struct mosquitto_message* msg) {
    if (!obj) {
        return;
    }
    static_cast<RcControlNode*>(obj)->onMessage(msg);
}

void RcControlNode::onConnect(int rc) {
    if (rc != MOSQ_ERR_SUCCESS) {
        mqtt_connected_ = false;
        std::cerr << "[ERR] MQTT connect callback rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ")\n";
        return;
    }

    mqtt_connected_ = true;
    const int rc_goal = mosquitto_subscribe(mosq_, nullptr, topic_goal_.c_str(), MQTT_QOS);
    const int rc_pose = mosquitto_subscribe(mosq_, nullptr, topic_pose_.c_str(), MQTT_QOS);
    const int rc_safety = mosquitto_subscribe(mosq_, nullptr, topic_safety_.c_str(), MQTT_QOS);

    std::cout << "[OK] subscribed: " << topic_goal_ << ", " << topic_pose_ << ", " << topic_safety_
              << " (rc=" << rc_goal << "," << rc_pose << "," << rc_safety << ")\n";
}

void RcControlNode::onDisconnect(int rc) {
    mqtt_connected_ = false;
    stopAllMotors();
    std::cerr << "[WARN] MQTT disconnected rc=" << rc << "\n";
}

void RcControlNode::onMessage(const struct mosquitto_message* msg) {
    if (!msg || !msg->topic || !msg->payload || msg->payloadlen <= 0) {
        return;
    }

    const std::string topic(msg->topic);
    const std::string payload(static_cast<const char*>(msg->payload), static_cast<size_t>(msg->payloadlen));

    {
        static auto last_rx_log = std::chrono::steady_clock::time_point::min();
        const auto now_rx = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now_rx - last_rx_log).count() >= 1000) {
            std::cout << "[MQTT RX] topic=" << topic
                      << " len=" << msg->payloadlen
                      << " payload=" << payload.substr(0, 120) << "\n";
            last_rx_log = now_rx;
        }
    }

    std::lock_guard<std::mutex> lk(data_mtx_);
    if (topic == topic_goal_) {
        RcGoal goal;
        if (parseGoalJson(payload, goal) && goal.frame == "world") {
            goal_ = goal;
            std::cout << "[GOAL] x=" << goal.x << " y=" << goal.y
                      << " ts_ms=" << goal.ts_ms << "\n";
        } else {
            std::cerr << "[WARN] goal parse failed or frame!=world. payload=" << payload << "\n";
        }
        return;
    }

    if (topic == topic_pose_) {
        RcPose pose;
        if (parsePoseJson(payload, pose) && pose.frame == "world") {
            pose_ = pose;
            last_pose_rx_ = std::chrono::steady_clock::now();
        } else {
            std::cerr << "[WARN] pose parse failed or frame!=world. payload=" << payload << "\n";
        }
        return;
    }

    if (topic == topic_safety_) {
        RcSafety safety;
        if (parseSafetyJson(payload, safety)) {
            safety_ = safety;
        }
        return;
    }

    std::cerr << "[WARN] unknown topic: " << topic << "\n";
}

void RcControlNode::controlStep() {
    RcGoal goal;
    RcPose pose;
    RcSafety safety;
    std::chrono::steady_clock::time_point last_pose_rx;
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        goal = goal_;
        pose = pose_;
        safety = safety_;
        last_pose_rx = last_pose_rx_;
    }

    static auto last_status_log = std::chrono::steady_clock::time_point::min();
    const auto now_step = std::chrono::steady_clock::now();
    const bool do_log = std::chrono::duration_cast<std::chrono::milliseconds>(now_step - last_status_log).count() >= 1000;
    if (do_log) {
        last_status_log = now_step;
    }

    ControlStatus control_status;
    RcCommand cmd;

    if (!goal.valid || !pose.valid) {
        control_status.robot_state = "WAIT_INPUT";
        sendCommandToRc(cmd);
        if (do_log) {
            std::cout << "[STATUS] WAIT_INPUT goal_valid=" << goal.valid
                      << " pose_valid=" << pose.valid << "\n";
        }
        publishStatus(buildStatus(goal, pose, cmd, control_status));
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pose_rx).count();
    if (age_ms > POSE_TIMEOUT_MS) {
        control_status.robot_state = "POSE_TIMEOUT";
        sendCommandToRc(cmd);
        if (do_log) {
            std::cout << "[STATUS] POSE_TIMEOUT age_ms=" << age_ms << "\n";
        }
        publishStatus(buildStatus(goal, pose, cmd, control_status));
        return;
    }

    if (safety.estop || safety.obstacle_stop || safety.planner_fail) {
        control_status.robot_state = "SAFE_STOP";
        sendCommandToRc(cmd);
        if (do_log) {
            std::cout << "[STATUS] SAFE_STOP estop=" << safety.estop
                      << " obstacle_stop=" << safety.obstacle_stop
                      << " planner_fail=" << safety.planner_fail << "\n";
        }
        publishStatus(buildStatus(goal, pose, cmd, control_status));
        return;
    }

    cmd = computeCommand(pose, goal, control_status);
    sendCommandToRc(cmd);

    if (do_log) {
        std::cout << "[POSE] x=" << pose.x << " y=" << pose.y << " yaw=" << pose.yaw << "\n";
        std::cout << "[CMD] robot_state=" << control_status.robot_state
                  << " v_mps=" << cmd.speed_mps
                  << " yaw_rate_rps=" << cmd.yaw_rate_rps
                  << " err_dist_m=" << control_status.err_dist_m
                  << " err_yaw_rad=" << control_status.err_yaw_rad << "\n";
    }

    publishStatus(buildStatus(goal, pose, cmd, control_status));
}

RcCommand RcControlNode::computeCommand(const RcPose& pose,
                                        const RcGoal& goal,
                                        ControlStatus& out_status) const {
    constexpr double ROTATE_ENTER_TH = 35.0 * PI / 180.0;
    constexpr double ROTATE_EXIT_TH = 20.0 * PI / 180.0;

    const double dx = goal.x - pose.x;
    const double dy = goal.y - pose.y;
    const double dist_m = std::sqrt(dx * dx + dy * dy);
    const double target_heading = std::atan2(dy, dx);
    const double err_yaw = normalizeAngle(target_heading - pose.yaw);

    RcCommand cmd;
    out_status.err_dist_m = dist_m;
    out_status.err_yaw_rad = err_yaw;

    if (goal.ts_ms != last_goal_.ts_ms) {
        last_goal_ = goal;
        reached_ = false;
        rotating_ = false;
        std::cout << "[SM] NEW GOAL -> reset\n";
    }

    if (reached_) {
        out_status.robot_state = "REACHED";
        out_status.reached = true;
        return cmd;
    }

    out_status.robot_state = "TRACKING";

    if (dist_m <= tolerance_m_) {
        reached_ = true;
        rotating_ = false;
        out_status.robot_state = "REACHED";
        out_status.reached = true;
        std::cout << "[SM] REACHED dist_m=" << dist_m << "\n";
        return cmd;
    }

    const double abs_err = std::fabs(err_yaw);
    if (abs_err > ROTATE_ENTER_TH) {
        rotating_ = true;
    }
    if (abs_err < ROTATE_EXIT_TH) {
        rotating_ = false;
    }

    if (rotating_) {
        out_status.robot_state = "ROTATE";
        cmd.speed_mps = 0.0;
        double rotate_err = err_yaw;
        if (std::fabs(rotate_err) > rotate_yaw_offset_rad_) {
            rotate_err -= (rotate_err > 0.0 ? rotate_yaw_offset_rad_ : -rotate_yaw_offset_rad_);
        } else {
            rotate_err = 0.0;
        }
        cmd.yaw_rate_rps = clamp(k_yaw_ * rotate_err, -max_yaw_rate_rps_, max_yaw_rate_rps_);
        return cmd;
    }

    const double heading_scale = std::max(0.2, std::cos(abs_err));
    cmd.speed_mps = clamp(k_linear_ * dist_m * heading_scale, 0.0, max_speed_mps_);
    cmd.yaw_rate_rps = 0.0;
    return cmd;
}

RcStatus RcControlNode::buildStatus(const RcGoal& goal,
                                    const RcPose& pose,
                                    const RcCommand& cmd,
                                    const ControlStatus& control_status) const {
    RcStatus status = CreateDefaultRcStatus(status_publish_interval_ms_);
    status.connected = mqtt_connected_.load();
    status.comm_state = status.connected ? "connected" : "disconnected";
    status.mode = goal.valid ? "auto" : "idle";
    status.mission = goal.valid ? "goal_tracking" : "none";
    status.battery = -1.0;
    status.speed = std::max(0.0, cmd.speed_mps);
    status.robot_state = control_status.robot_state;
    status.data_period = std::to_string(status_publish_interval_ms_) + "ms";

    if (pose.valid) {
        status.x = pose.x;
        status.y = pose.y;
        status.heading = radToDeg(pose.yaw);
    } else {
        status.x = -1.0;
        status.y = -1.0;
        status.heading = -1.0;
    }

    TargetInfo target;
    if (goal.valid) {
        target.x = goal.x;
        target.y = goal.y;
    }
    status.target = target;
    return status;
}

bool RcControlNode::publishStatus(const RcStatus& status) {
    if (!mosq_ || !mqtt_connected_.load()) {
        return false;
    }

    const std::string payload = SerializeRcStatusToJson(status);
    const int rc = mosquitto_publish(mosq_,
                                     nullptr,
                                     topic_status_.c_str(),
                                     static_cast<int>(payload.size()),
                                     payload.c_str(),
                                     MQTT_QOS,
                                     MQTT_RETAIN);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[WARN] rc status publish failed rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ")\n";
        return false;
    }
    return true;
}

bool RcControlNode::publishDisconnectedStatus() {
    if (!mosq_ || !mqtt_connected_.load()) {
        return false;
    }

    RcStatus status = CreateDisconnectedRcStatus(status_publish_interval_ms_);
    status.mode = "idle";
    status.mission = "none";
    status.target = TargetInfo{};
    return publishStatus(status);
}

void RcControlNode::sendCommandToRc(const RcCommand& cmd) const {
    if (!motor_ready_) {
        return;
    }

    double v_left = cmd.speed_mps - (cmd.yaw_rate_rps * track_width_m_ * 0.5);
    double v_right = cmd.speed_mps + (cmd.yaw_rate_rps * track_width_m_ * 0.5);

    if (std::fabs(v_left) < speed_deadband_mps_) {
        v_left = 0.0;
    }
    if (std::fabs(v_right) < speed_deadband_mps_) {
        v_right = 0.0;
    }

    const int left_dir = (v_left > 0.0) ? FORWARD : (v_left < 0.0 ? BACKWARD : STOP);
    const int right_dir = (v_right > 0.0) ? FORWARD : (v_right < 0.0 ? BACKWARD : STOP);

    const int left_pwm = speedToPwm(v_left);
    const int right_pwm = speedToPwm(v_right);

    setMotorControl(L_EN, L_IN1, L_IN2, left_pwm, left_dir);
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

    if (!motor_ready_) {
        std::cerr << "[ERR] motor pin setup failed.\n";
        return;
    }

    stopAllMotors();
    std::cout << "[OK] motor driver ready (HW PWM, L_EN=1, R_EN=24)\n";
#endif
}

void RcControlNode::stopAllMotors() const {
    if (!motor_ready_) {
        return;
    }
    setMotorControl(L_EN, L_IN1, L_IN2, 0, STOP);
    setMotorControl(R_EN, R_IN1, R_IN2, 0, STOP);
}

void RcControlNode::setMotorControl(int en, int in1, int in2, int speed_pwm, int dir) const {
#if !RC_HAS_WIRINGPI
    (void)en;
    (void)in1;
    (void)in2;
    (void)speed_pwm;
    (void)dir;
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
    if (a < speed_deadband_mps_) {
        return 0;
    }

    const double ratio = std::min(1.0, a / std::max(0.001, wheel_max_speed_mps_));
    int pwm = static_cast<int>(std::lround(ratio * pwm_max_));
    if (pwm > 0) {
        pwm = std::max(pwm, pwm_min_effective_);
    }
    return std::min(pwm, pwm_max_);
}

double RcControlNode::normalizeAngle(double rad) {
    while (rad > PI) {
        rad -= 2.0 * PI;
    }
    while (rad < -PI) {
        rad += 2.0 * PI;
    }
    return rad;
}

double RcControlNode::clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

bool RcControlNode::parseGoalJson(const std::string& payload, RcGoal& out_goal) {
    if (!extractDouble(payload, "x", out_goal.x)) {
        return false;
    }
    if (!extractDouble(payload, "y", out_goal.y)) {
        return false;
    }
    if (!extractString(payload, "frame", out_goal.frame)) {
        return false;
    }
    if (!extractInt64(payload, "ts_ms", out_goal.ts_ms)) {
        return false;
    }
    out_goal.valid = true;
    return true;
}

bool RcControlNode::parsePoseJson(const std::string& payload, RcPose& out_pose) {
    if (!extractDouble(payload, "x", out_pose.x)) {
        return false;
    }
    if (!extractDouble(payload, "y", out_pose.y)) {
        return false;
    }
    if (!extractDouble(payload, "yaw", out_pose.yaw)) {
        return false;
    }
    if (!extractString(payload, "frame", out_pose.frame)) {
        return false;
    }
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
    const std::string ini_path = (argc > 7) ? argv[7] : "/home/pi/VEDA_Final_Project/config/rc_control.ini";

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    RcControlNode node(host, port, topic_goal, topic_pose, topic_safety, topic_status);
    node.loadParamsFromIni(ini_path);

    if (!node.start()) {
        return 1;
    }
    node.run();
    node.stop();
    return 0;
}
