#include "rc_control_node.hpp"

#include <mosquitto.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "rc_json_utils.hpp"
#include "rc_motor_driver.hpp"
#include "rc_path_planner.hpp"

namespace {

constexpr int kPoseTimeoutMs = 800;
constexpr int kControlStepMs = 50;
constexpr int kMqttQos = 1;
constexpr bool kMqttRetain = true;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

double RadToDeg(double rad) {
    return rad * kRadToDeg;
}

} // namespace

RcControlNode::RcControlNode(const RcAppConfig& config)
    : config_(config),
      last_pose_rx_(std::chrono::steady_clock::time_point::min()),
      motor_driver_ptr_(new RcMotorDriver()) {
    motor_driver_ptr_->configure(config_.motor);
}

RcControlNode::~RcControlNode() {
    stop();
    delete motor_driver_ptr_;
    motor_driver_ptr_ = nullptr;
}

bool RcControlNode::start() {
    motor_driver_ptr_->setup();

    mosquitto_lib_init();
    mosq_ = mosquitto_new("auto_main", true, this);
    if (!mosq_) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        mosquitto_lib_cleanup();
        return false;
    }

    mosquitto_connect_callback_set(mosq_, &RcControlNode::onConnectStatic);
    mosquitto_disconnect_callback_set(mosq_, &RcControlNode::onDisconnectStatic);
    mosquitto_message_callback_set(mosq_, &RcControlNode::onMessageStatic);
    mosquitto_reconnect_delay_set(mosq_, 1, 10, true);

    RcStatus lwt_status = CreateDisconnectedRcStatus(config_.status_publish_interval_ms);
    lwt_status.mode = "idle";
    lwt_status.mission = "none";
    lwt_status.target = TargetInfo{};
    const std::string lwt_payload = SerializeRcStatusToJson(lwt_status);
    const int will_rc = mosquitto_will_set(mosq_,
                                           config_.topics.status.c_str(),
                                           static_cast<int>(lwt_payload.size()),
                                           lwt_payload.c_str(),
                                           kMqttQos,
                                           kMqttRetain);
    if (will_rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_will_set failed rc=" << will_rc
                  << " (" << mosquitto_strerror(will_rc) << ")\n";
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mosquitto_lib_cleanup();
        return false;
    }

    const int rc = mosquitto_connect(mosq_, config_.host.c_str(), config_.port, 60);
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
    std::cout << "[OK] auto_main started. broker=" << config_.host << ":" << config_.port
              << " status_topic=" << config_.topics.status << "\n";
    return true;
}

void RcControlNode::run(const std::atomic<bool>* external_run) {
    while (running_.load() && (!external_run || external_run->load())) {
        controlStep();
        std::this_thread::sleep_for(std::chrono::milliseconds(kControlStepMs));
    }
}

void RcControlNode::stop() {
    if (!running_.load() && !mosq_) {
        return;
    }

    running_ = false;
    if (motor_driver_ptr_) {
        motor_driver_ptr_->stopAll();
    }

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

void RcControlNode::onConnectStatic(struct mosquitto*, void* obj, int rc) {
    if (obj) {
        static_cast<RcControlNode*>(obj)->onConnect(rc);
    }
}

void RcControlNode::onDisconnectStatic(struct mosquitto*, void* obj, int rc) {
    if (obj) {
        static_cast<RcControlNode*>(obj)->onDisconnect(rc);
    }
}

void RcControlNode::onMessageStatic(struct mosquitto*, void* obj, const struct mosquitto_message* msg) {
    if (obj) {
        static_cast<RcControlNode*>(obj)->onMessage(msg);
    }
}

void RcControlNode::onConnect(int rc) {
    if (rc != MOSQ_ERR_SUCCESS) {
        mqtt_connected_ = false;
        std::cerr << "[ERR] MQTT connect callback rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ")\n";
        return;
    }

    mqtt_connected_ = true;
    const int rc_goal = mosquitto_subscribe(mosq_, nullptr, config_.topics.goal.c_str(), kMqttQos);
    const int rc_pose = mosquitto_subscribe(mosq_, nullptr, config_.topics.pose.c_str(), kMqttQos);
    const int rc_safety = mosquitto_subscribe(mosq_, nullptr, config_.topics.safety.c_str(), kMqttQos);

    std::cout << "[OK] subscribed: " << config_.topics.goal << ", " << config_.topics.pose << ", "
              << config_.topics.safety << " (rc=" << rc_goal << "," << rc_pose << "," << rc_safety << ")\n";
}

void RcControlNode::onDisconnect(int rc) {
    mqtt_connected_ = false;
    if (motor_driver_ptr_) {
        motor_driver_ptr_->stopAll();
    }
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
    if (topic == config_.topics.goal) {
        RcGoal goal;
        if (ParseGoalJson(payload, goal) && goal.frame == "world") {
            goal_ = goal;
            std::cout << "[GOAL] x=" << goal.x << " y=" << goal.y
                      << " ts_ms=" << goal.ts_ms << "\n";
        } else {
            std::cerr << "[WARN] goal parse failed or frame!=world. payload=" << payload << "\n";
        }
        return;
    }

    if (topic == config_.topics.pose) {
        RcPose pose;
        if (ParsePoseJson(payload, pose) && pose.frame == "world") {
            pose_ = pose;
            last_pose_rx_ = std::chrono::steady_clock::now();
        } else {
            std::cerr << "[WARN] pose parse failed or frame!=world. payload=" << payload << "\n";
        }
        return;
    }

    if (topic == config_.topics.safety) {
        RcSafety safety;
        if (ParseSafetyJson(payload, safety)) {
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
    const bool do_log =
        std::chrono::duration_cast<std::chrono::milliseconds>(now_step - last_status_log).count() >= 1000;
    if (do_log) {
        last_status_log = now_step;
    }

    ControlStatus control_status;
    RcCommand cmd;

    if (!pose.valid) {
        control_status.robot_state = "WAIT_INPUT";
        if (motor_driver_ptr_) {
            motor_driver_ptr_->sendCommand(cmd);
        }
        if (do_log) {
            std::cout << "[STATUS] WAIT_INPUT goal_valid=" << goal.valid
                      << " pose_valid=" << pose.valid << "\n";
        }
        publishStatus(buildStatus(goal, pose, cmd, control_status));
        return;
    }

    if (!goal.valid) {
        control_status.robot_state = "WAIT_GOAL";
        if (motor_driver_ptr_) {
            motor_driver_ptr_->sendCommand(cmd);
        }
        if (do_log) {
            std::cout << "[POSE] x=" << pose.x << " y=" << pose.y << " yaw=" << pose.yaw << "\n";
            std::cout << "[STATUS] WAIT_GOAL goal_valid=" << goal.valid
                      << " pose_valid=" << pose.valid << "\n";
        }
        publishStatus(buildStatus(goal, pose, cmd, control_status));
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pose_rx).count();
    if (age_ms > kPoseTimeoutMs) {
        control_status.robot_state = "POSE_TIMEOUT";
        if (motor_driver_ptr_) {
            motor_driver_ptr_->sendCommand(cmd);
        }
        if (do_log) {
            std::cout << "[STATUS] POSE_TIMEOUT age_ms=" << age_ms << "\n";
        }
        publishStatus(buildStatus(goal, pose, cmd, control_status));
        return;
    }

    if (safety.estop || safety.obstacle_stop || safety.planner_fail) {
        control_status.robot_state = "SAFE_STOP";
        if (motor_driver_ptr_) {
            motor_driver_ptr_->sendCommand(cmd);
        }
        if (do_log) {
            std::cout << "[STATUS] SAFE_STOP estop=" << safety.estop
                      << " obstacle_stop=" << safety.obstacle_stop
                      << " planner_fail=" << safety.planner_fail << "\n";
        }
        publishStatus(buildStatus(goal, pose, cmd, control_status));
        return;
    }

    updatePathPlan(pose, goal);
    const RcGoal tracking_goal = resolveTrackingGoal(pose, goal);
    cmd = computeCommand(pose, tracking_goal, control_status);
    if (motor_driver_ptr_) {
        motor_driver_ptr_->sendCommand(cmd);
    }

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

void RcControlNode::updatePathPlan(const RcPose& pose, const RcGoal& goal) {
    const auto now = std::chrono::steady_clock::now();
    const bool goal_changed = (goal.ts_ms != planned_goal_ts_ms_);
    const bool replan_due =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_plan_at_).count() >= 250;

    if (!goal_changed && !replan_due) {
        return;
    }

    planned_path_ = BuildSmoothPath(pose, goal, config_.control.tolerance_m);
    planned_path_index_ = 0;
    tracking_goal_is_final_ = planned_path_.empty();
    planned_goal_ts_ms_ = goal.ts_ms;
    last_plan_at_ = now;

    if (goal_changed && !planned_path_.empty()) {
        std::cout << "[PATH] waypoints=" << planned_path_.size()
                  << " goal=(" << goal.x << ", " << goal.y << ")\n";
    }
}

RcGoal RcControlNode::resolveTrackingGoal(const RcPose& pose, const RcGoal& goal) {
    RcGoal tracking_goal = goal;
    tracking_goal_is_final_ = true;
    while (planned_path_index_ < planned_path_.size()) {
        const RcWaypoint& waypoint = planned_path_[planned_path_index_];
        const double dx = waypoint.x - pose.x;
        const double dy = waypoint.y - pose.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist <= waypointTolerance(waypoint.is_final)) {
            ++planned_path_index_;
            continue;
        }

        tracking_goal.x = waypoint.x;
        tracking_goal.y = waypoint.y;
        tracking_goal_is_final_ = waypoint.is_final;
        return tracking_goal;
    }
    return tracking_goal;
}

double RcControlNode::waypointTolerance(bool is_final) const {
    if (is_final) {
        return std::max(config_.control.tolerance_m, 0.12);
    }
    return std::max(config_.control.tolerance_m * 1.5, 0.18);
}

RcCommand RcControlNode::computeCommand(const RcPose& pose,
                                        const RcGoal& goal,
                                        ControlStatus& out_status) {
    constexpr double kRotateEnterTh = 35.0 * kPi / 180.0;
    constexpr double kRotateExitTh = 20.0 * kPi / 180.0;

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

    const double stop_deadzone = tracking_goal_is_final_
                                     ? waypointTolerance(true)
                                     : waypointTolerance(false);
    if (dist_m <= stop_deadzone) {
        reached_ = true;
        rotating_ = false;
        out_status.robot_state = "REACHED";
        out_status.reached = true;
        std::cout << "[SM] REACHED dist_m=" << dist_m << "\n";
        return cmd;
    }

    const double abs_err = std::fabs(err_yaw);
    if (abs_err > kRotateEnterTh) {
        rotating_ = true;
    }
    if (abs_err < kRotateExitTh) {
        rotating_ = false;
    }

    if (rotating_) {
        out_status.robot_state = "ROTATE";
        cmd.speed_mps = 0.0;
        double rotate_err = err_yaw;
        if (std::fabs(rotate_err) > config_.control.rotate_yaw_offset_rad) {
            rotate_err -= (rotate_err > 0.0 ? config_.control.rotate_yaw_offset_rad
                                            : -config_.control.rotate_yaw_offset_rad);
        } else {
            rotate_err = 0.0;
        }
        cmd.yaw_rate_rps = clamp(config_.control.k_yaw * rotate_err,
                                 -config_.control.max_yaw_rate_rps,
                                 config_.control.max_yaw_rate_rps);
        return cmd;
    }

    const double tracking_yaw_limit = config_.control.max_yaw_rate_rps * 0.6;
    cmd.yaw_rate_rps = clamp(config_.control.k_yaw * err_yaw,
                             -tracking_yaw_limit,
                             tracking_yaw_limit);

    const double heading_scale = std::max(0.05, std::cos(abs_err));
    const double yaw_slowdown =
        1.0 - std::min(0.7, std::fabs(cmd.yaw_rate_rps) / std::max(0.001, config_.control.max_yaw_rate_rps));
    cmd.speed_mps = clamp(config_.control.k_linear * dist_m * heading_scale * yaw_slowdown,
                          0.0,
                          config_.control.max_speed_mps);
    return cmd;
}

RcStatus RcControlNode::buildStatus(const RcGoal& goal,
                                    const RcPose& pose,
                                    const RcCommand& cmd,
                                    const ControlStatus& control_status) const {
    RcStatus status = CreateDefaultRcStatus(config_.status_publish_interval_ms);
    status.connected = mqtt_connected_.load();
    status.comm_state = status.connected ? "connected" : "disconnected";
    status.mode = goal.valid ? "auto" : "idle";
    status.mission = goal.valid ? "goal_tracking" : "none";
    status.battery = -1.0;
    status.speed = std::max(0.0, cmd.speed_mps);
    status.robot_state = control_status.robot_state;
    status.data_period = std::to_string(config_.status_publish_interval_ms) + "ms";

    if (pose.valid) {
        status.x = pose.x;
        status.y = pose.y;
        status.heading = RadToDeg(pose.yaw);
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
                                     config_.topics.status.c_str(),
                                     static_cast<int>(payload.size()),
                                     payload.c_str(),
                                     kMqttQos,
                                     kMqttRetain);
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

    RcStatus status = CreateDisconnectedRcStatus(config_.status_publish_interval_ms);
    status.mode = "idle";
    status.mission = "none";
    status.target = TargetInfo{};
    return publishStatus(status);
}

double RcControlNode::normalizeAngle(double rad) {
    while (rad > kPi) {
        rad -= 2.0 * kPi;
    }
    while (rad < -kPi) {
        rad += 2.0 * kPi;
    }
    return rad;
}

double RcControlNode::clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}
