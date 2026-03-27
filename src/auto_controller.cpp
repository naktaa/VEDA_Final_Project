#include "auto_controller.hpp"

#include <mosquitto.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "rc_json_utils.hpp"
#include "rc_path_planner.hpp"
#include "tank_drive.hpp"

namespace {

constexpr int kPoseTimeoutMs = 800;
constexpr int kControlStepMs = 50;
constexpr double kPi = 3.14159265358979323846;

RcPose pose_for_control(const RcPose& pose) {
    RcPose corrected = pose;
    corrected.yaw = -corrected.yaw;
    return corrected;
}

} // namespace

AutoController::AutoController(const AppConfig& config)
    : config_(config),
      last_pose_rx_(std::chrono::steady_clock::time_point::min()) {}

AutoController::~AutoController() {
    stop();
}

bool AutoController::start(std::string* error) {
    if (running_.load()) {
        return true;
    }

    mosq_ = mosquitto_new("tank_auto_controller", true, this);
    if (!mosq_) {
        if (error) {
            *error = "mosquitto_new failed";
        }
        return false;
    }

    mosquitto_connect_callback_set(mosq_, &AutoController::onConnectStatic);
    mosquitto_disconnect_callback_set(mosq_, &AutoController::onDisconnectStatic);
    mosquitto_message_callback_set(mosq_, &AutoController::onMessageStatic);
    mosquitto_reconnect_delay_set(mosq_, 1, 10, true);

    const int rc = mosquitto_connect(mosq_,
                                     config_.mqtt.host.c_str(),
                                     config_.mqtt.port,
                                     config_.mqtt.keepalive_sec);
    if (rc != MOSQ_ERR_SUCCESS) {
        if (error) {
            *error = std::string("mosquitto_connect failed: ") + mosquitto_strerror(rc);
        }
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        return false;
    }

    const int loop_rc = mosquitto_loop_start(mosq_);
    if (loop_rc != MOSQ_ERR_SUCCESS) {
        if (error) {
            *error = std::string("mosquitto_loop_start failed: ") + mosquitto_strerror(loop_rc);
        }
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        return false;
    }

    running_ = true;
    return true;
}

void AutoController::run(const std::atomic<bool>* external_run) {
    while (running_.load() && (!external_run || external_run->load())) {
        controlStep();
        std::this_thread::sleep_for(std::chrono::milliseconds(kControlStepMs));
    }
}

void AutoController::stop() {
    if (!mosq_ && !running_.load()) {
        return;
    }

    running_ = false;
    tank_drive::stop_from(tank_drive::DriveSource::kAuto);

    if (mosq_) {
        mosquitto_loop_stop(mosq_, true);
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
    }
    mqtt_connected_ = false;
}

AutoStatusSnapshot AutoController::snapshot() const {
    std::lock_guard<std::mutex> lock(data_mtx_);
    AutoStatusSnapshot out;
    out.mqtt_connected = mqtt_connected_.load();
    out.goal = goal_;
    out.pose = pose_;
    out.safety = safety_;
    out.control_status = control_status_;
    out.command = last_command_;
    return out;
}

void AutoController::onConnectStatic(struct mosquitto*, void* obj, int rc) {
    if (obj) {
        static_cast<AutoController*>(obj)->onConnect(rc);
    }
}

void AutoController::onDisconnectStatic(struct mosquitto*, void* obj, int rc) {
    if (obj) {
        static_cast<AutoController*>(obj)->onDisconnect(rc);
    }
}

void AutoController::onMessageStatic(struct mosquitto*,
                                     void* obj,
                                     const struct mosquitto_message* msg) {
    if (obj) {
        static_cast<AutoController*>(obj)->onMessage(msg);
    }
}

void AutoController::onConnect(int rc) {
    if (rc != MOSQ_ERR_SUCCESS) {
        mqtt_connected_ = false;
        std::cerr << "[AUTO] MQTT connect callback failed: " << mosquitto_strerror(rc) << "\n";
        return;
    }

    mqtt_connected_ = true;
    mosquitto_subscribe(mosq_, nullptr, config_.mqtt.goal_topic.c_str(), 1);
    mosquitto_subscribe(mosq_, nullptr, config_.mqtt.pose_topic.c_str(), 1);
    mosquitto_subscribe(mosq_, nullptr, config_.mqtt.safety_topic.c_str(), 1);
    std::cerr << "[AUTO] subscribed goal=" << config_.mqtt.goal_topic
              << " pose=" << config_.mqtt.pose_topic
              << " safety=" << config_.mqtt.safety_topic << "\n";
}

void AutoController::onDisconnect(int rc) {
    mqtt_connected_ = false;
    tank_drive::stop_from(tank_drive::DriveSource::kAuto);
    std::cerr << "[AUTO] MQTT disconnected rc=" << rc << "\n";
}

void AutoController::onMessage(const struct mosquitto_message* msg) {
    if (!msg || !msg->topic || !msg->payload || msg->payloadlen <= 0) {
        return;
    }

    const std::string topic(msg->topic);
    const std::string payload(static_cast<const char*>(msg->payload),
                              static_cast<size_t>(msg->payloadlen));

    std::lock_guard<std::mutex> lock(data_mtx_);
    if (topic == config_.mqtt.goal_topic) {
        RcGoal next_goal;
        if (ParseGoalJson(payload, next_goal) && next_goal.frame == "world") {
            goal_ = next_goal;
            static int goal_rx_count = 0;
            if (++goal_rx_count % 10 == 0) {
                std::cerr << "[GOAL] x=" << next_goal.x
                          << " y=" << next_goal.y
                          << " frame=" << next_goal.frame
                          << " ts_ms=" << next_goal.ts_ms << "\n";
            }
        } else if (ParseGoalJson(payload, next_goal)) {
            std::cerr << "[GOAL] ignored: unsupported frame=" << next_goal.frame
                      << " payload=" << payload << "\n";
        } else {
            std::cerr << "[GOAL] ignored: parse failed payload=" << payload << "\n";
        }
        return;
    }

    if (topic == config_.mqtt.pose_topic) {
        RcPose next_pose;
        if (ParsePoseJson(payload, next_pose) && next_pose.frame == "world") {
            pose_ = next_pose;
            last_pose_rx_ = std::chrono::steady_clock::now();
            static int pose_rx_count = 0;
            if (++pose_rx_count % 10 == 0) {
                std::cerr << "[POSE] x=" << next_pose.x
                          << " y=" << next_pose.y
                          << " yaw=" << next_pose.yaw
                          << " frame=" << next_pose.frame
                          << " ts_ms=" << next_pose.ts_ms << "\n";
            }
        } else if (ParsePoseJson(payload, next_pose)) {
            std::cerr << "[POSE] ignored: unsupported frame=" << next_pose.frame
                      << " payload=" << payload << "\n";
        } else {
            std::cerr << "[POSE] ignored: parse failed payload=" << payload << "\n";
        }
        return;
    }

    if (topic == config_.mqtt.safety_topic) {
        RcSafety next_safety;
        if (ParseSafetyJson(payload, next_safety)) {
            safety_ = next_safety;
        }
    }
}

void AutoController::controlStep() {
    RcGoal goal;
    RcPose pose;
    RcSafety safety;
    std::chrono::steady_clock::time_point last_pose_rx;
    {
        std::lock_guard<std::mutex> lock(data_mtx_);
        goal = goal_;
        pose = pose_;
        safety = safety_;
        last_pose_rx = last_pose_rx_;
    }

    ControlStatus control_status;
    RcCommand cmd;
    const RcPose control_pose = pose_for_control(pose);

    if (!pose.valid) {
        control_status.robot_state = "WAIT_INPUT";
        tank_drive::stop_from(tank_drive::DriveSource::kAuto);
    } else if (!goal.valid) {
        control_status.robot_state = "WAIT_GOAL";
        tank_drive::stop_from(tank_drive::DriveSource::kAuto);
    } else {
        const auto age_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - last_pose_rx)
                .count();
        if (age_ms > kPoseTimeoutMs) {
            control_status.robot_state = "POSE_TIMEOUT";
            tank_drive::stop_from(tank_drive::DriveSource::kAuto);
        } else if (safety.estop || safety.obstacle_stop || safety.planner_fail) {
            control_status.robot_state = "SAFE_STOP";
            tank_drive::stop_from(tank_drive::DriveSource::kAuto);
        } else {
            updatePathPlan(control_pose, goal);
            const RcGoal tracking_goal = resolveTrackingGoal(control_pose, goal);
            cmd = computeCommand(control_pose, tracking_goal, control_status);
            tank_drive::command_auto(cmd, config_.motor);
        }
    }

    {
        std::lock_guard<std::mutex> lock(data_mtx_);
        control_status_ = control_status;
        last_command_ = cmd;
    }
}

void AutoController::updatePathPlan(const RcPose& pose, const RcGoal& goal) {
    const auto now = std::chrono::steady_clock::now();
    const bool goal_changed = (goal.ts_ms != planned_goal_ts_ms_);
    const bool replan_due =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_plan_at_).count() >= 250;

    if (!goal_changed && !replan_due) {
        return;
    }

    planned_path_ = BuildSmoothPath(pose, goal, config_.auto_control.tolerance_cm);
    planned_path_index_ = 0;
    tracking_goal_is_final_ = planned_path_.empty();
    planned_goal_ts_ms_ = goal.ts_ms;
    last_plan_at_ = now;
}

RcGoal AutoController::resolveTrackingGoal(const RcPose& pose, const RcGoal& goal) {
    RcGoal tracking_goal = goal;
    tracking_goal_is_final_ = true;
    while (planned_path_index_ < planned_path_.size()) {
        const RcWaypoint& waypoint = planned_path_[planned_path_index_];
        const double dx = waypoint.x - pose.x;
        const double dy = waypoint.y - pose.y;
        const double dist = std::sqrt((dx * dx) + (dy * dy));
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

double AutoController::waypointTolerance(bool is_final) const {
    if (is_final) {
        return std::max(config_.auto_control.tolerance_cm, 12.0);
    }
    return std::max(config_.auto_control.tolerance_cm * 1.5, 18.0);
}

RcCommand AutoController::computeCommand(const RcPose& pose,
                                         const RcGoal& goal,
                                         ControlStatus& out_status) {
    constexpr double kRotateEnterTh = 35.0 * kPi / 180.0;
    constexpr double kRotateExitTh = 20.0 * kPi / 180.0;
    constexpr double kRotateMinRateFracNear = 0.45;
    constexpr double kRotateMinRateFracFar = 0.85;

    const double dx = goal.x - pose.x;
    const double dy = goal.y - pose.y;
    const double dist_cm = std::sqrt((dx * dx) + (dy * dy));
    const double target_heading = std::atan2(dy, dx);
    const double err_yaw = normalizeAngle(target_heading - pose.yaw);

    RcCommand cmd;
    out_status.err_dist_cm = dist_cm;
    out_status.err_yaw_rad = err_yaw;

    if (goal.ts_ms != last_goal_.ts_ms) {
        last_goal_ = goal;
        reached_ = false;
        rotating_ = false;
    }

    if (reached_) {
        out_status.robot_state = "REACHED";
        out_status.reached = true;
        return cmd;
    }

    out_status.robot_state = "TRACKING";

    const double stop_deadzone =
        tracking_goal_is_final_ ? waypointTolerance(true) : waypointTolerance(false);
    if (dist_cm <= stop_deadzone) {
        reached_ = true;
        rotating_ = false;
        out_status.robot_state = "REACHED";
        out_status.reached = true;
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
        cmd.speed_cmps = 0.0;
        cmd.turn_effort =
            clamp((abs_err - kRotateExitTh) / std::max(0.001, kPi - kRotateExitTh), 0.0, 1.0);
        double rotate_err = err_yaw;
        if (std::fabs(rotate_err) > config_.auto_control.rotate_yaw_offset_rad) {
            rotate_err -= (rotate_err > 0.0 ? config_.auto_control.rotate_yaw_offset_rad
                                            : -config_.auto_control.rotate_yaw_offset_rad);
        } else {
            rotate_err = 0.0;
            cmd.turn_effort = 0.0;
        }
        cmd.yaw_rate_rps = clamp(config_.auto_control.k_yaw * rotate_err,
                                 -config_.auto_control.max_yaw_rate_rps,
                                 config_.auto_control.max_yaw_rate_rps);
        if (rotate_err != 0.0) {
            const double rotate_blend =
                clamp((abs_err - kRotateExitTh) / std::max(0.001, kPi - kRotateExitTh), 0.0, 1.0);
            const double min_rotate_rate =
                config_.auto_control.max_yaw_rate_rps *
                (kRotateMinRateFracNear +
                 ((kRotateMinRateFracFar - kRotateMinRateFracNear) * rotate_blend));
            if (std::fabs(cmd.yaw_rate_rps) < min_rotate_rate) {
                cmd.yaw_rate_rps = (rotate_err > 0.0 ? min_rotate_rate : -min_rotate_rate);
            }
        }
        return cmd;
    }

    cmd.turn_effort = 0.0;
    const double tracking_yaw_limit = config_.auto_control.max_yaw_rate_rps * 0.6;
    cmd.yaw_rate_rps = clamp(config_.auto_control.k_yaw * err_yaw,
                             -tracking_yaw_limit,
                             tracking_yaw_limit);

    const double heading_scale = std::max(0.05, std::cos(abs_err));
    const double yaw_slowdown =
        1.0 - std::min(0.7,
                       std::fabs(cmd.yaw_rate_rps) /
                           std::max(0.001, config_.auto_control.max_yaw_rate_rps));
    cmd.speed_cmps = clamp(config_.auto_control.k_linear * dist_cm * heading_scale * yaw_slowdown,
                           0.0,
                           config_.auto_control.max_speed_cmps);
    return cmd;
}

double AutoController::normalizeAngle(double rad) {
    while (rad > kPi) {
        rad -= 2.0 * kPi;
    }
    while (rad < -kPi) {
        rad += 2.0 * kPi;
    }
    return rad;
}

double AutoController::clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}
