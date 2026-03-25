#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <vector>

#include "rc_control_types.hpp"
#include "rc_path_planner.hpp"
#include "rc_status_types.h"

struct mosquitto;
struct mosquitto_message;
class RcMotorDriver;

class RcControlNode {
public:
    explicit RcControlNode(const RcAppConfig& config);
    ~RcControlNode();

    bool start();
    void run(const std::atomic<bool>* external_run = nullptr);
    void stop();

private:
    static void onConnectStatic(struct mosquitto* mosq, void* obj, int rc);
    static void onDisconnectStatic(struct mosquitto* mosq, void* obj, int rc);
    static void onMessageStatic(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg);

    void onConnect(int rc);
    void onDisconnect(int rc);
    void onMessage(const struct mosquitto_message* msg);
    void controlStep();
    void updatePathPlan(const RcPose& pose, const RcGoal& goal);
    RcGoal resolveTrackingGoal(const RcPose& pose, const RcGoal& goal);
    double waypointTolerance(bool is_final) const;

    RcCommand computeCommand(const RcPose& pose, const RcGoal& goal, ControlStatus& out_status);
    RcStatus buildStatus(const RcGoal& goal,
                         const RcPose& pose,
                         const RcCommand& cmd,
                         const ControlStatus& control_status) const;
    bool publishStatus(const RcStatus& status);
    bool publishDisconnectedStatus();

    static double normalizeAngle(double rad);
    static double clamp(double v, double lo, double hi);

private:
    RcAppConfig config_;
    struct mosquitto* mosq_ = nullptr;
    std::atomic<bool> running_{false};
    std::atomic<bool> mqtt_connected_{false};
    mutable std::mutex data_mtx_;

    RcGoal goal_;
    RcPose pose_;
    RcSafety safety_;
    std::chrono::steady_clock::time_point last_pose_rx_;

    mutable bool rotating_ = false;
    mutable bool reached_ = false;
    mutable RcGoal last_goal_;
    std::vector<RcWaypoint> planned_path_;
    size_t planned_path_index_ = 0;
    bool tracking_goal_is_final_ = true;
    long long planned_goal_ts_ms_ = -1;
    std::chrono::steady_clock::time_point last_plan_at_ = std::chrono::steady_clock::time_point::min();

    RcMotorDriver* motor_driver_ptr_ = nullptr;
};
