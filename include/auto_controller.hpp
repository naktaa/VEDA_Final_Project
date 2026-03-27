#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "app_config.hpp"
#include "rc_control_types.hpp"

struct mosquitto;
struct mosquitto_message;

struct AutoStatusSnapshot {
    bool mqtt_connected = false;
    RcGoal goal;
    RcPose pose;
    RcSafety safety;
    ControlStatus control_status;
    RcCommand command;
};

class AutoController {
public:
    explicit AutoController(const AppConfig& config);
    ~AutoController();

    bool start(std::string* error = nullptr);
    void run(const std::atomic<bool>* external_run = nullptr);
    void stop();
    void cancel_goal(const char* reason);

    AutoStatusSnapshot snapshot() const;

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

    static double normalizeAngle(double rad);
    static double clamp(double v, double lo, double hi);

private:
    AppConfig config_;
    struct mosquitto* mosq_ = nullptr;
    std::atomic<bool> running_{false};
    std::atomic<bool> mqtt_connected_{false};
    mutable std::mutex data_mtx_;

    RcGoal goal_;
    RcPose pose_;
    RcSafety safety_;
    ControlStatus control_status_;
    RcCommand last_command_;
    std::chrono::steady_clock::time_point last_pose_rx_;

    mutable bool rotating_ = false;
    mutable bool reached_ = false;
    mutable RcGoal last_goal_;
    std::vector<RcWaypoint> planned_path_;
    size_t planned_path_index_ = 0;
    bool tracking_goal_is_final_ = true;
    long long planned_goal_ts_ms_ = -1;
    std::chrono::steady_clock::time_point last_plan_at_ = std::chrono::steady_clock::time_point::min();
};
