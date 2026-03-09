#pragma once

#include <mosquitto.h>

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

struct RcGoal {
    double x = 0.0;
    double y = 0.0;
    std::string frame = "world";
    long long ts_ms = 0;
    bool valid = false;
};

struct RcPose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0; // rad
    std::string frame = "world";
    long long ts_ms = 0;
    bool valid = false;
};

struct RcSafety {
    bool estop = false;
    bool obstacle_stop = false;
    bool planner_fail = false;
};

struct RcStatus {
    std::string mode = "IDLE";
    bool reached = false;
    double err_dist = 0.0;
    double err_yaw = 0.0;
    double battery = -1.0; // unknown
};

struct RcCommand {
    double speed_mps = 0.0;
    double yaw_rate_rps = 0.0;
};

struct PlannerCell {
    int x = 0;
    int y = 0;
};

class RcControlNode {
public:
    RcControlNode(const std::string& broker_host,
                  int broker_port,
                  const std::string& topic_goal,
                  const std::string& topic_pose,
                  const std::string& topic_safety,
                  const std::string& topic_status);
    ~RcControlNode();

    bool start();
    void run();
    void stop();

    void setControlParams(double k_linear,
                          double k_yaw,
                          double max_speed_mps,
                          double max_yaw_rate_rps,
                          double tolerance_m);

private:
    static void onConnectStatic(struct mosquitto* mosq, void* obj, int rc);
    static void onMessageStatic(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg);

    void onConnect(int rc);
    void onMessage(const struct mosquitto_message* msg);

    void controlStep();
    RcCommand computeCommand(const RcPose& pose,
                             const RcGoal& goal,
                             RcStatus& out_status,
                             double reach_tolerance_m) const;
    bool publishStatus(const RcStatus& status);
    void sendCommandToRc(const RcCommand& cmd, int servo_us) const;
    int computeServoUs(const RcCommand& cmd) const;
    bool makePlanIfNeeded(const RcPose& pose, const RcGoal& goal, std::string& reason);
    bool pickActiveWaypoint(const RcPose& pose, RcGoal& out_target, bool& out_final_wp, double& out_dist_to_target);

    static double normalizeAngle(double rad);
    static double clamp(double v, double lo, double hi);

    static bool parseGoalJson(const std::string& payload, RcGoal& out_goal);
    static bool parsePoseJson(const std::string& payload, RcPose& out_pose);
    static bool parseSafetyJson(const std::string& payload, RcSafety& out_safety);
    static bool parseWallMarkerJson(const std::string& payload, int& out_id, double& out_x, double& out_y);

private:
    std::string host_;
    int port_ = 1883;

    std::string topic_goal_;
    std::string topic_pose_;
    std::string topic_safety_;
    std::string topic_status_;
    std::string topic_walls_ = "wiserisk/map/walls";

    double k_linear_ = 0.8;
    double k_yaw_ = 1.2;
    double max_speed_mps_ = 0.8;
    double max_yaw_rate_rps_ = 1.5;
    double tolerance_m_ = 0.15;
    double waypoint_tolerance_m_ = 0.12;

    struct mosquitto* mosq_ = nullptr;
    std::atomic<bool> running_{false};

    mutable std::mutex data_mtx_;
    RcGoal goal_;
    RcPose pose_;
    RcSafety safety_;
    bool reached_hold_ = false;

    std::map<int, std::pair<double, double>> wall_markers_{
        {10, {6.0, 4.0}},
        {11, {6.0, 5.0}},
        {12, {6.0, 3.0}},
        {13, {6.0, 6.0}},
    };

    std::vector<std::pair<double, double>> waypoints_;
    std::size_t waypoint_idx_ = 0;
    bool plan_valid_ = false;
    long long plan_goal_ts_ms_ = -1;
    long long last_plan_ts_ms_ = 0;
};
