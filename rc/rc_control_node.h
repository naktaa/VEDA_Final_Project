#pragma once

#include <mosquitto.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "rc_status_types.h"

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

struct ControlStatus {
    std::string robot_state = "WAIT_INPUT";
    bool reached = false;
    double err_dist_m = 0.0;
    double err_yaw_rad = 0.0;
};

struct RcCommand {
    double speed_mps = 0.0;
    double yaw_rate_rps = 0.0;
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
    void setMotorParams(double track_width_m,
                        double wheel_max_speed_mps,
                        double speed_deadband_mps,
                        int pwm_min_effective,
                        int pwm_max);
    bool loadParamsFromIni(const std::string& ini_path);

private:
    static void onConnectStatic(struct mosquitto* mosq, void* obj, int rc);
    static void onDisconnectStatic(struct mosquitto* mosq, void* obj, int rc);
    static void onMessageStatic(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg);

    void onConnect(int rc);
    void onDisconnect(int rc);
    void onMessage(const struct mosquitto_message* msg);
    void controlStep();

    RcCommand computeCommand(const RcPose& pose, const RcGoal& goal, ControlStatus& out_status) const;
    RcStatus buildStatus(const RcGoal& goal,
                         const RcPose& pose,
                         const RcCommand& cmd,
                         const ControlStatus& control_status) const;
    bool publishStatus(const RcStatus& status);
    bool publishDisconnectedStatus();

    void sendCommandToRc(const RcCommand& cmd) const;
    void setupMotorDriver();
    void stopAllMotors() const;
    void setMotorControl(int en, int in1, int in2, int speed_pwm, int dir) const;
    int speedToPwm(double speed_mps) const;

    static double normalizeAngle(double rad);
    static double clamp(double v, double lo, double hi);
    static bool parseGoalJson(const std::string& payload, RcGoal& out_goal);
    static bool parsePoseJson(const std::string& payload, RcPose& out_pose);
    static bool parseSafetyJson(const std::string& payload, RcSafety& out_safety);

private:
    std::string host_;
    int port_ = 1883;
    std::string topic_goal_;
    std::string topic_pose_;
    std::string topic_safety_;
    std::string topic_status_;

    double k_linear_ = 0.5;
    double k_yaw_ = 0.8;
    double max_speed_mps_ = 0.70;
    double max_yaw_rate_rps_ = 0.5;
    double tolerance_m_ = 0.10;
    double rotate_yaw_offset_rad_ = 5.0 * 3.14159265358979323846 / 180.0;

    struct mosquitto* mosq_ = nullptr;
    std::atomic<bool> running_{false};
    std::atomic<bool> mqtt_connected_{false};
    mutable std::mutex data_mtx_;

    RcGoal goal_;
    RcPose pose_;
    RcSafety safety_;
    std::chrono::steady_clock::time_point last_pose_rx_;

    bool motor_ready_ = false;
    mutable bool rotating_ = false;
    mutable bool reached_ = false;
    mutable RcGoal last_goal_;

    double track_width_m_ = 0.22;
    double wheel_max_speed_mps_ = 0.70;
    double speed_deadband_mps_ = 0.003;
    int pwm_min_effective_ = 110;
    int pwm_max_ = 220;
    int status_publish_interval_ms_ = 50;
};
