#pragma once

#include <mosquitto.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

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
    double battery = -1.0;
};

struct RcCommand {
    double speed_cmps = 0.0;
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
                          double max_speed_cmps,
                          double max_yaw_rate_rps,
                          double tolerance_cm);
    void setMotorParams(double track_width_cm,
                        double wheel_max_speed_cmps,
                        double speed_deadband_cmps,
                        int pwm_min_effective,
                        int pwm_max);
    bool loadParamsFromIni(const std::string& ini_path);

private:
    static void onConnectStatic(struct mosquitto* mosq, void* obj, int rc);
    static void onMessageStatic(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg);
    void onConnect(int rc);
    void onMessage(const struct mosquitto_message* msg);
    void controlStep();
    RcCommand computeCommand(const RcPose& pose, const RcGoal& goal, RcStatus& out_status) const;
    bool publishStatus(const RcStatus& status);
    void sendCommandToRc(const RcCommand& cmd) const;
    void setupMotorDriver();
    void stopAllMotors() const;
    void setMotorControl(int en, int in1, int in2, int speed_pwm, int dir) const;
    int speedToPwm(double speed_cmps) const;
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
    double max_speed_cmps_ = 70.0;
    double max_yaw_rate_rps_ = 0.5;
    double tolerance_cm_ = 10.0;
    double rotate_yaw_offset_rad_ = 5.0 * 3.14159265358979323846 / 180.0;

    struct mosquitto* mosq_ = nullptr;
    std::atomic<bool> running_{false};
    mutable std::mutex data_mtx_;

    RcGoal goal_;
    RcPose pose_;
    RcSafety safety_;
    std::chrono::steady_clock::time_point last_pose_rx_;

    bool motor_ready_ = false;
    mutable bool rotating_ = false;
    mutable bool reached_ = false;
    mutable RcGoal last_goal_;
    double track_width_cm_ = 22.0;
    double wheel_max_speed_cmps_ = 70.0;
    double speed_deadband_cmps_ = 0.3;
    int pwm_min_effective_ = 110;
    int pwm_max_ = 220;
};
