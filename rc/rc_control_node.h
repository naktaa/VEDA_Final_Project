#pragma once
#include <mosquitto.h>
#include <atomic>
#include <cstdint>
#include <chrono>
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

// ??State machine ?곹깭
enum class DriveState {
    IDLE,       // goal ?놁쓬
    MEASURE,    // 2珥?硫덉땄 + yaw 痢≪젙
    ROTATE,     // 紐⑺몴 諛⑺뼢?쇰줈 ?쒖옄由??뚯쟾
    PAUSE,      // ?뚯쟾 ??2珥?硫덉땄
    DRIVE,      // 怨좎젙 yaw濡?吏곸쭊
    REACHED     // ?꾩갑
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

    double k_linear_           = 0.5;
    double k_yaw_              = 0.6;
    double max_speed_cmps_     = 80;
    double max_yaw_rate_rps_   = 1.5;
    double tolerance_cm_       = 20;  // 20cm

    struct mosquitto* mosq_ = nullptr;
    std::atomic<bool> running_{false};
    mutable std::mutex data_mtx_;

    RcGoal   goal_;
    RcPose   pose_;
    RcSafety safety_;
    std::chrono::steady_clock::time_point last_pose_rx_;

    bool motor_ready_            = false;
    mutable bool rotating_       = false;  // hysteresis ?곹깭
    mutable bool reached_        = false;  // ?꾩갑 ????goal ???뚭퉴吏 ?뺤?
    mutable RcGoal last_goal_;             // goal 蹂寃?媛먯???
    double track_width_cm_       = 22;
    double wheel_max_speed_cmps_ = 70;
    double speed_deadband_cmps_  = 3;
    int pwm_min_effective_       = 80;
    int pwm_max_                 = 220;
};
