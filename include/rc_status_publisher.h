#pragma once

#include <mosquitto.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>

#include "rc_status_types.h"

class RcStatusPublisher {
public:
    static constexpr int kDefaultPublishIntervalMs = 500;

    struct Config {
        std::string broker_host = "127.0.0.1";
        int broker_port = 1883;
        std::string topic = "wiserisk/rc/status";
        std::string client_id = "rc_status_publisher";
        int publish_interval_ms = kDefaultPublishIntervalMs;
        int keepalive_sec = 30;
        int qos = 1;
        bool retain = true;
    };

    using StatusProvider = std::function<void(RcStatus& status)>;

    RcStatusPublisher();
    explicit RcStatusPublisher(const Config& config);
    ~RcStatusPublisher();

    bool start();
    void stop();

    // Call from existing control/main loop.
    void spinOnce();

    // Blocking loop for standalone execution.
    void run();

    bool publishNow();

    void setStatus(const RcStatus& status);
    RcStatus getStatusSnapshot() const;

    void setMode(const std::string& mode);
    void setMission(const std::string& mission);
    void setHeading(double heading);
    void setBattery(double battery);
    void setSpeed(double speed);
    void setPosition(double x, double y);

    // TODO(hw): connect real sensor/driving modules here so placeholders can be replaced.
    void setStatusProvider(StatusProvider provider);
    void clearStatusProvider();

private:
    static void onConnectStatic(struct mosquitto* mosq, void* obj, int rc);
    static void onDisconnectStatic(struct mosquitto* mosq, void* obj, int rc);

    void onConnect(int rc);
    void onDisconnect(int rc);

    bool ensureConnected();
    bool publishStatusSnapshot(const RcStatus& status);
    void applyStatusProvider(RcStatus& status) const;

private:
    Config config_;
    struct mosquitto* mosq_ = nullptr;

    std::atomic<bool> running_{false};
    std::atomic<bool> mqtt_connected_{false};

    mutable std::mutex status_mtx_;
    RcStatus status_;

    mutable std::mutex provider_mtx_;
    StatusProvider status_provider_;

    std::chrono::steady_clock::time_point next_publish_at_;
    std::chrono::steady_clock::time_point last_reconnect_try_;
};
