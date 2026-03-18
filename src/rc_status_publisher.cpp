#include "rc_status_publisher.h"

#include <iostream>
#include <thread>
#include <utility>

namespace {
constexpr int kReconnectRetryMs = 1000;
constexpr int kSpinSleepMs = 20;
}

RcStatusPublisher::RcStatusPublisher(Config config)
    : config_(std::move(config)),
      status_(CreateDefaultRcStatus(config_.publish_interval_ms)),
      next_publish_at_(std::chrono::steady_clock::now()),
      last_reconnect_try_(std::chrono::steady_clock::time_point::min()) {}

RcStatusPublisher::~RcStatusPublisher() {
    stop();
}

bool RcStatusPublisher::start() {
    if (running_.load()) {
        return true;
    }

    mosquitto_lib_init();

    mosq_ = mosquitto_new(config_.client_id.c_str(), true, this);
    if (!mosq_) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        mosquitto_lib_cleanup();
        return false;
    }

    mosquitto_connect_callback_set(mosq_, &RcStatusPublisher::onConnectStatic);
    mosquitto_disconnect_callback_set(mosq_, &RcStatusPublisher::onDisconnectStatic);
    mosquitto_reconnect_delay_set(mosq_, 1, 10, true);

    const RcStatus lwt_status = CreateDisconnectedRcStatus(config_.publish_interval_ms);
    const std::string lwt_payload = SerializeRcStatusToJson(lwt_status);
    int rc = mosquitto_will_set(mosq_,
                                config_.topic.c_str(),
                                static_cast<int>(lwt_payload.size()),
                                lwt_payload.c_str(),
                                config_.qos,
                                config_.retain);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_will_set failed rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ")\n";
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mosquitto_lib_cleanup();
        return false;
    }

    rc = mosquitto_connect(mosq_, config_.broker_host.c_str(), config_.broker_port, config_.keepalive_sec);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_connect failed rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ")\n";
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mosquitto_lib_cleanup();
        return false;
    }

    rc = mosquitto_loop_start(mosq_);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_loop_start failed rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ")\n";
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        mosquitto_lib_cleanup();
        return false;
    }

    mqtt_connected_ = true;
    {
        std::lock_guard<std::mutex> lk(status_mtx_);
        status_.connected = true;
        status_.comm_state = "connected";
    }

    running_ = true;
    next_publish_at_ = std::chrono::steady_clock::now();
    std::cout << "[OK] RC status publisher started. broker="
              << config_.broker_host << ":" << config_.broker_port
              << " topic=" << config_.topic
              << " interval=" << config_.publish_interval_ms << "ms\n";
    return true;
}

void RcStatusPublisher::stop() {
    running_ = false;
    mqtt_connected_ = false;

    if (mosq_) {
        mosquitto_loop_stop(mosq_, true);
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
    }
    mosquitto_lib_cleanup();
}

void RcStatusPublisher::spinOnce() {
    if (!running_.load()) {
        return;
    }

    if (!mqtt_connected_.load()) {
        ensureConnected();
    }

    const auto now = std::chrono::steady_clock::now();
    if (now < next_publish_at_) {
        return;
    }

    next_publish_at_ = now + std::chrono::milliseconds(config_.publish_interval_ms);
    publishNow();
}

void RcStatusPublisher::run() {
    while (running_.load()) {
        spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(kSpinSleepMs));
    }
}

bool RcStatusPublisher::publishNow() {
    RcStatus snapshot = getStatusSnapshot();

    applyStatusProvider(snapshot);

    snapshot.type = "rc_status";
    snapshot.src = "rc";
    snapshot.connected = mqtt_connected_.load();

    if (snapshot.comm_state.has_value()) {
        snapshot.comm_state = snapshot.connected ? "connected" : "disconnected";
    }

    if (!snapshot.data_period.has_value()) {
        snapshot.data_period = std::to_string(config_.publish_interval_ms) + "ms";
    }

    return publishStatusSnapshot(snapshot);
}

void RcStatusPublisher::setStatus(const RcStatus& status) {
    std::lock_guard<std::mutex> lk(status_mtx_);
    status_ = status;
    status_.type = "rc_status";
    status_.src = "rc";
}

RcStatus RcStatusPublisher::getStatusSnapshot() const {
    std::lock_guard<std::mutex> lk(status_mtx_);
    return status_;
}

void RcStatusPublisher::setMode(const std::string& mode) {
    std::lock_guard<std::mutex> lk(status_mtx_);
    status_.mode = mode;
}

void RcStatusPublisher::setMission(const std::string& mission) {
    std::lock_guard<std::mutex> lk(status_mtx_);
    status_.mission = mission;
}

void RcStatusPublisher::setHeading(double heading) {
    std::lock_guard<std::mutex> lk(status_mtx_);
    status_.heading = heading;
}

void RcStatusPublisher::setBattery(double battery) {
    std::lock_guard<std::mutex> lk(status_mtx_);
    status_.battery = battery;
}

void RcStatusPublisher::setSpeed(double speed) {
    std::lock_guard<std::mutex> lk(status_mtx_);
    status_.speed = speed;
}

void RcStatusPublisher::setPosition(double x, double y) {
    std::lock_guard<std::mutex> lk(status_mtx_);
    status_.x = x;
    status_.y = y;
}

void RcStatusPublisher::setStatusProvider(StatusProvider provider) {
    std::lock_guard<std::mutex> lk(provider_mtx_);
    status_provider_ = std::move(provider);
}

void RcStatusPublisher::clearStatusProvider() {
    std::lock_guard<std::mutex> lk(provider_mtx_);
    status_provider_ = nullptr;
}

void RcStatusPublisher::onConnectStatic(struct mosquitto*, void* obj, int rc) {
    if (!obj) {
        return;
    }
    static_cast<RcStatusPublisher*>(obj)->onConnect(rc);
}

void RcStatusPublisher::onDisconnectStatic(struct mosquitto*, void* obj, int rc) {
    if (!obj) {
        return;
    }
    static_cast<RcStatusPublisher*>(obj)->onDisconnect(rc);
}

void RcStatusPublisher::onConnect(int rc) {
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] MQTT connect callback rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ")\n";
        mqtt_connected_ = false;
        return;
    }

    mqtt_connected_ = true;
    {
        std::lock_guard<std::mutex> lk(status_mtx_);
        status_.connected = true;
        status_.comm_state = "connected";
    }
    std::cout << "[OK] MQTT connected\n";
}

void RcStatusPublisher::onDisconnect(int rc) {
    mqtt_connected_ = false;
    {
        std::lock_guard<std::mutex> lk(status_mtx_);
        status_.connected = false;
        status_.comm_state = "disconnected";
    }
    std::cerr << "[WARN] MQTT disconnected rc=" << rc << "\n";
}

bool RcStatusPublisher::ensureConnected() {
    if (!mosq_ || mqtt_connected_.load()) {
        return mqtt_connected_.load();
    }

    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_reconnect_try_).count();
    if (elapsed_ms < kReconnectRetryMs) {
        return false;
    }
    last_reconnect_try_ = now;

    const int rc = mosquitto_reconnect(mosq_);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[WARN] mosquitto_reconnect failed rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ")\n";
        return false;
    }

    std::cout << "[INFO] MQTT reconnect requested\n";
    return true;
}

bool RcStatusPublisher::publishStatusSnapshot(const RcStatus& status) {
    if (!mosq_) {
        std::cerr << "[ERR] publish skipped: MQTT client is null\n";
        return false;
    }

    const std::string payload = SerializeRcStatusToJson(status);
    const int rc = mosquitto_publish(mosq_,
                                     nullptr,
                                     config_.topic.c_str(),
                                     static_cast<int>(payload.size()),
                                     payload.c_str(),
                                     config_.qos,
                                     config_.retain);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] publish failed rc=" << rc
                  << " (" << mosquitto_strerror(rc) << ")\n";
        return false;
    }

    std::cout << "[OK] publish topic=" << config_.topic << " qos=" << config_.qos
              << " retain=" << (config_.retain ? "true" : "false") << "\n";
    return true;
}

void RcStatusPublisher::applyStatusProvider(RcStatus& status) const {
    std::lock_guard<std::mutex> lk(provider_mtx_);
    if (status_provider_) {
        status_provider_(status);
    }
}

