#include <mosquitto.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <regex>
#include <string>
#include <thread>

struct GoalMsg {
    double x = 0.0;
    double y = 0.0;
    std::string frame;
    long long ts_ms = 0;
};

static std::atomic<bool> g_run{true};

static void onSignal(int) {
    g_run = false;
}

static bool extractDouble(const std::string& json, const std::string& key, double& out) {
    const std::regex re("\"" + key + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = std::stod(m[1].str());
    return true;
}

static bool extractInt64(const std::string& json, const std::string& key, long long& out) {
    const std::regex re("\"" + key + "\"\\s*:\\s*([0-9]+)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = std::stoll(m[1].str());
    return true;
}

static bool extractString(const std::string& json, const std::string& key, std::string& out) {
    const std::regex re("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = m[1].str();
    return true;
}

static bool parseGoalJson(const std::string& payload, GoalMsg& goal) {
    if (!extractDouble(payload, "x", goal.x)) return false;
    if (!extractDouble(payload, "y", goal.y)) return false;
    if (!extractString(payload, "frame", goal.frame)) return false;
    if (!extractInt64(payload, "ts_ms", goal.ts_ms)) return false;
    return true;
}

static void onConnect(struct mosquitto* mosq, void* userdata, int rc) {
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] MQTT connect callback rc=" << rc << "\n";
        return;
    }

    const std::string* topic = static_cast<std::string*>(userdata);
    const int subRc = mosquitto_subscribe(mosq, nullptr, topic->c_str(), 1);
    if (subRc == MOSQ_ERR_SUCCESS) {
        std::cout << "[OK] subscribed topic: " << *topic << "\n";
    } else {
        std::cerr << "[ERR] subscribe failed rc=" << subRc << " topic=" << *topic << "\n";
    }
}

static void onMessage(struct mosquitto*, void*, const struct mosquitto_message* msg) {
    if (!msg || !msg->payload || msg->payloadlen <= 0) return;

    const std::string payload(static_cast<const char*>(msg->payload), msg->payloadlen);
    GoalMsg goal;
    if (!parseGoalJson(payload, goal)) {
        std::cerr << "[WARN] invalid goal json: " << payload << "\n";
        return;
    }
    if (goal.frame != "world") {
        std::cerr << "[WARN] unsupported frame: " << goal.frame << "\n";
        return;
    }

    std::cout << std::fixed << std::setprecision(3)
              << "[GOAL] x=" << goal.x
              << " y=" << goal.y
              << " frame=" << goal.frame
              << " ts_ms=" << goal.ts_ms << "\n";

    // Next step: pass this goal to RC control node (speed/steering command generation).
}

int main(int argc, char** argv) {
    const std::string brokerHost = (argc > 1) ? argv[1] : "192.168.100.10";
    const int brokerPort = (argc > 2) ? std::stoi(argv[2]) : 1883;
    const std::string goalTopic = (argc > 3) ? argv[3] : "wiserisk/rc/goal";

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("rc_goal_subscriber", true, const_cast<std::string*>(&goalTopic));
    if (!mosq) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        mosquitto_lib_cleanup();
        return 1;
    }

    mosquitto_connect_callback_set(mosq, onConnect);
    mosquitto_message_callback_set(mosq, onMessage);
    mosquitto_reconnect_delay_set(  mosq, 1, 10, true);

    const int rc = mosquitto_connect(mosq, brokerHost.c_str(), brokerPort, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_connect failed rc=" << rc << " (" << mosquitto_strerror(rc) << ")\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 2;
    }

    if (mosquitto_loop_start(mosq) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_loop_start failed\n";
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 3;
    }

    std::cout << "[INFO] listening broker=" << brokerHost << ":" << brokerPort
              << " topic=" << goalTopic << "\n";

    while (g_run) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    mosquitto_loop_stop(mosq, true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
