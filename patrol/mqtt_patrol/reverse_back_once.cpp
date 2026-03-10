#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <mosquitto.h>
#include <regex>
#include <string>
#include <thread>

static constexpr const char* kDefaultHost = "192.168.100.10";
static constexpr int kDefaultPort = 1883;
static constexpr const char* kDefaultGoalTopic = "wiserisk/rc/goal";
static constexpr const char* kDefaultPoseTopic = "wiserisk/p1/pose";

static bool extractDouble(const std::string& json, const std::string& key, double& out) {
    const std::regex re("\"" + key + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = std::stod(m[1].str());
    return true;
}

static bool extractString(const std::string& json, const std::string& key, std::string& out) {
    const std::regex re("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = m[1].str();
    return true;
}

static long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

static bool got_pose = false;
static double pose_x = 0.0;
static double pose_y = 0.0;
static double pose_yaw = 0.0;

static void onConnect(struct mosquitto* m, void*, int rc) {
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] connect rc=" << rc << "\n";
        return;
    }
    mosquitto_subscribe(m, nullptr, kDefaultPoseTopic, 1);
    std::cout << "[OK] subscribed " << kDefaultPoseTopic << "\n";
}

static void onMessage(struct mosquitto*, void*, const struct mosquitto_message* msg) {
    if (!msg || !msg->topic || !msg->payload || msg->payloadlen <= 0) return;
    const std::string topic(msg->topic);
    if (topic != kDefaultPoseTopic) return;
    const std::string payload(static_cast<const char*>(msg->payload), msg->payloadlen);

    double x = 0.0, y = 0.0, yaw = 0.0;
    std::string frame;
    if (!extractDouble(payload, "x", x)) return;
    if (!extractDouble(payload, "y", y)) return;
    if (!extractDouble(payload, "yaw", yaw)) return;
    if (!extractString(payload, "frame", frame)) return;
    if (frame != "world") return;

    pose_x = x;
    pose_y = y;
    pose_yaw = yaw;
    got_pose = true;
}

static void usage(const char* p) {
    std::cout << "usage: " << p << " [back_dist_m] [broker_ip]\n";
    std::cout << " default: back_dist=0.6, broker=192.168.100.10\n";
}

int main(int argc, char** argv) {
    double back_dist = 0.6;
    const char* host = kDefaultHost;

    if (argc >= 2) {
        if (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
            usage(argv[0]);
            return 0;
        }
        back_dist = std::atof(argv[1]);
    }
    if (argc >= 3) host = argv[2];

    if (back_dist <= 0.0 || back_dist > 3.0) {
        std::cerr << "[ERR] back_dist should be in (0, 3]\n";
        return 1;
    }

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("reverse_back_once", true, nullptr);
    if (!mosq) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        mosquitto_lib_cleanup();
        return 1;
    }

    mosquitto_connect_callback_set(mosq, onConnect);
    mosquitto_message_callback_set(mosq, onMessage);
    if (mosquitto_connect(mosq, host, kDefaultPort, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] cannot connect broker\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }
    if (mosquitto_loop_start(mosq) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] loop_start failed\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    std::cout << "[WAIT] pose 받는 중...\n";
    for (int i = 0; i < 1500 && !got_pose; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!got_pose) {
        std::cerr << "[ERR] pose timeout\n";
        mosquitto_loop_stop(mosq, true);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 1;
    }

    const double gx = pose_x - std::cos(pose_yaw) * back_dist;
    const double gy = pose_y - std::sin(pose_yaw) * back_dist;
    const long long ts_ms = nowMs();

    char payload[256];
    std::snprintf(payload, sizeof(payload),
                  "{\"x\":%.3f,\"y\":%.3f,\"frame\":\"world\",\"ts_ms\":%lld}",
                  gx, gy, ts_ms);
    const int rc = mosquitto_publish(mosq, nullptr, kDefaultGoalTopic,
                                    static_cast<int>(std::strlen(payload)), payload, 1, false);
    if (rc == MOSQ_ERR_SUCCESS) {
        std::cout << "[OK] sent goal: " << payload << "\n";
    } else {
        std::cerr << "[ERR] publish failed rc=" << rc << "\n";
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    mosquitto_loop_stop(mosq, true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return (rc == MOSQ_ERR_SUCCESS) ? 0 : 1;
}
