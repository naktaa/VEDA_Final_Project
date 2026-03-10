// g++ -O2 -std=c++17 patrol_10_to_13_test.cpp -o patrol_10_to_13_test -lmosquitto
/*
p1_tracker 실행해서 wall 좌표(pub) 먼저 들어오게 함
rc_control_node 실행
위 테스트 툴 실행
*/

#include <mosquitto.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <map>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <vector>

namespace {

static long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

static bool extractInt(const std::string& json, const std::string& key, int& out) {
    const std::regex re("\"" + key + "\"\\s*:\\s*([0-9]+)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    try {
        out = std::stoi(m[1].str());
    } catch (...) {
        return false;
    }
    return true;
}

static bool extractDouble(const std::string& json, const std::string& key, double& out) {
    const std::regex re("\"" + key + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    try {
        out = std::stod(m[1].str());
    } catch (...) {
        return false;
    }
    return true;
}

static bool extractBool(const std::string& json, const std::string& key, bool& out) {
    const std::regex re("\"" + key + "\"\\s*:\\s*(true|false)");
    std::smatch m;
    if (!std::regex_search(json, m, re) || m.size() < 2) return false;
    out = (m[1].str() == "true");
    return true;
}

struct MarkerInfo {
    bool seen = false;
    double x = 0.0;
    double y = 0.0;
};

struct AppState {
    std::string wall_topic;
    std::string status_topic;
    std::mutex mu;
    std::map<int, MarkerInfo> markers;
    int next_idx = 0;
    bool start_cycle = false;
    bool goal_reached = false;
    long long last_reach_ms = 0;
    long long last_goal_publish_ms = 0;
};

static std::atomic<bool> g_run{true};
AppState g_state;
long long g_last_wall_log_ms[14] = {0};

static void onSignal(int) {
    g_run = false;
}

static bool allMarkersReady(const AppState& st, const std::vector<int>& route) {
    for (int id : route) {
        const auto it = st.markers.find(id);
        if (it == st.markers.end() || !it->second.seen) return false;
    }
    return true;
}

static void publishGoal(mosquitto* mosq, const std::string& topic, int targetId, double x, double y) {
    char payload[256];
    const long long ts_ms = nowMs();
    std::snprintf(payload, sizeof(payload),
                  "{\"x\":%.3f,\"y\":%.3f,\"frame\":\"world\",\"ts_ms\":%lld,\"target_id\":%d}",
                  x, y, ts_ms, targetId);
    const int rc = mosquitto_publish(
        mosq, nullptr, topic.c_str(), (int)std::char_traits<char>::length(payload), payload, 1, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] publish goal failed rc=" << rc << " (" << mosquitto_strerror(rc) << ")\n";
    } else {
        std::cout << "[GOAL] published target_id=" << targetId
                  << " x=" << x << " y=" << y << "\n";
    }
}

static void onMessage(struct mosquitto*, void* userdata, const struct mosquitto_message* msg) {
    if (!msg || !msg->topic || !msg->payload || msg->payloadlen <= 0) return;
    const std::string topic(msg->topic);
    const std::string payload(static_cast<const char*>(msg->payload), msg->payloadlen);

    AppState* st = (userdata ? static_cast<AppState*>(userdata) : nullptr);
    if (!st) return;

    if (topic == st->wall_topic) {
        int id = 0;
        double x = 0.0;
        double y = 0.0;
        if (!extractInt(payload, "id", id) || id < 10 || id > 13) return;
        if (!extractDouble(payload, "x", x) || !extractDouble(payload, "y", y)) return;

        std::lock_guard<std::mutex> lk(st->mu);
        const bool was_seen = st->markers.count(id) && st->markers.at(id).seen;
        st->markers[id] = {true, x, y};
        const long long now_ms = nowMs();
        if ((now_ms - g_last_wall_log_ms[id]) >= 1000) {
            std::cout << "[WALL_RX] id=" << id
                      << " x=" << x << " y=" << y
                      << " seen_prev=" << (was_seen ? "true" : "false") << "\n";
            g_last_wall_log_ms[id] = now_ms;
        }
        if ((id == 10 || id == 12) && !was_seen) {
            std::cout << "[WALL_DETECT] id=" << id
                      << " x=" << x << " y=" << y << "\n";
        }
        return;
    }

    if (topic == st->status_topic) {
        bool reached = false;
        if (extractBool(payload, "reached", reached) && reached) {
            std::lock_guard<std::mutex> lk(st->mu);
            st->goal_reached = true;
            st->last_reach_ms = nowMs();
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    const std::string host = (argc > 1) ? argv[1] : "192.168.100.10";
    const int port = (argc > 2) ? std::stoi(argv[2]) : 1883;
    const std::string topic_goal = (argc > 3) ? argv[3] : "wiserisk/rc/goal";
    const std::string topic_walls = (argc > 4) ? argv[4] : "wiserisk/map/walls";
    const std::string topic_status = (argc > 5) ? argv[5] : "wiserisk/rc/status";

    g_state.wall_topic = topic_walls;
    g_state.status_topic = topic_status;

    const std::vector<int> route = {10, 12};

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    mosquitto_lib_init();
    mosquitto* mosq = mosquitto_new("patrol_10_to_13_test", true, nullptr);
    if (!mosq) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        mosquitto_lib_cleanup();
        return 1;
    }

    mosquitto_user_data_set(mosq, &g_state);
    mosquitto_message_callback_set(mosq, onMessage);
    mosquitto_reconnect_delay_set(mosq, 1, 5, true);

    if (mosquitto_connect(mosq, host.c_str(), port, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] connect failed\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 2;
    }

    if (mosquitto_loop_start(mosq) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] loop_start failed\n";
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 3;
    }

    if (mosquitto_subscribe(mosq, nullptr, topic_walls.c_str(), 1) != MOSQ_ERR_SUCCESS ||
        mosquitto_subscribe(mosq, nullptr, topic_status.c_str(), 1) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] subscribe failed\n";
        g_run = false;
    } else {
        std::cout << "[OK] subscribed: " << topic_walls << ", " << topic_status << "\n";
    }

    std::cout << "[INFO] waiting wall markers";
    for (size_t i = 0; i < route.size(); ++i) {
        std::cout << (i == 0 ? " " : ", ") << route[i];
    }
    std::cout << ", then send goals to " << topic_goal << "\n";
    const long long startMs = nowMs();

    while (g_run) {
        {
            std::lock_guard<std::mutex> lk(g_state.mu);
            if (!g_state.start_cycle && allMarkersReady(g_state, route)) {
                g_state.start_cycle = true;
                const int target = route[g_state.next_idx];
                const auto it = g_state.markers.find(target);
                if (it != g_state.markers.end()) {
                    publishGoal(mosq, topic_goal, target, it->second.x, it->second.y);
                    g_state.last_goal_publish_ms = nowMs();
                }
            } else if (g_state.start_cycle && g_state.goal_reached) {
                g_state.goal_reached = false;
                g_state.next_idx++;

                if (g_state.next_idx >= static_cast<int>(route.size())) {
                    std::cout << "[DONE] completed route:";
                    for (size_t i = 0; i < route.size(); ++i) {
                        std::cout << (i == 0 ? " " : " -> ") << route[i];
                    }
                    std::cout << "\n";
                    g_run = false;
                    break;
                }

                const int target = route[g_state.next_idx];
                const auto it = g_state.markers.find(target);
                if (it != g_state.markers.end()) {
                    publishGoal(mosq, topic_goal, target, it->second.x, it->second.y);
                    g_state.last_goal_publish_ms = nowMs();
                } else {
                    std::cerr << "[WARN] target marker not found: " << target << "\n";
                }
            }
        }

        const long long now_ms = nowMs();
        if (now_ms - startMs > 120000 && !g_state.start_cycle) {
            std::cout << "[WARN] waiting too long for markers. keep waiting...\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    mosquitto_loop_stop(mosq, true);
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
