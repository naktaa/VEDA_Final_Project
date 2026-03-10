#include <mosquitto.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

struct Options {
    std::string broker = "192.168.100.10";
    int port = 1883;
    std::string topic_pose = "wiserisk/p1/pose";
    std::string topic_goal = "wiserisk/rc/goal";
    std::string topic_safety = "wiserisk/rc/safety";
    std::string dir = "left";   // up/down/left/right
    double dist = 2.0;
    std::string mode = "move";  // move/fixed
    double x0 = 0.0;
    double y0 = 0.0;
    double yaw0 = 0.0;
};

static long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

static void printUsage() {
    std::cout
        << "RC MQTT test publisher (C++)\n"
        << "Options:\n"
        << "  --broker <ip>            (default: 192.168.100.10)\n"
        << "  --port <int>             (default: 1883)\n"
        << "  --topic-pose <topic>     (default: wiserisk/p1/pose)\n"
        << "  --topic-goal <topic>     (default: wiserisk/rc/goal)\n"
        << "  --topic-safety <topic>   (default: wiserisk/rc/safety)\n"
        << "  --dir <up|down|left|right> (default: left)\n"
        << "  --dist <m>               (default: 2.0)\n"
        << "  --mode <move|fixed>      (default: move)\n"
        << "  --x0 <double>            (default: 0.0)\n"
        << "  --y0 <double>            (default: 0.0)\n"
        << "  --yaw0 <rad>             (default: 0.0)\n";
}

static bool parseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto needValue = [&](const std::string& name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (a == "--help" || a == "-h") {
            printUsage();
            return false;
        } else if (a == "--broker") {
            const char* v = needValue(a);
            if (!v) return false;
            opt.broker = v;
        } else if (a == "--port") {
            const char* v = needValue(a);
            if (!v) return false;
            opt.port = std::stoi(v);
        } else if (a == "--topic-pose") {
            const char* v = needValue(a);
            if (!v) return false;
            opt.topic_pose = v;
        } else if (a == "--topic-goal") {
            const char* v = needValue(a);
            if (!v) return false;
            opt.topic_goal = v;
        } else if (a == "--topic-safety") {
            const char* v = needValue(a);
            if (!v) return false;
            opt.topic_safety = v;
        } else if (a == "--dir") {
            const char* v = needValue(a);
            if (!v) return false;
            opt.dir = v;
        } else if (a == "--dist") {
            const char* v = needValue(a);
            if (!v) return false;
            opt.dist = std::stod(v);
        } else if (a == "--mode") {
            const char* v = needValue(a);
            if (!v) return false;
            opt.mode = v;
        } else if (a == "--x0") {
            const char* v = needValue(a);
            if (!v) return false;
            opt.x0 = std::stod(v);
        } else if (a == "--y0") {
            const char* v = needValue(a);
            if (!v) return false;
            opt.y0 = std::stod(v);
        } else if (a == "--yaw0") {
            const char* v = needValue(a);
            if (!v) return false;
            opt.yaw0 = std::stod(v);
        } else {
            std::cerr << "Unknown option: " << a << "\n";
            printUsage();
            return false;
        }
    }

    if (!(opt.dir == "up" || opt.dir == "down" || opt.dir == "left" || opt.dir == "right")) {
        std::cerr << "Invalid --dir: " << opt.dir << "\n";
        return false;
    }
    if (!(opt.mode == "move" || opt.mode == "fixed")) {
        std::cerr << "Invalid --mode: " << opt.mode << "\n";
        return false;
    }
    return true;
}

static void directionToGoal(const std::string& dir, double dist, double& gx, double& gy) {
    if (dir == "up") {
        gx = dist; gy = 0.0;
    } else if (dir == "down") {
        gx = -dist; gy = 0.0;
    } else if (dir == "left") {
        gx = 0.0; gy = dist;
    } else {
        gx = 0.0; gy = -dist;
    }
}

static bool mqttPublish(struct mosquitto* mosq, const std::string& topic, const std::string& payload) {
    const int rc = mosquitto_publish(
        mosq, nullptr, topic.c_str(),
        static_cast<int>(payload.size()), payload.c_str(),
        1, false);
    return rc == MOSQ_ERR_SUCCESS;
}

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) {
        return 1;
    }

    mosquitto_lib_init();
    struct mosquitto* mosq = mosquitto_new("rc_mqtt_test_pub_cpp", true, nullptr);
    if (!mosq) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        mosquitto_lib_cleanup();
        return 2;
    }

    if (mosquitto_connect(mosq, opt.broker.c_str(), opt.port, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_connect failed: " << opt.broker << ":" << opt.port << "\n";
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        return 3;
    }

    double x = opt.x0;
    double y = opt.y0;
    double yaw = opt.yaw0;
    double goal_x = 0.0;
    double goal_y = 0.0;
    directionToGoal(opt.dir, opt.dist, goal_x, goal_y);

    {
        char goalBuf[192];
        std::snprintf(goalBuf, sizeof(goalBuf),
                      "{\"x\":%.3f,\"y\":%.3f,\"frame\":\"world\",\"ts_ms\":%lld}",
                      goal_x, goal_y, nowMs());
        mqttPublish(mosq, opt.topic_goal, goalBuf);
    }
    mqttPublish(mosq, opt.topic_safety, "{\"estop\":false,\"obstacle_stop\":false,\"planner_fail\":false}");

    std::cout << "Goal dir=" << opt.dir << " -> (" << goal_x << ", " << goal_y
              << "), mode=" << opt.mode << "\n";

    while (true) {
        const double dx = goal_x - x;
        const double dy = goal_y - y;
        const double dist = std::sqrt(dx * dx + dy * dy);

        if (opt.mode == "move" && dist > 0.15) {
            const double step = 0.05;
            x += step * (dx / dist);
            y += step * (dy / dist);
            yaw = std::atan2(dy, dx);
        }

        char poseBuf[256];
        std::snprintf(poseBuf, sizeof(poseBuf),
                      "{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.6f,\"frame\":\"world\",\"ts_ms\":%lld}",
                      x, y, yaw, nowMs());
        mqttPublish(mosq, opt.topic_pose, poseBuf);
        std::cout << "POSE -> x=" << x << " y=" << y << " yaw=" << yaw << " dist=" << dist << "\n";

        mosquitto_loop(mosq, 0, 1);

        if (opt.mode == "move" && dist <= 0.15) {
            std::cout << "Goal REACHED\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz
    }

    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    return 0;
}
