#include "auto_app.hpp"

#include <atomic>
#include <csignal>
#include <exception>
#include <iostream>

#include "rc_config.hpp"
#include "rc_control_node.hpp"

namespace {

std::atomic<bool> g_run{true};

void OnSignal(int) {
    g_run = false;
}

} // namespace

void PrintAutoUsage(const char* exe) {
    std::cout
        << "사용법: " << exe
        << " [mqtt_host] [mqtt_port] [goal_topic] [pose_topic] [safety_topic] [status_topic] [ini_path]\n";
}

bool ParseAutoConfig(int argc, char** argv, RcAppConfig& config, std::string& error) {
    try {
        if (argc > 1) config.host = argv[1];
        if (argc > 2) config.port = std::stoi(argv[2]);
        if (argc > 3) config.topics.goal = argv[3];
        if (argc > 4) config.topics.pose = argv[4];
        if (argc > 5) config.topics.safety = argv[5];
        if (argc > 6) config.topics.status = argv[6];
        if (argc > 7) config.ini_path = argv[7];
    } catch (const std::exception& ex) {
        error = std::string("인자 파싱 실패: ") + ex.what();
        return false;
    }

    if (config.port <= 0) {
        error = "mqtt_port는 1 이상이어야 합니다.";
        return false;
    }
    return true;
}

int RunAutoApp(const RcAppConfig& config) {
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    RcAppConfig runtime_config = config;
    LoadRcParamsFromIni(runtime_config.ini_path, runtime_config);

    RcControlNode node(runtime_config);
    if (!node.start()) {
        return 1;
    }

    node.run(&g_run);
    node.stop();
    return 0;
}
