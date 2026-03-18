#include "rc_status_publisher.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_run{true};

void onSignal(int) {
    g_run = false;
}
} // namespace

int main(int argc, char** argv) {
    RcStatusPublisher::Config config;
    config.broker_host = (argc > 1) ? argv[1] : "127.0.0.1";
    config.broker_port = (argc > 2) ? std::stoi(argv[2]) : 1883;
    config.topic = (argc > 3) ? argv[3] : "wiserisk/rc/status";
    config.publish_interval_ms = (argc > 4) ? std::stoi(argv[4]) : RcStatusPublisher::kDefaultPublishIntervalMs;

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    RcStatusPublisher publisher(config);

    // TODO(hw): 실제 하드웨어/주행 모듈이 연결되면 아래 provider 내부에서
    // battery, speed, x, y, heading을 실측값으로 채우도록 교체.
    // 현재는 placeholder 정책을 유지한다.
    publisher.setStatusProvider([](RcStatus& status) {
        (void)status;
    });

    if (!publisher.start()) {
        return 1;
    }

    std::cout << "[INFO] RC status publish loop started\n";
    while (g_run.load()) {
        // 기존 제어 루프에서 이 위치에 최신 상태값 반영 가능:
        // publisher.setMode("manual");
        // publisher.setMission("none");
        // publisher.setBattery(real_battery);
        // publisher.setSpeed(real_speed);
        // publisher.setPosition(real_x, real_y);
        // publisher.setHeading(real_heading);
        publisher.spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    publisher.stop();
    return 0;
}
