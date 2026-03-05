#include "servoserial_cpp.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

int main() {
    constexpr uint8_t PAN = 1;
    constexpr uint8_t TILT = 2;

    ServoSerialCpp servo;
    if (!servo.isOpen()) {
        std::cerr << "Serial device open failed.\n";
        return 1;
    }

    if (!servo.servoSerialDoubleControl(PAN, 1800, TILT, 2400)) {
        std::cerr << "Failed to send initial pan/tilt command.\n";
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    // Small sweep demo
    const int panTargets[] = {1200, 1800, 2400, 1800};
    const int tiltTargets[] = {1700, 2400, 3000, 2400};

    for (int i = 0; i < 4; ++i) {
        if (!servo.servoSerialDoubleControl(PAN, panTargets[i], TILT, tiltTargets[i])) {
            std::cerr << "Failed to send pan/tilt command at step " << i << ".\n";
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
    }

    return 0;
}
