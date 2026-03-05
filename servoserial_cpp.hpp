#ifndef SERVOSERIAL_CPP_HPP
#define SERVOSERIAL_CPP_HPP

#include <cstdint>
#include <string>

class ServoSerialCpp {
public:
    explicit ServoSerialCpp(const std::string& device = "/dev/serial0", int baud = 115200);
    ~ServoSerialCpp();

    ServoSerialCpp(const ServoSerialCpp&) = delete;
    ServoSerialCpp& operator=(const ServoSerialCpp&) = delete;

    bool isOpen() const;
    bool servoSerialControl(uint8_t index, int angle);
    bool servoSerialDoubleControl(uint8_t index1, int angle1, uint8_t index2, int angle2);

private:
    int fd_;

    static int clampAngle(uint8_t index, int angle);
    bool writePacket(const uint8_t* data, std::size_t len) const;
};

#endif
