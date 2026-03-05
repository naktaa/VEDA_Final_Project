#include "servoserial_cpp.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

namespace {
speed_t toBaudConstant(int baud) {
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    default:
        return B115200;
    }
}
} // namespace

ServoSerialCpp::ServoSerialCpp(const std::string& device, int baud) : fd_(-1) {
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        std::cerr << "Failed to open serial device " << device << ": " << std::strerror(errno) << '\n';
        return;
    }

    struct termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "tcgetattr failed: " << std::strerror(errno) << '\n';
        close(fd_);
        fd_ = -1;
        return;
    }

    cfmakeraw(&tty);
    const speed_t speed = toBaudConstant(baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // 100 ms read timeout

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr failed: " << std::strerror(errno) << '\n';
        close(fd_);
        fd_ = -1;
        return;
    }
}

ServoSerialCpp::~ServoSerialCpp() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

bool ServoSerialCpp::isOpen() const {
    return fd_ >= 0;
}

int ServoSerialCpp::clampAngle(uint8_t index, int angle) {
    if (index == 1) {
        if (angle < 600) {
            return 600;
        }
        if (angle > 3600) {
            return 3600;
        }
        return angle;
    }

    if (index == 2) {
        if (angle < 1300) {
            return 1300;
        }
        if (angle > 4095) {
            return 4095;
        }
        return angle;
    }

    if (angle < 0) {
        return 0;
    }
    if (angle > 4095) {
        return 4095;
    }
    return angle;
}

bool ServoSerialCpp::writePacket(const uint8_t* data, std::size_t len) const {
    if (!isOpen()) {
        return false;
    }
    const ssize_t written = write(fd_, data, len);
    if (written < 0 || static_cast<std::size_t>(written) != len) {
        std::cerr << "Serial write failed: " << std::strerror(errno) << '\n';
        return false;
    }
    return true;
}

bool ServoSerialCpp::servoSerialControl(uint8_t index, int angle) {
    const uint8_t pack1 = 0xFF;
    const uint8_t pack2 = 0xFF;
    const uint8_t len = 0x07;
    const uint8_t cmd = 0x03;
    const uint8_t addr = 0x2A;

    const int safeAngle = clampAngle(index, angle);
    const uint8_t posH = static_cast<uint8_t>((safeAngle >> 8) & 0xFF);
    const uint8_t posL = static_cast<uint8_t>(safeAngle & 0xFF);

    const uint8_t timeH = 0x00;
    const uint8_t timeL = 0x0A;

    const uint8_t checksum = static_cast<uint8_t>(~(index + len + cmd + addr + posH + posL + timeH + timeL));
    const uint8_t packet[] = {
        pack1, pack2, index, len, cmd, addr, posH, posL, timeH, timeL, checksum
    };
    return writePacket(packet, sizeof(packet));
}

bool ServoSerialCpp::servoSerialDoubleControl(uint8_t index1, int angle1, uint8_t index2, int angle2) {
    const uint8_t pack1 = 0xFF;
    const uint8_t pack2 = 0xFF;
    const uint8_t id = 0xFE;
    const uint8_t len = 0x0E;
    const uint8_t cmd = 0x83;
    const uint8_t addr1 = 0x2A;
    const uint8_t addr2 = 0x04;

    const int safeAngle1 = clampAngle(index1, angle1);
    const int safeAngle2 = clampAngle(index2, angle2);

    const uint8_t pos1H = static_cast<uint8_t>((safeAngle1 >> 8) & 0xFF);
    const uint8_t pos1L = static_cast<uint8_t>(safeAngle1 & 0xFF);
    const uint8_t pos2H = static_cast<uint8_t>((safeAngle2 >> 8) & 0xFF);
    const uint8_t pos2L = static_cast<uint8_t>(safeAngle2 & 0xFF);

    const uint8_t time1H = 0x00;
    const uint8_t time1L = 0x0A;
    const uint8_t time2H = 0x00;
    const uint8_t time2L = 0x0A;

    const uint8_t checksum = static_cast<uint8_t>(
        ~(id + len + cmd + addr1 + addr2 +
          index1 + pos1H + pos1L + time1H + time1L +
          index2 + pos2H + pos2L + time2H + time2L));

    const uint8_t packet[] = {
        pack1, pack2, id, len, cmd, addr1, addr2,
        index1, pos1H, pos1L, time1H, time1L,
        index2, pos2H, pos2L, time2H, time2L,
        checksum
    };
    return writePacket(packet, sizeof(packet));
}
