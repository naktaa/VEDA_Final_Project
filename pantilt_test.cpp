#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>

class ServoSerial {
public:
    ServoSerial(const std::string& device = "/dev/serial0", int baud = B115200) {
        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            perror("open serial");
            throw std::runtime_error("Failed to open serial port");
        }

        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            perror("tcgetattr");
            close(fd_);
            throw std::runtime_error("Failed to get serial attributes");
        }

        cfsetospeed(&tty, baud);
        cfsetispeed(&tty, baud);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            perror("tcsetattr");
            close(fd_);
            throw std::runtime_error("Failed to set serial attributes");
        }

        std::cout << "serial open: " << device << std::endl;
    }

    ~ServoSerial() {
        if (fd_ >= 0) {
            close(fd_);
            std::cout << "\nserial close" << std::endl;
        }
    }

    static int clampPan(int v) {
        if (v < 600) return 600;
        if (v > 3600) return 3600;
        return v;
    }

    static int clampTilt(int v) {
        if (v < 1300) return 1300;
        if (v > 4095) return 4095;
        return v;
    }

    bool controlSingle(uint8_t index, int angle) {
        uint8_t pack1 = 0xFF;
        uint8_t pack2 = 0xFF;
        uint8_t id = index;
        uint8_t len = 0x07;
        uint8_t cmd = 0x03;
        uint8_t addr = 0x2A;

        if (index == 1)
            angle = clampPan(angle);
        else if (index == 2)
            angle = clampTilt(angle);

        uint8_t pos_H = (angle >> 8) & 0xFF;
        uint8_t pos_L = angle & 0xFF;

        uint8_t time_H = 0x00;
        uint8_t time_L = 0x0A;

        uint8_t checksum =
            (~(id + len + cmd + addr + pos_H + pos_L + time_H + time_L)) & 0xFF;

        std::vector<uint8_t> data = {
            pack1, pack2, id, len, cmd, addr,
            pos_H, pos_L, time_H, time_L, checksum};

        return writePacket(data);
    }

    bool controlDouble(uint8_t index1, int angle1, uint8_t index2, int angle2) {
        uint8_t pack1 = 0xFF;
        uint8_t pack2 = 0xFF;
        uint8_t id = 0xFE;
        uint8_t len = 0x0E;
        uint8_t cmd = 0x83;
        uint8_t addr1 = 0x2A;
        uint8_t addr2 = 0x04;

        angle1 = clampPan(angle1);
        angle2 = clampTilt(angle2);

        uint8_t pos1_H = (angle1 >> 8) & 0xFF;
        uint8_t pos1_L = angle1 & 0xFF;
        uint8_t pos2_H = (angle2 >> 8) & 0xFF;
        uint8_t pos2_L = angle2 & 0xFF;

        uint8_t time1_H = 0x00;
        uint8_t time1_L = 0x0A;
        uint8_t time2_H = 0x00;
        uint8_t time2_L = 0x0A;

        uint8_t checksum =
            (~(id + len + cmd + addr1 + addr2 +
               index1 + pos1_H + pos1_L + time1_H + time1_L +
               index2 + pos2_H + pos2_L + time2_H + time2_L)) &
            0xFF;

        std::vector<uint8_t> data = {
            pack1, pack2, id, len, cmd, addr1, addr2,
            index1, pos1_H, pos1_L, time1_H, time1_L,
            index2, pos2_H, pos2_L, time2_H, time2_L,
            checksum};

        return writePacket(data);
    }

private:
    int fd_ = -1;

    bool writePacket(const std::vector<uint8_t>& data) {
        ssize_t written = write(fd_, data.data(), data.size());
        tcdrain(fd_);
        return written == static_cast<ssize_t>(data.size());
    }
};

class TerminalRawMode {
public:
    TerminalRawMode() {
        if (tcgetattr(STDIN_FILENO, &old_) == -1) {
            perror("tcgetattr");
            throw std::runtime_error("Failed to get terminal attrs");
        }
        raw_ = old_;
        raw_.c_lflag &= ~(ICANON | ECHO);
        raw_.c_cc[VMIN] = 0;
        raw_.c_cc[VTIME] = 0;

        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw_) == -1) {
            perror("tcsetattr");
            throw std::runtime_error("Failed to set raw terminal mode");
        }
    }

    ~TerminalRawMode() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    }

private:
    struct termios old_{};
    struct termios raw_{};
};

char getKeyNonBlocking() {
    fd_set set;
    struct timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; // 100ms

    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);

    int rv = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &timeout);
    if (rv > 0 && FD_ISSET(STDIN_FILENO, &set)) {
        char ch = 0;
        if (read(STDIN_FILENO, &ch, 1) == 1) return ch;
    }
    return 0;
}

int main() {
    try {
        ServoSerial servo("/dev/serial0", B115200);
        TerminalRawMode rawMode;

        constexpr uint8_t PAN_ID = 1;
        constexpr uint8_t TILT_ID = 2;

        // 시작 중심값
        int pan = 1800;
        int tilt = 2400;

        // 처음엔 안전하게 좁은 범위로
        const int PAN_MIN = 1000;
        const int PAN_MAX = 3200;
        const int TILT_MIN = 1600;
        const int TILT_MAX = 3400;

        const int STEP = 50;

        auto clamp = [](int v, int lo, int hi) {
            if (v < lo) return lo;
            if (v > hi) return hi;
            return v;
        };

        servo.controlDouble(PAN_ID, pan, TILT_ID, tilt);

        std::cout << "=== PAN/TILT WASD TEST ===\n";
        std::cout << "W/S : tilt up/down\n";
        std::cout << "A/D : pan left/right\n";
        std::cout << "SPACE: center\n";
        std::cout << "P: print current value\n";
        std::cout << "Q: quit\n\n";
        std::cout << "Current -> PAN=" << pan << " TILT=" << tilt << std::endl;

        while (true) {
            char key = getKeyNonBlocking();
            bool moved = false;

            // 방향 반전 적용
            if (key == 'a' || key == 'A') {
                pan = clamp(pan + STEP, PAN_MIN, PAN_MAX);
                moved = true;
            }
            else if (key == 'd' || key == 'D') {
                pan = clamp(pan - STEP, PAN_MIN, PAN_MAX);
                moved = true;
            }
            else if (key == 'w' || key == 'W') {
                tilt = clamp(tilt + STEP, TILT_MIN, TILT_MAX);
                moved = true;
            }
            else if (key == 's' || key == 'S') {
                tilt = clamp(tilt - STEP, TILT_MIN, TILT_MAX);
                moved = true;
            }
            else if (key == ' ') {
                pan = 1800;
                tilt = 2400;
                moved = true;
            }
            else if (key == 'p' || key == 'P') {
                std::cout << "\nCurrent -> PAN=" << pan
                          << " TILT=" << tilt << std::endl;
            }
            else if (key == 'q' || key == 'Q') {
                std::cout << "\nQuit\n";
                break;
            }

            if (moved) {
                servo.controlDouble(PAN_ID, pan, TILT_ID, tilt);
                std::cout << "\rPAN=" << pan << "  TILT=" << tilt << "        " << std::flush;
                usleep(30000);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
