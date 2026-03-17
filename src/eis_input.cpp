#include "eis_input.hpp"

#include <cstdio>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "eis_globals.hpp"
#include "tank_drive.hpp"

void keyboard_loop() {
    if (!isatty(STDIN_FILENO)) {
        fprintf(stderr, "[KEY] stdin is not a TTY. Keyboard control disabled.\n");
        return;
    }

    termios oldt {};
    if (tcgetattr(STDIN_FILENO, &oldt) != 0) {
        perror("[KEY] tcgetattr");
        return;
    }
    termios newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) != 0) {
        perror("[KEY] tcsetattr");
        return;
    }

    tank_drive::init();

    fprintf(stderr, "[KEY] 1=LK 2=Gyro 3=Hybrid (output fixed to CAM)\n");
    fprintf(stderr, "[KEY] WASD/QE + Space/X for tank, +/- speed, H help\n");

    pollfd pfd;
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;

    while (g_running) {
        int r = poll(&pfd, 1, 50);
        if (r > 0 && (pfd.revents & POLLIN)) {
            char ch = 0;
            ssize_t n = read(STDIN_FILENO, &ch, 1);
            if (n == 1) {
                bool consumed = false;
                if (ch == '1') {
                    g_mode = (int)EisMode::LK;
                    fprintf(stderr, "[MODE] %s\n", mode_str(EisMode::LK));
                    consumed = true;
                } else if (ch == '2') {
                    g_mode = (int)EisMode::GYRO;
                    fprintf(stderr, "[MODE] %s\n", mode_str(EisMode::GYRO));
                    consumed = true;
                } else if (ch == '3') {
                    g_mode = (int)EisMode::HYBRID;
                    fprintf(stderr, "[MODE] %s\n", mode_str(EisMode::HYBRID));
                    consumed = true;
                }

                if (!consumed) {
                    tank_drive::handle_key(ch);
                }
            }
        }
        tank_drive::tick();
    }

    tank_drive::shutdown();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}
