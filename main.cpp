#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <mosquitto.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>

static volatile bool g_running = true;

static void on_sigint(int) { g_running = false; }

static int make_listen_socket(int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return -1;

    int yes = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
        close(s);
        return -1;
    }
    if (listen(s, 5) < 0) {
        close(s);
        return -1;
    }
    return s;
}

static bool mqtt_init_and_connect(mosquitto** out,
                                 const std::string& host,
                                 int port,
                                 const std::string& client_id) {
    mosquitto_lib_init();

    mosquitto* m = mosquitto_new(client_id.c_str(), true, nullptr);
    if (!m) return false;

    // 필요하면 아래 주석 해제해서 사용자/비번 설정
    // mosquitto_username_pw_set(m, "user", "pass");

    // 자동 재연결 지연
    mosquitto_reconnect_delay_set(m, 1, 10, true);

    int rc = mosquitto_connect(m, host.c_str(), port, 30);
    if (rc != MOSQ_ERR_SUCCESS) {
        mosquitto_destroy(m);
        return false;
    }

    // 네트워크 루프는 별도 스레드로
    rc = mosquitto_loop_start(m);
    if (rc != MOSQ_ERR_SUCCESS) {
        mosquitto_disconnect(m);
        mosquitto_destroy(m);
        return false;
    }

    *out = m;
    return true;
}

static void mqtt_cleanup(mosquitto* m) {
    if (!m) return;
    mosquitto_loop_stop(m, true);
    mosquitto_disconnect(m);
    mosquitto_destroy(m);
    mosquitto_lib_cleanup();
}

int main(int argc, char** argv) {
    // ---- Config (필요 시 인자로도 받을 수 있게 단순화) ----
    const int TCP_PORT = 9000;
    const std::string MQTT_HOST = "127.0.0.1"; // 브로커가 로컬이면 그대로
    const int MQTT_PORT = 1883;
    const std::string MQTT_TOPIC = "wiserisk/cam/debug";
    const std::string LOG_PATH = "/var/log/wiserisk_cam_debug.log";

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    mosquitto* mosq = nullptr;
    if (!mqtt_init_and_connect(&mosq, MQTT_HOST, MQTT_PORT, "wiserisk-bridge")) {
        std::cerr << "[bridge] MQTT connect failed\n";
        return 1;
    }
    std::cout << "[bridge] MQTT connected to " << MQTT_HOST << ":" << MQTT_PORT
              << " topic=" << MQTT_TOPIC << "\n";

    int listen_fd = make_listen_socket(TCP_PORT);
    if (listen_fd < 0) {
        std::cerr << "[bridge] TCP listen failed on port " << TCP_PORT
                  << " errno=" << errno << " " << strerror(errno) << "\n";
        mqtt_cleanup(mosq);
        return 1;
    }
    std::cout << "[bridge] TCP listening on 0.0.0.0:" << TCP_PORT << "\n";

    std::ofstream logf(LOG_PATH, std::ios::app);
    if (!logf.is_open()) {
        std::cerr << "[bridge] cannot open log file: " << LOG_PATH << "\n";
        // 로그 파일은 선택사항이라 계속 진행
    }

    while (g_running) {
        sockaddr_in cli{};
        socklen_t clilen = sizeof(cli);

        int cfd = accept(listen_fd, (sockaddr*)&cli, &clilen);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[bridge] accept error errno=" << errno << " " << strerror(errno) << "\n";
            break;
        }

        char ip[64]{0};
        inet_ntop(AF_INET, &cli.sin_addr, ip, sizeof(ip));
        std::cout << "[bridge] client connected: " << ip << ":" << ntohs(cli.sin_port) << "\n";

        std::string buf;
        buf.reserve(8192);

        while (g_running) {
            char tmp[4096];
            ssize_t n = recv(cfd, tmp, sizeof(tmp), 0);
            if (n == 0) break;         // disconnected
            if (n < 0) {
                if (errno == EINTR) continue;
                std::cerr << "[bridge] recv error errno=" << errno << " " << strerror(errno) << "\n";
                break;
            }

            buf.append(tmp, tmp + n);

            // line-based framing
            size_t pos;
            while ((pos = buf.find('\n')) != std::string::npos) {
                std::string line = buf.substr(0, pos);
                buf.erase(0, pos + 1);

                // trim CR
                if (!line.empty() && line.back() == '\r') line.pop_back();
                if (line.empty()) continue;

                // 로그 파일 저장
                if (logf.is_open()) {
                    logf << line << "\n";
                    logf.flush();
                }

                // MQTT publish
                int rc = mosquitto_publish(
                    mosq,
                    nullptr,
                    MQTT_TOPIC.c_str(),
                    (int)line.size(),
                    line.data(),
                    0,      // qos
                    false   // retain
                );
                if (rc != MOSQ_ERR_SUCCESS) {
                    // 연결 끊겨도 loop thread가 재연결 시도함
                    std::cerr << "[bridge] mqtt publish failed rc=" << rc << "\n";
                }
            }
        }

        close(cfd);
        std::cout << "[bridge] client disconnected\n";
    }

    close(listen_fd);
    mqtt_cleanup(mosq);
    std::cout << "[bridge] exit\n";
    return 0;
}
