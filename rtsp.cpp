#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
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
#include <thread>



static GstRTSPMediaFactory*
make_udp_to_rtsp_factory(int port)
{
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();

    // [수정] caps 설정을 넣어줘야 서버가 UDP 패킷의 정체를 파악합니다.
    gchar *pipeline = g_strdup_printf(
        "( udpsrc port=%d caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! "
        "rtph264depay ! h264parse ! "
        "rtph264pay name=pay0 pt=96 config-interval=1 )",
        port
    );

    gst_rtsp_media_factory_set_launch(factory, pipeline);
    gst_rtsp_media_factory_set_shared(factory, TRUE); // 여러 명이 동시에 접속 가능하게 변경

    g_free(pipeline);
    return factory;
}


#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

static GstRTSPMediaFactory*
make_relay_factory(const gchar *uri)
{
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();

    gchar *pipeline = g_strdup_printf(
    "( rtspsrc location=%s protocols=tcp latency=1500 "
    "  do-rtsp-keep-alive=true ! "
    "  queue ! "
    "  rtph264depay ! h264parse ! tee name=t "
    "  t. ! queue ! fakesink sync=false "
    "  t. ! queue ! rtph264pay name=pay0 pt=96 )",
    uri
    );

    gst_rtsp_media_factory_set_launch(factory, pipeline);
    gst_rtsp_media_factory_set_shared(factory, FALSE);
    
    // 끊김 방지 설정
    //gst_rtsp_media_factory_set_suspend_mode(factory, GST_RTSP_SUSPEND_MODE_NONE);

    g_free(pipeline);
    return factory;
}

// 
int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);

    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    GstRTSPServer *server = gst_rtsp_server_new();

    gst_rtsp_server_set_service(server, "8554");

    GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);


    const gchar *uri = "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";

    gchar *path = g_strdup_printf("/cam%d", 0);
    gst_rtsp_mount_points_add_factory(mounts, path, make_relay_factory(uri));
    g_print("camera %d ready: /cam%d\n", 0, 0);
    g_free(path);
    

    g_object_unref(mounts);
    gst_rtsp_server_attach(server, NULL);

    g_print("\n===============================\n");
    g_print("RTSP server running\n");
    g_print("camera: rtsp://<serverIP>:8554/cam0\n");
    g_print("===============================\n");

    g_main_loop_run(loop);
    return 0;
}

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

/* ==========================
   전역 팩토리 (안전 관리)
   ========================== */
static GstRTSPMediaFactory *cam_factories[4] = {NULL,};

/* ==========================
   릴레이 팩토리 생성
   ========================== */
static GstRTSPMediaFactory*
make_relay_factory(const gchar *uri, int id)
{
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();

gchar *pipeline = g_strdup_printf(
"( rtspsrc location=%s protocols=tcp latency=1500 "
"  drop-on-latency=true "
"  do-rtsp-keep-alive=true "
"  user-agent=\"Relay-Cam-%d\" "
"  tcp-timeout=10000000 timeout=10000000 retry=1000 ! "
"  queue max-size-buffers=200 max-size-bytes=0 max-size-time=0 ! "
"  rtph264depay ! h264parse ! queue ! "
"  rtph264pay name=pay0 pt=96 config-interval=1 )",
uri, id
);


    gst_rtsp_media_factory_set_launch(factory, pipeline);
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    gst_rtsp_media_factory_set_suspend_mode(factory, GST_RTSP_SUSPEND_MODE_NONE);
    gst_rtsp_media_factory_set_buffer_size(factory, 2 * 1024 * 1024); //2MB 버퍼

    g_print("🛠️ Pipeline Configured (Cam %d): %s\n", id, pipeline);

    g_free(pipeline);
    return factory;
}

/* =========================================
   안전한 Warmup (외부 클라이언트 방식)
   ========================================= */
static gboolean do_warmup(gpointer data)
{
    static int cam_idx = 0;
    if (cam_idx >= 4) return FALSE;

    gchar *cmd = g_strdup_printf(
        "sleep 1 && "
        "gst-launch-1.0 rtspsrc location=rtsp://127.0.0.1:8554/cam%d "
        "latency=1500 ! fakesink sync=false &",
        cam_idx
    );

    g_print("🔥 [Cam%d] Warmup by fake client...\n", cam_idx);
    system(cmd);

    g_free(cmd);
    cam_idx++;

    return TRUE;   // 계속 10초 간격 실행
}

/* ==========================
   MAIN
   ========================== */
int main(int argc, char *argv[])
{
    // 기존 점유 프로세스 정리
    system("fuser -k 8554/tcp 2>/dev/null");

    gst_init(&argc, &argv);

    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    GstRTSPServer *server = gst_rtsp_server_new();

    /* ---- 스레드 풀 설정 ---- */
    GstRTSPThreadPool *pool = gst_rtsp_thread_pool_new();
    gst_rtsp_thread_pool_set_max_threads(pool, 16);
    gst_rtsp_server_set_thread_pool(server, pool);
    g_object_unref(pool);

    gst_rtsp_server_set_service(server, "8554");

    GstRTSPMountPoints *mounts =
        gst_rtsp_server_get_mount_points(server);

    const gchar *uris[2] = {
        "rtsp://admin:team3@@@@192.168.100.16/profile2/media.smp",
        "rtsp://192.168.100.8:8555/cam"
    };

    /* ---- 팩토리 등록 + 전역 저장 ---- */
    for (int i = 0; i < 3; i++) {
        gchar *path = g_strdup_printf("/cam%d", i);

        cam_factories[i] = make_relay_factory(uris[i], i);
        gst_rtsp_mount_points_add_factory(
            mounts, path, cam_factories[i]);

        g_print("camera %d ready complete: %s\n", i, path);
        g_free(path);
    }

    g_object_unref(mounts);
    gst_rtsp_server_attach(server, NULL);

    /* ---- 5초 후부터 10초 간격 Warmup ---- */
    g_timeout_add(12000, do_warmup, NULL);

    g_print("\n🚀 Server Running. Warmup will start in 5s...\n");
    g_print("접속 예시:\n");
    g_print("  rtsp://<서버IP>:8554/cam0 ~ cam3\n\n");

    g_main_loop_run(loop);
    return 0;
}


// #include <gst/gst.h>
// #include <gst/rtsp-server/rtsp-server.h>
// #include <arpa/inet.h>
// #include <errno.h>
// #include <fcntl.h>
// #include <mosquitto.h>
// #include <netinet/in.h>
// #include <signal.h>
// #include <stdio.h>
// #include <string.h>
// #include <sys/socket.h>
// #include <sys/types.h>
// #include <unistd.h>

// #include <fstream>
// #include <iostream>
// #include <string>
// #include <thread>

// // 전역 변수 및 시그널 제어
// static volatile bool g_running = true;
// static GMainLoop *g_gst_loop = nullptr;

// static void on_sigint(int) {
//     g_running = false;
//     if (g_gst_loop) {
//         g_main_loop_quit(g_gst_loop);
//     }
// }

// // --- RTSP Server Functions ---
// static GstRTSPMediaFactory* make_relay_factory(const gchar *uri) {
//     GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();
//     gchar *pipeline = g_strdup_printf(
//         "( rtspsrc location=%s protocols=tcp latency=1500 "
//         "  do-rtsp-keep-alive=true ! "
//         "  queue ! "
//         "  rtph264depay ! h264parse ! tee name=t "
//         "  t. ! queue ! fakesink sync=false "
//         "  t. ! queue ! rtph264pay name=pay0 pt=96 )",
//         uri
//     );

//     gst_rtsp_media_factory_set_launch(factory, pipeline);
//     gst_rtsp_media_factory_set_shared(factory, FALSE);
//     g_free(pipeline);
//     return factory;
// }

// // --- TCP & MQTT Bridge Functions ---
// static int make_listen_socket(int port) {
//     int s = socket(AF_INET, SOCK_STREAM, 0);
//     if (s < 0) return -1;
//     int yes = 1;
//     setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
//     sockaddr_in addr{};
//     addr.sin_family = AF_INET;
//     addr.sin_port = htons(port);
//     addr.sin_addr.s_addr = INADDR_ANY;
//     if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) { close(s); return -1; }
//     if (listen(s, 5) < 0) { close(s); return -1; }
//     return s;
// }

// static bool mqtt_init_and_connect(mosquitto** out, const std::string& host, int port, const std::string& client_id) {
//     mosquitto_lib_init();
//     mosquitto* m = mosquitto_new(client_id.c_str(), true, nullptr);
//     if (!m) return false;
//     mosquitto_reconnect_delay_set(m, 1, 10, true);
//     if (mosquitto_connect(m, host.c_str(), port, 30) != MOSQ_ERR_SUCCESS) {
//         mosquitto_destroy(m);
//         return false;
//     }
//     if (mosquitto_loop_start(m) != MOSQ_ERR_SUCCESS) {
//         mosquitto_disconnect(m);
//         mosquitto_destroy(m);
//         return false;
//     }
//     *out = m;
//     return true;
// }

// // 별도 스레드에서 실행될 브릿지 함수
// void bridge_thread_func() {
//     const int TCP_PORT = 9000;
//     const std::string MQTT_HOST = "127.0.0.1";
//     const int MQTT_PORT = 1883;
//     const std::string MQTT_TOPIC = "wiserisk/cam/debug";
//     const std::string LOG_PATH = "/var/log/wiserisk_cam_debug.log";

//     mosquitto* mosq = nullptr;
//     if (!mqtt_init_and_connect(&mosq, MQTT_HOST, MQTT_PORT, "wiserisk-bridge")) {
//         std::cerr << "[bridge] MQTT connect failed\n";
//         return;
//     }

//     int listen_fd = make_listen_socket(TCP_PORT);
//     if (listen_fd < 0) {
//         std::cerr << "[bridge] TCP listen failed\n";
//         return;
//     }

//     std::ofstream logf(LOG_PATH, std::ios::app);
//     std::cout << "[bridge] TCP listening on 9000, MQTT connected.\n";

//     while (g_running) {
//         sockaddr_in cli{};
//         socklen_t clilen = sizeof(cli);
        
//         // Non-blocking 처리를 하지 않았으므로 accept에서 대기함
//         // 시그널 발생 시 종료를 위해 timeout 처리를 추가하거나 간단하게 설계
//         int cfd = accept(listen_fd, (sockaddr*)&cli, &clilen);
//         if (cfd < 0) {
//             if (errno == EINTR) continue;
//             break;
//         }

//         std::string buf;
//         while (g_running) {
//             char tmp[4096];
//             ssize_t n = recv(cfd, tmp, sizeof(tmp), 0);
//             if (n <= 0) break; 

//             buf.append(tmp, n);
//             size_t pos;
//             while ((pos = buf.find('\n')) != std::string::npos) {
//                 std::string line = buf.substr(0, pos);
//                 buf.erase(0, pos + 1);
//                 if (!line.empty() && line.back() == '\r') line.pop_back();
//                 if (line.empty()) continue;

//                 if (logf.is_open()) { logf << line << "\n"; logf.flush(); }
//                 mosquitto_publish(mosq, nullptr, MQTT_TOPIC.c_str(), (int)line.size(), line.data(), 0, false);
//             }
//         }
//         close(cfd);
//     }

//     close(listen_fd);
//     if (mosq) {
//         mosquitto_loop_stop(mosq, true);
//         mosquitto_disconnect(mosq);
//         mosquitto_destroy(mosq);
//     }
//     mosquitto_lib_cleanup();
//     std::cout << "[bridge] thread exit\n";
// }

// int main(int argc, char *argv[]) {
//     signal(SIGINT, on_sigint);
//     signal(SIGTERM, on_sigint);

//     // 1. GStreamer 초기화
//     gst_init(&argc, &argv);
//     g_gst_loop = g_main_loop_new(NULL, FALSE);

//     // 2. TCP-MQTT 브릿지 스레드 시작
//     std::thread bridge_thread(bridge_thread_func);

//     // 3. RTSP 서버 설정
//     GstRTSPServer *server = gst_rtsp_server_new();
//     gst_rtsp_server_set_service(server, "8554");
//     GstRTSPMountPoints *mounts = gst_rtsp_server_get_mount_points(server);

//     const gchar *uri = "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
//     gst_rtsp_mount_points_add_factory(mounts, "/cam0", make_relay_factory(uri));
//     g_object_unref(mounts);

//     if (gst_rtsp_server_attach(server, NULL) == 0) {
//         std::cerr << "Failed to attach RTSP server\n";
//         return -1;
//     }

//     g_print("\n===============================\n");
//     g_print("Integrated Server Running\n");
//     g_print("RTSP: rtsp://<IP>:8554/cam0\n");
//     g_print("TCP: Port 9000 -> MQTT\n");
//     g_print("===============================\n");

//     // 4. 메인 루프 실행 (블로킹)
//     g_main_loop_run(g_gst_loop);

//     // 5. 종료 처리
//     g_running = false;
//     if (bridge_thread.joinable()) {
//         bridge_thread.join();
//     }
    
//     g_main_loop_unref(g_gst_loop);
//     g_object_unref(server);

//     return 0;
// }