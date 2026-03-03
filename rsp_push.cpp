/**
 * rtsp_server.cpp
 *
 * 구조:
 *   Pi카메라(들) → RTSP push → [이 서버] → RTSP pull → QT클라이언트
 *
 * 기능:
 *   - 다중 카메라: 카메라가 push하면 자동으로 경로 등록
 *   - MQTT 수신: 카메라에서 보내는 PERSON_ON/OFF 이벤트 처리
 *   - QT클라이언트는 rtsp://서버IP:8554/live/{cam_id} 로 pull
 *
 * 의존성 설치 (라즈베리파이):
 *   sudo apt install -y \
 *     libgstreamer1.0-dev \
 *     libgstreamer-plugins-base1.0-dev \
 *     libgstrtspserver-1.0-dev \
 *     libpaho-mqttpp3-dev libpaho-mqtt3as1 \
 *     gstreamer1.0-plugins-good \
 *     gstreamer1.0-plugins-bad \
 *     gstreamer1.0-plugins-ugly \
 *     gstreamer1.0-libav
 *
 * 빌드:
 *   g++ rsp_push.cpp -o rsp_push \
 *     $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-rtsp-server-1.0) \
 *     -lpaho-mqttpp3 -lpaho-mqtt3as \
 *     -std=c++17 -O2
 *
 * 실행:
 *   ./rtsp_server
 *
 * 카메라 등록 방법:
 *   카메라 측에서 아래 경로로 RTSP push하면 자동 relay됩니다:
 *   rtsp://서버IP:8554/live/pi_cam_01
 *   rtsp://서버IP:8554/live/pi_cam_02
 *   ...
 *
 *   Pi카메라 코드의 rtspclientsink location 을 위 URL로 설정하세요.
 */

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

#include <mqtt/async_client.h>

#include <iostream>
#include <string>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>
#include <vector>
#include <sstream>

/* ============================================================
 * 설정 (필요에 따라 수정)
 * ============================================================ */
static const char* RTSP_PORT      = "8554";
static const char* MQTT_HOST      = "tcp://localhost:1883";   // MQTT 브로커 주소
static const char* MQTT_CLIENT_ID = "pi_server";
static const char* MQTT_SUB_TOPIC = "vr/person/#";           // 모든 카메라 이벤트 구독

/* ============================================================
 * MQTT 이벤트 콜백 타입
 * ============================================================ */
using MqttEventCb = std::function<void(const std::string& topic, const std::string& payload)>;

/* ============================================================
 * CameraRelay
 *
 * 카메라 1대당 1개 인스턴스.
 * rtspsrc로 카메라의 push를 받아서 내부 appsink에 저장,
 * gst-rtsp-server의 appsrc로 QT클라이언트에게 서빙합니다.
 *
 * 단순화 버전: gst-rtsp-server의 factory launch 문자열에
 * rtspsrc를 직접 넣어 서버가 카메라에 연결하는 방식.
 * (카메라가 rtspclientsink로 push → 서버가 rtspsrc로 pull하여 relay)
 *
 * 참고: 이 방식은 카메라가 먼저 push를 시작해야 서버가 연결할 수 있습니다.
 * ============================================================ */
class CameraRelay {
public:
    std::string cam_id;
    std::string cam_rtsp_url;  // 카메라가 push하는 URL (서버 입장에서 pull할 URL)
    // gst-rtsp-server에 등록할 경로
    std::string serve_path;    // ex: /live/pi_cam_01

    CameraRelay(const std::string& id, const std::string& cam_url)
        : cam_id(id), cam_rtsp_url(cam_url)
    {
        serve_path = "/live/" + id;
    }

    /**
     * factory launch 문자열 반환.
     * gst-rtsp-server가 QT클라이언트 접속 시 이 파이프라인을 실행합니다.
     *
     * 파이프라인:
     *   rtspsrc → 디코드 → 인코드 → rtph264pay → 클라이언트
     *
     * 주의: rtspsrc location은 서버가 카메라에 접속하는 URL이어야 합니다.
     *       카메라 push URL과 동일하게 설정하세요.
     */
    std::string makeLaunchStr() const {
        std::ostringstream ss;
        ss  << "( rtspsrc location=" << cam_rtsp_url
            << " latency=100 protocols=tcp"
            << " ! rtph264depay"
            << " ! h264parse"
            << " ! rtph264pay name=pay0 pt=96 config-interval=1 )";
        return ss.str();
    }
};

/* ============================================================
 * RTSPServer
 * ============================================================ */
class RTSPServer {
public:
    RTSPServer() : loop_(nullptr), server_(nullptr), mounts_(nullptr) {}

    ~RTSPServer() {
        if (loop_) g_main_loop_quit(loop_);
        if (loop_thread_.joinable()) loop_thread_.join();
        if (mounts_) g_object_unref(mounts_);
        if (server_) g_object_unref(server_);
        if (loop_) g_main_loop_unref(loop_);
    }

    bool init() {
        loop_   = g_main_loop_new(nullptr, FALSE);
        server_ = gst_rtsp_server_new();
        if (!server_) {
            std::cerr << "[Server] gst_rtsp_server_new() failed\n";
            return false;
        }

        gst_rtsp_server_set_service(server_, RTSP_PORT);
        mounts_ = gst_rtsp_server_get_mount_points(server_);

        // 서버를 GLib 메인루프에 붙이기
        if (gst_rtsp_server_attach(server_, nullptr) == 0) {
            std::cerr << "[Server] gst_rtsp_server_attach() failed\n";
            return false;
        }

        // GLib 메인루프 별도 스레드에서 실행
        loop_thread_ = std::thread([this]() {
            g_main_loop_run(loop_);
        });

        std::cout << "[Server] RTSP server started on port " << RTSP_PORT << "\n";
        return true;
    }

    /**
     * 카메라 경로 등록.
     * 이미 등록된 cam_id면 무시합니다.
     */
    bool addCamera(const CameraRelay& cam) {
        std::lock_guard<std::mutex> lk(mtx_);

        if (registered_.count(cam.cam_id)) {
            std::cout << "[Server] Camera already registered: " << cam.cam_id << "\n";
            return false;
        }

        GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
        std::string launch = cam.makeLaunchStr();

        gst_rtsp_media_factory_set_launch(factory, launch.c_str());
        gst_rtsp_media_factory_set_shared(factory, TRUE);  // 여러 클라이언트 공유

        gst_rtsp_mount_points_add_factory(mounts_, cam.serve_path.c_str(), factory);
        g_object_unref(factory);

        registered_[cam.cam_id] = cam;

        std::cout << "[Server] Camera registered: " << cam.cam_id
                  << " → serve path: " << cam.serve_path << "\n"
                  << "         launch: " << launch << "\n";
        return true;
    }

    /**
     * 카메라 경로 제거 (카메라 연결 끊겼을 때 등)
     */
    void removeCamera(const std::string& cam_id) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = registered_.find(cam_id);
        if (it == registered_.end()) return;

        gst_rtsp_mount_points_remove_factory(mounts_, it->second.serve_path.c_str());
        registered_.erase(it);
        std::cout << "[Server] Camera removed: " << cam_id << "\n";
    }

    bool hasCamera(const std::string& cam_id) {
        std::lock_guard<std::mutex> lk(mtx_);
        return registered_.count(cam_id) > 0;
    }

private:
    GMainLoop*          loop_;
    GstRTSPServer*      server_;
    GstRTSPMountPoints* mounts_;
    std::thread         loop_thread_;
    std::mutex          mtx_;
    std::unordered_map<std::string, CameraRelay> registered_;
};

/* ============================================================
 * MQTTManager
 * ============================================================ */
class MQTTManager : public virtual mqtt::callback {
public:
    explicit MQTTManager(const std::string& host, const std::string& client_id)
        : cli_(host, client_id)
    {
        cli_.set_callback(*this);
    }

    void setEventCallback(MqttEventCb cb) { event_cb_ = cb; }

    bool connect() {
        mqtt::connect_options opts;
        opts.set_clean_session(true);
        opts.set_keep_alive_interval(20);
        opts.set_automatic_reconnect(true);

        try {
            cli_.connect(opts)->wait();
            std::cout << "[MQTT] Connected: " << MQTT_HOST << "\n";
            cli_.subscribe(MQTT_SUB_TOPIC, 0)->wait();
            std::cout << "[MQTT] Subscribed: " << MQTT_SUB_TOPIC << "\n";
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[MQTT] Connect failed: " << e.what() << "\n";
            return false;
        }
    }

    // mqtt::callback 구현
    void message_arrived(mqtt::const_message_ptr msg) override {
        if (event_cb_) {
            event_cb_(msg->get_topic(), msg->to_string());
        }
    }

    void connection_lost(const std::string& cause) override {
        std::cerr << "[MQTT] Connection lost: " << cause << "\n";
    }

    void connected(const std::string&) override {
        std::cout << "[MQTT] Reconnected.\n";
        try { cli_.subscribe(MQTT_SUB_TOPIC, 0); } catch(...) {}
    }

    bool publish(const std::string& topic, const std::string& payload, int qos = 0) {
        try {
            auto msg = mqtt::make_message(topic, payload);
            msg->set_qos(qos);
            cli_.publish(msg)->wait_for(std::chrono::seconds(2));
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[MQTT] Publish failed: " << e.what() << "\n";
            return false;
        }
    }

private:
    mqtt::async_client cli_;
    MqttEventCb event_cb_;
};

/* ============================================================
 * CameraManager
 *
 * MQTT 이벤트를 파싱하여 카메라 등록/제거 및 이벤트 처리.
 * ============================================================ */
class CameraManager {
public:
    CameraManager(RTSPServer& srv, MQTTManager& mqtt)
        : srv_(srv), mqtt_(mqtt)
    {}

    /**
     * MQTT 메시지 처리.
     * topic 예: vr/person/pi_cam_01
     * payload 예: {"cam":"pi_cam_01","event":"PERSON_ON","count":1,"t_ms":12345}
     */
    void onMqttMessage(const std::string& topic, const std::string& payload) {
        std::cout << "[MQTT] topic=" << topic << " payload=" << payload << "\n";

        // topic에서 cam_id 추출: vr/person/{cam_id}
        std::string cam_id = extractCamId(topic);
        if (cam_id.empty()) return;

        // 카메라가 아직 등록 안 됐으면 자동 등록
        if (!srv_.hasCamera(cam_id)) {
            autoRegisterCamera(cam_id);
        }

        // 이벤트 처리
        handleEvent(cam_id, payload);
    }

    /**
     * 카메라를 수동으로 미리 등록할 때 사용.
     * 카메라 RTSP push URL을 알고 있을 때.
     */
    void registerCamera(const std::string& cam_id, const std::string& cam_push_url) {
        CameraRelay cam(cam_id, cam_push_url);
        srv_.addCamera(cam);
    }

private:
    RTSPServer&   srv_;
    MQTTManager&  mqtt_;

    // 카메라 상태 저장
    struct CamState {
        bool person_present = false;
        int  person_count   = 0;
    };
    std::unordered_map<std::string, CamState> states_;
    std::mutex mtx_;

    std::string extractCamId(const std::string& topic) {
        // topic = "vr/person/pi_cam_01"
        auto pos = topic.rfind('/');
        if (pos == std::string::npos || pos + 1 >= topic.size()) return "";
        return topic.substr(pos + 1);
    }

    /**
     * MQTT 메시지를 처음 받았을 때 카메라 자동 등록.
     * 카메라 push URL을 서버 자신의 IP 기준으로 추정합니다.
     *
     * 실제 환경에서는 payload에 카메라 IP를 포함시키거나
     * 설정 파일로 관리하는 것을 권장합니다.
     */
    void autoRegisterCamera(const std::string& cam_id) {
        // 기본 규칙: 카메라는 rtspclientsink로 서버에 push하므로
        // 서버(localhost)의 같은 포트로 들어옵니다.
        // 단, gst-rtsp-server는 push를 직접 받지 못하므로
        // 여기서는 카메라가 별도의 내부 RTSP 서버를 열고
        // 이 서버가 pull하는 구조를 가정합니다.
        //
        // *** 실제 사용 시 아래 URL을 카메라 실제 주소로 변경하세요 ***
        // 예: "rtsp://192.168.100.{N}:8555/cam"
        std::string cam_url = "rtsp://192.168.100.20:8555/" + cam_id;

        std::cout << "[CameraManager] Auto-registering camera: " << cam_id
                  << " url=" << cam_url << "\n";

        CameraRelay cam(cam_id, cam_url);
        srv_.addCamera(cam);
    }

    void handleEvent(const std::string& cam_id, const std::string& payload) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto& state = states_[cam_id];

        // 간단한 파싱 (JSON 파서 없이)
        if (payload.find("PERSON_ON") != std::string::npos) {
            state.person_present = true;
            // count 파싱
            auto pos = payload.find("\"count\":");
            if (pos != std::string::npos) {
                state.person_count = std::stoi(payload.substr(pos + 8));
            }
            std::cout << "[Event] " << cam_id << " PERSON_ON count=" << state.person_count << "\n";

            // 필요한 추가 처리를 여기에 구현하세요
            // 예: 알림 전송, 로그 저장, 다른 시스템 트리거 등
            onPersonDetected(cam_id, state.person_count);

        } else if (payload.find("PERSON_OFF") != std::string::npos) {
            state.person_present = false;
            state.person_count   = 0;
            std::cout << "[Event] " << cam_id << " PERSON_OFF\n";

            onPersonLeft(cam_id);
        }
    }

    // ---- 이벤트 핸들러 (필요한 기능 여기에 추가) ----

    void onPersonDetected(const std::string& cam_id, int count) {
        // TODO: 원하는 동작 구현
        // 예시: 다른 MQTT 토픽으로 알림 발행
        std::ostringstream payload;
        payload << "{\"cam\":\"" << cam_id << "\",\"alert\":\"person_detected\",\"count\":" << count << "}";
        mqtt_.publish("vr/alert/" + cam_id, payload.str());
    }

    void onPersonLeft(const std::string& cam_id) {
        // TODO: 원하는 동작 구현
        std::ostringstream payload;
        payload << "{\"cam\":\"" << cam_id << "\",\"alert\":\"person_left\"}";
        mqtt_.publish("vr/alert/" + cam_id, payload.str());
    }
};

/* ============================================================
 * main
 * ============================================================ */
int main(int argc, char* argv[]) {
    gst_init(&argc, &argv);

    std::cout << "=== RTSP Relay Server ===\n";

    // ---- RTSP 서버 초기화 ----
    RTSPServer srv;
    if (!srv.init()) return 1;

    // ---- 카메라 수동 등록 (미리 알고 있는 경우) ----
    // 카메라가 MQTT를 통해 자동 등록되길 원하면 이 블록 제거 가능.
    //
    // 카메라 측 구조 변경 필요:
    //   지금 Pi카메라 코드는 rtspclientsink로 서버에 push하는데,
    //   이 서버(gst-rtsp-server)는 push를 받지 못합니다.
    //   → 카메라 코드를 rtspsrc로 변경하거나,
    //     카메라에 작은 RTSP 서버를 열고 이 서버가 pull하도록 해야 합니다.
    //
    // 아래는 카메라가 자체 RTSP 서버를 열고 있다고 가정한 예시:
    {
        // cam_id, 카메라의 RTSP URL (이 서버가 pull할 주소)
        srv.addCamera(CameraRelay("pi_cam_01", "rtsp://192.168.100.20:8555/cam"));
        // 카메라 추가 시:
        // srv.addCamera(CameraRelay("pi_cam_02", "rtsp://192.168.100.21:8555/cam"));
    }

    // ---- MQTT 초기화 ----
    MQTTManager mqtt(MQTT_HOST, MQTT_CLIENT_ID);
    CameraManager cam_mgr(srv, mqtt);

    mqtt.setEventCallback([&cam_mgr](const std::string& topic, const std::string& payload) {
        cam_mgr.onMqttMessage(topic, payload);
    });

    // MQTT 연결 (실패해도 RTSP는 동작)
    mqtt.connect();

    std::cout << "\n[Ready] QT 클라이언트 접속 URL:\n"
              << "  rtsp://서버IP:" << RTSP_PORT << "/live/pi_cam_01\n\n"
              << "Ctrl+C로 종료\n";

    // 메인 스레드 대기
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}