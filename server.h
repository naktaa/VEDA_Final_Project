#pragma once
#include <map>
#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <mosquitto.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

// 프레임 데이터 구조
struct FrameData {
    int room_id;
    int width;
    int height;
    std::vector<uint8_t> data;  // RGB
    uint64_t timestamp;
};

// 병실 RPi로부터 받는 패킷 헤더
struct VideoPacketHeader {
    uint32_t magic;        // 0x12345678
    uint32_t room_id;
    uint32_t packet_size;
    uint64_t timestamp;
    uint8_t frame_type;
} __attribute__((packed));

// Qt로 보내는 프레임 헤더
struct QtFrameHeader {
    uint32_t magic;        // 0xAABBCCDD
    uint32_t room_id;
    uint32_t width;
    uint32_t height;
    uint32_t data_size;
    uint64_t timestamp;
} __attribute__((packed));

// 각 병실 영상 수신기
class VideoReceiver {
private:
    int listen_port;
    int room_id;
    std::string room_name;
    int client_socket;
    
    AVCodecContext* codec_ctx;
    SwsContext* sws_ctx;
    
    std::queue<FrameData> frame_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    
    bool running;
    std::thread recv_thread;
    
public:
    VideoReceiver(int port, int room, const std::string& name) 
        : listen_port(port), room_id(room), room_name(name), 
          client_socket(-1), running(false), codec_ctx(nullptr), sws_ctx(nullptr) {
        
        // H.264 디코더 초기화
        const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        codec_ctx = avcodec_alloc_context3(codec);
        avcodec_open2(codec_ctx, codec, nullptr);
    }
    
    void start() {
        running = true;
        recv_thread = std::thread(&VideoReceiver::receiveLoop, this);
    }
    
    void receiveLoop() {
        // 서버 소켓 생성
        int server_sock = socket(AF_INET, SOCK_STREAM, 0);
        
        int opt = 1;
        setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(listen_port);
        
        if (bind(server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << room_name << " bind 실패" << std::endl;
            return;
        }
        
        listen(server_sock, 1);
        std::cout << room_name << " 대기 중 (포트 " << listen_port << ")" << std::endl;
        
        // 병실 RPi 연결 대기
        client_socket = accept(server_sock, nullptr, nullptr);
        std::cout << "✅ " << room_name << " RPi 연결됨!" << std::endl;
        
        close(server_sock);
        
        // 수신 루프
        while (running) {
            // ① 헤더 수신
            VideoPacketHeader header;
            ssize_t recv_size = recv(client_socket, &header, sizeof(header), MSG_WAITALL);
            
            if (recv_size != sizeof(header)) {
                std::cerr << "❌ " << room_name << " 연결 끊김, 재연결 대기..." << std::endl;
                close(client_socket);
                
                // 재연결 대기
                server_sock = socket(AF_INET, SOCK_STREAM, 0);
                setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
                bind(server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
                listen(server_sock, 1);
                
                client_socket = accept(server_sock, nullptr, nullptr);
                std::cout << "✅ " << room_name << " 재연결됨!" << std::endl;
                close(server_sock);
                continue;
            }
            
            // 매직 넘버 확인
            if (header.magic != 0x12345678) {
                std::cerr << room_name << " 잘못된 매직 넘버" << std::endl;
                continue;
            }
            
            // ② H.264 데이터 수신
            std::vector<uint8_t> h264_data(header.packet_size);
            recv_size = recv(client_socket, h264_data.data(), header.packet_size, MSG_WAITALL);
            
            if (recv_size != (ssize_t)header.packet_size) {
                std::cerr << room_name << " 패킷 손실" << std::endl;
                continue;
            }
            
            // ③ H.264 디코딩
            AVPacket* pkt = av_packet_alloc();
            pkt->data = h264_data.data();
            pkt->size = header.packet_size;
            
            if (avcodec_send_packet(codec_ctx, pkt) == 0) {
                AVFrame* frame_yuv = av_frame_alloc();
                
                while (avcodec_receive_frame(codec_ctx, frame_yuv) == 0) {
                    // 첫 프레임에서 해상도 확인 후 sws_ctx 초기화
                    if (!sws_ctx) {
                        sws_ctx = sws_getContext(
                            frame_yuv->width, frame_yuv->height, (AVPixelFormat)frame_yuv->format,
                            frame_yuv->width, frame_yuv->height, AV_PIX_FMT_RGB24,
                            SWS_BILINEAR, nullptr, nullptr, nullptr
                        );
                    }
                    
                    // ④ YUV → RGB 변환
                    AVFrame* frame_rgb = av_frame_alloc();
                    frame_rgb->format = AV_PIX_FMT_RGB24;
                    frame_rgb->width = frame_yuv->width;
                    frame_rgb->height = frame_yuv->height;
                    av_frame_get_buffer(frame_rgb, 0);
                    
                    sws_scale(sws_ctx, frame_yuv->data, frame_yuv->linesize, 0,
                             frame_yuv->height, frame_rgb->data, frame_rgb->linesize);
                    
                    // ⑤ FrameData 생성
                    FrameData frame_data;
                    frame_data.room_id = room_id;
                    frame_data.width = frame_rgb->width;
                    frame_data.height = frame_rgb->height;
                    frame_data.timestamp = header.timestamp;
                    
                    // RGB 데이터 복사
                    int data_size = frame_rgb->width * frame_rgb->height * 3;
                    frame_data.data.assign(frame_rgb->data[0], frame_rgb->data[0] + data_size);
                    
                    // ⑥ Queue에 저장
                    {
                        std::lock_guard<std::mutex> lock(queue_mutex);
                        
                        // 큐가 가득 차면 오래된 프레임 제거
                        if (frame_queue.size() >= 30) {
                            frame_queue.pop();
                        }
                        
                        frame_queue.push(std::move(frame_data));
                        queue_cv.notify_one();
                    }
                    
                    av_frame_free(&frame_rgb);
                }
                
                av_frame_free(&frame_yuv);
            }
            
            av_packet_free(&pkt);
        }
        
        close(client_socket);
    }
    
    bool getFrame(FrameData& frame, int timeout_ms = 100) {
        std::unique_lock<std::mutex> lock(queue_mutex);
        
        if (queue_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                             [this]{ return !frame_queue.empty(); })) {
            frame = std::move(frame_queue.front());
            frame_queue.pop();
            return true;
        }
        
        return false;
    }
    
    void stop() {
        running = false;
        if (recv_thread.joinable()) {
            recv_thread.join();
        }
    }
    
    ~VideoReceiver() {
        stop();
        if (codec_ctx) {
            avcodec_free_context(&codec_ctx);
        }
        if (sws_ctx) {
            sws_freeContext(sws_ctx);
        }
    }
};

// MQTT 이벤트 핸들러
class EventHandler {
private:
    struct mosquitto* mosq;
    std::function<void(std::string, std::string)> event_callback;
    
    static void on_message(struct mosquitto* mosq, void* obj, 
                          const struct mosquitto_message* msg) {
        EventHandler* handler = (EventHandler*)obj;
        
        std::string topic(msg->topic);
        std::string payload((char*)msg->payload, msg->payloadlen);
        
        std::cout << "📩 MQTT 이벤트 수신: " << topic << std::endl;
        std::cout << "   " << payload << std::endl;
        
        if (handler->event_callback) {
            handler->event_callback(topic, payload);
        }
    }
    
public:
    EventHandler() {
        mosquitto_lib_init();
        mosq = mosquitto_new("central_server", true, this);
        mosquitto_message_callback_set(mosq, on_message);
    }
    
    void start() {
        mosquitto_connect(mosq, "localhost", 1883, 60);
        mosquitto_subscribe(mosq, nullptr, "room/+/event", 0);
        mosquitto_loop_start(mosq);
        
        std::cout << "✅ MQTT 이벤트 구독 시작 (room/+/event)" << std::endl;
    }
    
    void setCallback(std::function<void(std::string, std::string)> callback) {
        event_callback = callback;
    }
    
    ~EventHandler() {
        mosquitto_loop_stop(mosq, false);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
    }
};

// 중앙 서버 메인 클래스
class CentralServer {
private:
    // 영상 수신기들
    std::map<int, std::unique_ptr<VideoReceiver>> receivers;
    
    // MQTT 이벤트 핸들러
    std::unique_ptr<EventHandler> event_handler;
    
    // Qt 클라이언트 관리
    int qt_server_socket;
    std::vector<int> qt_client_sockets;
    std::mutex clients_mutex;
    
    bool running;
    std::thread accept_thread;
    std::thread broadcast_thread;
    
public:
    CentralServer(int qt_port = 9000) : qt_server_socket(-1), running(false) {
        // Qt용 서버 소켓 생성
        qt_server_socket = socket(AF_INET, SOCK_STREAM, 0);
        
        int opt = 1;
        setsockopt(qt_server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(qt_port);
        
        bind(qt_server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
        listen(qt_server_socket, 5);
        
        std::cout << "✅ Qt 서버 시작 (포트 " << qt_port << ")" << std::endl;
        
        // MQTT 이벤트 핸들러 초기화
        event_handler = std::make_unique<EventHandler>();
        event_handler->setCallback([this](std::string topic, std::string payload) {
            this->onEventReceived(topic, payload);
        });
    }
    
    void addVideoReceiver(int room_id, int port, const std::string& room_name) {
        auto receiver = std::make_unique<VideoReceiver>(port, room_id, room_name);
        receivers[room_id] = std::move(receiver);
    }
    
    void start() {
        // 모든 비디오 수신기 시작
        for (auto& [room_id, receiver] : receivers) {
            receiver->start();
        }
        
        // MQTT 시작
        event_handler->start();
        
        // Qt 클라이언트 관리 스레드 시작
        running = true;
        accept_thread = std::thread(&CentralServer::acceptQtClients, this);
        broadcast_thread = std::thread(&CentralServer::broadcastToQt, this);
        
        std::cout << "✅ 중앙 서버 모든 스레드 시작 완료" << std::endl;
    }
    
    void acceptQtClients() {
        while (running) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_sock = accept(qt_server_socket, (struct sockaddr*)&client_addr, &client_len);
            
            if (client_sock > 0) {
                std::lock_guard<std::mutex> lock(clients_mutex);
                qt_client_sockets.push_back(client_sock);
                std::cout << "✅ Qt 클라이언트 연결: " << client_sock << std::endl;
            }
        }
    }
    
    void broadcastToQt() {
        while (running) {
            // 모든 병실 영상을 순회하면서 Qt로 전송
            for (auto& [room_id, receiver] : receivers) {
                FrameData frame;
                
                if (receiver->getFrame(frame, 10)) {
                    sendFrameToQt(frame);
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    
    void sendFrameToQt(const FrameData& frame) {
        std::lock_guard<std::mutex> lock(clients_mutex);
        
        // Qt 프레임 헤더 생성
        QtFrameHeader header;
        header.magic = 0xAABBCCDD;
        header.room_id = frame.room_id;
        header.width = frame.width;
        header.height = frame.height;
        header.data_size = frame.data.size();
        header.timestamp = frame.timestamp;
        
        // 연결 끊긴 클라이언트 목록
        std::vector<int> disconnected;
        
        // 모든 Qt 클라이언트에게 전송
        for (int client_sock : qt_client_sockets) {
            // 헤더 전송
            ssize_t sent = send(client_sock, &header, sizeof(header), MSG_NOSIGNAL);
            if (sent <= 0) {
                disconnected.push_back(client_sock);
                continue;
            }
            
            // RGB 데이터 전송
            sent = send(client_sock, frame.data.data(), frame.data.size(), MSG_NOSIGNAL);
            if (sent <= 0) {
                disconnected.push_back(client_sock);
            }
        }
        
        // 끊긴 클라이언트 제거
        for (int sock : disconnected) {
            close(sock);
            qt_client_sockets.erase(
                std::remove(qt_client_sockets.begin(), qt_client_sockets.end(), sock),
                qt_client_sockets.end()
            );
            std::cout << "❌ Qt 클라이언트 연결 해제: " << sock << std::endl;
        }
    }
    
    void onEventReceived(const std::string& topic, const std::string& payload) {
        // 이벤트를 Qt로도 전송할 수 있음
        // 또는 로그 파일에 저장
        
        // 예: 이벤트 발생 시 해당 병실 영상 강조 표시 명령 전송
        std::cout << "🚨 이벤트 처리: " << topic << std::endl;
        
        // TODO: Qt에 이벤트 알림 전송
    }
    
    void stop() {
        running = false;
        
        if (accept_thread.joinable()) {
            accept_thread.join();
        }
        if (broadcast_thread.joinable()) {
            broadcast_thread.join();
        }
        
        for (auto& [room_id, receiver] : receivers) {
            receiver->stop();
        }
        
        close(qt_server_socket);
        
        for (int sock : qt_client_sockets) {
            close(sock);
        }
    }
    
    ~CentralServer() {
        stop();
    }
};