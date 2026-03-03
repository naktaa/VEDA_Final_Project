// server_main.cpp
#include <iostream>
#include <thread>
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>
#include <queue>
#include <map>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

/* ========================================
   프레임 데이터 구조
   ======================================== */
struct Frame {
    int camera_id;
    std::vector<uint8_t> data;  // Raw YUV420P 데이터
    int width;
    int height;
    int64_t timestamp;
};

/* ========================================
   Thread-Safe 프레임 큐
   ======================================== */
class FrameQueue {
private:
    std::queue<Frame> queue;
    std::mutex mutex;
    const size_t max_size = 30;  // 최대 30프레임 버퍼
    
public:
    void push(const Frame& frame) {
        std::lock_guard<std::mutex> lock(mutex);
        
        while (queue.size() >= max_size) {
            queue.pop();  // 오래된 프레임 버림
        }
        
        queue.push(frame);
    }
    
    bool pop(Frame& frame) {
        std::lock_guard<std::mutex> lock(mutex);
        
        if (queue.empty()) return false;
        
        frame = queue.front();
        queue.pop();
        return true;
    }
    
    size_t size() {
        std::lock_guard<std::mutex> lock(mutex);
        return queue.size();
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(mutex);
        while (!queue.empty()) {
            queue.pop();
        }
    }
};

/* ========================================
   라즈베리파이 RTSP 클라이언트
   ======================================== */
class RaspberryPiRTSPClient {
private:
    std::string rtsp_url;
    int camera_id;
    std::atomic<bool> running;
    std::thread worker_thread;
    FrameQueue* output_queue;
    
    // 소켓
    int rtsp_socket;
    int rtp_socket;
    int rtcp_socket;
    
    // RTSP 상태
    int cseq;
    std::string session_id;
    
    // 디코더
    AVCodecContext* codec_ctx;
    AVFrame* av_frame;
    AVPacket* packet;
    SwsContext* sws_ctx;
    
    // RTP 헤더 구조체
    struct RTPHeader {
        uint8_t version;
        uint8_t padding;
        uint8_t extension;
        uint8_t csrc_count;
        uint8_t marker;
        uint8_t payload_type;
        uint16_t sequence_number;
        uint32_t timestamp;
        uint32_t ssrc;
    };
    
    // H264 Depacketizer
    class H264Depacketizer {
    private:
        std::vector<uint8_t> buffer;
        bool frame_complete;
        
    public:
        H264Depacketizer() : frame_complete(false) {}
        
        void addPayload(const uint8_t* payload, int len, bool marker) {
            buffer.insert(buffer.end(), payload, payload + len);
            frame_complete = marker;
        }
        
        bool isComplete() const { return frame_complete; }
        
        std::vector<uint8_t> getFrame() {
            auto result = std::move(buffer);
            buffer.clear();
            frame_complete = false;
            return result;
        }
        
        void reset() {
            buffer.clear();
            frame_complete = false;
        }
    };
    
    H264Depacketizer depacketizer;
    
    // URL 파싱
    struct URLInfo {
        std::string protocol;
        std::string host;
        int port;
        std::string path;
    };
    
    URLInfo parseURL(const std::string& url) {
        URLInfo info;
        
        // rtsp://192.168.100.10:8554/stream
        size_t proto_end = url.find("://");
        if (proto_end != std::string::npos) {
            info.protocol = url.substr(0, proto_end);
            
            size_t host_start = proto_end + 3;
            size_t port_start = url.find(":", host_start);
            size_t path_start = url.find("/", host_start);
            
            if (port_start != std::string::npos && port_start < path_start) {
                info.host = url.substr(host_start, port_start - host_start);
                info.port = std::stoi(url.substr(port_start + 1, path_start - port_start - 1));
            } else {
                info.host = url.substr(host_start, path_start - host_start);
                info.port = 8554;
            }
            
            info.path = url.substr(path_start);
        }
        
        return info;
    }
    
    // 소켓 생성
    int createTCPSocket(const std::string& host, int port) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            std::cerr << "Socket creation failed" << std::endl;
            return -1;
        }
        
        struct hostent* server = gethostbyname(host.c_str());
        if (!server) {
            std::cerr << "Host not found: " << host << std::endl;
            close(sock);
            return -1;
        }
        
        struct sockaddr_in serv_addr;
        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
        serv_addr.sin_port = htons(port);
        
        if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            std::cerr << "Connection failed to " << host << ":" << port << std::endl;
            close(sock);
            return -1;
        }
        
        return sock;
    }
    
    int createUDPSocket() {
        return socket(AF_INET, SOCK_DGRAM, 0);
    }
    
    int bindUDPSocket(int sock, int& port) {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = 0;  // 자동 할당
        
        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            return -1;
        }
        
        socklen_t len = sizeof(addr);
        getsockname(sock, (struct sockaddr*)&addr, &len);
        port = ntohs(addr.sin_port);
        
        // 타임아웃 설정 (5초)
        struct timeval tv;
        tv.tv_sec = 5;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        return 0;
    }
    
    // RTSP 요청 전송
    std::string sendRequest(const std::string& method,
                           const std::string& uri,
                           const std::map<std::string, std::string>& headers) {
        std::ostringstream request;
        request << method << " " << uri << " RTSP/1.0\r\n";
        request << "CSeq: " << cseq++ << "\r\n";
        
        for (const auto& [key, value] : headers) {
            request << key << ": " << value << "\r\n";
        }
        request << "\r\n";
        
        std::string req_str = request.str();
        send(rtsp_socket, req_str.c_str(), req_str.length(), 0);
        
        char buffer[4096];
        ssize_t len = recv(rtsp_socket, buffer, sizeof(buffer) - 1, 0);
        if (len > 0) {
            buffer[len] = '\0';
            return std::string(buffer, len);
        }
        
        return "";
    }
    
    // RTSP 메서드
    bool connectRTSP() {
        URLInfo url_info = parseURL(rtsp_url);
        
        std::cout << "[Cam " << camera_id << "] Connecting to " 
                  << url_info.host << ":" << url_info.port << std::endl;
        
        rtsp_socket = createTCPSocket(url_info.host, url_info.port);
        if (rtsp_socket < 0) return false;
        
        if (!sendOptions()) return false;
        if (!sendDescribe()) return false;
        if (!sendSetup()) return false;
        if (!sendPlay()) return false;
        
        std::cout << "[Cam " << camera_id << "] RTSP connection established" << std::endl;
        return true;
    }
    
    bool sendOptions() {
        std::string response = sendRequest("OPTIONS", rtsp_url, {});
        return response.find("200 OK") != std::string::npos;
    }
    
    bool sendDescribe() {
        std::map<std::string, std::string> headers;
        headers["Accept"] = "application/sdp";
        
        std::string response = sendRequest("DESCRIBE", rtsp_url, headers);
        return response.find("200 OK") != std::string::npos;
    }
    
    bool sendSetup() {
        rtp_socket = createUDPSocket();
        rtcp_socket = createUDPSocket();
        
        if (rtp_socket < 0 || rtcp_socket < 0) {
            std::cerr << "[Cam " << camera_id << "] Failed to create UDP sockets" << std::endl;
            return false;
        }
        
        int rtp_port, rtcp_port;
        bindUDPSocket(rtp_socket, rtp_port);
        bindUDPSocket(rtcp_socket, rtcp_port);
        
        std::cout << "[Cam " << camera_id << "] RTP port: " << rtp_port 
                  << ", RTCP port: " << rtcp_port << std::endl;
        
        std::map<std::string, std::string> headers;
        headers["Transport"] = "RTP/AVP;unicast;client_port=" + 
                              std::to_string(rtp_port) + "-" + 
                              std::to_string(rtcp_port);
        
        std::string response = sendRequest("SETUP", rtsp_url, headers);
        
        // Session ID 추출
        size_t pos = response.find("Session: ");
        if (pos != std::string::npos) {
            size_t end = response.find("\r\n", pos);
            session_id = response.substr(pos + 9, end - pos - 9);
            
            size_t semi = session_id.find(';');
            if (semi != std::string::npos) {
                session_id = session_id.substr(0, semi);
            }
            
            std::cout << "[Cam " << camera_id << "] Session ID: " << session_id << std::endl;
        }
        
        return response.find("200 OK") != std::string::npos;
    }
    
    bool sendPlay() {
        std::map<std::string, std::string> headers;
        headers["Session"] = session_id;
        headers["Range"] = "npt=0.000-";
        
        std::string response = sendRequest("PLAY", rtsp_url, headers);
        return response.find("200 OK") != std::string::npos;
    }
    
    void sendTeardown() {
        if (rtsp_socket < 0) return;
        
        std::map<std::string, std::string> headers;
        headers["Session"] = session_id;
        
        sendRequest("TEARDOWN", rtsp_url, headers);
        
        close(rtsp_socket);
        if (rtp_socket >= 0) close(rtp_socket);
        if (rtcp_socket >= 0) close(rtcp_socket);
        
        rtsp_socket = -1;
        rtp_socket = -1;
        rtcp_socket = -1;
    }
    
    // 디코더 초기화
    bool initDecoder() {
        const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec) {
            std::cerr << "[Cam " << camera_id << "] H264 codec not found" << std::endl;
            return false;
        }
        
        codec_ctx = avcodec_alloc_context3(codec);
        if (!codec_ctx) {
            std::cerr << "[Cam " << camera_id << "] Could not allocate codec context" << std::endl;
            return false;
        }
        
        if (avcodec_open2(codec_ctx, codec, NULL) < 0) {
            std::cerr << "[Cam " << camera_id << "] Could not open codec" << std::endl;
            avcodec_free_context(&codec_ctx);
            return false;
        }
        
        av_frame = av_frame_alloc();
        packet = av_packet_alloc();
        
        std::cout << "[Cam " << camera_id << "] Decoder initialized" << std::endl;
        return true;
    }
    
    void cleanupDecoder() {
        if (av_frame) av_frame_free(&av_frame);
        if (packet) av_packet_free(&packet);
        if (codec_ctx) avcodec_free_context(&codec_ctx);
        if (sws_ctx) sws_freeContext(sws_ctx);
        
        av_frame = nullptr;
        packet = nullptr;
        codec_ctx = nullptr;
        sws_ctx = nullptr;
    }
    
    // RTP 헤더 파싱
    RTPHeader parseRTPHeader(const uint8_t* data) {
        RTPHeader header;
        header.version = (data[0] >> 6) & 0x03;
        header.padding = (data[0] >> 5) & 0x01;
        header.extension = (data[0] >> 4) & 0x01;
        header.csrc_count = data[0] & 0x0F;
        header.marker = (data[1] >> 7) & 0x01;
        header.payload_type = data[1] & 0x7F;
        header.sequence_number = (data[2] << 8) | data[3];
        header.timestamp = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
        header.ssrc = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11];
        
        return header;
    }
    
    // H264 프레임 처리
    void processH264Frame(const std::vector<uint8_t>& h264Data) {
        if (h264Data.empty()) return;
        
        // AVPacket에 데이터 설정
        av_packet_unref(packet);
        packet->data = const_cast<uint8_t*>(h264Data.data());
        packet->size = h264Data.size();
        
        int ret = avcodec_send_packet(codec_ctx, packet);
        if (ret < 0) {
            return;
        }
        
        while (ret >= 0) {
            ret = avcodec_receive_frame(codec_ctx, av_frame);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                break;
            } else if (ret < 0) {
                std::cerr << "[Cam " << camera_id << "] Error decoding frame" << std::endl;
                return;
            }
            
            // YUV420P 프레임 데이터 추출
            Frame frame;
            frame.camera_id = camera_id;
            frame.width = av_frame->width;
            frame.height = av_frame->height;
            frame.timestamp = av_frame->pts;
            
            // YUV420P 데이터 크기 계산
            int y_size = frame.width * frame.height;
            int uv_size = y_size / 4;
            int total_size = y_size + uv_size * 2;
            
            frame.data.resize(total_size);
            
            // Y plane 복사
            memcpy(frame.data.data(), av_frame->data[0], y_size);
            // U plane 복사
            memcpy(frame.data.data() + y_size, av_frame->data[1], uv_size);
            // V plane 복사
            memcpy(frame.data.data() + y_size + uv_size, av_frame->data[2], uv_size);
            
            // 큐에 푸시
            if (output_queue) {
                output_queue->push(frame);
            }
        }
    }
    
    // RTP 수신 루프
    void receiveLoop() {
        if (!connectRTSP()) {
            std::cerr << "[Cam " << camera_id << "] RTSP connection failed" << std::endl;
            return;
        }
        
        if (!initDecoder()) {
            std::cerr << "[Cam " << camera_id << "] Decoder initialization failed" << std::endl;
            sendTeardown();
            return;
        }
        
        uint8_t buffer[2048];
        std::cout << "[Cam " << camera_id << "] Starting RTP receive loop" << std::endl;
        
        while (running) {
            ssize_t len = recv(rtp_socket, buffer, sizeof(buffer), 0);
            
            if (len <= 0) {
                if (running) {
                    std::cerr << "[Cam " << camera_id << "] RTP receive error or timeout" << std::endl;
                }
                continue;
            }
            
            if (len < 12) continue;  // RTP 헤더보다 작으면 무시
            
            // RTP 헤더 파싱
            RTPHeader header = parseRTPHeader(buffer);
            
            // RTP 페이로드 추출
            const uint8_t* payload = buffer + 12;
            int payload_len = len - 12;
            
            if (payload_len <= 0) continue;
            
            // H.264 NAL Unit 타입 확인
            uint8_t nal_type = payload[0] & 0x1F;
            
            if (nal_type == 28) {  // FU-A (Fragmentation Unit)
                if (payload_len < 2) continue;
                
                uint8_t fu_header = payload[1];
                bool start = fu_header & 0x80;
                bool end = fu_header & 0x40;
                
                if (start) {
                    // 새로운 NAL 시작 - NAL 헤더 재구성
                    uint8_t nal_header = (payload[0] & 0xE0) | (fu_header & 0x1F);
                    depacketizer.reset();
                    depacketizer.addPayload(&nal_header, 1, false);
                    depacketizer.addPayload(payload + 2, payload_len - 2, false);
                } else {
                    depacketizer.addPayload(payload + 2, payload_len - 2, end);
                }
                
                if (end && depacketizer.isComplete()) {
                    auto nalUnit = depacketizer.getFrame();
                    processH264Frame(nalUnit);
                }
            } else {
                // Single NAL Unit
                depacketizer.reset();
                depacketizer.addPayload(payload, payload_len, true);
                auto nalUnit = depacketizer.getFrame();
                processH264Frame(nalUnit);
            }
        }
        
        std::cout << "[Cam " << camera_id << "] RTP receive loop stopped" << std::endl;
    }
    
public:
    RaspberryPiRTSPClient(const std::string& url, int id, FrameQueue* queue)
        : rtsp_url(url), camera_id(id), running(false), output_queue(queue),
          rtsp_socket(-1), rtp_socket(-1), rtcp_socket(-1),
          cseq(1), codec_ctx(nullptr), av_frame(nullptr), packet(nullptr), sws_ctx(nullptr)
    {}
    
    ~RaspberryPiRTSPClient() {
        stop();
    }
    
    bool start() {
        if (running) return false;
        
        running = true;
        worker_thread = std::thread(&RaspberryPiRTSPClient::receiveLoop, this);
        
        return true;
    }
    
    void stop() {
        if (!running) return;
        
        running = false;
        sendTeardown();
        
        if (worker_thread.joinable()) {
            worker_thread.join();
        }
        
        cleanupDecoder();
    }
};

/* ========================================
   GStreamer 한화비전 릴레이 서버
   ======================================== */
class HanwhaRelayServer {
private:
    GMainLoop* loop;
    GstRTSPServer* server;
    GThread* gst_thread;
    GstRTSPMediaFactory* factory;
    
    static gpointer runMainLoop(gpointer data) {
        g_main_loop_run((GMainLoop*)data);
        return nullptr;
    }
    
    static GstRTSPMediaFactory* createRelayFactory(const gchar* uri) {
        GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
        
        gchar* pipeline = g_strdup_printf(
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
        
        std::cout << "Pipeline: " << pipeline << std::endl;
        g_free(pipeline);
        
        return factory;
    }
    
public:
    HanwhaRelayServer() {
        gst_init(nullptr, nullptr);
        
        loop = g_main_loop_new(nullptr, FALSE);
        server = gst_rtsp_server_new();
        gst_rtsp_server_set_service(server, "8554");
    }
    
    bool start(const std::string& camera_uri) {
        GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);
        
        factory = createRelayFactory(camera_uri.c_str());
        gst_rtsp_mount_points_add_factory(mounts, "/cam0", factory);
        
        std::cout << "✅ Hanwha Camera ready at rtsp://0.0.0.0:8554/cam0" << std::endl;
        
        g_object_unref(mounts);
        gst_rtsp_server_attach(server, nullptr);
        
        gst_thread = g_thread_new("hanwha-loop", runMainLoop, loop);
        
        std::cout << "🚀 Hanwha Relay Server started on port 8554" << std::endl;
        return true;
    }
    
    void stop() {
        if (loop) g_main_loop_quit(loop);
        if (gst_thread) g_thread_join(gst_thread);
    }
    
    ~HanwhaRelayServer() {
        stop();
        if (loop) g_main_loop_unref(loop);
        if (server) g_object_unref(server);
    }
};

/* ========================================
   메인 서버 애플리케이션
   ======================================== */
class CameraServer {
private:
    std::unique_ptr<HanwhaRelayServer> hanwha_server;
    std::vector<std::unique_ptr<RaspberryPiRTSPClient>> rpi_clients;
    std::vector<FrameQueue> frame_queues;  // 라즈베리파이용 큐
    
public:
    CameraServer() {
        system("fuser -k 8554/tcp 2>/dev/null");
        frame_queues.resize(2);  // 라즈베리파이 2대
    }
    
    bool startHanwhaRelay() {
        hanwha_server = std::make_unique<HanwhaRelayServer>();
        
        const std::string uri = "rtsp://admin:team3%40%40%40@192.168.100.16/profile2/media.smp";
        
        return hanwha_server->start(uri);
    }
    
    bool startRaspberryPiClients() {
        std::vector<std::string> rpi_urls = {
            "rtsp://192.168.100.10:8554/stream",
            "rtsp://192.168.100.11:8554/stream"
        };
        
        for (size_t i = 0; i < rpi_urls.size(); i++) {
            auto client = std::make_unique<RaspberryPiRTSPClient>(
                rpi_urls[i], 
                i,  // camera_id 0, 1
                &frame_queues[i]
            );
            
            if (!client->start()) {
                std::cerr << "Failed to start RaspberryPi camera " << i << std::endl;
                continue;
            }
            
            rpi_clients.push_back(std::move(client));
            std::cout << "✅ RaspberryPi Camera " << i << " started" << std::endl;
        }
        
        return !rpi_clients.empty();
    }
    
    void run() {
        std::cout << "\n===============================\n";
        std::cout << "=== Camera Server Running ===\n";
        std::cout << "===============================\n";
        std::cout << "Hanwha Camera: rtsp://localhost:8554/cam0\n";
        std::cout << "RaspberryPi cameras running in background\n";
        std::cout << "Press Ctrl+C to stop...\n";
        std::cout << "===============================\n\n";
        
        // 프레임 큐 모니터링 (디버깅용)
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            for (size_t i = 0; i < frame_queues.size(); i++) {
                std::cout << "[Monitor] RaspberryPi " << i 
                         << " queue size: " << frame_queues[i].size() << std::endl;
            }
        }
    }
    
    void stop() {
        std::cout << "\nStopping camera server..." << std::endl;
        
        for (auto& client : rpi_clients) {
            client->stop();
        }
        
        if (hanwha_server) {
            hanwha_server->stop();
        }
        
        std::cout << "Camera server stopped" << std::endl;
    }
    
    FrameQueue* getFrameQueue(int camera_id) {
        if (camera_id < 0 || camera_id >= (int)frame_queues.size()) {
            return nullptr;
        }
        return &frame_queues[camera_id];
    }
};

/* ========================================
   MAIN
   ======================================== */
int main(int argc, char* argv[]) {
    std::cout << "Starting Camera Server...\n" << std::endl;
    
    CameraServer server;
    
    // 1. 한화비전 릴레이 서버 시작 (1채널)
    if (!server.startHanwhaRelay()) {
        std::cerr << "Failed to start Hanwha relay server" << std::endl;
        return 1;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 2. 라즈베리파이 클라이언트 시작 (2채널)
    if (!server.startRaspberryPiClients()) {
        std::cerr << "Warning: No RaspberryPi cameras started" << std::endl;
        // 라즈베리파이 없어도 계속 진행
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 3. 서버 실행
    try {
        server.run();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        server.stop();
        return 1;
    }
    
    return 0;
}