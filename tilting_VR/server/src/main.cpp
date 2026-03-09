#include <atomic>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <termios.h>
#include <system_error>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <fcntl.h>
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>

namespace {

    using json = nlohmann::json;

    std::atomic<bool> g_running{true};

    constexpr float kImuAlpha = 0.2f;
    constexpr float kImuMaxDeg = 45.0f;
    constexpr int kPanId = 1;
    constexpr int kTiltId = 2;
    constexpr int kPanCenter = 1800;
    constexpr int kPanLeft = 2650;
    constexpr int kPanRight = 1150;
    constexpr int kTiltCenter = 2400;
    constexpr int kTiltUp = 2900;
    constexpr int kTiltDown = 2200;
    constexpr int kUpdateHz = 50;
    constexpr int kUpdateMs = 1000 / kUpdateHz;
    constexpr int kMaxStepPerTick = 20;

    float g_filteredPitch = 0.0f;
    float g_filteredRoll = 0.0f;
    std::mutex g_filterMutex;
    std::atomic<bool> g_servoReady{false};
    std::atomic<int> g_targetPan{kPanCenter};
    std::atomic<int> g_targetTilt{kTiltCenter};

    cv::Mat g_latestFrame;
    std::vector<uchar> g_latestJpeg;
    std::mutex g_frameMutex;
    std::atomic<uint64_t> g_frameCounter{0};
    std::mutex g_imuMutex;
    float g_lastPitch = 0.0f;
    float g_lastRoll = 0.0f;
    float g_lastYaw = 0.0f;
    uint64_t g_lastImuTimestamp = 0;
    std::mutex g_imuLogMutex;
    std::ofstream g_imuLogFile;
    bool g_imuLogReady = false;

    class ServoSerial {
    public:
        ServoSerial() = default;

        bool open_port(const std::string& device = "/dev/serial0", int baud = B115200) {
            fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
            if (fd_ < 0) {
                std::cerr << "[WARN] Failed to open serial " << device << ": " << std::strerror(errno)
                          << "\n";
                return false;
            }

            struct termios tty{};
            if (tcgetattr(fd_, &tty) != 0) {
                std::cerr << "[WARN] Failed to get serial attrs: " << std::strerror(errno) << "\n";
                close(fd_);
                fd_ = -1;
                return false;
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
                std::cerr << "[WARN] Failed to set serial attrs: " << std::strerror(errno) << "\n";
                close(fd_);
                fd_ = -1;
                return false;
            }

            std::cout << "[INFO] Serial opened: " << device << "\n";
            return true;
        }

        ~ServoSerial() {
            if (fd_ >= 0) {
                close(fd_);
                fd_ = -1;
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

        bool controlDouble(uint8_t index1, int angle1, uint8_t index2, int angle2) {
            if (fd_ < 0) {
                return false;
            }
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

    std::unique_ptr<ServoSerial> g_servo;

    std::string mime_type_for(const std::string& ext) {
        static const std::unordered_map<std::string, std::string> table = {
            {".html", "text/html; charset=utf-8"},
            {".js", "application/javascript; charset=utf-8"},
            {".css", "text/css; charset=utf-8"},
            {".json", "application/json; charset=utf-8"},
            {".png", "image/png"},
            {".jpg", "image/jpeg"},
            {".jpeg", "image/jpeg"},
            {".svg", "image/svg+xml"},
            {".ico", "image/x-icon"},
        };
        auto it = table.find(ext);
        return it == table.end() ? "application/octet-stream" : it->second;
    }

    float clampf(float v, float lo, float hi) {
        return std::max(lo, std::min(v, hi));
    }

    int lerp_int(int a, int b, float t) {
        return static_cast<int>(std::lround(static_cast<float>(a) + (b - a) * t));
    }

    int map_pan_from_roll(float roll) {
        const float r = clampf(roll, -kImuMaxDeg, kImuMaxDeg);
        if (r >= 0.0f) {
            const float t = r / kImuMaxDeg;
            return lerp_int(kPanCenter, kPanRight, t);
        }
        const float t = (-r) / kImuMaxDeg;
        return lerp_int(kPanCenter, kPanLeft, t);
    }

    int map_tilt_from_pitch(float pitch) {
        const float p = clampf(pitch, -kImuMaxDeg, kImuMaxDeg);
        if (p >= 0.0f) {
            const float t = p / kImuMaxDeg;
            return lerp_int(kTiltCenter, kTiltDown, t);
        }
        const float t = (-p) / kImuMaxDeg;
        return lerp_int(kTiltCenter, kTiltUp, t);
    }

void apply_imu(float pitch, float roll) {
  std::lock_guard<std::mutex> lock(g_filterMutex);
  // Phone orientation axes are swapped for our mount: roll drives tilt, pitch drives pan.
  g_filteredPitch = (1.0f - kImuAlpha) * g_filteredPitch + kImuAlpha * roll;
  g_filteredRoll = (1.0f - kImuAlpha) * g_filteredRoll + kImuAlpha * pitch;

        const int targetPan = map_pan_from_roll(g_filteredRoll);
        const int targetTilt = map_tilt_from_pitch(g_filteredPitch);
        g_targetPan.store(targetPan);
        g_targetTilt.store(targetTilt);
    }

    void stop_servo() {
        if (!g_servoReady.load() || !g_servo) {
            return;
        }
        g_servo->controlDouble(kPanId, kPanCenter, kTiltId, kTiltCenter);
    }

    int step_toward(int current, int target, int maxStep) {
        if (current < target) {
            return std::min(current + maxStep, target);
        }
        return std::max(current - maxStep, target);
    }

    void servo_update_loop() {
        if (!g_servoReady.load() || !g_servo) {
            return;
        }

        int currentPan = kPanCenter;
        int currentTilt = kTiltCenter;
        auto nextTick = std::chrono::steady_clock::now();

        while (g_running.load()) {
            const int targetPan = g_targetPan.load();
            const int targetTilt = g_targetTilt.load();

            currentPan = step_toward(currentPan, targetPan, kMaxStepPerTick);
            currentTilt = step_toward(currentTilt, targetTilt, kMaxStepPerTick);
            g_servo->controlDouble(kPanId, currentPan, kTiltId, currentTilt);

            nextTick += std::chrono::milliseconds(kUpdateMs);
            std::this_thread::sleep_until(nextTick);
        }
    }

    bool load_file_text(const std::filesystem::path& path, std::string& out) {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs.is_open()) {
            return false;
        }
        std::ostringstream ss;
        ss << ifs.rdbuf();
        out = ss.str();
        return true;
    }

    uint64_t epoch_ms_now() {
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
    }

    bool init_imu_log(const std::filesystem::path& logPath) {
        std::error_code ec;
        const bool exists = std::filesystem::exists(logPath, ec);
        const bool writeHeader = !exists || (std::filesystem::file_size(logPath, ec) == 0);

        g_imuLogFile.open(logPath, std::ios::app);
        if (!g_imuLogFile.is_open()) {
            std::cerr << "[WARN] Failed to open IMU log file: " << logPath << "\n";
            return false;
        }

        if (writeHeader) {
            g_imuLogFile << "server_epoch_ms,client_t_ms,pitch,roll,yaw,servo_ready\n";
        }
        g_imuLogReady = true;
        std::cout << "[INFO] IMU logging enabled: " << logPath << "\n";
        return true;
    }

    void append_imu_log(float pitch, float roll, float yaw, uint64_t clientT) {
        std::lock_guard<std::mutex> lock(g_imuLogMutex);
        if (!g_imuLogReady || !g_imuLogFile.is_open()) {
            return;
        }
        g_imuLogFile << epoch_ms_now() << "," << clientT << "," << pitch << "," << roll << "," << yaw
                     << "," << (g_servoReady.load() ? 1 : 0) << "\n";
        g_imuLogFile.flush();
    }

    void publish_frame(const cv::Mat& img) {
        std::vector<uchar> jpg;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
        if (!cv::imencode(".jpg", img, jpg, params)) {
            return;
        }
        std::lock_guard<std::mutex> lock(g_frameMutex);
        g_latestFrame = img;
        g_latestJpeg = std::move(jpg);
        g_frameCounter.fetch_add(1);
    }

    void camera_capture_loop(int camIndex, int width, int height, int fps) {
        cv::VideoCapture cap;
        std::ostringstream pipeline;
        pipeline << "libcamerasrc ! "
                 << "video/x-raw,width=" << width << ",height=" << height << ",framerate=" << fps
                 << "/1 ! videoconvert ! video/x-raw,format=BGR ! "
                 << "appsink drop=true max-buffers=1 sync=false";

        if (cap.open(pipeline.str(), cv::CAP_GSTREAMER)) {
            std::cout << "[INFO] Camera opened with GStreamer pipeline.\n";
        }
        else if (cap.open(camIndex, cv::CAP_V4L2)) {
            std::cout << "[INFO] Camera opened with V4L2 backend.\n";
        }
        else if (cap.open(camIndex)) {
            std::cout << "[INFO] Camera opened with OpenCV default backend.\n";
        }

        if (!cap.isOpened()) {
            std::cerr << "[WARN] Camera open failed. Falling back to test pattern.\n";
            while (g_running.load()) {
                cv::Mat img(height, width, CV_8UC3, cv::Scalar(20, 20, 20));
                auto t = std::chrono::steady_clock::now().time_since_epoch();
                int ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(t).count());
                int x = (ms / 10) % std::max(1, width - 100);
                cv::rectangle(img, cv::Rect(x, 30, 100, 80), cv::Scalar(0, 220, 255), cv::FILLED);
                cv::putText(img, "NO CAMERA", cv::Point(40, height - 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                            cv::Scalar(255, 255, 255), 2);
                publish_frame(img);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000 / std::max(1, fps)));
            }
            return;
        }

        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap.set(cv::CAP_PROP_FPS, fps);
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

        int readFailCount = 0;
        bool useSynthetic = false;

        while (g_running.load()) {
            if (useSynthetic) {
                cv::Mat img(height, width, CV_8UC3, cv::Scalar(25, 25, 25));
                auto t = std::chrono::steady_clock::now().time_since_epoch();
                int ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(t).count());
                int x = (ms / 10) % std::max(1, width - 120);
                cv::rectangle(img, cv::Rect(x, 24, 120, 80), cv::Scalar(0, 210, 255), cv::FILLED);
                cv::putText(img, "CAM READ FAIL -> TEST PATTERN", cv::Point(24, height - 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                publish_frame(img);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000 / std::max(1, fps)));
                continue;
            }

            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) {
                readFailCount++;
                if (readFailCount > 30) {
                    std::cerr << "[WARN] Camera opened but frame read failed repeatedly. "
                                 "Switching to test pattern.\n";
                    useSynthetic = true;
                }
                else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                continue;
            }

            readFailCount = 0;
            publish_frame(frame);
        }
    }

    void signal_handler(int) {
        g_running.store(false);
    }

} // namespace

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    g_servo = std::make_unique<ServoSerial>();
    if (!g_servo->open_port("/dev/serial0", B115200)) {
        g_servoReady.store(false);
        std::cerr << "[WARN] Serial servo init failed. IMU control will be disabled.\n";
    }
    else {
        g_servoReady.store(true);
        g_targetPan.store(kPanCenter);
        g_targetTilt.store(kTiltCenter);
        g_servo->controlDouble(kPanId, kPanCenter, kTiltId, kTiltCenter);
        std::cout << "[INFO] Serial servo ready. pan_id=" << kPanId << " tilt_id=" << kTiltId << "\n";
    }

    std::thread servoThread;
    if (g_servoReady.load()) {
        servoThread = std::thread(servo_update_loop);
    }

    const int camIndex = 0;
    const int width = 480;
    const int height = 360;
    const int fps = 15;

    std::thread cameraThread(camera_capture_loop, camIndex, width, height, fps);

    httplib::Server svr;
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();
    const auto webRoot = root / "web";
    const auto logDir = root / "logs";
    const auto imuLogPath = logDir / "imu_log.csv";

    std::error_code mkdirEc;
    std::filesystem::create_directories(logDir, mkdirEc);
    if (mkdirEc) {
        std::cerr << "[WARN] Failed to create log directory: " << logDir << "\n";
    }
    init_imu_log(imuLogPath);

    // Stop blocking listen() when SIGINT/SIGTERM flips g_running to false.
    std::thread shutdownWatcher([&svr]() {
        while (g_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        svr.stop();
    });

    svr.Get("/health", [](const httplib::Request&, httplib::Response& res) {
        json body = {{"ok", true}};
        res.set_content(body.dump(), "application/json");
    });

    svr.Get("/imu/latest", [](const httplib::Request&, httplib::Response& res) {
        std::lock_guard<std::mutex> lock(g_imuMutex);
        json body = {
            {"pitch", g_lastPitch},
            {"roll", g_lastRoll},
            {"yaw", g_lastYaw},
            {"t", g_lastImuTimestamp},
            {"servo_ready", g_servoReady.load()},
        };
        res.set_content(body.dump(), "application/json");
    });

    svr.Post("/imu", [](const httplib::Request& req, httplib::Response& res) {
        try {
            const json payload = json::parse(req.body);
            if (payload.value("type", "") == "imu") {
                float pitch = payload.value("pitch", 0.0f);
                float roll = payload.value("roll", 0.0f);
                float yaw = payload.value("yaw", 0.0f);
                uint64_t t = payload.value("t", static_cast<uint64_t>(0));
                {
                    std::lock_guard<std::mutex> lock(g_imuMutex);
                    g_lastPitch = pitch;
                    g_lastRoll = roll;
                    g_lastYaw = yaw;
                    g_lastImuTimestamp = t;
                }
                append_imu_log(pitch, roll, yaw, t);
                apply_imu(pitch, roll);
            }
            res.status = 204;
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(std::string("bad json: ") + e.what(), "text/plain; charset=utf-8");
        }
    });

    svr.Get("/stream.mjpg", [fps](const httplib::Request&, httplib::Response& res) {
        res.set_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        res.set_header("Pragma", "no-cache");
        res.set_header("Connection", "close");
        res.set_chunked_content_provider(
            "multipart/x-mixed-replace; boundary=frame",
            [fps](size_t, httplib::DataSink& sink) {
                static thread_local uint64_t lastFrame = 0;
                while (g_running.load()) {
                    std::vector<uchar> jpg;
                    uint64_t current = g_frameCounter.load();
                    {
                        std::lock_guard<std::mutex> lock(g_frameMutex);
                        if (!g_latestJpeg.empty() && current != lastFrame) {
                            jpg = g_latestJpeg;
                            lastFrame = current;
                        }
                    }

                    if (jpg.empty()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }

                    std::ostringstream partHeader;
                    partHeader << "--frame\r\n"
                               << "Content-Type: image/jpeg\r\n"
                               << "Content-Length: " << jpg.size() << "\r\n\r\n";
                    const std::string header = partHeader.str();
                    if (!sink.write(header.data(), header.size())) {
                        return false;
                    }
                    if (!sink.write(reinterpret_cast<const char*>(jpg.data()), jpg.size())) {
                        return false;
                    }
                    if (!sink.write("\r\n", 2)) {
                        return false;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / std::max(1, fps)));
                }
                return false;
            },
            [](bool) {});
    });

    svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
        res.status = 302;
        res.set_header("Location", "/web/");
    });

    svr.Get(R"(/web/?$)", [webRoot](const httplib::Request&, httplib::Response& res) {
        std::string body;
        if (!load_file_text(webRoot / "index.html", body)) {
            res.status = 404;
            return;
        }
        res.set_content(body, "text/html; charset=utf-8");
    });

    svr.Get(R"(/web/(.+))", [webRoot](const httplib::Request& req, httplib::Response& res) {
        const auto rel = req.matches[1].str();
        const auto filePath = webRoot / rel;
        std::string body;
        if (!load_file_text(filePath, body)) {
            res.status = 404;
            return;
        }
        res.set_content(body, mime_type_for(filePath.extension().string()).c_str());
    });

    std::cout << "[INFO] tilting_vr_server listening on 0.0.0.0:8000\n";
    std::cout << "[INFO] open http://<pi-ip>:8000/web/\n";
    svr.listen("0.0.0.0", 8000);

    g_running.store(false);
    if (shutdownWatcher.joinable()) {
        shutdownWatcher.join();
    }
    if (cameraThread.joinable()) {
        cameraThread.join();
    }
    if (servoThread.joinable()) {
        servoThread.join();
    }
    {
        std::lock_guard<std::mutex> lock(g_imuLogMutex);
        if (g_imuLogFile.is_open()) {
            g_imuLogFile.flush();
            g_imuLogFile.close();
        }
        g_imuLogReady = false;
    }
    stop_servo();
    return 0;
}
