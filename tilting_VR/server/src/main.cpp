#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <system_error>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <fcntl.h>
#include <httplib.h>
#include <linux/i2c-dev.h>
#include <nlohmann/json.hpp>
#include <sys/ioctl.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>

namespace {

using json = nlohmann::json;

std::atomic<bool> g_running{true};

constexpr int kMinPulse = 500;
constexpr int kMaxPulse = 2500;
constexpr float kFilterAlpha = 0.25f;
constexpr int kPcaAddress = 0x40;
constexpr int kPcaBus = 1;
constexpr int kPanChannel = 0;
constexpr int kTiltChannel = 1;

float g_filteredPitch = 0.0f;
float g_filteredRoll = 0.0f;
std::mutex g_servoMutex;
bool g_servoReady = false;

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

class PCA9685Controller {
 public:
  PCA9685Controller(int bus, int address)
      : devicePath_("/dev/i2c-" + std::to_string(bus)), address_(address), fd_(-1) {}

  bool init() {
    fd_ = open(devicePath_.c_str(), O_RDWR);
    if (fd_ < 0) {
      std::cerr << "[WARN] Failed to open " << devicePath_ << "\n";
      return false;
    }
    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
      std::cerr << "[WARN] Failed to set I2C address 0x" << std::hex << address_ << std::dec
                << "\n";
      close(fd_);
      fd_ = -1;
      return false;
    }
    if (!set_pwm_frequency(50.0f)) {
      return false;
    }
    return true;
  }

  ~PCA9685Controller() {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  bool set_channel_pulse_us(int channel, int pulseUs) {
    pulseUs = std::max(500, std::min(2500, pulseUs));
    int ticks = static_cast<int>(std::lround((pulseUs / 20000.0f) * 4096.0f));
    ticks = std::max(0, std::min(4095, ticks));
    return set_pwm(channel, 0, ticks);
  }

 private:
  static constexpr int MODE1 = 0x00;
  static constexpr int MODE2 = 0x01;
  static constexpr int PRESCALE = 0xFE;
  static constexpr int LED0_ON_L = 0x06;

  bool write_reg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(fd_, buffer, 2) != 2) {
      std::cerr << "[WARN] I2C write failed reg=0x" << std::hex << static_cast<int>(reg)
                << std::dec << "\n";
      return false;
    }
    return true;
  }

  bool read_reg(uint8_t reg, uint8_t& out) {
    if (write(fd_, &reg, 1) != 1) {
      return false;
    }
    if (read(fd_, &out, 1) != 1) {
      return false;
    }
    return true;
  }

  bool set_pwm(int channel, int on, int off) {
    if (channel < 0 || channel > 15) {
      return false;
    }
    uint8_t reg = static_cast<uint8_t>(LED0_ON_L + 4 * channel);
    return write_reg(reg, static_cast<uint8_t>(on & 0xFF)) &&
           write_reg(reg + 1, static_cast<uint8_t>((on >> 8) & 0x0F)) &&
           write_reg(reg + 2, static_cast<uint8_t>(off & 0xFF)) &&
           write_reg(reg + 3, static_cast<uint8_t>((off >> 8) & 0x0F));
  }

  bool set_pwm_frequency(float hz) {
    if (hz < 1.0f) {
      hz = 1.0f;
    }
    if (hz > 3500.0f) {
      hz = 3500.0f;
    }
    const float prescaleVal = 25000000.0f / (4096.0f * hz) - 1.0f;
    const auto prescale = static_cast<uint8_t>(std::lround(prescaleVal));

    uint8_t oldMode = 0;
    if (!read_reg(MODE1, oldMode)) {
      return false;
    }
    const uint8_t sleepMode = (oldMode & 0x7F) | 0x10;
    if (!write_reg(MODE1, sleepMode)) {
      return false;
    }
    if (!write_reg(PRESCALE, prescale)) {
      return false;
    }
    if (!write_reg(MODE1, oldMode)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (!write_reg(MODE1, static_cast<uint8_t>(oldMode | 0xA1))) {
      return false;
    }
    if (!write_reg(MODE2, 0x04)) {
      return false;
    }
    return true;
  }

  std::string devicePath_;
  int address_;
  int fd_;
};

std::unique_ptr<PCA9685Controller> g_pca;

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

int angle_to_pulse(float angle_deg) {
  const float min_angle = -90.0f;
  const float max_angle = 90.0f;
  float angle = clampf(angle_deg, min_angle, max_angle);
  float norm = (angle - min_angle) / (max_angle - min_angle);
  float pulse = static_cast<float>(kMinPulse) + norm * static_cast<float>(kMaxPulse - kMinPulse);
  return static_cast<int>(std::lround(pulse));
}

void apply_imu(float pitch, float roll) {
  std::lock_guard<std::mutex> lock(g_servoMutex);
  if (!g_servoReady || !g_pca) {
    return;
  }

  pitch = clampf(pitch, -60.0f, 60.0f);
  roll = clampf(roll, -60.0f, 60.0f);
  g_filteredPitch = (1.0f - kFilterAlpha) * g_filteredPitch + kFilterAlpha * pitch;
  g_filteredRoll = (1.0f - kFilterAlpha) * g_filteredRoll + kFilterAlpha * roll;

  int tiltPulse = angle_to_pulse(g_filteredPitch);
  int panPulse = angle_to_pulse(g_filteredRoll);
  g_pca->set_channel_pulse_us(kTiltChannel, tiltPulse);
  g_pca->set_channel_pulse_us(kPanChannel, panPulse);
}

void stop_servo() {
  std::lock_guard<std::mutex> lock(g_servoMutex);
  if (!g_servoReady || !g_pca) {
    return;
  }
  g_pca->set_channel_pulse_us(kPanChannel, 1500);
  g_pca->set_channel_pulse_us(kTiltChannel, 1500);
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
               << "," << (g_servoReady ? 1 : 0) << "\n";
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
  } else if (cap.open(camIndex, cv::CAP_V4L2)) {
    std::cout << "[INFO] Camera opened with V4L2 backend.\n";
  } else if (cap.open(camIndex)) {
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
      } else {
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

}  // namespace

int main() {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  g_pca = std::make_unique<PCA9685Controller>(kPcaBus, kPcaAddress);
  g_servoReady = g_pca->init();
  if (!g_servoReady) {
    std::cerr << "[WARN] PCA9685 init failed. IMU control will be disabled.\n";
  } else {
    std::cout << "[INFO] PCA9685 ready. pan_ch=" << kPanChannel << " tilt_ch=" << kTiltChannel
              << "\n";
    g_pca->set_channel_pulse_us(kPanChannel, 1500);
    g_pca->set_channel_pulse_us(kTiltChannel, 1500);
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
        {"servo_ready", g_servoReady},
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
