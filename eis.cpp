/**
 * eis_opus_test.cpp — 하이브리드 EIS (LK OptFlow + 동적 칼만 + 자이로 고주파 부스트)
 *
 * 보정 원리 (2단계):
 *   1단계: LK Optical Flow + 동적 칼만 필터 → prev_frame에 smoothed 변환 적용
 *          - 큰 움직임: Q↑ R↓ (빠르게 따라감 → 여백 최소화)
 *          - 작은 떨림: Q↓ R↑ (강하게 평활 → 부드러운 영상)
 *          - 시그모이드 블렌딩으로 부드러운 전환
 *   2단계: 자이로 고주파 필터 → 1단계 결과에 추가 회전 보정
 *          (30fps 사이의 고주파 진동만 잡음)
 *
 * RTSP 출력:
 *   /raw  — 원본 영상
 *   /cam  — EIS 보정 영상
 */

#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/video/video.h>
#include <glib.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <dirent.h>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <termios.h>

#include <linux/i2c-dev.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

#include <softPwm.h>
#include <wiringPi.h>

using namespace std;
using namespace cv;

// ======================== 설정 상수 ========================

static const int G_WIDTH = 640;
static const int G_HEIGHT = 480;
static const int G_FPS = 24;
static const bool FLIP_IMAGE = true;
static const int FLIP_MODE = 0; // 0: vertical, 1: horizontal, -1: both

// MPU-6050
static const int IMU_ADDR = 0x68;
static const double GYRO_SENSITIVITY = 131.0;

// ---- 자이로 고주파 필터 (2단계: 추가 회전 보정) ----
static const double SMOOTH_ALPHA = 0.99;
static const double ROLL_GAIN = 1.0;
static const double PITCH_GAIN = 0.5;
static const double MAX_ROLL_RAD = 8.0 * CV_PI / 180.0;  // 5→8도 (여유 확대)
static const double MAX_PITCH_RAD = 5.0 * CV_PI / 180.0; // 3→5도 (여유 확대)

// 적응형 Alpha
static const double ADAPT_THRESHOLD = 5.0 * CV_PI / 180.0;
static const double ADAPT_RATE = 15.0;
static const double ADAPT_MIN_ALPHA = 0.80;

// ---- LK Optical Flow (1단계: 메인 보정) ----
static const int LK_MAX_FEATURES = 200;
static const double LK_QUALITY = 0.01;
static const double LK_MIN_DIST = 30.0;

// ---- 칼만 필터 (레퍼런스 고정값) ----
static const double KF_Q = 0.02; // 레퍼런스 원본
static const double KF_R = 0.5;  // 레퍼런스 원본
// kalman_diff 클램프 (최대 warp 제한 → 여백 방지)
static const double MAX_DIFF_DX = 9999.0;                // pixels
static const double MAX_DIFF_DY = 9999.0;                // pixels
static const double MAX_DIFF_DA = 999.0 * CV_PI / 180.0; // radians (~4도)

// 크롭
static const double FIXED_CROP_PERCENT = 20.0; // 20→15% (화각 확보)

// 카메라 FOV
static const double HFOV_DEG = 62.2;
static const double VFOV_DEG = 48.8;

// IMU 축 매핑
static const int IMU_AXIS_ROLL = 0;
static const int IMU_AXIS_PITCH = 1;
static const int IMU_SIGN_ROLL = 1;
static const int IMU_SIGN_PITCH = 1;
static const int CALIB_SAMPLES = 300;
static const bool DEBUG_OVERLAY = false;
static const int IMU_BUFFER_SIZE = 20; // 1KHz × 40 = ~40ms ≈ 1프레임(33ms) + 여유
static const int IMU_STORE_SIZE = 80;  // 중심 평균을 위해 여유 있게 저장

// ======================== 전역 변수 ========================

static std::atomic<bool> g_running{true};
static GMainLoop* g_main_loop = nullptr;

static void request_shutdown() {
    g_running.store(false);
}

static gboolean on_mainloop_tick(gpointer) {
    if (!g_running.load()) {
        if (g_main_loop) g_main_loop_quit(g_main_loop);
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

static std::mutex g_mtx;
static GstAppSrc* g_rawsrc = nullptr;
static GstAppSrc* g_stabsrc = nullptr;

static std::mutex g_imu_mtx;
static std::atomic<bool> g_imu_ready{false};

struct ImuPose {
    double roll, pitch;
    double gyro_roll_rate, gyro_pitch_rate;
    double timestamp_ms; // steady_clock 기준 ms
};

// IMU 링버퍼: 최근 샘플 저장 (타임스탬프 포함)
static std::deque<ImuPose> g_imu_buffer;
static std::atomic<double> g_imu_actual_hz{0}; // 실측 샘플링 레이트 (atomic → lock 불필요)

// 프로그램 시작 시점 기준 ms 반환
static auto g_time_origin = std::chrono::steady_clock::now();
static double now_ms() {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - g_time_origin)
        .count();
}

// 프레임 중심 N/2 평균: frame_time_ms 기준 ±half_window_ms 범위의 샘플 평균
struct ImuAverageResult {
    ImuPose pose;
    int sample_count;        // 평균에 사용된 샘플 수
    double time_spread_ms;   // 사용된 샘플들의 시간 범위
    double center_offset_ms; // 실제 중심과 프레임 시간의 차이
};

static ImuAverageResult imu_average_centered(double frame_time_ms) {
    // g_imu_mtx 잠긴 상태에서 호출
    ImuAverageResult result = {{0, 0, 0, 0, 0}, 0, 0, 0};
    if (g_imu_buffer.empty()) return result;

    // 프레임 직전 1프레임 간격(~33ms) 윈도우 (지연 없음)
    const double frame_interval_ms = 1000.0 / G_FPS;
    double t_lo = frame_time_ms - frame_interval_ms;
    double t_hi = frame_time_ms;

    double r = 0, p = 0, rr = 0, pr = 0;
    double t_min = 1e18, t_max = -1e18;
    int cnt = 0;

    for (const auto& s : g_imu_buffer) {
        if (s.timestamp_ms >= t_lo && s.timestamp_ms <= t_hi) {
            r += s.roll;
            p += s.pitch;
            rr += s.gyro_roll_rate;
            pr += s.gyro_pitch_rate;
            t_min = std::min(t_min, s.timestamp_ms);
            t_max = std::max(t_max, s.timestamp_ms);
            cnt++;
        }
    }

    // 범위 내 샘플이 없으면 가장 최근 샘플 사용 (fallback)
    if (cnt == 0) {
        const auto& last = g_imu_buffer.back();
        result.pose = last;
        result.sample_count = 1;
        result.center_offset_ms = frame_time_ms - last.timestamp_ms;
        return result;
    }

    double n = (double)cnt;
    result.pose = {r / n, p / n, rr / n, pr / n, frame_time_ms};
    result.sample_count = cnt;
    result.time_spread_ms = (cnt > 1) ? (t_max - t_min) : 0;
    result.center_offset_ms = frame_time_ms - (t_min + t_max) / 2.0;
    return result;
}

// 자이로 고주파 필터 상태
struct HighPassState {
    double smooth_roll, smooth_pitch;
    bool initialized;
};

// 칼만 필터 (1D) — 레퍼런스 고정값 + diff 클램핑
struct KalmanState {
    double x, P, Q, R, sum;
};
static void kalman_init(KalmanState& kf, double q, double r) {
    kf.x = 0;
    kf.P = 1;
    kf.Q = q;
    kf.R = r;
    kf.sum = 0;
}
static void kalman_update(KalmanState& kf, double measurement) {
    kf.sum += measurement;
    double P_pred = kf.P + kf.Q;
    double K = P_pred / (P_pred + kf.R);
    kf.x = kf.x + K * (kf.sum - kf.x);
    kf.P = (1.0 - K) * P_pred;
}
static double kalman_diff(const KalmanState& kf, double max_diff) {
    double diff = kf.x - kf.sum;
    return std::clamp(diff, -max_diff, max_diff);
}

// ======================== 유틸리티 함수 ========================

static double pick_axis(int axis, double x, double y, double z) {
    switch (axis) {
    case 0:
        return x;
    case 1:
        return y;
    case 2:
        return z;
    default:
        return z;
    }
}
static int16_t to_i16(uint8_t h, uint8_t l) {
    return (int16_t)((h << 8) | l);
}

static double adaptiveAlpha(double base_alpha, double diff,
                            double threshold, double rate, double min_alpha) {
    if (std::abs(diff) <= threshold) return base_alpha;
    double excess = std::abs(diff) - threshold;
    return std::max(base_alpha - excess * rate, min_alpha);
}

static bool i2c_write_byte(int fd, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return (write(fd, buf, 2) == 2);
}
static bool i2c_read_bytes(int fd, uint8_t reg, uint8_t* data, int len) {
    if (write(fd, &reg, 1) != 1) return false;
    return (read(fd, data, len) == len);
}

static Mat ensureBGR(const Mat& in) {
    if (in.empty()) return Mat();
    Mat out = in;
    if (out.type() == CV_8UC4)
        cvtColor(out, out, COLOR_BGRA2BGR);
    else if (out.type() == CV_8UC1)
        cvtColor(out, out, COLOR_GRAY2BGR);
    else if (out.type() != CV_8UC3) {
        Mat tmp;
        out.convertTo(tmp, CV_8U);
        if (tmp.channels() == 1)
            cvtColor(tmp, out, COLOR_GRAY2BGR);
        else
            out = tmp;
    }
    if (out.cols != G_WIDTH || out.rows != G_HEIGHT)
        resize(out, out, Size(G_WIDTH, G_HEIGHT), 0, 0, INTER_LINEAR);
    if (!out.isContinuous()) out = out.clone();
    return out;
}

static Mat centerCropAndResize(const Mat& in, double cropPercent) {
    if (in.empty() || cropPercent <= 0) return in;
    int cw = (int)(in.cols * cropPercent / 200.0);
    int ch = (int)(in.rows * cropPercent / 200.0);
    Rect roi(cw, ch, in.cols - 2 * cw, in.rows - 2 * ch);
    Mat out;
    resize(in(roi), out, in.size(), 0, 0, INTER_LINEAR);
    return out;
}

// ======================== RTSP 설정 ========================

static void set_appsrc_caps(GstAppSrc* appsrc) {
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
                                        "format", G_TYPE_STRING, "BGR",
                                        "width", G_TYPE_INT, G_WIDTH,
                                        "height", G_TYPE_INT, G_HEIGHT,
                                        "framerate", GST_TYPE_FRACTION, G_FPS, 1, nullptr);
    gst_app_src_set_caps(appsrc, caps);
    gst_caps_unref(caps);
    g_object_set(G_OBJECT(appsrc),
                 "is-live", TRUE, "format", GST_FORMAT_TIME,
                 "do-timestamp", TRUE, "block", FALSE, nullptr);
}

static void on_media_configure(GstRTSPMediaFactory*, GstRTSPMedia* media, gpointer user_data) {
    const char* which = static_cast<const char*>(user_data);
    GstElement* element = gst_rtsp_media_get_element(media);
    const char* name = (strcmp(which, "raw") == 0) ? "rawsrc" : "stabsrc";
    GstElement* app = gst_bin_get_by_name_recurse_up(GST_BIN(element), name);
    gst_object_unref(element);
    if (!app) {
        g_printerr("Failed to get appsrc: %s\n", name);
        return;
    }
    GstAppSrc* appsrc = GST_APP_SRC(app);
    set_appsrc_caps(appsrc);
    {
        std::lock_guard<std::mutex> lk(g_mtx);
        if (strcmp(which, "raw") == 0) {
            g_rawsrc = appsrc;
            g_print("[RTSP] /raw connected\n");
        }
        else {
            g_stabsrc = appsrc;
            g_print("[RTSP] /cam connected\n");
        }
    }
}

static GstRTSPMediaFactory* make_factory(const char* appsrc_name) {
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new();
    std::string launch =
        "( appsrc name=" + std::string(appsrc_name) + " "
                                                      "is-live=true format=time do-timestamp=true block=false "
                                                      "! queue leaky=downstream max-size-buffers=1 max-size-time=0 max-size-bytes=0 "
                                                      "! videoconvert ! video/x-raw,format=I420 "
                                                      "! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 key-int-max=" +
        std::to_string(G_FPS) + " bframes=0 "
                                "! rtph264pay name=pay0 pt=96 config-interval=1 )";
    gst_rtsp_media_factory_set_launch(factory, launch.c_str());
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    gst_rtsp_media_factory_set_suspend_mode(factory, GST_RTSP_SUSPEND_MODE_NONE);
    gst_rtsp_media_factory_set_latency(factory, 0);
    return factory;
}

static bool push_bgr(GstAppSrc* appsrc, const Mat& frame, guint64 idx, const char* tag) {
    if (!appsrc) return false;
    Mat bgr = ensureBGR(frame);
    if (bgr.empty()) return false;
    const size_t bytes = bgr.total() * bgr.elemSize();
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, bytes, nullptr);
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        gst_buffer_unref(buffer);
        return false;
    }
    memcpy(map.data, bgr.data, bytes);
    gst_buffer_unmap(buffer, &map);
    GstFlowReturn ret = gst_app_src_push_buffer(appsrc, buffer);
    if (ret != GST_FLOW_OK) {
        g_printerr("[push:%s] failed\n", tag);
        return false;
    }
    return true;
}

static void sigint_handler(int) { request_shutdown(); }

// ======================== Manual Drive (Keyboard) ========================

namespace manual_drive {
constexpr int STOP = 0;
constexpr int FORWARD = 1;
constexpr int BACKWARD = 2;

// wiringPi pin numbers (same as patrol_track / rc_control_node)
constexpr int L_IN1 = 28;
constexpr int L_IN2 = 27;
constexpr int L_EN = 29;

constexpr int R_IN1 = 25;
constexpr int R_IN2 = 24;
constexpr int R_EN = 23;

termios g_old_tio{};
bool g_term_ready = false;
bool g_motor_ready = false;

int clampPwm(int v) {
    return std::max(0, std::min(255, v));
}

void setMotorControl(int en, int in1, int in2, int pwm, int dir) {
    if (!g_motor_ready) return;
    pwm = clampPwm(pwm);
    softPwmWrite(en, pwm);

    if (dir == FORWARD) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else if (dir == BACKWARD) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        softPwmWrite(en, 0);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
}

void stopAll() {
    if (!g_motor_ready) return;
    setMotorControl(L_EN, L_IN1, L_IN2, 0, STOP);
    setMotorControl(R_EN, R_IN1, R_IN2, 0, STOP);
}

void applyDrive(int left_cmd, int right_cmd, int pwm) {
    const int ldir = (left_cmd > 0) ? FORWARD : (left_cmd < 0 ? BACKWARD : STOP);
    const int rdir = (right_cmd > 0) ? FORWARD : (right_cmd < 0 ? BACKWARD : STOP);

    setMotorControl(L_EN, L_IN1, L_IN2, (ldir == STOP) ? 0 : pwm, ldir);
    setMotorControl(R_EN, R_IN1, R_IN2, (rdir == STOP) ? 0 : pwm, rdir);
}

void cleanupTerminal() {
    if (g_term_ready) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_old_tio);
        g_term_ready = false;
    }
}

bool setupTerminalRaw() {
    if (!isatty(STDIN_FILENO)) {
        fprintf(stderr, "[MANUAL] stdin is not a TTY; manual drive disabled.\n");
        return false;
    }
    if (tcgetattr(STDIN_FILENO, &g_old_tio) != 0) return false;
    termios new_tio = g_old_tio;
    new_tio.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_tio) != 0) return false;
    g_term_ready = true;
    return true;
}

bool setupMotor() {
    if (wiringPiSetup() == -1) {
        fprintf(stderr, "[MANUAL] wiringPiSetup failed; motor control disabled.\n");
        g_motor_ready = false;
        return false;
    }

    auto setupOne = [](int en, int in1, int in2) -> bool {
        pinMode(en, OUTPUT);
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        return (softPwmCreate(en, 0, 255) == 0);
    };

    const bool left_ok = setupOne(L_EN, L_IN1, L_IN2);
    const bool right_ok = setupOne(R_EN, R_IN1, R_IN2);
    g_motor_ready = left_ok && right_ok;

    if (!g_motor_ready) {
        fprintf(stderr, "[MANUAL] softPwmCreate failed; motor control disabled.\n");
        return false;
    }
    stopAll();
    return true;
}

void printHelp() {
    fprintf(stderr, "\n=== Manual Drive (Keyboard) ===\n");
    fprintf(stderr, "W/S (or ↑/↓): forward/backward\n");
    fprintf(stderr, "A/D (or ←/→): rotate left/right (tank spin)\n");
    fprintf(stderr, "Q/E: pivot left/right\n");
    fprintf(stderr, "Space/X: stop\n");
    fprintf(stderr, "+/-: speed up/down\n");
    fprintf(stderr, "H: help, ESC: quit\n");
    fprintf(stderr, "===============================\n\n");
}

bool run_evdev(const char* dev_path) {
    int fd = open(dev_path, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        perror("[MANUAL] evdev open");
        return false;
    }

    fprintf(stderr, "[MANUAL] evdev input: %s\n", dev_path);

    int pwm = 255;
    int left_cmd = 0;
    int right_cmd = 0;

    auto applyAction = [&](int action_key) {
        switch (action_key) {
            case KEY_W:
            case KEY_UP:
                pwm = 255;
                left_cmd = 1; right_cmd = 1;
                break;
            case KEY_S:
            case KEY_DOWN:
                pwm = 255;
                left_cmd = -1; right_cmd = -1;
                break;
            case KEY_A:
            case KEY_LEFT:
                pwm = 255;
                left_cmd = -1; right_cmd = 1;
                break;
            case KEY_D:
            case KEY_RIGHT:
                pwm = 255;
                left_cmd = 1; right_cmd = -1;
                break;
            case KEY_Q:
                pwm = 255;
                left_cmd = 0; right_cmd = 1;
                break;
            case KEY_E:
                pwm = 255;
                left_cmd = 1; right_cmd = 0;
                break;
            default:
                left_cmd = 0; right_cmd = 0;
                break;
        }
        applyDrive(left_cmd, right_cmd, pwm);
        fprintf(stderr, "[MANUAL] L=%d R=%d PWM=%d\n", left_cmd, right_cmd, pwm);
        fflush(stderr);
    };

    stopAll();
    printHelp();
    fprintf(stderr, "[MANUAL] start PWM=%d (evdev)\n", pwm);

    int active_key = 0;

    while (g_running.load()) {
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;
        pfd.revents = 0;

        const int pr = poll(&pfd, 1, 10);
        if (pr > 0 && (pfd.revents & POLLIN)) {
            struct input_event ev;
            ssize_t n = read(fd, &ev, sizeof(ev));
            while (n == sizeof(ev)) {
                if (ev.type == EV_KEY) {
                    const bool pressed = (ev.value != 0);
                    const int code = ev.code;

                    if (pressed) {
                        if (code == KEY_ESC) {
                            request_shutdown();
                            break;
                        }
                        if (code == KEY_SPACE || code == KEY_X) {
                            active_key = 0;
                            applyAction(0);
                        } else if (code == KEY_EQUAL || code == KEY_KPPLUS) {
                            pwm = clampPwm(pwm + 10);
                            fprintf(stderr, "[MANUAL] PWM %d\n", pwm);
                        } else if (code == KEY_MINUS || code == KEY_KPMINUS) {
                            pwm = clampPwm(pwm - 10);
                            fprintf(stderr, "[MANUAL] PWM %d\n", pwm);
                        } else if (code == KEY_H) {
                            printHelp();
                        } else {
                            // movement key pressed
                            active_key = code;
                            applyAction(active_key);
                        }
                    } else {
                        // key released -> stop if it was the active movement key
                        if (code == active_key) {
                            active_key = 0;
                            applyAction(0);
                        }
                    }
                }
                n = read(fd, &ev, sizeof(ev));
            }
        }
    }

    stopAll();
    fprintf(stderr, "[MANUAL] stopped (evdev)\n");
    close(fd);
    return true;
}

std::string find_kbd_device() {
    const char* dirs[] = {"/dev/input/by-id", "/dev/input/by-path"};
    for (const char* dir_path : dirs) {
        DIR* dir = opendir(dir_path);
        if (!dir) continue;
        while (true) {
            struct dirent* ent = readdir(dir);
            if (!ent) break;
            const std::string name = ent->d_name ? ent->d_name : "";
            if (name.find("event-kbd") == std::string::npos) continue;
            const std::string full = std::string(dir_path) + "/" + name;
            closedir(dir);
            return full;
        }
        closedir(dir);
    }
    return {};
}

void run() {
    if (!setupMotor()) return;

    const char* evdev = std::getenv("EIS_INPUT_EVENT");
    if (evdev && evdev[0] != '\0') {
        if (run_evdev(evdev)) return;
        fprintf(stderr, "[MANUAL] evdev failed, falling back to stdin.\n");
    } else {
        std::string auto_dev = find_kbd_device();
        if (!auto_dev.empty()) {
            if (run_evdev(auto_dev.c_str())) return;
            fprintf(stderr, "[MANUAL] auto evdev failed, falling back to stdin.\n");
        }
    }

    if (!setupTerminalRaw()) {
        stopAll();
        return;
    }

    int pwm = 255;
    int left_cmd = 0;
    int right_cmd = 0;
    auto last_motion_key_time = std::chrono::steady_clock::now();
    int motion_hold_timeout_ms = 80;
    if (const char* env = std::getenv("MANUAL_HOLD_MS")) {
        const int v = std::atoi(env);
        if (v >= 20 && v <= 1000) motion_hold_timeout_ms = v;
    }

    stopAll();
    printHelp();
    fprintf(stderr, "[MANUAL] start PWM=%d\n", pwm);

    while (g_running.load()) {
        char ch = 0;
        const ssize_t n = read(STDIN_FILENO, &ch, 1);
        if (n > 0) {
            if (ch >= 'A' && ch <= 'Z') ch = static_cast<char>(ch - 'A' + 'a');

            if (ch == 27) { // ESC or arrow key sequence
                char seq1 = 0;
                char seq2 = 0;
                const ssize_t n1 = read(STDIN_FILENO, &seq1, 1);
                if (n1 == 1 && seq1 == '[') {
                    const ssize_t n2 = read(STDIN_FILENO, &seq2, 1);
                    if (n2 == 1) {
                        switch (seq2) {
                            case 'A': ch = 'w'; break; // Up
                            case 'B': ch = 's'; break; // Down
                            case 'C': ch = 'd'; break; // Right
                            case 'D': ch = 'a'; break; // Left
                            default: ch = 27; break;
                        }
                    }
                }
                if (ch == 27) {
                    request_shutdown();
                    break;
                }
            }

            switch (ch) {
                case 'w':
                    pwm = 255;
                    left_cmd = 1; right_cmd = 1;
                    last_motion_key_time = std::chrono::steady_clock::now();
                    break;
                case 's':
                    pwm = 255;
                    left_cmd = -1; right_cmd = -1;
                    last_motion_key_time = std::chrono::steady_clock::now();
                    break;
                case 'a':
                    pwm = 255;
                    left_cmd = -1; right_cmd = 1;
                    last_motion_key_time = std::chrono::steady_clock::now();
                    break;
                case 'd':
                    pwm = 255;
                    left_cmd = 1; right_cmd = -1;
                    last_motion_key_time = std::chrono::steady_clock::now();
                    break;
                case 'q':
                    pwm = 255;
                    left_cmd = 0; right_cmd = 1;
                    last_motion_key_time = std::chrono::steady_clock::now();
                    break;
                case 'e':
                    pwm = 255;
                    left_cmd = 1; right_cmd = 0;
                    last_motion_key_time = std::chrono::steady_clock::now();
                    break;
                case 'x':
                case ' ':
                    left_cmd = 0; right_cmd = 0;
                    break;
                case '+':
                case '=':
                    pwm = clampPwm(pwm + 10);
                    fprintf(stderr, "[MANUAL] PWM %d\n", pwm);
                    break;
                case '-':
                case '_':
                    pwm = clampPwm(pwm - 10);
                    fprintf(stderr, "[MANUAL] PWM %d\n", pwm);
                    break;
                case 'h':
                    printHelp();
                    break;
                default:
                    break;
            }

            applyDrive(left_cmd, right_cmd, pwm);
            fprintf(stderr, "[MANUAL] L=%d R=%d PWM=%d\n", left_cmd, right_cmd, pwm);
            fflush(stderr);
        }

        const auto now = std::chrono::steady_clock::now();
        const auto idle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_motion_key_time).count();
        if ((left_cmd != 0 || right_cmd != 0) && idle_ms > motion_hold_timeout_ms) {
            left_cmd = 0;
            right_cmd = 0;
            applyDrive(left_cmd, right_cmd, pwm);
            fprintf(stderr, "[MANUAL] auto-stop (key released)\n");
            fflush(stderr);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    stopAll();
    cleanupTerminal();
    fprintf(stderr, "[MANUAL] stopped\n");
}
} // namespace manual_drive

// ======================== IMU 쓰레드 ========================

static void imu_loop() {
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        perror("[IMU] open");
        return;
    }
    if (ioctl(fd, I2C_SLAVE, IMU_ADDR) < 0) {
        perror("[IMU] ioctl");
        close(fd);
        return;
    }

    i2c_write_byte(fd, 0x6B, 0x00);
    i2c_write_byte(fd, 0x1B, 0x00);
    i2c_write_byte(fd, 0x1A, 0x03);
    usleep(100000);

    fprintf(stderr, "[IMU] Calibrating gyro (%d samples)...\n", CALIB_SAMPLES);
    double bias_gx = 0, bias_gy = 0, bias_gz = 0;
    int valid = 0;
    for (int i = 0; i < CALIB_SAMPLES; ++i) {
        uint8_t buf[6];
        if (!i2c_read_bytes(fd, 0x43, buf, 6)) continue;
        bias_gx += to_i16(buf[0], buf[1]);
        bias_gy += to_i16(buf[2], buf[3]);
        bias_gz += to_i16(buf[4], buf[5]);
        valid++;
        usleep(2000);
    }
    if (valid > 0) {
        bias_gx /= valid;
        bias_gy /= valid;
        bias_gz /= valid;
    }
    fprintf(stderr, "[IMU] Calibration done (bias: gx=%.1f gy=%.1f gz=%.1f)\n", bias_gx, bias_gy, bias_gz);

    double gyro_roll = 0, gyro_pitch = 0;
    g_imu_ready = true;
    fprintf(stderr, "[IMU] Ready (target: 1KHz, buffer: %d samples)\n", IMU_BUFFER_SIZE);

    auto last_time = std::chrono::steady_clock::now();

    while (g_running) {
        uint8_t buf[6];
        if (!i2c_read_bytes(fd, 0x43, buf, 6)) {
            usleep(500);
            continue;
        }
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;
        if (dt <= 0 || dt > 0.1) {
            usleep(500);
            continue;
        }

        double gx_rad = ((to_i16(buf[0], buf[1]) - bias_gx) / GYRO_SENSITIVITY) * (CV_PI / 180.0);
        double gy_rad = ((to_i16(buf[2], buf[3]) - bias_gy) / GYRO_SENSITIVITY) * (CV_PI / 180.0);
        double gz_rad = ((to_i16(buf[4], buf[5]) - bias_gz) / GYRO_SENSITIVITY) * (CV_PI / 180.0);

        double roll_rate = IMU_SIGN_ROLL * pick_axis(IMU_AXIS_ROLL, gx_rad, gy_rad, gz_rad);
        double pitch_rate = IMU_SIGN_PITCH * pick_axis(IMU_AXIS_PITCH, gx_rad, gy_rad, gz_rad);
        gyro_roll += roll_rate * dt;
        gyro_pitch += pitch_rate * dt;

        // 샘플 준비 (lock 밖에서)
        double ts = now_ms();
        ImuPose sample = {gyro_roll, gyro_pitch, roll_rate, pitch_rate, ts};

        { // 최소 임계구간: push + pop만
            std::lock_guard<std::mutex> lk(g_imu_mtx);
            g_imu_buffer.push_back(sample);
            if ((int)g_imu_buffer.size() > IMU_STORE_SIZE)
                g_imu_buffer.pop_front();
        }
        usleep(5000); // 200Hz (Pi CPU 부하 고려)
    }
    close(fd);
    fprintf(stderr, "[IMU] Thread exiting\n");
}

// ======================== 캡처 + EIS 보정 루프 ========================

static void capture_loop() {
    std::string cap_pipe =
        "libcamerasrc "
        "! video/x-raw,format=RGBx,width=640,height=480,framerate=24/1 "
        "! videoconvert ! video/x-raw,format=BGR "
        "! appsink name=appsink drop=true max-buffers=1 sync=false";

    GError* err = nullptr;
    GstElement* pipeline = gst_parse_launch(cap_pipe.c_str(), &err);
    if (!pipeline) {
        cerr << "[ERR] pipeline\n";
        if (err) g_error_free(err);
        g_running = false;
        return;
    }
    if (err) {
        cerr << "[WARN] " << err->message << "\n";
        g_error_free(err);
    }

    GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink");
    if (!sink) {
        cerr << "[ERR] appsink\n";
        gst_object_unref(pipeline);
        g_running = false;
        return;
    }
    GstAppSink* appsink = GST_APP_SINK(sink);
    gst_app_sink_set_emit_signals(appsink, FALSE);
    gst_app_sink_set_drop(appsink, TRUE);
    gst_app_sink_set_max_buffers(appsink, 1);

    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        cerr << "[ERR] pipeline start\n";
        gst_object_unref(sink);
        gst_object_unref(pipeline);
        g_running = false;
        return;
    }

    auto pull_frame = [&](Mat& out) -> bool {
        GstSample* sample = gst_app_sink_try_pull_sample(appsink, 100000000);
        if (!sample) return false;
        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstCaps* caps = gst_sample_get_caps(sample);
        if (!buffer || !caps) {
            gst_sample_unref(sample);
            return false;
        }
        GstVideoInfo info;
        bool info_ok = gst_video_info_from_caps(&info, caps);
        int w = info_ok ? (int)info.width : G_WIDTH;
        int h = info_ok ? (int)info.height : G_HEIGHT;
        int stride = info_ok ? (int)info.stride[0] : (w * 3);
        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return false;
        }
        Mat frame(h, w, CV_8UC3, map.data, stride);
        out = frame.clone();
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
        return !out.empty();
    };

    const double cx_cam = G_WIDTH * 0.5;
    const double cy_cam = G_HEIGHT * 0.5;
    const double fy = G_HEIGHT / (2.0 * tan(VFOV_DEG * CV_PI / 360.0));

    guint64 frameIdx = 0;

    // 자이로 고주파 필터
    HighPassState hp = {0, 0, false};

    // LK + 고정 칼만 (레퍼런스 방식)
    Mat prev_gray, prev_frame;
    int lk_count = 0;
    KalmanState kf_theta, kf_tx, kf_ty;
    kalman_init(kf_theta, KF_Q, KF_R);
    kalman_init(kf_tx, KF_Q, KF_R);
    kalman_init(kf_ty, KF_Q, KF_R);

    while (g_running) {
        Mat frame;
        if (!pull_frame(frame) || frame.empty()) continue;
        if (FLIP_IMAGE) {
            cv::flip(frame, frame, FLIP_MODE);
        }

        // 프레임 캡처 시점 타임스탬프
        double frame_time = now_ms();

        // lock 안에서 직접 평균 (디큐 복사 없음, ~80개 순회는 수 μs)
        ImuAverageResult imu_result;
        bool imu_ready;
        {
            std::lock_guard<std::mutex> lk(g_imu_mtx);
            imu_result = imu_average_centered(frame_time);
            imu_ready = g_imu_ready.load();
        }
        double actual_hz = g_imu_actual_hz.load();
        ImuPose pose = imu_result.pose;

        Mat stabilized;
        double jitter_roll = 0, jitter_pitch = 0;
        double lk_diff_dx = 0, lk_diff_dy = 0, lk_diff_da = 0;

        Mat curr_gray;
        cvtColor(frame, curr_gray, COLOR_BGR2GRAY);

        // ============================================================
        // 1단계: LK OptFlow + 칼만 → prev_frame에 smoothed 변환 적용
        //        (레퍼런스 고정 Q/R + diff 클램핑)
        // ============================================================

        if (lk_count == 0) {
            // 첫 프레임: 저장만
            stabilized = frame.clone();
            prev_gray = std::move(curr_gray);
            prev_frame = std::move(frame);
            lk_count++;
        }
        else {
            // 특징점 검출 + 추적
            vector<Point2f> feat_prev, feat_curr;
            goodFeaturesToTrack(prev_gray, feat_prev, LK_MAX_FEATURES, LK_QUALITY, LK_MIN_DIST);

            bool lk_ok = false;
            double sx = 1, sy = 1;

            if (feat_prev.size() >= 10) {
                vector<uchar> status;
                vector<float> err_vec;
                calcOpticalFlowPyrLK(prev_gray, curr_gray, feat_prev, feat_curr, status, err_vec);

                vector<Point2f> gp, gc;
                for (size_t i = 0; i < status.size(); i++) {
                    if (status[i]) {
                        gp.push_back(feat_prev[i]);
                        gc.push_back(feat_curr[i]);
                    }
                }

                if (gp.size() >= 6) {
                    Mat affine = estimateAffinePartial2D(gp, gc);
                    if (!affine.empty()) {
                        double dx = affine.at<double>(0, 2);
                        double dy = affine.at<double>(1, 2);
                        double da = atan2(affine.at<double>(1, 0), affine.at<double>(0, 0));
                        sx = affine.at<double>(0, 0) / cos(da);
                        sy = affine.at<double>(1, 1) / cos(da);

                        kalman_update(kf_theta, da);
                        kalman_update(kf_tx, dx);
                        kalman_update(kf_ty, dy);

                        if (lk_count >= 2) {
                            lk_diff_da = kalman_diff(kf_theta, MAX_DIFF_DA);
                            lk_diff_dx = kalman_diff(kf_tx, MAX_DIFF_DX);
                            lk_diff_dy = kalman_diff(kf_ty, MAX_DIFF_DY);
                        }
                        lk_count++;

                        da += lk_diff_da;
                        dx += lk_diff_dx;
                        dy += lk_diff_dy;

                        Mat smoothed = (Mat_<double>(2, 3) << sx * cos(da), sx * -sin(da), dx,
                                        sy * sin(da), sy * cos(da), dy);

                        // 2단계: 자이로 보정 행렬을 LK 행렬과 통합 (warpAffine 1회만 호출)
                        if (imu_ready) {
                            if (!hp.initialized) {
                                hp.smooth_roll = pose.roll;
                                hp.smooth_pitch = pose.pitch;
                                hp.initialized = true;
                            }
                            else {
                                double diff_r = pose.roll - hp.smooth_roll;
                                double diff_p = pose.pitch - hp.smooth_pitch;
                                double alpha_r = adaptiveAlpha(SMOOTH_ALPHA, diff_r, ADAPT_THRESHOLD, ADAPT_RATE, ADAPT_MIN_ALPHA);
                                double alpha_p = adaptiveAlpha(SMOOTH_ALPHA, diff_p, ADAPT_THRESHOLD, ADAPT_RATE, ADAPT_MIN_ALPHA);
                                hp.smooth_roll = alpha_r * hp.smooth_roll + (1.0 - alpha_r) * pose.roll;
                                hp.smooth_pitch = alpha_p * hp.smooth_pitch + (1.0 - alpha_p) * pose.pitch;
                            }

                            jitter_roll = pose.roll - hp.smooth_roll;
                            jitter_pitch = pose.pitch - hp.smooth_pitch;

                            double roll_corr = -std::clamp(jitter_roll * ROLL_GAIN, -MAX_ROLL_RAD, MAX_ROLL_RAD);
                            double pitch_corr = -std::clamp(jitter_pitch * PITCH_GAIN, -MAX_PITCH_RAD, MAX_PITCH_RAD);

                            if (std::abs(jitter_roll) > 0.02 * CV_PI / 180.0 ||
                                std::abs(jitter_pitch) > 0.02 * CV_PI / 180.0) {

                                double dy_pitch = fy * pitch_corr;
                                double cos_r = cos(roll_corr);
                                double sin_r = sin(roll_corr);
                                double gtx = (1.0 - cos_r) * cx_cam + sin_r * cy_cam;
                                double gty = -sin_r * cx_cam + (1.0 - cos_r) * cy_cam + dy_pitch;

                                Mat T_gyro = (Mat_<double>(2, 3) << cos_r, -sin_r, gtx,
                                              sin_r, cos_r, gty);

                                // 행렬 통합: T_gyro × smoothed (3x3 확장 후 곱셈, 다시 2x3)
                                Mat S3 = Mat::eye(3, 3, CV_64F);
                                smoothed.copyTo(S3(Rect(0, 0, 3, 2)));
                                Mat G3 = Mat::eye(3, 3, CV_64F);
                                T_gyro.copyTo(G3(Rect(0, 0, 3, 2)));
                                Mat combined3 = G3 * S3;
                                smoothed = combined3(Rect(0, 0, 3, 2)).clone();
                            }
                        }

                        warpAffine(prev_frame, stabilized, smoothed, frame.size());
                        lk_ok = true;
                    }
                }
            }

            if (!lk_ok) {
                stabilized = frame;  // shallow copy — centerCropAndResize가 새 Mat 생성
            }

            prev_gray = std::move(curr_gray);
            prev_frame = std::move(frame);
        }

        // 자이로 보정이 LK 안에서 통합되지 않은 경우 (LK 실패 or 첫 프레임) — 별도 처리
        if (imu_ready && !stabilized.empty() && !hp.initialized) {
            hp.smooth_roll = pose.roll;
            hp.smooth_pitch = pose.pitch;
            hp.initialized = true;
        }
        else if (!imu_ready) {
            hp.initialized = false;
        }

        // 고정 크롭
        stabilized = centerCropAndResize(stabilized, FIXED_CROP_PERCENT);

        // 디버그 오버레이
        if (DEBUG_OVERLAY) {
            char buf[200];
            const int font = FONT_HERSHEY_SIMPLEX;

            if (imu_ready) {
                snprintf(buf, sizeof(buf), "Gyro R:%+5.2f P:%+5.2f | Jit R:%+4.2f P:%+4.2f",
                         pose.roll * 180 / CV_PI, pose.pitch * 180 / CV_PI,
                         jitter_roll * 180 / CV_PI, jitter_pitch * 180 / CV_PI);
                putText(stabilized, buf, Point(8, 18), font, 0.4, Scalar(0, 255, 0), 1, LINE_AA);

                snprintf(buf, sizeof(buf), "LK dx:%+5.1f dy:%+5.1f da:%+4.2f",
                         lk_diff_dx, lk_diff_dy, lk_diff_da * 180 / CV_PI);
                putText(stabilized, buf, Point(8, 33), font, 0.4, Scalar(255, 200, 0), 1, LINE_AA);
            }
            else {
                putText(stabilized, "IMU: NOT READY", Point(8, 18), font, 0.45, Scalar(0, 0, 255), 1, LINE_AA);
            }
        }

        // RTSP 출력
        GstAppSrc *stabsrc;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            stabsrc = g_stabsrc;
        }

        push_bgr(stabsrc, stabilized, frameIdx, "cam");
        frameIdx++;
    }

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(sink);
    gst_object_unref(pipeline);
    fprintf(stderr, "[Capture] Thread exiting\n");
}

// ======================== main ========================

int main(int argc, char* argv[]) {
    system("fuser -k 8555/tcp 2>/dev/null");
    signal(SIGINT, sigint_handler);
    gst_init(&argc, &argv);

    GstRTSPServer* server = gst_rtsp_server_new();
    gst_rtsp_server_set_service(server, "8555");
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);

    // /raw 스트림 비활성화 (CPU 부하 절감 — x264enc 1개 제거)
    // GstRTSPMediaFactory* f_raw = make_factory("rawsrc");
    // g_signal_connect(f_raw, "media-configure", (GCallback)on_media_configure, (gpointer) "raw");
    // gst_rtsp_mount_points_add_factory(mounts, "/raw", f_raw);

    GstRTSPMediaFactory* f_stab = make_factory("stabsrc");
    g_signal_connect(f_stab, "media-configure", (GCallback)on_media_configure, (gpointer) "stab");
    gst_rtsp_mount_points_add_factory(mounts, "/cam", f_stab);

    g_object_unref(mounts);

    if (gst_rtsp_server_attach(server, nullptr) == 0) {
        cerr << "[ERR] RTSP attach\n";
        return -1;
    }

    fprintf(stderr, "==============================\n");
    fprintf(stderr, " EIS — Hybrid (LK+Kalman + Gyro HF boost)\n");
    fprintf(stderr, "==============================\n");
    fprintf(stderr, "  1단계: LK OptFlow + Kalman (Q=%.4f R=%.1f) + diff clamp\n", KF_Q, KF_R);
    fprintf(stderr, "  2단계: Gyro 고주파 부스트 (alpha=%.3f)\n", SMOOTH_ALPHA);
    fprintf(stderr, "  Crop: %.0f%%\n", FIXED_CROP_PERCENT);
    fprintf(stderr, "  rtsp://<PI_IP>:8555/cam  (raw 비활성화)\n");
    fprintf(stderr, "==============================\n");

    g_main_loop = g_main_loop_new(nullptr, FALSE);
    g_timeout_add(50, on_mainloop_tick, nullptr);

    std::thread imu_th(imu_loop);
    std::thread cap_th(capture_loop);
    std::thread manual_th(manual_drive::run);

    g_main_loop_run(g_main_loop);

    g_running = false;
    if (cap_th.joinable()) cap_th.join();
    if (imu_th.joinable()) imu_th.join();
    if (manual_th.joinable()) manual_th.join();
    g_main_loop_unref(g_main_loop);
    g_main_loop = nullptr;
    return 0;
}
