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
#include <gst/gstmeta.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <time.h>
#include <utility>
#include <vector>

#include <poll.h>
#include <termios.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std;
using namespace cv;

// ======================== 설정 상수 ========================

static const int G_WIDTH = 640;
static const int G_HEIGHT = 480;
static const int G_FPS = 20;

// MPU-6050
static const int IMU_ADDR = 0x68;
static const double GYRO_SENSITIVITY = 131.0;

// ---- 자이로 고주파 필터 (2단계: 추가 회전 보정) ----
static const double SMOOTH_ALPHA = 0.99;
static const double ROLL_GAIN = 1.0;
static const double PITCH_GAIN = 0.5;
static const double YAW_GAIN = 1.0;
static const double MAX_YAW_RAD = 10.0 * CV_PI / 180.0;  // 10deg clamp
static const double MAX_ROLL_RAD = 8.0 * CV_PI / 180.0;  // 5→8도 (여유 확대)
static const double MAX_PITCH_RAD = 5.0 * CV_PI / 180.0; // 3→5도 (여유 확대)
static const double IMU_RATE_SMOOTH_ALPHA = 0.90;

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
static const double FIXED_CROP_PERCENT = 5.0;  // min crop percent
static const bool DYNAMIC_CROP = true;
static const int CROP_WINDOW = 10;
static const double CROP_SMOOTH_ALPHA = 0.90;
static const double CROP_MIN_PERCENT = 5.0;
static const double CROP_MAX_PERCENT = 40.0;

// Test environment: camera is vertically flipped
static const bool FLIP_VERTICAL = true;

// 카메라 FOV
static const double HFOV_DEG = 62.2;
static const double VFOV_DEG = 48.8;

// IMU 축 매핑
static const int IMU_AXIS_ROLL = 0;
static const int IMU_AXIS_PITCH = 1;
static const int IMU_AXIS_YAW = 2;
static const int IMU_SIGN_ROLL = 1;
static const int IMU_SIGN_PITCH = 1;
static const int IMU_SIGN_YAW = -1;
static const int CALIB_SAMPLES = 300;
static const bool DEFAULT_DEBUG_OVERLAY = false;
static const int IMU_BUFFER_SIZE = 600; // 200Hz ?? 3? ??
static const int IMU_STORE_SIZE = 800;  // ??????/?? ??

// ??? ??????
static const double OFFSET_CALIB_DURATION_MS = 8000.0;
static const double OFFSET_COARSE_RANGE_MS = 50.0;
static const double OFFSET_COARSE_STEP_MS = 0.5;
static const double OFFSET_FINE_RANGE_MS = 5.0;
static const double OFFSET_FINE_STEP_MS = 0.1;

// IMU ?? ???(??? ??)
static const double IMU_AVG_WINDOW_MS = 20.0;
static const int IMU_AVG_MAX_SAMPLES = 20;

// ======================== 전역 변수 ========================

static std::atomic<bool> g_running{true};

static std::mutex g_mtx;
static GstAppSrc* g_rawsrc = nullptr;
static GstAppSrc* g_stabsrc = nullptr;

static std::mutex g_imu_mtx;
static std::atomic<bool> g_imu_ready{false};

enum class EisMode {
    LK = 0,
    GYRO = 1,
    HYBRID = 2
};

enum class OutputMode {
    BOTH = 0,
    RAW_ONLY = 1,
    CAM_ONLY = 2
};

enum class TsSourcePref {
    AUTO = 0,
    SENSOR = 1,
    PTS = 2,
    ARRIVAL = 3
};

static std::atomic<int> g_mode{(int)EisMode::LK};
static std::atomic<int> g_output_mode{(int)OutputMode::BOTH};
static std::atomic<bool> g_debug_overlay{DEFAULT_DEBUG_OVERLAY};
static std::atomic<int> g_log_every_frames{-1};
static std::atomic<int> g_ts_pref{(int)TsSourcePref::AUTO};
static std::atomic<bool> g_offset_sweep{false};
static double g_manual_imu_offset_ms = 0.0;

struct ImuPose {
    double roll, pitch, yaw; // integrated angles (rad)
    double gyro_roll_rate, gyro_pitch_rate, gyro_yaw_rate; // bias-corrected rates (rad/s)
    double smooth_roll_rate, smooth_pitch_rate, smooth_yaw_rate; // EMA smoothed rates (rad/s)
    double raw_gx, raw_gy, raw_gz; // raw sensor counts (LSB)
    double timestamp_ms; // monotonic raw ms
};

// IMU 링버퍼: 최근 샘플 저장 (타임스탬프 포함)
static std::deque<ImuPose> g_imu_buffer;
static std::atomic<double> g_imu_actual_hz{0}; // 실측 샘플링 레이트 (atomic → lock 불필요)

// 프로그램 시작 시점 기준 ms 반환
static int64_t clock_ns(clockid_t id) {
    struct timespec ts;
    clock_gettime(id, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static int64_t g_time_origin_raw_ns = clock_ns(CLOCK_MONOTONIC_RAW);
static double now_ms() {
    return (clock_ns(CLOCK_MONOTONIC_RAW) - g_time_origin_raw_ns) / 1e6;
}

// 프레임 중심 N/2 평균: frame_time_ms 기준 ±half_window_ms 범위의 샘플 평균

// 자이로 고주파 필터 상태
struct GyroSmoothState {
    double smooth_roll, smooth_pitch, smooth_yaw;
    bool initialized;
    double last_yaw;
    bool yaw_initialized;
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

static bool get_reference_timestamp_ns(GstBuffer* buffer, int64_t& out_ns) {
    gpointer state = nullptr;
    while (true) {
        GstMeta* meta = gst_buffer_iterate_meta(buffer, &state);
        if (!meta) break;
        if (meta->info->api == GST_REFERENCE_TIMESTAMP_META_API_TYPE) {
            GstReferenceTimestampMeta* r = (GstReferenceTimestampMeta*)meta;
            out_ns = (int64_t)r->timestamp;
            return true;
        }
    }
    return false;
}

static bool imu_average_from_seq(const std::deque<ImuPose>& buf,
                                 double target_ms,
                                 double window_ms,
                                 int max_samples,
                                 ImuPose& out,
                                 double& err_ms,
                                 int* out_count,
                                 double* out_min_ts,
                                 double* out_max_ts) {
    if (buf.empty() || window_ms <= 0) return false;
    const double t_lo = target_ms - window_ms;
    const double t_hi = target_ms + window_ms;

    int count = 0;
    double sum_roll = 0, sum_pitch = 0;
    double sum_rr = 0, sum_pr = 0, sum_yr = 0;
    double sum_srr = 0, sum_spr = 0, sum_syr = 0;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    double sum_sin_y = 0, sum_cos_y = 0;
    double sum_ts = 0;
    double min_ts = 1e18, max_ts = -1e18;

    // buffer is time-ordered
    for (const auto& s : buf) {
        if (s.timestamp_ms < t_lo) continue;
        if (s.timestamp_ms > t_hi) break;
        sum_roll += s.roll;
        sum_pitch += s.pitch;
        sum_rr += s.gyro_roll_rate;
        sum_pr += s.gyro_pitch_rate;
        sum_yr += s.gyro_yaw_rate;
        sum_srr += s.smooth_roll_rate;
        sum_spr += s.smooth_pitch_rate;
        sum_syr += s.smooth_yaw_rate;
        sum_gx += s.raw_gx;
        sum_gy += s.raw_gy;
        sum_gz += s.raw_gz;
        sum_sin_y += sin(s.yaw);
        sum_cos_y += cos(s.yaw);
        sum_ts += s.timestamp_ms;
        min_ts = std::min(min_ts, s.timestamp_ms);
        max_ts = std::max(max_ts, s.timestamp_ms);
        count++;
        if (max_samples > 0 && count >= max_samples) break;
    }

    if (count == 0) {
        // fallback to nearest sample
        const ImuPose* best = &buf.front();
        double best_err = std::abs(target_ms - best->timestamp_ms);
        for (const auto& s : buf) {
            double e = std::abs(target_ms - s.timestamp_ms);
            if (e < best_err) { best_err = e; best = &s; }
            if (s.timestamp_ms > target_ms && e > best_err) break;
        }
        out = *best;
        err_ms = target_ms - best->timestamp_ms;
        if (out_count) *out_count = 1;
        if (out_min_ts) *out_min_ts = best->timestamp_ms;
        if (out_max_ts) *out_max_ts = best->timestamp_ms;
        return true;
    }

    double inv = 1.0 / count;
    out.roll = sum_roll * inv;
    out.pitch = sum_pitch * inv;
    out.yaw = atan2(sum_sin_y, sum_cos_y);
    out.gyro_roll_rate = sum_rr * inv;
    out.gyro_pitch_rate = sum_pr * inv;
    out.gyro_yaw_rate = sum_yr * inv;
    out.smooth_roll_rate = sum_srr * inv;
    out.smooth_pitch_rate = sum_spr * inv;
    out.smooth_yaw_rate = sum_syr * inv;
    out.raw_gx = sum_gx * inv;
    out.raw_gy = sum_gy * inv;
    out.raw_gz = sum_gz * inv;
    out.timestamp_ms = sum_ts * inv;
    err_ms = target_ms - out.timestamp_ms;
    if (out_count) *out_count = count;
    if (out_min_ts) *out_min_ts = min_ts;
    if (out_max_ts) *out_max_ts = max_ts;
    return true;
}


static bool imu_sample_at(double target_ms, ImuPose& out, double& err_ms,
                          int* out_count = nullptr,
                          double* out_min_ts = nullptr,
                          double* out_max_ts = nullptr) {
    std::lock_guard<std::mutex> lk(g_imu_mtx);
    return imu_average_from_seq(g_imu_buffer, target_ms, IMU_AVG_WINDOW_MS, IMU_AVG_MAX_SAMPLES,
                                out, err_ms, out_count, out_min_ts, out_max_ts);
}

static double unwrap_angle(double prev, double curr) {
    double diff = curr - prev;
    while (diff > CV_PI) {
        curr -= 2.0 * CV_PI;
        diff = curr - prev;
    }
    while (diff < -CV_PI) {
        curr += 2.0 * CV_PI;
        diff = curr - prev;
    }
    return curr;
}

static Mat homography_from_angles(double roll, double pitch, double yaw, const Mat& K, const Mat& Kinv) {
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);

    Mat Rx = (Mat_<double>(3, 3) << 1, 0, 0,
              0, cr, -sr,
              0, sr, cr);
    Mat Ry = (Mat_<double>(3, 3) << cp, 0, sp,
              0, 1, 0,
              -sp, 0, cp);
    Mat Rz = (Mat_<double>(3, 3) << cy, -sy, 0,
              sy, cy, 0,
              0, 0, 1);
    Mat R = Rz * Ry * Rx;
    return K * R * Kinv;
}

static double required_scale_from_homography(const Mat& H, int w, int h) {
    std::vector<Point2f> corners;
    corners.emplace_back(0.0f, 0.0f);
    corners.emplace_back((float)w, 0.0f);
    corners.emplace_back((float)w, (float)h);
    corners.emplace_back(0.0f, (float)h);

    std::vector<Point2f> warped;
    perspectiveTransform(corners, warped, H);

    double minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
    for (const auto& p : warped) {
        minx = std::min(minx, (double)p.x);
        maxx = std::max(maxx, (double)p.x);
        miny = std::min(miny, (double)p.y);
        maxy = std::max(maxy, (double)p.y);
    }

    double overlap_w = std::min((double)w, maxx) - std::max(0.0, minx);
    double overlap_h = std::min((double)h, maxy) - std::max(0.0, miny);
    if (overlap_w <= 1.0 || overlap_h <= 1.0) return 10.0;

    double scale_x = (double)w / overlap_w;
    double scale_y = (double)h / overlap_h;
    return std::max(1.0, std::max(scale_x, scale_y));
}

static double compute_corr(const std::deque<std::pair<double, double>>& buf) {
    if (buf.size() < 5) return 0.0;
    double sum_x = 0, sum_y = 0;
    for (const auto& p : buf) {
        sum_x += p.first;
        sum_y += p.second;
    }
    double mean_x = sum_x / buf.size();
    double mean_y = sum_y / buf.size();
    double num = 0, den_x = 0, den_y = 0;
    for (const auto& p : buf) {
        double dx = p.first - mean_x;
        double dy = p.second - mean_y;
        num += dx * dy;
        den_x += dx * dx;
        den_y += dy * dy;
    }
    double denom = sqrt(den_x * den_y);
    if (denom < 1e-9) return 0.0;
    return num / denom;
}

enum class TsSource {
    SENSOR = 0,
    PTS = 1,
    ARRIVAL = 2
};

static const char* ts_source_str(TsSource s) {
    switch (s) {
    case TsSource::SENSOR: return "SENSOR";
    case TsSource::PTS: return "PTS";
    default: return "ARRIVAL";
    }
}

static const char* mode_str(EisMode m) {
    switch (m) {
    case EisMode::LK: return "LK";
    case EisMode::GYRO: return "GYRO";
    default: return "HYBRID";
    }
}

static const char* output_mode_str(OutputMode m) {
    switch (m) {
    case OutputMode::RAW_ONLY: return "RAW_ONLY";
    case OutputMode::CAM_ONLY: return "CAM_ONLY";
    default: return "BOTH";
    }
}

static const char* ts_pref_str(TsSourcePref p) {
    switch (p) {
    case TsSourcePref::SENSOR: return "SENSOR";
    case TsSourcePref::PTS: return "PTS";
    case TsSourcePref::ARRIVAL: return "ARRIVAL";
    default: return "AUTO";
    }
}

static void keyboard_loop() {
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

    fprintf(stderr, "[KEY] 1=LK 2=Gyro 3=Hybrid 4=RawOnly 5=CamOnly\n");

    pollfd pfd;
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;

    while (g_running) {
        int r = poll(&pfd, 1, 100);
        if (r > 0 && (pfd.revents & POLLIN)) {
            char ch = 0;
            ssize_t n = read(STDIN_FILENO, &ch, 1);
            if (n == 1) {
                if (ch == '1') {
                    g_mode = (int)EisMode::LK;
                    fprintf(stderr, "[MODE] %s\n", mode_str(EisMode::LK));
                } else if (ch == '2') {
                    g_mode = (int)EisMode::GYRO;
                    fprintf(stderr, "[MODE] %s\n", mode_str(EisMode::GYRO));
                } else if (ch == '3') {
                    g_mode = (int)EisMode::HYBRID;
                    fprintf(stderr, "[MODE] %s\n", mode_str(EisMode::HYBRID));
                } else if (ch == '4') {
                    g_output_mode = (int)OutputMode::RAW_ONLY;
                    fprintf(stderr, "[OUT] %s\n", output_mode_str(OutputMode::RAW_ONLY));
                } else if (ch == '5') {
                    g_output_mode = (int)OutputMode::CAM_ONLY;
                    fprintf(stderr, "[OUT] %s\n", output_mode_str(OutputMode::CAM_ONLY));
                }
            }
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
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
        "( appsrc name=" + std::string(appsrc_name) + " is-live=true format=time do-timestamp=true block=false "
                                                      "! videoconvert "
                                                      "! video/x-raw,format=I420 "
                                                      "! v4l2h264enc extra-controls=\"controls,video_bitrate=1500000,h264_i_frame_period=30\" "
                                                      "! video/x-h264,level=(string)4,profile=(string)baseline "
                                                      "! rtph264pay name=pay0 pt=96 config-interval=1 )";
    gst_rtsp_media_factory_set_launch(factory, launch.c_str());
    gst_rtsp_media_factory_set_shared(factory, TRUE);
    gst_rtsp_media_factory_set_suspend_mode(factory, GST_RTSP_SUSPEND_MODE_NONE);
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

static void sigint_handler(int) { g_running = false; }

static void print_usage(const char* prog) {
    fprintf(stderr, "Usage: %s [options]\n", prog);
    fprintf(stderr, "  --mode {lk|gyro|hybrid}\n");
    fprintf(stderr, "  --imu-offset-ms <double>\n");
    fprintf(stderr, "  --offset-sweep\n");
    fprintf(stderr, "  --overlay {0|1}\n");
    fprintf(stderr, "  --log-every-frames <N>\n");
    fprintf(stderr, "  --ts-source {auto|sensor|pts|arrival}\n");
}

static bool parse_args(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto eat_value = [&](std::string& out) -> bool {
            if (i + 1 >= argc) return false;
            out = argv[++i];
            return true;
        };

        std::string val;
        if (a == "--help" || a == "-h") {
            print_usage(argv[0]);
            return false;
        } else if (a == "--mode") {
            if (!eat_value(val)) return false;
        } else if (a.rfind("--mode=", 0) == 0) {
            val = a.substr(7);
        }

        if (!val.empty()) {
            std::string m = val;
            std::transform(m.begin(), m.end(), m.begin(), ::tolower);
            if (m == "lk") g_mode = (int)EisMode::LK;
            else if (m == "gyro") g_mode = (int)EisMode::GYRO;
            else if (m == "hybrid") g_mode = (int)EisMode::HYBRID;
            else return false;
            val.clear();
            continue;
        }

        if (a == "--imu-offset-ms") {
            if (!eat_value(val)) return false;
            g_manual_imu_offset_ms = std::stod(val);
        } else if (a.rfind("--imu-offset-ms=", 0) == 0) {
            g_manual_imu_offset_ms = std::stod(a.substr(16));
        } else if (a == "--offset-sweep") {
            g_offset_sweep = true;
        } else if (a == "--overlay") {
            if (!eat_value(val)) return false;
            g_debug_overlay = (std::stoi(val) != 0);
        } else if (a.rfind("--overlay=", 0) == 0) {
            g_debug_overlay = (std::stoi(a.substr(10)) != 0);
        } else if (a == "--log-every-frames") {
            if (!eat_value(val)) return false;
            g_log_every_frames = std::max(0, std::stoi(val));
        } else if (a.rfind("--log-every-frames=", 0) == 0) {
            g_log_every_frames = std::max(0, std::stoi(a.substr(19)));
        } else if (a == "--ts-source") {
            if (!eat_value(val)) return false;
            std::string s = val;
            std::transform(s.begin(), s.end(), s.begin(), ::tolower);
            if (s == "auto") g_ts_pref = (int)TsSourcePref::AUTO;
            else if (s == "sensor") g_ts_pref = (int)TsSourcePref::SENSOR;
            else if (s == "pts") g_ts_pref = (int)TsSourcePref::PTS;
            else if (s == "arrival") g_ts_pref = (int)TsSourcePref::ARRIVAL;
            else return false;
        } else if (a.rfind("--ts-source=", 0) == 0) {
            std::string s = a.substr(12);
            std::transform(s.begin(), s.end(), s.begin(), ::tolower);
            if (s == "auto") g_ts_pref = (int)TsSourcePref::AUTO;
            else if (s == "sensor") g_ts_pref = (int)TsSourcePref::SENSOR;
            else if (s == "pts") g_ts_pref = (int)TsSourcePref::PTS;
            else if (s == "arrival") g_ts_pref = (int)TsSourcePref::ARRIVAL;
            else return false;
        } else {
            fprintf(stderr, "[ERR] Unknown arg: %s\n", a.c_str());
            return false;
        }
    }
    return true;
}

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

    double gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
    double smooth_rr = 0, smooth_pr = 0, smooth_yr = 0;
    bool smooth_rate_init = false;
    g_imu_ready = true;
    fprintf(stderr, "[IMU] Ready (target: 200Hz, buffer: %d samples)\n", IMU_BUFFER_SIZE);

    int64_t last_time_ns = clock_ns(CLOCK_MONOTONIC_RAW);

    while (g_running) {
        uint8_t buf[6];
        if (!i2c_read_bytes(fd, 0x43, buf, 6)) {
            usleep(500);
            continue;
        }
        int64_t now_ns = clock_ns(CLOCK_MONOTONIC_RAW);
        double dt = (now_ns - last_time_ns) / 1e9;
        last_time_ns = now_ns;
        if (dt <= 0 || dt > 0.1) {
            usleep(500);
            continue;
        }

        double raw_gx = (double)to_i16(buf[0], buf[1]);
        double raw_gy = (double)to_i16(buf[2], buf[3]);
        double raw_gz = (double)to_i16(buf[4], buf[5]);
        double gx_rad = ((raw_gx - bias_gx) / GYRO_SENSITIVITY) * (CV_PI / 180.0);
        double gy_rad = ((raw_gy - bias_gy) / GYRO_SENSITIVITY) * (CV_PI / 180.0);
        double gz_rad = ((raw_gz - bias_gz) / GYRO_SENSITIVITY) * (CV_PI / 180.0);

        double roll_rate = IMU_SIGN_ROLL * pick_axis(IMU_AXIS_ROLL, gx_rad, gy_rad, gz_rad);
        double pitch_rate = IMU_SIGN_PITCH * pick_axis(IMU_AXIS_PITCH, gx_rad, gy_rad, gz_rad);
        double yaw_rate = IMU_SIGN_YAW * pick_axis(IMU_AXIS_YAW, gx_rad, gy_rad, gz_rad);
        gyro_roll += roll_rate * dt;
        gyro_pitch += pitch_rate * dt;
        gyro_yaw += yaw_rate * dt;

        if (!smooth_rate_init) {
            smooth_rr = roll_rate;
            smooth_pr = pitch_rate;
            smooth_yr = yaw_rate;
            smooth_rate_init = true;
        } else {
            smooth_rr = IMU_RATE_SMOOTH_ALPHA * smooth_rr + (1.0 - IMU_RATE_SMOOTH_ALPHA) * roll_rate;
            smooth_pr = IMU_RATE_SMOOTH_ALPHA * smooth_pr + (1.0 - IMU_RATE_SMOOTH_ALPHA) * pitch_rate;
            smooth_yr = IMU_RATE_SMOOTH_ALPHA * smooth_yr + (1.0 - IMU_RATE_SMOOTH_ALPHA) * yaw_rate;
        }

        // 샘플 준비 (lock 밖에서)
        double ts = now_ms();
        ImuPose sample = {gyro_roll, gyro_pitch, gyro_yaw,
                          roll_rate, pitch_rate, yaw_rate,
                          smooth_rr, smooth_pr, smooth_yr,
                          raw_gx, raw_gy, raw_gz,
                          ts};

        { // 최소 임계구간: push + pop만
            std::lock_guard<std::mutex> lk(g_imu_mtx);
            g_imu_buffer.push_back(sample);
            if ((int)g_imu_buffer.size() > IMU_STORE_SIZE)
                g_imu_buffer.pop_front();
        }
        if (dt > 0) {
            double hz = 1.0 / dt;
            double prev = g_imu_actual_hz.load();
            g_imu_actual_hz = (prev <= 0) ? hz : (prev * 0.9 + hz * 0.1);
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
        "! video/x-raw,format=RGBx,width=" + std::to_string(G_WIDTH) +
        ",height=" + std::to_string(G_HEIGHT) +
        ",framerate=" + std::to_string(G_FPS) + "/1 "
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

    bool sensor_map_ready = false;
    int64_t sensor_ts_first_ns = 0;
    int64_t sensor_raw_base_ns = 0;
    bool pts_map_ready = false;
    int64_t pts_first_ns = 0;
    int64_t pts_raw_base_ns = 0;

    bool ts_pref_warned = false;
    auto pull_frame = [&](Mat& out, double& out_time_ms, TsSource& out_src) -> bool {
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

        double t_arrival = now_ms();
        double t_sensor = t_arrival;
        double t_pts = t_arrival;
        bool have_sensor = false;
        bool have_pts = false;

        int64_t ref_ns = 0;
        if (get_reference_timestamp_ns(buffer, ref_ns)) {
            if (!sensor_map_ready) {
                sensor_map_ready = true;
                sensor_ts_first_ns = ref_ns;
                sensor_raw_base_ns = clock_ns(CLOCK_MONOTONIC_RAW);
            }
            int64_t rel_ns = ref_ns - sensor_ts_first_ns;
            t_sensor = (sensor_raw_base_ns + rel_ns - g_time_origin_raw_ns) / 1e6;
            have_sensor = true;
        }

        if (GST_CLOCK_TIME_IS_VALID(GST_BUFFER_PTS(buffer))) {
            int64_t pts_ns = (int64_t)GST_BUFFER_PTS(buffer);
            if (!pts_map_ready) {
                pts_map_ready = true;
                pts_first_ns = pts_ns;
                pts_raw_base_ns = clock_ns(CLOCK_MONOTONIC_RAW);
            }
            int64_t rel_ns = pts_ns - pts_first_ns;
            t_pts = (pts_raw_base_ns + rel_ns - g_time_origin_raw_ns) / 1e6;
            have_pts = true;
        }

        TsSourcePref pref = (TsSourcePref)g_ts_pref.load();
        out_src = TsSource::ARRIVAL;
        out_time_ms = t_arrival;
        bool used_pref = true;

        if (pref == TsSourcePref::AUTO) {
            if (have_sensor) { out_src = TsSource::SENSOR; out_time_ms = t_sensor; }
            else if (have_pts) { out_src = TsSource::PTS; out_time_ms = t_pts; }
        } else if (pref == TsSourcePref::SENSOR) {
            if (have_sensor) { out_src = TsSource::SENSOR; out_time_ms = t_sensor; }
            else if (have_pts) { out_src = TsSource::PTS; out_time_ms = t_pts; used_pref = false; }
            else used_pref = false;
        } else if (pref == TsSourcePref::PTS) {
            if (have_pts) { out_src = TsSource::PTS; out_time_ms = t_pts; }
            else if (have_sensor) { out_src = TsSource::SENSOR; out_time_ms = t_sensor; used_pref = false; }
            else used_pref = false;
        } else if (pref == TsSourcePref::ARRIVAL) {
            out_src = TsSource::ARRIVAL;
            out_time_ms = t_arrival;
        }

        if (!used_pref && !ts_pref_warned) {
            fprintf(stderr, "[TS] preferred %s not available -> fallback %s\n",
                    ts_pref_str(pref), ts_source_str(out_src));
            ts_pref_warned = true;
        }

        gst_sample_unref(sample);
        return !out.empty();
    };

    const Point2f center((float)G_WIDTH * 0.5f, (float)G_HEIGHT * 0.5f);

    guint64 frameIdx = 0;

    GyroSmoothState gs = {0, 0, 0, false, 0, false};

    Mat prev_gray;
    Mat prev_frame;
    int frame_count = 0;
    double prev_frame_time_ms = 0.0;
    bool prev_time_ok = false;
    KalmanState kf_theta, kf_tx, kf_ty;
    kalman_init(kf_theta, KF_Q, KF_R);
    kalman_init(kf_tx, KF_Q, KF_R);
    kalman_init(kf_ty, KF_Q, KF_R);

    struct CalibFrame { double t_ms; double lk_da; };
    std::vector<CalibFrame> calib_frames;
    double calib_start_ms = -1.0;
    bool do_offset_sweep = g_offset_sweep.load();
    bool offset_calibrated = !do_offset_sweep;
    double time_offset_ms = g_manual_imu_offset_ms;
    Mat prev_calib_gray;
    bool calib_prev_ok = false;

    // Sweep quality gates
    const double SWEEP_MAX_ERR_MS = 8.0;
    const int SWEEP_MIN_SAMPLES = 5;

    std::deque<double> crop_scales;
    double smooth_crop_percent = FIXED_CROP_PERCENT;

    double imu_err_sum = 0.0;
    double imu_err_max = 0.0;
    int imu_err_cnt = 0;
    TsSource last_ts_src = TsSource::ARRIVAL;

    std::deque<std::pair<double, double>> corr_buf;
    double corr_val = 0.0;
    const int corr_window = std::max(10, G_FPS);

    EisMode last_mode = (EisMode)g_mode.load();

    struct ImuSampleInfo {
        int count = 0;
        double min_ts = 0.0;
        double max_ts = 0.0;
        double err_ms = 0.0;
    };

    while (g_running) {
        Mat frame;
        double frame_time_ms = 0.0;
        TsSource ts_src = TsSource::ARRIVAL;
        if (!pull_frame(frame, frame_time_ms, ts_src) || frame.empty()) continue;

        if (FLIP_VERTICAL) {
            cv::flip(frame, frame, 0);
        }

        if (ts_src != last_ts_src) {
            fprintf(stderr, "[TS] source=%s\n", ts_source_str(ts_src));
            last_ts_src = ts_src;
        }

        EisMode mode = (EisMode)g_mode.load();
        if (mode != last_mode) {
            prev_gray.release();
            prev_frame.release();
            frame_count = 0;
            kalman_init(kf_theta, KF_Q, KF_R);
            kalman_init(kf_tx, KF_Q, KF_R);
            kalman_init(kf_ty, KF_Q, KF_R);
            gs.initialized = false;
            gs.yaw_initialized = false;
            prev_time_ok = false;
            corr_buf.clear();
            corr_val = 0.0;
            crop_scales.clear();
            smooth_crop_percent = FIXED_CROP_PERCENT;
            imu_err_sum = 0.0;
            imu_err_max = 0.0;
            imu_err_cnt = 0;
            last_mode = mode;
            fprintf(stderr, "[MODE] switched to %s\n", mode_str(mode));
        }

        Mat curr_gray;
        cvtColor(frame, curr_gray, COLOR_BGR2GRAY);

        if (g_imu_ready.load() && do_offset_sweep) {
            if (calib_start_ms < 0.0) calib_start_ms = frame_time_ms;

            if (!offset_calibrated && (frame_time_ms - calib_start_ms) <= OFFSET_CALIB_DURATION_MS) {
                if (calib_prev_ok) {
                    std::vector<Point2f> feat_prev, feat_curr;
                    goodFeaturesToTrack(prev_calib_gray, feat_prev, LK_MAX_FEATURES, LK_QUALITY, LK_MIN_DIST);
                    if (feat_prev.size() >= 10) {
                        std::vector<uchar> status;
                        std::vector<float> err_vec;
                        calcOpticalFlowPyrLK(prev_calib_gray, curr_gray, feat_prev, feat_curr, status, err_vec);

                        std::vector<Point2f> gp, gc;
                        for (size_t i = 0; i < status.size(); i++) {
                            if (status[i]) {
                                gp.push_back(feat_prev[i]);
                                gc.push_back(feat_curr[i]);
                            }
                        }
                        if (gp.size() >= 6) {
                            Mat affine = estimateAffinePartial2D(gp, gc);
                            if (!affine.empty()) {
                                double da = atan2(affine.at<double>(1, 0), affine.at<double>(0, 0));
                                calib_frames.push_back({frame_time_ms, da});
                            }
                        }
                    }
                }
                prev_calib_gray = curr_gray.clone();
                calib_prev_ok = true;
            } else if (!offset_calibrated && (frame_time_ms - calib_start_ms) > OFFSET_CALIB_DURATION_MS) {
                std::deque<ImuPose> imu_copy;
                {
                    std::lock_guard<std::mutex> lk(g_imu_mtx);
                    imu_copy = g_imu_buffer;
                }
                    if (calib_frames.size() >= 8 && imu_copy.size() >= 10) {
                        double base_off = g_manual_imu_offset_ms;
                        auto cost_for = [&](double delta_ms) -> double {
                            double off_ms = base_off + delta_ms;
                            double sum = 0.0;
                            int n = 0;
                        for (size_t i = 1; i < calib_frames.size(); ++i) {
                            double t0 = calib_frames[i - 1].t_ms + off_ms;
                            double t1 = calib_frames[i].t_ms + off_ms;
                            ImuPose p0, p1;
                            double err;
                            int c0 = 0, c1 = 0;
                            double r0_min = 0.0, r0_max = 0.0, r1_min = 0.0, r1_max = 0.0;
                            if (!imu_average_from_seq(imu_copy, t0, IMU_AVG_WINDOW_MS, IMU_AVG_MAX_SAMPLES,
                                                      p0, err, &c0, &r0_min, &r0_max)) continue;
                            if (c0 < SWEEP_MIN_SAMPLES || std::abs(err) > SWEEP_MAX_ERR_MS) continue;
                            if (!imu_average_from_seq(imu_copy, t1, IMU_AVG_WINDOW_MS, IMU_AVG_MAX_SAMPLES,
                                                      p1, err, &c1, &r1_min, &r1_max)) continue;
                            if (c1 < SWEEP_MIN_SAMPLES || std::abs(err) > SWEEP_MAX_ERR_MS) continue;
                            double imu_dyaw = p1.yaw - p0.yaw;
                            double diff = calib_frames[i].lk_da - imu_dyaw;
                            sum += diff * diff;
                            n++;
                        }
                        if (n == 0) return 1e18;
                        return sum / n;
                    };

                        double best_off = 0.0;
                        double best_cost = 1e18;
                        for (double off = -OFFSET_COARSE_RANGE_MS; off <= OFFSET_COARSE_RANGE_MS; off += OFFSET_COARSE_STEP_MS) {
                            double c = cost_for(off);
                            if (c < best_cost) { best_cost = c; best_off = off; }
                        }
                        for (double off = best_off - OFFSET_FINE_RANGE_MS; off <= best_off + OFFSET_FINE_RANGE_MS; off += OFFSET_FINE_STEP_MS) {
                            double c = cost_for(off);
                            if (c < best_cost) { best_cost = c; best_off = off; }
                        }
                        if (best_cost < 1e17) {
                            time_offset_ms = base_off + best_off;
                        } else {
                            time_offset_ms = base_off;
                            fprintf(stderr, "[SYNC] sweep failed (insufficient valid samples)\n");
                        }
                    } else {
                        time_offset_ms = g_manual_imu_offset_ms;
                    }
                offset_calibrated = true;
                fprintf(stderr, "[SYNC] offset=%.3f ms (base=%.3f, frames=%zu)\n",
                        time_offset_ms, g_manual_imu_offset_ms, calib_frames.size());
            }
        }

        ImuPose pose {};
        ImuPose pose_prev {};
        ImuSampleInfo info_curr {};
        ImuSampleInfo info_prev {};
        bool imu_ok = false;
        bool imu_prev_ok = false;
        double target_curr_ms = frame_time_ms + time_offset_ms;
        double target_prev_ms = 0.0;
        bool have_target_prev = false;
        if (g_imu_ready.load()) {
            imu_ok = imu_sample_at(target_curr_ms, pose, info_curr.err_ms,
                                   &info_curr.count, &info_curr.min_ts, &info_curr.max_ts);
            if (imu_ok) {
                double err_abs = std::abs(info_curr.err_ms);
                imu_err_sum += err_abs;
                imu_err_max = std::max(imu_err_max, err_abs);
                imu_err_cnt++;
            }
        }

        double gyro_dyaw = 0.0;
        bool gyro_delta_ok = false;
        if (prev_time_ok && g_imu_ready.load()) {
            target_prev_ms = prev_frame_time_ms + time_offset_ms;
            have_target_prev = true;
            imu_prev_ok = imu_sample_at(target_prev_ms, pose_prev, info_prev.err_ms,
                                        &info_prev.count, &info_prev.min_ts, &info_prev.max_ts);
            if (imu_prev_ok && imu_ok) {
                double yaw1 = unwrap_angle(pose_prev.yaw, pose.yaw);
                gyro_dyaw = yaw1 - pose_prev.yaw;
                gyro_delta_ok = true;
            }
        }
        prev_frame_time_ms = frame_time_ms;
        prev_time_ok = true;

        double jitter_roll = 0, jitter_pitch = 0, jitter_yaw = 0;
        if (imu_ok) {
            if (gs.yaw_initialized) {
                pose.yaw = unwrap_angle(gs.last_yaw, pose.yaw);
            }
            gs.last_yaw = pose.yaw;
            gs.yaw_initialized = true;

            if (!gs.initialized) {
                gs.smooth_roll = pose.roll;
                gs.smooth_pitch = pose.pitch;
                gs.smooth_yaw = pose.yaw;
                gs.initialized = true;
            } else {
                double diff_r = pose.roll - gs.smooth_roll;
                double diff_p = pose.pitch - gs.smooth_pitch;
                double diff_y = pose.yaw - gs.smooth_yaw;
                double alpha_r = adaptiveAlpha(SMOOTH_ALPHA, diff_r, ADAPT_THRESHOLD, ADAPT_RATE, ADAPT_MIN_ALPHA);
                double alpha_p = adaptiveAlpha(SMOOTH_ALPHA, diff_p, ADAPT_THRESHOLD, ADAPT_RATE, ADAPT_MIN_ALPHA);
                double alpha_y = adaptiveAlpha(SMOOTH_ALPHA, diff_y, ADAPT_THRESHOLD, ADAPT_RATE, ADAPT_MIN_ALPHA);
                gs.smooth_roll = alpha_r * gs.smooth_roll + (1.0 - alpha_r) * pose.roll;
                gs.smooth_pitch = alpha_p * gs.smooth_pitch + (1.0 - alpha_p) * pose.pitch;
                gs.smooth_yaw = alpha_y * gs.smooth_yaw + (1.0 - alpha_y) * pose.yaw;
            }

            jitter_roll = pose.roll - gs.smooth_roll;
            jitter_pitch = pose.pitch - gs.smooth_pitch;
            jitter_yaw = pose.yaw - gs.smooth_yaw;
        } else {
            gs.initialized = false;
            gs.yaw_initialized = false;
        }

        Mat prev_frame_for_warp = prev_frame;
        double lk_dx = 0.0, lk_dy = 0.0, lk_da_raw = 0.0;
        double lk_diff_dx = 0.0, lk_diff_dy = 0.0, lk_diff_da = 0.0;
        double lk_da_used = 0.0;
        bool lk_ok = false;
        Mat H_lk = Mat::eye(3, 3, CV_64F);

        if (frame_count == 0) {
            prev_gray = curr_gray.clone();
            prev_frame = frame.clone();
            frame_count = 1;
        } else {
            std::vector<Point2f> features_prev, features_curr;
            std::vector<uchar> status;
            std::vector<float> err_vec;
            goodFeaturesToTrack(prev_gray, features_prev, LK_MAX_FEATURES, LK_QUALITY, LK_MIN_DIST);
            if (features_prev.size() >= 10) {
                calcOpticalFlowPyrLK(prev_gray, curr_gray, features_prev, features_curr, status, err_vec);
                std::vector<Point2f> good_prev, good_curr;
                for (size_t i = 0; i < status.size(); i++) {
                    if (status[i]) {
                        good_prev.push_back(features_prev[i]);
                        good_curr.push_back(features_curr[i]);
                    }
                }
                if (good_prev.size() >= 6) {
                    Mat affine = estimateAffinePartial2D(good_prev, good_curr);
                    if (!affine.empty()) {
                        lk_dx = affine.at<double>(0, 2);
                        lk_dy = affine.at<double>(1, 2);
                        lk_da_raw = atan2(affine.at<double>(1, 0), affine.at<double>(0, 0));

                        kalman_update(kf_theta, lk_da_raw);
                        kalman_update(kf_tx, lk_dx);
                        kalman_update(kf_ty, lk_dy);

                        if (frame_count > 1) {
                            lk_diff_da = kalman_diff(kf_theta, MAX_DIFF_DA);
                            lk_diff_dx = kalman_diff(kf_tx, MAX_DIFF_DX);
                            lk_diff_dy = kalman_diff(kf_ty, MAX_DIFF_DY);
                        }
                        if (frame_count == 1) frame_count++;

                        lk_da_used = lk_da_raw + lk_diff_da;
                        double dx = lk_dx + lk_diff_dx;
                        double dy = lk_dy + lk_diff_dy;

                        H_lk = (Mat_<double>(3, 3) << cos(lk_da_used), -sin(lk_da_used), dx,
                                                    sin(lk_da_used),  cos(lk_da_used), dy,
                                                    0,               0,              1);
                        lk_ok = true;
                    }
                }
            }

            prev_gray = curr_gray.clone();
            prev_frame = frame.clone();
        }

        if (lk_ok && gyro_delta_ok) {
            corr_buf.emplace_back(lk_da_raw, gyro_dyaw);
            if ((int)corr_buf.size() > corr_window) corr_buf.pop_front();
            corr_val = compute_corr(corr_buf);
        }

        Mat H_gyro = Mat::eye(3, 3, CV_64F);
        double yaw_corr = 0.0;
        bool gyro_corr_ok = false;
        if (imu_ok) {
            yaw_corr = -std::clamp(jitter_yaw * YAW_GAIN, -MAX_YAW_RAD, MAX_YAW_RAD);
            if (std::abs(yaw_corr) > 1e-6) {
                Mat R2 = getRotationMatrix2D(center, yaw_corr * 180.0 / CV_PI, 1.0);
                H_gyro.at<double>(0, 0) = R2.at<double>(0, 0);
                H_gyro.at<double>(0, 1) = R2.at<double>(0, 1);
                H_gyro.at<double>(0, 2) = R2.at<double>(0, 2);
                H_gyro.at<double>(1, 0) = R2.at<double>(1, 0);
                H_gyro.at<double>(1, 1) = R2.at<double>(1, 1);
                H_gyro.at<double>(1, 2) = R2.at<double>(1, 2);
            }
            gyro_corr_ok = true;
        }

        Mat stabilized = frame.clone();
        Mat H_total = Mat::eye(3, 3, CV_64F);
        Mat warp_src = frame;
        bool apply_warp = false;

        if (mode == EisMode::LK) {
            if (lk_ok) {
                H_total = H_lk;
                warp_src = prev_frame_for_warp;
                apply_warp = true;
            }
        } else if (mode == EisMode::GYRO) {
            if (gyro_corr_ok) {
                H_total = H_gyro;
                warp_src = frame;
                apply_warp = true;
            }
        } else { // HYBRID
            if (lk_ok) {
                H_total = H_lk;
                warp_src = prev_frame_for_warp;
                apply_warp = true;
            }
            if (gyro_corr_ok) {
                H_total = H_gyro * H_total;
                if (!lk_ok) warp_src = frame;
                apply_warp = true;
            }
        }

        if (apply_warp) {
            warpPerspective(warp_src, stabilized, H_total, frame.size(), INTER_LINEAR, BORDER_CONSTANT);
        } else {
            stabilized = frame.clone();
            H_total = Mat::eye(3, 3, CV_64F);
        }

        if (DYNAMIC_CROP) {
            double scale = required_scale_from_homography(H_total, frame.cols, frame.rows);
            crop_scales.push_back(scale);
            if ((int)crop_scales.size() > CROP_WINDOW) crop_scales.pop_front();

            double max_scale = 1.0;
            for (double s : crop_scales) max_scale = std::max(max_scale, s);
            double target_crop = (1.0 - 1.0 / max_scale) * 100.0;
            target_crop = std::clamp(target_crop, CROP_MIN_PERCENT, CROP_MAX_PERCENT);

            smooth_crop_percent = CROP_SMOOTH_ALPHA * smooth_crop_percent + (1.0 - CROP_SMOOTH_ALPHA) * target_crop;
            stabilized = centerCropAndResize(stabilized, smooth_crop_percent);
        } else {
            stabilized = centerCropAndResize(stabilized, FIXED_CROP_PERCENT);
        }

        bool dbg = g_debug_overlay.load();
        if (dbg) {
            char buf[256];
            const int font = FONT_HERSHEY_SIMPLEX;
            double imu_err_ms = imu_ok ? info_curr.err_ms : 0.0;
            snprintf(buf, sizeof(buf), "TS:%s off:%+.2fms err:%+.2fms mode:%s out:%s",
                     ts_source_str(ts_src), time_offset_ms, imu_err_ms,
                     mode_str(mode), output_mode_str((OutputMode)g_output_mode.load()));
            putText(stabilized, buf, Point(8, 18), font, 0.4, Scalar(0, 255, 0), 1, LINE_AA);

            snprintf(buf, sizeof(buf), "LK da:%+.2f  Gyro dy:%+.2f  corr:%+.2f",
                     lk_da_raw * 180 / CV_PI, gyro_dyaw * 180 / CV_PI, corr_val);
            putText(stabilized, buf, Point(8, 33), font, 0.4, Scalar(255, 200, 0), 1, LINE_AA);

            snprintf(buf, sizeof(buf), "Gyro jit Y:%+.2f",
                     jitter_yaw * 180 / CV_PI);
            putText(stabilized, buf, Point(8, 48), font, 0.4, Scalar(0, 200, 255), 1, LINE_AA);
        }

        int log_every = g_log_every_frames.load();
        if (log_every > 0 && (frameIdx % (guint64)log_every == 0)) {
            if (imu_err_cnt > 0) {
                double avg_err = imu_err_sum / imu_err_cnt;
                fprintf(stderr, "[SYNC] imu_err avg=%.3f ms max=%.3f ms offset=%.3f ms\n",
                        avg_err, imu_err_max, time_offset_ms);
            }

            if (imu_ok) {
                fprintf(stderr,
                        "[GYRO] raw(gx,gy,gz)=%.1f %.1f %.1f  rate(rad/s)=%.3f %.3f %.3f  "
                        "smooth=%.3f %.3f %.3f  ang(deg)=%.2f %.2f %.2f  hz=%.1f\n",
                        pose.raw_gx, pose.raw_gy, pose.raw_gz,
                        pose.gyro_roll_rate, pose.gyro_pitch_rate, pose.gyro_yaw_rate,
                        pose.smooth_roll_rate, pose.smooth_pitch_rate, pose.smooth_yaw_rate,
                        pose.roll * 180 / CV_PI, pose.pitch * 180 / CV_PI, pose.yaw * 180 / CV_PI,
                        g_imu_actual_hz.load());
            }

            fprintf(stderr,
                    "[CMP] lk_da=%.3fdeg gyro_dyaw=%.3fdeg diff=%.3fdeg corr=%.3f\n",
                    lk_da_raw * 180 / CV_PI,
                    gyro_dyaw * 180 / CV_PI,
                    (lk_da_raw - gyro_dyaw) * 180 / CV_PI,
                    corr_val);

            if (imu_prev_ok || imu_ok) {
                double log_t0 = have_target_prev ? target_prev_ms : target_curr_ms;
                fprintf(stderr,
                        "[WIN] t0=%.2f err0=%+.2fms cnt0=%d range0=[%.2f,%.2f]  "
                        "t1=%.2f err1=%+.2fms cnt1=%d range1=[%.2f,%.2f]\n",
                        log_t0, info_prev.err_ms, info_prev.count,
                        info_prev.min_ts, info_prev.max_ts,
                        target_curr_ms, info_curr.err_ms, info_curr.count,
                        info_curr.min_ts, info_curr.max_ts);
            }
        }

        GstAppSrc *rawsrc, *stabsrc;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            rawsrc = g_rawsrc;
            stabsrc = g_stabsrc;
        }

        Mat raw_out = frame.clone();
        if (dbg) {
            char buf[160];
            snprintf(buf, sizeof(buf), "mode:%s off:%+.1fms",
                     mode_str(mode), time_offset_ms);
            putText(raw_out, buf, Point(8, 18), FONT_HERSHEY_SIMPLEX, 0.35, Scalar(0, 255, 0), 1, LINE_AA);
        }

        OutputMode out_mode = (OutputMode)g_output_mode.load();
        bool send_raw = (out_mode == OutputMode::BOTH || out_mode == OutputMode::RAW_ONLY);
        bool send_cam = (out_mode == OutputMode::BOTH || out_mode == OutputMode::CAM_ONLY);
        if (send_raw) push_bgr(rawsrc, raw_out, frameIdx, "raw");
        if (send_cam) push_bgr(stabsrc, stabilized, frameIdx, "cam");
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
    if (!parse_args(argc, argv)) return 0;
    if (g_log_every_frames.load() < 0) g_log_every_frames = G_FPS * 2;
    gst_init(&argc, &argv);

    GstRTSPServer* server = gst_rtsp_server_new();
    gst_rtsp_server_set_service(server, "8555");
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);

    GstRTSPMediaFactory* f_raw = make_factory("rawsrc");
    g_signal_connect(f_raw, "media-configure", (GCallback)on_media_configure, (gpointer) "raw");
    gst_rtsp_mount_points_add_factory(mounts, "/raw", f_raw);

    GstRTSPMediaFactory* f_stab = make_factory("stabsrc");
    g_signal_connect(f_stab, "media-configure", (GCallback)on_media_configure, (gpointer) "stab");
    gst_rtsp_mount_points_add_factory(mounts, "/cam", f_stab);

    g_object_unref(mounts);

    if (gst_rtsp_server_attach(server, nullptr) == 0) {
        cerr << "[ERR] RTSP attach\n";
        return -1;
    }

    fprintf(stderr, "==============================\n");
    fprintf(stderr, " EIS - LK 기반 + Gyro 정렬 검증\n");
    fprintf(stderr, "==============================\n");
    fprintf(stderr, "  Mode: %s\n", mode_str((EisMode)g_mode.load()));
    fprintf(stderr, "  IMU offset: %.3f ms  (sweep: %s)\n", g_manual_imu_offset_ms,
            g_offset_sweep.load() ? "on" : "off");
    fprintf(stderr, "  TS source: %s\n", ts_pref_str((TsSourcePref)g_ts_pref.load()));
    fprintf(stderr, "  Gyro smoothing alpha=%.3f  rate-smooth=%.3f\n", SMOOTH_ALPHA, IMU_RATE_SMOOTH_ALPHA);
    fprintf(stderr, "  LK Kalman Q=%.4f R=%.1f\n", KF_Q, KF_R);
    fprintf(stderr, "  Crop: %s (min %.0f%%)\n", DYNAMIC_CROP ? "dynamic" : "fixed", FIXED_CROP_PERCENT);
    fprintf(stderr, "  RTSP: rtsp://<PI_IP>:8555/raw | /cam\n");
    fprintf(stderr, "  Keys: 1=LK 2=Gyro 3=Hybrid 4=RawOnly 5=CamOnly\n");
    fprintf(stderr, "==============================\n");

    std::thread key_th(keyboard_loop);
    std::thread imu_th(imu_loop);
    std::thread cap_th(capture_loop);

    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop);

    g_running = false;
    if (cap_th.joinable()) cap_th.join();
    if (imu_th.joinable()) imu_th.join();
    if (key_th.joinable()) key_th.join();
    g_main_loop_unref(loop);
    return 0;
}
