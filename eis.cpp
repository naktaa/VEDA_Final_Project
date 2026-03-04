/**
 * eis.cpp — 하이브리드 EIS (자이로 Roll + LK 이동 보정, 단일 warp)
 *
 * 보정 원리:
 *   1) 자이로 고주파 필터 → roll jitter 추출 → 역회전 행렬
 *   2) LK OptFlow + 칼만 → 원본 프레임의 이동/회전 보정 행렬
 *   3) 두 행렬을 합쳐서 warpAffine 1회만 호출 (여백 최소화)
 *
 * RTSP 출력:
 *   /raw  — 원본 영상
 *   /cam  — 보정 영상
 */

#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/video/video.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std;
using namespace cv;

// ======================== 설정 상수 ========================

static const int G_WIDTH = 640;
static const int G_HEIGHT = 480;
static const int G_FPS = 24;

// MPU-6050
static const int IMU_ADDR = 0x68;
static const double GYRO_SENSITIVITY = 131.0;

// ---- 자이로 고주파 필터 (roll 보정만 담당) ----
static const double SMOOTH_ALPHA = 0.99;
static const double ROLL_GAIN  = 1.0;
static const double MAX_ROLL_RAD  = 10.0 * CV_PI / 180.0;

// 적응형 Alpha
static const double ADAPT_THRESHOLD = 3.0 * CV_PI / 180.0;
static const double ADAPT_RATE = 15.0;
static const double ADAPT_MIN_ALPHA = 0.85;

// ---- LK Optical Flow (이동 + 잔여 회전 보정) ----
static const int LK_MAX_FEATURES = 200;
static const double LK_QUALITY = 0.01;
static const double LK_MIN_DIST = 30.0;

// ---- 칼만 필터 ----
static const double KF_Q = 0.004;
static const double KF_R = 1.0;
static const double MAX_DIFF_DX = 30.0;
static const double MAX_DIFF_DY = 30.0;
static const double MAX_DIFF_DA = 5.0 * CV_PI / 180.0;

// 크롭
static const double FIXED_CROP_PERCENT = 20.0;

// 카메라 FOV
static const double HFOV_DEG = 62.2;
static const double VFOV_DEG = 48.8;

// IMU 축 매핑
static const int IMU_AXIS_ROLL  = 0;
static const int IMU_AXIS_PITCH = 1;
static const int IMU_AXIS_YAW   = 2;  // 새로 추가
static const int IMU_SIGN_ROLL  = 1;
static const int IMU_SIGN_PITCH = 1;
static const int IMU_SIGN_YAW   = 1;  // 새로 추가
static const int CALIB_SAMPLES = 300;
static const bool DEBUG_OVERLAY = true;
static const int IMU_BUFFER_SIZE = 20;
static const int IMU_STORE_SIZE = 80;

// ======================== 전역 변수 ========================

static std::atomic<bool> g_running{true};

static std::mutex g_mtx;
static GstAppSrc* g_rawsrc = nullptr;
static GstAppSrc* g_stabsrc = nullptr;

static std::mutex g_imu_mtx;
static std::atomic<bool> g_imu_ready{false};

struct ImuPose {
    double roll, pitch, yaw;
    double gyro_roll_rate, gyro_pitch_rate, gyro_yaw_rate;
    double timestamp_ms;
};

// IMU 링버퍼: 최근 샘플 저장 (타임스탬프 포함)
static std::deque<ImuPose> g_imu_buffer;

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
    int sample_count;
    double time_spread_ms;
    double center_offset_ms;
};

static ImuAverageResult imu_average_centered(double frame_time_ms) {
    ImuAverageResult result = {{0, 0, 0, 0, 0, 0, 0}, 0, 0, 0};
    if (g_imu_buffer.empty()) return result;

    const double frame_interval_ms = 1000.0 / G_FPS;
    double t_lo = frame_time_ms - frame_interval_ms;
    double t_hi = frame_time_ms;

    double r = 0, p = 0, y = 0, rr = 0, pr = 0, yr = 0;
    double t_min = 1e18, t_max = -1e18;
    int cnt = 0;

    for (const auto& s : g_imu_buffer) {
        if (s.timestamp_ms >= t_lo && s.timestamp_ms <= t_hi) {
            r += s.roll;
            p += s.pitch;
            y += s.yaw;
            rr += s.gyro_roll_rate;
            pr += s.gyro_pitch_rate;
            yr += s.gyro_yaw_rate;
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
    result.pose = {r / n, p / n, y / n, rr / n, pr / n, yr / n, frame_time_ms};
    result.sample_count = cnt;
    result.time_spread_ms = (cnt > 1) ? (t_max - t_min) : 0;
    result.center_offset_ms = frame_time_ms - (t_min + t_max) / 2.0;
    return result;
}

// 자이로 고주파 필터 상태
struct HighPassState {
    double smooth_roll, smooth_pitch, smooth_yaw;
    bool initialized;
};

// 칼만 필터 (1D) — Phase 2에서 사용 예정
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
    g_imu_ready = true;
    fprintf(stderr, "[IMU] Ready (target: 200Hz, buffer: %d samples)\n", IMU_BUFFER_SIZE);

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

        double roll_rate  = IMU_SIGN_ROLL  * pick_axis(IMU_AXIS_ROLL,  gx_rad, gy_rad, gz_rad);
        double pitch_rate = IMU_SIGN_PITCH * pick_axis(IMU_AXIS_PITCH, gx_rad, gy_rad, gz_rad);
        double yaw_rate   = IMU_SIGN_YAW   * pick_axis(IMU_AXIS_YAW,   gx_rad, gy_rad, gz_rad);
        gyro_roll  += roll_rate  * dt;
        gyro_pitch += pitch_rate * dt;
        gyro_yaw   += yaw_rate   * dt;

        // 샘플 준비 (lock 밖에서)
        double ts = now_ms();
        ImuPose sample = {gyro_roll, gyro_pitch, gyro_yaw, roll_rate, pitch_rate, yaw_rate, ts};

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
    const double fx = G_WIDTH  / (2.0 * tan(HFOV_DEG * CV_PI / 360.0));
    const double fy = G_HEIGHT / (2.0 * tan(VFOV_DEG * CV_PI / 360.0));

    guint64 frameIdx = 0;

    // 자이로 고주파 필터 (roll 보정)
    HighPassState hp = {0, 0, 0, false};

    // LK + 칼만 (원본 프레임 기반 이동/회전 보정)
    Mat prev_gray, prev_frame;
    int lk_count = 0;
    KalmanState kf_theta, kf_tx, kf_ty;
    kalman_init(kf_theta, KF_Q, KF_R);
    kalman_init(kf_tx, KF_Q, KF_R);
    kalman_init(kf_ty, KF_Q, KF_R);

    while (g_running) {
        Mat frame;
        if (!pull_frame(frame) || frame.empty()) continue;

        double frame_time = now_ms();

        // IMU 평균 샘플
        ImuAverageResult imu_result;
        bool imu_ready;
        {
            std::lock_guard<std::mutex> lk(g_imu_mtx);
            imu_result = imu_average_centered(frame_time);
            imu_ready = g_imu_ready.load();
        }
        ImuPose pose = imu_result.pose;

        // 자이로 roll jitter 추출
        double jitter_roll = 0;
        double roll_corr = 0;

        if (imu_ready) {
            if (!hp.initialized) {
                hp.smooth_roll = pose.roll;
                hp.initialized = true;
            } else {
                double diff_r = pose.roll - hp.smooth_roll;
                double alpha_r = adaptiveAlpha(SMOOTH_ALPHA, diff_r, ADAPT_THRESHOLD, ADAPT_RATE, ADAPT_MIN_ALPHA);
                hp.smooth_roll = alpha_r * hp.smooth_roll + (1.0 - alpha_r) * pose.roll;
                jitter_roll = pose.roll - hp.smooth_roll;
                roll_corr = -std::clamp(jitter_roll * ROLL_GAIN, -MAX_ROLL_RAD, MAX_ROLL_RAD);
            }
        } else {
            hp.initialized = false;
        }

        // ============================================================
        // LK OptFlow + 칼만 (원본 프레임 기반)
        // + 자이로 roll 보정을 행렬에 합쳐서 warpAffine 1회만 호출
        // ============================================================

        Mat stabilized;
        double lk_diff_dx = 0, lk_diff_dy = 0, lk_diff_da = 0;

        Mat curr_gray;
        cvtColor(frame, curr_gray, COLOR_BGR2GRAY);

        if (lk_count == 0) {
            prev_gray = curr_gray.clone();
            prev_frame = frame.clone();
            lk_count++;
            stabilized = frame.clone();
        }
        else {
            vector<Point2f> feat_prev, feat_curr;
            goodFeaturesToTrack(prev_gray, feat_prev, LK_MAX_FEATURES, LK_QUALITY, LK_MIN_DIST);

            bool lk_ok = false;

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
                        double sx = affine.at<double>(0, 0) / cos(da);
                        double sy = affine.at<double>(1, 1) / cos(da);

                        kalman_update(kf_theta, da);
                        kalman_update(kf_tx, dx);
                        kalman_update(kf_ty, dy);

                        if (lk_count >= 2) {
                            lk_diff_da = kalman_diff(kf_theta, MAX_DIFF_DA);
                            lk_diff_dx = kalman_diff(kf_tx, MAX_DIFF_DX);
                            lk_diff_dy = kalman_diff(kf_ty, MAX_DIFF_DY);
                        }
                        lk_count++;

                        // LK smoothed 변환 행렬 (prev_frame → 안정화 프레임)
                        double sda = da + lk_diff_da;
                        double sdx = dx + lk_diff_dx;
                        double sdy = dy + lk_diff_dy;

                        Mat smoothed = (Mat_<double>(2, 3) << sx * cos(sda), sx * -sin(sda), sdx,
                                        sy * sin(sda), sy * cos(sda), sdy);

                        // 자이로 roll 보정 행렬 (프레임 중심 기준 회전)
                        if (std::abs(roll_corr) > 0.0003) {  // ~0.02도
                            double cos_r = cos(roll_corr);
                            double sin_r = sin(roll_corr);
                            double gtx = (1.0 - cos_r) * cx_cam + sin_r * cy_cam;
                            double gty = -sin_r * cx_cam + (1.0 - cos_r) * cy_cam;

                            Mat T_gyro = (Mat_<double>(2, 3) << cos_r, -sin_r, gtx,
                                          sin_r, cos_r, gty);

                            // 행렬 합성: T_gyro × smoothed (3x3 확장 후 곱셈)
                            Mat S3 = Mat::eye(3, 3, CV_64F);
                            smoothed.copyTo(S3(Rect(0, 0, 3, 2)));
                            Mat G3 = Mat::eye(3, 3, CV_64F);
                            T_gyro.copyTo(G3(Rect(0, 0, 3, 2)));
                            Mat combined3 = G3 * S3;
                            smoothed = combined3(Rect(0, 0, 3, 2)).clone();
                        }

                        warpAffine(prev_frame, stabilized, smoothed, frame.size());
                        lk_ok = true;
                    }
                }
            }

            if (!lk_ok) {
                stabilized = frame.clone();
            }

            prev_gray = curr_gray.clone();
            prev_frame = frame.clone();
        }

        // 고정 크롭
        stabilized = centerCropAndResize(stabilized, FIXED_CROP_PERCENT);

        // 디버그 오버레이
        if (DEBUG_OVERLAY) {
            char buf[256];
            const int font = FONT_HERSHEY_SIMPLEX;

            if (imu_ready) {
                snprintf(buf, sizeof(buf), "Roll jit:%+5.2f  corr:%+5.2f",
                         jitter_roll * 180 / CV_PI, roll_corr * 180 / CV_PI);
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
        GstAppSrc *rawsrc, *stabsrc;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            rawsrc = g_rawsrc;
            stabsrc = g_stabsrc;
        }

        push_bgr(rawsrc, frame, frameIdx, "raw");
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

    GstRTSPMediaFactory* f_raw = make_factory("rawsrc");
    g_signal_connect(f_raw, "media-configure", (GCallback)on_media_configure, (gpointer)"raw");
    gst_rtsp_mount_points_add_factory(mounts, "/raw", f_raw);

    GstRTSPMediaFactory* f_stab = make_factory("stabsrc");
    g_signal_connect(f_stab, "media-configure", (GCallback)on_media_configure, (gpointer)"stab");
    gst_rtsp_mount_points_add_factory(mounts, "/cam", f_stab);

    g_object_unref(mounts);

    if (gst_rtsp_server_attach(server, nullptr) == 0) {
        cerr << "[ERR] RTSP attach\n";
        return -1;
    }

    fprintf(stderr, "==============================\n");
    fprintf(stderr, " EIS — Hybrid (Gyro Roll + LK, Single Warp)\n");
    fprintf(stderr, "==============================\n");
    fprintf(stderr, "  Gyro: roll HF filter (alpha=%.3f, max=±%.1f°)\n", SMOOTH_ALPHA, MAX_ROLL_RAD * 180 / CV_PI);
    fprintf(stderr, "  LK: OptFlow + Kalman (Q=%.4f R=%.1f)\n", KF_Q, KF_R);
    fprintf(stderr, "  Crop: %.0f%%\n", FIXED_CROP_PERCENT);
    fprintf(stderr, "  rtsp://<PI_IP>:8555/raw | /cam\n");
    fprintf(stderr, "==============================\n");

    std::thread imu_th(imu_loop);
    std::thread cap_th(capture_loop);

    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop);

    g_running = false;
    if (cap_th.joinable()) cap_th.join();
    if (imu_th.joinable()) imu_th.join();
    g_main_loop_unref(loop);
    return 0;
}