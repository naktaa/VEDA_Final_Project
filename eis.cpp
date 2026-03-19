/**
 * eis_opus_test.cpp ???òÏù¥Î∏åÎ¶¨??EIS (LK OptFlow + ?ôÏ†Å ÏπºÎßå + ?êÏù¥Î°?Í≥†Ï£º??Î∂Ä?§Ìä∏)
 *
 * Î≥¥Ï†ï ?êÎ¶¨ (2?®Í≥Ñ):
 *   1?®Í≥Ñ: LK Optical Flow + ?ôÏ†Å ÏπºÎßå ?ÑÌÑ∞ ??prev_frame??smoothed Î≥Ä???ÅÏö©
 *          - ???ÄÏßÅÏûÑ: Q??R??(Îπ†Î•¥Í≤??∞ÎùºÍ∞????¨Î∞± ÏµúÏÜå??
 *          - ?ëÏ? ?®Î¶º: Q??R??(Í∞ïÌïòÍ≤??âÌôú ??Î∂Ä?úÎü¨???ÅÏÉÅ)
 *          - ?úÍ∑∏Î™®Ïù¥??Î∏îÎ†å?©ÏúºÎ°?Î∂Ä?úÎü¨???ÑÌôò
 *   2?®Í≥Ñ: ?êÏù¥Î°?Í≥†Ï£º???ÑÌÑ∞ ??1?®Í≥Ñ Í≤∞Í≥º??Ï∂îÍ? ?åÏ†Ñ Î≥¥Ï†ï
 *          (30fps ?¨Ïù¥??Í≥†Ï£º??ÏßÑÎèôÎß??°Ïùå)
 *
 * RTSP Ï∂úÎ†•:
 *   /raw  ???êÎ≥∏ ?ÅÏÉÅ
 *   /cam  ??EIS Î≥¥Ï†ï ?ÅÏÉÅ
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

// ======================== ?§Ï†ï ?ÅÏàò ========================

static const int G_WIDTH = 640;
static const int G_HEIGHT = 480;
static const int G_FPS = 20;

// MPU-6050
static const int IMU_ADDR = 0x68;
static const double GYRO_SENSITIVITY = 131.0;

// ---- ?êÏù¥Î°?Í≥†Ï£º???ÑÌÑ∞ (2?®Í≥Ñ: Ï∂îÍ? ?åÏ†Ñ Î≥¥Ï†ï) ----
static const double SMOOTH_ALPHA = 0.99;
static const double ROLL_GAIN = 1.0;
static const double PITCH_GAIN = 0.5;
static const double MAX_ROLL_RAD = 8.0 * CV_PI / 180.0;  // 5????(?¨Ïú† ?ïÎ?)
static const double MAX_PITCH_RAD = 5.0 * CV_PI / 180.0; // 3????(?¨Ïú† ?ïÎ?)

// ?ÅÏùë??Alpha
static const double ADAPT_THRESHOLD = 5.0 * CV_PI / 180.0;
static const double ADAPT_RATE = 15.0;
static const double ADAPT_MIN_ALPHA = 0.80;

// ---- LK Optical Flow (1?®Í≥Ñ: Î©îÏù∏ Î≥¥Ï†ï) ----
static const int LK_MAX_FEATURES = 200;
static const double LK_QUALITY = 0.01;
static const double LK_MIN_DIST = 30.0;

// ---- ÏπºÎßå ?ÑÌÑ∞ (?àÌçº?∞Ïä§ Í≥†Ï†ïÍ∞? ----
static const double KF_Q = 0.02; // ?àÌçº?∞Ïä§ ?êÎ≥∏
static const double KF_R = 0.5;  // ?àÌçº?∞Ïä§ ?êÎ≥∏
// kalman_diff ?¥Îû®??(ÏµúÎ? warp ?úÌïú ???¨Î∞± Î∞©Ï?)
static const double MAX_DIFF_DX = 9999.0;                // pixels
static const double MAX_DIFF_DY = 9999.0;                // pixels
static const double MAX_DIFF_DA = 999.0 * CV_PI / 180.0; // radians (~4??

// ?¨Î°≠
static const double FIXED_CROP_PERCENT = 20.0; // 20??5% (?îÍ∞Å ?ïÎ≥¥)

// Ïπ¥Î©î??FOV
static const double HFOV_DEG = 62.2;
static const double VFOV_DEG = 48.8;

// IMU Ï∂?Îß§Ìïë
static const int IMU_AXIS_ROLL = 0;
static const int IMU_AXIS_PITCH = 1;
static const int IMU_SIGN_ROLL = 1;
static const int IMU_SIGN_PITCH = 1;
static const int CALIB_SAMPLES = 300;
static const bool DEBUG_OVERLAY = false;
static const int IMU_BUFFER_SIZE = 20; // 1KHz √ó 40 = ~40ms ??1?ÑÎ†à??33ms) + ?¨Ïú†
static const int IMU_STORE_SIZE = 80;  // Ï§ëÏã¨ ?âÍ∑†???ÑÌï¥ ?¨Ïú† ?àÍ≤å ?Ä??

// ======================== ?ÑÏó≠ Î≥Ä??========================

static std::atomic<bool> g_running{true};

static std::mutex g_mtx;
static GstAppSrc* g_rawsrc = nullptr;
static GstAppSrc* g_stabsrc = nullptr;

static std::mutex g_imu_mtx;
static std::atomic<bool> g_imu_ready{false};

struct ImuPose {
    double roll, pitch;
    double gyro_roll_rate, gyro_pitch_rate;
    double timestamp_ms; // steady_clock Í∏∞Ï? ms
};

// IMU ÎßÅÎ≤Ñ?? ÏµúÍ∑º ?òÌîå ?Ä??(?Ä?ÑÏä§?¨ÌîÑ ?¨Ìï®)
static std::deque<ImuPose> g_imu_buffer;
static std::atomic<double> g_imu_actual_hz{0}; // ?§Ï∏° ?òÌîåÎß??àÏù¥??(atomic ??lock Î∂àÌïÑ??

// ?ÑÎ°úÍ∑∏Îû® ?úÏûë ?úÏ†ê Í∏∞Ï? ms Î∞òÌôò
static auto g_time_origin = std::chrono::steady_clock::now();
static double now_ms() {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - g_time_origin)
        .count();
}

// ?ÑÎ†à??Ï§ëÏã¨ N/2 ?âÍ∑†: frame_time_ms Í∏∞Ï? ¬±half_window_ms Î≤îÏúÑ???òÌîå ?âÍ∑†
struct ImuAverageResult {
    ImuPose pose;
    int sample_count;        // ?âÍ∑†???¨Ïö©???òÌîå ??
    double time_spread_ms;   // ?¨Ïö©???òÌîå?§Ïùò ?úÍ∞Ñ Î≤îÏúÑ
    double center_offset_ms; // ?§Ï†ú Ï§ëÏã¨Í≥??ÑÎ†à???úÍ∞Ñ??Ï∞®Ïù¥
};

static ImuAverageResult imu_average_centered(double frame_time_ms) {
    // g_imu_mtx ?†Í∏¥ ?ÅÌÉú?êÏÑú ?∏Ï∂ú
    ImuAverageResult result = {{0, 0, 0, 0, 0}, 0, 0, 0};
    if (g_imu_buffer.empty()) return result;

    // ?ÑÎ†à??ÏßÅÏ†Ñ 1?ÑÎ†à??Í∞ÑÍ≤©(~33ms) ?àÎèÑ??(ÏßÄ???ÜÏùå)
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

    // Î≤îÏúÑ ???òÌîå???ÜÏúºÎ©?Í∞Ä??ÏµúÍ∑º ?òÌîå ?¨Ïö© (fallback)
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

// ?êÏù¥Î°?Í≥†Ï£º???ÑÌÑ∞ ?ÅÌÉú
struct HighPassState {
    double smooth_roll, smooth_pitch;
    bool initialized;
};

// ÏπºÎßå ?ÑÌÑ∞ (1D) ???àÌçº?∞Ïä§ Í≥†Ï†ïÍ∞?+ diff ?¥Îû®??
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

// ======================== ?†Ìã∏Î¶¨Ìã∞ ?®Ïàò ========================

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

// ======================== RTSP ?§Ï†ï ========================

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
    const bool is_cam = (strcmp(appsrc_name, "stabsrc") == 0);
    const int video_bitrate = is_cam ? 1000000 : 1500000;
    const int i_frame_period = is_cam ? 20 : 30;
    std::string launch =
        "( appsrc name=" + std::string(appsrc_name) + " is-live=true format=time do-timestamp=true block=false "
                                                      "! videoconvert "
                                                      "! video/x-raw,format=I420 "
                                                      "! v4l2h264enc extra-controls=\"controls,video_bitrate=" + std::to_string(video_bitrate) + ",h264_i_frame_period=" + std::to_string(i_frame_period) + "\" "
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

// ======================== IMU ?∞Î†à??========================

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

        // ?òÌîå Ï§ÄÎπ?(lock Î∞ñÏóê??
        double ts = now_ms();
        ImuPose sample = {gyro_roll, gyro_pitch, roll_rate, pitch_rate, ts};

        { // ÏµúÏÜå ?ÑÍ≥ÑÍµ¨Í∞Ñ: push + popÎß?
            std::lock_guard<std::mutex> lk(g_imu_mtx);
            g_imu_buffer.push_back(sample);
            if ((int)g_imu_buffer.size() > IMU_STORE_SIZE)
                g_imu_buffer.pop_front();
        }
        usleep(5000); // 200Hz (Pi CPU Î∂Ä??Í≥†Î†§)
    }
    close(fd);
    fprintf(stderr, "[IMU] Thread exiting\n");
}

// ======================== Ï∫°Ï≤ò + EIS Î≥¥Ï†ï Î£®ÌîÑ ========================

static void capture_loop() {
    std::string cap_pipe =
        "libcamerasrc "
        "! video/x-raw,format=RGBx,width=640,height=480,framerate=" + std::to_string(G_FPS) + "/1 "
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

    guint64 frameIdx = 0;

    while (g_running) {
        Mat frame;
        if (!pull_frame(frame) || frame.empty()) continue;

        GstAppSrc* rawsrc;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            rawsrc = g_rawsrc;
        }

        push_bgr(rawsrc, frame, frameIdx, "raw");
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
    g_signal_connect(f_raw, "media-configure", (GCallback)on_media_configure, (gpointer) "raw");
    gst_rtsp_mount_points_add_factory(mounts, "/raw", f_raw);

    g_object_unref(mounts);

    if (gst_rtsp_server_attach(server, nullptr) == 0) {
        cerr << "[ERR] RTSP attach\n";
        return -1;
    }

    fprintf(stderr, "==============================\n");
    fprintf(stderr, " RAW stream only (no LK/EIS/IMU correction)\n");
    fprintf(stderr, "==============================\n");
    fprintf(stderr, "  rtsp://<PI_IP>:8555/raw\n");
    fprintf(stderr, "==============================\n");

    std::thread cap_th(capture_loop);

    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop);

    g_running = false;
    if (cap_th.joinable()) cap_th.join();
    g_main_loop_unref(loop);
    return 0;
}
