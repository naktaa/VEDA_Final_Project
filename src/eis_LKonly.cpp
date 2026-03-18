/**
 * eis_LKonly.cpp - LK Optical Flow + Kalman (reference-based) with HW H.264 RTSP
 *   - Capture: GStreamer libcamerasrc
 *   - Stabilize: LK + Kalman (dx/dy/da)
 *   - Output: RTSP /cam only (hardware encoding pipeline)
 */

#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/video/video.h>

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "tank_drive.hpp"
#include "eis_config.hpp"

using namespace std;
using namespace cv;

// ======================== Settings ========================

static const int    G_WIDTH  = 640;
static const int    G_HEIGHT = 480;
static int          G_FPS    = 20;

static int    BORDER_CROP = 30;

static int    MAX_FEATURES = 200;
static double FEATURE_QUALITY = 0.01;
static double FEATURE_MIN_DIST = 30.0;

static double KF_Q = 0.004;
static double KF_R = 0.5;

static bool   DEBUG_OVERLAY = false;
static bool   FLIP_BOTH = true;

// ======================== Global ========================

static std::atomic<bool> g_running{true};
static std::mutex g_mtx;
static GstAppSrc* g_stabsrc = nullptr;

// ======================== Kalman ========================

struct KalmanState {
    double x;
    double P;
    double Q;
    double R;
    double sum;
};

static void kalman_init(KalmanState& kf, double q, double r) {
    kf.x = 0.0;
    kf.P = 1.0;
    kf.Q = q;
    kf.R = r;
    kf.sum = 0.0;
}

static void kalman_update(KalmanState& kf, double measurement) {
    kf.sum += measurement;
    double P_pred = kf.P + kf.Q;
    double K = P_pred / (P_pred + kf.R);
    kf.x = kf.x + K * (kf.sum - kf.x);
    kf.P = (1.0 - K) * P_pred;
}

static double kalman_diff(const KalmanState& kf) {
    return kf.x - kf.sum;
}

// ======================== Utilities ========================

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

// ======================== RTSP ========================

static void set_appsrc_caps(GstAppSrc* appsrc) {
    GstCaps* caps = gst_caps_new_simple(
        "video/x-raw",
        "format",    G_TYPE_STRING,  "BGR",
        "width",     G_TYPE_INT,     G_WIDTH,
        "height",    G_TYPE_INT,     G_HEIGHT,
        "framerate", GST_TYPE_FRACTION, G_FPS, 1,
        nullptr);
    gst_app_src_set_caps(appsrc, caps);
    gst_caps_unref(caps);
    g_object_set(G_OBJECT(appsrc),
                 "is-live",      TRUE,
                 "format",       GST_FORMAT_TIME,
                 "do-timestamp", TRUE,
                 "block",        FALSE,
                 nullptr);
}

static void on_media_configure(GstRTSPMediaFactory*, GstRTSPMedia* media, gpointer user_data) {
    const char* name = static_cast<const char*>(user_data);
    GstElement* element = gst_rtsp_media_get_element(media);
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
        g_stabsrc = appsrc;
        g_print("[RTSP] /cam connected\n");
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

static bool push_bgr(GstAppSrc* appsrc, const Mat& frame, guint64 idx) {
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
        g_printerr("[push] failed (frame=%" G_GUINT64_FORMAT ")\n", idx);
        return false;
    }
    return true;
}

static void sigint_handler(int) { g_running = false; }

static void apply_config_map(const ConfigMap& cfg) {
    auto get = [&](const char* k) -> const std::string* {
        auto it = cfg.find(k);
        if (it == cfg.end()) return nullptr;
        return &it->second;
    };
    auto set_int = [&](const char* k, int& dst) {
        if (auto v = get(k)) dst = std::stoi(*v);
    };
    auto set_double = [&](const char* k, double& dst) {
        if (auto v = get(k)) dst = std::stod(*v);
    };
    auto set_bool = [&](const char* k, bool& dst) {
        if (auto v = get(k)) {
            std::string s = *v;
            std::transform(s.begin(), s.end(), s.begin(), ::tolower);
            dst = (s == "1" || s == "true" || s == "yes" || s == "on");
        }
    };

    set_int("lk.fps", G_FPS);
    set_int("lk.border_crop", BORDER_CROP);
    set_int("lk.max_features", MAX_FEATURES);
    set_double("lk.feature_quality", FEATURE_QUALITY);
    set_double("lk.feature_min_dist", FEATURE_MIN_DIST);
    set_double("lk.kf_q", KF_Q);
    set_double("lk.kf_r", KF_R);
    set_bool("lk.debug_overlay", DEBUG_OVERLAY);
    set_bool("lk.flip_both", FLIP_BOTH);
}

static void keyboard_loop() {
    if (!isatty(STDIN_FILENO)) {
        fprintf(stderr, "[KEY] stdin is not a TTY. Keyboard control disabled.\n");
        return;
    }

    termios oldt{};
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

    tank_drive::init();
    fprintf(stderr, "[KEY] WASD/QE + Space/X stop, +/- speed, H help\n");

    pollfd pfd;
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;

    while (g_running) {
        int r = poll(&pfd, 1, 50);
        if (r > 0 && (pfd.revents & POLLIN)) {
            char ch = 0;
            ssize_t n = read(STDIN_FILENO, &ch, 1);
            if (n == 1) {
                tank_drive::handle_key(ch);
            }
        }
        tank_drive::tick();
    }

    tank_drive::shutdown();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

// ======================== Capture + Stabilize ========================

static void capture_loop() {
    std::string cap_pipe =
        "libcamerasrc ! "
        "video/x-raw,width=" + std::to_string(G_WIDTH) +
        ",height=" + std::to_string(G_HEIGHT) +
        ",framerate=" + std::to_string(G_FPS) + "/1 "
        "! videoconvert ! video/x-raw,format=BGR "
        "! appsink name=appsink emit-signals=false sync=false max-buffers=1 drop=true";

    GError* err = nullptr;
    GstElement* pipeline = gst_parse_launch(cap_pipe.c_str(), &err);
    if (!pipeline) {
        fprintf(stderr, "[ERR] Failed to create capture pipeline\n");
        if (err) g_error_free(err);
        g_running = false;
        return;
    }
    if (err) {
        fprintf(stderr, "[WARN] %s\n", err->message);
        g_error_free(err);
    }

    GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink");
    if (!sink) {
        fprintf(stderr, "[ERR] Failed to get appsink\n");
        gst_object_unref(pipeline);
        g_running = false;
        return;
    }
    GstAppSink* appsink = GST_APP_SINK(sink);
    gst_app_sink_set_emit_signals(appsink, FALSE);
    gst_app_sink_set_drop(appsink, TRUE);
    gst_app_sink_set_max_buffers(appsink, 1);

    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "[ERR] Failed to start pipeline\n");
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

    KalmanState kf_theta, kf_tx, kf_ty;
    kalman_init(kf_theta, KF_Q, KF_R);
    kalman_init(kf_tx, KF_Q, KF_R);
    kalman_init(kf_ty, KF_Q, KF_R);

    Mat prev_gray;
    Mat prev_frame;
    int frame_count = 0;
    guint64 frameIdx = 0;

    int vert_border = BORDER_CROP * G_HEIGHT / G_WIDTH;

    while (g_running) {
        Mat frame;
        if (!pull_frame(frame) || frame.empty()) continue;

        Mat curr_gray;
        cvtColor(frame, curr_gray, COLOR_BGR2GRAY);

        Mat stabilized;
        // Flip both vertically and horizontally (camera mounted inverted)
        if (FLIP_BOTH) {
            cv::flip(frame, frame, -1);
            cv::flip(curr_gray, curr_gray, -1);
        }
        if (frame_count == 0) {
            prev_gray = curr_gray.clone();
            prev_frame = frame.clone();
            frame_count++;
            stabilized = frame.clone();
        } else {
            vector<Point2f> features_prev, features_curr;
            vector<uchar> status;
            vector<float> err_vec;

            goodFeaturesToTrack(prev_gray, features_prev, MAX_FEATURES, FEATURE_QUALITY, FEATURE_MIN_DIST);
            if (features_prev.size() < 10) {
                prev_gray = curr_gray.clone();
                prev_frame = frame.clone();
                stabilized = frame.clone();
            } else {
                calcOpticalFlowPyrLK(prev_gray, curr_gray, features_prev, features_curr, status, err_vec);
                vector<Point2f> good_prev, good_curr;
                for (size_t i = 0; i < status.size(); i++) {
                    if (status[i]) {
                        good_prev.push_back(features_prev[i]);
                        good_curr.push_back(features_curr[i]);
                    }
                }
                if (good_prev.size() < 6) {
                    prev_gray = curr_gray.clone();
                    prev_frame = frame.clone();
                    stabilized = frame.clone();
                } else {
                    Mat affine = estimateAffinePartial2D(good_prev, good_curr);
                    if (affine.empty()) {
                        prev_gray = curr_gray.clone();
                        prev_frame = frame.clone();
                        stabilized = frame.clone();
                    } else {
                        double dx = affine.at<double>(0, 2);
                        double dy = affine.at<double>(1, 2);
                        double da = atan2(affine.at<double>(1, 0), affine.at<double>(0, 0));

                        double sx = affine.at<double>(0, 0) / cos(da);
                        double sy = affine.at<double>(1, 1) / cos(da);

                        kalman_update(kf_theta, da);
                        kalman_update(kf_tx, dx);
                        kalman_update(kf_ty, dy);

                        double diff_da = 0.0, diff_dx = 0.0, diff_dy = 0.0;
                        if (frame_count == 1) {
                            frame_count++;
                        } else {
                            diff_da = kalman_diff(kf_theta);
                            diff_dx = kalman_diff(kf_tx);
                            diff_dy = kalman_diff(kf_ty);
                        }

                        da += diff_da;
                        dx += diff_dx;
                        dy += diff_dy;

                        Mat smoothed = (Mat_<double>(2, 3) <<
                            sx * cos(da), sx * -sin(da), dx,
                            sy * sin(da), sy *  cos(da), dy);

                        warpAffine(prev_frame, stabilized, smoothed, frame.size());

                        stabilized = stabilized(
                            Range(vert_border, stabilized.rows - vert_border),
                            Range(BORDER_CROP, stabilized.cols - BORDER_CROP));
                        resize(stabilized, stabilized, frame.size());

                        if (DEBUG_OVERLAY) {
                            char buf[128];
                            snprintf(buf, sizeof(buf),
                                     "dx:%+5.1f dy:%+5.1f da:%+5.2f deg",
                                     diff_dx, diff_dy, diff_da * 180.0 / CV_PI);
                            putText(stabilized, buf, Point(8, 18),
                                    FONT_HERSHEY_SIMPLEX, 0.45,
                                    Scalar(0, 255, 0), 1, LINE_AA);
                        }
                    }
                }
                prev_gray = curr_gray.clone();
                prev_frame = frame.clone();
            }
        }

        GstAppSrc* stabsrc = nullptr;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            stabsrc = g_stabsrc;
        }
        push_bgr(stabsrc, stabilized, frameIdx);
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

    // Load local config for LK-only
    std::string cfg_path = "config_lk_local.ini";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) {
            cfg_path = argv[i + 1];
            break;
        } else if (a.rfind("--config=", 0) == 0) {
            cfg_path = a.substr(9);
            break;
        }
    }
    ConfigMap cfg;
    if (!load_config_if_exists(cfg_path, cfg)) {
        // fallback to shared local config
        load_config_if_exists("config_local.ini", cfg);
    }
    if (!cfg.empty()) {
        fprintf(stderr, "[CFG] loaded %s\n", cfg_path.c_str());
        apply_config_map(cfg);
    } else {
        fprintf(stderr, "[CFG] no config (%s)\n", cfg_path.c_str());
    }

    GstRTSPServer* server = gst_rtsp_server_new();
    gst_rtsp_server_set_service(server, "8555");
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);

    GstRTSPMediaFactory* f_stab = make_factory("stabsrc");
    g_signal_connect(f_stab, "media-configure", (GCallback)on_media_configure, (gpointer) "stabsrc");
    gst_rtsp_mount_points_add_factory(mounts, "/cam", f_stab);
    g_object_unref(mounts);

    if (gst_rtsp_server_attach(server, nullptr) == 0) {
        fprintf(stderr, "[ERR] Failed to attach RTSP server\n");
        return -1;
    }

    fprintf(stderr, "==============================\n");
    fprintf(stderr, " EIS LK-only (Reference)\n");
    fprintf(stderr, "==============================\n");
    fprintf(stderr, "RTSP: rtsp://<PI_IP>:8555/cam\n");
    fprintf(stderr, "Settings:\n");
    fprintf(stderr, "  %dx%d @ %dfps\n", G_WIDTH, G_HEIGHT, G_FPS);
    fprintf(stderr, "  Features: %d  quality=%.3f  min_dist=%.0f\n",
            MAX_FEATURES, FEATURE_QUALITY, FEATURE_MIN_DIST);
    fprintf(stderr, "  Kalman Q=%.4f  R=%.1f\n", KF_Q, KF_R);
    fprintf(stderr, "  Border crop: %d px\n", BORDER_CROP);
    fprintf(stderr, "==============================\n");

    std::thread key_th(keyboard_loop);
    std::thread cap_th(capture_loop);
    GMainLoop* loop = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(loop);

    g_running = false;
    cap_th.join();
    if (key_th.joinable()) key_th.join();
    g_main_loop_unref(loop);
    return 0;
}
