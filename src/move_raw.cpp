#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/gstmeta.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/video/video.h>

#include <algorithm>
#include <cctype>
#include <csignal>
#include <string>
#include <thread>

#include "eis_common.hpp"
#include "eis_config.hpp"
#include "eis_globals.hpp"
#include "eis_input.hpp"
#include "libcamera_grabber.hpp"
#include "rtsp_server.hpp"

namespace {

struct CapturedFrame {
    cv::Mat frame;
    double time_ms = 0.0;
    TsSource src = TsSource::ARRIVAL;
    int64_t sensor_ts_ns = 0;
    int exp_us = 0;
    uint64_t index = 0;
};

GMainLoop* g_main_loop_ptr = nullptr;

void sigint_handler(int) {
    g_running = false;
    if (g_main_loop_ptr) g_main_loop_quit(g_main_loop_ptr);
}

bool parse_ts_source_pref(const std::string& in, int& out_pref) {
    std::string s = in;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if (s == "sensor") out_pref = (int)TsSourcePref::SENSOR;
    else if (s == "pts") out_pref = (int)TsSourcePref::PTS;
    else if (s == "arrival") out_pref = (int)TsSourcePref::ARRIVAL;
    else if (s == "auto") out_pref = (int)TsSourcePref::AUTO; // compatibility alias
    else return false;
    return true;
}

void print_usage(const char* prog) {
    fprintf(stderr, "Usage: %s [options]\n", prog);
    fprintf(stderr, "  --config <path>\n");
    fprintf(stderr, "  --ts-source {sensor|pts|arrival} (auto alias supported)\n");
    fprintf(stderr, "  --libcamera-xrgb {0|1}\n");
}

void apply_config_map(const ConfigMap& cfg) {
    auto get = [&](const char* k) -> const std::string* {
        auto it = cfg.find(k);
        if (it == cfg.end()) return nullptr;
        return &it->second;
    };

    if (auto v = get("ts_source")) {
        int pref = g_ts_pref.load();
        if (parse_ts_source_pref(*v, pref)) g_ts_pref = pref;
    }

    if (auto v = get("libcamera_xrgb")) {
        std::string s = *v;
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        g_libcamera_xrgb = (s == "1" || s == "true" || s == "yes" || s == "on");
    }
}

bool parse_args(int argc, char* argv[]) {
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
        } else if (a == "--config") {
            if (!eat_value(val)) return false;
            continue;
        } else if (a.rfind("--config=", 0) == 0) {
            continue;
        } else if (a == "--ts-source") {
            if (!eat_value(val)) return false;
            int pref = 0;
            if (!parse_ts_source_pref(val, pref)) return false;
            g_ts_pref = pref;
        } else if (a.rfind("--ts-source=", 0) == 0) {
            int pref = 0;
            if (!parse_ts_source_pref(a.substr(12), pref)) return false;
            g_ts_pref = pref;
        } else if (a == "--libcamera-xrgb") {
            if (!eat_value(val)) return false;
            g_libcamera_xrgb = (std::stoi(val) != 0);
        } else if (a.rfind("--libcamera-xrgb=", 0) == 0) {
            g_libcamera_xrgb = (std::stoi(a.substr(18)) != 0);
        } else {
            fprintf(stderr, "[ERR] Unknown arg: %s\n", a.c_str());
            return false;
        }
    }
    return true;
}

bool get_reference_timestamp_ns(GstBuffer* buffer, int64_t& out_ns) {
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

void raw_capture_loop() {
    TsSourcePref pref = (TsSourcePref)g_ts_pref.load();
    bool want_libcamera = (pref == TsSourcePref::AUTO || pref == TsSourcePref::SENSOR);
    bool use_libcamera = false;
    LibcameraGrabber lc;

    GstElement* pipeline = nullptr;
    GstElement* sink = nullptr;
    GstAppSink* appsink = nullptr;
    GError* err = nullptr;

    bool sensor_meta_warned = false;

    if (want_libcamera) {
        if (lc.init()) {
            use_libcamera = true;
        } else {
            fprintf(stderr, "[TS] libcamera init failed -> fallback GStreamer\n");
        }
    }

    if (!use_libcamera) {
        std::string cap_pipe =
            "libcamerasrc "
            "! video/x-raw,format=RGBx,width=" + std::to_string(G_WIDTH) +
            ",height=" + std::to_string(G_HEIGHT) +
            ",framerate=" + std::to_string(G_FPS) + "/1 "
            "! videoconvert ! video/x-raw,format=BGR "
            "! appsink name=appsink drop=true max-buffers=1 sync=false";

        pipeline = gst_parse_launch(cap_pipe.c_str(), &err);
        if (!pipeline) {
            fprintf(stderr, "[ERR] capture pipeline create failed\n");
            if (err) g_error_free(err);
            g_running = false;
            return;
        }
        if (err) {
            fprintf(stderr, "[WARN] %s\n", err->message);
            g_error_free(err);
        }

        sink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink");
        if (!sink) {
            fprintf(stderr, "[ERR] appsink not found\n");
            gst_object_unref(pipeline);
            g_running = false;
            return;
        }

        appsink = GST_APP_SINK(sink);
        gst_app_sink_set_emit_signals(appsink, FALSE);
        gst_app_sink_set_drop(appsink, TRUE);
        gst_app_sink_set_max_buffers(appsink, 1);

        if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            fprintf(stderr, "[ERR] capture pipeline start failed\n");
            gst_object_unref(sink);
            gst_object_unref(pipeline);
            g_running = false;
            return;
        }
    }

    auto pull_frame_gst = [&](CapturedFrame& out) -> bool {
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

        cv::Mat frame(h, w, CV_8UC3, map.data, stride);
        out.frame = frame.clone();
        gst_buffer_unmap(buffer, &map);

        out.src = TsSource::ARRIVAL;
        out.time_ms = now_ms();

        int64_t ref_ns = 0;
        if (get_reference_timestamp_ns(buffer, ref_ns)) {
            out.src = TsSource::SENSOR;
            out.time_ms = (ref_ns - g_time_origin_raw_ns) / 1e6;
        } else if (GST_CLOCK_TIME_IS_VALID(GST_BUFFER_PTS(buffer))) {
            out.src = TsSource::PTS;
            out.time_ms = (int64_t)GST_BUFFER_PTS(buffer) / 1e6;
        }

        gst_sample_unref(sample);
        return !out.frame.empty();
    };

    auto pull_frame_lc = [&](CapturedFrame& out) -> bool {
        if (!lc.get_frame(out.frame,
                          out.time_ms,
                          out.src,
                          out.sensor_ts_ns,
                          out.exp_us,
                          out.index)) {
            return false;
        }

        if (out.src != TsSource::SENSOR && !sensor_meta_warned) {
            fprintf(stderr, "[TS] SensorTimestamp missing -> using ARRIVAL\n");
            sensor_meta_warned = true;
        }
        return !out.frame.empty();
    };

    auto pull_frame = [&](CapturedFrame& out) -> bool {
        return use_libcamera ? pull_frame_lc(out) : pull_frame_gst(out);
    };

    TsSource last_ts_src = TsSource::ARRIVAL;
    guint64 frame_idx = 0;

    while (g_running) {
        CapturedFrame cap;
        if (!pull_frame(cap) || cap.frame.empty()) continue;

        cv::Mat frame = cap.frame;
        if (FLIP_VERTICAL) cv::flip(frame, frame, -1);

        if (cap.src != last_ts_src) {
            fprintf(stderr, "[TS] source=%s\n", ts_source_str(cap.src));
            last_ts_src = cap.src;
        }

        GstAppSrc* outsrc = nullptr;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            outsrc = g_stabsrc;
        }

        if (outsrc) {
            push_bgr(outsrc, frame, frame_idx, "raw");
        }
        frame_idx++;
    }

    if (use_libcamera) {
        lc.shutdown();
    } else {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(sink);
        gst_object_unref(pipeline);
    }

    fprintf(stderr, "[RAW] capture thread exiting\n");
}

} // namespace

int main(int argc, char* argv[]) {
    system("fuser -k 8555/tcp 2>/dev/null");
    signal(SIGINT, sigint_handler);

    std::string cfg_path = "config_local.ini";
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
    if (load_config_if_exists(cfg_path, cfg)) {
        fprintf(stderr, "[CFG] loaded %s\n", cfg_path.c_str());
        apply_config_map(cfg);
    } else {
        fprintf(stderr, "[CFG] no config (%s)\n", cfg_path.c_str());
    }

    if (!parse_args(argc, argv)) return 0;

    gst_init(&argc, &argv);

    GstRTSPServer* server = gst_rtsp_server_new();
    gst_rtsp_server_set_service(server, "8555");
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);

    GstRTSPMediaFactory* f_raw = make_factory("stabsrc");
    g_signal_connect(f_raw, "media-configure", (GCallback)on_media_configure, (gpointer)"stab");
    gst_rtsp_mount_points_add_factory(mounts, "/cam", f_raw);

    g_object_unref(mounts);

    if (gst_rtsp_server_attach(server, nullptr) == 0) {
        fprintf(stderr, "[ERR] RTSP attach failed\n");
        return -1;
    }

    fprintf(stderr, "==============================\n");
    fprintf(stderr, " RAW MOVE STREAM (NO EIS)\n");
    fprintf(stderr, "==============================\n");
    fprintf(stderr, "  TS source pref: %s\n", ts_pref_str((TsSourcePref)g_ts_pref.load()));
    fprintf(stderr, "  Libcamera XRGB: %s\n", g_libcamera_xrgb.load() ? "true" : "false");
    fprintf(stderr, "  Output: rtsp://<PI_IP>:8555/cam\n");
    fprintf(stderr, "  Keyboard: WASD/QE + Space/X (+/- speed, H help)\n");
    fprintf(stderr, "==============================\n");

    std::thread key_th(keyboard_loop);
    std::thread cap_th(raw_capture_loop);

    g_main_loop_ptr = g_main_loop_new(nullptr, FALSE);
    g_main_loop_run(g_main_loop_ptr);

    g_running = false;
    if (cap_th.joinable()) cap_th.join();
    if (key_th.joinable()) key_th.join();

    g_main_loop_unref(g_main_loop_ptr);
    g_main_loop_ptr = nullptr;
    return 0;
}
