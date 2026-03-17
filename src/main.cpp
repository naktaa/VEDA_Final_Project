#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

#include <algorithm>
#include <csignal>
#include <string>
#include <thread>

#include "eis_common.hpp"
#include "eis_globals.hpp"
#include "eis_input.hpp"
#include "eis_imu.hpp"
#include "eis_capture.hpp"
#include "rtsp_server.hpp"

static void sigint_handler(int) { g_running = false; }

static void print_usage(const char* prog) {
    fprintf(stderr, "Usage: %s [options]\n", prog);
    fprintf(stderr, "  --mode {lk|gyro|hybrid}\n");
    fprintf(stderr, "  --imu-offset-ms <double>\n");
    fprintf(stderr, "  --offset-sweep\n");
    fprintf(stderr, "  --overlay {0|1}\n");
    fprintf(stderr, "  --log-every-frames <N>\n");
    fprintf(stderr, "  --ts-source {auto|sensor|pts|arrival}\n");
    fprintf(stderr, "  --gyro-warp {jitter|delta}\n");
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
        } else if (a == "--gyro-warp") {
            if (!eat_value(val)) return false;
            std::string s = val;
            std::transform(s.begin(), s.end(), s.begin(), ::tolower);
            if (s == "jitter") g_gyro_warp_mode = (int)GyroWarpMode::JITTER;
            else if (s == "delta") g_gyro_warp_mode = (int)GyroWarpMode::DELTA;
            else return false;
        } else if (a.rfind("--gyro-warp=", 0) == 0) {
            std::string s = a.substr(12);
            std::transform(s.begin(), s.end(), s.begin(), ::tolower);
            if (s == "jitter") g_gyro_warp_mode = (int)GyroWarpMode::JITTER;
            else if (s == "delta") g_gyro_warp_mode = (int)GyroWarpMode::DELTA;
            else return false;
        } else {
            fprintf(stderr, "[ERR] Unknown arg: %s\n", a.c_str());
            return false;
        }
    }
    return true;
}

int main(int argc, char* argv[]) {
    system("fuser -k 8555/tcp 2>/dev/null");
    signal(SIGINT, sigint_handler);
    if (!parse_args(argc, argv)) return 0;
    if (g_log_every_frames.load() < 0) {
        g_log_every_frames = g_debug_overlay.load() ? (G_FPS * 2) : 0;
    }

    gst_init(&argc, &argv);

    GstRTSPServer* server = gst_rtsp_server_new();
    gst_rtsp_server_set_service(server, "8555");
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);

    GstRTSPMediaFactory* f_stab = make_factory("stabsrc");
    g_signal_connect(f_stab, "media-configure", (GCallback)on_media_configure, (gpointer) "stab");
    gst_rtsp_mount_points_add_factory(mounts, "/cam", f_stab);

    g_object_unref(mounts);

    if (gst_rtsp_server_attach(server, nullptr) == 0) {
        fprintf(stderr, "[ERR] RTSP attach\n");
        return -1;
    }

    fprintf(stderr, "==============================\n");
    fprintf(stderr, " EIS - Gyro-first (Quaternion)\n");
    fprintf(stderr, "==============================\n");
    fprintf(stderr, "  Mode: %s (LK/HYBRID are mapped to gyro-only in this stage)\n",
            mode_str((EisMode)g_mode.load()));
    fprintf(stderr, "  IMU offset: %.3f ms  (sweep: %s)\n", g_manual_imu_offset_ms,
            g_offset_sweep.load() ? "on" : "off");
    fprintf(stderr, "  TS source: %s\n", ts_pref_str((TsSourcePref)g_ts_pref.load()));
    fprintf(stderr, "  GyroEIS smooth alpha=%.3f  warp=%s\n",
            SMOOTH_ALPHA, gyro_warp_str((GyroWarpMode)g_gyro_warp_mode.load()));
    fprintf(stderr, "  Crop: fixed %.0f%%\n", FIXED_CROP_PERCENT);
    fprintf(stderr, "  RTSP: rtsp://<PI_IP>:8555/cam\n");
    fprintf(stderr, "  Keys: 1=LK 2=Gyro 3=Hybrid (output fixed to CAM)\n");
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
