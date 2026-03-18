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
#include "eis_config.hpp"

static void sigint_handler(int) { g_running = false; }

static void print_usage(const char* prog) {
    fprintf(stderr, "Usage: %s [options]\n", prog);
    fprintf(stderr, "  --config <path>\n");
    fprintf(stderr, "  --mode {lk|gyro|hybrid}\n");
    fprintf(stderr, "  --imu-offset-ms <double>\n");
    fprintf(stderr, "  --offset-sweep\n");
    fprintf(stderr, "  --overlay {0|1}\n");
    fprintf(stderr, "  --log-every-frames <N>\n");
    fprintf(stderr, "  --ts-source {auto|sensor|pts|arrival}\n");
    fprintf(stderr, "  --gyro-warp {jitter|delta}\n");
    fprintf(stderr, "  --profile {run|calib|debug}\n");
    fprintf(stderr, "  --libcamera-xrgb {0|1}\n");
    fprintf(stderr, "  --smooth-alpha <0~1>\n");
    fprintf(stderr, "  --max-roll-deg <deg>\n");
    fprintf(stderr, "  --max-pitch-deg <deg>\n");
    fprintf(stderr, "  --max-yaw-deg <deg>\n");
}

static void apply_config_map(const ConfigMap& cfg) {
    auto get = [&](const char* k) -> const std::string* {
        auto it = cfg.find(k);
        if (it == cfg.end()) return nullptr;
        return &it->second;
    };

    auto set_bool = [&](const char* k, std::atomic<bool>& dst) {
        if (auto v = get(k)) {
            std::string s = *v;
            std::transform(s.begin(), s.end(), s.begin(), ::tolower);
            dst = (s == "1" || s == "true" || s == "yes" || s == "on");
        }
    };
    auto set_int = [&](const char* k, std::atomic<int>& dst) {
        if (auto v = get(k)) dst = std::stoi(*v);
    };
    auto set_double = [&](const char* k, std::atomic<double>& dst) {
        if (auto v = get(k)) dst = std::stod(*v);
    };

    if (auto v = get("mode")) {
        std::string m = *v;
        std::transform(m.begin(), m.end(), m.begin(), ::tolower);
        if (m == "lk") g_mode = (int)EisMode::LK;
        else if (m == "gyro") g_mode = (int)EisMode::GYRO;
        else if (m == "hybrid") g_mode = (int)EisMode::HYBRID;
    }
    if (auto v = get("imu_offset_ms")) g_manual_imu_offset_ms = std::stod(*v);
    set_bool("offset_sweep", g_offset_sweep);
    set_bool("overlay", g_debug_overlay);
    set_int("log_every_frames", g_log_every_frames);

    if (auto v = get("ts_source")) {
        std::string s = *v;
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        if (s == "auto") g_ts_pref = (int)TsSourcePref::AUTO;
        else if (s == "sensor") g_ts_pref = (int)TsSourcePref::SENSOR;
        else if (s == "pts") g_ts_pref = (int)TsSourcePref::PTS;
        else if (s == "arrival") g_ts_pref = (int)TsSourcePref::ARRIVAL;
    }
    if (auto v = get("gyro_warp")) {
        std::string s = *v;
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        if (s == "jitter") g_gyro_warp_mode = (int)GyroWarpMode::JITTER;
        else if (s == "delta") g_gyro_warp_mode = (int)GyroWarpMode::DELTA;
    }
    if (auto v = get("profile")) {
        std::string s = *v;
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        if (s == "run") g_profile = (int)RunProfile::RUN;
        else if (s == "calib") g_profile = (int)RunProfile::CALIB;
        else if (s == "debug") g_profile = (int)RunProfile::DEBUG;
    }
    set_bool("libcamera_xrgb", g_libcamera_xrgb);

    set_double("smooth_alpha", g_smooth_alpha);
    if (auto v = get("max_roll_deg")) g_max_roll_rad = std::stod(*v) * CV_PI / 180.0;
    if (auto v = get("max_pitch_deg")) g_max_pitch_rad = std::stod(*v) * CV_PI / 180.0;
    if (auto v = get("max_yaw_deg")) g_max_yaw_rad = std::stod(*v) * CV_PI / 180.0;

    set_bool("lk.trans_enable", g_lk_trans_enable);
    set_int("lk.max_features", g_lk_max_features);
    if (auto v = get("lk.quality")) g_lk_quality = std::stod(*v);
    if (auto v = get("lk.min_dist")) g_lk_min_dist = std::stod(*v);
    set_int("lk.trans_min_features", g_lk_trans_min_features);
    set_int("lk.trans_every_n", g_lk_trans_every_n);
    if (auto v = get("lk.trans_alpha")) g_lk_trans_alpha = std::stod(*v);
    if (auto v = get("lk.trans_max_corr_px")) g_lk_trans_max_corr_px = std::stod(*v);
    if (auto v = get("lk.trans_scale")) g_lk_trans_scale = std::stod(*v);
}

static bool parse_args(int argc, char* argv[]) {
    bool profile_set = false;
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
            // handled in main; ignore here
            val.clear();
            continue;
        } else if (a.rfind("--config=", 0) == 0) {
            // handled in main; ignore here
            continue;
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
        } else if (a == "--profile") {
            if (!eat_value(val)) return false;
            std::string s = val;
            std::transform(s.begin(), s.end(), s.begin(), ::tolower);
            if (s == "run") g_profile = (int)RunProfile::RUN;
            else if (s == "calib") g_profile = (int)RunProfile::CALIB;
            else if (s == "debug") g_profile = (int)RunProfile::DEBUG;
            else return false;
            profile_set = true;
        } else if (a.rfind("--profile=", 0) == 0) {
            std::string s = a.substr(10);
            std::transform(s.begin(), s.end(), s.begin(), ::tolower);
            if (s == "run") g_profile = (int)RunProfile::RUN;
            else if (s == "calib") g_profile = (int)RunProfile::CALIB;
            else if (s == "debug") g_profile = (int)RunProfile::DEBUG;
            else return false;
            profile_set = true;
        } else if (a == "--libcamera-xrgb") {
            if (!eat_value(val)) return false;
            g_libcamera_xrgb = (std::stoi(val) != 0);
        } else if (a.rfind("--libcamera-xrgb=", 0) == 0) {
            g_libcamera_xrgb = (std::stoi(a.substr(18)) != 0);
        } else if (a == "--smooth-alpha") {
            if (!eat_value(val)) return false;
            g_smooth_alpha = std::stod(val);
        } else if (a.rfind("--smooth-alpha=", 0) == 0) {
            g_smooth_alpha = std::stod(a.substr(15));
        } else if (a == "--max-roll-deg") {
            if (!eat_value(val)) return false;
            g_max_roll_rad = std::stod(val) * CV_PI / 180.0;
        } else if (a.rfind("--max-roll-deg=", 0) == 0) {
            g_max_roll_rad = std::stod(a.substr(16)) * CV_PI / 180.0;
        } else if (a == "--max-pitch-deg") {
            if (!eat_value(val)) return false;
            g_max_pitch_rad = std::stod(val) * CV_PI / 180.0;
        } else if (a.rfind("--max-pitch-deg=", 0) == 0) {
            g_max_pitch_rad = std::stod(a.substr(17)) * CV_PI / 180.0;
        } else if (a == "--max-yaw-deg") {
            if (!eat_value(val)) return false;
            g_max_yaw_rad = std::stod(val) * CV_PI / 180.0;
        } else if (a.rfind("--max-yaw-deg=", 0) == 0) {
            g_max_yaw_rad = std::stod(a.substr(15)) * CV_PI / 180.0;
        } else {
            fprintf(stderr, "[ERR] Unknown arg: %s\n", a.c_str());
            return false;
        }
    }

    if (!profile_set && g_offset_sweep.load()) {
        g_profile = (int)RunProfile::CALIB;
    }
    return true;
}

int main(int argc, char* argv[]) {
    system("fuser -k 8555/tcp 2>/dev/null");
    signal(SIGINT, sigint_handler);

    // Load local config (if present) before parsing CLI
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
    RunProfile profile = (RunProfile)g_profile.load();
    if (g_log_every_frames.load() < 0) {
        g_log_every_frames = g_debug_overlay.load() ? (G_FPS * 2) : 0;
    }
    if (profile == RunProfile::RUN || profile == RunProfile::CALIB) {
        g_debug_overlay = false;
        g_log_every_frames = 0;
        if (profile == RunProfile::RUN) {
            g_offset_sweep = false;
        }
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
    fprintf(stderr, "  Mode: %s (HYBRID = gyro rotation + LK translation)\n",
            mode_str((EisMode)g_mode.load()));
    fprintf(stderr, "  IMU offset: %.3f ms  (sweep: %s)\n", g_manual_imu_offset_ms,
            g_offset_sweep.load() ? "on" : "off");
    fprintf(stderr, "  Profile: %s\n", profile_str((RunProfile)g_profile.load()));
    fprintf(stderr, "  Libcamera XRGB: %s\n", g_libcamera_xrgb.load() ? "true" : "false");
    fprintf(stderr, "  Gyro alpha=%.3f max(deg) R=%.1f P=%.1f Y=%.1f\n",
            g_smooth_alpha.load(),
            g_max_roll_rad.load() * 180.0 / CV_PI,
            g_max_pitch_rad.load() * 180.0 / CV_PI,
            g_max_yaw_rad.load() * 180.0 / CV_PI);
    fprintf(stderr, "  IMU map: roll axis=%d sign=%d | pitch axis=%d sign=%d | yaw axis=%d sign=%d\n",
            IMU_AXIS_ROLL, IMU_SIGN_ROLL, IMU_AXIS_PITCH, IMU_SIGN_PITCH, IMU_AXIS_YAW, IMU_SIGN_YAW);
    fprintf(stderr, "  TS source: %s\n", ts_pref_str((TsSourcePref)g_ts_pref.load()));
    fprintf(stderr, "  GyroEIS smooth alpha=%.3f  warp=%s\n",
            SMOOTH_ALPHA, gyro_warp_str((GyroWarpMode)g_gyro_warp_mode.load()));
    fprintf(stderr, "  Crop: fixed %.0f%%\n", FIXED_CROP_PERCENT);
    fprintf(stderr, "  LK trans: %s  every=%d  alpha=%.2f  max_corr=%.1fpx  scale=%.2f\n",
            g_lk_trans_enable.load() ? "on" : "off",
            g_lk_trans_every_n.load(),
            g_lk_trans_alpha.load(),
            g_lk_trans_max_corr_px.load(),
            g_lk_trans_scale.load());
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
