#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

#include <algorithm>
#include <cctype>
#include <csignal>
#include <string>
#include <thread>

#include "eis_capture.hpp"
#include "eis_common.hpp"
#include "eis_config.hpp"
#include "eis_globals.hpp"
#include "eis_imu.hpp"
#include "eis_input.hpp"
#include "rtsp_server.hpp"

static void sigint_handler(int) { g_running = false; }

static void print_usage(const char* prog) {
    fprintf(stderr, "Usage: %s [options]\n", prog);
    fprintf(stderr, "  --config <path>\n");
    fprintf(stderr, "  --mode {lk|gyro|hybrid}\n");
    fprintf(stderr, "  --imu-offset-ms <double>\n");
    fprintf(stderr, "  --offset-sweep\n");
    fprintf(stderr, "  --overlay {0|1}\n");
    fprintf(stderr, "  --log-every-frames <N>\n");
    fprintf(stderr, "  --ts-source {sensor|pts|arrival} (auto alias supported)\n");
    fprintf(stderr, "  --gyro-warp-mode {delta_direct|highpass}\n");
    fprintf(stderr, "  --gyro-gain-roll <double>\n");
    fprintf(stderr, "  --gyro-gain-pitch <double>\n");
    fprintf(stderr, "  --gyro-gain-yaw <double>\n");
    fprintf(stderr, "  --gyro-max-roll-deg <deg>\n");
    fprintf(stderr, "  --gyro-max-pitch-deg <deg>\n");
    fprintf(stderr, "  --gyro-max-yaw-deg <deg>\n");
    fprintf(stderr, "  --gyro-hp-lpf-alpha <0~1>\n");
    fprintf(stderr, "  --gyro-hp-gain-roll <double>\n");
    fprintf(stderr, "  --gyro-hp-gain-pitch <double>\n");
    fprintf(stderr, "  --gyro-hp-gain-yaw <double>\n");
    fprintf(stderr, "  --gyro-large-rot-thresh-deg <deg>\n");
    fprintf(stderr, "  --gyro-large-rot-gain-scale <0~1>\n");
    fprintf(stderr, "  --lk-trans-every-n <int>\n");
    fprintf(stderr, "  --lk-trans-alpha <0~1>\n");
    fprintf(stderr, "  --lk-trans-max-corr-px <px>\n");
    fprintf(stderr, "  --lk-trans-scale <0~1>\n");
    fprintf(stderr, "  --lk-trans-min-inliers <int>\n");
    fprintf(stderr, "  --lk-trans-jerk-limit-px <px>\n");
    fprintf(stderr, "  --lk-trans-confidence-gate <0~1>\n");
    fprintf(stderr, "  --profile {run|calib|debug}\n");
    fprintf(stderr, "  --libcamera-xrgb {0|1}\n");
}

static bool parse_ts_source_pref(const std::string& in, int& out_pref) {
    std::string s = in;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if (s == "sensor") out_pref = (int)TsSourcePref::SENSOR;
    else if (s == "pts") out_pref = (int)TsSourcePref::PTS;
    else if (s == "arrival") out_pref = (int)TsSourcePref::ARRIVAL;
    else if (s == "auto") out_pref = (int)TsSourcePref::AUTO; // compatibility alias
    else return false;
    return true;
}

static bool parse_gyro_warp_mode(const std::string& in, int& out_mode) {
    std::string s = in;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if (s == "delta_direct") out_mode = (int)GyroWarpMode::DELTA_DIRECT;
    else if (s == "highpass") out_mode = (int)GyroWarpMode::HIGHPASS;
    else return false;
    return true;
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
        int ts_pref = g_ts_pref.load();
        if (parse_ts_source_pref(*v, ts_pref)) g_ts_pref = ts_pref;
    }

    if (auto v = get("gyro_warp_mode")) {
        int warp_mode = g_gyro_warp_mode.load();
        if (parse_gyro_warp_mode(*v, warp_mode)) g_gyro_warp_mode = warp_mode;
    }

    if (auto v = get("profile")) {
        std::string s = *v;
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        if (s == "run") g_profile = (int)RunProfile::RUN;
        else if (s == "calib") g_profile = (int)RunProfile::CALIB;
        else if (s == "debug") g_profile = (int)RunProfile::DEBUG;
    }

    set_bool("libcamera_xrgb", g_libcamera_xrgb);

    set_double("gyro_gain_roll", g_gyro_gain_roll);
    set_double("gyro_gain_pitch", g_gyro_gain_pitch);
    set_double("gyro_gain_yaw", g_gyro_gain_yaw);

    if (auto v = get("gyro_max_roll_deg")) g_gyro_max_roll_rad = std::stod(*v) * CV_PI / 180.0;
    if (auto v = get("gyro_max_pitch_deg")) g_gyro_max_pitch_rad = std::stod(*v) * CV_PI / 180.0;
    if (auto v = get("gyro_max_yaw_deg")) g_gyro_max_yaw_rad = std::stod(*v) * CV_PI / 180.0;

    set_double("gyro_hp_lpf_alpha", g_gyro_hp_lpf_alpha);
    set_double("gyro_hp_gain_roll", g_gyro_hp_gain_roll);
    set_double("gyro_hp_gain_pitch", g_gyro_hp_gain_pitch);
    set_double("gyro_hp_gain_yaw", g_gyro_hp_gain_yaw);

    set_double("gyro_large_rot_thresh_deg", g_gyro_large_rot_thresh_deg);
    set_double("gyro_large_rot_gain_scale", g_gyro_large_rot_gain_scale);

    set_bool("lk.trans_enable", g_lk_trans_enable);
    set_int("lk.max_features", g_lk_max_features);
    if (auto v = get("lk.quality")) g_lk_quality = std::stod(*v);
    if (auto v = get("lk.min_dist")) g_lk_min_dist = std::stod(*v);
    set_int("lk.trans_min_features", g_lk_trans_min_features);
    set_int("lk.trans_min_inliers", g_lk_trans_min_inliers);
    set_int("lk.trans_every_n", g_lk_trans_every_n);
    if (auto v = get("lk.trans_alpha")) g_lk_trans_alpha = std::stod(*v);
    if (auto v = get("lk.trans_max_corr_px")) g_lk_trans_max_corr_px = std::stod(*v);
    if (auto v = get("lk.trans_scale")) g_lk_trans_scale = std::stod(*v);
    if (auto v = get("lk.trans_jerk_limit_px")) g_lk_trans_jerk_limit_px = std::stod(*v);
    if (auto v = get("lk.trans_confidence_gate")) g_lk_trans_confidence_gate = std::stod(*v);
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

        auto parse_mode = [&](const std::string& value) -> bool {
            std::string m = value;
            std::transform(m.begin(), m.end(), m.begin(), ::tolower);
            if (m == "lk") g_mode = (int)EisMode::LK;
            else if (m == "gyro") g_mode = (int)EisMode::GYRO;
            else if (m == "hybrid") g_mode = (int)EisMode::HYBRID;
            else return false;
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
        } else if (a == "--mode") {
            if (!eat_value(val) || !parse_mode(val)) return false;
            continue;
        } else if (a.rfind("--mode=", 0) == 0) {
            if (!parse_mode(a.substr(7))) return false;
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
            int pref = 0;
            if (!parse_ts_source_pref(val, pref)) return false;
            g_ts_pref = pref;
        } else if (a.rfind("--ts-source=", 0) == 0) {
            int pref = 0;
            if (!parse_ts_source_pref(a.substr(12), pref)) return false;
            g_ts_pref = pref;
        } else if (a == "--gyro-warp-mode") {
            if (!eat_value(val)) return false;
            int mode = 0;
            if (!parse_gyro_warp_mode(val, mode)) return false;
            g_gyro_warp_mode = mode;
        } else if (a.rfind("--gyro-warp-mode=", 0) == 0) {
            int mode = 0;
            if (!parse_gyro_warp_mode(a.substr(17), mode)) return false;
            g_gyro_warp_mode = mode;
        } else if (a == "--gyro-gain-roll") {
            if (!eat_value(val)) return false;
            g_gyro_gain_roll = std::stod(val);
        } else if (a.rfind("--gyro-gain-roll=", 0) == 0) {
            g_gyro_gain_roll = std::stod(a.substr(17));
        } else if (a == "--gyro-gain-pitch") {
            if (!eat_value(val)) return false;
            g_gyro_gain_pitch = std::stod(val);
        } else if (a.rfind("--gyro-gain-pitch=", 0) == 0) {
            g_gyro_gain_pitch = std::stod(a.substr(18));
        } else if (a == "--gyro-gain-yaw") {
            if (!eat_value(val)) return false;
            g_gyro_gain_yaw = std::stod(val);
        } else if (a.rfind("--gyro-gain-yaw=", 0) == 0) {
            g_gyro_gain_yaw = std::stod(a.substr(16));
        } else if (a == "--gyro-max-roll-deg") {
            if (!eat_value(val)) return false;
            g_gyro_max_roll_rad = std::stod(val) * CV_PI / 180.0;
        } else if (a.rfind("--gyro-max-roll-deg=", 0) == 0) {
            g_gyro_max_roll_rad = std::stod(a.substr(20)) * CV_PI / 180.0;
        } else if (a == "--gyro-max-pitch-deg") {
            if (!eat_value(val)) return false;
            g_gyro_max_pitch_rad = std::stod(val) * CV_PI / 180.0;
        } else if (a.rfind("--gyro-max-pitch-deg=", 0) == 0) {
            g_gyro_max_pitch_rad = std::stod(a.substr(21)) * CV_PI / 180.0;
        } else if (a == "--gyro-max-yaw-deg") {
            if (!eat_value(val)) return false;
            g_gyro_max_yaw_rad = std::stod(val) * CV_PI / 180.0;
        } else if (a.rfind("--gyro-max-yaw-deg=", 0) == 0) {
            g_gyro_max_yaw_rad = std::stod(a.substr(19)) * CV_PI / 180.0;
        } else if (a == "--gyro-hp-lpf-alpha") {
            if (!eat_value(val)) return false;
            g_gyro_hp_lpf_alpha = std::stod(val);
        } else if (a.rfind("--gyro-hp-lpf-alpha=", 0) == 0) {
            g_gyro_hp_lpf_alpha = std::stod(a.substr(20));
        } else if (a == "--gyro-hp-gain-roll") {
            if (!eat_value(val)) return false;
            g_gyro_hp_gain_roll = std::stod(val);
        } else if (a.rfind("--gyro-hp-gain-roll=", 0) == 0) {
            g_gyro_hp_gain_roll = std::stod(a.substr(20));
        } else if (a == "--gyro-hp-gain-pitch") {
            if (!eat_value(val)) return false;
            g_gyro_hp_gain_pitch = std::stod(val);
        } else if (a.rfind("--gyro-hp-gain-pitch=", 0) == 0) {
            g_gyro_hp_gain_pitch = std::stod(a.substr(21));
        } else if (a == "--gyro-hp-gain-yaw") {
            if (!eat_value(val)) return false;
            g_gyro_hp_gain_yaw = std::stod(val);
        } else if (a.rfind("--gyro-hp-gain-yaw=", 0) == 0) {
            g_gyro_hp_gain_yaw = std::stod(a.substr(19));
        } else if (a == "--gyro-large-rot-thresh-deg") {
            if (!eat_value(val)) return false;
            g_gyro_large_rot_thresh_deg = std::stod(val);
        } else if (a.rfind("--gyro-large-rot-thresh-deg=", 0) == 0) {
            g_gyro_large_rot_thresh_deg = std::stod(a.substr(28));
        } else if (a == "--gyro-large-rot-gain-scale") {
            if (!eat_value(val)) return false;
            g_gyro_large_rot_gain_scale = std::stod(val);
        } else if (a.rfind("--gyro-large-rot-gain-scale=", 0) == 0) {
            g_gyro_large_rot_gain_scale = std::stod(a.substr(28));
        } else if (a == "--lk-trans-every-n") {
            if (!eat_value(val)) return false;
            g_lk_trans_every_n = std::max(1, std::stoi(val));
        } else if (a.rfind("--lk-trans-every-n=", 0) == 0) {
            g_lk_trans_every_n = std::max(1, std::stoi(a.substr(19)));
        } else if (a == "--lk-trans-alpha") {
            if (!eat_value(val)) return false;
            g_lk_trans_alpha = std::stod(val);
        } else if (a.rfind("--lk-trans-alpha=", 0) == 0) {
            g_lk_trans_alpha = std::stod(a.substr(17));
        } else if (a == "--lk-trans-max-corr-px") {
            if (!eat_value(val)) return false;
            g_lk_trans_max_corr_px = std::stod(val);
        } else if (a.rfind("--lk-trans-max-corr-px=", 0) == 0) {
            g_lk_trans_max_corr_px = std::stod(a.substr(23));
        } else if (a == "--lk-trans-scale") {
            if (!eat_value(val)) return false;
            g_lk_trans_scale = std::stod(val);
        } else if (a.rfind("--lk-trans-scale=", 0) == 0) {
            g_lk_trans_scale = std::stod(a.substr(17));
        } else if (a == "--lk-trans-min-inliers") {
            if (!eat_value(val)) return false;
            g_lk_trans_min_inliers = std::max(0, std::stoi(val));
        } else if (a.rfind("--lk-trans-min-inliers=", 0) == 0) {
            g_lk_trans_min_inliers = std::max(0, std::stoi(a.substr(23)));
        } else if (a == "--lk-trans-jerk-limit-px") {
            if (!eat_value(val)) return false;
            g_lk_trans_jerk_limit_px = std::stod(val);
        } else if (a.rfind("--lk-trans-jerk-limit-px=", 0) == 0) {
            g_lk_trans_jerk_limit_px = std::stod(a.substr(25));
        } else if (a == "--lk-trans-confidence-gate") {
            if (!eat_value(val)) return false;
            g_lk_trans_confidence_gate = std::stod(val);
        } else if (a.rfind("--lk-trans-confidence-gate=", 0) == 0) {
            g_lk_trans_confidence_gate = std::stod(a.substr(27));
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
        } else if (a == "--gyro-warp" || a.rfind("--gyro-warp=", 0) == 0 ||
                   a == "--smooth-alpha" || a.rfind("--smooth-alpha=", 0) == 0 ||
                   a == "--max-roll-deg" || a.rfind("--max-roll-deg=", 0) == 0 ||
                   a == "--max-pitch-deg" || a.rfind("--max-pitch-deg=", 0) == 0 ||
                   a == "--max-yaw-deg" || a.rfind("--max-yaw-deg=", 0) == 0) {
            fprintf(stderr, "[ERR] Deprecated arg: %s\n", a.c_str());
            return false;
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
    g_signal_connect(f_stab, "media-configure", (GCallback)on_media_configure, (gpointer)"stab");
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
    fprintf(stderr, "  IMU offset: %.3f ms  (sweep: %s)\n",
            g_manual_imu_offset_ms,
            g_offset_sweep.load() ? "on" : "off");
    fprintf(stderr, "  Profile: %s\n", profile_str((RunProfile)g_profile.load()));
    fprintf(stderr, "  Libcamera XRGB: %s\n", g_libcamera_xrgb.load() ? "true" : "false");
    fprintf(stderr, "  Gyro warp mode: %s\n", gyro_warp_str((GyroWarpMode)g_gyro_warp_mode.load()));
    fprintf(stderr, "  Gyro gain: R=%.2f P=%.2f Y=%.2f\n",
            g_gyro_gain_roll.load(),
            g_gyro_gain_pitch.load(),
            g_gyro_gain_yaw.load());
    fprintf(stderr, "  Gyro clamp max(deg): R=%.2f P=%.2f Y=%.2f\n",
            g_gyro_max_roll_rad.load() * 180.0 / CV_PI,
            g_gyro_max_pitch_rad.load() * 180.0 / CV_PI,
            g_gyro_max_yaw_rad.load() * 180.0 / CV_PI);
    fprintf(stderr, "  Gyro HP: alpha=%.3f gain R=%.2f P=%.2f Y=%.2f\n",
            g_gyro_hp_lpf_alpha.load(),
            g_gyro_hp_gain_roll.load(),
            g_gyro_hp_gain_pitch.load(),
            g_gyro_hp_gain_yaw.load());
    fprintf(stderr, "  Large-rot protect: thresh=%.2fdeg scale=%.2f\n",
            g_gyro_large_rot_thresh_deg.load(),
            g_gyro_large_rot_gain_scale.load());
    fprintf(stderr,
            "  IMU map: roll axis=%d sign=%d | pitch axis=%d sign=%d | yaw axis=%d sign=%d\n",
            IMU_AXIS_ROLL,
            IMU_SIGN_ROLL,
            IMU_AXIS_PITCH,
            IMU_SIGN_PITCH,
            IMU_AXIS_YAW,
            IMU_SIGN_YAW);
    fprintf(stderr, "  TS source: %s\n", ts_pref_str((TsSourcePref)g_ts_pref.load()));
    fprintf(stderr, "  Crop: fixed %.0f%%\n", FIXED_CROP_PERCENT);
    fprintf(stderr,
            "  LK trans: %s  every=%d  alpha=%.2f  max_corr=%.1fpx  scale=%.2f  "
            "min_inliers=%d jerk=%.1fpx conf=%.2f\n",
            g_lk_trans_enable.load() ? "on" : "off",
            g_lk_trans_every_n.load(),
            g_lk_trans_alpha.load(),
            g_lk_trans_max_corr_px.load(),
            g_lk_trans_scale.load(),
            g_lk_trans_min_inliers.load(),
            g_lk_trans_jerk_limit_px.load(),
            g_lk_trans_confidence_gate.load());
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
