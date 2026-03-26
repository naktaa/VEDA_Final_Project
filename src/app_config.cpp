#include "app_config.hpp"

#include <algorithm>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>

namespace {

using ConfigMap = std::unordered_map<std::string, std::string>;
constexpr double kPi = 3.14159265358979323846;

std::string trim(const std::string& input) {
    const auto begin = input.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return {};
    const auto end = input.find_last_not_of(" \t\r\n");
    return input.substr(begin, end - begin + 1);
}

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool parse_bool(const std::string& value, bool fallback) {
    const std::string normalized = lower(trim(value));
    if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on") {
        return true;
    }
    if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off") {
        return false;
    }
    return fallback;
}

void load_config_map(const std::string& path, ConfigMap& output) {
    std::ifstream input(path);
    std::string line;
    std::string section;

    while (std::getline(input, line)) {
        const std::string cleaned = trim(line);
        if (cleaned.empty() || cleaned[0] == '#' || cleaned[0] == ';') continue;
        if (cleaned.front() == '[' && cleaned.back() == ']') {
            section = lower(trim(cleaned.substr(1, cleaned.size() - 2)));
            continue;
        }

        const auto pos = cleaned.find('=');
        if (pos == std::string::npos) continue;

        const std::string key = lower(trim(cleaned.substr(0, pos)));
        const std::string value = trim(cleaned.substr(pos + 1));
        const std::string full_key = section.empty() ? key : section + "." + key;
        output[full_key] = value;
    }
}

template <typename T>
void load_int(const ConfigMap& map, const std::string& key, T& output) {
    auto it = map.find(key);
    if (it == map.end()) return;
    output = static_cast<T>(std::stoi(it->second));
}

void load_double(const ConfigMap& map, const std::string& key, double& output) {
    auto it = map.find(key);
    if (it == map.end()) return;
    output = std::stod(it->second);
}

void load_string(const ConfigMap& map, const std::string& key, std::string& output) {
    auto it = map.find(key);
    if (it == map.end()) return;
    output = it->second;
}

void load_bool(const ConfigMap& map, const std::string& key, bool& output) {
    auto it = map.find(key);
    if (it == map.end()) return;
    output = parse_bool(it->second, output);
}

int frame_duration_from_fps(int fps) {
    return fps > 0 ? static_cast<int>((1000000 + (fps / 2)) / fps) : 0;
}

std::string now_string() {
    const std::time_t now = std::time(nullptr);
    std::tm tm {};
    localtime_r(&now, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

double deg_to_rad(double deg) {
    return deg * kPi / 180.0;
}

void append_section(std::ostringstream& oss, const std::string& section) {
    oss << "[" << section << "]\n";
}

void append_kv(std::ostringstream& oss, const std::string& key, const std::string& value) {
    oss << key << "=" << value << "\n";
}

template <typename T>
void append_kv(std::ostringstream& oss, const std::string& key, T value) {
    oss << key << "=" << value << "\n";
}

} // namespace

std::string make_default_config_text() {
    AppConfig cfg;
    std::ostringstream oss;
    oss << "# 하이브리드 EIS 로컬 설정 파일입니다.\n";
    oss << "# 이 파일은 git에 올리지 않고 로컬에서만 수정해서 사용합니다.\n\n";

    oss << "# [camera] 카메라 해상도, 프레임 타이밍, 노출, FOV\n";
    append_section(oss, "camera");
    append_kv(oss, "width", cfg.camera.width);
    append_kv(oss, "height", cfg.camera.height);
    append_kv(oss, "fps", cfg.camera.fps);
    append_kv(oss, "frame_duration_us", cfg.camera.frame_duration_us);
    append_kv(oss, "exposure_us", cfg.camera.exposure_us);
    append_kv(oss, "flip", cfg.camera.flip ? 1 : 0);
    append_kv(oss, "hfov_deg", cfg.camera.hfov_deg);
    append_kv(oss, "vfov_deg", cfg.camera.vfov_deg);
    append_kv(oss, "libcamera_xrgb", cfg.camera.libcamera_xrgb ? 1 : 0);
    append_kv(oss, "capture_backend", cfg.camera.capture_backend);
    oss << "\n";

    oss << "# [imu] IMU 버스, 샘플링 속도, FIFO/인터럽트, 축 매핑\n";
    append_section(oss, "imu");
    append_kv(oss, "bus", cfg.imu.bus);
    append_kv(oss, "addr", cfg.imu.addr);
    append_kv(oss, "int_gpio_chip", cfg.imu.int_gpio_chip);
    append_kv(oss, "int_line_offset", cfg.imu.int_line_offset);
    append_kv(oss, "int_pin_wpi", cfg.imu.int_pin_wpi);
    append_kv(oss, "target_hz", cfg.imu.target_hz);
    append_kv(oss, "use_fifo", cfg.imu.use_fifo ? 1 : 0);
    append_kv(oss, "gyro_sensitivity", cfg.imu.gyro_sensitivity);
    append_kv(oss, "axis_roll", cfg.imu.axis_roll);
    append_kv(oss, "axis_pitch", cfg.imu.axis_pitch);
    append_kv(oss, "axis_yaw", cfg.imu.axis_yaw);
    append_kv(oss, "sign_roll", cfg.imu.sign_roll);
    append_kv(oss, "sign_pitch", cfg.imu.sign_pitch);
    append_kv(oss, "sign_yaw", cfg.imu.sign_yaw);
    oss << "\n";

    oss << "# [eis] LK, gyro, crop, turn-follow 관련 튜닝\n";
    append_section(oss, "eis");
    append_kv(oss, "lk_max_features", cfg.eis.lk_max_features);
    append_kv(oss, "lk_min_features", cfg.eis.lk_min_features);
    append_kv(oss, "lk_min_inliers", cfg.eis.lk_min_inliers);
    append_kv(oss, "lk_quality", cfg.eis.lk_quality);
    append_kv(oss, "lk_min_dist", cfg.eis.lk_min_dist);
    append_kv(oss, "lk_ransac_thresh", cfg.eis.lk_ransac_thresh);
    append_kv(oss, "lk_confidence_gate", cfg.eis.lk_confidence_gate);
    append_kv(oss, "lk_translation_alpha", cfg.eis.lk_translation_alpha);
    append_kv(oss, "lk_translation_max_corr_px", cfg.eis.lk_translation_max_corr_px);
    append_kv(oss, "lk_translation_turn_scale", cfg.eis.lk_translation_turn_scale);
    append_kv(oss, "lk_rotation_anchor_alpha", cfg.eis.lk_rotation_anchor_alpha);
    append_kv(oss, "lk_rotation_gain", cfg.eis.lk_rotation_gain);
    append_kv(oss, "gyro_gain_roll", cfg.eis.gyro_gain_roll);
    append_kv(oss, "gyro_gain_pitch", cfg.eis.gyro_gain_pitch);
    append_kv(oss, "gyro_gain_yaw", cfg.eis.gyro_gain_yaw);
    append_kv(oss, "gyro_max_roll_deg", cfg.eis.gyro_max_roll_deg);
    append_kv(oss, "gyro_max_pitch_deg", cfg.eis.gyro_max_pitch_deg);
    append_kv(oss, "gyro_max_yaw_deg", cfg.eis.gyro_max_yaw_deg);
    append_kv(oss, "gyro_hp_lpf_alpha", cfg.eis.gyro_hp_lpf_alpha);
    append_kv(oss, "gyro_hp_gain_roll", cfg.eis.gyro_hp_gain_roll);
    append_kv(oss, "gyro_hp_gain_pitch", cfg.eis.gyro_hp_gain_pitch);
    append_kv(oss, "gyro_hp_gain_yaw", cfg.eis.gyro_hp_gain_yaw);
    append_kv(oss, "gyro_large_rot_thresh_deg", cfg.eis.gyro_large_rot_thresh_deg);
    append_kv(oss, "gyro_large_rot_gain_scale", cfg.eis.gyro_large_rot_gain_scale);
    append_kv(oss, "crop_budget_percent", cfg.eis.crop_budget_percent);
    append_kv(oss, "turn_enter_yaw_rate_dps", cfg.eis.turn_enter_yaw_rate_dps);
    append_kv(oss, "turn_exit_yaw_rate_dps", cfg.eis.turn_exit_yaw_rate_dps);
    append_kv(oss, "turn_hold_frames", cfg.eis.turn_hold_frames);
    append_kv(oss, "recover_frames", cfg.eis.recover_frames);
    append_kv(oss, "turn_follow_correction_scale", cfg.eis.turn_follow_correction_scale);
    append_kv(oss, "debug_overlay", cfg.eis.debug_overlay ? 1 : 0);
    oss << "\n";

    oss << "# [rtsp] 스트림 경로와 인코더 튜닝\n";
    append_section(oss, "rtsp");
    append_kv(oss, "port", cfg.rtsp.port);
    append_kv(oss, "path", cfg.rtsp.path);
    append_kv(oss, "raw_path", cfg.rtsp.raw_path);
    append_kv(oss, "bitrate", cfg.rtsp.bitrate);
    append_kv(oss, "iframe_period", cfg.rtsp.iframe_period);
    oss << "\n";

    oss << "# [mqtt] 수동/자동 제어와 상태 송신에 쓰는 MQTT broker 및 topic\n";
    append_section(oss, "mqtt");
    append_kv(oss, "host", cfg.mqtt.host);
    append_kv(oss, "port", cfg.mqtt.port);
    append_kv(oss, "keepalive_sec", cfg.mqtt.keepalive_sec);
    append_kv(oss, "control_topic", cfg.mqtt.control_topic);
    append_kv(oss, "goal_topic", cfg.mqtt.goal_topic);
    append_kv(oss, "pose_topic", cfg.mqtt.pose_topic);
    append_kv(oss, "safety_topic", cfg.mqtt.safety_topic);
    append_kv(oss, "status_topic", cfg.mqtt.status_topic);
    append_kv(oss, "status_publish_interval_ms", cfg.mqtt.status_publish_interval_ms);
    oss << "\n";

    oss << "# [auto] goal tracking 제어 파라미터\n";
    append_section(oss, "auto");
    append_kv(oss, "k_linear", cfg.auto_control.k_linear);
    append_kv(oss, "k_yaw", cfg.auto_control.k_yaw);
    append_kv(oss, "max_speed_cmps", cfg.auto_control.max_speed_cmps);
    append_kv(oss, "max_yaw_rate_rps", cfg.auto_control.max_yaw_rate_rps);
    append_kv(oss, "tolerance_cm", cfg.auto_control.tolerance_cm);
    append_kv(oss, "rotate_yaw_offset_deg", cfg.auto_control.rotate_yaw_offset_rad * 180.0 / kPi);
    oss << "\n";

    oss << "# [motor] auto 제어용 wheel 모델 및 PWM 변환 값\n";
    append_section(oss, "motor");
    append_kv(oss, "track_width_cm", cfg.motor.track_width_cm);
    append_kv(oss, "wheel_max_speed_cmps", cfg.motor.wheel_max_speed_cmps);
    append_kv(oss, "speed_deadband_cmps", cfg.motor.speed_deadband_cmps);
    append_kv(oss, "pwm_min_effective", cfg.motor.pwm_min_effective);
    append_kv(oss, "pwm_max", cfg.motor.pwm_max);
    oss << "\n";

    oss << "# [manual] Qt/controller 수동 주행 기본 PWM\n";
    append_section(oss, "manual");
    append_kv(oss, "default_pwm", cfg.manual_drive.default_pwm);
    oss << "\n";

    oss << "# [controller] evdev 컨트롤러 입력 설정\n";
    append_section(oss, "controller");
    append_kv(oss, "enabled", cfg.controller.enabled ? 1 : 0);
    append_kv(oss, "input_device", cfg.controller.input_device);
    append_kv(oss, "device_name_hint", cfg.controller.device_name_hint);
    append_kv(oss, "idle_stop_ms", cfg.controller.idle_stop_ms);
    append_kv(oss, "speed_step", cfg.controller.speed_step);
    append_kv(oss, "log_only", cfg.controller.log_only ? 1 : 0);
    oss << "\n";

    oss << "# [http] 폰에서 보는 tiltVR 웹/MJPEG 서버 설정\n";
    append_section(oss, "http");
    append_kv(oss, "enable", cfg.http.enable ? 1 : 0);
    append_kv(oss, "port", cfg.http.port);
    append_kv(oss, "web_root", cfg.http.web_root);
    oss << "\n";

    oss << "# [ptz] 팬틸트 SG90 + PCA9685 설정\n";
    append_section(oss, "ptz");
    append_kv(oss, "i2c_device", cfg.ptz.i2c_device);
    append_kv(oss, "i2c_address", cfg.ptz.i2c_address);
    append_kv(oss, "pwm_frequency_hz", cfg.ptz.pwm_frequency_hz);
    append_kv(oss, "pan_channel", cfg.ptz.pan_channel);
    append_kv(oss, "tilt_channel", cfg.ptz.tilt_channel);
    append_kv(oss, "pan_center_deg", cfg.ptz.pan_center_deg);
    append_kv(oss, "pan_left_deg", cfg.ptz.pan_left_deg);
    append_kv(oss, "pan_right_deg", cfg.ptz.pan_right_deg);
    append_kv(oss, "tilt_center_deg", cfg.ptz.tilt_center_deg);
    append_kv(oss, "tilt_up_deg", cfg.ptz.tilt_up_deg);
    append_kv(oss, "tilt_down_deg", cfg.ptz.tilt_down_deg);
    append_kv(oss, "imu_timeout_ms", cfg.ptz.imu_timeout_ms);
    oss << "\n";

    oss << "# [calib] calib_offset가 저장하는 bias/offset 결과\n";
    append_section(oss, "calib");
    append_kv(oss, "bias_x", cfg.calib.bias_x);
    append_kv(oss, "bias_y", cfg.calib.bias_y);
    append_kv(oss, "bias_z", cfg.calib.bias_z);
    append_kv(oss, "imu_offset_ms", cfg.calib.imu_offset_ms);
    append_kv(oss, "ts_source", cfg.calib.ts_source);
    append_kv(oss, "last_calibration", cfg.calib.last_calibration.empty() ? "never" : cfg.calib.last_calibration);
    append_kv(oss, "bias_duration_ms", cfg.calib.bias_duration_ms);
    append_kv(oss, "sweep_duration_ms", cfg.calib.sweep_duration_ms);
    append_kv(oss, "coarse_range_ms", cfg.calib.coarse_range_ms);
    append_kv(oss, "coarse_step_ms", cfg.calib.coarse_step_ms);
    append_kv(oss, "fine_range_ms", cfg.calib.fine_range_ms);
    append_kv(oss, "fine_step_ms", cfg.calib.fine_step_ms);

    return oss.str();
}

bool ensure_local_config_exists(const std::string& template_path,
                                const std::string& local_path,
                                bool* created,
                                std::string* error) {
    if (created) *created = false;
    try {
        if (std::filesystem::exists(local_path)) {
            return true;
        }

        if (std::filesystem::exists(template_path)) {
            std::filesystem::copy_file(template_path, local_path, std::filesystem::copy_options::overwrite_existing);
        } else {
            std::ofstream output(local_path);
            output << make_default_config_text();
        }
        if (created) *created = true;
        return true;
    } catch (const std::exception& ex) {
        if (error) *error = ex.what();
        return false;
    }
}

bool load_app_config(const std::string& path, AppConfig& config, std::string* error) {
    try {
        if (!std::filesystem::exists(path)) {
            if (error) *error = "config file not found: " + path;
            return false;
        }

        ConfigMap map;
        load_config_map(path, map);

        load_int(map, "camera.width", config.camera.width);
        load_int(map, "camera.height", config.camera.height);
        load_int(map, "camera.fps", config.camera.fps);
        load_int(map, "camera.frame_duration_us", config.camera.frame_duration_us);
        load_int(map, "camera.exposure_us", config.camera.exposure_us);
        load_bool(map, "camera.flip", config.camera.flip);
        load_double(map, "camera.hfov_deg", config.camera.hfov_deg);
        load_double(map, "camera.vfov_deg", config.camera.vfov_deg);
        load_bool(map, "camera.libcamera_xrgb", config.camera.libcamera_xrgb);
        load_string(map, "camera.capture_backend", config.camera.capture_backend);
        const int normalized_frame_duration = frame_duration_from_fps(config.camera.fps);
        if (normalized_frame_duration > 0) {
            config.camera.frame_duration_us = normalized_frame_duration;
        }

        load_int(map, "imu.bus", config.imu.bus);
        load_int(map, "imu.addr", config.imu.addr);
        load_string(map, "imu.int_gpio_chip", config.imu.int_gpio_chip);
        load_int(map, "imu.int_line_offset", config.imu.int_line_offset);
        load_int(map, "imu.int_pin_wpi", config.imu.int_pin_wpi);
        load_int(map, "imu.target_hz", config.imu.target_hz);
        load_bool(map, "imu.use_fifo", config.imu.use_fifo);
        load_double(map, "imu.gyro_sensitivity", config.imu.gyro_sensitivity);
        load_int(map, "imu.axis_roll", config.imu.axis_roll);
        load_int(map, "imu.axis_pitch", config.imu.axis_pitch);
        load_int(map, "imu.axis_yaw", config.imu.axis_yaw);
        load_int(map, "imu.sign_roll", config.imu.sign_roll);
        load_int(map, "imu.sign_pitch", config.imu.sign_pitch);
        load_int(map, "imu.sign_yaw", config.imu.sign_yaw);

        load_int(map, "eis.lk_max_features", config.eis.lk_max_features);
        load_int(map, "eis.lk_min_features", config.eis.lk_min_features);
        load_int(map, "eis.lk_min_inliers", config.eis.lk_min_inliers);
        load_double(map, "eis.lk_quality", config.eis.lk_quality);
        load_double(map, "eis.lk_min_dist", config.eis.lk_min_dist);
        load_double(map, "eis.lk_ransac_thresh", config.eis.lk_ransac_thresh);
        load_double(map, "eis.lk_confidence_gate", config.eis.lk_confidence_gate);
        load_double(map, "eis.lk_translation_alpha", config.eis.lk_translation_alpha);
        load_double(map, "eis.lk_translation_max_corr_px", config.eis.lk_translation_max_corr_px);
        load_double(map, "eis.lk_translation_turn_scale", config.eis.lk_translation_turn_scale);
        load_double(map, "eis.lk_rotation_anchor_alpha", config.eis.lk_rotation_anchor_alpha);
        load_double(map, "eis.lk_rotation_gain", config.eis.lk_rotation_gain);
        load_double(map, "eis.gyro_gain_roll", config.eis.gyro_gain_roll);
        load_double(map, "eis.gyro_gain_pitch", config.eis.gyro_gain_pitch);
        load_double(map, "eis.gyro_gain_yaw", config.eis.gyro_gain_yaw);
        load_double(map, "eis.gyro_max_roll_deg", config.eis.gyro_max_roll_deg);
        load_double(map, "eis.gyro_max_pitch_deg", config.eis.gyro_max_pitch_deg);
        load_double(map, "eis.gyro_max_yaw_deg", config.eis.gyro_max_yaw_deg);
        load_double(map, "eis.gyro_hp_lpf_alpha", config.eis.gyro_hp_lpf_alpha);
        load_double(map, "eis.gyro_hp_gain_roll", config.eis.gyro_hp_gain_roll);
        load_double(map, "eis.gyro_hp_gain_pitch", config.eis.gyro_hp_gain_pitch);
        load_double(map, "eis.gyro_hp_gain_yaw", config.eis.gyro_hp_gain_yaw);
        load_double(map, "eis.gyro_large_rot_thresh_deg", config.eis.gyro_large_rot_thresh_deg);
        load_double(map, "eis.gyro_large_rot_gain_scale", config.eis.gyro_large_rot_gain_scale);
        load_double(map, "eis.crop_budget_percent", config.eis.crop_budget_percent);
        load_double(map, "eis.turn_enter_yaw_rate_dps", config.eis.turn_enter_yaw_rate_dps);
        load_double(map, "eis.turn_exit_yaw_rate_dps", config.eis.turn_exit_yaw_rate_dps);
        load_int(map, "eis.turn_hold_frames", config.eis.turn_hold_frames);
        load_int(map, "eis.recover_frames", config.eis.recover_frames);
        load_double(map, "eis.turn_follow_correction_scale", config.eis.turn_follow_correction_scale);
        load_bool(map, "eis.debug_overlay", config.eis.debug_overlay);

        load_string(map, "rtsp.port", config.rtsp.port);
        load_string(map, "rtsp.path", config.rtsp.path);
        load_string(map, "rtsp.raw_path", config.rtsp.raw_path);
        load_int(map, "rtsp.bitrate", config.rtsp.bitrate);
        load_int(map, "rtsp.iframe_period", config.rtsp.iframe_period);

        load_string(map, "mqtt.host", config.mqtt.host);
        load_int(map, "mqtt.port", config.mqtt.port);
        load_int(map, "mqtt.keepalive_sec", config.mqtt.keepalive_sec);
        load_string(map, "mqtt.control_topic", config.mqtt.control_topic);
        load_string(map, "mqtt.goal_topic", config.mqtt.goal_topic);
        load_string(map, "mqtt.pose_topic", config.mqtt.pose_topic);
        load_string(map, "mqtt.safety_topic", config.mqtt.safety_topic);
        load_string(map, "mqtt.status_topic", config.mqtt.status_topic);
        load_int(map, "mqtt.status_publish_interval_ms", config.mqtt.status_publish_interval_ms);
        auto legacy_topic_it = map.find("mqtt.topic");
        if (legacy_topic_it != map.end()) {
            config.mqtt.control_topic = legacy_topic_it->second;
        }

        load_double(map, "auto.k_linear", config.auto_control.k_linear);
        load_double(map, "auto.k_yaw", config.auto_control.k_yaw);
        load_double(map, "auto.max_speed_cmps", config.auto_control.max_speed_cmps);
        load_double(map, "auto.max_yaw_rate_rps", config.auto_control.max_yaw_rate_rps);
        load_double(map, "auto.tolerance_cm", config.auto_control.tolerance_cm);
        load_double(map, "auto.rotate_yaw_offset_rad", config.auto_control.rotate_yaw_offset_rad);
        auto rotate_deg_it = map.find("auto.rotate_yaw_offset_deg");
        if (rotate_deg_it != map.end()) {
            config.auto_control.rotate_yaw_offset_rad = deg_to_rad(std::stod(rotate_deg_it->second));
        }

        load_double(map, "motor.track_width_cm", config.motor.track_width_cm);
        load_double(map, "motor.wheel_max_speed_cmps", config.motor.wheel_max_speed_cmps);
        load_double(map, "motor.speed_deadband_cmps", config.motor.speed_deadband_cmps);
        load_int(map, "motor.pwm_min_effective", config.motor.pwm_min_effective);
        load_int(map, "motor.pwm_max", config.motor.pwm_max);

        load_int(map, "manual.default_pwm", config.manual_drive.default_pwm);

        load_bool(map, "controller.enabled", config.controller.enabled);
        load_string(map, "controller.input_device", config.controller.input_device);
        load_string(map, "controller.device_name_hint", config.controller.device_name_hint);
        load_int(map, "controller.idle_stop_ms", config.controller.idle_stop_ms);
        load_int(map, "controller.speed_step", config.controller.speed_step);
        load_bool(map, "controller.log_only", config.controller.log_only);

        load_bool(map, "http.enable", config.http.enable);
        load_int(map, "http.port", config.http.port);
        load_string(map, "http.web_root", config.http.web_root);

        load_string(map, "ptz.i2c_device", config.ptz.i2c_device);
        load_int(map, "ptz.i2c_address", config.ptz.i2c_address);
        load_int(map, "ptz.pwm_frequency_hz", config.ptz.pwm_frequency_hz);
        load_int(map, "ptz.pan_channel", config.ptz.pan_channel);
        load_int(map, "ptz.tilt_channel", config.ptz.tilt_channel);
        load_int(map, "ptz.imu_timeout_ms", config.ptz.imu_timeout_ms);
        auto load_float = [&](const std::string& key, float& output) {
            auto it = map.find(key);
            if (it != map.end()) {
                output = std::stof(it->second);
            }
        };
        load_float("ptz.pan_center_deg", config.ptz.pan_center_deg);
        load_float("ptz.pan_left_deg", config.ptz.pan_left_deg);
        load_float("ptz.pan_right_deg", config.ptz.pan_right_deg);
        load_float("ptz.tilt_center_deg", config.ptz.tilt_center_deg);
        load_float("ptz.tilt_up_deg", config.ptz.tilt_up_deg);
        load_float("ptz.tilt_down_deg", config.ptz.tilt_down_deg);

        load_double(map, "calib.bias_x", config.calib.bias_x);
        load_double(map, "calib.bias_y", config.calib.bias_y);
        load_double(map, "calib.bias_z", config.calib.bias_z);
        load_double(map, "calib.imu_offset_ms", config.calib.imu_offset_ms);
        load_string(map, "calib.ts_source", config.calib.ts_source);
        load_string(map, "calib.last_calibration", config.calib.last_calibration);
        load_int(map, "calib.bias_duration_ms", config.calib.bias_duration_ms);
        load_int(map, "calib.sweep_duration_ms", config.calib.sweep_duration_ms);
        load_double(map, "calib.coarse_range_ms", config.calib.coarse_range_ms);
        load_double(map, "calib.coarse_step_ms", config.calib.coarse_step_ms);
        load_double(map, "calib.fine_range_ms", config.calib.fine_range_ms);
        load_double(map, "calib.fine_step_ms", config.calib.fine_step_ms);
        return true;
    } catch (const std::exception& ex) {
        if (error) *error = ex.what();
        return false;
    }
}

bool write_app_config(const std::string& path, const AppConfig& config, std::string* error) {
    try {
        std::ofstream output(path, std::ios::trunc);
        if (!output.is_open()) {
            if (error) *error = "failed to open config for writing: " + path;
            return false;
        }

        AppConfig copy = config;
        if (copy.calib.last_calibration.empty()) {
            copy.calib.last_calibration = "never";
        }
        output << "# Updated by hybrid EIS runtime tools.\n";
        output << "# Last write: " << now_string() << "\n\n";

        std::ostringstream oss;
        append_section(oss, "camera");
        append_kv(oss, "width", copy.camera.width);
        append_kv(oss, "height", copy.camera.height);
        append_kv(oss, "fps", copy.camera.fps);
        append_kv(oss, "frame_duration_us", copy.camera.frame_duration_us);
        append_kv(oss, "exposure_us", copy.camera.exposure_us);
        append_kv(oss, "flip", copy.camera.flip ? 1 : 0);
        append_kv(oss, "hfov_deg", copy.camera.hfov_deg);
        append_kv(oss, "vfov_deg", copy.camera.vfov_deg);
        append_kv(oss, "libcamera_xrgb", copy.camera.libcamera_xrgb ? 1 : 0);
        append_kv(oss, "capture_backend", copy.camera.capture_backend);
        oss << "\n";

        append_section(oss, "imu");
        append_kv(oss, "bus", copy.imu.bus);
        append_kv(oss, "addr", copy.imu.addr);
        append_kv(oss, "int_gpio_chip", copy.imu.int_gpio_chip);
        append_kv(oss, "int_line_offset", copy.imu.int_line_offset);
        append_kv(oss, "int_pin_wpi", copy.imu.int_pin_wpi);
        append_kv(oss, "target_hz", copy.imu.target_hz);
        append_kv(oss, "use_fifo", copy.imu.use_fifo ? 1 : 0);
        append_kv(oss, "gyro_sensitivity", copy.imu.gyro_sensitivity);
        append_kv(oss, "axis_roll", copy.imu.axis_roll);
        append_kv(oss, "axis_pitch", copy.imu.axis_pitch);
        append_kv(oss, "axis_yaw", copy.imu.axis_yaw);
        append_kv(oss, "sign_roll", copy.imu.sign_roll);
        append_kv(oss, "sign_pitch", copy.imu.sign_pitch);
        append_kv(oss, "sign_yaw", copy.imu.sign_yaw);
        oss << "\n";

        append_section(oss, "eis");
        append_kv(oss, "lk_max_features", copy.eis.lk_max_features);
        append_kv(oss, "lk_min_features", copy.eis.lk_min_features);
        append_kv(oss, "lk_min_inliers", copy.eis.lk_min_inliers);
        append_kv(oss, "lk_quality", copy.eis.lk_quality);
        append_kv(oss, "lk_min_dist", copy.eis.lk_min_dist);
        append_kv(oss, "lk_ransac_thresh", copy.eis.lk_ransac_thresh);
        append_kv(oss, "lk_confidence_gate", copy.eis.lk_confidence_gate);
        append_kv(oss, "lk_translation_alpha", copy.eis.lk_translation_alpha);
        append_kv(oss, "lk_translation_max_corr_px", copy.eis.lk_translation_max_corr_px);
        append_kv(oss, "lk_translation_turn_scale", copy.eis.lk_translation_turn_scale);
        append_kv(oss, "lk_rotation_anchor_alpha", copy.eis.lk_rotation_anchor_alpha);
        append_kv(oss, "lk_rotation_gain", copy.eis.lk_rotation_gain);
        append_kv(oss, "gyro_gain_roll", copy.eis.gyro_gain_roll);
        append_kv(oss, "gyro_gain_pitch", copy.eis.gyro_gain_pitch);
        append_kv(oss, "gyro_gain_yaw", copy.eis.gyro_gain_yaw);
        append_kv(oss, "gyro_max_roll_deg", copy.eis.gyro_max_roll_deg);
        append_kv(oss, "gyro_max_pitch_deg", copy.eis.gyro_max_pitch_deg);
        append_kv(oss, "gyro_max_yaw_deg", copy.eis.gyro_max_yaw_deg);
        append_kv(oss, "gyro_hp_lpf_alpha", copy.eis.gyro_hp_lpf_alpha);
        append_kv(oss, "gyro_hp_gain_roll", copy.eis.gyro_hp_gain_roll);
        append_kv(oss, "gyro_hp_gain_pitch", copy.eis.gyro_hp_gain_pitch);
        append_kv(oss, "gyro_hp_gain_yaw", copy.eis.gyro_hp_gain_yaw);
        append_kv(oss, "gyro_large_rot_thresh_deg", copy.eis.gyro_large_rot_thresh_deg);
        append_kv(oss, "gyro_large_rot_gain_scale", copy.eis.gyro_large_rot_gain_scale);
        append_kv(oss, "crop_budget_percent", copy.eis.crop_budget_percent);
        append_kv(oss, "turn_enter_yaw_rate_dps", copy.eis.turn_enter_yaw_rate_dps);
        append_kv(oss, "turn_exit_yaw_rate_dps", copy.eis.turn_exit_yaw_rate_dps);
        append_kv(oss, "turn_hold_frames", copy.eis.turn_hold_frames);
        append_kv(oss, "recover_frames", copy.eis.recover_frames);
        append_kv(oss, "turn_follow_correction_scale", copy.eis.turn_follow_correction_scale);
        append_kv(oss, "debug_overlay", copy.eis.debug_overlay ? 1 : 0);
        oss << "\n";

        append_section(oss, "rtsp");
        append_kv(oss, "port", copy.rtsp.port);
        append_kv(oss, "path", copy.rtsp.path);
        append_kv(oss, "raw_path", copy.rtsp.raw_path);
        append_kv(oss, "bitrate", copy.rtsp.bitrate);
        append_kv(oss, "iframe_period", copy.rtsp.iframe_period);
        oss << "\n";

        append_section(oss, "mqtt");
        append_kv(oss, "host", copy.mqtt.host);
        append_kv(oss, "port", copy.mqtt.port);
        append_kv(oss, "keepalive_sec", copy.mqtt.keepalive_sec);
        append_kv(oss, "control_topic", copy.mqtt.control_topic);
        append_kv(oss, "goal_topic", copy.mqtt.goal_topic);
        append_kv(oss, "pose_topic", copy.mqtt.pose_topic);
        append_kv(oss, "safety_topic", copy.mqtt.safety_topic);
        append_kv(oss, "status_topic", copy.mqtt.status_topic);
        append_kv(oss, "status_publish_interval_ms", copy.mqtt.status_publish_interval_ms);
        oss << "\n";

        append_section(oss, "auto");
        append_kv(oss, "k_linear", copy.auto_control.k_linear);
        append_kv(oss, "k_yaw", copy.auto_control.k_yaw);
        append_kv(oss, "max_speed_cmps", copy.auto_control.max_speed_cmps);
        append_kv(oss, "max_yaw_rate_rps", copy.auto_control.max_yaw_rate_rps);
        append_kv(oss, "tolerance_cm", copy.auto_control.tolerance_cm);
        append_kv(oss, "rotate_yaw_offset_deg", copy.auto_control.rotate_yaw_offset_rad * 180.0 / kPi);
        oss << "\n";

        append_section(oss, "motor");
        append_kv(oss, "track_width_cm", copy.motor.track_width_cm);
        append_kv(oss, "wheel_max_speed_cmps", copy.motor.wheel_max_speed_cmps);
        append_kv(oss, "speed_deadband_cmps", copy.motor.speed_deadband_cmps);
        append_kv(oss, "pwm_min_effective", copy.motor.pwm_min_effective);
        append_kv(oss, "pwm_max", copy.motor.pwm_max);
        oss << "\n";

        append_section(oss, "manual");
        append_kv(oss, "default_pwm", copy.manual_drive.default_pwm);
        oss << "\n";

        append_section(oss, "controller");
        append_kv(oss, "enabled", copy.controller.enabled ? 1 : 0);
        append_kv(oss, "input_device", copy.controller.input_device);
        append_kv(oss, "device_name_hint", copy.controller.device_name_hint);
        append_kv(oss, "idle_stop_ms", copy.controller.idle_stop_ms);
        append_kv(oss, "speed_step", copy.controller.speed_step);
        append_kv(oss, "log_only", copy.controller.log_only ? 1 : 0);
        oss << "\n";

        append_section(oss, "http");
        append_kv(oss, "enable", copy.http.enable ? 1 : 0);
        append_kv(oss, "port", copy.http.port);
        append_kv(oss, "web_root", copy.http.web_root);
        oss << "\n";

        append_section(oss, "ptz");
        append_kv(oss, "i2c_device", copy.ptz.i2c_device);
        append_kv(oss, "i2c_address", copy.ptz.i2c_address);
        append_kv(oss, "pwm_frequency_hz", copy.ptz.pwm_frequency_hz);
        append_kv(oss, "pan_channel", copy.ptz.pan_channel);
        append_kv(oss, "tilt_channel", copy.ptz.tilt_channel);
        append_kv(oss, "pan_center_deg", copy.ptz.pan_center_deg);
        append_kv(oss, "pan_left_deg", copy.ptz.pan_left_deg);
        append_kv(oss, "pan_right_deg", copy.ptz.pan_right_deg);
        append_kv(oss, "tilt_center_deg", copy.ptz.tilt_center_deg);
        append_kv(oss, "tilt_up_deg", copy.ptz.tilt_up_deg);
        append_kv(oss, "tilt_down_deg", copy.ptz.tilt_down_deg);
        append_kv(oss, "imu_timeout_ms", copy.ptz.imu_timeout_ms);
        oss << "\n";

        append_section(oss, "calib");
        append_kv(oss, "bias_x", copy.calib.bias_x);
        append_kv(oss, "bias_y", copy.calib.bias_y);
        append_kv(oss, "bias_z", copy.calib.bias_z);
        append_kv(oss, "imu_offset_ms", copy.calib.imu_offset_ms);
        append_kv(oss, "ts_source", copy.calib.ts_source);
        append_kv(oss, "last_calibration", copy.calib.last_calibration);
        append_kv(oss, "bias_duration_ms", copy.calib.bias_duration_ms);
        append_kv(oss, "sweep_duration_ms", copy.calib.sweep_duration_ms);
        append_kv(oss, "coarse_range_ms", copy.calib.coarse_range_ms);
        append_kv(oss, "coarse_step_ms", copy.calib.coarse_step_ms);
        append_kv(oss, "fine_range_ms", copy.calib.fine_range_ms);
        append_kv(oss, "fine_step_ms", copy.calib.fine_step_ms);

        output << oss.str();
        return true;
    } catch (const std::exception& ex) {
        if (error) *error = ex.what();
        return false;
    }
}
