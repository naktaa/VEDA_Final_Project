#include "rc_config.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {

constexpr double kMToCm = 100.0;

std::string TrimCopy(const std::string& s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

} // namespace

bool LoadRcParamsFromIni(const std::string& ini_path, RcAppConfig& config) {
    std::ifstream fin(ini_path);
    if (!fin.is_open()) {
        std::cerr << "[WARN] failed to open ini: " << ini_path << "\n";
        return false;
    }

    bool used_legacy_meter_keys = false;
    std::string line;
    std::string section;
    int line_no = 0;

    while (std::getline(fin, line)) {
        ++line_no;
        const std::string trimmed = TrimCopy(line);
        if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';') {
            continue;
        }

        if (trimmed.front() == '[' && trimmed.back() == ']') {
            section = TrimCopy(trimmed.substr(1, trimmed.size() - 2));
            continue;
        }

        const auto eq = trimmed.find('=');
        if (eq == std::string::npos) {
            std::cerr << "[WARN] invalid ini line " << line_no << ": " << trimmed << "\n";
            continue;
        }

        const std::string key = TrimCopy(trimmed.substr(0, eq));
        const std::string value = TrimCopy(trimmed.substr(eq + 1));
        const std::string scoped_key = section.empty() ? key : section + "." + key;

        try {
            if (scoped_key == "control.k_linear") {
                config.control.k_linear = std::stod(value);
            } else if (scoped_key == "control.k_yaw") {
                config.control.k_yaw = std::stod(value);
            } else if (scoped_key == "control.max_speed_cmps") {
                config.control.max_speed_cmps = std::stod(value);
            } else if (scoped_key == "control.max_speed_mps") {
                config.control.max_speed_cmps = std::stod(value) * kMToCm;
                used_legacy_meter_keys = true;
            } else if (scoped_key == "control.max_yaw_rate_rps") {
                config.control.max_yaw_rate_rps = std::stod(value);
            } else if (scoped_key == "control.tolerance_cm") {
                config.control.tolerance_cm = std::stod(value);
            } else if (scoped_key == "control.tolerance_m") {
                config.control.tolerance_cm = std::stod(value) * kMToCm;
                used_legacy_meter_keys = true;
            } else if (scoped_key == "motor.track_width_cm") {
                config.motor.track_width_cm = std::stod(value);
            } else if (scoped_key == "motor.track_width_m") {
                config.motor.track_width_cm = std::stod(value) * kMToCm;
                used_legacy_meter_keys = true;
            } else if (scoped_key == "motor.wheel_max_speed_cmps") {
                config.motor.wheel_max_speed_cmps = std::stod(value);
            } else if (scoped_key == "motor.wheel_max_speed_mps") {
                config.motor.wheel_max_speed_cmps = std::stod(value) * kMToCm;
                used_legacy_meter_keys = true;
            } else if (scoped_key == "motor.speed_deadband_cmps") {
                config.motor.speed_deadband_cmps = std::stod(value);
            } else if (scoped_key == "motor.speed_deadband_mps") {
                config.motor.speed_deadband_cmps = std::stod(value) * kMToCm;
                used_legacy_meter_keys = true;
            } else if (scoped_key == "motor.pwm_min_effective") {
                config.motor.pwm_min_effective = std::stoi(value);
            } else if (scoped_key == "motor.pwm_max") {
                config.motor.pwm_max = std::stoi(value);
            } else {
                std::cerr << "[WARN] unknown tuning ini key: " << scoped_key << "\n";
            }
        } catch (const std::exception&) {
            std::cerr << "[WARN] invalid ini value for " << scoped_key
                      << " at line " << line_no << ": " << value << "\n";
        }
    }

    if (used_legacy_meter_keys) {
        std::cerr << "[WARN] legacy *_m*/*_mps* RC config keys detected; converted to centimeter-based values\n";
    }

    std::cout << "[OK] loaded tuning params from " << ini_path
              << " | control=(" << config.control.k_linear << ", " << config.control.k_yaw << ", "
              << config.control.max_speed_cmps << ", " << config.control.max_yaw_rate_rps << ", "
              << config.control.tolerance_cm << ")"
              << " motor=(" << config.motor.track_width_cm << ", " << config.motor.wheel_max_speed_cmps
              << ", " << config.motor.speed_deadband_cmps << ", " << config.motor.pwm_min_effective
              << ", " << config.motor.pwm_max << ")\n";
    return true;
}

bool EnsureRcLocalConfigExists(const std::filesystem::path& template_path,
                               const std::filesystem::path& local_path,
                               bool* created,
                               std::string* error) {
    if (created) {
        *created = false;
    }

    try {
        if (std::filesystem::exists(local_path)) {
            return true;
        }

        std::filesystem::create_directories(local_path.parent_path());
        if (std::filesystem::exists(template_path)) {
            std::filesystem::copy_file(
                template_path,
                local_path,
                std::filesystem::copy_options::overwrite_existing);
        } else {
            if (error) {
                *error = "template ini not found: " + template_path.string();
            }
            return false;
        }

        if (created) {
            *created = true;
        }
        return true;
    } catch (const std::exception& ex) {
        if (error) {
            *error = ex.what();
        }
        return false;
    }
}
