#include "rc_config.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {

constexpr double kCmToM = 0.01;

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

    bool used_legacy_cm_keys = false;
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
            } else if (scoped_key == "control.max_speed_mps") {
                config.control.max_speed_mps = std::stod(value);
            } else if (scoped_key == "control.max_speed_cmps") {
                config.control.max_speed_mps = std::stod(value) * kCmToM;
                used_legacy_cm_keys = true;
            } else if (scoped_key == "control.max_yaw_rate_rps") {
                config.control.max_yaw_rate_rps = std::stod(value);
            } else if (scoped_key == "control.tolerance_m") {
                config.control.tolerance_m = std::stod(value);
            } else if (scoped_key == "control.tolerance_cm") {
                config.control.tolerance_m = std::stod(value) * kCmToM;
                used_legacy_cm_keys = true;
            } else if (scoped_key == "motor.track_width_m") {
                config.motor.track_width_m = std::stod(value);
            } else if (scoped_key == "motor.track_width_cm") {
                config.motor.track_width_m = std::stod(value) * kCmToM;
                used_legacy_cm_keys = true;
            } else if (scoped_key == "motor.wheel_max_speed_mps") {
                config.motor.wheel_max_speed_mps = std::stod(value);
            } else if (scoped_key == "motor.wheel_max_speed_cmps") {
                config.motor.wheel_max_speed_mps = std::stod(value) * kCmToM;
                used_legacy_cm_keys = true;
            } else if (scoped_key == "motor.speed_deadband_mps") {
                config.motor.speed_deadband_mps = std::stod(value);
            } else if (scoped_key == "motor.speed_deadband_cmps") {
                config.motor.speed_deadband_mps = std::stod(value) * kCmToM;
                used_legacy_cm_keys = true;
            } else if (scoped_key == "motor.pwm_min_effective") {
                config.motor.pwm_min_effective = std::stoi(value);
            } else if (scoped_key == "motor.pwm_max") {
                config.motor.pwm_max = std::stoi(value);
            } else {
                std::cerr << "[WARN] unknown ini key: " << scoped_key << "\n";
            }
        } catch (const std::exception&) {
            std::cerr << "[WARN] invalid ini value for " << scoped_key
                      << " at line " << line_no << ": " << value << "\n";
        }
    }

    if (used_legacy_cm_keys) {
        std::cerr << "[WARN] legacy *_cm* RC config keys detected; converted to meter-based values\n";
    }

    std::cout << "[OK] loaded ini params from " << ini_path
              << " | control=(" << config.control.k_linear << ", " << config.control.k_yaw << ", "
              << config.control.max_speed_mps << ", " << config.control.max_yaw_rate_rps << ", "
              << config.control.tolerance_m << ")"
              << " motor=(" << config.motor.track_width_m << ", " << config.motor.wheel_max_speed_mps
              << ", " << config.motor.speed_deadband_mps << ", " << config.motor.pwm_min_effective
              << ", " << config.motor.pwm_max << ")\n";
    return true;
}
