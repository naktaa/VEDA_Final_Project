#include <cstdio>
#include <string>

#include "app_config.hpp"
#include "offset_calibrator.hpp"

namespace {

constexpr const char* kTemplateConfigPath = "config_template.ini";
constexpr const char* kLocalConfigPath = "config_local.ini";

} // namespace

int main() {
    bool created = false;
    std::string error;
    if (!ensure_local_config_exists(kTemplateConfigPath, kLocalConfigPath, &created, &error)) {
        std::fprintf(stderr, "[CALIB] config setup failed: %s\n", error.c_str());
        return 1;
    }
    if (created) {
        std::fprintf(stderr, "[CALIB] created %s from template. Review it and run again.\n", kLocalConfigPath);
        return 1;
    }

    AppConfig config;
    if (!load_app_config(kLocalConfigPath, config, &error)) {
        std::fprintf(stderr, "[CALIB] config load failed: %s\n", error.c_str());
        return 1;
    }

    OffsetCalibrator calibrator;
    if (!calibrator.run(config, &error)) {
        std::fprintf(stderr, "[CALIB] calibration failed: %s\n", error.c_str());
        return 1;
    }

    if (!write_app_config(kLocalConfigPath, config, &error)) {
        std::fprintf(stderr, "[CALIB] config write failed: %s\n", error.c_str());
        return 1;
    }

    std::fprintf(stderr,
                 "[CALIB] saved bias=(%.2f, %.2f, %.2f) offset=%.2fms -> %s\n",
                 config.calib.bias_x,
                 config.calib.bias_y,
                 config.calib.bias_z,
                 config.calib.imu_offset_ms,
                 kLocalConfigPath);
    return 0;
}
