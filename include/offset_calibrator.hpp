#pragma once

#include <string>

#include "app_config.hpp"

class OffsetCalibrator {
public:
    bool run(AppConfig& config, std::string* error = nullptr);
};
