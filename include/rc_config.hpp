#pragma once

#include <string>

#include "rc_control_types.hpp"

bool LoadRcParamsFromIni(const std::string& ini_path, RcAppConfig& config);
