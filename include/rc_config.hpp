#pragma once

#include <filesystem>
#include <string>

#include "rc_control_types.hpp"

bool EnsureRcLocalConfigExists(const std::filesystem::path& template_path,
                               const std::filesystem::path& local_path,
                               bool* created,
                               std::string* error);
bool LoadRcParamsFromIni(const std::string& ini_path, RcAppConfig& config);
