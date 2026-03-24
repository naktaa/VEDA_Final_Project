#pragma once

#include <string>

#include "rc_control_types.hpp"

void PrintAutoUsage(const char* exe);
bool ParseAutoConfig(int argc, char** argv, RcAppConfig& config, std::string& error);
int RunAutoApp(const RcAppConfig& config);
