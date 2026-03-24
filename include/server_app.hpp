#pragma once

#include <string>

#include "server_types.hpp"

namespace veda_server {

void PrintServerUsage(const char* exe);
bool ParseServerConfig(int argc, char** argv, ServerConfig& config, std::string& error);
int RunServerApp(const ServerConfig& config);

} // namespace veda_server
