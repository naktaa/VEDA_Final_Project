#include "server_app.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    veda_server::ServerConfig config;
    std::string error;
    if (!veda_server::ParseServerConfig(argc, argv, config, error)) {
        std::cerr << "[ERR] " << error << "\n";
        veda_server::PrintServerUsage(argv[0]);
        return 1;
    }

    return veda_server::RunServerApp(config);
}
