#include "auto_app.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    RcAppConfig config;
    ResolveAutoPaths(argv[0], config);

    std::string error;
    if (!ParseAutoConfig(argc, argv, config, error)) {
        std::cerr << "[ERR] " << error << "\n";
        PrintAutoUsage(argv[0]);
        return 1;
    }

    return RunAutoApp(config);
}
