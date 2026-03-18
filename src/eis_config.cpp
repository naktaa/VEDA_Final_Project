#include "eis_config.hpp"

#include <fstream>
#include <sstream>
#include <algorithm>

static inline std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

bool load_config_map(const std::string& path, ConfigMap& out) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    std::string line;
    while (std::getline(f, line)) {
        std::string s = trim(line);
        if (s.empty()) continue;
        if (s[0] == '#' || s[0] == ';') continue;
        auto pos = s.find('=');
        if (pos == std::string::npos) continue;
        std::string key = trim(s.substr(0, pos));
        std::string val = trim(s.substr(pos + 1));
        if (key.empty()) continue;
        out[key] = val;
    }
    return true;
}

bool load_config_if_exists(const std::string& path, ConfigMap& out) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    f.close();
    return load_config_map(path, out);
}
