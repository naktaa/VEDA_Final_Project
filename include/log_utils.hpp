#pragma once

#include <cerrno>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>

namespace log_utils {

inline std::string file_timestamp() {
    const std::time_t now = std::time(nullptr);
    std::tm tm {};
    localtime_r(&now, &tm);
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &tm);
    return buffer;
}

inline std::string human_timestamp() {
    const std::time_t now = std::time(nullptr);
    std::tm tm {};
    localtime_r(&now, &tm);
    char buffer[64];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm);
    return buffer;
}

inline bool open_log_file(const std::string& prefix,
                          std::ofstream& stream,
                          std::string& out_path,
                          std::string* error = nullptr) {
    std::error_code ec;
    std::filesystem::create_directories("logs", ec);
    if (ec) {
        if (error) {
            *error = "failed to create logs directory: " + ec.message();
        }
        return false;
    }

    out_path = "logs/" + prefix + "_" + file_timestamp() + ".txt";
    stream.open(out_path, std::ios::out | std::ios::trunc);
    if (!stream.is_open()) {
        if (error) {
            *error = "failed to open " + out_path + ": " + std::strerror(errno);
        }
        return false;
    }
    return true;
}

} // namespace log_utils
