#pragma once

#include <string>
#include <unordered_map>

using ConfigMap = std::unordered_map<std::string, std::string>;

// Load simple key=value config file. Lines starting with # or ; are ignored.
// Returns false if file not found or unreadable.
bool load_config_map(const std::string& path, ConfigMap& out);

// Helper: try load if file exists. Returns true when loaded.
bool load_config_if_exists(const std::string& path, ConfigMap& out);
