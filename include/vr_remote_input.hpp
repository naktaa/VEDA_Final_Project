#pragma once

#include <atomic>
#include <functional>
#include <string>

enum class SideButtonAction {
    kEstop,
    kModeToggle,
};

struct VrRemoteInputConfig {
    std::string input_device = "";
    std::string device_name_hint = "VR-PARK";
    int idle_stop_ms = 180;
    int speed_step = 10;
    bool log_only = false;
    SideButtonAction side_button_action = SideButtonAction::kEstop;
};

bool run_vr_remote_input_loop(const VrRemoteInputConfig& config,
                              std::atomic<bool>& running,
                              const std::function<void(const char*)>& on_manual_override = {},
                              const std::function<bool()>& on_side_button = {},
                              const std::function<bool()>& on_left_button = {});
