#pragma once

#include <atomic>
#include <functional>
#include <string>

enum class SideButtonAction {
    kEstop,
    kModeToggle,
};

enum class VrUiAction {
    kToggleSession,
    kZeroCalibrate,
};

struct VrRemoteInputConfig {
    std::string input_device = "/dev/input/event4";
    int idle_stop_ms = 180;
    int speed_step = 10;
    bool log_only = false;
    SideButtonAction side_button_action = SideButtonAction::kEstop;
    std::function<void(VrUiAction)> ui_action_callback;
};

bool run_vr_remote_input_loop(const VrRemoteInputConfig& config, std::atomic<bool>& running);
