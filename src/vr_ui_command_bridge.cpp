#include "vr_ui_command_bridge.hpp"

void VrUiCommandBridge::handle_session_button() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!snapshot_.session_active) {
        snapshot_.session_start_seq += 1;
        return;
    }
    snapshot_.session_stop_seq += 1;
}

void VrUiCommandBridge::request_zero_calibrate() {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.zero_calibrate_seq += 1;
}

void VrUiCommandBridge::set_state(bool session_active, bool vr_mode_active) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.session_active = session_active;
    snapshot_.vr_mode_active = vr_mode_active;
}

VrUiCommandSnapshot VrUiCommandBridge::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}
