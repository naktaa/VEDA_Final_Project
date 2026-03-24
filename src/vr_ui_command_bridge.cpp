#include "vr_ui_command_bridge.hpp"

void VrUiCommandBridge::request_session_toggle() {
    session_toggle_seq_.fetch_add(1, std::memory_order_relaxed);
}

void VrUiCommandBridge::request_zero_calibrate() {
    zero_calibrate_seq_.fetch_add(1, std::memory_order_relaxed);
}

VrUiCommandSnapshot VrUiCommandBridge::snapshot() const {
    VrUiCommandSnapshot snapshot;
    snapshot.session_toggle_seq = session_toggle_seq_.load(std::memory_order_relaxed);
    snapshot.zero_calibrate_seq = zero_calibrate_seq_.load(std::memory_order_relaxed);
    return snapshot;
}
