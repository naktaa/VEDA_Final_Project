#include "vr_ui_command_bridge.hpp"

void VrUiCommandBridge::handle_session_button() {
    const bool currently_active = session_active_.load(std::memory_order_relaxed);
    const bool vr_mode_active = vr_mode_active_.load(std::memory_order_relaxed);
    if (!currently_active || !vr_mode_active) {
        session_active_.store(true, std::memory_order_relaxed);
        vr_mode_active_.store(true, std::memory_order_relaxed);
        session_start_seq_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    if (currently_active) {
        session_active_.store(false, std::memory_order_relaxed);
        vr_mode_active_.store(false, std::memory_order_relaxed);
        session_stop_seq_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
}

void VrUiCommandBridge::set_state(bool session_active, bool vr_mode_active) {
    session_active_.store(session_active, std::memory_order_relaxed);
    vr_mode_active_.store(vr_mode_active, std::memory_order_relaxed);
}

void VrUiCommandBridge::request_zero_calibrate() {
    zero_calibrate_seq_.fetch_add(1, std::memory_order_relaxed);
}

VrUiCommandSnapshot VrUiCommandBridge::snapshot() const {
    VrUiCommandSnapshot snapshot;
    snapshot.session_start_seq = session_start_seq_.load(std::memory_order_relaxed);
    snapshot.session_stop_seq = session_stop_seq_.load(std::memory_order_relaxed);
    snapshot.zero_calibrate_seq = zero_calibrate_seq_.load(std::memory_order_relaxed);
    snapshot.session_active = session_active_.load(std::memory_order_relaxed);
    snapshot.vr_mode_active = vr_mode_active_.load(std::memory_order_relaxed);
    return snapshot;
}
