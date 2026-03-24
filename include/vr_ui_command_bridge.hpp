#pragma once

#include <atomic>
#include <cstdint>

struct VrUiCommandSnapshot {
    uint64_t session_start_seq = 0;
    uint64_t session_stop_seq = 0;
    uint64_t zero_calibrate_seq = 0;
    bool session_active = false;
    bool vr_mode_active = false;
};

class VrUiCommandBridge {
public:
    void handle_session_button();
    void set_state(bool session_active, bool vr_mode_active);
    void request_zero_calibrate();
    VrUiCommandSnapshot snapshot() const;

private:
    std::atomic<uint64_t> session_start_seq_{0};
    std::atomic<uint64_t> session_stop_seq_{0};
    std::atomic<uint64_t> zero_calibrate_seq_{0};
    std::atomic<bool> session_active_{false};
    std::atomic<bool> vr_mode_active_{false};
};
