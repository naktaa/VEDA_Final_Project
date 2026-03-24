#pragma once

#include <cstdint>
#include <mutex>

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
    void request_zero_calibrate();
    void set_state(bool session_active, bool vr_mode_active);
    VrUiCommandSnapshot snapshot() const;

private:
    mutable std::mutex mutex_;
    VrUiCommandSnapshot snapshot_;
};
