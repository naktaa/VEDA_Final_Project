#pragma once

#include <atomic>
#include <cstdint>

struct VrUiCommandSnapshot {
    uint64_t session_toggle_seq = 0;
    uint64_t zero_calibrate_seq = 0;
};

class VrUiCommandBridge {
public:
    void request_session_toggle();
    void request_zero_calibrate();
    VrUiCommandSnapshot snapshot() const;

private:
    std::atomic<uint64_t> session_toggle_seq_{0};
    std::atomic<uint64_t> zero_calibrate_seq_{0};
};
