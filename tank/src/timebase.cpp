#include "timebase.hpp"

namespace {

int64_t init_origin_ns() {
    struct timespec ts {};
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int64_t g_origin_ns = init_origin_ns();

} // namespace

int64_t clock_ns(clockid_t id) {
    struct timespec ts {};
    clock_gettime(id, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int64_t monotonic_raw_origin_ns() {
    return g_origin_ns;
}

double now_ms() {
    return static_cast<double>(clock_ns(CLOCK_MONOTONIC_RAW) - g_origin_ns) / 1e6;
}
