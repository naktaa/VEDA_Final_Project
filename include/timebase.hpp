#pragma once

#include <cstdint>
#include <time.h>

int64_t clock_ns(clockid_t id);
int64_t monotonic_raw_origin_ns();
double now_ms();
