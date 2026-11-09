#ifndef WIRING_TIMING_H
#define WIRING_TIMING_H

#include <stdint.h>

static inline uint32_t air105_micros_from_systick(
    uint32_t millis_count,
    uint32_t first_value,
    uint32_t current_value,
    uint32_t reload,
    uint32_t clock_hz,
    uint32_t tick_pending)
{
    uint32_t elapsed_cycles;
    uint32_t tick_count = millis_count;

    if (tick_pending || first_value < current_value) {
        tick_count++;
    }

    elapsed_cycles = reload - current_value;
    return tick_count * 1000UL
        + (uint32_t)(((uint64_t)elapsed_cycles * 1000000ULL) / clock_hz);
}

static inline uint32_t air105_systick_delta(
    uint32_t previous_value,
    uint32_t current_value,
    uint32_t period)
{
    if (previous_value >= current_value) {
        return previous_value - current_value;
    }

    return previous_value + period - current_value;
}

static inline uint64_t air105_microseconds_to_cycles(
    uint32_t microseconds,
    uint32_t clock_hz)
{
    return ((uint64_t)microseconds * clock_hz) / 1000000ULL;
}

#endif