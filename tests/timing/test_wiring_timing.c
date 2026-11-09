#include <assert.h>
#include <stdint.h>

#include "../../system/clock_config.h"
#include "../../cores/air105/wiring_timing.h"

_Static_assert(F_CPU == AIR105_STARTUP_CPU_CLOCK_HZ,
               "compile-time clock must match startup clock");

int main(void)
{
    const uint32_t reload = 203999U;
    const uint32_t clock_hz = 204000000U;

    assert(air105_micros_from_systick(12U, reload, reload, reload,
                                      clock_hz, 0U) == 12000U);
    assert(air105_micros_from_systick(12U, reload - 204U, reload - 204U,
                                      reload, clock_hz, 0U) == 12001U);

    /* A pending SysTick or a VAL increase means the ISR has not accounted
       for the just-completed period yet. */
    assert(air105_micros_from_systick(12U, 1U, reload - 204U, reload,
                                      clock_hz, 0U) == 13001U);
    assert(air105_micros_from_systick(12U, reload - 204U, reload - 204U,
                                      reload, clock_hz, 1U) == 13001U);

    /* The public micros clock wraps at 32 bits, including the millisecond
       multiplication and the sub-millisecond phase. */
    assert(air105_micros_from_systick(UINT32_MAX, reload, reload, reload,
                                      clock_hz, 0U) == UINT32_MAX - 999U);
    assert(air105_micros_from_systick(4294967U, reload - 204U * 296U,
                                      reload - 204U * 296U, reload,
                                      clock_hz, 0U) == 0U);

    assert(air105_systick_delta(100U, 40U, 204000U) == 60U);
    assert(air105_systick_delta(1U, 203997U, 204000U) == 4U);
    assert(air105_microseconds_to_cycles(0U, clock_hz) == 0U);
    assert(air105_microseconds_to_cycles(1U, clock_hz) == 204U);
    assert(air105_microseconds_to_cycles(50000U, clock_hz) == 10200000U);

    return 0;
}