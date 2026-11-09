/**
 * @file wiring.c
 * @brief Arduino timing functions — millis, micros, delay, delayMicroseconds
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * SysTick uses the 204 MHz processor clock and generates a 1 ms interrupt.
 *
 * SPDX-License-Identifier: MIT
 */

#include "Arduino.h"
#include "wiring_timing.h"

static void wait_cycles(uint64_t cycles)
{
    uint32_t period = SysTick->LOAD + 1UL;
    uint32_t previous_value = SysTick->VAL;

    while (cycles != 0) {
        uint32_t current_value = SysTick->VAL;
        uint32_t elapsed_cycles = air105_systick_delta(
            previous_value, current_value, period
        );

        if (elapsed_cycles >= cycles) {
            break;
        }

        cycles -= elapsed_cycles;
        previous_value = current_value;
    }
}

/* ---- init() — hardware setup before user sketch ---- */
void init(void)
{
    /* SysTick → 1 ms interrupts */
    SysTick_Init();
}

/* ---- millis() ---- */
unsigned long millis(void)
{
    return _millis_count;
}

/* ---- micros() ---- */
unsigned long micros(void)
{
    uint32_t ms, ms_after, first_value, current_value, tick_pending;
    uint32_t reload = SysTick->LOAD;

    do {
        ms = _millis_count;
        first_value = SysTick->VAL;
        tick_pending = SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;
        current_value = SysTick->VAL;
        ms_after = _millis_count;
    } while (ms != ms_after);

    return air105_micros_from_systick(
        ms, first_value, current_value, reload, SystemCoreClock, tick_pending
    );
}

/* ---- delay() ---- */
void delay(unsigned long ms)
{
    if (ms == 0) {
        return;
    }

    if (__get_PRIMASK() != 0U || __get_IPSR() != 0U) {
        wait_cycles((uint64_t)ms * (SystemCoreClock / 1000UL));
        return;
    }

    uint64_t remaining_us = (uint64_t)ms * 1000ULL;
    while (remaining_us != 0) {
        uint32_t chunk_us = remaining_us > UINT32_MAX
            ? UINT32_MAX
            : (uint32_t)remaining_us;
        uint32_t start = micros();

        while ((uint32_t)(micros() - start) < chunk_us) {
            yield();
        }

        remaining_us -= chunk_us;
    }
}

/* ---- delayMicroseconds() ---- */
void delayMicroseconds(unsigned int us)
{
    if (us == 0) {
        return;
    }

    uint64_t cycles = air105_microseconds_to_cycles(us, SystemCoreClock);
    if (us <= 10U || __get_PRIMASK() != 0U || __get_IPSR() != 0U) {
        wait_cycles(cycles);
        return;
    }

    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < us) {
        yield();
    }
}
