/**
 * @file wiring.c
 * @brief Arduino timing functions — millis, micros, delay, delayMicroseconds
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Uses SysTick (configured for 1 ms period at HCLK = SystemCoreClock / 2).
 *
 * SPDX-License-Identifier: MIT
 */

#include "Arduino.h"

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
    uint32_t ms, tick, tick2;
    uint32_t load = SysTick->LOAD + 1;  /* reload value = HCLK/1000 */

    /*
     * Read millis count and SysTick->VAL atomically.
     * SysTick is a down-counter: 0 = just reloaded, LOAD = just started.
     * Microsecond offset = (LOAD - VAL) / (HCLK / 1_000_000)
     * where HCLK = SystemCoreClock / 2.
     */
    do {
        ms   = _millis_count;
        tick = SysTick->VAL;
        tick2 = _millis_count;
    } while (ms != tick2);

    /* ticks elapsed within this millisecond */
    uint32_t elapsed_ticks = load - 1 - tick;
    /* HCLK ticks per microsecond */
    uint32_t ticks_per_us = (SystemCoreClock / 2) / 1000000UL;

    return (ms * 1000UL) + (elapsed_ticks / ticks_per_us);
}

/* ---- delay() ---- */
void delay(unsigned long ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms) {
        yield();
    }
}

/* ---- delayMicroseconds() ---- */
void delayMicroseconds(unsigned int us)
{
    /*
     * DWT cycle-counter based delay for sub-microsecond accuracy.
     * HCLK = SystemCoreClock / 2 cycles per second.
     */
    if (us == 0) return;

    /* Enable DWT cycle counter if not already running */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t hclk = SystemCoreClock / 2;
    uint32_t cycles = (hclk / 1000000UL) * us;
    uint32_t start = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < cycles) {
        /* spin */
    }
}
