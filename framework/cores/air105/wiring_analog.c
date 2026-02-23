/**
 * @file wiring_analog.c
 * @brief Arduino analog I/O stubs — analogRead, analogWrite, analogReference
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Phase 1 stubs — full ADC/PWM/DAC implementation will follow.
 *
 * SPDX-License-Identifier: MIT
 */

#include "Arduino.h"

void analogReference(uint8_t type)
{
    (void)type;
    /* Air105 ADC has a fixed 0-1.8 V input range; nothing to configure. */
}

int analogRead(uint8_t pin)
{
    (void)pin;
    /* TODO: Implement ADC read via Air105 ADC peripheral */
    return 0;
}

void analogWrite(uint8_t pin, int val)
{
    (void)pin;
    (void)val;
    /* TODO: Implement PWM output via Air105 timer peripheral */
}

/* ---- random / map ---- */
static unsigned long _random_seed = 1;

void randomSeed(unsigned long seed)
{
    if (seed != 0) _random_seed = seed;
}

long arduino_random(long howbig)
{
    if (howbig == 0) return 0;
    /* Simple LCG — replace with TRNG if available */
    _random_seed = _random_seed * 1103515245UL + 12345UL;
    return (long)((_random_seed >> 16) % (unsigned long)howbig);
}

long map(long value, long fromLow, long fromHigh, long toLow, long toHigh)
{
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}
