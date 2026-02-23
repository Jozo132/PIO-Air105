/**
 * @file wiring_extras.cpp
 * @brief C++ wrappers for Arduino functions that need overloading
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * The random() function conflicts with newlib's random(void) in C mode,
 * so the overloaded versions are defined here in C++.
 *
 * SPDX-License-Identifier: MIT
 */

#include "Arduino.h"

/* ---- random() overloads (C++ only) ---- */
extern "C" long arduino_random(long howbig);

long random(long howbig)
{
    return arduino_random(howbig);
}

long random(long howsmall, long howbig)
{
    if (howsmall >= howbig) return howsmall;
    long diff = howbig - howsmall;
    return arduino_random(diff) + howsmall;
}
