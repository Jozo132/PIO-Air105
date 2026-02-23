/**
 * @file main.cpp
 * @brief Arduino main() — init, setup/loop pattern
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * This is NOT user code. The Arduino build system compiles this file
 * as part of the core library. Users write setup() and loop().
 *
 * SPDX-License-Identifier: MIT
 */

#include "Arduino.h"

/* Weak default: users may override yield() for cooperative tasks */
void yield(void) __attribute__((weak));
void yield(void) {}

int main(void)
{
    /* Hardware-level init (SysTick, GPIO clocks, etc.) */
    init();

    /* User-provided setup */
    setup();

    /* Infinite loop calling user's loop() */
    for (;;) {
        loop();
        yield();
    }
    return 0;
}
