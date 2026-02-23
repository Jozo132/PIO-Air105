/**
 * @file Stream.cpp
 * @brief Arduino Stream helper implementations
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 * SPDX-License-Identifier: MIT
 */

#include "Stream.h"
#include "Arduino.h"

int Stream::timedRead()
{
    unsigned long start = millis();
    do {
        int c = read();
        if (c >= 0) return c;
    } while (millis() - start < _timeout);
    return -1;
}

int Stream::timedPeek()
{
    unsigned long start = millis();
    do {
        int c = peek();
        if (c >= 0) return c;
    } while (millis() - start < _timeout);
    return -1;
}

size_t Stream::readBytes(char *buffer, size_t length)
{
    size_t count = 0;
    while (count < length) {
        int c = timedRead();
        if (c < 0) break;
        *buffer++ = (char)c;
        count++;
    }
    return count;
}
