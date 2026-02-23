/**
 * @file Printable.h
 * @brief Interface for objects that can print themselves
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 * SPDX-License-Identifier: MIT
 */
#ifndef Printable_h
#define Printable_h

#include <stddef.h>

class Print;

class Printable {
public:
    virtual size_t printTo(Print &p) const = 0;
};

#endif /* Printable_h */
