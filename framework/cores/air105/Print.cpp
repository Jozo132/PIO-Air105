/**
 * @file Print.cpp
 * @brief Arduino Print class implementation
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 * SPDX-License-Identifier: MIT
 */

#include "Print.h"
#include <string.h>
#include <math.h>

/* ---- write buffer (default: byte-by-byte) ---- */
size_t Print::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        if (write(*buffer++)) n++;
        else break;
    }
    return n;
}

/* ---- print() implementations ---- */

/* On ARM, flash strings are directly accessible like regular strings */
size_t Print::print(const __FlashStringHelper *ifsh)
{
    return print(reinterpret_cast<const char *>(ifsh));
}

size_t Print::print(const char str[])
{
    return write(str);
}

size_t Print::print(char c)
{
    return write((uint8_t)c);
}

size_t Print::print(unsigned char b, int base)
{
    return print((unsigned long)b, base);
}

size_t Print::print(int n, int base)
{
    return print((long)n, base);
}

size_t Print::print(unsigned int n, int base)
{
    return print((unsigned long)n, base);
}

size_t Print::print(long n, int base)
{
    if (base == 0) {
        return write((uint8_t)n);
    }
    if (base == 10) {
        if (n < 0) {
            size_t t = write('-');
            n = -n;
            return printNumber((unsigned long)n, 10) + t;
        }
        return printNumber((unsigned long)n, 10);
    }
    return printNumber((unsigned long)n, (uint8_t)base);
}

size_t Print::print(unsigned long n, int base)
{
    if (base == 0) return write((uint8_t)n);
    return printNumber(n, (uint8_t)base);
}

size_t Print::print(double n, int digits)
{
    return printFloat(n, (uint8_t)digits);
}

size_t Print::print(const Printable &x)
{
    return x.printTo(*this);
}

/* ---- println() implementations ---- */

size_t Print::println(void)
{
    return write("\r\n");
}

size_t Print::println(const __FlashStringHelper *ifsh)
{
    size_t n = print(ifsh);
    n += println();
    return n;
}

size_t Print::println(const char c[])
{
    size_t n = print(c);
    n += println();
    return n;
}

size_t Print::println(char c)
{
    size_t n = print(c);
    n += println();
    return n;
}

size_t Print::println(unsigned char b, int base)
{
    size_t n = print(b, base);
    n += println();
    return n;
}

size_t Print::println(int num, int base)
{
    size_t n = print(num, base);
    n += println();
    return n;
}

size_t Print::println(unsigned int num, int base)
{
    size_t n = print(num, base);
    n += println();
    return n;
}

size_t Print::println(long num, int base)
{
    size_t n = print(num, base);
    n += println();
    return n;
}

size_t Print::println(unsigned long num, int base)
{
    size_t n = print(num, base);
    n += println();
    return n;
}

size_t Print::println(double num, int digits)
{
    size_t n = print(num, digits);
    n += println();
    return n;
}

size_t Print::println(const Printable &x)
{
    size_t n = print(x);
    n += println();
    return n;
}

/* ---- Private helpers ---- */

size_t Print::printNumber(unsigned long n, uint8_t base)
{
    char buf[8 * sizeof(long) + 1]; /* worst case: BIN of 32-bit */
    char *str = &buf[sizeof(buf) - 1];
    *str = '\0';

    if (base < 2) base = 10;

    do {
        unsigned long m = n;
        n /= base;
        char c = (char)(m - base * n);
        *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while (n);

    return write(str);
}

size_t Print::printFloat(double number, uint8_t digits)
{
    size_t n = 0;

    if (isnan(number)) return print("nan");
    if (isinf(number)) return print("inf");
    if (number > 4294967040.0) return print("ovf");  /* constant determined empirically */
    if (number < -4294967040.0) return print("ovf");

    /* Handle negative numbers */
    if (number < 0.0) {
        n += print('-');
        number = -number;
    }

    /* Round correctly so that print(1.999, 2) prints "2.00" */
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; i++)
        rounding /= 10.0;
    number += rounding;

    /* Extract integer part */
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    n += print(int_part);

    /* Print decimal point and fractional part */
    if (digits > 0) {
        n += print('.');
        while (digits-- > 0) {
            remainder *= 10.0;
            unsigned int toPrint = (unsigned int)remainder;
            n += print(toPrint);
            remainder -= toPrint;
        }
    }

    return n;
}
