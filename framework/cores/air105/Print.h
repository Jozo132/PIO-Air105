/**
 * @file Print.h
 * @brief Arduino Print base class — provides print/println formatting
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 * SPDX-License-Identifier: MIT
 */
#ifndef Print_h
#define Print_h

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "Printable.h"

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class Print {
public:
    Print() : write_error(0) {}

    int getWriteError() { return write_error; }
    void clearWriteError() { write_error = 0; }

    /* Subclass must implement this */
    virtual size_t write(uint8_t c) = 0;

    /* Default: write buffer byte-by-byte */
    virtual size_t write(const uint8_t *buffer, size_t size);
    size_t write(const char *str) {
        if (str == NULL) return 0;
        return write((const uint8_t *)str, strlen(str));
    }
    size_t write(const char *buffer, size_t size) {
        return write((const uint8_t *)buffer, size);
    }

    /* ---- print() family ---- */
    size_t print(const char[]);
    size_t print(char);
    size_t print(unsigned char, int = DEC);
    size_t print(int, int = DEC);
    size_t print(unsigned int, int = DEC);
    size_t print(long, int = DEC);
    size_t print(unsigned long, int = DEC);
    size_t print(double, int = 2);
    size_t print(const Printable &);

    /* ---- println() family ---- */
    size_t println(const char[]);
    size_t println(char);
    size_t println(unsigned char, int = DEC);
    size_t println(int, int = DEC);
    size_t println(unsigned int, int = DEC);
    size_t println(long, int = DEC);
    size_t println(unsigned long, int = DEC);
    size_t println(double, int = 2);
    size_t println(const Printable &);
    size_t println(void);

protected:
    void setWriteError(int err = 1) { write_error = err; }

private:
    int write_error;
    size_t printNumber(unsigned long n, uint8_t base);
    size_t printFloat(double number, uint8_t digits);
};

#endif /* Print_h */
