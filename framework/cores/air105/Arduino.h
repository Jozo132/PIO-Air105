/**
 * @file Arduino.h
 * @brief Main Arduino header for Air105 — pulls in all API declarations
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 * SPDX-License-Identifier: MIT
 */
#ifndef Arduino_h
#define Arduino_h

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* ---- Vendor CMSIS + register headers ---- */
#include "air105.h"

/* ---- Framework system header ---- */
#include "system_air105.h"

/* ---- Variant-specific pin definitions ---- */
#include "variant.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Arduino type aliases ---- */
typedef bool     boolean;
typedef uint8_t  byte;
typedef uint16_t word;

/* ---- Digital pin states ---- */
#define HIGH  0x1
#define LOW   0x0

/* ---- Pin modes ---- */
#define INPUT          0x0
#define OUTPUT         0x1
#define INPUT_PULLUP   0x2

/* ---- Interrupt modes ---- */
#define CHANGE   0x04
#define FALLING  0x03
#define RISING   0x02

/* ---- Analog reference (placeholder — Air105 has fixed 1.8 V range) ---- */
#define DEFAULT  0

/* ---- Bit/byte macros ---- */
#define PI          3.1415926535897932384626433832795
#define HALF_PI     1.5707963267948966192313216916398
#define TWO_PI      6.283185307179586476925286766559
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG  57.295779513082320876798154814105
#define EULER       2.718281828459045235360287471352

#define min(a, b)       ((a) < (b) ? (a) : (b))
#define max(a, b)       ((a) > (b) ? (a) : (b))
#define abs(x)          ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define round(x)        ((x) >= 0 ? (long)((x) + 0.5) : (long)((x) - 0.5))
#define radians(deg)    ((deg) * DEG_TO_RAD)
#define degrees(rad)    ((rad) * RAD_TO_DEG)
#define sq(x)           ((x) * (x))

#define lowByte(w)      ((uint8_t)((w) & 0xFF))
#define highByte(w)     ((uint8_t)((w) >> 8))

#define bitRead(value, bit)            (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)             ((value) |= (1UL << (bit)))
#define bitClear(value, bit)           ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit)          ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) \
    ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define bit(b) (1UL << (b))

/* ---- Timing ---- */
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

/* ---- Digital I/O ---- */
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int  digitalRead(uint8_t pin);

/* ---- Analog I/O (stubs for Phase 1) ---- */
int  analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);
void analogReference(uint8_t type);

/* ---- Interrupts ---- */
void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode);
void detachInterrupt(uint8_t interruptNum);
#define digitalPinToInterrupt(p) (p)   /* Air105: every GPIO can interrupt */

#define interrupts()    __enable_irq()
#define noInterrupts()  __disable_irq()

/* ---- Random ---- */
void randomSeed(unsigned long seed);
long map(long value, long fromLow, long fromHigh, long toLow, long toHigh);
/* random() declared in C++ section below (overloads conflict with newlib) */
#ifndef __cplusplus
long arduino_random(long howbig);
#endif

/* ---- Internal init (called before setup) ---- */
void init(void);

/* ---- User sketch entry points (defined by user) ---- */
void setup(void);
void loop(void);

/* ---- Yield (for cooperative multitasking hook) ---- */
void yield(void);

#ifdef __cplusplus
} /* extern "C" */

/* ---- C++ headers ---- */
#include "Print.h"
#include "Stream.h"
#include "HardwareSerial.h"

/* min/max as templates to avoid macro pitfalls in C++ */
#undef min
#undef max
template<class T> inline T min(T a, T b) { return a < b ? a : b; }
template<class T> inline T max(T a, T b) { return a > b ? a : b; }

/* random() C++ overloads (avoids conflict with newlib's random(void)) */
long random(long howbig);
long random(long howsmall, long howbig);

#endif /* __cplusplus */

#endif /* Arduino_h */
