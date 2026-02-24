/**
 * @file variant.h
 * @brief Board variant for LuatOS Air105 Dev Board
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Defines the pin mapping, LED, Serial and SPI/I2C defaults.
 * The Air105 has 6 GPIO ports (A-F) × 16 pins = 96 possible GPIOs.
 * Not all are exposed on the dev board.
 *
 * Pin numbering: pin = port * 16 + bit
 *   PA0=0, PA1=1, … PA15=15, PB0=16, … PF15=95
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef _VARIANT_AIR105_DEVBOARD_
#define _VARIANT_AIR105_DEVBOARD_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Total digital pins in the pin map ---- */
#define NUM_DIGITAL_PINS  96

/* ---- Analog pins (ADC channels) ---- */
#define NUM_ANALOG_INPUTS 8

/* ---- Built-in LEDs (LuatOS Air105 Core Board) ---- */
/*
 * LED pins on common Air105 dev boards:
 *   PD14 = Green LED
 *   PD15 = Blue LED
 *   PC3  = Red LED (active low on some boards)
 */
#define LED_BUILTIN         (3 * 16 + 14)   /* PD14 = pin 62, default LED (green) */
#define LED_BUILTIN_GREEN   LED_BUILTIN     /* PD14 = pin 62 */
#define LED_BUILTIN_BLUE    (3 * 16 + 15)   /* PD15 = pin 63 */
#define LED_BUILTIN_RED     (2 * 16 + 3)    /* PC3  = pin 35 */

/* Legacy aliases */
#define LED_GREEN           LED_BUILTIN_GREEN
#define LED_BLUE            LED_BUILTIN_BLUE
#define LED_RED             LED_BUILTIN_RED

/* ---- Serial pins (UART1 = default "Serial") ---- */
#define PIN_SERIAL_TX     (4 * 16 + 6)     /* PE6 */
#define PIN_SERIAL_RX     (4 * 16 + 7)     /* PE7 */

/* ---- SPI pins (SPI0 = default) ---- */
#define PIN_SPI_SCK       (1 * 16 + 2)     /* PB2 */
#define PIN_SPI_MISO      (1 * 16 + 4)     /* PB4 */
#define PIN_SPI_MOSI      (1 * 16 + 3)     /* PB3 */
#define PIN_SPI_SS        (1 * 16 + 5)     /* PB5 */
#define SS                PIN_SPI_SS
#define MOSI              PIN_SPI_MOSI
#define MISO              PIN_SPI_MISO
#define SCK               PIN_SPI_SCK

/* ---- I2C pins (I2C0 = default) ---- */
#define PIN_WIRE_SDA      (1 * 16 + 6)     /* PB6 */
#define PIN_WIRE_SCL      (1 * 16 + 7)     /* PB7 */
#define SDA               PIN_WIRE_SDA
#define SCL               PIN_WIRE_SCL

/* ---- ADC pins ---- */
/*
 * Air105 ADC channel mapping:
 *   - Channel 0: VBAT (internal, use analogReadVBAT())
 *   - Channel 1: PC0 (A0)
 *   - Channel 2: PC1 (A1)
 *   - Channel 3: Not available
 *   - Channel 4: PC3 (A2)
 *   - Channel 5: PC4 (A3)
 *   - Channel 6: PC5 (A4)
 */
#define A0  (2 * 16 + 0)  /* PC0 — ADC channel 1 */
#define A1  (2 * 16 + 1)  /* PC1 — ADC channel 2 */
#define A2  (2 * 16 + 3)  /* PC3 — ADC channel 4 (channel 3 not available) */
#define A3  (2 * 16 + 4)  /* PC4 — ADC channel 5 */
#define A4  (2 * 16 + 5)  /* PC5 — ADC channel 6 */

/* ---- PWM pins ---- */
/*
 * Air105 PWM channel mapping (6 channels via TIMM0):
 *   - Channel 0: PB0 (PWM0)
 *   - Channel 1: PB1 (PWM1)
 *   - Channel 2: PA2 (PWM2)
 *   - Channel 3: PA3 (PWM3)
 *   - Channel 4: PC6 (PWM4)
 *   - Channel 5: PC7 (PWM5)
 */
#define PWM0  (1 * 16 + 0)  /* PB0 — PWM channel 0 */
#define PWM1  (1 * 16 + 1)  /* PB1 — PWM channel 1 */
#define PWM2  (0 * 16 + 2)  /* PA2 — PWM channel 2 */
#define PWM3  (0 * 16 + 3)  /* PA3 — PWM channel 3 */
#define PWM4  (2 * 16 + 6)  /* PC6 — PWM channel 4 */
#define PWM5  (2 * 16 + 7)  /* PC7 — PWM channel 5 */

/* ---- User button ---- */
#define USER_BUTTON       (0 * 16 + 10)    /* PA10 — adjust per board */

#ifdef __cplusplus
}
#endif

#endif /* _VARIANT_AIR105_DEVBOARD_ */
