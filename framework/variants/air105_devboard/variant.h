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
 *   PD14 = Blue LED
 *   PD15 = Green LED
 *   PC3  = Red LED (active low on some boards)
 */
#define LED_BUILTIN         (2 * 16 + 3)    /* PC3  = pin 35, default LED (red) */
#define LED_BUILTIN_RED     LED_BUILTIN     /* PC3  = pin 35 */
#define LED_BUILTIN_GREEN   (3 * 16 + 15)   /* PD15 = pin 63 */
#define LED_BUILTIN_BLUE    (3 * 16 + 14)   /* PD14 = pin 62 */

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
/* NOTE: I2C0 shares PE6/PE7 with UART1 (Serial).
 *       You cannot use Wire and Serial simultaneously.
 *       Use Serial0, Serial2, or Serial3 instead when using I2C. */
#define PIN_WIRE_SDA      (4 * 16 + 6)     /* PE6 (AF2) */
#define PIN_WIRE_SCL      (4 * 16 + 7)     /* PE7 (AF2) */
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
/* Note: CORE-Air105-V1.0 has no user button, only BOOT/RESET buttons */
/* USER_BUTTON definition is for boards that have one - adjust per board */
// #define USER_BUTTON       (0 * 16 + 10)    /* PA10 — uncomment and adjust per board */

/* ---- STM32duino-compatible pin names ---- */
/* Port A: PA0-PA15 */
#define PA0   (0 * 16 + 0)
#define PA1   (0 * 16 + 1)
#define PA2   (0 * 16 + 2)
#define PA3   (0 * 16 + 3)
#define PA4   (0 * 16 + 4)
#define PA5   (0 * 16 + 5)
#define PA6   (0 * 16 + 6)
#define PA7   (0 * 16 + 7)
#define PA8   (0 * 16 + 8)
#define PA9   (0 * 16 + 9)
#define PA10  (0 * 16 + 10)
#define PA11  (0 * 16 + 11)
#define PA12  (0 * 16 + 12)
#define PA13  (0 * 16 + 13)
#define PA14  (0 * 16 + 14)
#define PA15  (0 * 16 + 15)

/* Port B: PB0-PB15 */
#define PB0   (1 * 16 + 0)
#define PB1   (1 * 16 + 1)
#define PB2   (1 * 16 + 2)
#define PB3   (1 * 16 + 3)
#define PB4   (1 * 16 + 4)
#define PB5   (1 * 16 + 5)
#define PB6   (1 * 16 + 6)
#define PB7   (1 * 16 + 7)
#define PB8   (1 * 16 + 8)
#define PB9   (1 * 16 + 9)
#define PB10  (1 * 16 + 10)
#define PB11  (1 * 16 + 11)
#define PB12  (1 * 16 + 12)
#define PB13  (1 * 16 + 13)
#define PB14  (1 * 16 + 14)
#define PB15  (1 * 16 + 15)

/* Port C: PC0-PC15 */
#define PC0   (2 * 16 + 0)
#define PC1   (2 * 16 + 1)
#define PC2   (2 * 16 + 2)
#define PC3   (2 * 16 + 3)
#define PC4   (2 * 16 + 4)
#define PC5   (2 * 16 + 5)
#define PC6   (2 * 16 + 6)
#define PC7   (2 * 16 + 7)
#define PC8   (2 * 16 + 8)
#define PC9   (2 * 16 + 9)
#define PC10  (2 * 16 + 10)
#define PC11  (2 * 16 + 11)
#define PC12  (2 * 16 + 12)
#define PC13  (2 * 16 + 13)
#define PC14  (2 * 16 + 14)
#define PC15  (2 * 16 + 15)

/* Port D: PD0-PD15 */
#define PD0   (3 * 16 + 0)
#define PD1   (3 * 16 + 1)
#define PD2   (3 * 16 + 2)
#define PD3   (3 * 16 + 3)
#define PD4   (3 * 16 + 4)
#define PD5   (3 * 16 + 5)
#define PD6   (3 * 16 + 6)
#define PD7   (3 * 16 + 7)
#define PD8   (3 * 16 + 8)
#define PD9   (3 * 16 + 9)
#define PD10  (3 * 16 + 10)
#define PD11  (3 * 16 + 11)
#define PD12  (3 * 16 + 12)
#define PD13  (3 * 16 + 13)
#define PD14  (3 * 16 + 14)
#define PD15  (3 * 16 + 15)

/* Port E: PE0-PE15 */
#define PE0   (4 * 16 + 0)
#define PE1   (4 * 16 + 1)
#define PE2   (4 * 16 + 2)
#define PE3   (4 * 16 + 3)
#define PE4   (4 * 16 + 4)
#define PE5   (4 * 16 + 5)
#define PE6   (4 * 16 + 6)
#define PE7   (4 * 16 + 7)
#define PE8   (4 * 16 + 8)
#define PE9   (4 * 16 + 9)
#define PE10  (4 * 16 + 10)
#define PE11  (4 * 16 + 11)
#define PE12  (4 * 16 + 12)
#define PE13  (4 * 16 + 13)
#define PE14  (4 * 16 + 14)
#define PE15  (4 * 16 + 15)

/* Port F: PF0-PF15 */
#define PF0   (5 * 16 + 0)
#define PF1   (5 * 16 + 1)
#define PF2   (5 * 16 + 2)
#define PF3   (5 * 16 + 3)
#define PF4   (5 * 16 + 4)
#define PF5   (5 * 16 + 5)
#define PF6   (5 * 16 + 6)
#define PF7   (5 * 16 + 7)
#define PF8   (5 * 16 + 8)
#define PF9   (5 * 16 + 9)
#define PF10  (5 * 16 + 10)
#define PF11  (5 * 16 + 11)
#define PF12  (5 * 16 + 12)
#define PF13  (5 * 16 + 13)
#define PF14  (5 * 16 + 14)
#define PF15  (5 * 16 + 15)

#ifdef __cplusplus
}
#endif

#endif /* _VARIANT_AIR105_DEVBOARD_ */
