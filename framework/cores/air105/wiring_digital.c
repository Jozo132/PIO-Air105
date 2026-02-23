/**
 * @file wiring_digital.c
 * @brief Arduino digital I/O — pinMode, digitalWrite, digitalRead
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Air105 GPIO register model:
 *   OEN  : 1 = input (hi-Z), 0 = output  (inverted vs typical ARM)
 *   IODR : bits [15:0] = output latch, bits [31:16] = input state
 *   BSRR : bits [15:0] = set,          bits [31:16] = reset (atomic)
 *   PUE  : 1 = pull-up enabled
 *   ALT  : 2 bits per pin — function 1 = GPIO mode
 *
 * Pin number encoding (from variant.h):
 *   pin = port * 16 + bit   (PA0=0, PA1=1, … PB0=16, … PF15=95)
 *
 * SPDX-License-Identifier: MIT
 */

#include "Arduino.h"

/* Resolve pin number to port pointer and bit mask.
 * NOTE: Cannot use GPIO->GPIO[n] because the vendor macro GPIO expands
 * via the preprocessor, colliding with the struct member name.
 * Use GPIO_GROUP (= ((GPIO_TypeDef*)GPIO_BASE)) which indexes correctly.
 */
static inline GPIO_TypeDef* pinToPort(uint8_t pin) {
    return &GPIO_GROUP[pin >> 4];
}
static inline uint16_t pinToBit(uint8_t pin) {
    return (uint16_t)(1U << (pin & 0x0F));
}

/**
 * @brief Set alternate-function for this pin to GPIO mode (AF 1)
 */
static void pinSetGpioMode(uint8_t pin) {
    uint8_t port = pin >> 4;
    uint8_t bit  = pin & 0x0F;
    uint32_t shift = bit * 2;
    GPIO_ALT_GROUP[port] = (GPIO_ALT_GROUP[port] & ~(3UL << shift)) | (1UL << shift);
}

void pinMode(uint8_t pin, uint8_t mode)
{
    if (pin >= NUM_DIGITAL_PINS) return;

    GPIO_TypeDef *port = pinToPort(pin);
    uint16_t mask = pinToBit(pin);

    /* Set pin to GPIO function */
    pinSetGpioMode(pin);

    switch (mode) {
        case OUTPUT:
            port->PUE &= ~mask;    /* disable pull-up */
            port->OEN &= ~mask;    /* OEN=0 → output  */
            break;

        case INPUT_PULLUP:
            port->OEN |=  mask;    /* OEN=1 → input */
            port->PUE |=  mask;    /* enable pull-up */
            break;

        case INPUT:
        default:
            port->OEN |=  mask;    /* OEN=1 → input */
            port->PUE &= ~mask;   /* disable pull-up */
            break;
    }
}

void digitalWrite(uint8_t pin, uint8_t val)
{
    if (pin >= NUM_DIGITAL_PINS) return;

    GPIO_TypeDef *port = pinToPort(pin);
    uint16_t mask = pinToBit(pin);

    if (val) {
        port->BSRR = mask;                  /* set (bits [15:0]) */
    } else {
        port->BSRR = (uint32_t)mask << 16;  /* reset (bits [31:16]) */
    }
}

int digitalRead(uint8_t pin)
{
    if (pin >= NUM_DIGITAL_PINS) return LOW;

    GPIO_TypeDef *port = pinToPort(pin);
    uint16_t mask = pinToBit(pin);

    /* Input state is in upper 16 bits of IODR */
    return (port->IODR >> 16) & mask ? HIGH : LOW;
}
