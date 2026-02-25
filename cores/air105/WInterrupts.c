/**
 * @file WInterrupts.c
 * @brief Arduino external interrupt API — attachInterrupt / detachInterrupt
 *        STM32duino-compatible API
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Air105 has one EXTI IRQ per GPIO port (EXTI0=PortA … EXTI5=PortF).
 * Each port's interrupt-type register has 2 bits per pin:
 *   00 = high level, 01 = low level, 10 = rising edge, 11 = falling edge
 *
 * SPDX-License-Identifier: MIT
 */

#include "Arduino.h"

/* Callback table: one per possible GPIO pin (96 max) */
static void (*_gpio_isr[NUM_DIGITAL_PINS])(void);

/* Mode tracking for CHANGE mode (need to toggle edge direction) */
static uint8_t _gpio_mode[NUM_DIGITAL_PINS];

/* IRQn for EXTI port 0-5 */
static const IRQn_Type exti_irqn[6] = {
    EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn,
    EXTI3_IRQn, EXTI4_IRQn, EXTI5_IRQn
};

/**
 * Attach an interrupt to a GPIO pin (STM32duino-compatible)
 * @param pin GPIO pin number (0-95)
 * @param userFunc Callback function
 * @param mode RISING, FALLING, or CHANGE
 */
void attachInterrupt(uint32_t pin, void (*userFunc)(void), uint32_t mode)
{
    if (pin >= NUM_DIGITAL_PINS || userFunc == NULL) return;

    uint8_t port = pin >> 4;
    uint8_t bit  = pin & 0x0F;

    _gpio_isr[pin] = userFunc;
    _gpio_mode[pin] = (uint8_t)mode;

    /* Configure interrupt type: 2 bits per pin in INTP_TYPE_STA[port] 
     * Hardware values: 00=disabled, 01=rising edge, 10=falling edge, 11=both edges
     * STM32duino: RISING=4, FALLING=3, CHANGE=2
     */
    uint32_t type_val;
    switch (mode) {
        case RISING:  type_val = 0x01; break;  /* 01 = rising edge */
        case FALLING: type_val = 0x02; break;  /* 10 = falling edge */
        case CHANGE:  type_val = 0x03; break;  /* 11 = both edges */
        default:      type_val = 0x02; break;  /* Default to falling */
    }

    uint32_t shift = bit * 2;
    GPIO->INTP_TYPE_STA[port].INTP_TYPE =
        (GPIO->INTP_TYPE_STA[port].INTP_TYPE & ~(3UL << shift))
        | (type_val << shift);

    /* Clear any pending interrupt for this pin */
    GPIO->INTP_TYPE_STA[port].INTP_STA = (1UL << bit);

    /* Enable interrupt in WAKEx_EN register (required for EXTI to work) */
    uint32_t wake_sn = pin / 32;
    uint32_t wake_pos = 1UL << (pin % 32);
    switch (wake_sn) {
        case 0: GPIO->WAKE_P0_EN |= wake_pos; break;
        case 1: GPIO->WAKE_P1_EN |= wake_pos; break;
        case 2: GPIO->WAKE_P2_EN |= wake_pos; break;
    }

    /* Enable NVIC for this port's EXTI IRQ */
    NVIC_SetPriority(exti_irqn[port], 3);
    NVIC_EnableIRQ(exti_irqn[port]);
}

/**
 * Detach interrupt from a GPIO pin
 * @param pin GPIO pin number (0-95)
 */
void detachInterrupt(uint32_t pin)
{
    if (pin >= NUM_DIGITAL_PINS) return;

    uint8_t port = pin >> 4;
    uint8_t bit  = pin & 0x0F;

    /* Disable this pin's interrupt type bits → 0 */
    uint32_t shift = bit * 2;
    GPIO->INTP_TYPE_STA[port].INTP_TYPE &= ~(3UL << shift);

    /* Disable interrupt in WAKEx_EN register */
    uint32_t wake_sn = pin / 32;
    uint32_t wake_pos = 1UL << (pin % 32);
    switch (wake_sn) {
        case 0: GPIO->WAKE_P0_EN &= ~wake_pos; break;
        case 1: GPIO->WAKE_P1_EN &= ~wake_pos; break;
        case 2: GPIO->WAKE_P2_EN &= ~wake_pos; break;
    }

    _gpio_isr[pin] = NULL;
    _gpio_mode[pin] = 0;
}

/* ------------------------------------------------------------------ */
/* EXTI IRQ handlers — one per port                                    */
/* ------------------------------------------------------------------ */
static void exti_handler(uint8_t port)
{
    uint32_t status = GPIO->INTP_TYPE_STA[port].INTP_STA;
    /* Clear all pending bits */
    GPIO->INTP_TYPE_STA[port].INTP_STA = status;

    uint8_t base = port << 4;
    for (uint8_t bit = 0; bit < 16 && status; bit++) {
        if (status & (1UL << bit)) {
            uint8_t pin = base + bit;
            if (pin < NUM_DIGITAL_PINS && _gpio_isr[pin]) {
                /* CHANGE mode uses hardware both-edges (0x03), no toggle needed */
                _gpio_isr[pin]();
            }
            status &= ~(1UL << bit);
        }
    }
}

void EXTI0_IRQHandler(void) { exti_handler(0); }
void EXTI1_IRQHandler(void) { exti_handler(1); }
void EXTI2_IRQHandler(void) { exti_handler(2); }
void EXTI3_IRQHandler(void) { exti_handler(3); }
void EXTI4_IRQHandler(void) { exti_handler(4); }
void EXTI5_IRQHandler(void) { exti_handler(5); }
