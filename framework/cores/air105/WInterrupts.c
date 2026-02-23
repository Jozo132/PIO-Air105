/**
 * @file WInterrupts.c
 * @brief Arduino external interrupt API — attachInterrupt / detachInterrupt
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

/* IRQn for EXTI port 0-5 */
static const IRQn_Type exti_irqn[6] = {
    EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn,
    EXTI3_IRQn, EXTI4_IRQn, EXTI5_IRQn
};

void attachInterrupt(uint8_t pin, void (*userFunc)(void), int mode)
{
    if (pin >= NUM_DIGITAL_PINS || userFunc == NULL) return;

    uint8_t port = pin >> 4;
    uint8_t bit  = pin & 0x0F;

    _gpio_isr[pin] = userFunc;

    /* Configure interrupt type: 2 bits per pin in INTP_TYPE_STA[port] */
    uint32_t type_val;
    switch (mode) {
        case RISING:  type_val = 0x02; break;
        case FALLING: type_val = 0x03; break;
        case CHANGE:  type_val = 0x02; break; /* rising; TODO: emulate CHANGE */
        default:      type_val = 0x03; break;
    }

    uint32_t shift = bit * 2;
    GPIO_MODULE_TypeDef *gpio_mod = (GPIO_MODULE_TypeDef *)GPIO_BASE;
    gpio_mod->INTP_TYPE_STA[port].INTP_TYPE =
        (gpio_mod->INTP_TYPE_STA[port].INTP_TYPE & ~(3UL << shift))
        | (type_val << shift);

    /* Clear any pending interrupt for this pin */
    gpio_mod->INTP_TYPE_STA[port].INTP_STA = (1UL << bit);

    /* Enable NVIC for this port's EXTI IRQ */
    NVIC_SetPriority(exti_irqn[port], 3);
    NVIC_EnableIRQ(exti_irqn[port]);
}

void detachInterrupt(uint8_t pin)
{
    if (pin >= NUM_DIGITAL_PINS) return;

    uint8_t port = pin >> 4;
    uint8_t bit  = pin & 0x0F;

    /* Disable this pin's interrupt type bits → 0 */
    uint32_t shift = bit * 2;
    GPIO_MODULE_TypeDef *gpio_mod = (GPIO_MODULE_TypeDef *)GPIO_BASE;
    gpio_mod->INTP_TYPE_STA[port].INTP_TYPE &= ~(3UL << shift);

    _gpio_isr[pin] = NULL;
}

/* ------------------------------------------------------------------ */
/* EXTI IRQ handlers — one per port                                    */
/* ------------------------------------------------------------------ */
static void exti_handler(uint8_t port)
{
    GPIO_MODULE_TypeDef *gpio_mod = (GPIO_MODULE_TypeDef *)GPIO_BASE;
    uint32_t status = gpio_mod->INTP_TYPE_STA[port].INTP_STA;
    /* Clear all pending bits */
    gpio_mod->INTP_TYPE_STA[port].INTP_STA = status;

    uint8_t base = port << 4;
    for (uint8_t bit = 0; bit < 16 && status; bit++) {
        if (status & (1UL << bit)) {
            uint8_t pin = base + bit;
            if (pin < NUM_DIGITAL_PINS && _gpio_isr[pin]) {
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
