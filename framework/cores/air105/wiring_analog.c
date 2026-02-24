/**
 * @file wiring_analog.c
 * @brief Arduino analog I/O implementation — analogRead, analogWrite, analogReference
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Air105 ADC: 7 channels (0=VBAT, 1-6 external), 12-bit resolution, 0-1.8V range
 * ADC channel to GPIO mapping:
 *   - Channel 0: VBAT (internal, no GPIO)
 *   - Channel 1: PC0 (pin 32), AF2
 *   - Channel 2: PC1 (pin 33), AF2
 *   - Channel 3: Not available
 *   - Channel 4: PC3 (pin 35), AF2
 *   - Channel 5: PC4 (pin 36), AF2
 *   - Channel 6: PC5 (pin 37), AF2
 *
 * SPDX-License-Identifier: MIT
 */

#include "Arduino.h"

/* ---- ADC Register Definitions ---- */
#define ADC_BASE_ADDR       0x40014000UL
#define GPIO_BASE_ADDR      0x4001D000UL
#define GPIO_ALT_OFFSET     0x180

/* ADC register structure */
typedef struct {
    volatile uint32_t CR1;      /* Control Register 1 */
    volatile uint32_t SR;       /* Status Register */
    volatile uint32_t FIFO;     /* FIFO Control */
    volatile uint32_t DATA;     /* Data Register */
    volatile uint32_t FIFO_FL;  /* FIFO Fill Level */
    volatile uint32_t FIFO_THR; /* FIFO Threshold */
    volatile uint32_t CR2;      /* Control Register 2 */
} ADC_Regs;

#define ADC ((ADC_Regs *)ADC_BASE_ADDR)

/* ADC control bits */
#define ADC_CR1_SAMP_ENABLE     (1UL << 6)   /* Enable sampling */
#define ADC_SR_DONE             (1UL << 0)   /* Conversion done */

/* GPIO alternate function register pointer */
#define GPIO_ALT  ((volatile uint32_t *)(GPIO_BASE_ADDR + GPIO_ALT_OFFSET))

/* ADC internal state */
static uint8_t _adc_initialized = 0;
static uint8_t _adc_use_divider = 0;  /* 0 = 0-1.8V, 1 = 0-3.6V with internal divider */

/**
 * @brief Convert Arduino pin number to ADC channel
 * @param pin Arduino pin number (e.g., A0=32, A1=33, etc.)
 * @return ADC channel (1-6), or 0xFF if not an ADC pin
 */
static uint8_t pinToAdcChannel(uint8_t pin)
{
    /* Port C pins with ADC function (port 2, bits 0-5) */
    if ((pin >> 4) != 2) return 0xFF;  /* Must be port C */
    
    uint8_t bit = pin & 0x0F;
    switch (bit) {
        case 0: return 1;    /* PC0 -> ADC channel 1 */
        case 1: return 2;    /* PC1 -> ADC channel 2 */
        case 3: return 4;    /* PC3 -> ADC channel 4 (channel 3 not available) */
        case 4: return 5;    /* PC4 -> ADC channel 5 */
        case 5: return 6;    /* PC5 -> ADC channel 6 */
        default: return 0xFF; /* Not an ADC pin */
    }
}

/**
 * @brief Initialize ADC peripheral
 */
static void adcInit(void)
{
    if (_adc_initialized) return;
    
    /* Initialize ADC:
     * - Set FIFO threshold
     * - Clear FIFO
     * - Disable internal resistance divider (use 0-1.8V range by default)
     */
    ADC->FIFO_THR = 13;           /* FIFO threshold = 14 samples - 1 */
    ADC->FIFO = 3;                /* Clear FIFO */
    ADC->CR2 &= ~(1UL << 14);     /* Disable CR2 bit 14 */
    ADC->CR2 &= ~(1UL << 13);     /* Disable internal resistance divider */
    ADC->CR1 = 0;                 /* Disable ADC */
    
    _adc_initialized = 1;
}

/**
 * @brief Configure GPIO pin for ADC function (alternate function 2)
 * @param pin Arduino pin number
 */
static void configureAdcPin(uint8_t pin)
{
    uint8_t port = pin >> 4;
    uint8_t bit = pin & 0x0F;
    
    /* Set alternate function 2 for ADC */
    uint32_t mask = ~(0x03UL << (bit * 2));
    uint32_t func = (0x02UL << (bit * 2));  /* AF2 for ADC */
    GPIO_ALT[port] = (GPIO_ALT[port] & mask) | func;
}

void analogReference(uint8_t type)
{
    adcInit();
    
    /* Air105 supports two ADC ranges via internal resistance divider:
     * - DEFAULT: 0-1.8V (divider off)
     * - INTERNAL: 0-3.6V (divider on) - note: channel 6 has different scaling
     */
    if (type == INTERNAL) {
        ADC->CR2 |= (1UL << 13);  /* Enable internal resistance divider */
        _adc_use_divider = 1;
    } else {
        ADC->CR2 &= ~(1UL << 13); /* Disable divider, use 0-1.8V */
        _adc_use_divider = 0;
    }
}

int analogRead(uint8_t pin)
{
    /* Convert pin to ADC channel */
    uint8_t channel = pinToAdcChannel(pin);
    if (channel == 0xFF) return 0;  /* Not an ADC pin */
    
    /* Initialize ADC if needed */
    adcInit();
    
    /* Configure GPIO for ADC function */
    configureAdcPin(pin);
    
    /* Clear FIFO and wait for it to be ready */
    ADC->FIFO = 3;
    while (ADC->FIFO & (1UL << 1)) { /* Wait for FIFO not busy */ }
    
    /* Start single conversion: enable sampling + channel select */
    ADC->CR1 = ADC_CR1_SAMP_ENABLE | channel;
    
    /* Wait for conversion complete */
    while (!(ADC->SR & ADC_SR_DONE)) { /* Spin */ }
    
    /* Stop sampling */
    ADC->CR1 = 0;
    
    /* Read result (12-bit value) */
    uint32_t result = ADC->DATA & 0x0FFF;
    
    /* Clear FIFO */
    ADC->FIFO = 3;
    
    return (int)result;
}

/**
 * @brief Read battery voltage via ADC channel 0 (VBAT)
 * @return Battery voltage in millivolts
 */
int analogReadVBAT(void)
{
    adcInit();
    
    /* Clear FIFO */
    ADC->FIFO = 3;
    while (ADC->FIFO & (1UL << 1)) { }
    
    /* Read channel 0 (VBAT) */
    ADC->CR1 = ADC_CR1_SAMP_ENABLE | 0;
    while (!(ADC->SR & ADC_SR_DONE)) { }
    ADC->CR1 = 0;
    
    uint32_t raw = ADC->DATA & 0x0FFF;
    ADC->FIFO = 3;
    
    /* VBAT scaling: value * 1880 * 14 / 5 >> 12 = mV */
    uint32_t mv = (raw * 1880UL * 14UL / 5UL) >> 12;
    return (int)mv;
}

void analogWrite(uint8_t pin, int val)
{
    (void)pin;
    (void)val;
    /* TODO: Implement PWM output via Air105 timer peripheral */
}

/* ---- random / map ---- */
static unsigned long _random_seed = 1;

void randomSeed(unsigned long seed)
{
    if (seed != 0) _random_seed = seed;
}

long arduino_random(long howbig)
{
    if (howbig == 0) return 0;
    /* Simple LCG — replace with TRNG if available */
    _random_seed = _random_seed * 1103515245UL + 12345UL;
    return (long)((_random_seed >> 16) % (unsigned long)howbig);
}

long map(long value, long fromLow, long fromHigh, long toLow, long toHigh)
{
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}
