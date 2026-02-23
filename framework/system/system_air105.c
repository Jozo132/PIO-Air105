/**
 * @file system_air105.c
 * @brief System initialization for Air105 MCU — Arduino framework
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Provides SystemInit(), SystemCoreClockUpdate(), and the SysTick handler
 * used by the Arduino timing functions (millis/micros).
 *
 * Clock model (matches vendor luatos-soc-air105 convention):
 *   SystemCoreClock = PLL output frequency (NOT HCLK).
 *   HCLK = PLL / 2   (HCLK_DIV_1_2)
 *   PCLK = HCLK / 2  (PCLK_DIV_1_2, default)
 *   SysTick runs at HCLK = SystemCoreClock / 2.
 *
 * All vendor-compatible peripheral divisors (UART, SPI, I2C, Timer)
 * use SystemCoreClock with explicit right-shifts to derive the actual
 * bus clock, so keeping SystemCoreClock = PLL is intentional.
 *
 * SPDX-License-Identifier: MIT
 */

#include "air105.h"
#include <stdint.h>

/* ---- Constants ---- */
#ifndef HSE_VALUE
#define HSE_VALUE  12000000UL   /* 12 MHz internal oscillator */
#endif

/* Exported variables */
uint32_t SystemCoreClock;

/* Symbols from linker script */
extern const uint32_t __isr_start_address;

/**
 * @brief Update SystemCoreClock from hardware FREQ_SEL register
 *
 * SystemCoreClock = PLL frequency = HSE_VALUE * (XTAL + 1)
 */
void SystemCoreClockUpdate(void)
{
    uint32_t xtal_val = (SYSCTRL->FREQ_SEL & SYSCTRL_FREQ_SEL_XTAL_Mask)
                        >> SYSCTRL_FREQ_SEL_XTAL_Pos;
    SystemCoreClock = HSE_VALUE * (xtal_val + 1);
}

/**
 * @brief System initialization — called from Reset_Handler before main()
 *
 * Configures:
 *  - Vector table relocation (VTOR)
 *  - PLL to 204 MHz (max rated), HCLK = PLL/2, PCLK = HCLK/2
 *  - FPU enable (Cortex-M4F)
 *  - All APB and needed AHB peripheral clocks
 */
void SystemInit(void)
{
    /* Relocate vector table */
    SCB->VTOR = (uint32_t)(&__isr_start_address);

    /*
     * Configure PLL:
     *  - 0x20000000 : required undocumented bit (from vendor reference)
     *  - XTAL_204Mhz: PLL = 12 * 17 = 204 MHz
     *  - CLOCK_SOURCE_INC: use internal 12 MHz oscillator
     *  - HCLK_DIV_1_2: HCLK = PLL / 2 = 102 MHz
     *  - PCLK_DIV_1_2: PCLK = HCLK / 2 = 51 MHz (bit 0 = 0)
     */
    SYSCTRL->FREQ_SEL = 0x20000000
                       | SYSCTRL_FREQ_SEL_XTAL_204Mhz
                       | SYSCTRL_FREQ_SEL_CLOCK_SOURCE_INC
                       | SYSCTRL_FREQ_SEL_HCLK_DIV_1_2;

    /* Enable FPU (CP10 + CP11 full access) */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));
#endif

    /* Enable all APB peripheral clocks */
    SYSCTRL->CG_CTRL1 = SYSCTRL_APBPeriph_ALL;

    /* Enable DMA + USB on AHB */
    SYSCTRL->CG_CTRL2 = SYSCTRL_AHBPeriph_DMA | SYSCTRL_AHBPeriph_USB;

    /* Update SystemCoreClock variable from FREQ_SEL register */
    SystemCoreClockUpdate();

    /* Enable interrupts */
    __enable_irq();
}

/* ------------------------------------------------------------------ */
/* SysTick — 1 ms tick for Arduino millis()/micros()                  */
/* ------------------------------------------------------------------ */
volatile uint32_t _millis_count = 0;

/**
 * @brief SysTick interrupt handler — increments millisecond counter
 */
void SysTick_Handler(void)
{
    _millis_count++;
}

/**
 * @brief Configure SysTick for 1 ms interrupts
 *
 * SysTick runs at HCLK = SystemCoreClock / 2.
 * Called from init() in the Arduino core before setup().
 */
void SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 2 / 1000);
}
