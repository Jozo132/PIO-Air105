/**
 * @file system_air105.h
 * @brief System-level declarations for Air105 Arduino framework
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 * SPDX-License-Identifier: MIT
 */
#ifndef SYSTEM_AIR105_H
#define SYSTEM_AIR105_H

#include <stdint.h>
#include "clock_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Runtime PLL/core rate, read from FREQ_SEL by SystemCoreClockUpdate().
 * At the startup configuration: CPU/PLL=204 MHz, HCLK=102 MHz, PCLK=51 MHz.
 * This runtime variable is distinct from the compile-time F_CPU macro.
 */
extern uint32_t SystemCoreClock;

void SystemInit(void);
void SystemCoreClockUpdate(void);
void SysTick_Init(void);

/* Millisecond counter incremented by SysTick_Handler */
extern volatile uint32_t _millis_count;

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_AIR105_H */
