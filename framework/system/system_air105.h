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

#ifdef __cplusplus
extern "C" {
#endif

/* SystemCoreClock = PLL frequency (e.g. 204 MHz).
 * HCLK = SystemCoreClock / 2,  PCLK = HCLK / 2.
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
