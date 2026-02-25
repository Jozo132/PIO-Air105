/**
 * @file air105_conf.h
 * @brief Minimal Air105 configuration header for PIO-Air105 Arduino framework
 *
 * The original vendor air105_conf.h pulls in all vendor HAL peripheral driver
 * headers (air105_gpio.h, air105_uart.h, etc.).  The PIO-Air105 Arduino
 * framework does NOT use the vendor HAL — all peripheral access is through
 * direct register manipulation via the structs defined in air105.h.
 *
 * This empty override prevents the vendor HAL headers from being required
 * on the include path while keeping air105.h happy (it #includes this file
 * unconditionally at the end).
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __AIR105_CONF_H
#define __AIR105_CONF_H

/* Nothing needed — PIO-Air105 uses direct register access */

#endif /* __AIR105_CONF_H */
