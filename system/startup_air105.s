/**
 * @file startup_air105.s
 * @brief Startup assembly for Air105 MCU — Arduino framework
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Vector table, Reset_Handler (.data copy, .bss zero, SystemInit, main).
 * All external IRQs route through individual weak symbols (not a single
 * global handler) so Arduino users can override them directly.
 *
 * Based on Armv7-M / Cortex-M4 exception model.
 * Written from scratch for PIO-Air105. SPDX-License-Identifier: MIT
 */

    .syntax unified
    .cpu    cortex-m4
    .fpu    fpv4-sp-d16
    .thumb

/* Linker-provided symbols */
.global g_pfnVectors
.global Default_Handler

.word   _sidata         /* LMA of .data in flash      */
.word   _sdata          /* VMA start of .data in RAM   */
.word   _edata          /* VMA end of .data in RAM     */
.word   _sbss           /* start of .bss               */
.word   _ebss           /* end of .bss                 */

/* ================================================================== */
/*  Reset_Handler                                                     */
/* ================================================================== */
    .section .text.Reset_Handler
    .weak    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    /* Clear registers */
    movs r0, #0
    movs r1, #0
    movs r2, #0
    movs r3, #0
    movs r4, #0
    movs r5, #0
    movs r6, #0
    movs r7, #0

    /* Set VTOR to our vector table */
    ldr  r0, =g_pfnVectors
    ldr  r1, =0xE000ED08
    str  r0, [r1]

    /* Set stack pointer */
    ldr  sp, =_estack

    /* Copy .data from flash (LMA) to SRAM (VMA) */
    movs r1, #0
    b    LoopCopyDataInit

CopyDataInit:
    ldr  r3, =_sidata
    ldr  r3, [r3, r1]
    str  r3, [r0, r1]
    adds r1, r1, #4

LoopCopyDataInit:
    ldr  r0, =_sdata
    ldr  r3, =_edata
    adds r2, r0, r1
    cmp  r2, r3
    bcc  CopyDataInit

    /* Zero .bss */
    ldr  r2, =_sbss
    b    LoopFillZerobss

FillZerobss:
    movs r3, #0
    str  r3, [r2], #4

LoopFillZerobss:
    ldr  r3, =_ebss
    cmp  r2, r3
    bcc  FillZerobss

    /* Call SystemInit (clock, FPU, peripheral clocks) */
    bl   SystemInit

    /* Call C++ static constructors */
    bl   __libc_init_array

    /* Call main() — Arduino's main.cpp */
    bl   main

    /* If main returns, loop forever */
    b    .

    .size Reset_Handler, .-Reset_Handler

/* ================================================================== */
/*  Default_Handler — infinite loop for unhandled interrupts           */
/* ================================================================== */
    .section .text.Default_Handler, "ax", %progbits
Default_Handler:
Infinite_Loop:
    b    Infinite_Loop
    .size Default_Handler, .-Default_Handler

/* ================================================================== */
/*  Vector Table                                                      */
/* ================================================================== */
    .section .isr_vector, "a", %progbits
    .type    g_pfnVectors, %object
    .size    g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    .word _estack               /* 0:  Initial Stack Pointer        */
    .word Reset_Handler         /* 1:  Reset                        */
    .word NMI_Handler           /* 2:  NMI                          */
    .word HardFault_Handler     /* 3:  Hard Fault                   */
    .word MemManage_Handler     /* 4:  Memory Management Fault      */
    .word BusFault_Handler      /* 5:  Bus Fault                    */
    .word UsageFault_Handler    /* 6:  Usage Fault                  */
    .word 0                     /* 7:  Reserved                     */
    .word 0                     /* 8:  Reserved                     */
    .word 0                     /* 9:  Reserved                     */
    .word 0                     /* 10: Reserved                     */
    .word SVC_Handler           /* 11: SVCall                       */
    .word DebugMon_Handler      /* 12: Debug Monitor                */
    .word 0                     /* 13: Reserved                     */
    .word PendSV_Handler        /* 14: PendSV                       */
    .word SysTick_Handler       /* 15: SysTick                      */

    /* --- External Interrupts (IRQn 0–51) --- */
    .word DMA_IRQHandler          /*  0: DMA                        */
    .word USB_IRQHandler          /*  1: USB                        */
    .word USBDMA_IRQHandler       /*  2: USBDMA                    */
    .word LCD_IRQHandler          /*  3: LCD                        */
    .word SCI0_IRQHandler         /*  4: SCI0                      */
    .word UART0_IRQHandler        /*  5: UART0                     */
    .word UART1_IRQHandler        /*  6: UART1                     */
    .word SPI0_IRQHandler         /*  7: SPI0                      */
    .word CRYPT0_IRQHandler       /*  8: CRYPT0                    */
    .word TIM0_0_IRQHandler       /*  9: Timer 0                   */
    .word TIM0_1_IRQHandler       /* 10: Timer 1                   */
    .word TIM0_2_IRQHandler       /* 11: Timer 2                   */
    .word TIM0_3_IRQHandler       /* 12: Timer 3                   */
    .word EXTI0_IRQHandler        /* 13: GPIO Port A               */
    .word EXTI1_IRQHandler        /* 14: GPIO Port B               */
    .word EXTI2_IRQHandler        /* 15: GPIO Port C               */
    .word RTC_IRQHandler          /* 16: RTC                       */
    .word SENSOR_IRQHandler       /* 17: Sensor                    */
    .word TRNG_IRQHandler         /* 18: TRNG                      */
    .word ADC0_IRQHandler         /* 19: ADC                       */
    .word SSC_IRQHandler          /* 20: SSC                       */
    .word TIM0_4_IRQHandler       /* 21: Timer 4                   */
    .word TIM0_5_IRQHandler       /* 22: Timer 5                   */
    .word KBD_IRQHandler          /* 23: Keyboard                  */
    .word MSR_IRQHandler          /* 24: MSR                       */
    .word EXTI3_IRQHandler        /* 25: GPIO Port D               */
    .word SPI1_IRQHandler         /* 26: SPI1                      */
    .word SPI2_IRQHandler         /* 27: SPI2                      */
    .word 0                       /* 28: Reserved                  */
    .word SCI2_IRQHandler         /* 29: SCI2                      */
    .word 0                       /* 30: Reserved                  */
    .word 0                       /* 31: Reserved                  */
    .word UART2_IRQHandler        /* 32: UART2                     */
    .word UART3_IRQHandler        /* 33: UART3                     */
    .word 0                       /* 34: Reserved                  */
    .word QSPI_IRQHandler         /* 35: QSPI                     */
    .word I2C0_IRQHandler         /* 36: I2C0                      */
    .word EXTI4_IRQHandler        /* 37: GPIO Port E               */
    .word EXTI5_IRQHandler        /* 38: GPIO Port F               */
    .word TIM0_6_IRQHandler       /* 39: Timer 6                   */
    .word TIM0_7_IRQHandler       /* 40: Timer 7                   */
    .word 0                       /* 41: Reserved                  */
    .word DCMI_IRQHandler         /* 42: DCMI                      */
    .word 0                       /* 43: Reserved                  */
    .word 0                       /* 44: Reserved                  */
    .word 0                       /* 45: Reserved                  */
    .word QR_IRQHandler           /* 46: QR                        */
    .word GPU_IRQHandler          /* 47: GPU                       */
    .word 0                       /* 48: Reserved                  */
    .word AWD_IRQHandler          /* 49: Analog Watchdog           */
    .word DAC_IRQHandler          /* 50: DAC                       */
    .word SPI5_IRQHandler         /* 51: SPI5                      */

/* ================================================================== */
/*  Weak aliases — all point to Default_Handler unless overridden     */
/* ================================================================== */

    .weak      NMI_Handler
    .thumb_set NMI_Handler, Default_Handler
    .weak      HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler
    .weak      MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler
    .weak      BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler
    .weak      UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler
    .weak      SVC_Handler
    .thumb_set SVC_Handler, Default_Handler
    .weak      DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler
    .weak      PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler

    /* SysTick_Handler is NOT weak — defined in system_air105.c */

    .weak      DMA_IRQHandler
    .thumb_set DMA_IRQHandler, Default_Handler
    .weak      USB_IRQHandler
    .thumb_set USB_IRQHandler, Default_Handler
    .weak      USBDMA_IRQHandler
    .thumb_set USBDMA_IRQHandler, Default_Handler
    .weak      LCD_IRQHandler
    .thumb_set LCD_IRQHandler, Default_Handler
    .weak      SCI0_IRQHandler
    .thumb_set SCI0_IRQHandler, Default_Handler
    .weak      UART0_IRQHandler
    .thumb_set UART0_IRQHandler, Default_Handler
    .weak      UART1_IRQHandler
    .thumb_set UART1_IRQHandler, Default_Handler
    .weak      SPI0_IRQHandler
    .thumb_set SPI0_IRQHandler, Default_Handler
    .weak      CRYPT0_IRQHandler
    .thumb_set CRYPT0_IRQHandler, Default_Handler
    .weak      TIM0_0_IRQHandler
    .thumb_set TIM0_0_IRQHandler, Default_Handler
    .weak      TIM0_1_IRQHandler
    .thumb_set TIM0_1_IRQHandler, Default_Handler
    .weak      TIM0_2_IRQHandler
    .thumb_set TIM0_2_IRQHandler, Default_Handler
    .weak      TIM0_3_IRQHandler
    .thumb_set TIM0_3_IRQHandler, Default_Handler
    .weak      EXTI0_IRQHandler
    .thumb_set EXTI0_IRQHandler, Default_Handler
    .weak      EXTI1_IRQHandler
    .thumb_set EXTI1_IRQHandler, Default_Handler
    .weak      EXTI2_IRQHandler
    .thumb_set EXTI2_IRQHandler, Default_Handler
    .weak      RTC_IRQHandler
    .thumb_set RTC_IRQHandler, Default_Handler
    .weak      SENSOR_IRQHandler
    .thumb_set SENSOR_IRQHandler, Default_Handler
    .weak      TRNG_IRQHandler
    .thumb_set TRNG_IRQHandler, Default_Handler
    .weak      ADC0_IRQHandler
    .thumb_set ADC0_IRQHandler, Default_Handler
    .weak      SSC_IRQHandler
    .thumb_set SSC_IRQHandler, Default_Handler
    .weak      TIM0_4_IRQHandler
    .thumb_set TIM0_4_IRQHandler, Default_Handler
    .weak      TIM0_5_IRQHandler
    .thumb_set TIM0_5_IRQHandler, Default_Handler
    .weak      KBD_IRQHandler
    .thumb_set KBD_IRQHandler, Default_Handler
    .weak      MSR_IRQHandler
    .thumb_set MSR_IRQHandler, Default_Handler
    .weak      EXTI3_IRQHandler
    .thumb_set EXTI3_IRQHandler, Default_Handler
    .weak      SPI1_IRQHandler
    .thumb_set SPI1_IRQHandler, Default_Handler
    .weak      SPI2_IRQHandler
    .thumb_set SPI2_IRQHandler, Default_Handler
    .weak      SCI2_IRQHandler
    .thumb_set SCI2_IRQHandler, Default_Handler
    .weak      UART2_IRQHandler
    .thumb_set UART2_IRQHandler, Default_Handler
    .weak      UART3_IRQHandler
    .thumb_set UART3_IRQHandler, Default_Handler
    .weak      QSPI_IRQHandler
    .thumb_set QSPI_IRQHandler, Default_Handler
    .weak      I2C0_IRQHandler
    .thumb_set I2C0_IRQHandler, Default_Handler
    .weak      EXTI4_IRQHandler
    .thumb_set EXTI4_IRQHandler, Default_Handler
    .weak      EXTI5_IRQHandler
    .thumb_set EXTI5_IRQHandler, Default_Handler
    .weak      TIM0_6_IRQHandler
    .thumb_set TIM0_6_IRQHandler, Default_Handler
    .weak      TIM0_7_IRQHandler
    .thumb_set TIM0_7_IRQHandler, Default_Handler
    .weak      DCMI_IRQHandler
    .thumb_set DCMI_IRQHandler, Default_Handler
    .weak      QR_IRQHandler
    .thumb_set QR_IRQHandler, Default_Handler
    .weak      GPU_IRQHandler
    .thumb_set GPU_IRQHandler, Default_Handler
    .weak      AWD_IRQHandler
    .thumb_set AWD_IRQHandler, Default_Handler
    .weak      DAC_IRQHandler
    .thumb_set DAC_IRQHandler, Default_Handler
    .weak      SPI5_IRQHandler
    .thumb_set SPI5_IRQHandler, Default_Handler

    .end
