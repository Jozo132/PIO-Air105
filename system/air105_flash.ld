/*
 * air105_flash.ld — Linker script for Air105 Arduino framework (standalone mode)
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Memory layout:
 *   FLASH: 4092 KB at 0x01001000 (first 4K after flash base reserved for header)
 *   RAM:   640 KB at 0x20000000
 *
 * Stack placed at top of RAM (grows downward).
 * Heap grows upward from end of .bss.
 *
 * SPDX-License-Identifier: MIT
 */

ENTRY(Reset_Handler)

/* Configurable sizes — can be overridden with -Wl,--defsym */
_Min_Heap_Size  = DEFINED(_Min_Heap_Size)  ? _Min_Heap_Size  : 0x10000;  /* 64 KB */
_Min_Stack_Size = DEFINED(_Min_Stack_Size) ? _Min_Stack_Size : 0x2000;   /* 8 KB  */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x01001000, LENGTH = 4092K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 640K
}

/* Stack at top of RAM */
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    /* ---- Vector table (must be first in flash) ---- */
    .isr_vector :
    {
        . = ALIGN(4);
        __isr_start_address = .;
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* ---- Code ---- */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* ---- Read-only data ---- */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* ---- ARM unwinding tables ---- */
    .ARM.extab :
    {
        . = ALIGN(4);
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } > FLASH

    .ARM :
    {
        . = ALIGN(4);
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
        . = ALIGN(4);
    } > FLASH

    /* ---- C/C++ init/fini arrays ---- */
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH

    /* ---- Load address of .data (in flash) ---- */
    _sidata = LOADADDR(.data);

    /* ---- Initialized data (copied from flash to RAM by startup) ---- */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        *(.RamFunc)
        *(.RamFunc*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    /* ---- Zero-initialized data ---- */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > RAM

    /* ---- Heap + Stack ---- */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        _heap_start = .;
        . = . + _Min_Heap_Size;
        _heap_end = .;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } > RAM

    /* ---- Discard standard library objects (we use nano.specs) ---- */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
