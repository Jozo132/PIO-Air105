# framework-arduino-air105

Arduino-compatible C++ framework for the Air105 (MH1903S) Cortex-M4F MCU.

## Directory Structure

```
framework/
├── cores/air105/                   Arduino core implementation
│   ├── Arduino.h                   Main header (type/macro defs, API decls)
│   ├── main.cpp                    Arduino main() — init → setup → loop
│   ├── wiring.c                    millis / micros / delay / delayMicroseconds
│   ├── wiring_digital.c            pinMode / digitalWrite / digitalRead
│   ├── wiring_analog.c             analogRead / analogWrite stubs
│   ├── WInterrupts.c               attachInterrupt / detachInterrupt
│   ├── Print.h / Print.cpp         Arduino Print base class
│   ├── Printable.h                 Printable interface
│   ├── Stream.h / Stream.cpp       Arduino Stream base class
│   ├── RingBuffer.h                Lock-free RX ring buffer
│   ├── HardwareSerial.h / .cpp     UART driver (Serial, Serial0-3)
│   └── (future: SPI.h, Wire.h, …)
├── variants/
│   └── air105_devboard/            LuatOS Air105 Core Board variant
│       ├── variant.h               Pin numbers, LED, SPI, I2C defs
│       ├── variant.cpp             Board-specific init (if needed)
│       └── pins_arduino.h          Standard Arduino pin alias header
├── system/                         Low-level startup / system init
│   ├── startup_air105.s            Vector table + Reset_Handler
│   ├── air105_flash.ld             Linker script (4 MB flash, 640 KB RAM)
│   ├── system_air105.c             SystemInit (PLL, FPU, clocks, SysTick)
│   └── system_air105.h             System function declarations
├── package.json                    PlatformIO framework package metadata
└── README.md                       This file
```

## Clock Model

| Clock     | Frequency | Description                          |
|-----------|-----------|--------------------------------------|
| PLL       | 204 MHz   | Internal 12 MHz × 17                |
| HCLK      | 102 MHz   | CPU / SysTick (PLL ÷ 2)             |
| PCLK      | 51 MHz    | APB peripherals (HCLK ÷ 2)          |
| SystemCoreClock | 204 MHz | = PLL (vendor convention) |

`SystemCoreClock` is set to the **PLL frequency** (not HCLK). This matches
the vendor luatos-soc-air105 convention, where hardware timing formulas use
`SystemCoreClock >> N` to derive actual bus clocks.

## Dependencies

The framework uses these from `vendor/luatos-soc-air105/` (MIT):
- `bsp/air105/chip/include/` — register definition headers (`air105.h`)
- `bsp/cmsis/include/` — CMSIS Cortex-M4 headers (`core_cm4.h`)

No vendor `.c` files are compiled. All HAL code is written from scratch.
