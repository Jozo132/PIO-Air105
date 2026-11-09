# PIO-Air105

**PlatformIO + Arduino IDE support package for the Air105 MCU**

Air105 is a 32-bit Arm Cortex-M4F-class microcontroller (MH1903/MH1903S family) in QFN-88 with 4 MB on-chip Flash and 640 KB SRAM, originally designed by the OpenLuat/Luat ecosystem.

## Project Goals

- **Dual IDE support** — works with both PlatformIO and Arduino IDE
- **Arduino-compatible C++ framework** — familiar API (`Serial`, `SPI`, `Wire`, `digitalRead`/`Write`, `analogRead`, PWM, etc.)
- **Cross-platform USB bootloader upload** — native serial backends for Windows and POSIX hosts
- **Self-contained** — all build dependencies bundled, no git submodules needed for users

## Project Status

🚧 **Active development** — core Arduino API functional, platform installable from GitHub.

### Implemented APIs
- **Digital I/O** — `pinMode`, `digitalWrite`, `digitalRead`
- **Analog I/O** — `analogRead`, `analogWrite` (PWM), `analogReadVBAT`
- **Timing** — `millis`, `micros`, `delay`, `delayMicroseconds`
- **Serial** — `HardwareSerial` (UART0–UART3)
- **SPI** — Full STM32duino-compatible API (SPIM0/1/2)
- **I2C** — `Wire` (I2C0, PE6=SDA, PE7=SCL)
- **Timers** — `HardwareTimer` (STM32duino-compatible, TIM0–TIM7)
- **GPIO Interrupts** — `attachInterrupt` / `detachInterrupt`
- **DMA** — 8-channel DMA with M2M, M2P, P2M transfers
- **Print / Stream** — Arduino `Print` and `Stream` base classes

## Version 0.2.0

### Timing and Clock Configuration

- The startup CPU/PLL clock is 204 MHz; PlatformIO sets `F_CPU` from the board configuration and rejects unsupported clock overrides.
- `millis()` and `micros()` use SysTick, including pending-tick handling at the millisecond boundary.
- `delay()` and `delayMicroseconds()` remain synchronous. They call `yield()` during normal thread-context waits longer than 10 microseconds; short microsecond waits use cycle polling. Delays in interrupt or interrupt-masked context also use cycle polling.
- SysTick can retain only one pending tick. Interrupts disabled for more than one millisecond can therefore cause `millis()` and `micros()` to lose elapsed whole ticks.

### Serial Buffers

- Each UART has separate 256-byte RX and TX ring buffers by default. One slot is reserved to distinguish full from empty, leaving 255 bytes usable per buffer.
- TX `write()` waits for ring space; `flush()` waits for queued and hardware transmission to finish. `availableForWrite()` reports free TX-ring capacity.
- RX cannot wait in its interrupt handler. If the RX ring fills, new bytes are discarded; `getRxOverflowCount()` reports the number discarded.
- Buffer sizes can be changed at compile time:

```ini
build_flags =
    -DSERIAL_RX_BUFFER_SIZE=512
    -DSERIAL_TX_BUFFER_SIZE=1024
```

### Native Uploader

- The active uploader uses Python’s standard library and native Windows or POSIX serial APIs; it does not require PySerial, Crypto, or `scpukit` at runtime.
- Unsigned firmware uploads are supported. RSA-signed secure-boot uploads are not supported by the native uploader.

The serial stress test transfers 8 KiB in each direction and checks for data mismatches and RX overflow. Host ring-buffer tests cover default and overridden sizes; see `tests/serial/` and `tests/timing/`.

## Getting Started

### Quick Start (PlatformIO)

1. Create a new PlatformIO project directory with `src/main.cpp`
2. Add this `platformio.ini`:

```ini
[env:air105]
platform = https://github.com/Jozo132/PIO-Air105.git
board = air105_coreboard
framework = arduino

upload_port = COM9        ; ← your serial port
monitor_speed = 115200
```

3. Write your sketch in `src/main.cpp`:

```cpp
#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial.println("Hello Air105!");
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}
```

4. Build and upload:
```bash
pio run              # Build
pio run -t upload    # Upload via USB bootloader
pio device monitor   # Serial monitor
```

### Quick Start (Arduino IDE)

1. Open **File → Preferences → Additional Boards Manager URLs** and add:
   ```
   https://raw.githubusercontent.com/Jozo132/PIO-Air105/master/package_air105_index.json
   ```
   > **Note:** The Board Manager JSON index is not yet published. For now, install manually:
   > Clone this repo and add its path via **File → Preferences → Sketchbook location**, or
   > symlink/copy it into your Arduino `hardware/` folder as `hardware/air105/air105/`.

2. Select **Tools → Board → Air105 Core Board**
3. Select your COM port under **Tools → Port**
4. Upload your sketch (the same code from the PlatformIO example above works here too)

## Repository Structure

```
PIO-Air105/
├── platform.json                  # PlatformIO platform manifest
├── platform.py                    # PlatformIO platform class
├── platform.txt                   # Arduino IDE build recipes
├── boards.txt                     # Arduino IDE board definitions
├── programmers.txt                # Arduino IDE programmer defs (empty)
├── builder/                       # PlatformIO build scripts
│   ├── main.py                    #   Toolchain config, targets, upload
│   └── frameworks/
│       └── arduino.py             #   Arduino framework source builder
├── boards/
│   └── air105_coreboard.json      # PlatformIO board definition
├── cores/air105/                  # Arduino API implementation
│   ├── Arduino.h                  #   Main include (pulls in all APIs)
│   ├── HardwareSerial.*           #   UART0–UART3
│   ├── HardwareTimer.*            #   TIM0–TIM7 (STM32duino-compatible)
│   ├── SPI.*                      #   SPI (STM32duino-compatible)
│   ├── Wire.*                     #   I2C (Wire library)
│   ├── DMA.*                      #   8-channel DMA controller
│   └── ...                        #   wiring, Print, Stream, etc.
├── variants/air105_devboard/      # Board-specific pin mappings
├── system/                        # Low-level system files
│   ├── cmsis/                     #   CMSIS core headers (Cortex-M4)
│   ├── air105.h                   #   Chip register map
│   ├── air105_flash.ld            #   Linker script
│   ├── startup_air105.s           #   Startup assembly
│   └── system_air105.*            #   System init (clock, SysTick)
├── tools/uploader/                # USB bootloader upload tool
├── examples/blinky/               # Example project
├── tests/platformio-blinky/       # Development test project
├── tests/serial/                  # Serial ring-buffer and high-volume tests
├── tests/timing/                  # Timing arithmetic and hardware tests
└── vendor/                        # Upstream references (git submodules)
```

## Vendor Submodules & Licenses

| Submodule | Source | License | Our Use |
|-----------|--------|---------|---------|
| `vendor/luatos-soc-air105` | [openLuat/luatos-soc-air105](https://github.com/openLuat/luatos-soc-air105) | MIT | Primary HAL/driver extraction source |
| `vendor/platformio-air105` | [racerxdl/platformio-air105](https://github.com/racerxdl/platformio-air105) | Apache-2.0 | PlatformIO platform starting point |
| `vendor/air105-uploader` | [racerxdl/air105-uploader](https://github.com/racerxdl/air105-uploader) | Apache-2.0 | Serial ISP protocol reference |
| `vendor/mhdumper` | [racerxdl/mhdumper](https://github.com/racerxdl/mhdumper) | MIT | ROM API analysis reference |

## Honorable Mentions & Acknowledgments

This project would not be possible without the pioneering work of many individuals and communities. We gratefully acknowledge:

### Lucas Teske ([@racerxdl](https://github.com/racerxdl))

The single most significant community contributor to Air105/MH1903 open-source tooling. His work includes:

- **[platformio-air105](https://github.com/racerxdl/platformio-air105)** (Apache-2.0) — The first PlatformIO platform definition for Air105/MH1903, proving the concept is viable. Self-described as "WIP" but achieved working UART0 output and basic startup.
- **[air105-uploader](https://github.com/racerxdl/air105-uploader)** (Apache-2.0) — Reverse-engineered the vendor's closed-source ISP flashing tool to create a cross-platform uploader. This was a critical contribution for non-Windows development.
- **[framework-megahunt](https://github.com/racerxdl/framework-megahunt)** — PlatformIO framework with HAL, linker scripts, IRQ handler overrides, and iterative bring-up fixes. ⚠️ *No license specified; code cannot be used directly, but the engineering insights and approach are invaluable as prior art.*
- **[mhdumper](https://github.com/racerxdl/mhdumper)** (MIT) — Boot ROM dumper for Megahunt devices, enabling ROM API reverse engineering.
- **[megahunt-bootroms](https://github.com/racerxdl/megahunt-bootroms)** — Collected boot ROM binary dumps for AIR105 and MH1903 variants. ⚠️ *No license specified; referenced for understanding only.*

### OpenLuat / Luat Community

- **[luatos-soc-air105](https://github.com/openLuat/luatos-soc-air105)** (MIT) — The official Air105 SoC SDK providing the ground-truth HAL, peripheral drivers, startup code, and FreeRTOS integration. Archived on 2024-08-15 but remains the most complete and tested codebase for this chip.
- **[LuatOS](https://github.com/openLuat/LuatOS)** — The broader Lua-based IoT firmware framework that originally drove Air105 support.
- **Wendal Chen ([@wendal](https://github.com/wendal))** — Provided the basic programming program used in LuatOS (referenced in [issue #83](https://github.com/openLuat/LuatOS/issues/83)), which became the seed for the community uploader work.

### Wiz-IO ([@Wiz-IO](https://github.com/Wiz-IO))

- **[framework-wizio-pico](https://github.com/Wiz-IO/framework-wizio-pico)** — The upstream PlatformIO framework for Raspberry Pi Pico that `framework-megahunt` was forked from. Its PlatformIO framework architecture pattern influenced the community Air105 integration approach. ⚠️ *No license specified.*

### OpenOCD Community Contributors

- Anonymous/community contributors who documented ROM QSPI function pointer tables at `0x00008010` and characterized flash programming hazards (DMA instability, multi-sector erase failures). This reverse engineering work is critical safety knowledge for anyone programming Air105 flash.

### iosetting

- **[air105_project](https://gitee.com/iosetting/air105_project)** — Keil5 MDK project template for Air105, referenced in the official LuatOS SDK README as a third-party resource.

---

## Why a Fresh Framework Instead of Forking?

The most useful community framework (`framework-megahunt`) and its upstream (`framework-wizio-pico`) have **no license specified**, which under copyright law means all rights are reserved. We cannot legally fork or copy code from these repositories.

Our approach:
1. **Fork what we can** — `platformio-air105` (Apache-2.0) as platform starting point, `luatos-soc-air105` (MIT) as HAL/driver source
2. **Build fresh what we must** — Arduino framework layer, USB bootloader, linker scripts, startup code — all extracted from MIT-licensed sources or written from scratch
3. **Honor what inspired us** — acknowledge all prior art that informed our architecture decisions

## Getting Started (Development)

To develop or contribute to this platform:

```bash
git clone --recursive https://github.com/Jozo132/PIO-Air105.git
cd PIO-Air105/tests/platformio-blinky
pio run
```

The test project uses `platform = ../..` to reference the local repo as the platform source.

## License

This project is licensed under the [MIT License](LICENSE).

Upstream submodules retain their original licenses (see table above). The `vendor/` directory contains unmodified upstream code under their respective licenses.
