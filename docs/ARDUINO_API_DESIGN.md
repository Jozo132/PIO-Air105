# Arduino-Compatible C++ Framework Design — Air105

> Design specification for the `framework-arduino-air105` component.
> Modeled after [STM32duino](https://github.com/stm32duino/Arduino_Core_STM32) architecture but purpose-built for the Air105 / MH1903S MCU.

## 1 — Design Philosophy

Standard Arduino sketches that use the documented Arduino API must compile and run on Air105 **without modification**, the same way STM32duino lets STM32 boards run Arduino code out of the box.

Concretely:

```cpp
// This must Just Work™ on Air105
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  Serial.println("Hello from Air105!");
}
```

Third-party Arduino libraries (Adafruit, SparkFun, U8g2, FastLED, etc.) that rely only on the standard Arduino API should work without any Air105-specific patches.

---

## 2 — Layer Architecture

```
┌─────────────────────────────────────────────┐
│  User Sketch (.ino / .cpp)                  │
├─────────────────────────────────────────────┤
│  Arduino API Layer (cores/air105/)          │  ← Standard Arduino API
│    Digital I/O, Analog I/O, Serial,         │
│    SPI, Wire, Timing, Interrupts, USB       │
├─────────────────────────────────────────────┤
│  Air105 HAL (hal/)                          │  ← Thin C HAL extracted from
│    GPIO, UART, SPI, I2C, Timer, ADC, DAC,   │     luatos-soc-air105 (MIT)
│    DMA, USB, SYSCTRL, Cache, TRNG, WDT      │
├─────────────────────────────────────────────┤
│  CMSIS (cmsis/)                             │  ← Arm CMSIS headers
│    core_cm4.h, system_air105.c              │
├─────────────────────────────────────────────┤
│  Startup + Linker Scripts                   │  ← startup_air105.s
│    Vector table, Reset_Handler,             │     air105_flash.ld
│    SystemInit, stack/heap config            │
├─────────────────────────────────────────────┤
│  Hardware: Air105 SoC                       │
│    Cortex-M4F, 4 MB Flash, 640 KB SRAM      │
└─────────────────────────────────────────────┘
```

### 2.1 — Layer Responsibilities

| Layer | Language | Responsibility |
|-------|----------|---------------|
| **User Sketch** | C++ | Application code using Arduino API |
| **Arduino API** | C++ | Implements the Arduino API contract; delegates to HAL |
| **Air105 HAL** | C | Register-level peripheral drivers. Stateless where possible. Extracted from MIT-licensed luatos-soc-air105 SDK. |
| **CMSIS** | C/ASM | Arm standard headers, intrinsics, SysTick, NVIC |
| **Startup/Linker** | ASM/LD | Vector table, clock init, memory layout |

### 2.2 — Why This Matches STM32duino

STM32duino uses the exact same layering: `Arduino API → STM32 HAL (from ST Cube) → CMSIS → Startup`. The difference is that ST publishes official HAL/LL drivers whereas we extract ours from the MIT-licensed LuatOS SDK. The Arduino API layer we write from scratch.

---

## 3 — Directory Structure

```
framework-arduino-air105/
├── cores/air105/
│   ├── main.cpp                  # Arduino main() — calls setup()/loop()
│   ├── wiring.c                  # millis(), micros(), delay(), delayMicroseconds()
│   ├── wiring_digital.c          # pinMode(), digitalWrite(), digitalRead()
│   ├── wiring_analog.c           # analogRead(), analogWrite() (PWM)
│   ├── wiring_shift.c            # shiftIn(), shiftOut()
│   ├── wiring_pulse.c            # pulseIn(), pulseInLong()
│   ├── hooks.c                   # yield(), __cxa_pure_virtual, etc.
│   ├── itoa.c                    # Integer ↔ string (Arduino convention)
│   ├── WInterrupts.c             # attachInterrupt(), detachInterrupt()
│   ├── WMath.cpp                 # random(), randomSeed(), map()
│   ├── WString.cpp               # Arduino String class
│   ├── Print.cpp                 # Print base class
│   ├── Stream.cpp                # Stream base class
│   ├── HardwareSerial.h / .cpp   # Serial on UART (see §5)
│   ├── USBSerial.h / .cpp        # CDC-ACM USB Serial (optional)
│   ├── Tone.cpp                  # tone(), noTone()
│   ├── IPAddress.h / .cpp        # (stub, for library compat)
│   ├── Arduino.h                 # Master include — pulls everything together
│   ├── api/                      # ArduinoCore-API submodule (if using new-style API)
│   └── air105/                   # SoC-specific internals
│       ├── air105_hal.h          # Aggregate HAL include
│       ├── PinNamesVar.h         # Internal pin enum ↔ GPIO port/pin mapping
│       ├── PeripheralPins.h      # Pin → peripheral alternate function tables
│       ├── clock_config.c        # PLL/clock tree initialization
│       ├── interrupt_handlers.c  # Default IRQ handler stubs (weak symbols)
│       └── syscalls.c            # _write(), _sbrk(), newlib stubs for printf/heap
│
├── variants/
│   └── air105_core_board/
│       ├── variant.h             # Pin definitions, LED_BUILTIN, SERIAL_TX/RX, etc.
│       ├── variant.cpp           # Pin map arrays, SystemClock_Config()
│       ├── ldscript.ld           # Linker script for this board variant
│       └── pins_arduino.h       # Arduino-standard pin header (includes variant.h)
│
├── libraries/
│   ├── SPI/
│   │   ├── SPI.h
│   │   └── SPI.cpp              # Arduino SPI class → Air105 SPI peripheral
│   ├── Wire/
│   │   ├── Wire.h
│   │   └── Wire.cpp             # Arduino Wire class → Air105 I2C0
│   └── (future: Servo, SoftwareSerial, EEPROM emulation, etc.)
│
├── hal/                          # Air105 register-level HAL (C)
│   ├── include/
│   │   ├── air105.h              # Master SoC header: register structs, base addrs
│   │   ├── air105_gpio.h
│   │   ├── air105_uart.h
│   │   ├── air105_spi.h
│   │   ├── air105_i2c.h
│   │   ├── air105_timer.h
│   │   ├── air105_adc.h
│   │   ├── air105_dac.h
│   │   ├── air105_dma.h
│   │   ├── air105_usb.h
│   │   ├── air105_wdt.h
│   │   ├── air105_trng.h
│   │   ├── air105_crc.h
│   │   ├── air105_cache.h
│   │   └── air105_sysctrl.h
│   └── src/
│       ├── air105_gpio.c
│       ├── air105_uart.c
│       ├── air105_spi.c
│       ├── air105_i2c.c
│       ├── air105_timer.c
│       ├── air105_adc.c
│       ├── air105_dac.c
│       ├── air105_dma.c
│       ├── air105_usb.c
│       ├── air105_wdt.c
│       ├── air105_trng.c
│       ├── air105_crc.c
│       ├── air105_cache.c
│       └── air105_sysctrl.c
│
├── cmsis/
│   ├── core_cm4.h                # Arm CMSIS Cortex-M4 header
│   ├── cmsis_gcc.h               # GCC intrinsics
│   └── system_air105.h / .c      # SystemInit(), SystemCoreClock
│
├── system/
│   ├── startup_air105.s          # Vector table + Reset_Handler
│   └── air105_flash.ld           # Default linker script
│
├── platform.txt                  # PlatformIO/Arduino IDE build recipes
├── boards.txt                    # Board definitions (Arduino IDE format)
└── programmers.txt               # Programmer definitions
```

---

## 4 — Pin Model

### 4.1 — Pin Numbering

Following STM32duino convention, we define a flat `uint8_t` pin space where each Arduino pin number maps to a physical GPIO port + pin. The variant file defines the mapping.

```cpp
// variant.h — Air105 Core Board (39 exposed GPIOs)
// Pin numbers 0..N

enum {
  PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,    // Port A
  PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7,   // Port B
  PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7,     // Port C
  PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7,    // Port D
  PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7,     // Port E
  PF0, PF1, PF2, PF3, PF4, PF5,              // Port F
  NUM_DIGITAL_PINS
};
```

> **Note:** The exact port/pin mapping will be determined from the Air105 GPIO register layout and core board schematic. Air105 has up to 54 GPIOs on-chip, 39 exposed on the core board.

### 4.2 — Alternate Function Mapping

Each pin can be muxed to a peripheral function (UART TX/RX, SPI MOSI/MISO/SCK, I2C SDA/SCL, PWM, ADC). We maintain a `PinMap` table (same pattern as STM32duino):

```cpp
// PeripheralPins.c
const PinMap PinMap_UART_TX[] = {
  {PA_X, UART1, AF_UART1_TX},   // Serial (application UART)
  {PA_Y, UART2, AF_UART2_TX},   // Serial1
  {PA_Z, UART3, AF_UART3_TX},   // Serial2
  {NC, NC, 0}
};

const PinMap PinMap_PWM[] = {
  {PB_X, TIMER0, AF_TIM0_PWM},
  {PB_Y, TIMER1, AF_TIM1_PWM},
  // ... 8 timers available for PWM
  {NC, NC, 0}
};

const PinMap PinMap_ADC[] = {
  {PC_X, ADC_CH1, 0},   // 0–1.8V default
  {PC_Y, ADC_CH2, 0},
  // ... up to 6 application channels (CH0 is VBAT)
  {NC, NC, 0}
};
```

### 4.3 — Standard Aliases

```cpp
// pins_arduino.h
#define LED_BUILTIN     PC7       // Core board LED (to be confirmed from schematic)
#define PIN_SERIAL_TX   PA_X      // UART1 TX
#define PIN_SERIAL_RX   PA_Y      // UART1 RX
#define PIN_SPI_MOSI    PB_X      // SPI0 MOSI
#define PIN_SPI_MISO    PB_Y      // SPI0 MISO
#define PIN_SPI_SCK     PB_Z      // SPI0 SCK
#define PIN_SPI_SS      PB_W      // SPI0 CS (GPIO-managed)
#define PIN_WIRE_SDA    PD_X      // I2C0 SDA
#define PIN_WIRE_SCL    PD_Y      // I2C0 SCL
#define A0              PC_X      // ADC channel 1
#define A1              PC_Y      // ADC channel 2
// ... etc.
```

---

## 5 — API Surface & Hardware Mapping

### 5.1 — Digital I/O

| Arduino API | Air105 Implementation |
|------------|----------------------|
| `pinMode(pin, mode)` | Configure GPIO port/pin via SYSCTRL mux register + GPIO direction register. Modes: `INPUT` (hi-Z), `INPUT_PULLUP` (built-in ~51 kΩ), `INPUT_PULLDOWN` (if available), `OUTPUT` (push-pull), `OUTPUT_OPEN_DRAIN` |
| `digitalWrite(pin, val)` | Write GPIO data register bit |
| `digitalRead(pin)` | Read GPIO data register bit |

**Implementation notes:**
- GPIO base: `0x4001_D000`
- Air105 GPIOs default to pull-up (~51 kΩ). `INPUT` mode must explicitly disable pull-up.
- Bit-band access (SRAM & peripheral alias regions) enables atomic single-bit writes without read-modify-write — we should use this for `digitalWrite()` performance.
- Max software toggle speed documented at ~3 MHz.

### 5.2 — Analog Input (ADC)

| Arduino API | Air105 Implementation |
|------------|----------------------|
| `analogRead(pin)` | Read ADC channel mapped to pin. 12-bit result (0–4095). |
| `analogReadResolution(bits)` | Store resolution; shift 12-bit result to match (default 10-bit for Arduino compat) |
| `analogReference(type)` | `DEFAULT` = internal 1.8V; `INTERNAL_3V6` = enable internal divider for 0–3.6V range |

**Implementation notes:**
- ADC base: `0x4001_4000`
- 12-bit native resolution, 857 kHz max sampling rate
- 7 channels total: CH0 dedicated to VBAT monitoring (0–5V via internal divider), CH1–CH6 for application use
- **Default range is 0–1.8V** (internal divider NOT enabled by default, matching LuatOS behavior)
- Air105-specific extension: `analogReference(INTERNAL_3V6)` enables internal divider for 0–3.6V input range
- `analogRead()` default returns 10-bit (0–1023) for Arduino compatibility; `analogReadResolution(12)` unlocks native 12-bit

### 5.3 — Analog Output / PWM

| Arduino API | Air105 Implementation |
|------------|----------------------|
| `analogWrite(pin, value)` | PWM output on the timer channel mapped to that pin |
| `analogWriteResolution(bits)` | Configure PWM resolution (default 8-bit) |
| `analogWriteFrequency(pin, freq)` | Air105 extension: set PWM frequency (default 1 kHz) |

**Implementation notes:**
- Timer base: `0x4001_3000`
- 8 independent timer/counter channels, each can generate PWM
- PWM max frequency: PCLK/2 (e.g., at 204 MHz sysclk, PCLK is likely sysclk/2 or /4)
- Each timer has independent prescaler and reload value
- `analogWrite(pin, 0)` = constant LOW; `analogWrite(pin, 255)` = constant HIGH

### 5.4 — DAC Output

| Arduino API | Air105 Implementation |
|------------|----------------------|
| `analogWrite(DAC_PIN, value)` | True analog output via DAC peripheral |

**Implementation notes:**
- DAC base: `0x4001_4100`
- 10-bit resolution, FIFO-based
- When `analogWrite()` is called on the DAC pin, the framework detects it and routes to the DAC peripheral instead of PWM
- Air105 has only 1 DAC channel

### 5.5 — Serial (UART)

| Arduino Object | Air105 Peripheral | Notes |
|---------------|-------------------|-------|
| `Serial` | **UART1** (`0x4001_7000`) | Primary application serial. **NOT UART0.** |
| `Serial1` | **UART2** (`0x4004_4000`) | Second application serial |
| `Serial2` | **UART3** (`0x4004_5000`) | Third application serial |
| *(reserved)* | **UART0** (`0x4001_6000`) | Bootloader/download only — not exposed as Arduino Serial |

> **UART0 is permanently reserved** for the bootloader download and logging interface. Exposing it as an Arduino Serial would conflict with the flashing mechanism and confuse users. This matches how STM32duino reserves certain UARTs on some boards.

```cpp
// HardwareSerial.h
class HardwareSerial : public Stream {
public:
  void begin(unsigned long baud);
  void begin(unsigned long baud, uint16_t config); // SERIAL_8N1, etc.
  void end();
  int available();
  int read();
  int peek();
  size_t write(uint8_t byte);
  size_t write(const uint8_t *buf, size_t size);
  void flush();
  operator bool();

  // Air105 extensions
  void setRxBufferSize(size_t size);       // Override default 256-byte ring buffer
  void enableDMA(bool tx = true, bool rx = true);  // Enable DMA transfers
};

extern HardwareSerial Serial;   // UART1
extern HardwareSerial Serial1;  // UART2
extern HardwareSerial Serial2;  // UART3
```

**Implementation notes:**
- All UARTs support DMA (TX/RX channels in DMA handshaking map)
- Default: interrupt-driven with 256-byte ring buffer (configurable)
- High-speed baud rates supported (LuatOS uses 1.5 Mbaud for flashing)
- Standard configs: `SERIAL_8N1`, `SERIAL_8E1`, `SERIAL_8O1`, `SERIAL_8N2`, etc.

### 5.6 — USB Serial (CDC-ACM)

| Arduino Object | Air105 Peripheral | Notes |
|---------------|-------------------|-------|
| `SerialUSB` | USB FS Device (`0x4000_0C00`) | CDC-ACM virtual COM port |

```cpp
// USBSerial.h
class USBSerial : public Stream {
public:
  void begin(unsigned long baud = 0); // baud is ignored (USB-native)
  void end();
  int available();
  int read();
  size_t write(uint8_t byte);
  size_t write(const uint8_t *buf, size_t size);
  void flush();
  operator bool();             // Returns true when host is connected
};

extern USBSerial SerialUSB;
```

**Implementation notes:**
- USB2.0 Full-Speed device on AHB bus
- Buffer descriptor table model (no general-purpose DMA for USB stated)
- `SerialUSB` provides the same `Stream` interface as `Serial`
- Dual role: communication + potential upload mechanism (USB bootloader re-entry)
- `operator bool()` returns DTR state — sketches can wait for terminal connection

### 5.7 — SPI

```cpp
// SPI.h
class SPIClass {
public:
  SPIClass(uint8_t spiPort = 0);

  void begin();
  void end();
  void beginTransaction(SPISettings settings);
  void endTransaction();
  uint8_t transfer(uint8_t data);
  uint16_t transfer16(uint16_t data);
  void transfer(void *buf, size_t count);

  // Air105 extensions
  void enableDMA(bool enable = true);

private:
  uint8_t _csPin;      // GPIO-managed CS
  // ...
};

extern SPIClass SPI;    // SPI0 master
extern SPIClass SPI1;   // SPI1 master (optional, for advanced users)
```

| Arduino Object | Air105 Peripheral | Base Address |
|---------------|-------------------|--------------|
| `SPI` | SPI_M0 | `0x4001_A000` |
| `SPI1` | SPI_M1 | `0x4001_8000` |

**Implementation notes:**
- **Critical: Air105 requires manual GPIO CS control.** The SPI hardware does NOT automatically assert/deassert chip-select.
- `beginTransaction()` sets the mode/speed AND drives the CS pin LOW via GPIO.
- `endTransaction()` drives the CS pin HIGH.
- This is transparent to Arduino libraries — they call `beginTransaction()`/`endTransaction()` as usual and CS "just works."
- SPI0 also has a slave mode (SPI_S0 at `0x4001_B000`), selectable via SYSCTRL — exposed through a separate `SPISlave` class if needed.
- DMA handshaking available for all SPI controllers (SPI0/1/2/5).

### 5.8 — Wire (I2C)

```cpp
// Wire.h
class TwoWire {
public:
  void begin();                          // Master mode
  void begin(uint8_t address);           // Slave mode
  void setClock(uint32_t freq);          // 100000 or 400000
  void beginTransmission(uint8_t addr);
  uint8_t endTransmission(bool stop = true);
  uint8_t requestFrom(uint8_t addr, uint8_t qty, bool stop = true);
  size_t write(uint8_t data);
  size_t write(const uint8_t *data, size_t qty);
  int available();
  int read();
  int peek();

  // Air105 extensions
  void enableDMA(bool enable = true);
};

extern TwoWire Wire;   // I2C0
```

| Arduino Object | Air105 Peripheral | Base Address |
|---------------|-------------------|--------------|
| `Wire` | I2C0 | `0x4004_9000` |

**Implementation notes:**
- Only 1 I2C instance on Air105
- Standard mode (100 kHz) and Fast mode (400 kHz) supported
- Programmable glitch suppression via `IC_FS_SPKLEN` register
- External 4.7 kΩ pull-ups recommended per design manual
- DMA handshaking available for TX/RX
- Slave mode supported (if `begin(address)` is called)

### 5.9 — Timing Functions

| Arduino API | Air105 Implementation |
|------------|----------------------|
| `millis()` | SysTick-based 32-bit millisecond counter (wraps at ~49.7 days) |
| `micros()` | SysTick cycle counter with division (or DWT CYCCNT for sub-µs) |
| `delay(ms)` | Busy-wait using `millis()` with `yield()` calls |
| `delayMicroseconds(us)` | DWT CYCCNT-based busy-wait (cycle-accurate) |

**Implementation notes:**
- SysTick configured for 1 ms tick at `SystemCoreClock`
- `delay()` calls `yield()` in its spin loop — this enables cooperative tasking and `serialEvent()` processing
- `micros()` uses SysTick current value + overflow count for <1 µs precision at 204 MHz

### 5.10 — External Interrupts

| Arduino API | Air105 Implementation |
|------------|----------------------|
| `attachInterrupt(digitalPinToInterrupt(pin), ISR, mode)` | Configure GPIO interrupt on the specified pin |
| `detachInterrupt(digitalPinToInterrupt(pin))` | Disable GPIO interrupt |

Supported modes: `RISING`, `FALLING`, `CHANGE`, `LOW`, `HIGH`

**Implementation notes:**
- Air105 GPIO supports per-pin interrupt configuration
- Interrupt status register is write-1-to-clear style
- ISR callback stored in a function pointer table indexed by pin number
- `digitalPinToInterrupt()` macro maps Arduino pin to interrupt index

### 5.11 — Miscellaneous

| Arduino API | Implementation |
|------------|---------------|
| `tone(pin, freq)` / `noTone(pin)` | Timer-based square wave generation |
| `shiftIn()` / `shiftOut()` | Bit-bang via `digitalWrite`/`digitalRead` |
| `pulseIn()` / `pulseInLong()` | Cycle-count based pulse measurement |
| `random()` / `randomSeed()` | Can optionally seed from TRNG (128-bit true random) |
| `yield()` | Weak symbol — default empty, overridable for RTOS |
| `init()` | System init: clock tree, SysTick, default peripheral clocks |

---

## 6 — Air105-Specific Extensions

Beyond the standard Arduino API, we expose chip-specific features that don't exist on AVR/SAM/STM32. These live in separate headers so standard sketches don't need them.

```cpp
#include <Air105_Extensions.h>

// True Random Number Generator
uint32_t trueRandom32();                  // TRNG: 32-bit true random
void trueRandomBytes(uint8_t *buf, size_t len);  // Fill buffer from TRNG

// Hardware CRC
uint32_t hardwareCRC32(const uint8_t *data, size_t len);
uint16_t hardwareCRC16(const uint8_t *data, size_t len);

// DAC (explicit control beyond analogWrite)
void dacBegin();
void dacWrite(uint16_t value);           // 10-bit
void dacEnd();

// Watchdog (WARNING: cannot be disabled once enabled!)
void watchdogEnable(uint32_t timeout_ms);
void watchdogReset();                     // Feed the watchdog
// NOTE: No watchdogDisable() — Air105 WDT cannot be turned off

// OTP Memory
bool otpRead(uint32_t offset, uint8_t *buf, size_t len);
bool otpWrite(uint32_t offset, const uint8_t *buf, size_t len);  // One-time!

// System Info
uint32_t getSystemClock();               // Current PLL frequency in Hz
float getCpuTemperature();               // If die temp sensor exists
float getVBAT();                         // ADC CH0 with internal divider → battery voltage

// Power Management
void enterSleepMode();                   // WFI-based light sleep
void enterDeepSleep(uint32_t wakeup_ms); // RTC-based deep sleep (if supported)

// USB Bootloader re-entry
void resetToBootloader();                // Jump to USB bootloader for firmware update
```

---

## 7 — Clock & Startup Sequence

### 7.1 — Boot Flow

```
Power On / Reset
     │
     ▼
ROM Bootloader (in mask ROM)
     │ checks boot pins / flags
     ▼
Application Vector Table (at flash offset after bootloader region)
     │
     ▼
Reset_Handler (startup_air105.s)
     │
     ├─ Copy .data from flash to SRAM
     ├─ Zero .bss
     ├─ Call SystemInit()
     │     ├─ Configure PLL → target frequency (default 204 MHz)
     │     ├─ Configure flash cache / wait states
     │     ├─ Enable peripheral clocks as needed
     │     └─ Set SystemCoreClock variable
     ├─ Call __libc_init_array() (C++ static constructors)
     │
     ▼
main() [cores/air105/main.cpp]
     │
     ├─ init()             → SysTick, default GPIO, UART clocks
     ├─ initVariant()      → Board-specific init (variant.cpp, weak)
     ├─ setup()            → User setup function
     │
     └─ loop forever:
           ├─ loop()       → User loop function
           └─ serialEventRun()  → Check for serial events
```

### 7.2 — Default Clock Configuration

| Clock Source | Frequency | Notes |
|-------------|-----------|-------|
| Internal RC | 12 MHz ±2% | Default at reset |
| External Crystal | 12 MHz | Optional, higher accuracy |
| PLL Output | **204 MHz** (default) | Selectable: 108–204 MHz in 12 MHz steps |
| PCLK (APB) | PLL/2 or PLL/4 | Peripheral bus clock |
| SysTick | PLL (core clock) | 1 ms tick for `millis()` |
| RTC | Internal 32 kHz | Switchable to external 32.768 kHz crystal |

---

## 8 — Linker Script Layout

```
MEMORY {
  /* Bootloader occupies first region of flash */
  BOOT (rx)  : ORIGIN = 0x01000000, LENGTH = 64K    /* USB bootloader */
  FLASH (rx) : ORIGIN = 0x01010000, LENGTH = 4032K  /* Application */
  SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 640K   /* All SRAM */
}

SECTIONS {
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text       : { *(.text*) } > FLASH
  .rodata     : { *(.rodata*) } > FLASH
  .data       : { *(.data*) } > SRAM AT > FLASH
  .bss        : { *(.bss*) } > SRAM
  ._user_heap_stack : {
    . = ALIGN(8);
    _heap_start = .;
    . = . + _Min_Heap_Size;    /* default 64 KB */
    . = . + _Min_Stack_Size;   /* default 8 KB */
    _estack = .;
  } > SRAM
}
```

> **Note:** Flash origin `0x01000000` and app offset `0x01010000` are based on community-observed entrypoint at `0x1001000`. The exact bootloader size will be determined during USB bootloader development. SRAM is the full 640 KB at `0x20000000`.

---

## 9 — Build Integration (PlatformIO)

### 9.1 — Board Definition

```json
// boards/air105_core.json
{
  "build": {
    "core": "air105",
    "cpu": "cortex-m4",
    "f_cpu": "204000000L",
    "mcu": "air105",
    "variant": "air105_core_board"
  },
  "frameworks": ["arduino"],
  "name": "Air105 Core Board",
  "upload": {
    "protocol": "usb-dfu",
    "maximum_size": 4128768,
    "maximum_ram_size": 655360
  },
  "url": "https://wiki.luatos.com/chips/air105/board.html",
  "vendor": "OpenLuat"
}
```

### 9.2 — Build Flags

```ini
# platformio.ini
[env:air105_core]
platform = air105
board = air105_core
framework = arduino

# Optional overrides
build_flags =
  -DAIR105_CLOCK_204MHZ          ; Default PLL frequency
  -DSERIAL_BUFFER_SIZE=512       ; Override default 256-byte ring buffer

upload_protocol = usb-dfu        ; Direct USB upload (no special tools)
```

### 9.3 — Compiler Flags (platform.txt)

```
compiler.flags.cpu = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
compiler.flags.opt = -Os -ffunction-sections -fdata-sections -fno-exceptions
compiler.flags.warn = -Wall -Wextra
compiler.flags.defines = -DAIR105 -DARM_MATH_CM4
linker.flags = -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs
```

---

## 10 — Compatibility Matrix

### 10.1 — Standard Arduino Libraries (must work)

| Library | Depends On | Status |
|---------|-----------|--------|
| Adafruit GFX | SPI, Wire, Print | Should work — standard API only |
| Adafruit BusIO | SPI, Wire | Should work |
| U8g2 | SPI, Wire, GPIO | Should work |
| SD | SPI | Should work (needs testing at Air105 SPI speeds) |
| FastLED | GPIO, SPI, timing | Needs testing — tight timing on `digitalWrite` at 3 MHz toggle |
| Servo | Timer/PWM | Should work via `analogWrite` timer backend |
| OneWire | GPIO, `delayMicroseconds` | Should work — bit-bang protocol |
| IRremote | Timer, GPIO interrupt | Should work |
| ArduinoJSON | None (pure C++) | Will work |
| PubSubClient (MQTT) | Stream (any Serial/WiFi) | Will work over Serial bridge |

### 10.2 — What WON'T Work (and why)

| Feature | Reason |
|---------|--------|
| WiFi / BLE libraries | Air105 has no wireless radio |
| EEPROM.h (native) | No EEPROM peripheral — need flash emulation layer |
| Ethernet.h (native) | No Ethernet MAC — needs external W5500 over SPI |
| `analogWrite` on arbitrary pins | Only pins with timer/PWM alternate function |

---

## 11 — Testing Strategy

### 11.1 — Unit Tests (host-compiled)

- Pure logic in `WString`, `Print`, `WMath` → compile and test on host with GCC
- Pin map table consistency checks (no duplicate mappings, all peripherals covered)

### 11.2 — Hardware-in-the-Loop (HIL) Tests

| Test | Method | Pass Criteria |
|------|--------|---------------|
| `digitalWrite` toggle | Oscilloscope on pin | Square wave at expected frequency |
| `analogRead` accuracy | Known voltage divider input | Reading within ±2% of expected |
| `Serial` loopback | TX→RX loopback wire | Sent data == received data at 115200–1500000 baud |
| `SPI` loopback | MOSI→MISO wire + logic analyzer | Correct clock polarity/phase, data integrity |
| `Wire` scan | Known I2C device (e.g., BME280) | Device responds at correct address |
| `millis()` accuracy | Compare to external reference over 60 seconds | Drift < 0.1% |
| `attachInterrupt` | External button + debounce | ISR fires on correct edge |
| PWM duty cycle | Oscilloscope | Duty within ±1% of programmed value |
| USB Serial (CDC) | Host PC terminal | Bidirectional communication, `operator bool()` works |

### 11.3 — Arduino Library Smoke Tests

Run example sketches from popular libraries (Adafruit SSD1306, U8g2, SD) and verify they compile and produce correct output.

---

## 12 — Implementation Phases

| Phase | Deliverable | Dependencies |
|-------|------------|--------------|
| **Phase 1: Skeleton** | Blinky LED (`digitalWrite`, `delay`) compiles and runs | HAL GPIO, CMSIS, startup, linker script, SysTick |
| **Phase 2: Serial** | `Serial.println("Hello")` works on UART1 | HAL UART, interrupt-driven ring buffer |
| **Phase 3: Digital I/O** | Full `pinMode`/`digitalRead`/`digitalWrite` + interrupts | HAL GPIO, pin mux tables |
| **Phase 4: Analog** | `analogRead` + `analogWrite` (PWM) | HAL ADC, HAL Timer/PWM |
| **Phase 5: SPI + Wire** | SPI and I2C libraries pass loopback tests | HAL SPI, HAL I2C, GPIO CS management |
| **Phase 6: USB** | `SerialUSB` works, USB bootloader upload functional | HAL USB, CDC-ACM stack, bootloader |
| **Phase 7: Polish** | All tests pass, documentation, PlatformIO registry publish | Everything above |

---

## 13 — Key Constraints & Gotchas

These are non-obvious Air105 behaviors that the Arduino wrapper must handle silently:

| # | Constraint | Impact | Our Solution |
|---|-----------|--------|-------------|
| 1 | **UART0 is reserved** for download/logging | Cannot be used as `Serial` | `Serial` = UART1; UART0 not exposed |
| 2 | **SPI has no hardware CS** | Libraries expect CS management | `beginTransaction()` auto-asserts GPIO CS pin |
| 3 | **ADC default range is 0–1.8V** | Voltages >1.8V will clip without divider | Default to 10-bit/1.8V; provide `analogReference(INTERNAL_3V6)` |
| 4 | **Watchdog cannot be disabled** | `wdt_disable()` would be a trap | No `watchdogDisable()` function; document prominently |
| 5 | **ADC CH0 is VBAT-dedicated** | Cannot use as general-purpose analog input | Exclude from `A0..An` mappings; provide `getVBAT()` extension |
| 6 | **GPIO defaults to pull-up** (~51 kΩ) | `INPUT` mode isn't truly floating | `INPUT` explicitly disables pull-up in `pinMode()` |
| 7 | **Flash programming: no DMA** | Bootloader must avoid ROM DMA APIs | Use polling-mode flash writes with verify |
| 8 | **Max PLL is 204 MHz** | `F_CPU` must match actual PLL setting | Validate at compile time via `static_assert` |
| 9 | **Only 1 I2C peripheral** | No `Wire1` available | Only `Wire` exposed; document limitation |
| 10 | **USB is Full-Speed only** (12 Mbps) | No High-Speed USB | CDC throughput limited; adequate for serial + upload |

---

## 14 — References

- [Arduino Language Reference](https://www.arduino.cc/reference/en/)
- [Arduino Core API Specification](https://github.com/arduino/ArduinoCore-API)
- [STM32duino Core](https://github.com/stm32duino/Arduino_Core_STM32) — primary architectural reference
- [Air105 Chip Datasheet (Air105芯片数据手册_1.1)](https://wiki.luatos.com/) — register-level documentation
- [Air105 MCU Design Manual (v1.6)](https://wiki.luatos.com/) — hardware design guidance
- [luatos-soc-air105](https://github.com/openLuat/luatos-soc-air105) (MIT) — HAL source
- [racerxdl/platformio-air105](https://github.com/racerxdl/platformio-air105) (Apache-2.0) — PlatformIO platform reference
- [racerxdl/air105-uploader](https://github.com/racerxdl/air105-uploader) (Apache-2.0) — upload protocol reference
