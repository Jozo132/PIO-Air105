# Air105 MCU Architecture, Peripherals, and Support Status

## Executive summary

Air105 is a 32‑bit microcontroller in QFN‑88 (10 mm × 10 mm) with **4 MB on‑chip Flash** and **640 KB SRAM**, designed and documented in Chinese by the OpenLuat/Luat ecosystem. citeturn35view0turn11view0 The available primary documentation set is unusually strong at the **register level**, including a full SoC memory map, peripheral base addresses, register descriptions, and several subsystem implementation details such as bit‑banding, DMA handshaking, ADC internal divider behavior, and USB suspend/resume. citeturn35view0turn9view0turn35view1turn35view3turn38view3

However, one important ambiguity remains: **the official Air105 chip-level manual describes a “32‑bit processor” but does not clearly name the CPU core** (in the accessible sections). citeturn35view0 Community work and third‑party tooling consistently treat Air105 as the Megahunt MH1903/MH1903S family and describe it as an Arm Cortex‑M4F‑class device (often associated with Arm SecurCore SC300). citeturn0search15turn24view0turn17view1turn17view3turn30search1 In practice, this is enough to proceed with a stable ABI assumption because the entire Cortex‑M / Armv7‑M ecosystem uses established calling conventions (AAPCS32) and exception/vector behavior. citeturn30search2turn30search0turn30search9

Software support divides into two tracks:

* **Vendor/official track (LuatOS + Air105 SoC SDK)**: compiles via `xmake`, supports both Windows and Linux dependency setup, and historically shipped working firmware artifacts (`.soc`). citeturn39view0turn38view0turn20view0 The Air105 SoC-specific repository was **archived/sealed on 2024‑08‑15**, which matters for long‑term maintenance planning. citeturn17view0turn21view0
* **Community track (PlatformIO‑style custom platforms, uploaders, OpenOCD work)**: there is functioning progress but it is explicitly described as “WIP” and relies heavily on reverse engineering and extraction from the LuatOS codebase/tooling. citeturn17view1turn17view3turn23view0turn37view0

The fastest path to “stable as possible” is to treat LuatOS as the ground truth for *behavior*, while building a rigorous, automated *test matrix* that can validate each peripheral across: (1) LuatOS firmware API; (2) bare‑metal C with an Arm GCC toolchain; and (3) PlatformIO integration, using identical electrical fixtures and measurable acceptance criteria.

## Official sources and chip-level architecture

The primary Chinese documentation accessible in this research includes:

* **Air105 chip datasheet/reference manual (Air105芯片数据手册_1.1)**: SoC block diagram, address map, and detailed peripheral register descriptions. citeturn35view0turn9view0  
* **Air105 MCU hardware design manual (Air105_MCU设计手册 v1.6)**: packaging, pin mux tables, power/charging circuits, oscillator/PLL options, and board-level design guidance. citeturn11view0turn35view4  
* **Air105 core board user manual (Air105核心板使用手册 v1.1)**: board‑level resource summary (what is actually pinned out / intended for use), including validated headline specs such as ADC sampling rate and PWM availability. citeturn35view5  

### System composition and buses

The SoC is organized around an AHB/APB bus matrix with major blocks including Flash/cache/QSPI flash controller, DMA, USB, OTP, a camera interface (DCMI), and a set of APB peripherals (GPIO, UART, SPI, I2C, timers/PWM, watchdog, TRNG, etc.). citeturn35view0turn9view0 The datasheet also explicitly places key high-throughput blocks (USB, LCD, DCMI, QSPI flash controller, DMA) in AHB address space. citeturn35view0turn9view0

### CPU core and ISA

*Official documentation visible here*: the SoC is described only as a “32‑bit processor.” citeturn35view0  
*Community and cross‑referenced evidence*: Air105 is widely treated as an MH1903/MH1903S‑class MCU and described as Cortex‑M4F‑class (and sometimes SC300‑class). citeturn0search15turn24view0turn17view1turn30search1

For stabilization work (toolchains, ROM calling, debugging), the operational assumption is therefore:

* **Armv7‑M / Thumb‑2 execution model**, compatible with the Cortex‑M4 programmer’s model and exception model. citeturn30search0turn30search10  
* **AAPCS32 calling convention** when mixing C and assembly and when calling ROM-resident function pointers. citeturn30search2turn30search9  

This assumption is reinforced by community ROM API usage patterns (function pointer indirection with standard register arguments). citeturn37view0

### Memory map and bit-banding

Key memory regions explicitly documented:

* SRAM base **0x2000_0000**, size **640 KB** (0x2000_0000–0x2009_FFFF). citeturn9view0  
* Peripheral regions mapped at **0x4000_0000** and above with explicit base ranges per peripheral. citeturn9view0turn35view0  
* Bit-band access is supported for SRAM and peripheral regions with the standard alias formula, enabling atomic bit manipulation via word writes. citeturn9view0  

### Clocking and power domains

Clocking and oscillator guidance are unusually explicit:

* Internal **12 MHz oscillator** (±2%) and option for an external **12 MHz crystal**. citeturn35view4turn11view0  
* PLL output frequency options: **108, 120, 132, 144, 156, 168, 180, 192, 204 MHz**. citeturn35view4turn11view0  
* A 32 kHz clock domain exists; RTC defaults to internal 32 kHz but can be switched to an external 32 kHz crystal. citeturn35view4turn11view0  

Power/boot control and charging:

* Two primary supply models are documented:  
  * **VCC (4.0–5.5 V)** into an internal 5 V→3.3 V LDO (with limited drive; design manual discusses current capability and adding external regulation for larger loads). citeturn11view0turn35view4  
  * **Direct 3.3 V supply** to VDD33/AVDD33/VBAT33 (this disables the documented power‑key on/off method and auto‑boots on power application). citeturn11view0  
* The chip integrates a Li‑ion charging function, with a documented max charge current of **200 mA** and charge voltage thresholds. citeturn11view0  

### Packaging and pinout model

* Package: **QFN‑88, 10 mm × 10 mm**. citeturn11view0  
* GPIO: design manual describes **up to 54 GPIO**, IO muxing, configurable interrupt mode, and default pull-up behavior (≈51 kΩ). citeturn11view0  
* Core board reality: the core board exposes a subset (documented as **39 available external GPIO** in the core board manual). citeturn35view5  

image_group{"layout":"carousel","aspect_ratio":"16:9","query":["Air105 chip pinout OpenLuat","Air105 核心板 引脚 air105_evb_pinout","Air105 QFN88 pinout diagram","Air105 OpenLuat development board Core-Air105"],"num_per_query":1}

### Boot and bootloader-related interfaces

Three separate “boot-related” mechanisms appear in the combined evidence:

* **Serial download + logging UART**: LuatOS documentation states Air105 has 4 UARTs and that **UART0 is fixed for download and logging**. citeturn40view2turn35view5  
* **High-speed flashing workflow**: the development board documentation warns to set the flashing baudrate to **1,500,000** to avoid garbled logs after flashing. citeturn36view3  
* **Secure boot signing path (community evidence)**: the community uploader supports an RSA signing key “only required if secure‑boot is enabled,” and notes a default entrypoint offset of **0x1001000** (community observation). citeturn17view3  

For ROM‑resident flash programming, a community OpenOCD effort documents a ROM function pointer table for QSPI flash operations starting at **0x00008010** (e.g., ROM_QSPI_Init/ReadID/EraseSector/ProgramPage) accessed through an indirection table. citeturn37view0 This is highly actionable for stabilizing open tooling, but must be treated as “engineering evidence” and validated on real silicon because the same source notes guessed/unstable elements (e.g., erase and DMA behavior). citeturn37view0

## Hardware peripheral inventory and register-level notes

The table below is built from the chip datasheet address map + peripheral chapters, and then cross‑checked against the core board manual and LuatOS usage notes where available. citeturn35view0turn9view0turn35view5turn40view1turn40view2

| Subsystem | Instances (evidence) | Base address (evidence) | Capabilities and limitations (what is explicitly documented) | DMA / IRQ evidence |
|---|---:|---|---|---|
| SRAM | 640 KB | 0x2000_0000–0x2009_FFFF citeturn9view0 | Byte/halfword/word access; bit-band aliasing supported. citeturn9view0 | N/A |
| Flash + cache path | 4 MB on-chip flash (system block) | Cache block at 0x4008_0000–0x4008_FFFF citeturn35view0turn9view0 | Cache mediates instruction fetch from flash; supports region decryption control registers (CACHE_SADDR/EADDR). citeturn8view2turn15search1 | Cache interacts with QSPI flash controller; QSPI has DMA control register. citeturn35view2 |
| QSPI flash controller (QFlash_controller) | 1 | 0x400A_2000–0x400A_2FFF citeturn35view2turn9view0 | Single/Dual/Quad I/O; configurable SPI polarity/phase and command format; FIFO-based operation; configurable DMA access. citeturn35view2turn34view1 | Explicit DMA_CNTL register present. citeturn35view2 Community ROM QSPI APIs exist but show DMA-related instability risks. citeturn37view0 |
| DMA | ≥8 channels (0–7 inferred) | DMA window 0x4000_0800–0x4000_0BFF citeturn35view0turn9view0 | Hardware handshaking select fields exist for channels 4–7 and enumerate sources including DCMI, LCD, UART0/1/2/3 TX/RX, DAC, SPI0/1/2, I2C, QSPI, SPI5. citeturn38view3turn8view3 | Multi-channel handshaking mapping is explicitly defined. citeturn38view3 |
| GPIO | Up to 54 (chip); 39 on core board | GPIO window at 0x4001_D000–0x4001_DFFF citeturn9view0turn11view0turn35view5 | Per-pin IO muxing; configurable input/output/interrupt; push‑pull or open‑drain; default pull-ups documented. citeturn11view0 Core board manual documents software IO toggle speed (~3 MHz). citeturn35view5 | GPIO interrupt status register behavior documented (write‑1‑to‑clear style). citeturn15search1 |
| UART | 4 (UART0–UART3) | UART0 0x4001_6000; UART1 0x4001_7000; UART2 0x4004_4000; UART3 0x4004_5000 citeturn9view0turn38view4 | LuatOS explicitly: UART0 is reserved for download + logs; UART1 typically used for application serial. citeturn40view2turn35view5 | DMA handshaking lists UART0/1/2/3 TX/RX as sources. citeturn38view3turn8view3 |
| SPI (regular controllers) | 4 in LuatOS use (0/1/2/5); SPI0 supports slave mode | SPIM0 0x4001_A000; SPIM1 0x4001_8000; SPIM2 0x4001_9000; SPIM5 0x400A_3000; SPIS0 0x4001_B000 citeturn9view0turn38view4turn35view0 | SPI0 has separate master and slave register sets; mode selection controlled by SYSCTRL registers; SPI1/2 are master-only per datasheet narrative. citeturn32view2 LuatOS SPI usage: **manual CS control is required** (wrap transfers with explicit GPIO CS low/high). citeturn40view1 | DMA handshaking lists SPI0/1/2 and SPI5 TX/RX. citeturn38view3turn8view3 |
| I2C | 1 | I2C0 0x4004_9000–0x4004_9FFF citeturn38view4turn9view0 | Standard/Fast mode; datasheet documents programmable glitch suppression (IC_FS_SPKLEN) and a WFI-based low-power entry sequence exists at SYSCTRL level. citeturn8view0turn8view1 Design manual recommends pull-ups (e.g., 4.7 kΩ). citeturn11view0 | DMA handshaking lists I2C TX/RX. citeturn38view3turn8view3 |
| Timers / PWM | 1 block, 8 timers | Timer block at 0x4001_3000–0x4001_3FFF citeturn31view3turn9view0 | 8 independent down-counters; independent interrupts; PWM mode; oneshot PWM; max PWM frequency **PCLK/2**. citeturn31view3turn31view2 Core board manual describes “5+1 PWM interface” availability (board/pin exposure + firmware usage). citeturn35view5 | Register controls include per-timer interrupt clear via read-to-clear EOI register. citeturn31view2 |
| Watchdog | 1 | 0x4001_C000–0x4001_CFFF citeturn9view0turn31view3 | Clocked from PCLK; can run in reset or interrupt mode; interrupt mode uses NMI as the interrupt source; once enabled it cannot be disabled. citeturn31view3 | Explicit NMI interrupt mode. citeturn31view3 |
| ADC | 7 channels (chip); 4 channels claimed on core board resource list | ADC 0x4001_4000–0x4001_40FF citeturn35view1turn32view0 | 12‑bit; max sampling **857 kHz**; reference **1.8 V**; CH0 dedicated to CHARGE_VBAT with internal divider (0–5 V); CH1–CH6 support selectable internal divider for 0–3.6 V vs 0–1.8 V range. citeturn35view1turn32view0 Dev board note: LuatOS does not enable internal divider by default (so CH1–CH6 range 0–1.8 V). citeturn36view3 | FIFO registers + threshold configuration are documented; this supports IRQ-driven acquisition patterns. citeturn35view1turn32view0 |
| DAC | 1 | DAC 0x4001_4100–0x4001_41FF citeturn33view0turn9view0 | 10‑bit; FIFO-based; explicit DMA enable; operational “running” status bit documented. citeturn33view0 | DMA source listed in DMA handshaking map. citeturn38view3 |
| USB | USB2.0 full-speed device | USB 0x4000_0C00–0x4000_0FFF citeturn35view3turn9view0 | USB2.0 full-speed device; supports suspend/resume; buffer descriptor table model described. citeturn35view3turn32view3 | DMA behavior is not explicitly stated as general-purpose, but the USB block is on AHB and designed for direct buffer access. citeturn35view3turn32view3 |
| DCMI | 1 | DCMIS 0x4006_0000–0x4006_FFFF citeturn33view2turn9view0 | Parallel camera interface: supports 8/10/12/14‑bit data, programmable pixel clock polarity, continuous or snapshot capture, cropping, and Bayer/monochrome formats. citeturn33view2 | Listed in DMA handshaking as a source. citeturn38view3 |
| LCD peripheral | 1 | LCD 0x4000_1000–0x4000_13FF citeturn9view0turn35view0 | The chip-level manual confirms presence and address range; DMA handshaking enumerates LCD as a DMA source/sink. citeturn35view0turn38view3 | Explicit appearance in DMA handshaking. citeturn38view3 |
| Keyboard matrix | 1 | KEYBOARD 0x4004_8000–0x4004_8FFF citeturn38view4turn35view0 | Address presence is documented; LuatOS build/feature lists include a “keyboard” library option. citeturn38view4turn39view0 | Not established here. |
| TRNG | 1 | TRNG 0x4001_E000–0x4001_EFFF citeturn9view0turn15search1 | Generates 128-bit true random numbers per cycle; optional interrupt request; “attack detection” flag behavior is described. citeturn15search1 | Interrupt enable bit documented. citeturn15search1 |
| CRC | 1 | CRC 0x4001_2000–0x4001_2FFF citeturn9view0turn15search1 | CRC16/CRC32 support; polynomials listed; byte-wise input; configurable bit-reversal and XOR output. citeturn15search1 | N/A |
| OTP | 1 | OTP 0x4000_8000–0x4000_BFFF citeturn9view0turn35view0 | Presence in memory map; LuatOS build/feature lists include an OTP library option and PlatformIO framework work references OTP fixes. citeturn9view0turn39view0turn23view2 | Not established here. |

### Assembly-level calling and ROM API implications

Two ingredients matter for “call it via assembly” work:

1) **Standard Arm procedure call rules (AAPCS32)**: first 4 arguments in r0–r3, return in r0, stack alignment rules apply. citeturn30search2turn30search9  
2) **ROM function pointer tables**: community OpenOCD work documents ROM QSPI functions accessed by loading a function pointer from a fixed ROM table and then branching/calling to it. citeturn37view0  

In practice, assembly stubs must do correct argument placement and must not violate stack alignment when mixing with C (especially if FPU is present, or when exception entry stacking occurs). citeturn30search0turn30search2

## Community and toolchain ecosystem status

### Official LuatOS + Air105 SoC SDK status

The Air105 SoC SDK repository explicitly states it has been **sealed since 2024‑08‑15** and is no longer maintained. citeturn17view0 Its commit history shows the “sealed” change as commit **eec53f4** on 2024‑08‑15. citeturn21view0 This matters because any remaining silicon/ROM quirks, driver bugs, or build-system changes must now be maintained downstream.

The documented build path for Air105 LuatOS firmware is:

* Install `7-zip` and `xmake` (Windows) or equivalents (Linux). citeturn39view0  
* Run `xmake` in the Air105 SDK repo; output firmware artifacts appear under `build/out/` with a `.soc` extension. citeturn38view0turn38view1  

LuatOS peripheral usage documentation is explicit on practical constraints:

* **SPI**: Air105 has SPI channels 0/1/2/5; CS must be controlled manually via GPIO around `spi.transfer`. citeturn40view1  
* **UART**: Air105 has UART0–UART3 and UART0 is used for download/logging. citeturn40view2  

### Community PlatformIO-style integration

There is no evidence in the sources reviewed that Air105 is supported by an official PlatformIO “platform” or board manifest; instead, the community has built custom PlatformIO packages:

* `platformio-air105` explicitly describes itself as a “WIP” and a hacky integration that imports most content from the LuatOS Air105 SDK. citeturn17view1turn23view0  
  * Commit evidence includes integration with the uploader tool (commit **58ab285**) and later fixes to memory layout / MH1903 naming (commit **3dc5fb3**). citeturn23view0  
* `framework-megahunt` advertises “AIR105 (MH1903) for PlatformIO” and its commit history explicitly notes partial bring-up milestones like “made UART0 work” (commit **3e762c3**) and later IRQ handler override exposure (commit **2df6cc7**). citeturn20view3turn23view2  

This is useful evidence for “what works”: at least basic startup and UART0 were made functional in the framework, but the commit language also strongly implies that many subsystems required iterative fixing and separation between “AIR105” and “MH1903 files.” citeturn23view2turn17view2

### Uploading and flashing: reverse engineering vs ROM APIs

A key stability risk for non‑Windows environments is flashing:

* The vendor’s Windows flashing tool source (`soc_download.exe`) is not published; a public issue (#83) asked for the source to port it to Linux. citeturn18view0  
* The community `air105-uploader` exists, but explicitly states it was produced by reverse engineering the official ISP tool and even requires an RSA private key if secure boot is enabled; it also asserts a default entrypoint at **0x1001000** (community observation). citeturn17view3turn23view1  

In parallel, an OpenOCD porting effort documents use of ROM QSPI APIs, but also records operational hazards:

* DMA usage in ROM QSPI programming was attempted and found unreliable (“DMA always failure” and occasional “dead” flashing situations); the author chose to avoid DMA. citeturn37view0  
* Multi‑sector erase optimization caused higher failure probability; >60 sectors per erase “always failed” and even “success” could correspond to an incompletely erased sector, causing non‑booting firmware. citeturn37view0  

These are exactly the kind of failure modes that must be turned into reproducible hardware tests and regression criteria for “stable as possible.”

### Comparative table of support stacks

| Stack | Build tooling | Flashing path | What is clearly “working” (evidence) | Known limitations / risks (evidence) |
|---|---|---|---|---|
| LuatOS (official firmware + libs) | `xmake` on Windows/Linux; outputs `.soc` citeturn39view0turn38view0 | Official serial flashing practices exist; dev board specifies 1.5 Mbaud for burning citeturn36view3 | SPI and UART documented and usable; UART0 reserved; SPI manual CS approach shown citeturn40view1turn40view2 | Air105 SoC SDK archived since 2024‑08‑15 (maintenance burden shifts to community) citeturn17view0turn21view0 |
| PlatformIO custom platform (`platformio-air105` + `framework-megahunt`) | PlatformIO packaging + custom framework; active commit history through 2025 citeturn17view1turn23view0turn23view2 | Integration with `air105-uploader` is explicitly committed citeturn23view0turn17view3 | UART0 bring-up is explicitly stated in commit history citeturn23view2 | Explicitly “WIP” and described as hacky; drivers likely incomplete and require verification per peripheral citeturn17view1turn23view2 |
| OpenOCD community driver approach | OpenOCD patching + ROM QSPI table calls citeturn37view0 | JTAG/SWD flash via ROM QSPI APIs; experiments documented citeturn37view0 | Demonstrated ability to erase/program via ROM calls citeturn37view0 | DMA+erase behavior unstable; multi-sector erase can “succeed” but still lead to non-booting images citeturn37view0 |

## Validation plan for peripheral and toolchain support

A stabilization program needs repeatable tests that produce artifacts (logs, waveforms, USB captures) and a pass/fail rubric. The plan below assumes a Core‑Air105 dev board and a small set of external fixtures (logic analyzer, scope, UART bridge, SPI flash, I2C device, optional camera/LCD).

### Test cases

The tests are ordered from “bring-up invariants” to high‑risk subsystems. Each test should be executed under: (A) LuatOS reference firmware; (B) bare‑metal C; (C) PlatformIO framework.

**Boot and clock sanity**

1. Flash a minimal image that toggles a GPIO at a known rate (e.g., 1 kHz square wave).  
2. Measure frequency with a scope/logic analyzer.  
3. Repeat at several PLL targets if configurable (e.g., 108 MHz vs 204 MHz).  
Expected: measured toggle frequency matches computed value within tolerance; failures often indicate wrong clock tree or wrong delay calibration. Clock options and PLL frequencies are explicitly documented. citeturn35view4turn11view0

**UART validation**

1. Confirm UART0 is left free for flashing/logs and use UART1 for application tests (per LuatOS guidance). citeturn40view2  
2. Run a UART loopback test (TX↔RX) at 115200 and at a high rate (where electrical setup permits).  
3. Verify no framing/parity errors and stable throughput.

**SPI and CS semantics**

1. Wire an external SPI flash (e.g., W25Q128-style) to HSPI (SPI channel 5) as in LuatOS reference. citeturn40view1  
2. Run “read JEDEC ID” and “page program + readback” tests.  
3. Confirm that chip select must be explicitly controlled as GPIO (wrap transfers with CS low/high). citeturn40view1  
Expected: correct IDs, stable readback, no bus contention.

**I2C + glitch filter robustness**

1. Connect a standard I2C EEPROM/sensor.  
2. Test standard (100 kHz) and fast (400 kHz) modes.  
3. Introduce controlled noise (long wires or injected spikes) and verify the glitch suppression configuration behaves as documented (IC_FS_SPKLEN behavior). citeturn8view0  
Expected: stable ACK/no spurious arbitration loss given reasonable noise.

**ADC: range modes, reference, and throughput**

1. Apply known voltages with a calibrated source: 0.2 V, 1.0 V, 1.7 V on channels 1–6.  
2. Toggle the internal divider configuration (0–1.8 V vs 0–3.6 V range) and verify scaling. citeturn35view1turn32view0  
3. Validate channel 0 behavior is fixed to CHARGE_VBAT and supports 0–5 V via internal divider. citeturn35view1  
4. Stress sampling rate (approach 857 kHz) and confirm FIFO/interrupt stability. citeturn35view1turn32view0  
Expected: monotonic conversion, correct scaling, stable FIFO behavior.

**DAC: FIFO and DMA-assisted waveform**

1. Set DAC to generate a stepped ramp through all codes, measure linearity on a scope (optionally with an RC filter).  
2. Enable DMA mode (where supported) and generate continuous waveform patterns. DAC supports DMA. citeturn33view0turn38view3  
Expected: stable waveform without underflow/overflow; correct “running” status behavior.

**Timers / PWM**

1. Validate timer interrupts for all 8 timers and read-to-clear EOI behavior. citeturn31view2turn31view3  
2. Generate PWM at multiple frequencies up to the documented maximum of PCLK/2 and compare duty and jitter. citeturn31view3  
3. Verify “oneshot PWM” behavior. citeturn31view3turn31view2

**Watchdog/NMI behavior**

1. Configure watchdog in interrupt mode and verify NMI fires before reset if not serviced. citeturn31view3  
2. Confirm watchdog cannot be disabled once enabled (attempt disable and confirm it remains active). citeturn31view3  
Expected: deterministic NMI and reset sequence.

**USB device mode**

1. Build a minimal USB device (CDC or HID) and verify enumeration and descriptors. USB is documented as FS device with suspend/resume. citeturn35view3turn32view3  
2. Validate suspend/resume transitions and clock gating behavior (host idle → suspend). citeturn32view3  
Expected: consistent enumeration and stable transfers.

**DCMI camera path**

1. Attach a supported camera module (the SoC DCMI supports 8–14-bit and snapshot/continuous/crop). citeturn33view2  
2. Capture a known pattern and verify line/frame sync and crop windows.  
Expected: stable frames at target resolution; no DMA overruns when enabled.

**Flash programming stability (critical for “stable as possible”)**

1. Reproduce OpenOCD ROM QSPI program/erase flows under controlled conditions. citeturn37view0  
2. Explicitly test:
   * ProgramPage with and without DMA pointers (expected: DMA path may be unstable). citeturn37view0  
   * Erase in batches of N sectors (N = 1, 10, 15, 20, 50, 60+) to characterize failure probability. citeturn37view0  
3. Add readback verification and boot verification (UART log indicates correct boot).  
Expected: define a conservative maximum “safe erase batch size” and implement retry + verify logic.

### Recommended testing flowchart

```mermaid
flowchart TD
  A[Establish reference firmware baseline (LuatOS)] --> B[Bring-up: GPIO toggle + clock verification]
  B --> C[UART1 loopback + logging discipline (UART0 reserved)]
  C --> D[SPI5 (HSPI) external flash ID + read/write]
  D --> E[I2C scan + noise/glitch filter checks]
  E --> F[ADC scaling tests (0–1.8V / 0–3.6V) + throughput]
  F --> G[Timers/PWM + watchdog NMI/reset validation]
  G --> H[USB FS enumeration + suspend/resume]
  H --> I[DCMI capture + optional LCD output]
  I --> J[Flash programming robustness: erase/program/verify matrix]
  J --> K[Cross-run the same suite on bare-metal GCC and PlatformIO]
  K --> L[Automate regression runs + publish known-good configurations]
```

## Gaps, prioritization, and stabilization roadmap

### Key gaps

* **CPU core identity clarity**: official accessible docs describe a “32‑bit processor” but do not clearly name the core; community evidence indicates Cortex‑M4F/SC300‑class, but this should be validated by reading CPUID and FPU presence at runtime under SWD. citeturn35view0turn0search15turn30search0  
* **Errata visibility**: no publicly accessible, authoritative errata document surfaced in the sources used here; stability work must therefore treat observed anomalies (flash erase failures, DMA instability in ROM APIs) as de‑facto errata until proven otherwise. citeturn37view0  
* **Open flashing toolchain completeness**: vendor tool source is not published (issue #83 demonstrates this gap), so Linux/macOS support relies on reverse‑engineered uploaders or ROM‑API‑based OpenOCD drivers. citeturn18view0turn17view3turn37view0  

### Prioritized tasks with estimated effort and risk

**High priority**

* **Create a hardware-verified flash programming specification** (ROM API contracts + safe erase/program parameters).  
  * Effort: 1–2 weeks of bench time.  
  * Risk: medium–high (ROM behavior may vary with clock/power/temp; incomplete erase “success” is especially hazardous). citeturn37view0  
* **Establish a golden peripheral regression suite** that runs on-device and emits machine-readable logs over UART1, with waveforms validated by external instruments where needed.  
  * Effort: 1–3 weeks to reach broad coverage.  
  * Risk: medium (requires fixture discipline).  
* **Document and codify LuatOS behavioral constraints** that affect correctness (manual CS control, UART0 reservation, ADC divider defaults).  
  * Effort: 2–4 days.  
  * Risk: low. citeturn40view1turn40view2turn36view3  

**Medium priority**

* **PlatformIO stabilization**: align linker scripts, memory map, vector table placement, and uploader integration, then run the same regression suite.  
  * Effort: 2–4 weeks depending on gaps found.  
  * Risk: medium (framework-megahunt history indicates iterative fixes were required). citeturn23view2turn23view0  
* **DMA validation per peripheral**: because DMA handshaking is rich (UART/SPI/I2C/DAC/DSMI/LCD/QSPI), a structured matrix is needed to confirm correctness under load.  
  * Effort: 1–2 weeks.  
  * Risk: medium (DMA misconfiguration can appear as sporadic data corruption). citeturn38view3turn35view2  

**Lower priority but strategically valuable**

* **Produce a community SVD/CMSIS pack** for debugger friendliness (register views, interrupt names, peripheral base addresses).  
  * Effort: 1–2 weeks once register definitions are harvested.  
  * Risk: low–medium (naming accuracy and IRQ mapping must be validated). citeturn9view0turn35view0  
* **Formalize “secure boot enabled” workflows**: define signing formats, key provisioning, and failure recovery using the uploader’s RSA path.  
  * Effort: 1–2 weeks.  
  * Risk: high (bricking risk if OTP/keys are mishandled). citeturn17view3turn9view0  

### Practical stabilization principle

Given the Air105 SoC SDK is archived and community tooling records flash/DMA hazards, the most effective strategy is to:

1) Adopt the **chip datasheet’s register model** as the canonical truth for address/register behavior. citeturn35view0turn9view0  
2) Treat LuatOS as a **behavioral reference implementation** (especially for pin muxing, peripheral init order, and safe defaults). citeturn40view1turn40view2turn36view3  
3) Convert every observed instability into a **measurable test + conservative workaround** (e.g., maximum safe erase batch size, DMA usage constraints, mandatory verify-after-program). citeturn37view0turn35view2turn38view3