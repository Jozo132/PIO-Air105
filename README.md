# PIO-Air105

**PlatformIO Arduino-compatible support package for the Air105 MCU**

Air105 is a 32-bit Arm Cortex-M4F-class microcontroller (MH1903/MH1903S family) in QFN-88 with 4 MB on-chip Flash and 640 KB SRAM, originally designed by the OpenLuat/Luat ecosystem.

## Project Goals

- **PlatformIO platform** — full board/toolchain integration for Air105
- **Arduino-compatible C++ framework** — familiar API (`Serial`, `SPI`, `Wire`, `digitalRead`/`Write`, `analogRead`, PWM, etc.)
- **Direct USB bootloader upload** — no vendor-specific Windows-only flashing tools required
- **Prebuilt helper tools** — built from scratch where needed for a self-contained workflow

## Project Status

🚧 **Early development** — gathering sources, validating peripheral behavior, designing architecture.

## Repository Structure

```
PIO-Air105/
├── vendor/                        # Upstream references (git submodules)
│   ├── luatos-soc-air105/         # MIT — Official Air105 SoC SDK (HAL/driver source)
│   ├── platformio-air105/         # Apache-2.0 — Community PlatformIO platform definition
│   ├── air105-uploader/           # Apache-2.0 — Serial ISP uploader (protocol reference)
│   └── mhdumper/                  # MIT — Megahunt boot ROM dumper (ROM API reference)
├── deep-research-report.md        # Comprehensive Air105 architecture research
├── LICENSE                        # MIT
├── README.md
└── .gitignore
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

## Getting Started

> 🚧 Coming soon — the project is in early development.

## License

This project is licensed under the [MIT License](LICENSE).

Upstream submodules retain their original licenses (see table above). The `vendor/` directory contains unmodified upstream code under their respective licenses.
