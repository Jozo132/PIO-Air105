# PlatformIO + Arduino Blinky Test

Minimal smoke-test for the PIO-Air105 Arduino compatibility layer.

## What It Tests

| Arduino API | How |
|-------------|-----|
| `pinMode` / `digitalWrite` | LED blink at 1 Hz |
| `millis()` / `delay()` | Timing the blink and report intervals |
| `Serial.begin` / `print` | Status output on UART1 at 115200 baud |
| `analogRead` | Reads ADC channel A0 every 2 seconds |
| `analogWrite` | PWM ramp 0→255→0 on pin 3 |
| `attachInterrupt` | Counts button presses on pin 2 (FALLING edge) |

## Expected Serial Output

```
========================================
 PIO-Air105 — Arduino Blinky Test
========================================
 F_CPU:       204 MHz
 millis():    12

[OK] pinMode / digitalWrite — LED_BUILTIN configured
[OK] attachInterrupt — button on pin 2 (FALLING)
[OK] analogWrite — PWM on pin 3 initialized
[OK] analogRead — ready (10-bit, 0-1.8V default)

Entering loop... LED blinks, PWM ramps, serial reports every 2 s.
Press button on pin 2 to count interrupts.

[2s]  LED=ON   PWM=128  ADC= 512  BTN=0
[4s]  LED=OFF  PWM=  0  ADC= 510  BTN=3
[6s]  LED=ON   PWM=128  ADC= 515  BTN=5
```

## How to Build

> **Note:** This will not compile yet — the framework core and HAL must be
> implemented first (see `docs/ARDUINO_API_DESIGN.md`, Phase 1).

```bash
cd tests/platformio-blinky
pio run -e air105_core
```

## How to Upload

```bash
pio run -e air105_core -t upload
```

Upload uses the USB-DFU bootloader (once implemented).

## Hardware Setup

- **Air105 Core Board** (OpenLuat)
- **LED** — on-board LED (LED_BUILTIN)
- **Button** — connect between pin 2 and GND (internal pull-up enabled)
- **Potentiometer** — wiper to A0, ends to GND and 1.8V (default ADC range)
- **PWM** — observe pin 3 with an LED + resistor or oscilloscope
