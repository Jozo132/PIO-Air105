# Arduino IDE Blinky Test

Minimal smoke-test for the PIO-Air105 Arduino compatibility layer, packaged as a standard `.ino` sketch for the Arduino IDE.

## What It Tests

| Arduino API | How |
|-------------|-----|
| `pinMode` / `digitalWrite` | LED blink at 1 Hz |
| `millis()` / `delay()` | Timing the blink and report intervals |
| `Serial.begin` / `print` | Status output on UART1 at 115200 baud |
| `analogRead` | Reads ADC channel A0 every 2 seconds |
| `analogWrite` | PWM ramp 0→255→0 on pin 3 |
| `attachInterrupt` | Counts button presses on pin 2 (FALLING edge) |

## Arduino IDE Setup

> **Note:** The board package is not yet published. This sketch documents
> the target user experience once the framework is complete.

1. Open **File → Preferences** and add the Air105 board manager URL:
   ```
   https://raw.githubusercontent.com/Jozo132/PIO-Air105/main/package_air105_index.json
   ```
   *(URL is a placeholder — will be updated when the package is published)*

2. Open **Tools → Board → Boards Manager**, search for **Air105**, and install.

3. Select **Tools → Board → Air105 Core Board**.

4. Open `arduino-blinky.ino` and click **Upload**.

## PlatformIO Alternative

If you prefer PlatformIO (recommended for active development), see the sibling test at `tests/platformio-blinky/`.

## Hardware Setup

- **Air105 Core Board** (OpenLuat)
- **LED** — on-board LED (`LED_BUILTIN`)
- **Button** — connect between pin 2 and GND (internal pull-up enabled)
- **Potentiometer** — wiper to A0, ends to GND and 1.8V (default ADC range)
- **PWM** — LED + resistor or oscilloscope on pin 3

## Expected Output (Serial Monitor @ 115200)

```
========================================
 PIO-Air105 — Arduino Blinky Test
========================================
 F_CPU:       204 MHz
 millis():    12

[OK] LED_BUILTIN configured
[OK] Button interrupt on pin 2
[OK] PWM on pin 3
[OK] ADC ready (10-bit, 0-1.8V)

Running... reports every 2 s.

[2s] LED=ON   PWM=128  ADC=512  BTN=0
[4s] LED=OFF  PWM=0    ADC=510  BTN=3
```
