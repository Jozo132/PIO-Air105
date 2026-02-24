/**
 * PIO-Air105 — Timer Interrupt Test (STM32duino-compatible API)
 * 
 * Demonstrates:
 *   - HardwareTimer: TIM6 blinks RED (500ms), TIM7 blinks GREEN (2Hz)
 *   - Note: CORE-Air105-V1.0 board has no user button, only BOOT/RESET
 */

#include <Arduino.h>

/* Timer instances */
HardwareTimer timer6(TIM6);
HardwareTimer timer7(TIM7);

/* Interrupt callback for RED LED (microseconds) */
void blinkRed() {
    static bool state = false;
    state = !state;
    digitalWrite(LED_BUILTIN_RED, state);
}

/* Interrupt callback for GREEN LED (Hz) */
void blinkGreen() {
    static bool state = false;
    state = !state;
    digitalWrite(LED_BUILTIN_GREEN, state);
}

void setup() {
    pinMode(LED_BUILTIN_RED, OUTPUT);
    pinMode(LED_BUILTIN_GREEN, OUTPUT);
    pinMode(LED_BUILTIN_BLUE, OUTPUT);
    
    /* Timer 6: Blink RED at 1Hz (toggle every 500ms) */
    timer6.setOverflow(500000, MICROSEC_FORMAT);
    timer6.attachInterrupt(blinkRed);
    timer6.resume();
    
    /* Timer 7: Blink GREEN at 1Hz (toggle at 2Hz = 500ms effective blink) */
    timer7.setOverflow(2, HERTZ_FORMAT);
    timer7.attachInterrupt(blinkGreen);
    timer7.resume();
}

void loop() {
    /* All LEDs controlled by interrupts - nothing to do here */
}
