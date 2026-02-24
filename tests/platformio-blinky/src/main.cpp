/**
 * PIO-Air105 — Timer Interrupt Blink Test (STM32duino-compatible API)
 * 
 * Demonstrates HardwareTimer class:
 *   - TIM6: Blinks RED LED using MICROSEC_FORMAT (500ms = 500000us)
 *   - TIM7: Blinks GREEN LED using HERTZ_FORMAT (2 Hz)
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
    /* LEDs are controlled by timer interrupts - nothing to do here */
}
