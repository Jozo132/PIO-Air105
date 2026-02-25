/**
 * PIO-Air105 — Blinky Example
 *
 * Blinks the onboard RED LED at 1 Hz and prints to Serial.
 * Demonstrates basic Arduino API on the Air105 MCU.
 */

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial.println("PIO-Air105 Blinky Example");

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("LED ON");
    delay(500);

    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("LED OFF");
    delay(500);
}
