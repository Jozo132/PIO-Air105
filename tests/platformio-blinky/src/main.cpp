/**
 * PIO-Air105 — Minimal Blinky Test
 * =================================
 * Simple test that blinks LED and sends serial data.
 * Used to verify basic Arduino framework functionality.
 * 
 * Note: Using Serial0 (UART0, PA0/PA1) as it's connected to CH340 USB-Serial
 */

#include <Arduino.h>

// Use Serial0 (UART0) which is connected to the CH340 USB-serial chip
#define DEBUG_SERIAL Serial0

void setup() {
    // Initialize serial at 115200 baud
    DEBUG_SERIAL.begin(115200);
    
    // Configure LED pin
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Send startup message
    DEBUG_SERIAL.println("Air105 Arduino Framework");
    DEBUG_SERIAL.println("========================");
    DEBUG_SERIAL.print("F_CPU: ");
    DEBUG_SERIAL.print(F_CPU / 1000000UL);
    DEBUG_SERIAL.println(" MHz");
}

void loop() {
    // Blink LED
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    
    DEBUG_SERIAL.print("ON  - millis: ");
    DEBUG_SERIAL.println(millis());
    
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    
    DEBUG_SERIAL.print("OFF - millis: ");
    DEBUG_SERIAL.println(millis());
}

