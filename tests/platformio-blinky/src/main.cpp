/**
 * PIO-Air105 — Serial Blinky Test
 * ================================
 * Uses Serial0 (UART0 → CH340 USB-serial)
 * 
 * Connect with RTS=OFF (RTS controls boot mode)
 */

#include <Arduino.h>

void setup() {
    Serial0.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial0.println("Air105 Started!");
    Serial0.print("SystemCoreClock: ");
    Serial0.println(SystemCoreClock);
}

uint32_t counter = 0;

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial0.print("Blink ");
    Serial0.println(counter++);
    delay(500);
    
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}

