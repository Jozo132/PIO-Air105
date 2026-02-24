/**
 * PIO-Air105 — Serial Blinky + ADC Test
 * ======================================
 * Uses Serial0 (UART0 → CH340 USB-serial)
 * Tests ADC reading on A0 (PC0) and battery voltage
 * 
 * ADC input range: 0-1.8V (12-bit, 0-4095)
 * Connect with RTS=OFF (RTS controls boot mode)
 */

#include <Arduino.h>

void setup() {
    Serial0.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial0.println("Air105 Started!");
    Serial0.print("SystemCoreClock: ");
    Serial0.println(SystemCoreClock);
    Serial0.println("ADC Test: reading A0 (PC0) and VBAT");
}

uint32_t counter = 0;

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    
    /* Read ADC channel A0 (PC0 = pin 32, ADC channel 1) */
    int adcValue = analogRead(A0);
    
    /* Read battery voltage */
    int vbat = analogReadVBAT();
    
    Serial0.print("Blink ");
    Serial0.print(counter++);
    Serial0.print(" | A0=");
    Serial0.print(adcValue);
    Serial0.print(" | VBAT=");
    Serial0.print(vbat);
    Serial0.println("mV");
    
    delay(500);
    
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}

