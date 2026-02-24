/**
 * PIO-Air105 — Serial Blinky + ADC + PWM Test
 * ============================================
 * Uses Serial0 (UART0 → CH340 USB-serial)
 * Tests:
 *   - Three built-in LEDs (red, green, blue)
 *   - ADC reading on A0 (PC0) and battery voltage
 *   - PWM output on PWM4 (PC6)
 * 
 * ADC input range: 0-1.8V (12-bit, 0-4095)
 * PWM: 1 kHz default, 8-bit resolution (0-255)
 * Connect with RTS=OFF (RTS controls boot mode)
 */

#include <Arduino.h>

uint32_t counter = 0;
uint8_t pwmValue = 0;
int8_t pwmDir = 5;  /* PWM step direction */

void setup() {
    Serial0.begin(115200);
    
    /* Initialize all three LEDs */
    pinMode(LED_BUILTIN_RED, OUTPUT);
    pinMode(LED_BUILTIN_GREEN, OUTPUT);
    pinMode(LED_BUILTIN_BLUE, OUTPUT);
    
    /* Turn all LEDs off initially */
    digitalWrite(LED_BUILTIN_RED, LOW);
    digitalWrite(LED_BUILTIN_GREEN, LOW);
    digitalWrite(LED_BUILTIN_BLUE, LOW);
    
    Serial0.println("Air105 Started!");
    Serial0.print("SystemCoreClock: ");
    Serial0.println(SystemCoreClock);
    Serial0.println("Test: LEDs (RGB cycle), ADC (A0 + VBAT), PWM (PC6)");
}

void loop() {
    /* Cycle through LED colors based on counter */
    uint8_t phase = (counter / 2) % 3;
    digitalWrite(LED_BUILTIN_RED,   (phase == 0) ? HIGH : LOW);
    digitalWrite(LED_BUILTIN_GREEN, (phase == 1) ? HIGH : LOW);
    digitalWrite(LED_BUILTIN_BLUE,  (phase == 2) ? HIGH : LOW);
    
    /* Update PWM (triangle wave) */
    pwmValue += pwmDir;
    if (pwmValue >= 250) pwmDir = -5;
    if (pwmValue <= 5) pwmDir = 5;
    analogWrite(PWM4, pwmValue);  /* PWM on PC6 */
    
    /* Read ADC */
    int adcValue = analogRead(A0);
    int vbat = analogReadVBAT();
    
    /* Print status */
    Serial0.print("Cnt=");
    Serial0.print(counter++);
    Serial0.print(" LED=");
    const char* colors[] = {"RED", "GRN", "BLU"};
    Serial0.print(colors[phase]);
    Serial0.print(" PWM=");
    Serial0.print(pwmValue);
    Serial0.print(" A0=");
    Serial0.print(adcValue);
    Serial0.print(" VBAT=");
    Serial0.print(vbat);
    Serial0.println("mV");
    
    delay(100);  /* 10 Hz update rate */
}

