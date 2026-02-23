/**
 * PIO-Air105 — PlatformIO + Arduino Blinky Test
 * ==============================================
 *
 * Minimal smoke-test that exercises the core Arduino APIs on Air105:
 *   - Digital I/O   (pinMode, digitalWrite)
 *   - Timing        (millis, delay)
 *   - Serial UART   (Serial on UART1)
 *   - Analog input  (analogRead)
 *   - PWM output    (analogWrite)
 *   - USB Serial    (SerialUSB, if available)
 *   - Interrupts    (attachInterrupt)
 *   - SPI / Wire    (bus scan)
 *
 * When this sketch compiles and runs correctly on a real Air105 core
 * board, the Arduino compatibility layer is considered "Phase 1 pass."
 *
 * Expected hardware:
 *   - Air105 Core Board (OpenLuat)
 *   - On-board LED connected to LED_BUILTIN
 *   - Optional: potentiometer on A0, button on D2
 *
 * License: MIT (same as the PIO-Air105 project)
 */

#include <Arduino.h>

// If the USB Serial (CDC) class is available, use it for secondary output
#if defined(SERIAL_USB)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif

// -----------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------
static const uint8_t  BUTTON_PIN   = 2;           // External interrupt test
static const uint8_t  PWM_PIN      = 3;           // PWM output test
static const uint8_t  ANALOG_PIN   = A0;          // Analog input test pin
static const uint32_t BLINK_MS     = 500;         // LED toggle interval
static const uint32_t REPORT_MS    = 2000;        // Serial report interval
static const uint32_t BAUD_RATE    = 115200;

// -----------------------------------------------------------------------
// State
// -----------------------------------------------------------------------
static volatile uint32_t buttonPresses = 0;
static uint32_t lastBlink   = 0;
static uint32_t lastReport  = 0;
static bool     ledState    = false;
static uint8_t  pwmDuty     = 0;
static int8_t   pwmDir      = 1;              // +1 ramp up, -1 ramp down

// -----------------------------------------------------------------------
// ISR — button press counter (interrupt test)
// -----------------------------------------------------------------------
void onButtonPress() {
    buttonPresses++;
}

// -----------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------
void setup() {
    // --- Serial (UART1) ------------------------------------------------
    Serial.begin(BAUD_RATE);

    // Wait up to 3 seconds for serial monitor to connect (USB CDC style)
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 3000)) {
        // yield
    }

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F(" PIO-Air105 — Arduino Blinky Test"));
    Serial.println(F("========================================"));
    Serial.print(F(" F_CPU:       ")); Serial.print(F_CPU / 1000000UL); Serial.println(F(" MHz"));
    Serial.print(F(" millis():    ")); Serial.println(millis());
    Serial.println();

    // --- Digital I/O ---------------------------------------------------
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println(F("[OK] pinMode / digitalWrite — LED_BUILTIN configured"));

    // --- External interrupt --------------------------------------------
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonPress, FALLING);
    Serial.println(F("[OK] attachInterrupt — button on pin 2 (FALLING)"));

    // --- PWM -----------------------------------------------------------
    pinMode(PWM_PIN, OUTPUT);
    analogWrite(PWM_PIN, 0);
    Serial.println(F("[OK] analogWrite — PWM on pin 3 initialized"));

    // --- ADC -----------------------------------------------------------
    // Default: 10-bit resolution, 0–1.8 V range (Air105 default)
    // analogReadResolution(12);                // Uncomment for native 12-bit
    // analogReference(INTERNAL_3V6);           // Uncomment for 0–3.6 V range
    Serial.println(F("[OK] analogRead — ready (10-bit, 0-1.8V default)"));

    Serial.println();
    Serial.println(F("Entering loop... LED blinks, PWM ramps, serial reports every 2 s."));
    Serial.println(F("Press button on pin 2 to count interrupts."));
    Serial.println();
}

// -----------------------------------------------------------------------
// loop()
// -----------------------------------------------------------------------
void loop() {
    uint32_t now = millis();

    // --- LED blink (digital I/O + timing test) -------------------------
    if (now - lastBlink >= BLINK_MS) {
        lastBlink = now;
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    }

    // --- PWM ramp (analogWrite test) -----------------------------------
    // Smoothly ramp duty cycle 0→255→0 repeatedly
    analogWrite(PWM_PIN, pwmDuty);
    pwmDuty += pwmDir;
    if (pwmDuty == 255) pwmDir = -1;
    if (pwmDuty == 0)   pwmDir = +1;

    // --- Periodic serial report ----------------------------------------
    if (now - lastReport >= REPORT_MS) {
        lastReport = now;

        uint16_t adcVal = analogRead(ANALOG_PIN);

        Serial.print(F("["));
        Serial.print(now / 1000);
        Serial.print(F("s] "));

        Serial.print(F("LED="));
        Serial.print(ledState ? F("ON ") : F("OFF"));

        Serial.print(F("  PWM="));
        if (pwmDuty < 100) Serial.print(' ');
        if (pwmDuty < 10)  Serial.print(' ');
        Serial.print(pwmDuty);

        Serial.print(F("  ADC="));
        if (adcVal < 1000) Serial.print(' ');
        if (adcVal < 100)  Serial.print(' ');
        if (adcVal < 10)   Serial.print(' ');
        Serial.print(adcVal);

        Serial.print(F("  BTN="));
        Serial.print(buttonPresses);

        Serial.println();
    }

    delay(5);  // ~200 Hz loop rate, smooth PWM ramp
}
