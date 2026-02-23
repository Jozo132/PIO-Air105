/*
 * PIO-Air105 — Arduino IDE Blinky Test
 * =====================================
 *
 * This is the same smoke-test as the PlatformIO version, but packaged
 * as a standard .ino sketch for the Arduino IDE.
 *
 * To use this with the Arduino IDE you need to:
 *   1. Install the Air105 board package (once it is published)
 *   2. Select "Air105 Core Board" from Tools → Board
 *   3. Open this .ino file and click Upload
 *
 * For development before the board package is published, see the
 * PlatformIO test sample instead (tests/platformio-blinky/).
 *
 * Exercises:
 *   - Digital I/O   (pinMode, digitalWrite)
 *   - Timing        (millis, delay)
 *   - Serial UART   (Serial on UART1)
 *   - Analog input  (analogRead)
 *   - PWM output    (analogWrite)
 *   - Interrupts    (attachInterrupt)
 *
 * Expected hardware:
 *   - Air105 Core Board (OpenLuat)
 *   - On-board LED connected to LED_BUILTIN
 *   - Optional: potentiometer on A0, button on D2
 *
 * License: MIT (same as the PIO-Air105 project)
 */

// -----------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------
const uint8_t  BUTTON_PIN   = 2;            // External interrupt test
const uint8_t  PWM_PIN      = 3;            // PWM output test
const uint8_t  ANALOG_PIN   = A0;           // Analog input test pin
const uint32_t BLINK_MS     = 500;          // LED toggle interval
const uint32_t REPORT_MS    = 2000;         // Serial report interval

// -----------------------------------------------------------------------
// State
// -----------------------------------------------------------------------
volatile uint32_t buttonPresses = 0;
uint32_t lastBlink   = 0;
uint32_t lastReport  = 0;
bool     ledState    = false;
uint8_t  pwmDuty     = 0;
int8_t   pwmDir      = 1;

// -----------------------------------------------------------------------
// ISR
// -----------------------------------------------------------------------
void onButtonPress() {
  buttonPresses++;
}

// -----------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Wait for serial monitor (USB CDC boards)
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) { /* wait */ }

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F(" PIO-Air105 — Arduino Blinky Test"));
  Serial.println(F("========================================"));
  Serial.print(F(" F_CPU:       "));
  Serial.print(F_CPU / 1000000UL);
  Serial.println(F(" MHz"));
  Serial.print(F(" millis():    "));
  Serial.println(millis());
  Serial.println();

  // Digital I/O
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println(F("[OK] LED_BUILTIN configured"));

  // External interrupt
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonPress, FALLING);
  Serial.println(F("[OK] Button interrupt on pin 2"));

  // PWM
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  Serial.println(F("[OK] PWM on pin 3"));

  // ADC
  Serial.println(F("[OK] ADC ready (10-bit, 0-1.8V)"));

  Serial.println();
  Serial.println(F("Running... reports every 2 s."));
  Serial.println();
}

// -----------------------------------------------------------------------
// loop()
// -----------------------------------------------------------------------
void loop() {
  uint32_t now = millis();

  // LED blink
  if (now - lastBlink >= BLINK_MS) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
  }

  // PWM ramp
  analogWrite(PWM_PIN, pwmDuty);
  pwmDuty += pwmDir;
  if (pwmDuty == 255) pwmDir = -1;
  if (pwmDuty == 0)   pwmDir = +1;

  // Periodic report
  if (now - lastReport >= REPORT_MS) {
    lastReport = now;
    uint16_t adcVal = analogRead(ANALOG_PIN);

    Serial.print(F("["));
    Serial.print(now / 1000);
    Serial.print(F("s] LED="));
    Serial.print(ledState ? F("ON ") : F("OFF"));
    Serial.print(F("  PWM="));
    Serial.print(pwmDuty);
    Serial.print(F("  ADC="));
    Serial.print(adcVal);
    Serial.print(F("  BTN="));
    Serial.println(buttonPresses);
  }

  delay(5);
}
