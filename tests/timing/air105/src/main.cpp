#include <Arduino.h>
#include <HardwareTimer.h>

static volatile uint32_t timer_interrupt_count;
static uint32_t batch_microseconds[4];
static uint8_t batch_index;

static void backgroundTimerCallback()
{
    ++timer_interrupt_count;
}

static void write32(uint32_t value)
{
    uint8_t bytes[4] = {
        (uint8_t)value,
        (uint8_t)(value >> 8),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 24)
    };
    Serial.write(bytes, sizeof(bytes));
}

static void marker(char event, char id)
{
    Serial.write((uint8_t)(event == 'B' ? id : id + ('a' - 'A')));
    Serial.flush();
}

static void reportBatch(char id, uint32_t start_us)
{
    batch_microseconds[batch_index] = micros() - start_us;
    ++batch_index;
    marker('E', id);
}

static void runMicrosecondDelayBatch(char id, unsigned int delay_us, uint32_t count)
{
    marker('B', id);
    uint32_t start_us = micros();
    for (uint32_t i = 0; i < count; ++i) {
        delayMicroseconds(delay_us);
    }
    reportBatch(id, start_us);
}

static void runMillisecondDelayBatch(char id, unsigned long delay_ms, uint32_t count)
{
    marker('B', id);
    uint32_t start_us = micros();
    for (uint32_t i = 0; i < count; ++i) {
        delay(delay_ms);
    }
    reportBatch(id, start_us);
}

static bool checkTickBoundary()
{
    uint32_t phase;
    uint32_t start_us;
    uint32_t start_ms;
    uint32_t masked_delay_us;
    uint32_t masked_delay_ms_delta;
    uint32_t delay_serviced_ms_delta;
    do {
        start_us = micros();
        phase = start_us % 1000U;
    } while (phase < 990U);

    start_ms = millis();
    noInterrupts();
    delayMicroseconds(50);
    uint32_t elapsed_us = micros() - start_us;
    uint32_t masked_ms_delta = millis() - start_ms;
    interrupts();
    while (millis() == start_ms) {
    }
    uint32_t serviced_ms_delta = millis() - start_ms;

    do {
        start_us = micros();
        phase = start_us % 1000U;
    } while (phase > 20U);

    start_ms = millis();
    noInterrupts();
    delay(1);
    masked_delay_us = micros() - start_us;
    masked_delay_ms_delta = millis() - start_ms;
    interrupts();
    while (millis() == start_ms) {
    }
    delay_serviced_ms_delta = millis() - start_ms;

    HardwareTimer backgroundTimer(TIM6);
    backgroundTimer.setOverflow(1000U, MICROSEC_FORMAT);
    backgroundTimer.attachInterrupt(backgroundTimerCallback);
    backgroundTimer.resume();
    uint32_t ticks_before_delay = timer_interrupt_count;
    delay(10);
    bool timer_during_delay = timer_interrupt_count - ticks_before_delay >= 8U;
    uint32_t ticks_before_micro_delay = timer_interrupt_count;
    delayMicroseconds(10000U);
    bool timer_during_micro_delay =
        timer_interrupt_count - ticks_before_micro_delay >= 8U;
    backgroundTimer.pause();
    backgroundTimer.detachInterrupt();

    return (
        elapsed_us >= 40U && elapsed_us <= 100U
        && masked_ms_delta == 0U && serviced_ms_delta == 1U
        && masked_delay_us >= 900U && masked_delay_us <= 1100U
        && masked_delay_ms_delta == 0U && delay_serviced_ms_delta == 1U
        && timer_during_delay && timer_during_micro_delay
    );
}

void setup()
{
    Serial.begin(115200);
    delay(5000);
    Serial.write((uint8_t)'S');
    write32(F_CPU);
    write32(SystemCoreClock);
    Serial.flush();

    uint32_t previous = micros();
    bool monotonic = true;
    for (uint32_t i = 0; i < 1000U; ++i) {
        uint32_t current = micros();
        if ((int32_t)(current - previous) < 0) {
            monotonic = false;
            break;
        }
        previous = current;
    }
    Serial.write((uint8_t)(monotonic ? 'M' : 'm'));
    Serial.flush();

    runMicrosecondDelayBatch('A', 1U, 100000U);
    runMicrosecondDelayBatch('B', 100U, 10000U);
    runMillisecondDelayBatch('C', 1U, 100U);
    runMillisecondDelayBatch('D', 100U, 10U);
    bool timing_checks_passed = checkTickBoundary();
    Serial.write((uint8_t)(timing_checks_passed ? 'T' : 't'));
    Serial.flush();
    for (uint8_t i = 0; i < batch_index; ++i) {
        write32(batch_microseconds[i]);
    }
    Serial.flush();
}

void loop()
{
}
