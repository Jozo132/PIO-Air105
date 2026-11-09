#include <Arduino.h>

static const uint32_t transfer_size = 8192U;

static uint8_t testByte(uint32_t index)
{
    return (uint8_t)(index * 37U + (index >> 3));
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

static void runTransmitTest()
{
    for (uint32_t i = 0; i < transfer_size; ++i) {
        Serial.write(testByte(i));
    }
    Serial.flush();
    Serial.write((uint8_t)'t');
    Serial.flush();
}

static void runEchoTest()
{
    bool data_matches = true;
    for (uint32_t i = 0; i < transfer_size; ++i) {
        while (Serial.available() == 0) {
            yield();
        }

        int value = Serial.read();
        if (value != testByte(i)) {
            data_matches = false;
        }
        Serial.write((uint8_t)value);
    }

    Serial.flush();
    Serial.write((uint8_t)(data_matches ? 'e' : 'x'));
    write32(Serial.getRxOverflowCount());
    Serial.flush();
}

void setup()
{
    Serial.begin(115200);
    delay(5000);
    Serial.write((uint8_t)'R');
    Serial.flush();
}

void loop()
{
    int command = Serial.read();
    if (command == 'T') {
        runTransmitTest();
    } else if (command == 'E') {
        runEchoTest();
    }
}