#include "../../cores/air105/RingBuffer.h"

#ifndef RING_BUFFER_CUSTOM_SIZE_TEST
static_assert(SERIAL_RX_BUFFER_SIZE == 256, "default RX ring size changed");
static_assert(SERIAL_TX_BUFFER_SIZE == 256, "default TX ring size changed");
#endif

template <uint16_t BufferSize>
static bool testRingBuffer()
{
    RingBuffer<BufferSize> buffer;
    const uint16_t usable_size = BufferSize - 1U;

    if (buffer.available() != 0 || buffer.availableForStore() != usable_size) {
        return false;
    }

    for (uint16_t i = 0; i < usable_size; ++i) {
        if (!buffer.store((uint8_t)i)) {
            return false;
        }
    }

    if (buffer.store(0xFF) || buffer.available() != usable_size
        || buffer.availableForStore() != 0 || buffer.peek() != 0) {
        return false;
    }

    for (uint16_t i = 0; i < usable_size; ++i) {
        if (buffer.read() != (uint8_t)i) {
            return false;
        }
    }

    return buffer.read() == -1 && buffer.available() == 0
        && buffer.availableForStore() == usable_size;
}

int main()
{
    return testRingBuffer<SERIAL_RX_BUFFER_SIZE>()
        && testRingBuffer<SERIAL_TX_BUFFER_SIZE>() ? 0 : 1;
}