/**
 * @file RingBuffer.h
 * @brief Lock-free single-producer single-consumer ring buffer for Serial RX
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 * SPDX-License-Identifier: MIT
 */
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>

#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE 256
#endif

#ifndef SERIAL_TX_BUFFER_SIZE
#define SERIAL_TX_BUFFER_SIZE 256
#endif

#if SERIAL_RX_BUFFER_SIZE < 2 || SERIAL_RX_BUFFER_SIZE > 65535
#error "SERIAL_RX_BUFFER_SIZE must be between 2 and 65535"
#endif

#if SERIAL_TX_BUFFER_SIZE < 2 || SERIAL_TX_BUFFER_SIZE > 65535
#error "SERIAL_TX_BUFFER_SIZE must be between 2 and 65535"
#endif

template <uint16_t BufferSize>
class RingBuffer {
public:
    RingBuffer() : _head(0), _tail(0) {}

    bool store(uint8_t c) {
        uint16_t next = (uint16_t)((_head + 1U) % BufferSize);
        if (next == _tail) return false;
        _buf[_head] = c;
        _head = next;
        return true;
    }

    int read() {
        if (_head == _tail) return -1;
        uint8_t c = _buf[_tail];
        _tail = (uint16_t)((_tail + 1U) % BufferSize);
        return c;
    }

    int peek() const {
        if (_head == _tail) return -1;
        return _buf[_tail];
    }

    int available() const {
        return (int)(((uint32_t)BufferSize + _head - _tail) % BufferSize);
    }

    uint16_t availableForStore() const {
        return (uint16_t)(BufferSize - 1U - (uint16_t)available());
    }

    void clear() { _head = _tail = 0; }

private:
    volatile uint16_t _head;
    volatile uint16_t _tail;
    uint8_t _buf[BufferSize];
};

#endif /* RING_BUFFER_H */
