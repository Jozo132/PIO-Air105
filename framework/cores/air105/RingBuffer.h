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
#include <string.h>

#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE 256
#endif

class RingBuffer {
public:
    RingBuffer() : _head(0), _tail(0) {}

    /** Store one byte (called from ISR). Returns false if full. */
    bool store(uint8_t c) {
        uint16_t next = (uint16_t)((_head + 1) % SERIAL_RX_BUFFER_SIZE);
        if (next == _tail) return false; // full
        _buf[_head] = c;
        _head = next;
        return true;
    }

    /** Read one byte. Returns -1 if empty. */
    int read() {
        if (_head == _tail) return -1;
        uint8_t c = _buf[_tail];
        _tail = (uint16_t)((_tail + 1) % SERIAL_RX_BUFFER_SIZE);
        return c;
    }

    /** Peek at next byte without consuming. Returns -1 if empty. */
    int peek() const {
        if (_head == _tail) return -1;
        return _buf[_tail];
    }

    /** Number of bytes available */
    int available() const {
        return (int)(((unsigned int)SERIAL_RX_BUFFER_SIZE + _head - _tail)
                     % SERIAL_RX_BUFFER_SIZE);
    }

    /** Flush (discard) all data */
    void clear() { _head = _tail = 0; }

private:
    volatile uint16_t _head;
    volatile uint16_t _tail;
    uint8_t _buf[SERIAL_RX_BUFFER_SIZE];
};

#endif /* RING_BUFFER_H */
