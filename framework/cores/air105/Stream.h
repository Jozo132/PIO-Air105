/**
 * @file Stream.h
 * @brief Arduino Stream base class — extends Print with read APIs
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Minimal implementation for Phase 1.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef Stream_h
#define Stream_h

#include "Print.h"

class Stream : public Print {
public:
    Stream() : _timeout(1000) {}

    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;

    void setTimeout(unsigned long timeout) { _timeout = timeout; }
    unsigned long getTimeout(void) { return _timeout; }

    /* Find/parse helpers can be added later */
    size_t readBytes(char *buffer, size_t length);
    size_t readBytes(uint8_t *buffer, size_t length) {
        return readBytes((char *)buffer, length);
    }

protected:
    unsigned long _timeout;
    int timedRead();
    int timedPeek();
};

#endif /* Stream_h */
