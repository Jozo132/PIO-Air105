/**
 * @file HardwareSerial.h
 * @brief Arduino HardwareSerial for Air105 — UART0-UART3
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Air105 has 4 UARTs (DesignWare APB UART, 16-byte FIFO).
 * Default Arduino "Serial" maps to UART0 (connected to CH340 USB-to-serial).
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HardwareSerial_h
#define HardwareSerial_h

#include "Stream.h"
#include "RingBuffer.h"
#include "air105.h"

class HardwareSerial : public Stream {
public:
    /**
     * @param uart  Pointer to UART_TypeDef (UART0, UART1, UART2, UART3)
     * @param irqn  IRQ number for this UART
     */
    HardwareSerial(UART_TypeDef *uart, IRQn_Type irqn);

    void begin(unsigned long baud);
    void begin(unsigned long baud, uint8_t config);
    void end();

    virtual int available() override;
    int availableForWrite();
    virtual int peek() override;
    virtual int read() override;
    virtual void flush() override;

    uint32_t getRxOverflowCount() const { return _rxOverflowCount; }

    virtual size_t write(uint8_t c) override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;
    using Print::write; /* pull in write(str) etc. */

    operator bool() { return true; }

    /* Called from UART IRQ handler */
    void _rx_isr_handler();

private:
    UART_TypeDef *_uart;
    IRQn_Type     _irqn;
    RingBuffer<SERIAL_RX_BUFFER_SIZE> _rxBuf;
    RingBuffer<SERIAL_TX_BUFFER_SIZE> _txBuf;
    volatile uint32_t _rxOverflowCount;
    bool          _begun;

    void _configurePins();
    void _tx_isr_handler();
};

/* ---- Global Serial instances ---- */
extern HardwareSerial Serial;   /* UART0 (CH340 USB-to-serial) */
extern HardwareSerial Serial1;  /* UART1 */
extern HardwareSerial Serial2;  /* UART2 */
extern HardwareSerial Serial3;  /* UART3 */

#endif /* HardwareSerial_h */
