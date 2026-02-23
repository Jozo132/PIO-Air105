/**
 * @file HardwareSerial.cpp
 * @brief Arduino HardwareSerial implementation for Air105
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * DesignWare APB UART register-level driver.
 * Baud divisor = (SystemCoreClock >> 6) / BaudRate
 *   (SystemCoreClock = PLL, >>6 accounts for HCLK/2 + PCLK/2 + UART 16× oversample)
 *
 * SPDX-License-Identifier: MIT
 */

#include "HardwareSerial.h"
#include "Arduino.h"

/* ---- UART line config constants (same encoding as LCR register) ---- */
#define SERIAL_8N1  0x03   /* 8 data, no parity, 1 stop */
#define SERIAL_8E1  0x1B   /* 8 data, even parity, 1 stop */
#define SERIAL_8O1  0x0B   /* 8 data, odd parity, 1 stop */
#define SERIAL_8N2  0x07   /* 8 data, no parity, 2 stop */

/* ---- Instances ---- */
HardwareSerial Serial(UART1, UART1_IRQn);
HardwareSerial Serial0(UART0, UART0_IRQn);
HardwareSerial Serial2(UART2, UART2_IRQn);
HardwareSerial Serial3(UART3, UART3_IRQn);

/* ---- Constructor ---- */
HardwareSerial::HardwareSerial(UART_TypeDef *uart, IRQn_Type irqn)
    : _uart(uart), _irqn(irqn), _begun(false) {}

/* ---- begin() ---- */
void HardwareSerial::begin(unsigned long baud)
{
    begin(baud, SERIAL_8N1);
}

void HardwareSerial::begin(unsigned long baud, uint8_t config)
{
    _begun = true;

    /* Configure GPIO pins for this UART */
    _configurePins();

    /* Reset UART */
    _uart->SRR = UART_SRR_UR | UART_SRR_RFR | UART_SRR_XFR;

    /* Wait for reset to complete */
    while (_uart->USR & UART_USR_BUSY) {}

    /* Set baud rate: divisor = (SystemCoreClock >> 6) / baud */
    uint32_t divisor = (SystemCoreClock >> 6) / baud;
    if (divisor == 0) divisor = 1;

    _uart->LCR = UART_LCR_DLAB;       /* Enable divisor latch access */
    _uart->OFFSET_0.DLL = divisor & 0xFF;
    _uart->OFFSET_4.DLH = (divisor >> 8) & 0xFF;
    _uart->LCR = 0;                   /* Disable DLAB */

    /* Set line control: data bits, parity, stop bits */
    _uart->LCR = config & 0x3F;       /* bits [5:0] of LCR */

    /* Enable and reset FIFOs, set RX trigger to 1 char */
    _uart->OFFSET_8.FCR = UART_FCR_FIFOE | UART_FCR_RFIFOR | UART_FCR_XFIFOR;

    /* Enable RX interrupt */
    _uart->OFFSET_4.IER = UART_IER_ERBFI | UART_IER_ELSI;

    /* Enable NVIC */
    NVIC_SetPriority(_irqn, 2);
    NVIC_EnableIRQ(_irqn);
}

void HardwareSerial::end()
{
    NVIC_DisableIRQ(_irqn);
    _uart->OFFSET_4.IER = 0;
    _begun = false;
    _rxBuf.clear();
}

/* ---- Stream/Print interface ---- */

int HardwareSerial::available()
{
    return _rxBuf.available();
}

int HardwareSerial::peek()
{
    return _rxBuf.peek();
}

int HardwareSerial::read()
{
    return _rxBuf.read();
}

void HardwareSerial::flush()
{
    /* Wait until transmit FIFO and shift register are empty */
    while (!(_uart->LSR & UART_LSR_TEMT)) {}
}

size_t HardwareSerial::write(uint8_t c)
{
    if (!_begun) return 0;

    /* Wait for TX FIFO not full */
    while (!(_uart->USR & UART_USR_TFNF)) {}
    _uart->OFFSET_0.THR = c;
    return 1;
}

size_t HardwareSerial::write(const uint8_t *buffer, size_t size)
{
    if (!_begun) return 0;
    size_t n = 0;
    while (n < size) {
        while (!(_uart->USR & UART_USR_TFNF)) {}
        _uart->OFFSET_0.THR = buffer[n++];
    }
    return n;
}

/* ---- RX interrupt handler (called from UART IRQ) ---- */
void HardwareSerial::_rx_isr_handler()
{
    uint32_t iir = _uart->OFFSET_8.IIR & 0x0F;

    /* Process receive data available or character timeout */
    if (iir == 0x04 || iir == 0x0C) {
        while (_uart->USR & UART_USR_RFNE) {
            uint8_t c = (uint8_t)_uart->OFFSET_0.RBR;
            _rxBuf.store(c);
        }
    }

    /* Line status error — read LSR to clear */
    if (iir == 0x06) {
        (void)_uart->LSR;
    }
}

/* ---- Pin configuration ---- */
void HardwareSerial::_configurePins()
{
    /*
     * GPIO alternate function setup for each UART.
     * Air105 UART pin assignments (from datasheet / board schematic):
     *
     *   UART0: TX=PA1 AF0, RX=PA0 AF0  (download/debug)
     *   UART1: TX=PE6 AF0, RX=PE7 AF0  (default "Serial")
     *   UART2: TX=PD0 AF3, RX=PD1 AF3
     *   UART3: TX=PD2 AF3, RX=PD3 AF3
     *
     * NOTE: These are typical assignments for the LuatOS Air105 dev board.
     * Boards with different UART routing should override in the variant.
     */
    uint8_t tx_pin, rx_pin, af;

    if (_uart == UART0) {
        tx_pin = 1;  /* PA1 */ rx_pin = 0;  /* PA0 */ af = 0;
    } else if (_uart == UART1) {
        tx_pin = 4*16+6; /* PE6 */ rx_pin = 4*16+7; /* PE7 */ af = 0;
    } else if (_uart == UART2) {
        tx_pin = 3*16+0; /* PD0 */ rx_pin = 3*16+1; /* PD1 */ af = 3;
    } else if (_uart == UART3) {
        tx_pin = 3*16+2; /* PD2 */ rx_pin = 3*16+3; /* PD3 */ af = 3;
    } else {
        return;
    }

    /* Set alternate function for TX and RX pins */
    auto setAF = [](uint8_t pin, uint8_t func) {
        uint8_t port = pin >> 4;
        uint8_t bit  = pin & 0x0F;
        uint32_t shift = bit * 2;
        GPIO_ALT_GROUP[port] = (GPIO_ALT_GROUP[port] & ~(3UL << shift))
                               | ((uint32_t)func << shift);
    };

    setAF(tx_pin, af);
    setAF(rx_pin, af);
}

/* ---- C IRQ handlers dispatching to C++ objects ---- */
extern "C" {

void UART0_IRQHandler(void)
{
    Serial0._rx_isr_handler();
}

void UART1_IRQHandler(void)
{
    Serial._rx_isr_handler();
}

void UART2_IRQHandler(void)
{
    Serial2._rx_isr_handler();
}

void UART3_IRQHandler(void)
{
    Serial3._rx_isr_handler();
}

} /* extern "C" */
