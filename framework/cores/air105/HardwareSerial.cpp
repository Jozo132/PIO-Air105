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

    /* Determine UART ID for peripheral reset (UART0=0, UART1=1, etc.) */
    uint8_t uartId = 0;
    if (_uart == UART0) uartId = 0;
    else if (_uart == UART1) uartId = 1;
    else if (_uart == UART2) uartId = 2;
    else if (_uart == UART3) uartId = 3;

    /* Configure GPIO pins for this UART */
    _configurePins();

    /* Peripheral soft reset via SYSCTRL (vendor method) */
    SYSCTRL->SOFT_RST1 = (1 << uartId);
    while (SYSCTRL->SOFT_RST1 & (1 << uartId)) {}

    /* Set baud rate: divisor = (SystemCoreClock >> 6) / baud
     * Clock chain: PLL(204M) -> HCLK(102M) -> PCLK(51M) -> UART/16 = 3.1875M
     * SystemCoreClock >> 6 accounts for these divisions
     */
    uint32_t divisor = (SystemCoreClock >> 6) / baud;
    if (divisor == 0) divisor = 1;

    _uart->LCR |= UART_LCR_DLAB;      /* Enable divisor latch access */
    _uart->OFFSET_0.DLL = (divisor & 0xFF);
    _uart->OFFSET_4.DLH = ((divisor >> 8) & 0xFF);
    _uart->LCR &= ~UART_LCR_DLAB;     /* Disable DLAB */

    /* Set line control: data bits, parity, stop bits */
    _uart->LCR = config & 0x3F;       /* bits [5:0] of LCR */

    /* Enable and reset FIFOs */
    _uart->OFFSET_8.FCR = UART_FCR_FIFOE | UART_FCR_RFIFOR | UART_FCR_XFIFOR;

    /* Disable ALL interrupts - TX-only polling mode for now */
    _uart->OFFSET_4.IER = 0;
    
    /* Do NOT enable NVIC - no interrupts used */
    NVIC_DisableIRQ(_irqn);
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
    /* Always read LSR first to clear any error conditions */
    volatile uint32_t lsr = _uart->LSR;
    (void)lsr;
    
    /* Read IIR - this clears some interrupt sources */
    uint32_t iir = _uart->OFFSET_8.IIR;
    
    /* Check if interrupt is pending (bit 0 = 0 means interrupt pending) */
    if (iir & 0x01) {
        return;  /* No interrupt pending */
    }
    
    uint32_t int_id = (iir >> 1) & 0x07;
    
    switch (int_id) {
        case 0:  /* Modem status - read MSR to clear */
            (void)_uart->MSR;
            break;
            
        case 1:  /* TX holding register empty - ignore, we poll for TX */
            break;
            
        case 2:  /* Received data available */
        case 6:  /* Character timeout */
            while (_uart->USR & UART_USR_RFNE) {
                uint8_t c = (uint8_t)_uart->OFFSET_0.RBR;
                _rxBuf.store(c);
            }
            break;
            
        case 3:  /* Receiver line status (error) - already cleared by LSR read above */
            break;
            
        default:
            /* Unknown interrupt - read MSR to clear any pending */
            (void)_uart->MSR;
            break;
    }
}

/* ---- Pin configuration ---- */
void HardwareSerial::_configurePins()
{
    /*
     * GPIO alternate function setup for each UART.
     * Air105 UART pin assignments (from vendor io_map.h):
     *
     *   UART0: TX=PA1 AF0, RX=PA0 AF0  (download/debug, CH340)
     *   UART1: TX=PE6 AF0, RX=PE7 AF0  (default "Serial")
     *   UART2: TX=PD0 AF3, RX=PD1 AF3
     *   UART3: TX=PD2 AF3, RX=PD3 AF3
     *
     * Pin encoding: pin_num = (port * 16) + bit
     *   Port A = 0, Port B = 1, Port C = 2, Port D = 3, Port E = 4, Port F = 5
     */
    uint8_t tx_pin, rx_pin, af;

    if (_uart == UART0) {
        tx_pin = 1;       /* PA1 = 0*16+1 = 1 */
        rx_pin = 0;       /* PA0 = 0*16+0 = 0 */
        af = 0;
    } else if (_uart == UART1) {
        tx_pin = 4*16+6;  /* PE6 = 4*16+6 = 70 */
        rx_pin = 4*16+7;  /* PE7 = 4*16+7 = 71 */
        af = 0;
    } else if (_uart == UART2) {
        tx_pin = 3*16+0;  /* PD0 = 3*16+0 = 48 */
        rx_pin = 3*16+1;  /* PD1 = 3*16+1 = 49 */
        af = 3;
    } else if (_uart == UART3) {
        tx_pin = 3*16+2;  /* PD2 = 3*16+2 = 50 */
        rx_pin = 3*16+3;  /* PD3 = 3*16+3 = 51 */
        af = 3;
    } else {
        return;
    }

    /* GPIO_Iomux implementation (matches vendor core_gpio.c) */
    auto setIomux = [](uint8_t pin, uint8_t func) {
        uint8_t port = (pin >> 4);            /* port = pin / 16 */
        uint8_t bit = (pin & 0x0F);           /* bit within port */
        uint32_t mask = ~(0x03UL << (bit * 2));
        uint32_t val = (uint32_t)func << (bit * 2);
        GPIO->ALT[port] = (GPIO->ALT[port] & mask) | val;
    };

    setIomux(tx_pin, af);
    setIomux(rx_pin, af);
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
