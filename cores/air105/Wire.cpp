/**
 * @file Wire.cpp
 * @brief Arduino TwoWire (I2C) implementation for Air105
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Direct register-level driver for the DesignWare I2C controller.
 * Master-mode only, polling-based (no IRQ / DMA).
 *
 * I2C0 pins: PE6 = SDA, PE7 = SCL  (AF2)
 *
 * Clock formula (from vendor core_i2c.c):
 *   I2C peripheral clock = SystemCoreClock / 8   (APB div chain)
 *   Cnt = (SystemCoreClock >> 3) / Speed
 *   SDA_HOLD = SDA_SETUP = Cnt >> 2
 *   SCL_HCNT = Cnt - FS_SPKLEN
 *   SCL_LCNT = Cnt
 *
 * SPDX-License-Identifier: MIT
 */

#include "Wire.h"
#include "Arduino.h"

/* ---- Timeout helper ---- */
static inline bool _timedOut(uint32_t startMs, uint32_t timeoutMs) {
    return (millis() - startMs) >= timeoutMs;
}

/* ---- Instance ---- */
TwoWire Wire(I2C0);

/* ---- Constructor ---- */
TwoWire::TwoWire(I2C_TypeDef *i2c)
    : _i2c(i2c), _begun(false), _clockSpeed(I2C_SPEED_STANDARD),
      _txLen(0), _txAddr(0),
      _rxLen(0), _rxIdx(0),
      _onReceiveCb(nullptr), _onRequestCb(nullptr)
{}

/* ---- begin (master) ---- */
void TwoWire::begin()
{
    _begun = true;

    /* Configure GPIO pins for I2C (PE6=SDA, PE7=SCL, AF2) */
    _configurePins();

    /* Soft-reset the I2C0 peripheral via SYSCTRL
     * SYSCTRL_APBPeriph_I2C0 = 0x00040000 = bit 18
     */
    SYSCTRL->SOFT_RST1 = SYSCTRL_APBPeriph_I2C0;
    while (SYSCTRL->SOFT_RST1 & SYSCTRL_APBPeriph_I2C0) {}

    /* Configure speed registers */
    _configureSpeed(_clockSpeed);
}

/* ---- begin (slave) — placeholder, not supported ---- */
void TwoWire::begin(uint8_t address)
{
    /* Slave mode is not implemented for Air105.
     * Fall back to master mode. */
    (void)address;
    begin();
}

/* ---- end ---- */
void TwoWire::end()
{
    _disable();
    _begun = false;
    _txLen = 0;
    _rxLen = 0;
    _rxIdx = 0;
}

/* ---- setClock ---- */
void TwoWire::setClock(uint32_t frequency)
{
    _clockSpeed = frequency;
    if (_begun) {
        _configureSpeed(frequency);
    }
}

/* ---- beginTransmission ---- */
void TwoWire::beginTransmission(uint8_t address)
{
    _txAddr = address;
    _txLen  = 0;
}

/* ---- endTransmission ---- */
uint8_t TwoWire::endTransmission(bool sendStop)
{
    if (!_begun) return OTHER_ERROR;

    /* Disable I2C to change target address */
    _disable();

    /* Set target address (7-bit) */
    _i2c->IC_TAR = (uint32_t)_txAddr & I2C_IC_TAR_TAR;

    /* Enable I2C */
    _enable();

    /* Send all buffered bytes */
    for (uint8_t i = 0; i < _txLen; i++) {
        uint32_t cmd = (uint32_t)_txBuf[i]; /* Write command (CMD bit = 0) */

        /* Last byte: optionally set STOP bit */
        if (i == (_txLen - 1) && sendStop) {
            cmd |= I2C_IC_DATA_CMD_STOP;
        }

        /* Wait for TX FIFO not full */
        if (!_waitTxNotFull()) {
            _txLen = 0;
            return TIMEOUT;
        }

        _i2c->IC_DATA_CMD = cmd;
    }

    /* If sendStop, wait for transfer to complete (controller becomes idle) */
    if (sendStop) {
        if (!_waitIdle()) {
            _txLen = 0;
            return TIMEOUT;
        }
    }

    /* Check for TX abort */
    uint32_t abrt = _i2c->IC_TX_ABRT_SOURCE;
    if (abrt) {
        /* Clear abort by reading IC_CLR_TX_ABRT */
        (void)_i2c->IC_CLR_TX_ABRT;
        _txLen = 0;

        if (abrt & I2C_IC_TX_ABRT_SOURCE_7B_ADDR_NOACK) {
            return NACK_ADDR;
        }
        if (abrt & I2C_IC_TX_ABRT_SOURCE_TXDATA_NOACK) {
            return NACK_DATA;
        }
        return OTHER_ERROR;
    }

    _txLen = 0;
    return SUCCESS;
}

/* ---- requestFrom ---- */
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, bool sendStop)
{
    if (!_begun) return 0;
    if (quantity == 0) return 0;
    if (quantity > WIRE_BUFFER_LENGTH) quantity = WIRE_BUFFER_LENGTH;

    _rxLen = 0;
    _rxIdx = 0;

    /* Disable I2C to change target address */
    _disable();

    /* Set target address (7-bit) */
    _i2c->IC_TAR = (uint32_t)address & I2C_IC_TAR_TAR;

    /* Enable I2C */
    _enable();

    /* Issue read commands and collect data */
    uint8_t rxCount = 0;
    for (uint8_t i = 0; i < quantity; i++) {
        /* Build read command (CMD bit = 1 = read) */
        uint32_t cmd = I2C_IC_DATA_CMD_CMD;

        /* Last byte: optionally set STOP bit */
        if (i == (quantity - 1) && sendStop) {
            cmd |= I2C_IC_DATA_CMD_STOP;
        }

        /* Wait for TX FIFO not full (read commands go through TX FIFO) */
        if (!_waitTxNotFull()) break;

        _i2c->IC_DATA_CMD = cmd;

        /* Wait for RX data */
        if (!_waitRxNotEmpty()) break;

        _rxBuf[rxCount++] = (uint8_t)(_i2c->IC_DATA_CMD & 0xFF);
    }

    _rxLen = rxCount;
    _rxIdx = 0;

    return rxCount;
}

/* ---- requestFrom with internal address ---- */
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity,
                              uint32_t iaddress, uint8_t isize, uint8_t sendStop)
{
    if (!_begun) return 0;

    /* Send internal address first (register address) */
    if (isize > 0) {
        beginTransmission(address);
        /* Send MSB first for multi-byte addresses */
        if (isize > 3) write((uint8_t)(iaddress >> 24));
        if (isize > 2) write((uint8_t)(iaddress >> 16));
        if (isize > 1) write((uint8_t)(iaddress >> 8));
        write((uint8_t)(iaddress & 0xFF));
        endTransmission(false); /* Repeated start (no stop) */
    }

    return requestFrom(address, quantity, (bool)sendStop);
}

/* ---- write (single byte) ---- */
size_t TwoWire::write(uint8_t data)
{
    if (_txLen >= WIRE_BUFFER_LENGTH) return 0;
    _txBuf[_txLen++] = data;
    return 1;
}

/* ---- write (buffer) ---- */
size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
    size_t n = 0;
    for (size_t i = 0; i < quantity; i++) {
        if (write(data[i]) == 0) break;
        n++;
    }
    return n;
}

/* ---- available / read / peek / flush ---- */
int TwoWire::available()
{
    return _rxLen - _rxIdx;
}

int TwoWire::read()
{
    if (_rxIdx >= _rxLen) return -1;
    return _rxBuf[_rxIdx++];
}

int TwoWire::peek()
{
    if (_rxIdx >= _rxLen) return -1;
    return _rxBuf[_rxIdx];
}

void TwoWire::flush()
{
    /* Nothing to flush — polling mode */
}

/* ---- Callbacks (slave mode placeholders) ---- */
void TwoWire::onReceive(void (*callback)(int))
{
    _onReceiveCb = callback;
}

void TwoWire::onRequest(void (*callback)(void))
{
    _onRequestCb = callback;
}

/* ==================================================================== */
/* Internal helpers                                                      */
/* ==================================================================== */

/* ---- Pin mux: PE6 = SDA (AF2), PE7 = SCL (AF2) ---- */
void TwoWire::_configurePins()
{
    auto setIomux = [](uint8_t pin, uint8_t func) {
        uint8_t port = (pin >> 4);
        uint8_t bit  = (pin & 0x0F);
        uint32_t mask = ~(0x03UL << (bit * 2));
        uint32_t val  = (uint32_t)func << (bit * 2);
        GPIO->ALT[port] = (GPIO->ALT[port] & mask) | val;
    };

    /* PE6 = 4*16+6 = 70  → AF2 (I2C0 SDA)
     * PE7 = 4*16+7 = 71  → AF2 (I2C0 SCL) */
    setIomux(70, 2);  /* PE6 AF2 = I2C0 SDA */
    setIomux(71, 2);  /* PE7 AF2 = I2C0 SCL */
}

/* ---- Configure I2C speed registers ---- */
void TwoWire::_configureSpeed(uint32_t speed)
{
    /* I2C peripheral clock = SystemCoreClock / 8 */
    uint32_t cnt = (SystemCoreClock >> 3) / speed;

    _disable();

    _i2c->IC_SDA_HOLD  = cnt >> 2;
    _i2c->IC_SDA_SETUP = cnt >> 2;

    if (speed <= 100000) {
        /* Standard mode (100 kHz) */
        _i2c->IC_SS_SCL_HCNT = cnt - _i2c->IC_FS_SPKLEN;
        _i2c->IC_SS_SCL_LCNT = cnt;
        _i2c->IC_CON = I2C_IC_CON_RESTART_EN
                      | I2C_IC_CON_SPEED_0
                      | I2C_IC_CON_MASTER_MODE
                      | I2C_IC_CON_SLAVE_DISABLE;
    } else {
        /* Fast mode (400 kHz) */
        _i2c->IC_FS_SCL_HCNT = cnt - _i2c->IC_FS_SPKLEN;
        _i2c->IC_FS_SCL_LCNT = cnt;
        _i2c->IC_CON = I2C_IC_CON_RESTART_EN
                      | I2C_IC_CON_SPEED_1
                      | I2C_IC_CON_MASTER_MODE
                      | I2C_IC_CON_SLAVE_DISABLE;
    }

    _enable();

    /* No interrupts — polling mode */
    _i2c->IC_RX_TL = 0;
    _i2c->IC_TX_TL = 0;
    _i2c->IC_INTR_MASK = 0;
}

/* ---- Disable I2C controller ---- */
void TwoWire::_disable()
{
    _i2c->IC_ENABLE = 0;
    /* Wait until actually disabled */
    uint32_t start = millis();
    while (_i2c->IC_ENABLE_STATUS & I2C_IC_ENABLE_STATUS_IC_EN) {
        if (_timedOut(start, 10)) break;
    }
}

/* ---- Enable I2C controller ---- */
void TwoWire::_enable()
{
    _i2c->IC_ENABLE = I2C_IC_ENABLE_ENABLE;
}

/* ---- Wait for TX FIFO not full ---- */
bool TwoWire::_waitTxNotFull(uint32_t timeoutMs)
{
    uint32_t start = millis();
    while (!(_i2c->IC_STATUS & I2C_IC_STATUS_TFNF)) {
        if (_timedOut(start, timeoutMs)) return false;
        /* Check for abort */
        if (_i2c->IC_RAW_INTR_STAT & I2C_IC_RAW_INTR_STAT_TX_ABRT) return false;
    }
    return true;
}

/* ---- Wait for RX FIFO not empty ---- */
bool TwoWire::_waitRxNotEmpty(uint32_t timeoutMs)
{
    uint32_t start = millis();
    while (!(_i2c->IC_STATUS & I2C_IC_STATUS_RFNE)) {
        if (_timedOut(start, timeoutMs)) return false;
        /* Check for abort */
        if (_i2c->IC_RAW_INTR_STAT & I2C_IC_RAW_INTR_STAT_TX_ABRT) return false;
    }
    return true;
}

/* ---- Wait for bus idle (no master activity) ---- */
bool TwoWire::_waitIdle(uint32_t timeoutMs)
{
    uint32_t start = millis();
    while (_i2c->IC_STATUS & I2C_IC_STATUS_MST_ACTIVITY) {
        if (_timedOut(start, timeoutMs)) return false;
    }
    /* Also wait for TX FIFO to be completely empty */
    while (!(_i2c->IC_STATUS & I2C_IC_STATUS_TFE)) {
        if (_timedOut(start, timeoutMs)) return false;
    }
    return true;
}
