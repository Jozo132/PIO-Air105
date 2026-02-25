/**
 * @file Wire.h
 * @brief Arduino TwoWire (I2C) for Air105 — STM32duino-compatible API
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Air105 has one I2C peripheral (I2C0, DesignWare APB I2C).
 *   - Default pins: PE6 = SDA, PE7 = SCL (AF2)
 *   - Speeds: 100 kHz (standard) and 400 kHz (fast mode)
 *
 * NOTE: I2C0 shares PE6/PE7 with UART1 (Serial).
 * You CANNOT use Wire and Serial at the same time.
 * Use Serial0/Serial2/Serial3 instead when using I2C.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef TwoWire_h
#define TwoWire_h

#include "Stream.h"
#include "air105.h"

#define WIRE_BUFFER_LENGTH  32

/* I2C speed presets */
#define I2C_SPEED_STANDARD  100000UL
#define I2C_SPEED_FAST      400000UL

class TwoWire : public Stream {
public:
    TwoWire(I2C_TypeDef *i2c);

    /* ---- Arduino Wire API ---- */
    void begin();                                       /* Master mode */
    void begin(uint8_t address);                        /* Slave mode (not supported, placeholder) */
    void begin(int address) { begin((uint8_t)address); }
    void end();

    void setClock(uint32_t frequency);

    void beginTransmission(uint8_t address);
    void beginTransmission(int address) { beginTransmission((uint8_t)address); }
    uint8_t endTransmission(bool sendStop = true);

    uint8_t requestFrom(uint8_t address, uint8_t quantity, bool sendStop = true);
    uint8_t requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop);
    uint8_t requestFrom(int address, int quantity) {
        return requestFrom((uint8_t)address, (uint8_t)quantity, true);
    }
    uint8_t requestFrom(int address, int quantity, int sendStop) {
        return requestFrom((uint8_t)address, (uint8_t)quantity, (bool)sendStop);
    }

    virtual size_t write(uint8_t data) override;
    virtual size_t write(const uint8_t *data, size_t quantity) override;
    using Print::write;

    virtual int available() override;
    virtual int read() override;
    virtual int peek() override;
    virtual void flush() override;

    /* STM32duino-compatible callbacks */
    void onReceive(void (*callback)(int));
    void onRequest(void (*callback)(void));

    operator bool() { return _begun; }

    /* Wire error codes (return from endTransmission) */
    static const uint8_t SUCCESS          = 0;
    static const uint8_t NACK_ADDR        = 2;
    static const uint8_t NACK_DATA        = 3;
    static const uint8_t OTHER_ERROR      = 4;
    static const uint8_t TIMEOUT          = 5;

private:
    I2C_TypeDef *_i2c;
    bool         _begun;
    uint32_t     _clockSpeed;

    /* TX buffer (for beginTransmission/write/endTransmission) */
    uint8_t  _txBuf[WIRE_BUFFER_LENGTH];
    uint8_t  _txLen;
    uint8_t  _txAddr;

    /* RX buffer (for requestFrom/available/read) */
    uint8_t  _rxBuf[WIRE_BUFFER_LENGTH];
    uint8_t  _rxLen;
    uint8_t  _rxIdx;

    /* Callbacks (slave mode, placeholders) */
    void (*_onReceiveCb)(int);
    void (*_onRequestCb)(void);

    /* Internal helpers */
    void _configurePins();
    void _configureSpeed(uint32_t speed);
    void _disable();
    void _enable();
    bool _waitTxNotFull(uint32_t timeoutMs = 100);
    bool _waitRxNotEmpty(uint32_t timeoutMs = 100);
    bool _waitIdle(uint32_t timeoutMs = 100);
};

extern TwoWire Wire;

#endif /* TwoWire_h */
