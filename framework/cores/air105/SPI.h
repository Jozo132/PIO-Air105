/**
 * @file SPI.h
 * @brief Arduino SPI library for Air105 — STM32duino-compatible API
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Air105 has three standard SPI master peripherals (DesignWare SSI):
 *   - SPIM0 (SPI0): PB12=SCK, PB13=CS, PB14=MOSI, PB15=MISO (AF0)
 *   - SPIM1 (SPI1): PA6=SCK,  PA7=CS,  PA8=MOSI,  PA9=MISO  (AF3)
 *   - SPIM2 (SPI2): PB2=SCK,  PB3=CS,  PB4=MOSI,  PB5=MISO  (AF0)
 *
 * Additionally HSPI (High-Speed SPI, different register layout) exists
 * but is NOT supported by this Arduino-compatible class.
 *
 * Default instance `SPI` maps to SPIM2 (PB2-PB5), matching variant.h.
 *
 * Clock formula:
 *   PCLK = SystemCoreClock / 4
 *   Actual SPI clock = PCLK / BAUDR   (BAUDR must be even, >= 2)
 *
 * CS Management:
 *   The library sets the SS pin to GPIO mode so the hardware chip-select
 *   does NOT automatically drive it.  The user controls CS manually via
 *   digitalWrite() — standard Arduino practice.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef _SPI_H_INCLUDED_
#define _SPI_H_INCLUDED_

#include <stdint.h>
#include <stddef.h>
#include "air105.h"

/* ---- Arduino SPI mode constants ---- */
#define SPI_MODE0  0x00  /* CPOL=0, CPHA=0 */
#define SPI_MODE1  0x01  /* CPOL=0, CPHA=1 */
#define SPI_MODE2  0x02  /* CPOL=1, CPHA=0 */
#define SPI_MODE3  0x03  /* CPOL=1, CPHA=1 */

/* ---- Bit ordering ---- */
#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

/* ---- Legacy clock-divider presets ---- */
#define SPI_CLOCK_DIV2    2
#define SPI_CLOCK_DIV4    4
#define SPI_CLOCK_DIV8    8
#define SPI_CLOCK_DIV16   16
#define SPI_CLOCK_DIV32   32
#define SPI_CLOCK_DIV64   64
#define SPI_CLOCK_DIV128  128
#define SPI_CLOCK_DIV256  256

/* ---- STM32duino transfer mode (for pin-parameterized API) ---- */
enum SPITransferMode {
    SPI_CONTINUE = 0,   /* Keep CS asserted after transfer */
    SPI_LAST     = 1    /* Deassert CS after transfer */
};

/* ---- CS pin sentinel ---- */
#define CS_PIN_CONTROLLED_BY_USER  0xFF

/* ================================================================== */
/*  SPISettings                                                       */
/* ================================================================== */

class SPISettings {
public:
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
        : _clock(clock), _bitOrder(bitOrder), _dataMode(dataMode) {}

    SPISettings()
        : _clock(4000000), _bitOrder(MSBFIRST), _dataMode(SPI_MODE0) {}

private:
    uint32_t _clock;
    uint8_t  _bitOrder;
    uint8_t  _dataMode;
    friend class SPIClass;
};

/* ================================================================== */
/*  SPIClass                                                          */
/* ================================================================== */

class SPIClass {
public:
    /**
     * @param spi      Pointer to SPI_TypeDef (SPIM0, SPIM1, or SPIM2)
     * @param sckPin   SCK  pin number (variant.h encoding: port*16+bit)
     * @param misoPin  MISO pin number
     * @param mosiPin  MOSI pin number
     * @param ssPin    SS/CS pin number (switched to GPIO, user-controlled)
     * @param afValue  Alternate-function value for pin mux (0 or 3)
     */
    SPIClass(SPI_TypeDef *spi,
             uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin, uint8_t ssPin,
             uint8_t afValue);

    /* ---- Standard Arduino SPI API ---- */
    void begin();
    void begin(uint32_t ssPin);                               /* STM32duino: begin with specific SS pin */
    void end();

    void beginTransaction(SPISettings settings);
    void beginTransaction(uint32_t ssPin, SPISettings settings); /* STM32duino: with pin */
    void endTransaction();
    void endTransaction(uint32_t ssPin);                         /* STM32duino: with pin */

    uint8_t  transfer(uint8_t data);
    uint16_t transfer16(uint16_t data);
    void     transfer(void *buf, size_t count);

    /* STM32duino pin-parameterized transfer API */
    uint8_t  transfer(uint32_t ssPin, uint8_t data, SPITransferMode mode = SPI_LAST);
    uint16_t transfer16(uint32_t ssPin, uint16_t data, SPITransferMode mode = SPI_LAST);
    void     transfer(uint32_t ssPin, void *buf, size_t count, SPITransferMode mode = SPI_LAST);

    /* ---- STM32duino pin reconfiguration ---- */
    void setMISO(uint32_t miso);
    void setMOSI(uint32_t mosi);
    void setSCLK(uint32_t sclk);
    void setSSEL(uint32_t ssel);

    /* ---- Legacy API (pre-1.6) ---- */
    void setBitOrder(uint8_t bitOrder);
    void setDataMode(uint8_t dataMode);
    void setClockDivider(uint8_t clockDiv);

    /* ---- Interrupt guarding (stubs — Air105 is bare-metal) ---- */
    void usingInterrupt(int interruptNumber);
    void notUsingInterrupt(int interruptNumber);

    /* ---- Deprecated (Arduino 0023 era, stubs for compat) ---- */
    void attachInterrupt();
    void detachInterrupt();

    operator bool() { return _begun; }

private:
    SPI_TypeDef *_spi;
    uint8_t  _sckPin;
    uint8_t  _misoPin;
    uint8_t  _mosiPin;
    uint8_t  _ssPin;
    uint8_t  _afValue;      /* AF number for pin mux */
    uint8_t  _bitOrder;
    uint8_t  _dataMode;
    uint32_t _clockFreq;
    bool     _begun;

    /* Internal helpers */
    void    _setIomux(uint8_t pin, uint8_t func);
    void    _configurePins();
    void    _applyConfig(uint32_t clock, uint8_t dataMode);
    uint8_t _reverseByte(uint8_t b);
};

/* ---- Global instances ---- */
extern SPIClass SPI;    /* SPIM2 — default (PB2/PB3/PB4/PB5) */

#endif /* _SPI_H_INCLUDED_ */
