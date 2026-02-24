/**
 * @file DMA.h
 * @brief Arduino DMA library for Air105 (MH1903S) — STM32duino-compatible API
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Air105 contains a DesignWare DMA controller with 8 independent channels
 * (DMA_Channel_0 – DMA_Channel_7).  Each channel can perform memory-to-memory,
 * memory-to-peripheral, or peripheral-to-memory transfers.
 *
 * Peripheral request mapping is done through SYSCTRL->DMA_CHAN (channels 0-3)
 * and SYSCTRL->DMA_CHAN1 (channels 4-7), using the DMA_REQUEST_xxx codes below.
 *
 * Key hardware parameters:
 *   - 8 channels, each with 0x58-byte register set
 *   - DMA base address:  0x40000800
 *   - Single shared IRQ: DMA_IRQn (IRQ 0)
 *   - Max block size per transfer: 4095 (12-bit CTL_H.BLOCK_TS)
 *   - DMA clock enabled via SYSCTRL->CG_CTRL2 |= SYSCTRL_AHBPeriph_DMA
 *     (already done in SystemInit)
 *
 * API Design:
 *   This library mirrors STM32duino / STM32 HAL DMA conventions:
 *   - DMA_HandleTypeDef: per-channel handle with configuration + state
 *   - DMA.begin():       global controller init (call once)
 *   - DMA.init():        configure a channel handle
 *   - DMA.start():       begin a polling transfer
 *   - DMA.startIT():     begin an interrupt-driven transfer
 *   - DMA.abort():       stop a channel
 *   - DMA.poll():        block until transfer completes (with timeout)
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef _DMA_H_INCLUDED_
#define _DMA_H_INCLUDED_

#include <stdint.h>
#include <stddef.h>
#include "air105.h"

/* ================================================================== */
/*  Constants                                                         */
/* ================================================================== */

/** Number of hardware DMA channels on the Air105 */
#define DMA_NUM_CHANNELS  8

/** Maximum block-transfer size in a single DMA transfer (12-bit field) */
#define DMA_MAX_BLOCK_SIZE  4095

/* ---- Transfer direction (matches STM32 HAL naming) ---- */
#define DMA_MEMORY_TO_MEMORY    0   /**< Memory → Memory */
#define DMA_MEMORY_TO_PERIPH    1   /**< Memory → Peripheral (TX) */
#define DMA_PERIPH_TO_MEMORY    2   /**< Peripheral → Memory (RX) */

/* ---- Data width (STM32 HAL naming) ---- */
#define DMA_PDATAALIGN_BYTE       0   /**< Peripheral data width: 8-bit */
#define DMA_PDATAALIGN_HALFWORD   1   /**< Peripheral data width: 16-bit */
#define DMA_PDATAALIGN_WORD       2   /**< Peripheral data width: 32-bit */

#define DMA_MDATAALIGN_BYTE       0   /**< Memory data width: 8-bit */
#define DMA_MDATAALIGN_HALFWORD   1   /**< Memory data width: 16-bit */
#define DMA_MDATAALIGN_WORD       2   /**< Memory data width: 32-bit */

/* ---- Address increment mode ---- */
#define DMA_PINC_ENABLE     0   /**< Peripheral address increments */
#define DMA_PINC_DISABLE    2   /**< Peripheral address fixed (no change) */

#define DMA_MINC_ENABLE     0   /**< Memory address increments */
#define DMA_MINC_DISABLE    2   /**< Memory address fixed (no change) */

/* ---- Burst transaction length ---- */
#define DMA_PBURST_SINGLE   0   /**< Peripheral: 1 data item per burst */
#define DMA_PBURST_INC4     1   /**< Peripheral: 4 data items per burst */
#define DMA_PBURST_INC8     2   /**< Peripheral: 8 data items per burst */
#define DMA_PBURST_INC16    3   /**< Peripheral: 16 data items per burst */

#define DMA_MBURST_SINGLE   0   /**< Memory: 1 data item per burst */
#define DMA_MBURST_INC4     1   /**< Memory: 4 data items per burst */
#define DMA_MBURST_INC8     2   /**< Memory: 8 data items per burst */
#define DMA_MBURST_INC16    3   /**< Memory: 16 data items per burst */

/* ---- Channel priority (CFG_L bits [7:5]) ---- */
#define DMA_PRIORITY_LOW       0x00   /**< Priority level 0 (lowest) */
#define DMA_PRIORITY_MEDIUM    0x20   /**< Priority level 1 */
#define DMA_PRIORITY_HIGH      0x40   /**< Priority level 2 */
#define DMA_PRIORITY_VERY_HIGH 0x60   /**< Priority level 3 (highest) */

/* ================================================================== */
/*  Peripheral Request Codes                                          */
/*  (written to SYSCTRL->DMA_CHAN / DMA_CHAN1 to select handshake)     */
/* ================================================================== */
#define DMA_REQUEST_MEM2MEM    0xFF   /**< Special: no peripheral (M2M) */

#define DMA_REQUEST_DCMI_TX    0x00   /**< DCMI transmit */
#define DMA_REQUEST_LCD        0x01   /**< LCD controller */
#define DMA_REQUEST_UART0_TX   0x02   /**< UART0 transmit */
#define DMA_REQUEST_UART0_RX   0x03   /**< UART0 receive */
#define DMA_REQUEST_UART1_TX   0x04   /**< UART1 transmit */
#define DMA_REQUEST_UART1_RX   0x05   /**< UART1 receive */
#define DMA_REQUEST_DAC        0x06   /**< DAC output */
#define DMA_REQUEST_SPI0_TX    0x0A   /**< SPI0 transmit */
#define DMA_REQUEST_SPI0_RX    0x0B   /**< SPI0 receive */
#define DMA_REQUEST_SPI1_TX    0x0C   /**< SPI1 transmit */
#define DMA_REQUEST_SPI1_RX    0x0D   /**< SPI1 receive */
#define DMA_REQUEST_SPI2_TX    0x0E   /**< SPI2 transmit */
#define DMA_REQUEST_SPI2_RX    0x0F   /**< SPI2 receive */
#define DMA_REQUEST_UART2_TX   0x14   /**< UART2 transmit */
#define DMA_REQUEST_UART2_RX   0x15   /**< UART2 receive */
#define DMA_REQUEST_UART3_TX   0x16   /**< UART3 transmit */
#define DMA_REQUEST_UART3_RX   0x17   /**< UART3 receive */
#define DMA_REQUEST_I2C_TX     0x18   /**< I2C transmit */
#define DMA_REQUEST_I2C_RX     0x19   /**< I2C receive */
#define DMA_REQUEST_QSPI_TX    0x1A   /**< QSPI transmit */
#define DMA_REQUEST_HSPI_RX    0x20   /**< HSPI receive */
#define DMA_REQUEST_HSPI_TX    0x21   /**< HSPI transmit */

/* ================================================================== */
/*  DMA handle state (matches STM32 HAL pattern)                      */
/* ================================================================== */
typedef enum {
    DMA_STATE_RESET   = 0x00,   /**< Not yet initialized / released */
    DMA_STATE_READY   = 0x01,   /**< Initialized, idle */
    DMA_STATE_BUSY    = 0x02,   /**< Transfer in progress */
    DMA_STATE_ERROR   = 0x03,   /**< Error occurred */
    DMA_STATE_ABORT   = 0x04    /**< Transfer aborted */
} DMA_State_t;

/* ================================================================== */
/*  DMA Init structure (matches STM32 HAL Init sub-struct)            */
/* ================================================================== */
typedef struct {
    uint32_t Request;           /**< Peripheral request code (DMA_REQUEST_xxx) */
    uint32_t Direction;         /**< Transfer direction (DMA_MEMORY_TO_xxx) */
    uint32_t PeriphInc;         /**< Peripheral increment mode (DMA_PINC_xxx) */
    uint32_t MemInc;            /**< Memory increment mode (DMA_MINC_xxx) */
    uint32_t PeriphDataAlign;   /**< Peripheral data alignment (DMA_PDATAALIGN_xxx) */
    uint32_t MemDataAlign;      /**< Memory data alignment (DMA_MDATAALIGN_xxx) */
    uint32_t PeriphBurst;       /**< Peripheral burst size (DMA_PBURST_xxx) */
    uint32_t MemBurst;          /**< Memory burst size (DMA_MBURST_xxx) */
    uint32_t Priority;          /**< Channel priority (DMA_PRIORITY_xxx) */
} DMA_InitDef;

/* ================================================================== */
/*  Callback type                                                     */
/* ================================================================== */

/**
 * @brief DMA completion/error callback signature
 * @param channel  Channel index (0-7) that completed
 * @param isError  true if the transfer ended due to an error
 * @param userData Opaque pointer passed during attachInterrupt
 */
typedef void (*dma_callback_t)(uint8_t channel, bool isError, void *userData);

/* ================================================================== */
/*  DMA Handle (one per channel, matches STM32 HAL DMA_HandleTypeDef) */
/* ================================================================== */
typedef struct {
    DMA_TypeDef      *Instance;         /**< Channel registers (DMA_Channel_x) */
    DMA_InitDef       Init;             /**< Configuration snapshot */
    uint8_t           channelIndex;     /**< 0-7 */
    volatile DMA_State_t State;         /**< Current state */
    dma_callback_t    XferCpltCallback;      /**< Transfer-complete callback */
    dma_callback_t    XferErrorCallback;     /**< Transfer-error callback */
    void             *callbackUserData;      /**< Opaque user pointer for callbacks */
} DMA_HandleTypeDef;

/* ================================================================== */
/*  DMAClass  (Arduino-facing singleton)                              */
/* ================================================================== */

class DMAClass {
public:
    /* ---- Global init / deinit ---- */

    /**
     * @brief Initialize the DMA controller (enable global DMA, set up IRQ).
     *
     * Safe to call multiple times; will only init once.
     * SystemInit() already enables SYSCTRL_AHBPeriph_DMA; this function
     * additionally enables the DMA engine and sets up the shared IRQ.
     */
    void begin();

    /**
     * @brief Disable the DMA controller and IRQ.
     */
    void end();

    /* ---- Channel management ---- */

    /**
     * @brief Allocate the first free DMA channel.
     * @return Channel index (0-7), or -1 if all channels are in use.
     */
    int8_t allocateChannel();

    /**
     * @brief Allocate a specific DMA channel.
     * @param ch Desired channel index (0-7).
     * @return ch on success, -1 if already allocated or invalid.
     */
    int8_t allocateChannel(uint8_t ch);

    /**
     * @brief Release a previously allocated channel.
     * @param ch Channel index (0-7).
     *
     * Also aborts any in-progress transfer and resets state.
     */
    void freeChannel(uint8_t ch);

    /**
     * @brief Check whether a channel is currently allocated.
     * @param ch Channel index (0-7).
     * @return true if allocated.
     */
    bool isChannelAllocated(uint8_t ch) const;

    /**
     * @brief Get a pointer to the handle for a channel.
     * @param ch Channel index (0-7).
     * @return Pointer to the internal DMA_HandleTypeDef, or nullptr if invalid.
     */
    DMA_HandleTypeDef *handle(uint8_t ch);

    /* ---- Configuration ---- */

    /**
     * @brief Initialize a DMA channel from its handle's Init struct.
     * @param hdma  Pointer to a configured DMA_HandleTypeDef.
     * @return true on success.
     *
     * The handle must already have:
     *   - hdma->channelIndex set (via allocateChannel)
     *   - hdma->Init fields populated
     *
     * Or more conveniently, use the internal handle from handle(ch) and
     * populate its Init, then call init(handle(ch)).
     */
    bool init(DMA_HandleTypeDef *hdma);

    /**
     * @brief Configure a channel with all parameters in one call.
     * @param ch           Channel index (0-7) — must be allocated first.
     * @param request      Peripheral request (DMA_REQUEST_xxx).
     * @param direction    Transfer direction (DMA_MEMORY_TO_xxx).
     * @param periphInc    Peripheral address increment (DMA_PINC_xxx).
     * @param memInc       Memory address increment (DMA_MINC_xxx).
     * @param periphAlign  Peripheral data width (DMA_PDATAALIGN_xxx).
     * @param memAlign     Memory data width (DMA_MDATAALIGN_xxx).
     * @param priority     Priority (DMA_PRIORITY_xxx).
     * @return true on success.
     */
    bool configure(uint8_t ch,
                   uint32_t request,
                   uint32_t direction,
                   uint32_t periphInc   = DMA_PINC_DISABLE,
                   uint32_t memInc      = DMA_MINC_ENABLE,
                   uint32_t periphAlign = DMA_PDATAALIGN_BYTE,
                   uint32_t memAlign    = DMA_MDATAALIGN_BYTE,
                   uint32_t priority    = DMA_PRIORITY_LOW);

    /* ---- Transfer start (polling) ---- */

    /**
     * @brief Start a DMA transfer (polling mode — no interrupt).
     * @param hdma   Handle (must be inited).
     * @param srcAddr  Source address.
     * @param dstAddr  Destination address.
     * @param dataLen  Number of data items to transfer (not bytes if width > 8).
     * @return true if the transfer was started successfully.
     */
    bool start(DMA_HandleTypeDef *hdma,
               uint32_t srcAddr, uint32_t dstAddr, uint32_t dataLen);

    /**
     * @brief Start a DMA peripheral transfer (direction determines address roles).
     * @param ch         Channel index (0-7).
     * @param periphAddr Peripheral register address (e.g. &SPI->DR).
     * @param memAddr    Memory buffer address.
     * @param dataLen    Number of data items.
     * @return true on success.
     */
    bool start(uint8_t ch, uint32_t periphAddr, void *memAddr, uint32_t dataLen);

    /* ---- Transfer start (interrupt) ---- */

    /**
     * @brief Start a DMA transfer with completion interrupt.
     * @param hdma     Handle (must be inited, callback attached).
     * @param srcAddr  Source address.
     * @param dstAddr  Destination address.
     * @param dataLen  Number of data items.
     * @return true if started.
     */
    bool startIT(DMA_HandleTypeDef *hdma,
                 uint32_t srcAddr, uint32_t dstAddr, uint32_t dataLen);

    /**
     * @brief Start an interrupt-driven peripheral transfer.
     * @param ch         Channel index (0-7).
     * @param periphAddr Peripheral register address.
     * @param memAddr    Memory buffer address.
     * @param dataLen    Number of data items.
     * @return true on success.
     */
    bool startIT(uint8_t ch, uint32_t periphAddr, void *memAddr, uint32_t dataLen);

    /* ---- Memory-to-memory convenience ---- */

    /**
     * @brief Blocking memory-to-memory copy via DMA.
     * @param dst  Destination pointer.
     * @param src  Source pointer.
     * @param len  Number of bytes to copy.
     * @return true on success.
     *
     * Internally allocates a free channel, performs the transfer, and releases it.
     */
    bool memcpy(void *dst, const void *src, uint32_t len);

    /* ---- Control ---- */

    /**
     * @brief Abort an ongoing transfer on the given handle.
     * @param hdma Handle.
     */
    void abort(DMA_HandleTypeDef *hdma);

    /**
     * @brief Abort an ongoing transfer on channel ch.
     * @param ch Channel index (0-7).
     */
    void abort(uint8_t ch);

    /**
     * @brief Poll for transfer completion (blocking).
     * @param hdma      Handle.
     * @param timeoutMs Timeout in milliseconds (0 = infinite).
     * @return true if completed without error, false on timeout or error.
     */
    bool poll(DMA_HandleTypeDef *hdma, uint32_t timeoutMs = 0);

    /**
     * @brief Poll for transfer completion on channel ch.
     * @param ch        Channel index (0-7).
     * @param timeoutMs Timeout in milliseconds (0 = infinite).
     * @return true if completed OK.
     */
    bool poll(uint8_t ch, uint32_t timeoutMs = 0);

    /* ---- Status ---- */

    /**
     * @brief Check if a channel is currently busy with a transfer.
     * @param ch Channel index (0-7).
     * @return true if busy.
     */
    bool isBusy(uint8_t ch) const;

    /**
     * @brief Get current state of a channel.
     * @param ch Channel index (0-7).
     * @return DMA_State_t enum value.
     */
    DMA_State_t getState(uint8_t ch) const;

    /**
     * @brief Get number of remaining data items for a channel.
     * @param ch Channel index (0-7).
     * @return Item count still to be transferred (from CTL_H.BLOCK_TS).
     */
    uint32_t getTransferCount(uint8_t ch) const;

    /* ---- Interrupt callbacks ---- */

    /**
     * @brief Attach a transfer-complete callback to a channel.
     * @param ch       Channel index (0-7).
     * @param cb       Callback function.
     * @param userData Opaque pointer forwarded to callback.
     */
    void attachInterrupt(uint8_t ch, dma_callback_t cb, void *userData = nullptr);

    /**
     * @brief Detach the callback from a channel.
     * @param ch Channel index (0-7).
     */
    void detachInterrupt(uint8_t ch);

    /* ---- Internal — called from DMA_IRQHandler ---- */
    void _irqHandler();

private:
    bool             _initialized;
    uint8_t          _allocated;    /* bitmask: bit N = channel N allocated */
    DMA_HandleTypeDef _handles[DMA_NUM_CHANNELS];

    /* Low-level helpers */
    void _clearChannelFlags(uint8_t ch);
    void _disableChannel(uint8_t ch);
    void _enableChannel(uint8_t ch);
    void _setPeriphMux(uint8_t ch, uint32_t periphCode);
    bool _startTransfer(DMA_HandleTypeDef *hdma,
                        uint32_t srcAddr, uint32_t dstAddr,
                        uint32_t dataLen, bool enableIRQ);
};

/* ================================================================== */
/*  Global singleton                                                  */
/* ================================================================== */
extern DMAClass DMAControl;

#endif /* _DMA_H_INCLUDED_ */
