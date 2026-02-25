/**
 * @file DMA.cpp
 * @brief Arduino DMA implementation for Air105 (MH1903S)
 * @author J.Vovk <Jozo132@gmail.com>
 * @url https://github.com/Jozo132/PIO-Air105
 *
 * Direct register-level driver for the DesignWare DMA controller.
 * 8 channels, shared DMA_IRQn (IRQ 0).
 *
 * Register layout (per channel, 0x58-byte stride):
 *   SAR_L/H     — Source address
 *   DAR_L/H     — Destination address
 *   LLP_L/H     — Linked-list pointer (unused, single-block only)
 *   CTL_L/H     — Control (direction, width, burst, block size)
 *   SSTAT/DSTAT  — Status (unused)
 *   SSTATAR/DSTATAR — Status address (unused)
 *   CFG_L/H     — Configuration (priority, handshake interface)
 *   SGR_L/H     — Scatter (unused)
 *   DSR_L/H     — Gather (unused)
 *
 * Global registers (DMA_MODULE_TypeDef at DMA_BASE + 0x2C0):
 *   Raw/Status/Mask/Clear for Tfr, Block, SrcTran, DstTran, Err
 *   DmaCfgReg   — Global DMA enable
 *   ChEnReg     — Channel enable/writeenable
 *
 * Peripheral handshake routing (SYSCTRL->DMA_CHAN / DMA_CHAN1):
 *   CH0-CH3: DMA_CHAN  — 6-bit peripheral code, positions 0/8/16/24
 *   CH4-CH7: DMA_CHAN1 — 6-bit peripheral code, positions 0/8/16/24
 *
 * SPDX-License-Identifier: MIT
 */

#include "DMA.h"
#include "Arduino.h"

/* ================================================================== */
/*  Global instance                                                   */
/* ================================================================== */
DMAClass DMAControl;

/* ================================================================== */
/*  DesignWare DMA CTL_L register bit definitions                     */
/* ================================================================== */
#define CTL_INT_EN              (1U << 0)

#define CTL_DST_TR_WIDTH_POS    1
#define CTL_DST_TR_WIDTH_MASK   (0x07U << CTL_DST_TR_WIDTH_POS)

#define CTL_SRC_TR_WIDTH_POS    4
#define CTL_SRC_TR_WIDTH_MASK   (0x07U << CTL_SRC_TR_WIDTH_POS)

#define CTL_DINC_POS            7
#define CTL_DINC_MASK           (0x03U << CTL_DINC_POS)

#define CTL_SINC_POS            9
#define CTL_SINC_MASK           (0x03U << CTL_SINC_POS)

#define CTL_DEST_MSIZE_POS      11
#define CTL_DEST_MSIZE_MASK     (0x07U << CTL_DEST_MSIZE_POS)

#define CTL_SRC_MSIZE_POS       14
#define CTL_SRC_MSIZE_MASK      (0x07U << CTL_SRC_MSIZE_POS)

#define CTL_TT_FC_POS           20
#define CTL_TT_FC_MASK          (0x07U << CTL_TT_FC_POS)
#define CTL_TT_FC_M2M           (0x00U << CTL_TT_FC_POS)
#define CTL_TT_FC_M2P           (0x01U << CTL_TT_FC_POS)
#define CTL_TT_FC_P2M           (0x02U << CTL_TT_FC_POS)

/* CTL_H */
#define CTL_BLOCK_TS_POS        0
#define CTL_BLOCK_TS_MASK       (0x0FFFU << CTL_BLOCK_TS_POS)

/* CFG_L register */
#define CFG_HS_SEL_DST_POS      10
#define CFG_HS_SEL_SRC_POS      11

/* CFG_H register */
#define CFG_SRC_PER_POS         7
#define CFG_SRC_PER_MASK        (0x0FU << CFG_SRC_PER_POS)
#define CFG_DEST_PER_POS        11
#define CFG_DEST_PER_MASK       (0x0FU << CFG_DEST_PER_POS)

/* ChEnReg_L helpers */
#define CH_EN_BIT(ch)           (1U << (ch))
#define CH_WE_BIT(ch)           (1U << ((ch) + 8))

/* ================================================================== */
/*  DMA_IRQHandler (C linkage — overrides weak default in startup)    */
/* ================================================================== */
extern "C" void DMA_IRQHandler(void)
{
    DMAControl._irqHandler();
}

/* ================================================================== */
/*  Global init / deinit                                              */
/* ================================================================== */

void DMAClass::begin()
{
    if (_initialized) return;

    /* Zero-fill all handles */
    for (uint8_t i = 0; i < DMA_NUM_CHANNELS; i++) {
        _handles[i] = DMA_HandleTypeDef{};
        _handles[i].channelIndex = i;
        _handles[i].State = DMA_STATE_RESET;
    }
    _allocated = 0;

    /* Ensure DMA AHB clock is enabled (SystemInit already does this) */
    SYSCTRL->CG_CTRL2 |= SYSCTRL_AHBPeriph_DMA;

    /* Enable global DMA controller */
    DMA->DmaCfgReg_L = 1;

    /* Disable all channels */
    DMA->ChEnReg_L = 0x0000FF00;   /* WE bits set, EN bits clear */

    /* Clear all interrupt flags */
    DMA->ClearTfr_L     = 0xFF;
    DMA->ClearBlock_L   = 0xFF;
    DMA->ClearSrcTran_L = 0xFF;
    DMA->ClearDstTran_L = 0xFF;
    DMA->ClearErr_L     = 0xFF;

    /* Unmask Tfr-complete and Err interrupts for all channels */
    DMA->MaskTfr_L = 0x0000FFFF;   /* [15:8]=WE, [7:0]=EN — all 8 channels */
    DMA->MaskErr_L = 0x0000FFFF;

    /* Configure NVIC */
    NVIC_SetPriority(DMA_IRQn, 3);
    NVIC_EnableIRQ(DMA_IRQn);

    _initialized = true;
}

void DMAClass::end()
{
    if (!_initialized) return;

    NVIC_DisableIRQ(DMA_IRQn);

    /* Disable all channels */
    DMA->ChEnReg_L = 0x0000FF00;

    /* Clear all interrupt flags */
    DMA->ClearTfr_L     = 0xFF;
    DMA->ClearBlock_L   = 0xFF;
    DMA->ClearSrcTran_L = 0xFF;
    DMA->ClearDstTran_L = 0xFF;
    DMA->ClearErr_L     = 0xFF;

    /* Mask all interrupts */
    DMA->MaskTfr_L = 0x0000FF00;
    DMA->MaskErr_L = 0x0000FF00;

    /* Disable global DMA */
    DMA->DmaCfgReg_L = 0;

    /* Reset all handles */
    for (uint8_t i = 0; i < DMA_NUM_CHANNELS; i++) {
        _handles[i].State = DMA_STATE_RESET;
        _handles[i].XferCpltCallback  = nullptr;
        _handles[i].XferErrorCallback = nullptr;
    }
    _allocated   = 0;
    _initialized = false;
}

/* ================================================================== */
/*  Channel management                                                */
/* ================================================================== */

int8_t DMAClass::allocateChannel()
{
    if (!_initialized) begin();

    for (uint8_t i = 0; i < DMA_NUM_CHANNELS; i++) {
        if (!(_allocated & (1U << i))) {
            _allocated |= (1U << i);
            _handles[i].Instance     = (DMA_TypeDef *)((uint32_t)DMA_Channel_0 + 0x58 * i);
            _handles[i].channelIndex = i;
            _handles[i].State        = DMA_STATE_RESET;
            return (int8_t)i;
        }
    }
    return -1;
}

int8_t DMAClass::allocateChannel(uint8_t ch)
{
    if (ch >= DMA_NUM_CHANNELS) return -1;
    if (!_initialized) begin();

    if (_allocated & (1U << ch)) return -1;   /* already taken */

    _allocated |= (1U << ch);
    _handles[ch].Instance     = (DMA_TypeDef *)((uint32_t)DMA_Channel_0 + 0x58 * ch);
    _handles[ch].channelIndex = ch;
    _handles[ch].State        = DMA_STATE_RESET;
    return (int8_t)ch;
}

void DMAClass::freeChannel(uint8_t ch)
{
    if (ch >= DMA_NUM_CHANNELS) return;

    /* Abort any in-flight transfer */
    abort(ch);

    _handles[ch].State             = DMA_STATE_RESET;
    _handles[ch].XferCpltCallback  = nullptr;
    _handles[ch].XferErrorCallback = nullptr;
    _handles[ch].callbackUserData  = nullptr;
    _allocated &= ~(1U << ch);
}

bool DMAClass::isChannelAllocated(uint8_t ch) const
{
    if (ch >= DMA_NUM_CHANNELS) return false;
    return (_allocated & (1U << ch)) != 0;
}

DMA_HandleTypeDef *DMAClass::handle(uint8_t ch)
{
    if (ch >= DMA_NUM_CHANNELS) return nullptr;
    return &_handles[ch];
}

/* ================================================================== */
/*  Configuration                                                     */
/* ================================================================== */

bool DMAClass::init(DMA_HandleTypeDef *hdma)
{
    if (!hdma) return false;
    uint8_t ch = hdma->channelIndex;
    if (ch >= DMA_NUM_CHANNELS) return false;
    if (!(_allocated & (1U << ch))) return false;

    DMA_TypeDef *reg = hdma->Instance;
    uint32_t chBit = CH_EN_BIT(ch);

    /* Make sure channel is not running */
    if (DMA->ChEnReg_L & chBit) {
        _disableChannel(ch);
    }

    /* Clear all pending flags for this channel */
    _clearChannelFlags(ch);

    /* Build CTL_L based on direction */
    uint32_t ctl_l = 0;
    const DMA_InitDef *cfg = &hdma->Init;

    if (cfg->Direction == DMA_MEMORY_TO_PERIPH) {
        /* Source = memory, Destination = peripheral */
        ctl_l = CTL_TT_FC_M2P
               | (cfg->MemBurst      << CTL_SRC_MSIZE_POS)
               | (cfg->PeriphBurst   << CTL_DEST_MSIZE_POS)
               | (cfg->MemInc        << CTL_SINC_POS)
               | (cfg->PeriphInc     << CTL_DINC_POS)
               | (cfg->MemDataAlign  << CTL_SRC_TR_WIDTH_POS)
               | (cfg->PeriphDataAlign << CTL_DST_TR_WIDTH_POS);
    } else if (cfg->Direction == DMA_PERIPH_TO_MEMORY) {
        /* Source = peripheral, Destination = memory */
        ctl_l = CTL_TT_FC_P2M
               | (cfg->PeriphBurst   << CTL_SRC_MSIZE_POS)
               | (cfg->MemBurst      << CTL_DEST_MSIZE_POS)
               | (cfg->PeriphInc     << CTL_SINC_POS)
               | (cfg->MemInc        << CTL_DINC_POS)
               | (cfg->PeriphDataAlign << CTL_SRC_TR_WIDTH_POS)
               | (cfg->MemDataAlign  << CTL_DST_TR_WIDTH_POS);
    } else {
        /* Memory-to-memory */
        ctl_l = CTL_TT_FC_M2M
               | (cfg->MemBurst  << CTL_SRC_MSIZE_POS)
               | (cfg->MemBurst  << CTL_DEST_MSIZE_POS)
               | (DMA_MINC_ENABLE << CTL_SINC_POS)
               | (DMA_MINC_ENABLE << CTL_DINC_POS)
               | (cfg->MemDataAlign << CTL_SRC_TR_WIDTH_POS)
               | (cfg->MemDataAlign << CTL_DST_TR_WIDTH_POS);
    }

    reg->CTL_L = ctl_l;
    reg->CTL_H = 0;                /* block size set at start time */
    reg->LLP_L = 0;                /* no linked list / single-block */
    reg->LLP_H = 0;

    /* CFG_L: priority + handshake selection */
    if (cfg->Direction == DMA_MEMORY_TO_MEMORY) {
        /* M2M: software handshake on both sides */
        reg->CFG_L = cfg->Priority
                    | (1U << CFG_HS_SEL_SRC_POS)
                    | (1U << CFG_HS_SEL_DST_POS);
    } else {
        /* Peripheral transfer: hardware handshake */
        reg->CFG_L = cfg->Priority;   /* HS_SEL = 0 → hardware */
    }

    /* CFG_H: map handshake interface to channel number */
    reg->CFG_H = (ch << CFG_SRC_PER_POS) | (ch << CFG_DEST_PER_POS);

    /* Set SYSCTRL peripheral mapping (only for non-M2M) */
    if (cfg->Request != DMA_REQUEST_MEM2MEM) {
        _setPeriphMux(ch, cfg->Request);
    }

    hdma->State = DMA_STATE_READY;
    return true;
}

bool DMAClass::configure(uint8_t ch,
                         uint32_t request,
                         uint32_t direction,
                         uint32_t periphInc,
                         uint32_t memInc,
                         uint32_t periphAlign,
                         uint32_t memAlign,
                         uint32_t priority)
{
    if (ch >= DMA_NUM_CHANNELS) return false;
    if (!(_allocated & (1U << ch))) return false;

    DMA_HandleTypeDef *hdma = &_handles[ch];
    hdma->Init.Request        = request;
    hdma->Init.Direction      = direction;
    hdma->Init.PeriphInc      = periphInc;
    hdma->Init.MemInc         = memInc;
    hdma->Init.PeriphDataAlign = periphAlign;
    hdma->Init.MemDataAlign   = memAlign;
    hdma->Init.PeriphBurst    = DMA_PBURST_SINGLE;
    hdma->Init.MemBurst       = DMA_MBURST_SINGLE;
    hdma->Init.Priority       = priority;

    return init(hdma);
}

/* ================================================================== */
/*  Transfer start (polling)                                          */
/* ================================================================== */

bool DMAClass::start(DMA_HandleTypeDef *hdma,
                     uint32_t srcAddr, uint32_t dstAddr, uint32_t dataLen)
{
    return _startTransfer(hdma, srcAddr, dstAddr, dataLen, false);
}

bool DMAClass::start(uint8_t ch, uint32_t periphAddr, void *memAddr, uint32_t dataLen)
{
    if (ch >= DMA_NUM_CHANNELS) return false;
    DMA_HandleTypeDef *hdma = &_handles[ch];
    if (hdma->State != DMA_STATE_READY) return false;

    uint32_t src, dst;
    if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH) {
        src = (uint32_t)memAddr;
        dst = periphAddr;
    } else if (hdma->Init.Direction == DMA_PERIPH_TO_MEMORY) {
        src = periphAddr;
        dst = (uint32_t)memAddr;
    } else {
        /* M2M: periphAddr = src, memAddr = dst */
        src = periphAddr;
        dst = (uint32_t)memAddr;
    }
    return _startTransfer(hdma, src, dst, dataLen, false);
}

/* ================================================================== */
/*  Transfer start (interrupt)                                        */
/* ================================================================== */

bool DMAClass::startIT(DMA_HandleTypeDef *hdma,
                       uint32_t srcAddr, uint32_t dstAddr, uint32_t dataLen)
{
    return _startTransfer(hdma, srcAddr, dstAddr, dataLen, true);
}

bool DMAClass::startIT(uint8_t ch, uint32_t periphAddr, void *memAddr, uint32_t dataLen)
{
    if (ch >= DMA_NUM_CHANNELS) return false;
    DMA_HandleTypeDef *hdma = &_handles[ch];
    if (hdma->State != DMA_STATE_READY) return false;

    uint32_t src, dst;
    if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH) {
        src = (uint32_t)memAddr;
        dst = periphAddr;
    } else if (hdma->Init.Direction == DMA_PERIPH_TO_MEMORY) {
        src = periphAddr;
        dst = (uint32_t)memAddr;
    } else {
        src = periphAddr;
        dst = (uint32_t)memAddr;
    }
    return _startTransfer(hdma, src, dst, dataLen, true);
}

/* ================================================================== */
/*  Memory-to-memory convenience                                      */
/* ================================================================== */

bool DMAClass::memcpy(void *dst, const void *src, uint32_t len)
{
    if (!dst || !src || len == 0) return false;

    /* Try to use word-width if both addresses and length are 4-aligned */
    uint32_t dataAlign = DMA_MDATAALIGN_BYTE;
    uint32_t items     = len;
    if (((uint32_t)dst & 3) == 0 && ((uint32_t)src & 3) == 0 && (len & 3) == 0) {
        dataAlign = DMA_MDATAALIGN_WORD;
        items     = len >> 2;
    } else if (((uint32_t)dst & 1) == 0 && ((uint32_t)src & 1) == 0 && (len & 1) == 0) {
        dataAlign = DMA_MDATAALIGN_HALFWORD;
        items     = len >> 1;
    }

    int8_t ch = allocateChannel();
    if (ch < 0) return false;

    DMA_HandleTypeDef *hdma = &_handles[ch];
    hdma->Init.Request        = DMA_REQUEST_MEM2MEM;
    hdma->Init.Direction      = DMA_MEMORY_TO_MEMORY;
    hdma->Init.PeriphInc      = DMA_PINC_ENABLE;
    hdma->Init.MemInc         = DMA_MINC_ENABLE;
    hdma->Init.PeriphDataAlign = dataAlign;
    hdma->Init.MemDataAlign   = dataAlign;
    hdma->Init.PeriphBurst    = DMA_PBURST_SINGLE;
    hdma->Init.MemBurst       = DMA_MBURST_SINGLE;
    hdma->Init.Priority       = DMA_PRIORITY_LOW;

    bool ok = init(hdma);
    if (ok) {
        ok = start(hdma, (uint32_t)src, (uint32_t)dst, items);
        if (ok) {
            ok = poll(hdma, 1000);   /* 1-second timeout */
        }
    }

    freeChannel(ch);
    return ok;
}

/* ================================================================== */
/*  Control                                                           */
/* ================================================================== */

void DMAClass::abort(DMA_HandleTypeDef *hdma)
{
    if (!hdma) return;
    uint8_t ch = hdma->channelIndex;
    if (ch >= DMA_NUM_CHANNELS) return;

    _disableChannel(ch);
    _clearChannelFlags(ch);
    hdma->State = DMA_STATE_READY;
}

void DMAClass::abort(uint8_t ch)
{
    if (ch >= DMA_NUM_CHANNELS) return;
    abort(&_handles[ch]);
}

bool DMAClass::poll(DMA_HandleTypeDef *hdma, uint32_t timeoutMs)
{
    if (!hdma) return false;
    uint8_t ch     = hdma->channelIndex;
    uint32_t chBit = CH_EN_BIT(ch);
    uint32_t start_ms = millis();

    while (DMA->ChEnReg_L & chBit) {
        /* Check for error */
        if (DMA->RawErr_L & chBit) {
            DMA->ClearErr_L = chBit;
            _disableChannel(ch);
            _clearChannelFlags(ch);
            hdma->State = DMA_STATE_ERROR;
            return false;
        }
        /* Timeout check */
        if (timeoutMs > 0 && (millis() - start_ms) >= timeoutMs) {
            _disableChannel(ch);
            _clearChannelFlags(ch);
            hdma->State = DMA_STATE_ERROR;
            return false;
        }
    }

    /* Transfer complete — clear flags */
    _clearChannelFlags(ch);
    hdma->State = DMA_STATE_READY;
    return true;
}

bool DMAClass::poll(uint8_t ch, uint32_t timeoutMs)
{
    if (ch >= DMA_NUM_CHANNELS) return false;
    return poll(&_handles[ch], timeoutMs);
}

/* ================================================================== */
/*  Status                                                            */
/* ================================================================== */

bool DMAClass::isBusy(uint8_t ch) const
{
    if (ch >= DMA_NUM_CHANNELS) return false;
    return (DMA->ChEnReg_L & CH_EN_BIT(ch)) != 0;
}

DMA_State_t DMAClass::getState(uint8_t ch) const
{
    if (ch >= DMA_NUM_CHANNELS) return DMA_STATE_RESET;
    return _handles[ch].State;
}

uint32_t DMAClass::getTransferCount(uint8_t ch) const
{
    if (ch >= DMA_NUM_CHANNELS) return 0;
    DMA_TypeDef *reg = _handles[ch].Instance;
    if (!reg) return 0;
    return (reg->CTL_H & CTL_BLOCK_TS_MASK);
}

/* ================================================================== */
/*  Interrupt callbacks                                               */
/* ================================================================== */

void DMAClass::attachInterrupt(uint8_t ch, dma_callback_t cb, void *userData)
{
    if (ch >= DMA_NUM_CHANNELS) return;
    _handles[ch].XferCpltCallback  = cb;
    _handles[ch].XferErrorCallback = cb;     /* same callback, isError flag differs */
    _handles[ch].callbackUserData  = userData;
}

void DMAClass::detachInterrupt(uint8_t ch)
{
    if (ch >= DMA_NUM_CHANNELS) return;
    _handles[ch].XferCpltCallback  = nullptr;
    _handles[ch].XferErrorCallback = nullptr;
    _handles[ch].callbackUserData  = nullptr;
}

/* ================================================================== */
/*  IRQ handler                                                       */
/* ================================================================== */

void DMAClass::_irqHandler()
{
    uint32_t statusTfr = DMA->StatusTfr_L;
    uint32_t statusErr = DMA->StatusErr_L;

    for (uint8_t i = 0; i < DMA_NUM_CHANNELS; i++) {
        uint32_t chBit = (1U << i);

        /* Transfer-complete interrupt */
        if (statusTfr & chBit) {
            DMA->ClearTfr_L = chBit;
            _handles[i].State = DMA_STATE_READY;
            if (_handles[i].XferCpltCallback) {
                _handles[i].XferCpltCallback(i, false, _handles[i].callbackUserData);
            }
        }

        /* Error interrupt */
        if (statusErr & chBit) {
            DMA->ClearErr_L = chBit;
            _handles[i].State = DMA_STATE_ERROR;
            if (_handles[i].XferErrorCallback) {
                _handles[i].XferErrorCallback(i, true, _handles[i].callbackUserData);
            }
        }
    }

    /* Clear block / src / dst transaction flags (not used for callbacks) */
    DMA->ClearBlock_L   = 0xFF;
    DMA->ClearSrcTran_L = 0xFF;
    DMA->ClearDstTran_L = 0xFF;
}

/* ================================================================== */
/*  Private helpers                                                   */
/* ================================================================== */

void DMAClass::_clearChannelFlags(uint8_t ch)
{
    uint32_t chBit = CH_EN_BIT(ch);
    DMA->ClearTfr_L     = chBit;
    DMA->ClearBlock_L   = chBit;
    DMA->ClearSrcTran_L = chBit;
    DMA->ClearDstTran_L = chBit;
    DMA->ClearErr_L     = chBit;
}

void DMAClass::_disableChannel(uint8_t ch)
{
    /* Write WE bit without EN → disables the channel */
    DMA->ChEnReg_L = CH_WE_BIT(ch);

    /* Spin until hardware confirms disabled */
    while (DMA->ChEnReg_L & CH_EN_BIT(ch)) { /* wait */ }
}

void DMAClass::_enableChannel(uint8_t ch)
{
    /* Write both WE and EN bits → enables the channel */
    DMA->ChEnReg_L = CH_WE_BIT(ch) | CH_EN_BIT(ch);
}

void DMAClass::_setPeriphMux(uint8_t ch, uint32_t periphCode)
{
    /*
     * SYSCTRL->DMA_CHAN  : channels 0-3, each 8-bit field at pos 0/8/16/24
     * SYSCTRL->DMA_CHAN1 : channels 4-7, each 8-bit field at pos 0/8/16/24
     *
     * Lower 6 bits of each field select the peripheral.
     */
    if (ch <= 3) {
        uint32_t shift = ch * 8;
        SYSCTRL->DMA_CHAN = (SYSCTRL->DMA_CHAN & ~(0x3FU << shift))
                           | (periphCode << shift);
    } else {
        uint32_t shift = (ch - 4) * 8;
        SYSCTRL->DMA_CHAN1 = (SYSCTRL->DMA_CHAN1 & ~(0x3FU << shift))
                            | (periphCode << shift);
    }
}

bool DMAClass::_startTransfer(DMA_HandleTypeDef *hdma,
                              uint32_t srcAddr, uint32_t dstAddr,
                              uint32_t dataLen, bool enableIRQ)
{
    if (!hdma) return false;
    if (hdma->State == DMA_STATE_BUSY) return false;
    if (dataLen == 0 || dataLen > DMA_MAX_BLOCK_SIZE) return false;

    uint8_t ch      = hdma->channelIndex;
    DMA_TypeDef *reg = hdma->Instance;
    uint32_t chBit  = CH_EN_BIT(ch);

    /* Ensure channel is idle */
    if (DMA->ChEnReg_L & chBit) {
        _disableChannel(ch);
    }
    _clearChannelFlags(ch);

    /* Set addresses */
    reg->SAR_L = srcAddr;
    reg->DAR_L = dstAddr;

    /* Set block size */
    reg->CTL_H = (reg->CTL_H & ~CTL_BLOCK_TS_MASK) | (dataLen & CTL_BLOCK_TS_MASK);

    /* Enable/disable per-channel interrupt */
    if (enableIRQ) {
        reg->CTL_L |= CTL_INT_EN;
    } else {
        reg->CTL_L &= ~CTL_INT_EN;
    }

    /* Clear multi-block bits (we only do single-block transfers) */
    reg->CTL_L &= ~(3U << 27);   /* LLP_DST_EN, LLP_SRC_EN = 0 */
    reg->LLP_L  = 0;

    hdma->State = DMA_STATE_BUSY;

    /* Fire! */
    _enableChannel(ch);

    return true;
}
