/*
 * HardwareTimer.cpp — STM32duino-compatible Timer API for Air105
 */

#include "HardwareTimer.h"
#include "air105.h"

/* Timer control register bits */
#define TIM_CTRL_ENABLE     0x01
#define TIM_CTRL_MODE       0x02
#define TIM_CTRL_INT_MASK   0x04  /* 1=masked/disabled, 0=enabled */

/* IRQ numbers for timer channels */
static const IRQn_Type timerIRQn[8] = {
    TIM0_0_IRQn, TIM0_1_IRQn, TIM0_2_IRQn, TIM0_3_IRQn,
    TIM0_4_IRQn, TIM0_5_IRQn, TIM0_6_IRQn, TIM0_7_IRQn
};

/* Global callback storage for each timer channel */
static callback_function_t timerCallbacks[8] = {NULL};

/* Common IRQ handler - forwards to user callback */
static void timerIRQHandler(uint8_t channel) {
    /* Clear interrupt by reading EOI */
    volatile uint32_t dummy = TIMM0->TIM[channel].EOI;
    (void)dummy;
    
    /* Call user callback if registered */
    if (timerCallbacks[channel] != NULL) {
        timerCallbacks[channel]();
    }
}

/* IRQ handlers for each timer channel */
extern "C" {
    void TIM0_0_IRQHandler(void) { timerIRQHandler(0); }
    void TIM0_1_IRQHandler(void) { timerIRQHandler(1); }
    void TIM0_2_IRQHandler(void) { timerIRQHandler(2); }
    void TIM0_3_IRQHandler(void) { timerIRQHandler(3); }
    void TIM0_4_IRQHandler(void) { timerIRQHandler(4); }
    void TIM0_5_IRQHandler(void) { timerIRQHandler(5); }
    void TIM0_6_IRQHandler(void) { timerIRQHandler(6); }
    void TIM0_7_IRQHandler(void) { timerIRQHandler(7); }
}

/* ---- HardwareTimer Implementation ---- */

HardwareTimer::HardwareTimer(TimerInstance_t timer) {
    _timer = (uint8_t)timer;
    _overflow = 0;
    _running = false;
    _priority = 3;
    
    /* Ensure timer is stopped */
    if (_timer < 8) {
        TIMM0->TIM[_timer].ControlReg = 0;
    }
}

HardwareTimer::~HardwareTimer() {
    pause();
    detachInterrupt();
}

void HardwareTimer::pause(void) {
    if (_timer >= 8) return;
    
    TIMM0->TIM[_timer].ControlReg = 0;
    _running = false;
}

void HardwareTimer::resume(void) {
    if (_timer >= 8) return;
    
    applySettings();
    _running = true;
}

void HardwareTimer::refresh(void) {
    if (_running) {
        applySettings();
    }
}

uint32_t HardwareTimer::getTimerClockFreq(void) {
    /* Air105 timer clock = SystemCoreClock / 4 */
    return SystemCoreClock / 4;
}

void HardwareTimer::setOverflow(uint32_t val, TimerFormat_t format) {
    uint32_t timerClock = getTimerClockFreq();
    
    switch (format) {
        case MICROSEC_FORMAT:
            /* Convert microseconds to ticks */
            /* ticks = us * timerClock / 1,000,000 */
            _overflow = (uint32_t)(((uint64_t)val * timerClock) / 1000000ULL);
            break;
            
        case HERTZ_FORMAT:
            /* Convert Hz to ticks */
            /* ticks = timerClock / Hz */
            if (val > 0) {
                _overflow = timerClock / val;
            }
            break;
            
        case TICK_FORMAT:
        default:
            _overflow = val;
            break;
    }
    
    /* Ensure at least 1 tick */
    if (_overflow < 1) {
        _overflow = 1;
    }
}

uint32_t HardwareTimer::getOverflow(TimerFormat_t format) {
    uint32_t timerClock = getTimerClockFreq();
    
    switch (format) {
        case MICROSEC_FORMAT:
            /* Convert ticks to microseconds */
            return (uint32_t)(((uint64_t)_overflow * 1000000ULL) / timerClock);
            
        case HERTZ_FORMAT:
            /* Convert ticks to Hz */
            if (_overflow > 0) {
                return timerClock / _overflow;
            }
            return 0;
            
        case TICK_FORMAT:
        default:
            return _overflow;
    }
}

void HardwareTimer::attachInterrupt(callback_function_t callback) {
    if (_timer >= 8) return;
    
    timerCallbacks[_timer] = callback;
    
    if (callback != NULL) {
        NVIC_SetPriority(timerIRQn[_timer], _priority);
        NVIC_EnableIRQ(timerIRQn[_timer]);
    }
}

void HardwareTimer::detachInterrupt(void) {
    if (_timer >= 8) return;
    
    NVIC_DisableIRQ(timerIRQn[_timer]);
    timerCallbacks[_timer] = NULL;
}

bool HardwareTimer::hasInterrupt(void) {
    if (_timer >= 8) return false;
    return timerCallbacks[_timer] != NULL;
}

void HardwareTimer::setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority) {
    (void)subPriority;  /* Air105 doesn't use sub-priority */
    
    _priority = (uint8_t)(preemptPriority & 0x0F);
    
    if (_timer < 8 && timerCallbacks[_timer] != NULL) {
        NVIC_SetPriority(timerIRQn[_timer], _priority);
    }
}

void HardwareTimer::applySettings(void) {
    if (_timer >= 8 || _overflow == 0) return;
    
    /* Disable timer first */
    TIMM0->TIM[_timer].ControlReg = 0;
    
    /* Set period (LoadCount is the reload value) */
    TIMM0->TIM[_timer].LoadCount = _overflow - 1;
    
    /* Enable timer in user-defined mode (INT_MASK=0 enables interrupts) */
    TIMM0->TIM[_timer].ControlReg = TIM_CTRL_ENABLE | TIM_CTRL_MODE;
}
