/*
 * HardwareTimer.h — STM32duino-compatible Timer API for Air105
 * 
 * Provides a HardwareTimer class compatible with Arduino_Core_STM32.
 * 
 * Air105 has 8 timer channels (TIM0-TIM7) in one TIMM0 module.
 * Channels 0-5 are typically used for PWM, 6-7 for user timers.
 * 
 * Usage:
 *   HardwareTimer timer(TIM6);
 *   timer.setOverflow(500000, MICROSEC_FORMAT);  // 500ms
 *   timer.attachInterrupt(myCallback);
 *   timer.resume();
 */

#ifndef HARDWARETIMER_H
#define HARDWARETIMER_H

#include "Arduino.h"

#ifdef __cplusplus

/* Timer format for setOverflow() */
typedef enum {
    TICK_FORMAT = 0,      /* Raw timer ticks */
    MICROSEC_FORMAT = 1,  /* Microseconds */
    HERTZ_FORMAT = 2      /* Frequency in Hz */
} TimerFormat_t;

/* Timer instance definitions */
typedef enum {
    TIM0 = 0,
    TIM1 = 1,
    TIM2 = 2,
    TIM3 = 3,
    TIM4 = 4,
    TIM5 = 5,
    TIM6 = 6,
    TIM7 = 7
} TimerInstance_t;

/* Callback function type */
typedef void (*callback_function_t)(void);

class HardwareTimer {
public:
    /**
     * Constructor
     * @param timer Timer instance (TIM0-TIM7)
     */
    HardwareTimer(TimerInstance_t timer);
    
    /**
     * Destructor - stops timer and detaches interrupt
     */
    ~HardwareTimer();
    
    /**
     * Pause the timer (stop counting)
     */
    void pause(void);
    
    /**
     * Resume the timer (start/continue counting)
     */
    void resume(void);
    
    /**
     * Force reload of all registers
     */
    void refresh(void);
    
    /**
     * Set the timer overflow period/frequency
     * @param val The overflow value
     * @param format TICK_FORMAT, MICROSEC_FORMAT, or HERTZ_FORMAT
     */
    void setOverflow(uint32_t val, TimerFormat_t format = TICK_FORMAT);
    
    /**
     * Get the current overflow value
     * @param format The format to return (TICK_FORMAT, MICROSEC_FORMAT, HERTZ_FORMAT)
     * @return The overflow value in requested format
     */
    uint32_t getOverflow(TimerFormat_t format = TICK_FORMAT);
    
    /**
     * Attach an interrupt callback for timer overflow
     * @param callback Function to call when timer overflows
     */
    void attachInterrupt(callback_function_t callback);
    
    /**
     * Detach the timer overflow interrupt
     */
    void detachInterrupt(void);
    
    /**
     * Check if an interrupt is attached
     * @return true if interrupt callback is set
     */
    bool hasInterrupt(void);
    
    /**
     * Set interrupt priority
     * @param preemptPriority Preemption priority (0-15, lower = higher priority)
     * @param subPriority Sub-priority (ignored on Air105, single priority level)
     */
    void setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority = 0);
    
    /**
     * Get the timer instance number
     */
    uint8_t getTimerInstance(void) { return _timer; }

private:
    uint8_t _timer;           /* Timer channel (0-7) */
    uint32_t _overflow;       /* Overflow value in ticks */
    bool _running;            /* Timer running state */
    uint8_t _priority;        /* Interrupt priority */
    
    /* Calculate timer clock frequency */
    uint32_t getTimerClockFreq(void);
    
    /* Apply current settings to hardware */
    void applySettings(void);
};

/* Global timer instances that can be used like STM32duino */
/* Users can create their own: HardwareTimer myTimer(TIM6); */

#endif /* __cplusplus */

#endif /* HARDWARETIMER_H */
