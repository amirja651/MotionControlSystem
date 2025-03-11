/*
 * ESP32 High-Precision Motion Control System
 * Timer Manager
 *
 * Manages hardware timers for precise timing of stepper pulses,
 * control loops, and other time-critical operations.
 */

#ifndef TIMER_MANAGER_H
#define TIMER_MANAGER_H

#include <Arduino.h>

#include "../Configuration.h"
#include "../utils/Logger.h"
#include "driver/timer.h"
#include "esp_timer.h"

/**
 * Type definition for timer callback function
 */
typedef void (*TimerCallback)();

/**
 * Timer Manager singleton class
 */
class TimerManager {
    public:
    /**
     * Get the singleton instance
     *
     * @param logger Optional logger instance
     * @return TimerManager instance
     */
    static TimerManager* getInstance(Logger* logger = nullptr);

    /**
     * Start a hardware timer
     *
     * @param timerIndex Timer index (0 or 1)
     * @param intervalUs Interval in microseconds
     * @param autoReload Whether to auto-reload the timer
     * @param callback Callback function to call on timer expiry
     * @return True if timer started successfully, false otherwise
     */
    bool startTimer(uint8_t timerIndex, uint32_t intervalUs, bool autoReload,
                    TimerCallback callback);

    /**
     * Stop a hardware timer
     *
     * @param timerIndex Timer index (0 or 1)
     * @return True if timer stopped successfully, false otherwise
     */
    bool stopTimer(uint8_t timerIndex);

    /**
     * Check if a timer is running
     *
     * @param timerIndex Timer index (0 or 1)
     * @return True if timer is running, false otherwise
     */
    bool isTimerRunning(uint8_t timerIndex) const;

    /**
     * Get timer interval
     *
     * @param timerIndex Timer index (0 or 1)
     * @return Timer interval in microseconds, or 0 if timer not running
     */
    uint32_t getTimerInterval(uint8_t timerIndex) const;

    /**
     * Set timer interval
     *
     * @param timerIndex Timer index (0 or 1)
     * @param intervalUs New interval in microseconds
     * @return True if interval set successfully, false otherwise
     */
    bool setTimerInterval(uint8_t timerIndex, uint32_t intervalUs);

    /**
     * Get timer callback
     *
     * @param timerIndex Timer index (0 or 1)
     * @return Timer callback function, or nullptr if timer not running
     */
    TimerCallback getTimerCallback(uint8_t timerIndex) const;

    /**
     * Timer interrupt handler for timer 0
     */
    void handleTimer0Interrupt();

    /**
     * Timer interrupt handler for timer 1
     */
    void handleTimer1Interrupt();

   private:
    /**
     * Private constructor for singleton
     * 
     * @param logger Pointer to logger instance
     */
    TimerManager(Logger* logger = nullptr);

    // Add logger member
    Logger* m_logger;

    /**
     * Destructor
     */
    ~TimerManager();

    // Static singleton instance
    static TimerManager* s_instance;

    // Timer state
    bool m_timerRunning[2];
    uint32_t m_timerIntervalUs[2];
    bool m_timerAutoReload[2];
    TimerCallback m_timerCallbacks[2];

    // Timer configuration
    timer_group_t m_timerGroup;
    timer_idx_t m_timerIdx[2];

    /**
     * Configure timer hardware
     *
     * @param timerIndex Timer index (0 or 1)
     * @param intervalUs Interval in microseconds
     * @param autoReload Whether to auto-reload the timer
     * @return True if configuration successful, false otherwise
     */
    bool configureTimer(uint8_t timerIndex, uint32_t intervalUs, bool autoReload);
};

#endif  // TIMER_MANAGER_H