/*
 * ESP32 High-Precision Motion Control System
 * Timer Manager Implementation
 */

#include "TimerManager.h"

// Static instance for singleton
TimerManager* TimerManager::s_instance = nullptr;

// Static timer ISR handlers
static void IRAM_ATTR timer0ISR(void* arg) {
    // Clear the interrupt
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);

    // Re-enable the alarm if auto-reload is enabled
    if (TimerManager::getInstance()->isTimerRunning(0)) {
        timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
    }

    // Call the user callback
    TimerCallback callback = TimerManager::getInstance()->getTimerCallback(0);
    if (callback != nullptr) {
        callback();
    }
}

static void IRAM_ATTR timer1ISR(void* arg) {
    // Clear the interrupt
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);

    // Re-enable the alarm if auto-reload is enabled
    if (TimerManager::getInstance()->isTimerRunning(1)) {
        timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1);
    }

    // Call the user callback
    TimerCallback callback = TimerManager::getInstance()->getTimerCallback(1);
    if (callback != nullptr) {
        callback();
    }
}

TimerManager* TimerManager::getInstance() {
    if (s_instance == nullptr) {
        s_instance = new TimerManager();
    }
    return s_instance;
}

TimerManager::TimerManager() : m_timerGroup(TIMER_GROUP_0) {
    // Initialize timer state
    for (int i = 0; i < 2; i++) {
        m_timerRunning[i] = false;
        m_timerIntervalUs[i] = 0;
        m_timerAutoReload[i] = false;
        m_timerCallbacks[i] = nullptr;
    }

    // Set timer indices
    m_timerIdx[0] = TIMER_0;
    m_timerIdx[1] = TIMER_1;
}

TimerManager::~TimerManager() {
    // Stop all timers
    stopTimer(0);
    stopTimer(1);
}

bool TimerManager::startTimer(uint8_t timerIndex, uint32_t intervalUs, bool autoReload,
                              TimerCallback callback) {
    // Validate parameters
    if (timerIndex > 1 || intervalUs == 0 || callback == nullptr) {
        return false;
    }

    // Stop the timer if it's already running
    if (m_timerRunning[timerIndex]) {
        stopTimer(timerIndex);
    }

    // Store timer parameters
    m_timerIntervalUs[timerIndex] = intervalUs;
    m_timerAutoReload[timerIndex] = autoReload;
    m_timerCallbacks[timerIndex] = callback;

    // Configure the timer hardware
    if (!configureTimer(timerIndex, intervalUs, autoReload)) {
        return false;
    }

    // Start the timer
    timer_start(m_timerGroup, m_timerIdx[timerIndex]);

    // Mark timer as running
    m_timerRunning[timerIndex] = true;

    return true;
}

bool TimerManager::stopTimer(uint8_t timerIndex) {
    // Validate parameters
    if (timerIndex > 1) {
        return false;
    }

    // If timer is not running, nothing to do
    if (!m_timerRunning[timerIndex]) {
        return true;
    }

    // Stop the timer
    timer_pause(m_timerGroup, m_timerIdx[timerIndex]);

    // Mark timer as not running
    m_timerRunning[timerIndex] = false;

    return true;
}

bool TimerManager::isTimerRunning(uint8_t timerIndex) const {
    if (timerIndex > 1) {
        return false;
    }

    return m_timerRunning[timerIndex];
}

uint32_t TimerManager::getTimerInterval(uint8_t timerIndex) const {
    if (timerIndex > 1) {
        return 0;
    }

    return m_timerIntervalUs[timerIndex];
}

bool TimerManager::setTimerInterval(uint8_t timerIndex, uint32_t intervalUs) {
    // Validate parameters
    if (timerIndex > 1 || intervalUs == 0) {
        return false;
    }

    // Store new interval
    m_timerIntervalUs[timerIndex] = intervalUs;

    // If timer is running, update its configuration
    if (m_timerRunning[timerIndex]) {
        // We need to stop and restart the timer to change its interval
        bool wasRunning = true;
        stopTimer(timerIndex);

        // Restart the timer with the new interval
        return startTimer(timerIndex, intervalUs, m_timerAutoReload[timerIndex],
                          m_timerCallbacks[timerIndex]);
    }

    return true;
}

TimerCallback TimerManager::getTimerCallback(uint8_t timerIndex) const {
    if (timerIndex > 1) {
        return nullptr;
    }

    return m_timerCallbacks[timerIndex];
}

void TimerManager::handleTimer0Interrupt() {
    // This method is called from the timer0 ISR
    // The actual implementation is in the static timer0ISR function
}

void TimerManager::handleTimer1Interrupt() {
    // This method is called from the timer1 ISR
    // The actual implementation is in the static timer1ISR function
}

bool TimerManager::configureTimer(uint8_t timerIndex, uint32_t intervalUs, bool autoReload) {
    // Validate parameters
    if (timerIndex > 1) {
        return false;
    }

    // Calculate timer counter value
    // ESP32 timers are 80MHz, so divide by 80 to get microseconds
    uint64_t alarmValue = (uint64_t)intervalUs * 80;

    // Timer configuration
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,                                              // Enable alarm
        .counter_en = TIMER_PAUSE,                                               // Start paused
        .intr_type = TIMER_INTR_LEVEL,                                           // Level interrupt
        .counter_dir = TIMER_COUNT_UP,                                           // Count up
        .auto_reload = autoReload ? TIMER_AUTORELOAD_EN : TIMER_AUTORELOAD_DIS,  // Auto-reload
        .divider = 1                                                             // 80MHz (APB_CLK)
    };

    // Initialize timer
    timer_init(m_timerGroup, m_timerIdx[timerIndex], &config);

    // Set counter value to 0
    timer_set_counter_value(m_timerGroup, m_timerIdx[timerIndex], 0);

    // Set alarm value
    timer_set_alarm_value(m_timerGroup, m_timerIdx[timerIndex], alarmValue);

    // Enable timer interrupt
    timer_enable_intr(m_timerGroup, m_timerIdx[timerIndex]);

    // Register interrupt handler
    if (timerIndex == 0) {
        timer_isr_register(m_timerGroup, m_timerIdx[timerIndex], timer0ISR, nullptr,
                           ESP_INTR_FLAG_IRAM, nullptr);
    } else {
        timer_isr_register(m_timerGroup, m_timerIdx[timerIndex], timer1ISR, nullptr,
                           ESP_INTR_FLAG_IRAM, nullptr);
    }

    return true;
}