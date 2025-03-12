/*
 * ESP32 High-Precision Motion Control System
 * System Manager Implementation
 */

#include "SystemManager.h"

SystemManager::SystemManager()
    : m_statusReporter(nullptr),
      m_safetyMonitor(nullptr),
      m_motorManager(nullptr),
      m_taskScheduler(nullptr),
      m_logger(nullptr),
      m_eepromManager(nullptr),
      m_systemState(SystemState::INITIALIZING),
      m_startTimeMs(0),
      m_cpuUsageCore0(0.0f),
      m_cpuUsageCore1(0.0f),
      m_freeMemory(0),
      m_lastMetricsUpdateMs(0),
      m_controlLoopStartTimeUs(0),
      m_controlLoopExecutionTimeUs(0) {
}

SystemManager::~SystemManager() {
    // Clean up resources
    delete m_statusReporter;
    delete m_safetyMonitor;
    delete m_motorManager;
    delete m_taskScheduler;
    delete m_logger;
    delete m_eepromManager;
}

bool SystemManager::initialize() {
    // Record start time
    m_startTimeMs = millis();

    // Create system components in the correct order

    // EEPROM Manager first as it's needed by other components
    m_eepromManager = new EEPROMManager();
    if (!m_eepromManager->initialize()) {
        return false;
    }

    // Logger for system-wide logging
    m_logger = new Logger();
    if (!m_logger->initialize()) {
        return false;
    }

    m_logger->logInfo("System initializing...", LogModule::SYSTEM);

    // Task scheduler
    m_taskScheduler = new TaskScheduler();
    if (!m_taskScheduler->initialize()) {
        m_logger->logError("Failed to initialize task scheduler", LogModule::SYSTEM);
        return false;
    }

    // Motor manager for motion control
    m_motorManager = new MotorManager(CONFIG_MAX_MOTORS, m_logger);
    if (!m_motorManager->initialize()) {
        m_logger->logError("Failed to initialize motor manager", LogModule::SYSTEM);
        return false;
    }

    // Safety monitor
    m_safetyMonitor = new SafetyMonitor(m_motorManager, m_logger);
    if (!m_safetyMonitor->initialize()) {
        m_logger->logError("Failed to initialize safety monitor", LogModule::SYSTEM);
        return false;
    }

    // Status reporter
    m_statusReporter = new StatusReporter(this);
    if (!m_statusReporter->initialize()) {
        m_logger->logError("Failed to initialize status reporter", LogModule::SYSTEM);
        return false;
    }

    // Load system configuration from EEPROM
    if (!loadSystemConfiguration()) {
        m_logger->logWarning("Could not load system configuration, using defaults",
                             LogModule::SYSTEM);
    }

    // Update system metrics
    updateSystemMetrics();

    // Set system state to ready
    m_systemState = SystemState::READY;

    m_logger->logInfo("System initialized successfully", LogModule::SYSTEM);

    return true;
}

SystemState SystemManager::getSystemState() const {
    return m_systemState;
}

SafetyMonitor *SystemManager::getSafetyMonitor() {
    return m_safetyMonitor;
}

MotorManager *SystemManager::getMotorManager() {
    return m_motorManager;
}

Logger *SystemManager::getLogger() {
    return m_logger;
}

TaskScheduler *SystemManager::getTaskScheduler() {
    return m_taskScheduler;
}

StatusReporter *SystemManager::getStatusReporter() {
    return m_statusReporter;
}

void SystemManager::setStatusReporter(StatusReporter *reporter) {
    m_statusReporter = reporter;
}

EEPROMManager *SystemManager::getEEPROMManager() {
    return m_eepromManager;
}

void SystemManager::setSystemState(SystemState state) {
    // Only allow specific state transitions
    switch (m_systemState) {
        case SystemState::INITIALIZING:
            // Can transition to READY or ERROR
            if (state == SystemState::READY || state == SystemState::ERROR) {
                m_systemState = state;
            }
            break;

        case SystemState::READY:
            // Can transition to RUNNING, ERROR, or EMERGENCY_STOP
            if (state == SystemState::RUNNING || state == SystemState::ERROR
                || state == SystemState::EMERGENCY_STOP) {
                m_systemState = state;
            }
            break;

        case SystemState::RUNNING:
            // Can transition to READY, ERROR, or EMERGENCY_STOP
            if (state == SystemState::READY || state == SystemState::ERROR
                || state == SystemState::EMERGENCY_STOP) {
                m_systemState = state;
            }
            break;

        case SystemState::ERROR:
            // Can transition to READY, EMERGENCY_STOP, or SHUTDOWN
            if (state == SystemState::READY || state == SystemState::EMERGENCY_STOP
                || state == SystemState::SHUTDOWN) {
                m_systemState = state;
            }
            break;

        case SystemState::EMERGENCY_STOP:
            // Can transition to READY or SHUTDOWN
            if (state == SystemState::READY || state == SystemState::SHUTDOWN) {
                m_systemState = state;
            }
            break;

        case SystemState::SHUTDOWN:
            // Terminal state, no transitions allowed
            break;
    }

    // Log state change
    if (m_logger != nullptr) {
        m_logger->logInfo("System state changed to " + String(static_cast<int>(m_systemState)),
                          LogModule::SYSTEM);
    }
}

void SystemManager::triggerEmergencyStop(SafetyCode reason) {
    if (m_safetyMonitor != nullptr) {
        m_safetyMonitor->triggerEmergencyStop(reason);
    }

    // Disable all motors
    if (m_motorManager != nullptr) {
        m_motorManager->stopAllMotors(true);
    }

    // Set system state
    setSystemState(SystemState::EMERGENCY_STOP);

    // Log emergency stop
    if (m_logger != nullptr) {
        m_logger->logError("EMERGENCY STOP triggered: " + String(static_cast<int>(reason)),
                           LogModule::SAFETY_MONITOR);
    }
}

bool SystemManager::resetEmergencyStop() {
    if (m_safetyMonitor == nullptr) {
        m_logger->logError(
            "Failed to reset emergency stop because the Safety monitor is not initialized.",
            LogModule::SYSTEM);
        return false;
    }

    // Try to reset emergency stop
    if (m_safetyMonitor->resetEmergencyStop()) {
        // Set system state back to ready
        setSystemState(SystemState::READY);

        // Log reset
        if (m_logger != nullptr) {
            m_logger->logInfo("Emergency stop reset", LogModule::SYSTEM);
        }

        return true;
    }

    return false;
}

bool SystemManager::isEmergencyStop() const {
    return m_systemState == SystemState::EMERGENCY_STOP;
}

bool SystemManager::saveSystemConfiguration() {
    if (m_eepromManager == nullptr) {
        m_logger->logError(
            "Failed to save system configuration because the EEPROM manager is not initialized.",
            LogModule::SYSTEM);
        return false;
    }

    bool success = true;

    // Save motor manager configuration
    if (m_motorManager != nullptr) {
        success &= m_motorManager->saveToEEPROM();
    }

    // Additional configuration saving can be added here

    // Commit all changes
    success &= m_eepromManager->commit();

    if (success && m_logger != nullptr) {
        m_logger->logInfo("System configuration saved", LogModule::SYSTEM);
    }

    return success;
}

bool SystemManager::loadSystemConfiguration() {
    if (m_eepromManager == nullptr) {
        m_logger->logError(
            "Failed to load system configuration because the EEPROM manager is not initialized.",
            LogModule::SYSTEM);
        return false;
    }

    bool success = true;

    // Load motor manager configuration
    if (m_motorManager != nullptr) {
        success &= m_motorManager->loadFromEEPROM();
    }

    // Additional configuration loading can be added here

    if (success && m_logger != nullptr) {
        m_logger->logInfo("System configuration loaded", LogModule::SYSTEM);
    }

    return success;
}

uint32_t SystemManager::getUptimeMs() const {
    return millis() - m_startTimeMs;
}

float SystemManager::getCPUUsage(uint8_t core) const {
    if (core == 0) {
        return m_cpuUsageCore0;
    } else if (core == 1) {
        return m_cpuUsageCore1;
    }
    return 0.0f;
}

uint32_t SystemManager::getFreeMemory() const {
    return m_freeMemory;
}

void SystemManager::resetSystem() {
    // Log reset
    if (m_logger != nullptr) {
        m_logger->logInfo("System reset requested", LogModule::SYSTEM);
    }

    // Save configuration before reset
    saveSystemConfiguration();

    // Set state to shutdown
    setSystemState(SystemState::SHUTDOWN);

    // Wait for logging to complete
    delay(100);

    // Restart ESP32
    ESP.restart();
}

void SystemManager::updateSystemMetrics() {
    // Only update periodically (every second)
    uint32_t currentTimeMs = millis();
    if (currentTimeMs - m_lastMetricsUpdateMs < 1000) {
        return;
    }

    m_lastMetricsUpdateMs = currentTimeMs;

    // Update CPU usage
    m_cpuUsageCore0 = calculateCPUUsage(0);
    m_cpuUsageCore1 = calculateCPUUsage(1);

    // Update free memory
    m_freeMemory = calculateFreeMemory();
}

float SystemManager::calculateCPUUsage(uint8_t core) {
    // This is an estimation since ESP32 doesn't provide direct CPU usage metrics
    // A more accurate implementation would use FreeRTOS task statistics

    // Start timing
    uint32_t startTime = micros();

    // Baseline delay (empty loop)
    for (volatile int i = 0; i < 1000; i++) {
        // Empty loop for baseline measurement
    }

    uint32_t baselineTime = micros() - startTime;

    // Delay when CPU is busy
    startTime = micros();

    // This loop will be affected by other tasks running on the core
    for (volatile int i = 0; i < 1000; i++) {
        // Same empty loop, but now affected by CPU load
    }

    uint32_t loadedTime = micros() - startTime;

    // Calculate load percentage
    // Higher ratio means more time spent in the loop, indicating less CPU load
    float ratio = static_cast<float>(baselineTime) / static_cast<float>(loadedTime);

    // Convert to percentage (0-100)
    float usage = (1.0f - ratio) * 100.0f;

    // Clamp to valid range
    if (usage < 0.0f)
        usage = 0.0f;
    if (usage > 100.0f)
        usage = 100.0f;

    return usage;
}

uint32_t SystemManager::calculateFreeMemory() {
    // ESP32 method for getting free heap size
    return ESP.getFreeHeap();
}

// Implementation of the new methods

/**
 * Save current positions of all motors to EEPROM
 */
void SystemManager::saveMotorPositions() {
    if (m_motorManager == nullptr) {
        m_logger->logError(
            "Failed to save motor positions because the Motor manager is not initialized.",
            LogModule::SYSTEM);
        return;
    }

    if (m_eepromManager == nullptr) {
        m_logger->logError(
            "Failed to save motor positions because the EEPROM manager is not initialized.",
            LogModule::SYSTEM);
        return;
    }

    // Loop through all motors and save their positions
    for (uint8_t i = 0; i < m_motorManager->getMotorCount(); i++) {
        Motor *motor = m_motorManager->getMotor(i);
        if (motor != nullptr) {
            int32_t position = motor->getCurrentPosition();
            // Save each motor's position in its own EEPROM address slot
            m_eepromManager->saveUserData(
                &position, sizeof(position), MOTOR_POSITIONS_ADDR + (i * sizeof(int32_t)));
        }
    }

    // Commit changes to ensure they are written to EEPROM
    m_eepromManager->commit();
}

/**
 * Restore motor positions from EEPROM after power failure
 */
void SystemManager::restoreMotorPositions() {
    if (m_motorManager == nullptr) {
        m_logger->logError(
            "Failed to restore motor positions because the Motor manager is not initialized.",
            LogModule::SYSTEM);
        return;
    }

    if (m_eepromManager == nullptr) {
        m_logger->logError(
            "Failed to restore motor positions because the EEPROM manager is not initialized.",
            LogModule::SYSTEM);
        return;
    }

    // Loop through all motors and restore their positions
    for (uint8_t i = 0; i < m_motorManager->getMotorCount(); i++) {
        Motor *motor = m_motorManager->getMotor(i);
        if (motor != nullptr) {
            int32_t position = 0;
            // Load position from EEPROM
            m_eepromManager->loadUserData(
                &position, sizeof(position), MOTOR_POSITIONS_ADDR + (i * sizeof(int32_t)));

            // Enable motor if not already enabled
            if (!motor->isEnabled()) {
                motor->enable();
            }

            // Move to the previously saved position
            // Restore motor positions
            motor->moveToPosition(position,
                                  CONFIG_DEFAULT_MAX_VELOCITY / 2.0f,  // Default half max velocity
                                  CONFIG_DEFAULT_ACCELERATION,         // Default acceleration
                                  CONFIG_DEFAULT_DECELERATION);        // Default deceleration
        }
    }
}

/**
 * Check if the previous shutdown was normal
 *
 * @return true if shutdown was normal, false if power failure
 */
bool SystemManager::wasNormalShutdown() {
// Skip power failure recovery if power monitoring is disabled
#if !CONFIG_POWER_MONITORING_ENABLED
    return true;  // Always report normal shutdown if power monitoring is disabled
#endif

    if (m_eepromManager == nullptr) {
        m_logger->logError(
            "Failed to load user data because the EEPROM manager is not initialized.",
            LogModule::SYSTEM);
        return false;
    }

    bool normalShutdown = false;
    m_eepromManager->loadUserData(&normalShutdown, sizeof(normalShutdown), SHUTDOWN_FLAG_ADDR);
    return normalShutdown;
}

/**
 * Set the normal shutdown flag in EEPROM
 *
 * @param state true for normal shutdown, false to reset flag
 */
void SystemManager::setNormalShutdown(bool state) {
    if (m_eepromManager == nullptr) {
        m_logger->logError(
            "Failed to set normal shutdown and save user data because the EEPROM manager is not "
            "initialized.",
            LogModule::SYSTEM);
        return;
    }

    m_eepromManager->saveUserData(&state, sizeof(state), SHUTDOWN_FLAG_ADDR);
    m_eepromManager->commit();
}

int SystemManager::getMotorPositionsAddr() {
    return MOTOR_POSITIONS_ADDR;
};
// End of Code