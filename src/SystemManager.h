/*
 * ESP32 High-Precision Motion Control System
 * System Manager
 * 
 * Central coordination of system components, managing initialization,
 * task scheduling, and system-wide operations.
 */

#ifndef SYSTEM_MANAGER_H
#define SYSTEM_MANAGER_H

#include <Arduino.h>
#include "Configuration.h"
#include "MotorManager.h"
#include "core/SafetyMonitor.h"
#include "utils/Logger.h"
#include "utils/TaskScheduler.h"
#include "utils/EEPROMManager.h"
#include "communication/StatusReporter.h"

/**
 * System state enumeration
 */
enum class SystemState {
    INITIALIZING,         // System is initializing
    READY,                // System is ready for operation
    RUNNING,              // System is running
    ERROR,                // System is in error state
    EMERGENCY_STOP,       // System is in emergency stop mode
    SHUTDOWN              // System is shutting down
};

/**
 * System manager class for central orchestration
 */
class SystemManager {
public:
    /**
     * Constructor
     */
    SystemManager();
    
    /**
     * Destructor
     */
    ~SystemManager();
    
    /**
     * Initialize the system
     * 
     * @return True if initialization successful, false otherwise
     */
    bool initialize();
    
    /**
     * Get current system state
     * 
     * @return Current system state
     */
    SystemState getSystemState() const;
    
    /**
     * Get safety monitor
     * 
     * @return Pointer to safety monitor
     */
    SafetyMonitor* getSafetyMonitor();
    
    /**
     * Get motor manager
     * 
     * @return Pointer to motor manager
     */
    MotorManager* getMotorManager();
    
    /**
     * Get logger
     * 
     * @return Pointer to logger
     */
    Logger* getLogger();
    
    /**
     * Get task scheduler
     * 
     * @return Pointer to task scheduler
     */
    TaskScheduler* getTaskScheduler();
    
    /**
     * Get status reporter
     * 
     * @return Pointer to status reporter
     */
    StatusReporter* getStatusReporter();
    
    /**
     * Get EEPROM manager
     * 
     * @return Pointer to EEPROM manager
     */
    EEPROMManager* getEEPROMManager();
    
    /**
     * Set system state
     * 
     * @param state New system state
     */
    void setSystemState(SystemState state);
    
    /**
     * Trigger emergency stop
     * 
     * @param reason Reason for emergency stop
     */
    void triggerEmergencyStop(SafetyCode reason);
    
    /**
     * Reset emergency stop
     * 
     * @return True if reset successful, false otherwise
     */
    bool resetEmergencyStop();
    
    /**
     * Check if system is in emergency stop
     * 
     * @return True if in emergency stop, false otherwise
     */
    bool isEmergencyStop() const;
    
    /**
     * Save system configuration to EEPROM
     * 
     * @return True if saved successfully, false otherwise
     */
    bool saveSystemConfiguration();
    
    /**
     * Load system configuration from EEPROM
     * 
     * @return True if loaded successfully, false otherwise
     */
    bool loadSystemConfiguration();
    
    /**
     * Get system uptime in milliseconds
     * 
     * @return Uptime in milliseconds
     */
    uint32_t getUptimeMs() const;
    
    /**
     * Get CPU usage percentage
     * 
     * @param core Core number (0 or 1)
     * @return CPU usage percentage (0-100)
     */
    float getCPUUsage(uint8_t core) const;
    
    /**
     * Get free memory in bytes
     * 
     * @return Free memory in bytes
     */
    uint32_t getFreeMemory() const;
    
    /**
     * Reset the system
     */
    void resetSystem();
    
    /**
     * Update system metrics
     * Should be called periodically to update CPU usage and memory stats
     */
    void updateSystemMetrics();
    
private:
    // System components
    MotorManager* m_motorManager;
    SafetyMonitor* m_safetyMonitor;
    Logger* m_logger;
    TaskScheduler* m_taskScheduler;
    StatusReporter* m_statusReporter;
    EEPROMManager* m_eepromManager;
    
    // System state
    SystemState m_systemState;
    uint32_t m_startTimeMs;
    
    // System metrics
    float m_cpuUsageCore0;
    float m_cpuUsageCore1;
    uint32_t m_freeMemory;
    uint32_t m_lastMetricsUpdateMs;
    
    // Performance measurement
    uint32_t m_controlLoopStartTimeUs;
    uint32_t m_controlLoopExecutionTimeUs;
    
    /**
     * Calculate CPU usage
     * 
     * @param core Core number (0 or 1)
     * @return CPU usage percentage (0-100)
     */
    float calculateCPUUsage(uint8_t core);
    
    /**
     * Calculate free memory
     * 
     * @return Free memory in bytes
     */
    uint32_t calculateFreeMemory();
};

#endif // SYSTEM_MANAGER_H