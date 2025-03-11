/*
 * ESP32 High-Precision Motion Control System
 * Safety Monitor
 *
 * Provides comprehensive safety monitoring and error handling,
 * including limit switch monitoring, error detection, and emergency stop.
 */

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <Arduino.h>

#include "../Configuration.h"
#include "../MotorManager.h"
#include "../utils/CircularBuffer.h"
#include "../utils/Logger.h"
#include "Motor.h"

/**
 * System safety status codes
 */
enum class SystemSafetyStatus {
    NORMAL,         // System operating normally
    WARNING,        // Non-critical warning condition
    ERROR,          // Error condition
    EMERGENCY_STOP  // Emergency stop condition
};

/**
 * Safety warning and error codes
 */
enum class SafetyCode {
    NONE,  // No error/warning

    // Warnings (non-critical)
    POSITION_DEVIATION,        // Position deviation above threshold but within tolerance
    VELOCITY_DEVIATION,        // Velocity deviation above threshold but within tolerance
    HIGH_TEMPERATURE,          // Temperature high but below critical threshold
    POWER_SUPPLY_FLUCTUATION,  // Power supply voltage fluctuation

    // Errors (critical)
    POSITION_ERROR,          // Position error exceeded maximum tolerance
    VELOCITY_ERROR,          // Velocity error exceeded maximum tolerance
    LIMIT_SWITCH_TRIGGERED,  // Limit switch triggered unexpectedly
    DRIVER_FAULT,            // Motor driver reported a fault
    ENCODER_ERROR,           // Encoder reading error
    CONTROL_LOOP_TIMING,     // Control loop timing error
    CRITICAL_TEMPERATURE,    // Critical temperature exceeded
    POWER_FAILURE,           // Power supply failure
    COMMUNICATION_TIMEOUT,   // Communication timeout
    EMERGENCY_STOP_PRESSED   // Emergency stop button pressed
};

/**
 * Safety monitor class for system-wide safety monitoring
 */
class SafetyMonitor {
   public:
    /**
     * Constructor
     *
     * @param motorManager Pointer to the motor manager
     * @param logger Pointer to the logger
     */
    SafetyMonitor(MotorManager* motorManager, Logger* logger);

    /**
     * Initialize the safety monitor
     *
     * @return True if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * Check safety conditions
     * This should be called frequently to detect safety issues
     *
     * @return Current system safety status
     */
    SystemSafetyStatus checkSafety();

    /**
     * Trigger emergency stop
     *
     * @param reason Reason for emergency stop
     */
    void triggerEmergencyStop(SafetyCode reason);

    /**
     * Reset emergency stop condition
     *
     * @return True if reset successful, false if conditions not safe
     */
    bool resetEmergencyStop();

    /**
     * Check if system is in emergency stop condition
     *
     * @return True if in emergency stop, false otherwise
     */
    bool isEmergencyStop() const;

    /**
     * Get current safety status
     *
     * @return Current safety status
     */
    SystemSafetyStatus getStatus() const;

    /**
     * Get last safety code
     *
     * @return Last safety code
     */
    SafetyCode getLastSafetyCode() const;

    /**
     * Set emergency stop pin
     *
     * @param pin Pin number for emergency stop button
     * @param activeLevel Level that indicates active (HIGH or LOW)
     */
    void setEmergencyStopPin(uint8_t pin, uint8_t activeLevel = LOW);

    /**
     * Set position tolerance thresholds
     *
     * @param warningThreshold Warning threshold in steps
     * @param errorThreshold Error threshold in steps
     */
    void setPositionTolerances(uint32_t warningThreshold, uint32_t errorThreshold);

    /**
     * Set velocity tolerance thresholds
     *
     * @param warningThreshold Warning threshold in steps/s
     * @param errorThreshold Error threshold in steps/s
     */
    void setVelocityTolerances(float warningThreshold, float errorThreshold);

    /**
     * Set temperature tolerance thresholds
     *
     * @param warningThreshold Warning threshold in °C
     * @param errorThreshold Error threshold in °C
     */
    void setTemperatureTolerances(float warningThreshold, float errorThreshold);

    /**
     * Set voltage tolerance thresholds
     *
     * @param minVoltage Minimum acceptable voltage
     * @param maxVoltage Maximum acceptable voltage
     * @param warningThreshold Warning threshold percentage
     */
    void setVoltageTolerances(float minVoltage, float maxVoltage, float warningThreshold);

    /**
     * Get safety statistics
     *
     * @param warningCount Number of warnings
     * @param errorCount Number of errors
     * @param emergencyStopCount Number of emergency stops
     */
    void getSafetyStats(uint32_t& warningCount, uint32_t& errorCount, uint32_t& emergencyStopCount);

    /**
     * Clear safety statistics
     */
    void clearSafetyStats();

   private:
    // References to system components
    MotorManager* m_motorManager;
    Logger* m_logger;

    // Safety state
    SystemSafetyStatus m_status;
    SafetyCode m_lastCode;
    bool m_emergencyStopActive;

    // Safety statistics
    uint32_t m_warningCount;
    uint32_t m_errorCount;
    uint32_t m_emergencyStopCount;

    // Emergency stop pin configuration
    uint8_t m_emergencyStopPin;
    uint8_t m_emergencyStopActiveLevel;
    bool m_emergencyStopPinConfigured;

    // Safety thresholds
    uint32_t m_positionWarningThreshold;
    uint32_t m_positionErrorThreshold;
    float m_velocityWarningThreshold;
    float m_velocityErrorThreshold;
    float m_temperatureWarningThreshold;
    float m_temperatureErrorThreshold;
    float m_minVoltage;
    float m_maxVoltage;
    float m_voltageWarningThreshold;

    // History buffers for trend analysis
    CircularBuffer<float, 16> m_temperatureBuffer;  // 10 -> 16 amir
    CircularBuffer<float, 16> m_voltageBuffer;      // 10 -> 16 amir

    // Timestamps
    uint32_t m_lastCheckTimeMs;

    /**
     * Read voltage from analog input sensor
     *
     * @return Measured voltage value
     */
    float readVoltageSensor();

    /**
     * Check emergency stop button
     *
     * @return True if emergency stop is pressed, false otherwise
     */
    bool checkEmergencyStopButton();

    /**
     * Check motor safety conditions
     *
     * @return Safety code indicating any issues found
     */
    SafetyCode checkMotorSafety();

    /**
     * Check system voltage and temperature
     *
     * @return Safety code indicating any issues found
     */
    SafetyCode checkSystemConditions();

    /**
     * Log safety event
     *
     * @param status System safety status
     * @param code Safety code
     */
    void logSafetyEvent(SystemSafetyStatus status, SafetyCode code);

    /**
     * Convert safety status to string
     *
     * @param status Safety status
     * @return String representation of status
     */
    String safetyStatusToString(SystemSafetyStatus status) const;

    /**
     * Convert safety code to string
     *
     * @param code Safety code
     * @return String representation of code
     */
    String safetyCodeToString(SafetyCode code) const;
};

#endif  // SAFETY_MONITOR_H