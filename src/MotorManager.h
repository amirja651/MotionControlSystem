/*
 * ESP32 High-Precision Motion Control System
 * Motor Manager
 *
 * Provides centralized management of multiple motors, allowing for
 * synchronized motor control and coordinated multi-axis movements.
 */

#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>

#include "Configuration.h"
#include "core/Motor.h"
#include "utils/EEPROMManager.h"
#include "utils/Logger.h"

/**
 * Motor Manager Class
 *
 * Central management of multiple motors with coordination
 * capabilities
 */
class MotorManager {
public:
    /**
     * Constructor
     *
     * @param maxMotors Maximum number of motors to manage
     * @param logger Pointer to the logger instance
     */
    MotorManager(uint8_t        maxMotors     = CONFIG_MAX_MOTORS,
                 EEPROMManager* eepromManager = nullptr,
                 Logger*        logger        = nullptr);

    // In the existing class implementation, update log calls to use
    // the module-specific logger:

    // Example (these would be in MotorManager.cpp):
    // From:
    // m_logger->logInfo("Motor Manager created with " +
    // String(m_maxMotors) + " motors capacity"); To:
    // m_logger->logInfo("Motor Manager created with " +
    // String(m_maxMotors) + " motors capacity",
    // LogModule::MOTOR_MANAGER);

    // Another example:
    // From:
    // m_logger->logError("Cannot add motor: maximum motor count
    // reached"); To: m_logger->logError("Cannot add motor: maximum
    // motor count reached", LogModule::MOTOR_MANAGER);

    /**
     * Destructor
     */
    ~MotorManager();

    /**
     * Initialize the motor manager
     *
     * @return True if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * Add a motor to the manager
     *
     * @param config Motor configuration
     * @return True if motor added successfully, false otherwise
     */
    bool addMotor(const MotorConfig& config);

    /**
     * Get a motor by index
     *
     * @param index Motor index
     * @return Pointer to motor or nullptr if invalid index
     */
    Motor* getMotor(uint8_t index);

    /**
     * Get the number of motors
     *
     * @return Number of motors
     */
    uint8_t getMotorCount() const;

    /**
     * Enable all motors
     */
    void enableAllMotors();

    /**
     * Disable all motors
     */
    void disableAllMotors();

    /**
     * Stop all motors
     *
     * @param emergency Whether to perform emergency stop
     */
    void stopAllMotors(bool emergency = false);

    /**
     * Load motor configurations from EEPROM
     *
     * @return True if loaded successfully, false otherwise
     */
    bool loadFromEEPROM();

    /**
     * Save motor configurations to EEPROM
     *
     * @return True if saved successfully, false otherwise
     */
    bool saveToEEPROM();

    /**
     * Move multiple motors synchronously to target positions
     *
     * @param positions Array of target positions
     * @param motorIndices Array of motor indices to move
     * @param motorCount Number of motors to move
     * @param maxVelocity Maximum velocity
     * @param acceleration Acceleration rate
     * @param deceleration Deceleration rate
     * @return True if successful, false otherwise
     */
    bool moveMultipleMotors(const int32_t* positions,
                            const uint8_t* motorIndices,
                            uint8_t        motorCount,
                            float          maxVelocity,
                            float          acceleration,
                            float          deceleration);

    /**
     * Check if all specified motors have completed their movements
     *
     * @param motorIndices Array of motor indices to check
     * @param motorCount Number of motors to check
     * @return True if all motors have completed, false otherwise
     */
    bool areMotorsIdle(const uint8_t* motorIndices, uint8_t motorCount);

    /**
     * Calculate synchronized motion profile for multiple motors
     *
     * @param positions Array of target positions
     * @param motorIndices Array of motor indices to move
     * @param motorCount Number of motors to check
     * @param maxVelocity Maximum velocity
     * @param acceleration Acceleration rate
     * @param deceleration Deceleration rate
     * @return Duration of the synchronized move in seconds, or 0.0 if
     * failed
     */
    float calculateSynchronizedMove(const int32_t* positions,
                                    const uint8_t* motorIndices,
                                    uint8_t        motorCount,
                                    float          maxVelocity,
                                    float          acceleration,
                                    float          deceleration);

    /**
     * Check for any errors across all motors
     *
     * @return True if any motor has an error, false otherwise
     */
    bool hasErrors();

    /**
     * Clear errors on all motors
     */
    void clearAllErrors();

    /**
     * Reset all encoders to zero
     */
    void resetAllEncoders();

private:
    // Array of motors
    Motor** m_motors;

    // Maximum number of motors
    uint8_t m_maxMotors;

    // Current number of motors
    uint8_t m_motorCount;

    // EEPROM manager for parameter storage
    EEPROMManager* m_eepromManager;

    // Logger instance
    Logger* m_logger;
};

#endif  // MOTOR_MANAGER_H