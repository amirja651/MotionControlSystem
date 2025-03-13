/*
 * ESP32 High-Precision Motion Control System
 * Motor Manager Implementation
 */

#include "MotorManager.h"

MotorManager::MotorManager(uint8_t        maxMotors,
                           EEPROMManager* eepromManager,
                           Logger*        logger)
    : m_eepromManager(eepromManager),
      m_maxMotors(maxMotors > 0 ? maxMotors : CONFIG_MAX_MOTORS),
      m_motorCount(0),
      m_logger(logger) {
    // Allocate memory for motor pointers
    m_motors = new Motor*[m_maxMotors];

    // Initialize all pointers to nullptr
    for (uint8_t i = 0; i < m_maxMotors; i++) {
        m_motors[i] = nullptr;
    }

    // Log creation if logger is available
    m_logger->logInfo("Motor Manager created with " + String(m_maxMotors)
                          + " motors capacity",
                      LogModule::MOTOR_MANAGER);
}

MotorManager::~MotorManager() {
    // Free allocated motors
    for (uint8_t i = 0; i < m_motorCount; i++) {
        if (m_motors[i] != nullptr) {
            delete m_motors[i];
            m_motors[i] = nullptr;
        }
    }

    // Free the motor array
    delete[] m_motors;
    delete m_logger;
    delete m_eepromManager;
}

bool MotorManager::initialize() {
    // If there are predefined motor configurations, add
    // them
    for (uint8_t i = 0; i < CONFIG_MAX_MOTORS; i++) {
        if (i < m_maxMotors) {
            m_logger->logInfo("Adding default motor " + String(i),
                              LogModule::MOTOR_MANAGER);
            addMotor(DEFAULT_MOTOR_CONFIGS[i]);
        }
    }

    // Load saved parameters if available
    loadFromEEPROM();

    m_logger->logInfo(
        "Motor Manager initialized with " + String(m_motorCount) + " motors",
        LogModule::MOTOR_MANAGER);

    return true;
}

bool MotorManager::addMotor(const MotorConfig& config) {
    // Check if we have room for another motor
    if (m_motorCount >= m_maxMotors) {
        m_logger->logError("Cannot add motor: maximum motor count reached",
                           LogModule::MOTOR_MANAGER);
        return false;
    }

    // Check if motor index is valid
    if (config.index >= m_maxMotors) {
        m_logger->logError("Invalid motor index: " + String(config.index),
                           LogModule::MOTOR_MANAGER);
        return false;
    }


    // Validate pin numbers
    if (ValidatePinNumbers(config.limitMinPin,
                           config.index,
                           "Invalid limit min pin for motor "))
        return false;

    // Validate pin numbers
    if (ValidatePinNumbers(config.limitMaxPin,
                           config.index,
                           "Invalid limit max pin for motor "))
        return false;

    // Get GPIO manager instance
    GPIOManager* gpioManager = GPIOManager::getInstance(m_logger);
    if (gpioManager == nullptr) {
        m_logger->logError("GPIO Manager not available for motor manager",
                           LogModule::MOTOR_MANAGER);
        return false;
    }


    if (gpioAllocatePin(config.limitMinPin,
                        config.index,
                        PinMode::INPUT_PULLUP_PIN,
                        "LimitMin",
                        "Failed to allocate limit min pin: ",
                        gpioManager))
        return false;

    if (gpioAllocatePin(config.limitMaxPin,
                        config.index,
                        PinMode::INPUT_PULLUP_PIN,
                        "LimitMax",
                        "Failed to allocate limit max pin: ",
                        gpioManager))
        return false;

    // Create new motor
    Motor* motor = new Motor(config, m_logger);

    // Initialize the motor
    if (!motor->initialize()) {
        m_logger->logError(
            "Failed to initialize motor " + String(config.index),
            LogModule::MOTOR_MANAGER);
        delete motor;
        return false;
    }

    // If there's already a motor at this index, replace it
    if (m_motors[config.index] != nullptr) {
        m_logger->logInfo(
            "Replacing existing motor at index " + String(config.index),
            LogModule::MOTOR_MANAGER);
        delete m_motors[config.index];
    } else {
        // Increment motor count only if this is a new slot
        m_motorCount++;
    }

    // Store motor
    m_motors[config.index] = motor;

    m_logger->logInfo("Motor " + String(config.index) + " added successfully",
                      LogModule::MOTOR_MANAGER);

    return true;
}

bool MotorManager::gpioAllocatePin(uint8_t       pin,
                                   uint8_t       index,
                                   PinMode       mode,
                                   const String& owner,
                                   const String& errStr,
                                   GPIOManager*  gpioManager) {
    if (!gpioManager->allocatePin(
            pin, mode, "Motor" + String(index) + owner)) {
        m_logger->logError("Motor" + String(index) + errStr + String(pin),
                           LogModule::MOTOR_MANAGER);
        return false;
    }
    return true;
}

bool MotorManager::ValidatePinNumbers(uint8_t       pin,
                                      uint8_t       index,
                                      const String& errStr) {
    if (pin != 0xFF && pin > 39) {
        m_logger->logError(errStr + String(index) + ": " + String(pin),
                           LogModule::MOTOR_MANAGER);
        return false;
    }
    return true;
}

Motor* MotorManager::getMotor(uint8_t index) {
    if (index < m_maxMotors) {
        return m_motors[index];
    }
    return nullptr;
}

uint8_t MotorManager::getMotorCount() const { return m_motorCount; }

void MotorManager::enableAllMotors() {
    for (uint8_t i = 0; i < m_maxMotors; i++) {
        if (m_motors[i] != nullptr) {
            m_motors[i]->enable();
        }
    }
}

void MotorManager::disableAllMotors() {
    for (uint8_t i = 0; i < m_maxMotors; i++) {
        if (m_motors[i] != nullptr) {
            m_motors[i]->disable();
        }
    }
}

void MotorManager::stopAllMotors(bool emergency) {
    for (uint8_t i = 0; i < m_maxMotors; i++) {
        if (m_motors[i] != nullptr) {
            if (emergency) {
                m_motors[i]->emergencyStop();
            } else {
                m_motors[i]->abort();
            }
        }
    }
}

bool MotorManager::loadFromEEPROM() {
    // Load motor parameters for each motor
    for (uint8_t i = 0; i < m_maxMotors; i++) {
        if (m_motors[i] != nullptr) {
            // Load PID parameters
            float kp, ki, kd, ff;
            if (m_eepromManager->loadPIDParameters(i, kp, ki, kd, ff)) {
                m_motors[i]->setPIDParameters(kp, ki, kd, ff);
            }

            // Load soft limits
            int32_t minLimit, maxLimit;
            bool    limitsEnabled;
            if (m_eepromManager->loadSoftLimits(
                    i, minLimit, maxLimit, limitsEnabled)) {
                m_motors[i]->setSoftLimits(minLimit, maxLimit, limitsEnabled);
            }
        }
    }

    return true;
}

bool MotorManager::saveToEEPROM() {
    // Save motor parameters for each motor
    for (uint8_t i = 0; i < m_maxMotors; i++) {
        if (m_motors[i] != nullptr) {
            // Get PID parameters
            PIDController* pid = m_motors[i]->getPIDController();
            m_eepromManager->savePIDParameters(i,
                                               pid->getProportionalTerm(),
                                               pid->getIntegralTerm(),
                                               pid->getDerivativeTerm(),
                                               pid->getFeedForwardTerm());

            // Save soft limits (would need to add a method
            // to Motor to get these values) For now, just a
            // placeholder m_eepromManager.saveSoftLimits(i,
            // minLimit, maxLimit, limitsEnabled);
        }
    }

    return m_eepromManager->commit();
}

bool MotorManager::moveMultipleMotors(const int32_t* positions,
                                      const uint8_t* motorIndices,
                                      uint8_t        motorCount,
                                      float          maxVelocity,
                                      float          acceleration,
                                      float          deceleration) {
    // Validate parameters
    if (positions == nullptr || motorIndices == nullptr || motorCount == 0) {
        return false;
    }

    // Calculate synchronized move duration
    float syncDuration = calculateSynchronizedMove(positions,
                                                   motorIndices,
                                                   motorCount,
                                                   maxVelocity,
                                                   acceleration,
                                                   deceleration);

    if (syncDuration <= 0.0f) {
        return false;
    }

    // Start the move for each motor
    for (uint8_t i = 0; i < motorCount; i++) {
        uint8_t index = motorIndices[i];

        if (index < m_maxMotors && m_motors[index] != nullptr) {
            // Scale velocity and acceleration for this
            // motor based on distance
            Motor*  motor    = m_motors[index];
            int32_t distance = abs(positions[i] - motor->getCurrentPosition());

            // If no movement needed for this motor, skip
            if (distance == 0) {
                continue;
            }

            // Calculate velocity needed to complete in
            // syncDuration Trapezoidal profile: s = v_max *
            // t - (a * t²) / 2 Solve for v_max
            float motorVelocity = distance / syncDuration;

            // Cap to maximum allowed velocity
            if (motorVelocity > maxVelocity) {
                motorVelocity = maxVelocity;
            }

            // Move the motor
            motor->moveToPosition(
                positions[i], motorVelocity, acceleration, deceleration);
        }
    }

    return true;
}

bool MotorManager::areMotorsIdle(const uint8_t* motorIndices,
                                 uint8_t        motorCount) {
    // Check each specified motor
    for (uint8_t i = 0; i < motorCount; i++) {
        uint8_t index = motorIndices[i];

        if (index < m_maxMotors && m_motors[index] != nullptr) {
            if (m_motors[index]->isMoving()) {
                return false;
            }
        }
    }

    return true;
}

float MotorManager::calculateSynchronizedMove(const int32_t* positions,
                                              const uint8_t* motorIndices,
                                              uint8_t        motorCount,
                                              float          maxVelocity,
                                              float          acceleration,
                                              float          deceleration) {
    float maxDuration = 0.0f;

    // Find the motor that will take the longest time
    for (uint8_t i = 0; i < motorCount; i++) {
        uint8_t index = motorIndices[i];

        if (index < m_maxMotors && m_motors[index] != nullptr) {
            Motor*  motor    = m_motors[index];
            int32_t distance = abs(positions[i] - motor->getCurrentPosition());

            // If no movement needed for this motor, skip
            if (distance == 0) {
                continue;
            }

            // Calculate time required for this move
            float moveTime = MathUtils::calculateTrapezoidalProfileDuration(
                static_cast<float>(distance), maxVelocity, acceleration);

            // Keep track of the longest duration
            if (moveTime > maxDuration) {
                maxDuration = moveTime;
            }
        }
    }

    return maxDuration;
}

bool MotorManager::hasErrors() {
    for (uint8_t i = 0; i < m_maxMotors; i++) {
        if (m_motors[i] != nullptr) {
            if (m_motors[i]->checkErrors() != MotorError::NONE) {
                return true;
            }
        }
    }

    return false;
}

void MotorManager::clearAllErrors() {
    for (uint8_t i = 0; i < m_maxMotors; i++) {
        if (m_motors[i] != nullptr) {
            m_motors[i]->clearError();
        }
    }
}

void MotorManager::resetAllEncoders() {
    for (uint8_t i = 0; i < m_maxMotors; i++) {
        if (m_motors[i] != nullptr) {
            m_motors[i]->resetPosition();
        }
    }
}
// End of Code