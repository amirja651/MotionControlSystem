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
            if (!addMotor(DEFAULT_MOTOR_CONFIGS[i])) {
                return false;
            }
        }
    }

    // Load saved parameters if available
    loadFromEEPROM();

    m_logger->logDebug(
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

    if (config.stepPin != 0xFF && config.stepPin > 39) {
        m_logger->logError("Invalid limit step pin for motor "
                               + String(config.index) + ": "
                               + String(config.stepPin),
                           LogModule::MOTOR_MANAGER);
        return false;
    }

    if (config.dirPin != 0xFF && config.dirPin > 39) {
        m_logger->logError("Invalid limit dir pin for motor "
                               + String(config.index) + ": "
                               + String(config.dirPin),
                           LogModule::MOTOR_MANAGER);
        return false;
    }

    if (config.enablePin != 0xFF && config.enablePin > 39) {
        m_logger->logError("Invalid limit enable pin for motor "
                               + String(config.index) + ": "
                               + String(config.enablePin),
                           LogModule::MOTOR_MANAGER);
        return false;
    }

    if (config.encoderAPin != 0xFF && config.encoderAPin > 39) {
        m_logger->logError("Invalid limit encoder A pin for motor "
                               + String(config.index) + ": "
                               + String(config.encoderAPin),
                           LogModule::MOTOR_MANAGER);
        return false;
    }

    if (config.encoderBPin != 0xFF && config.encoderBPin > 39) {
        m_logger->logError("Invalid limit encoder B pin for motor "
                               + String(config.index) + ": "
                               + String(config.encoderBPin),
                           LogModule::MOTOR_MANAGER);
        return false;
    }

    if (config.limitMinPin != 0xFF && config.limitMinPin > 39) {
        m_logger->logError("Invalid limit min pin for motor "
                               + String(config.index) + ": "
                               + String(config.limitMinPin),
                           LogModule::MOTOR_MANAGER);
        return false;
    }

    if (config.limitMaxPin != 0xFF && config.limitMaxPin > 39) {
        m_logger->logError("Invalid limit max pin for motor "
                               + String(config.index) + ": "
                               + String(config.limitMaxPin),
                           LogModule::MOTOR_MANAGER);
        return false;
    }

    // Get GPIO manager instance
    GPIOManager* gpioManager = GPIOManager::getInstance(m_logger);
    if (gpioManager == nullptr) {
        m_logger->logError("GPIO Manager not available for motor manager",
                           LogModule::MOTOR_MANAGER);
        return false;
    }

    // Attempt to allocate pin for analog input
    if (!gpioManager->allocatePin(config.stepPin,
                                  PinMode::OUTPUT_PIN,
                                  "Motor" + String(config.index) + "_Step")) {
        return false;
    }

    if (!gpioManager->allocatePin(config.dirPin,
                                  PinMode::OUTPUT_PIN,
                                  "Motor" + String(config.index) + "_Dir")) {
        return false;
    }

    if (config.enablePin != 0xFF) {
        if (!gpioManager->allocatePin(
                config.enablePin,
                PinMode::OUTPUT_PIN,
                "Motor" + String(config.index) + "_Enable")) {
            return false;
        }
    } else {
        m_logger->logWarning("No enable pin, motor" + String(config.index)
                                 + " operate witout enable pin",
                             LogModule::MOTOR_MANAGER);
    }

    if (config.encoderAPin != 0xFF) {
        if (!gpioManager->allocatePin(
                config.encoderAPin,
                PinMode::INPUT_PULLUP_PIN,
                "Motor" + String(config.index) + "_EncoderA")) {
            return false;
        }
    }

    if (config.encoderBPin != 0xFF) {
        if (!gpioManager->allocatePin(
                config.encoderBPin,
                PinMode::INPUT_PULLUP_PIN,
                "Motor" + String(config.index) + "_EncoderB")) {
            return false;
        }
    }

    if (config.limitMinPin != 0xFF) {
        if (!gpioManager->allocatePin(
                config.limitMinPin,
                PinMode::INPUT_PULLUP_PIN,
                "Motor" + String(config.index) + "_LimitMin")) {
            return false;
        }
    } else {
        m_logger->logWarning("No physical limit sensor (Min), motor"
                                 + String(config.index)
                                 + " moveoperate witout limit sensor",
                             LogModule::MOTOR_MANAGER);
    }

    if (config.limitMaxPin != 0xFF) {
        if (!gpioManager->allocatePin(
                config.limitMaxPin,
                PinMode::INPUT_PULLUP_PIN,
                "Motor" + String(config.index) + "_LimitMax")) {
            return false;
        }
    } else {
        m_logger->logWarning("No physical limit sensor (Max), motor"
                                 + String(config.index)
                                 + " operate witout limit sensor",
                             LogModule::MOTOR_MANAGER);
    }

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