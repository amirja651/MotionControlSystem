/*
 * ESP32 High-Precision Motion Control System
 * Motor Manager Implementation
 */

#include "MotorManager.h"

MotorManager::MotorManager(uint8_t maxMotors)
    : m_maxMotors(maxMotors > 0 ? maxMotors : CONFIG_MAX_MOTORS)
    , m_motorCount(0)
{
    // Allocate memory for motor pointers
    m_motors = new Motor*[m_maxMotors];
    
    // Initialize all pointers to nullptr
    for (uint8_t i = 0; i < m_maxMotors; i++) {
        m_motors[i] = nullptr;
    }
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
}

bool MotorManager::initialize() {
    // Initialize EEPROM manager
    if (!m_eepromManager.initialize()) {
        return false;
    }
    
    // If there are predefined motor configurations, add them
    for (uint8_t i = 0; i < CONFIG_MAX_MOTORS; i++) {
        if (i < m_maxMotors) {
            addMotor(DEFAULT_MOTOR_CONFIGS[i]);
        }
    }
    
    // Load saved parameters if available
    loadFromEEPROM();
    
    return true;
}

bool MotorManager::addMotor(const MotorConfig& config) {
    // Check if we have room for another motor
    if (m_motorCount >= m_maxMotors) {
        return false;
    }
    
    // Check if motor index is valid
    if (config.index >= m_maxMotors) {
        return false;
    }
    
    // Create new motor
    Motor* motor = new Motor(config);
    
    // Initialize the motor
    if (!motor->initialize()) {
        delete motor;
        return false;
    }
    
    // If there's already a motor at this index, replace it
    if (m_motors[config.index] != nullptr) {
        delete m_motors[config.index];
    } else {
        // Increment motor count only if this is a new slot
        m_motorCount++;
    }
    
    // Store motor
    m_motors[config.index] = motor;
    
    return true;
}

Motor* MotorManager::getMotor(uint8_t index) {
    if (index < m_maxMotors) {
        return m_motors[index];
    }
    return nullptr;
}

uint8_t MotorManager::getMotorCount() const {
    return m_motorCount;
}

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
            if (m_eepromManager.loadPIDParameters(i, kp, ki, kd, ff)) {
                m_motors[i]->setPIDParameters(kp, ki, kd, ff);
            }
            
            // Load soft limits
            int32_t minLimit, maxLimit;
            bool limitsEnabled;
            if (m_eepromManager.loadSoftLimits(i, minLimit, maxLimit, limitsEnabled)) {
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
            PIDController& pid = m_motors[i]->getPIDController();
            m_eepromManager.savePIDParameters(i, 
                                              pid.getProportionalTerm(), 
                                              pid.getIntegralTerm(), 
                                              pid.getDerivativeTerm(), 
                                              pid.getFeedForwardTerm());
            
            // Save soft limits (would need to add a method to Motor to get these values)
            // For now, just a placeholder
            //m_eepromManager.saveSoftLimits(i, minLimit, maxLimit, limitsEnabled);
        }
    }
    
    return m_eepromManager.commit();
}

bool MotorManager::moveMultipleMotors(const int32_t* positions, const uint8_t* motorIndices, 
                                     uint8_t motorCount, float maxVelocity, float acceleration, 
                                     float deceleration) {
    // Validate parameters
    if (positions == nullptr || motorIndices == nullptr || motorCount == 0) {
        return false;
    }
    
    // Calculate synchronized move duration
    float syncDuration = calculateSynchronizedMove(positions, motorIndices, motorCount, 
                                                 maxVelocity, acceleration, deceleration);
    
    if (syncDuration <= 0.0f) {
        return false;
    }
    
    // Start the move for each motor
    for (uint8_t i = 0; i < motorCount; i++) {
        uint8_t index = motorIndices[i];
        
        if (index < m_maxMotors && m_motors[index] != nullptr) {
            // Scale velocity and acceleration for this motor based on distance
            Motor* motor = m_motors[index];
            int32_t distance = abs(positions[i] - motor->getCurrentPosition());
            
            // If no movement needed for this motor, skip
            if (distance == 0) {
                continue;
            }
            
            // Calculate velocity needed to complete in syncDuration
            // Trapezoidal profile: s = v_max * t - (a * t²) / 2
            // Solve for v_max
            float motorVelocity = distance / syncDuration;
            
            // Cap to maximum allowed velocity
            if (motorVelocity > maxVelocity) {
                motorVelocity = maxVelocity;
            }
            
            // Move the motor
            motor->moveToPosition(positions[i], motorVelocity, acceleration, deceleration);
        }
    }
    
    return true;
}

bool MotorManager::areMotorsIdle(const uint8_t* motorIndices, uint8_t motorCount) {
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

float MotorManager::calculateSynchronizedMove(const int32_t* positions, const uint8_t* motorIndices, 
                                            uint8_t motorCount, float maxVelocity, float acceleration, 
                                            float deceleration) {
    float maxDuration = 0.0f;
    
    // Find the motor that will take the longest time
    for (uint8_t i = 0; i < motorCount; i++) {
        uint8_t index = motorIndices[i];
        
        if (index < m_maxMotors && m_motors[index] != nullptr) {
            Motor* motor = m_motors[index];
            int32_t distance = abs(positions[i] - motor->getCurrentPosition());
            
            // If no movement needed for this motor, skip
            if (distance == 0) {
                continue;
            }
            
            // Calculate time required for this move
            float moveTime = MathUtils::calculateTrapezoidalProfileDuration(
                static_cast<float>(distance), 
                maxVelocity, 
                acceleration
            );
            
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