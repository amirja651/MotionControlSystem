/*
 * ESP32 High-Precision Motion Control System
 * Safety Monitor Implementation
 */

#include "SafetyMonitor.h"

SafetyMonitor::SafetyMonitor(MotorManager* motorManager, Logger* logger)
    : m_motorManager(motorManager),
      m_logger(logger),
      m_status(SystemSafetyStatus::NORMAL),
      m_lastCode(SafetyCode::NONE),
      m_emergencyStopActive(false),
      m_warningCount(0),
      m_errorCount(0),
      m_emergencyStopCount(0),
      m_emergencyStopPin(0xFF),
      m_emergencyStopActiveLevel(LOW),
      m_emergencyStopPinConfigured(false),
      m_positionWarningThreshold(50),
      m_positionErrorThreshold(CONFIG_SAFETY_POSITION_TOLERANCE),
      m_velocityWarningThreshold(50.0f),
      m_velocityErrorThreshold(CONFIG_SAFETY_VELOCITY_TOLERANCE),
      m_temperatureWarningThreshold(CONFIG_SAFETY_MAX_TEMPERATURE_C - 10.0f),
      m_temperatureErrorThreshold(CONFIG_SAFETY_MAX_TEMPERATURE_C),
      m_minVoltage(CONFIG_VOLTAGE_MIN),
      m_maxVoltage(CONFIG_VOLTAGE_MAX),
      m_voltageWarningThreshold(CONFIG_VOLTAGE_WARNING_THRESHOLD),
      m_lastCheckTimeMs(0) {
}

bool SafetyMonitor::initialize() {
    // Initialize emergency stop pin if configured
    if (m_emergencyStopPinConfigured && m_emergencyStopPin != 0xFF) {
        pinMode(m_emergencyStopPin, INPUT_PULLUP);
    }

    // Log initialization
    if (m_logger != nullptr) {
        m_logger->logInfo("Safety monitor initialized");
    }

    // Set initial check time
    m_lastCheckTimeMs = millis();

    return true;
}

SystemSafetyStatus SafetyMonitor::checkSafety() {
    // Update check time
    uint32_t currentTimeMs = millis();
    uint32_t elapsedTimeMs = currentTimeMs - m_lastCheckTimeMs;
    m_lastCheckTimeMs = currentTimeMs;

    // If emergency stop is already active, don't need to check further
    if (m_emergencyStopActive) {
        return SystemSafetyStatus::EMERGENCY_STOP;
    }

    // Check emergency stop button first (highest priority)
    if (checkEmergencyStopButton()) {
        triggerEmergencyStop(SafetyCode::EMERGENCY_STOP_PRESSED);
        return m_status;
    }

    // Check motor safety
    SafetyCode motorSafetyCode = checkMotorSafety();
    if (motorSafetyCode != SafetyCode::NONE) {
        if (motorSafetyCode <= SafetyCode::POWER_SUPPLY_FLUCTUATION) {
            // Warning
            m_status = SystemSafetyStatus::WARNING;
            m_lastCode = motorSafetyCode;
            m_warningCount++;

            logSafetyEvent(m_status, motorSafetyCode);
        } else {
            // Error
            m_status = SystemSafetyStatus::ERROR;
            m_lastCode = motorSafetyCode;
            m_errorCount++;

            logSafetyEvent(m_status, motorSafetyCode);

            // For critical errors, trigger emergency stop
            if (motorSafetyCode == SafetyCode::POSITION_ERROR ||
                motorSafetyCode == SafetyCode::LIMIT_SWITCH_TRIGGERED) {
                triggerEmergencyStop(motorSafetyCode);
            }
        }

        return m_status;
    }

    // Check system conditions
    SafetyCode systemSafetyCode = checkSystemConditions();
    if (systemSafetyCode != SafetyCode::NONE) {
        if (systemSafetyCode <= SafetyCode::POWER_SUPPLY_FLUCTUATION) {
            // Warning
            m_status = SystemSafetyStatus::WARNING;
            m_lastCode = systemSafetyCode;
            m_warningCount++;

            logSafetyEvent(m_status, systemSafetyCode);
        } else {
            // Error
            m_status = SystemSafetyStatus::ERROR;
            m_lastCode = systemSafetyCode;
            m_errorCount++;

            logSafetyEvent(m_status, systemSafetyCode);

            // For critical errors, trigger emergency stop
            if (systemSafetyCode == SafetyCode::CRITICAL_TEMPERATURE ||
                systemSafetyCode == SafetyCode::POWER_FAILURE) {
                triggerEmergencyStop(systemSafetyCode);
            }
        }

        return m_status;
    }

    // If we get here, everything is normal
    if (m_status != SystemSafetyStatus::NORMAL) {
        m_status = SystemSafetyStatus::NORMAL;
        m_lastCode = SafetyCode::NONE;

        // Log return to normal
        if (m_logger != nullptr) {
            m_logger->logInfo("System safety status: Normal");
        }
    }

    return m_status;
}

void SafetyMonitor::triggerEmergencyStop(SafetyCode reason) {
    // If already in emergency stop, don't do anything
    if (m_emergencyStopActive) {
        return;
    }

    // Set emergency stop flag
    m_emergencyStopActive = true;

    // Set status and code
    m_status = SystemSafetyStatus::EMERGENCY_STOP;
    m_lastCode = reason;

    // Update statistics
    m_emergencyStopCount++;

    // Log emergency stop
    logSafetyEvent(m_status, reason);

    // Stop all motors
    if (m_motorManager != nullptr) {
        m_motorManager->stopAllMotors(true);
    }
}

bool SafetyMonitor::resetEmergencyStop() {
    // If not in emergency stop, nothing to do
    if (!m_emergencyStopActive) {
        return true;
    }

    // Check if emergency stop button is still pressed
    if (checkEmergencyStopButton()) {
        // Can't reset while button is pressed
        return false;
    }

    // Reset emergency stop flag
    m_emergencyStopActive = false;

    // Set status back to normal
    m_status = SystemSafetyStatus::NORMAL;
    m_lastCode = SafetyCode::NONE;

    // Log reset
    if (m_logger != nullptr) {
        m_logger->logInfo("Emergency stop reset");
    }

    return true;
}

bool SafetyMonitor::isEmergencyStop() const {
    return m_emergencyStopActive;
}

SystemSafetyStatus SafetyMonitor::getStatus() const {
    return m_status;
}

SafetyCode SafetyMonitor::getLastSafetyCode() const {
    return m_lastCode;
}

void SafetyMonitor::setEmergencyStopPin(uint8_t pin, uint8_t activeLevel) {
    m_emergencyStopPin = pin;
    m_emergencyStopActiveLevel = activeLevel;
    m_emergencyStopPinConfigured = true;

    // Configure pin
    if (m_emergencyStopPin != 0xFF) {
        pinMode(m_emergencyStopPin, INPUT_PULLUP);
    }
}

void SafetyMonitor::setPositionTolerances(uint32_t warningThreshold, uint32_t errorThreshold) {
    m_positionWarningThreshold = warningThreshold;
    m_positionErrorThreshold = errorThreshold;
}

void SafetyMonitor::setVelocityTolerances(float warningThreshold, float errorThreshold) {
    m_velocityWarningThreshold = warningThreshold;
    m_velocityErrorThreshold = errorThreshold;
}

void SafetyMonitor::setTemperatureTolerances(float warningThreshold, float errorThreshold) {
    m_temperatureWarningThreshold = warningThreshold;
    m_temperatureErrorThreshold = errorThreshold;
}

void SafetyMonitor::setVoltageTolerances(float minVoltage, float maxVoltage,
                                         float warningThreshold) {
    m_minVoltage = minVoltage;
    m_maxVoltage = maxVoltage;
    m_voltageWarningThreshold = warningThreshold;
}

void SafetyMonitor::getSafetyStats(uint32_t& warningCount, uint32_t& errorCount,
                                   uint32_t& emergencyStopCount) {
    warningCount = m_warningCount;
    errorCount = m_errorCount;
    emergencyStopCount = m_emergencyStopCount;
}

void SafetyMonitor::clearSafetyStats() {
    m_warningCount = 0;
    m_errorCount = 0;
    m_emergencyStopCount = 0;
}

bool SafetyMonitor::checkEmergencyStopButton() {
    // If emergency stop pin not configured, return false
    if (!m_emergencyStopPinConfigured || m_emergencyStopPin == 0xFF) {
        return false;
    }

    // Read pin state
    int pinState = digitalRead(m_emergencyStopPin);

    // Check if button is pressed (active level)
    return pinState == m_emergencyStopActiveLevel;
}

SafetyCode SafetyMonitor::checkMotorSafety() {
    // Check if motor manager is available
    if (m_motorManager == nullptr) {
        return SafetyCode::NONE;
    }

    // Check for errors in any motor
    if (m_motorManager->hasErrors()) {
        // Get the first error motor
        for (uint8_t i = 0; i < m_motorManager->getMotorCount(); i++) {
            Motor* motor = m_motorManager->getMotor(i);
            if (motor != nullptr) {
                MotorError error = motor->checkErrors();

                if (error != MotorError::NONE) {
                    // Convert motor error to safety code
                    switch (error) {
                        case MotorError::POSITION_ERROR:
                            return SafetyCode::POSITION_ERROR;

                        case MotorError::VELOCITY_ERROR:
                            return SafetyCode::VELOCITY_ERROR;

                        case MotorError::LIMIT_SWITCH_TRIGGERED:
                            return SafetyCode::LIMIT_SWITCH_TRIGGERED;

                        case MotorError::DRIVER_FAULT:
                            return SafetyCode::DRIVER_FAULT;

                        case MotorError::ENCODER_ERROR:
                            return SafetyCode::ENCODER_ERROR;

                        default:
                            return SafetyCode::NONE;
                    }
                }
            }
        }
    }

    // Check for position deviations
    for (uint8_t i = 0; i < m_motorManager->getMotorCount(); i++) {
        Motor* motor = m_motorManager->getMotor(i);
        if (motor != nullptr && motor->isEnabled() &&
            motor->getControlMode() == MotorControlMode::POSITION) {
            int32_t positionError = abs(motor->getCurrentPosition() - motor->getTargetPosition());

            if (positionError > m_positionErrorThreshold) {
                return SafetyCode::POSITION_ERROR;
            } else if (positionError > m_positionWarningThreshold) {
                return SafetyCode::POSITION_DEVIATION;
            }
        }
    }

    // Check for velocity deviations
    for (uint8_t i = 0; i < m_motorManager->getMotorCount(); i++) {
        Motor* motor = m_motorManager->getMotor(i);
        if (motor != nullptr && motor->isEnabled() &&
            motor->getControlMode() == MotorControlMode::VELOCITY) {
            float velocityError = fabs(motor->getCurrentVelocity() - motor->getTargetVelocity());

            if (velocityError > m_velocityErrorThreshold) {
                return SafetyCode::VELOCITY_ERROR;
            } else if (velocityError > m_velocityWarningThreshold) {
                return SafetyCode::VELOCITY_DEVIATION;
            }
        }
    }

    return SafetyCode::NONE;
}

SafetyCode SafetyMonitor::checkSystemConditions() {
    // This is a placeholder for system condition checks
    // In a real implementation, you would check things like temperature, voltage, etc.

    // Example: Check temperature (simulated)
    float temperature = 0.0f;

    // On a real system, you would read from a temperature sensor
    // For example: temperature = readTemperatureSensor();

    // Add to temperature buffer for trend analysis
    m_temperatureBuffer.push(temperature);

    // Check temperature limits
    if (temperature > m_temperatureErrorThreshold) {
        return SafetyCode::CRITICAL_TEMPERATURE;
    } else if (temperature > m_temperatureWarningThreshold) {
        return SafetyCode::HIGH_TEMPERATURE;
    }

// Only check voltage if power monitoring is enabled
#if CONFIG_POWER_MONITORING_ENABLED

    // Example: Check voltage (simulated)
    float voltage = 3.3f;

    // On a real system, you would read from a voltage sensor
    voltage = readVoltageSensor();

    // Add to voltage buffer for trend analysis
    m_voltageBuffer.push(voltage);

    // Check voltage limits
    if (voltage < m_minVoltage) {
        return SafetyCode::POWER_FAILURE;
    } else if (voltage > m_maxVoltage) {
        return SafetyCode::POWER_FAILURE;
    }

    // Check voltage fluctuations
    float voltageMin = m_voltageBuffer.minimum();
    float voltageMax = m_voltageBuffer.maximum();

    if (voltageMax - voltageMin > m_voltageWarningThreshold * (m_maxVoltage - m_minVoltage)) {
        return SafetyCode::POWER_SUPPLY_FLUCTUATION;
    }
#endif  // CONFIG_POWER_MONITORING_ENABLED

    return SafetyCode::NONE;
}

void SafetyMonitor::logSafetyEvent(SystemSafetyStatus status, SafetyCode code) {
    if (m_logger == nullptr) {
        return;
    }

    String statusStr;
    switch (status) {
        case SystemSafetyStatus::NORMAL:
            statusStr = "Normal";
            break;
        case SystemSafetyStatus::WARNING:
            statusStr = "Warning";
            break;
        case SystemSafetyStatus::ERROR:
            statusStr = "Error";
            break;
        case SystemSafetyStatus::EMERGENCY_STOP:
            statusStr = "Emergency Stop";
            break;
        default:
            statusStr = "Unknown";
            break;
    }

    String codeStr;
    switch (code) {
        case SafetyCode::NONE:
            codeStr = "None";
            break;
        case SafetyCode::POSITION_DEVIATION:
            codeStr = "Position Deviation";
            break;
        case SafetyCode::VELOCITY_DEVIATION:
            codeStr = "Velocity Deviation";
            break;
        case SafetyCode::HIGH_TEMPERATURE:
            codeStr = "High Temperature";
            break;
        case SafetyCode::POWER_SUPPLY_FLUCTUATION:
            codeStr = "Power Supply Fluctuation";
            break;
        case SafetyCode::POSITION_ERROR:
            codeStr = "Position Error";
            break;
        case SafetyCode::VELOCITY_ERROR:
            codeStr = "Velocity Error";
            break;
        case SafetyCode::LIMIT_SWITCH_TRIGGERED:
            codeStr = "Limit Switch Triggered";
            break;
        case SafetyCode::DRIVER_FAULT:
            codeStr = "Driver Fault";
            break;
        case SafetyCode::ENCODER_ERROR:
            codeStr = "Encoder Error";
            break;
        case SafetyCode::CONTROL_LOOP_TIMING:
            codeStr = "Control Loop Timing";
            break;
        case SafetyCode::CRITICAL_TEMPERATURE:
            codeStr = "Critical Temperature";
            break;
        case SafetyCode::POWER_FAILURE:
            codeStr = "Power Failure";
            break;
        case SafetyCode::COMMUNICATION_TIMEOUT:
            codeStr = "Communication Timeout";
            break;
        case SafetyCode::EMERGENCY_STOP_PRESSED:
            codeStr = "Emergency Stop Pressed";
            break;
        default:
            codeStr = "Unknown";
            break;
    }

    String message = "Safety " + statusStr + ": " + codeStr;

    // Log based on status
    switch (status) {
        case SystemSafetyStatus::WARNING:
            m_logger->logWarning(message);
            break;
        case SystemSafetyStatus::ERROR:
        case SystemSafetyStatus::EMERGENCY_STOP:
            m_logger->logError(message);
            break;
        default:
            m_logger->logInfo(message);
            break;
    }
}

float SafetyMonitor::readVoltageSensor() {
// Skip voltage reading if power monitoring is disabled
#if !CONFIG_POWER_MONITORING_ENABLED
    return (m_minVoltage + m_maxVoltage) / 2.0f;  // Return a safe value in the middle of the range
#else

    // Get GPIO manager instance
    GPIOManager* gpioManager = GPIOManager::getInstance();

    // Validate GPIO manager
    if (gpioManager == nullptr) {
        // Log error or handle initialization failure
        if (m_logger) {
            m_logger->logError("GPIO Manager not available for voltage sensing");
        }
        return 0.0f;
    }

    // Allocate analog input pin if not already allocated
    if (!gpioManager->isPinAvailable(CONFIG_VOLTAGE_SENSE_PIN)) {
        // Attempt to allocate pin for analog input
        if (!gpioManager->allocatePin(CONFIG_VOLTAGE_SENSE_PIN, PinMode::ANALOG_INPUT_PIN,
                                      "VoltageMonitoring")) {
            if (m_logger) {
                m_logger->logError("Failed to allocate voltage sense pin");
            }
            return 0.0f;
        }
    }

    // Read raw analog value
    int rawValue = analogRead(CONFIG_VOLTAGE_SENSE_PIN);

    // Convert raw value to voltage
    // Assumes 12-bit ADC (0-4095) and voltage divider configuration
    float voltage = (rawValue * 3.3f) / 4095.0f;

    // Apply voltage divider scaling factor if needed
    // Example: If using a voltage divider with R1=10k, R2=1k
    // voltage *= (R1 + R2) / R2;

    // Optional: Log voltage reading for debugging
    if (m_logger) {
        m_logger->logDebug("Voltage Sensor: Raw=" + String(rawValue) +
                           ", Voltage=" + String(voltage, 2) + "V");
    }

    return voltage;
#endif  // CONFIG_POWER_MONITORING_ENABLED
}
// End of Code