/*
 * ESP32 High-Precision Motion Control System
 * Stepper Motor Driver Implementation
 */

#include "StepperDriver.h"

// Static pointer for timer interrupt callback
static StepperDriver* s_activeDriver = nullptr;

// Timer ISR
static void IRAM_ATTR stepperTimerISR() {
    if (s_activeDriver != nullptr) {
        s_activeDriver->handleTimerInterrupt();
    }
}

StepperDriver::StepperDriver(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, bool invertDir,
                             bool invertEnable)
    : m_stepPin(stepPin),
      m_dirPin(dirPin),
      m_enablePin(enablePin),
      m_invertDir(invertDir),
      m_invertEnable(invertEnable),
      m_enabled(false),
      m_direction(true),
      m_speed(0.0f),
      m_position(0),
      m_targetPosition(0),
      m_isMoving(false),
      m_microstepMode(MicrostepMode::SIXTEENTH_STEP),
      m_microsteps(16),
      m_maxStepsPerSecond(CONFIG_DEFAULT_MAX_VELOCITY),
      m_stepIntervalUs(0),
      m_lastStepTimeUs(0),
      m_stepsToGo(0),
      m_timerManager(nullptr),
      m_usingTimer(false) {
}

StepperDriver::~StepperDriver() {
    // Ensure motor is stopped and timer is disabled
    stop(true);

    // Remove this driver as the active driver if it is
    if (s_activeDriver == this) {
        s_activeDriver = nullptr;
    }
}

bool StepperDriver::initialize() {
    // Configure pins
    pinMode(m_stepPin, OUTPUT);
    pinMode(m_dirPin, OUTPUT);
    pinMode(m_enablePin, OUTPUT);

    // Initialize pins to default state
    digitalWrite(m_stepPin, LOW);
    digitalWrite(m_dirPin, m_invertDir ? HIGH : LOW);        // Default to forward direction
    digitalWrite(m_enablePin, m_invertEnable ? LOW : HIGH);  // Default to disabled

    // Get timer manager
    m_timerManager = TimerManager::getInstance();

    // Set this as the active driver for timer callbacks
    s_activeDriver = this;

    return true;
}

void StepperDriver::enable() {
    digitalWrite(m_enablePin, m_invertEnable ? HIGH : LOW);
    m_enabled = true;
}

void StepperDriver::disable() {
    stop(true);  // Stop any ongoing movement
    digitalWrite(m_enablePin, m_invertEnable ? LOW : HIGH);
    m_enabled = false;
}

bool StepperDriver::isEnabled() const {
    return m_enabled;
}

void StepperDriver::setDirection(bool direction) {
    m_direction = direction;
    digitalWrite(m_dirPin, (direction ^ m_invertDir) ? HIGH : LOW);
}

void StepperDriver::setSpeed(float speed) {
    // Constrain speed to maximum
    if (fabs(speed) > m_maxStepsPerSecond) {
        speed = speed > 0 ? m_maxStepsPerSecond : -m_maxStepsPerSecond;
    }

    // Update direction if speed sign changed
    bool newDirection = speed >= 0;
    if (newDirection != m_direction) {
        setDirection(newDirection);
    }

    // Store absolute speed
    m_speed = fabs(speed);

    // Calculate step interval
    m_stepIntervalUs = calculateStepInterval(m_speed);

    // If we're moving but not in timer mode, start the timer
    if (m_isMoving && !m_usingTimer && m_speed > 0.0f) {
        startTimer(m_stepIntervalUs);
    }
}

int32_t StepperDriver::step(int32_t steps) {
    // If steps is 0, do nothing
    if (steps == 0) {
        return 0;
    }

    // Set direction based on sign of steps
    setDirection(steps > 0);

    // Store absolute steps to move
    m_stepsToGo = abs(steps);

    // Start moving
    m_isMoving = true;

    // If speed is set, start the timer for continuous stepping
    if (m_speed > 0.0f) {
        startTimer(m_stepIntervalUs);
    } else {
        // For manual stepping without timer
        for (int32_t i = 0; i < abs(steps); i++) {
            pulseStep();
            delayMicroseconds(CONFIG_MIN_STEP_INTERVAL_US);
        }
        m_isMoving = false;
    }

    return steps;  // Return requested steps, actual steps will be handled by timer
}

bool StepperDriver::moveTo(int32_t position) {
    // Calculate steps to move
    int32_t stepsToMove = position - m_position;

    // If no steps needed, return success
    if (stepsToMove == 0) {
        return true;
    }

    // Store target position
    m_targetPosition = position;

    // Move the calculated steps
    step(stepsToMove);

    return true;
}

void StepperDriver::setOutput(float output) {
    // For stepper motors, output (-1.0 to 1.0) maps to speed and direction
    float speed = fabs(output) * m_maxStepsPerSecond;
    setSpeed(output < 0 ? -speed : speed);
}

void StepperDriver::stop(bool emergency) {
    // Stop timer
    stopTimer();

    // Reset movement state
    m_isMoving = false;
    m_stepsToGo = 0;

    // If not emergency stop, we would typically decelerate here
    // For simplicity, we just stop immediately for now
}

bool StepperDriver::isMoving() const {
    return m_isMoving;
}

DriverType StepperDriver::getType() const {
    return DriverType::STEPPER;
}

int32_t StepperDriver::getCurrentPosition() const {
    return m_position;
}

bool StepperDriver::setCurrentPosition(int32_t position) {
    m_position = position;
    m_targetPosition = position;  // Reset target to current position
    return true;
}

bool StepperDriver::checkFault() {
    // Basic stepper drivers don't have fault detection
    return false;
}

bool StepperDriver::clearFault() {
    // Basic stepper drivers don't have fault detection
    return true;
}

String StepperDriver::getConfig() const {
    // Return configuration as a JSON-formatted string
    String config = "{";
    config += "\"type\":\"stepper\",";
    config += "\"stepPin\":" + String(m_stepPin) + ",";
    config += "\"dirPin\":" + String(m_dirPin) + ",";
    config += "\"enablePin\":" + String(m_enablePin) + ",";
    config += "\"invertDir\":" + String(m_invertDir ? "true" : "false") + ",";
    config += "\"invertEnable\":" + String(m_invertEnable ? "true" : "false") + ",";
    config += "\"microsteps\":" + String(m_microsteps) + ",";
    config += "\"maxSpeed\":" + String(m_maxStepsPerSecond);
    config += "}";
    return config;
}

bool StepperDriver::setConfig(const String& config) {
    // Parse JSON configuration and apply settings
    // This is a simplified implementation without full JSON parsing

    // Extract microstepping value
    int microstepsStart = config.indexOf("\"microsteps\":") + 13;
    if (microstepsStart > 13) {
        int microstepsEnd = config.indexOf(",", microstepsStart);
        if (microstepsEnd < 0) {
            microstepsEnd = config.indexOf("}", microstepsStart);
        }

        if (microstepsEnd > microstepsStart) {
            String microstepsStr = config.substring(microstepsStart, microstepsEnd);
            int microsteps = microstepsStr.toInt();

            // Set microstepping mode
            if (microsteps == 1)
                setMicrostepMode(MicrostepMode::FULL_STEP);
            else if (microsteps == 2)
                setMicrostepMode(MicrostepMode::HALF_STEP);
            else if (microsteps == 4)
                setMicrostepMode(MicrostepMode::QUARTER_STEP);
            else if (microsteps == 8)
                setMicrostepMode(MicrostepMode::EIGHTH_STEP);
            else if (microsteps == 16)
                setMicrostepMode(MicrostepMode::SIXTEENTH_STEP);
            else if (microsteps == 32)
                setMicrostepMode(MicrostepMode::THIRTYSECOND_STEP);
        }
    }

    // Extract max speed
    int maxSpeedStart = config.indexOf("\"maxSpeed\":") + 11;
    if (maxSpeedStart > 11) {
        int maxSpeedEnd = config.indexOf(",", maxSpeedStart);
        if (maxSpeedEnd < 0) {
            maxSpeedEnd = config.indexOf("}", maxSpeedStart);
        }

        if (maxSpeedEnd > maxSpeedStart) {
            String maxSpeedStr = config.substring(maxSpeedStart, maxSpeedEnd);
            float maxSpeed = maxSpeedStr.toFloat();

            if (maxSpeed > 0) {
                setMaxStepsPerSecond(maxSpeed);
            }
        }
    }

    return true;
}

bool StepperDriver::setMicrostepMode(MicrostepMode mode) {
    m_microstepMode = mode;

    // Convert enum to actual microsteps
    switch (mode) {
        case MicrostepMode::FULL_STEP:
            m_microsteps = 1;
            break;
        case MicrostepMode::HALF_STEP:
            m_microsteps = 2;
            break;
        case MicrostepMode::QUARTER_STEP:
            m_microsteps = 4;
            break;
        case MicrostepMode::EIGHTH_STEP:
            m_microsteps = 8;
            break;
        case MicrostepMode::SIXTEENTH_STEP:
            m_microsteps = 16;
            break;
        case MicrostepMode::THIRTYSECOND_STEP:
            m_microsteps = 32;
            break;
    }

    // Hardware configuration for microstepping would go here
    // This depends on the specific stepper driver IC being used

    return true;
}

MicrostepMode StepperDriver::getMicrostepMode() const {
    return m_microstepMode;
}

void StepperDriver::setMaxStepsPerSecond(float maxStepsPerSecond) {
    m_maxStepsPerSecond = maxStepsPerSecond;
}

float StepperDriver::getMaxStepsPerSecond() const {
    return m_maxStepsPerSecond;
}

void StepperDriver::handleTimerInterrupt() {
    // This is called from the timer ISR
    if (!m_isMoving || m_stepsToGo <= 0) {
        // Stop if we're done moving
        stopTimer();
        m_isMoving = false;
        return;
    }

    // Generate a step pulse
    pulseStep();

    // Update steps remaining
    m_stepsToGo--;

    // Update position based on direction
    if (m_direction) {
        m_position++;
    } else {
        m_position--;
    }

    // Stop if we've reached the target
    if (m_stepsToGo <= 0) {
        stopTimer();
        m_isMoving = false;
    }
}

void StepperDriver::pulseStep() {
    // Generate a step pulse
    digitalWrite(m_stepPin, HIGH);
    // Need a short delay to ensure the pulse is seen by the driver
    delayMicroseconds(1);  // 1us should be sufficient for most drivers
    digitalWrite(m_stepPin, LOW);

    // Record time of step
    m_lastStepTimeUs = micros();
}

uint32_t StepperDriver::calculateStepInterval(float speed) {
    // Calculate step interval in microseconds
    if (speed <= 0.0f) {
        return UINT32_MAX;  // Very large interval for effectively no movement
    }

    // Convert steps per second to microseconds per step
    uint32_t intervalUs = static_cast<uint32_t>(1000000.0f / speed);

    // Enforce minimum step interval
    if (intervalUs < CONFIG_MIN_STEP_INTERVAL_US) {
        intervalUs = CONFIG_MIN_STEP_INTERVAL_US;
    }

    return intervalUs;
}

bool StepperDriver::startTimer(uint32_t intervalUs) {
    // If no timer manager, can't use timer
    if (m_timerManager == nullptr) {
        return false;
    }

    // If already using timer, stop it first
    if (m_usingTimer) {
        stopTimer();
    }

    // Register the timer callback
    bool success = m_timerManager->startTimer(CONFIG_STEP_TIMER, intervalUs,
                                              true,  // Auto-reload
                                              stepperTimerISR);

    if (success) {
        m_usingTimer = true;
    }

    return success;
}

void StepperDriver::stopTimer() {
    if (m_usingTimer && m_timerManager != nullptr) {
        m_timerManager->stopTimer(CONFIG_STEP_TIMER);
        m_usingTimer = false;
    }
}