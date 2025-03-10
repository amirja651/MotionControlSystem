/*
 * ESP32 High-Precision Motion Control System
 * Single Motor Example
 *
 * This file demonstrates a simple configuration for controlling
 * a single stepper motor with the motion control system.
 */

#ifndef SINGLE_MOTOR_EXAMPLE_H
#define SINGLE_MOTOR_EXAMPLE_H

#if EXAMPLE_ENABLED

#include <Arduino.h>

#include "../Configuration.h"
#include "../MotorManager.h"
#include "../SystemManager.h"
#include "../communication/CommandProtocol.h"
#include "../communication/SerialCommand.h"
#include "../core/Motor.h"
#include "../core/PIDController.h"
#include "../core/SafetyMonitor.h"
#include "../hardware/GPIOManager.h"
#include "../hardware/drivers/StepperDriver.h"

/**
 * Single Motor Example class for basic motor control
 */
class SingleMotorExample {
   public:
    /**
     * Constructor
     */
    SingleMotorExample() : m_initialized(false), m_motorIndex(0) {
        // Initialize system components
        m_systemManager = new SystemManager();
    }

    /**
     * Destructor
     */
    ~SingleMotorExample() {
        // Clean up resources
        delete m_systemManager;
    }

    /**
     * Initialize the application
     *
     * @return True if initialization successful, false otherwise
     */
    bool initialize() {
        // Initialize system manager
        if (!m_systemManager->initialize()) {
            Serial.println("Failed to initialize system manager");
            return false;
        }

        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            Serial.println("Motor manager not available");
            return false;
        }

        // Configure motor
        configureMotor(motorManager);

        // Configure safety parameters
        configureSafety();

        // Configure serial commands
        configureCommands();

        // Load configuration from EEPROM
        m_systemManager->loadSystemConfiguration();

        // Store last update time for periodic operations
        m_lastUpdateTimeMs = millis();

        // Set initialization flag
        m_initialized = true;

        // Log initialization
        m_systemManager->getLogger()->logInfo("Single Motor Example initialized");

        return true;
    }

    /**
     * Run the application main loop
     */
    void run() {
        if (!m_initialized) {
            return;
        }

        // Process system updates in main loop
        processSystem();

        // Process application-specific logic
        processApplication();
    }

   private:
    // System components
    SystemManager* m_systemManager;

    // Application state
    bool m_initialized;
    uint32_t m_lastUpdateTimeMs;
    uint8_t m_motorIndex;

    /**
     * Configure motor for this application
     *
     * @param motorManager Pointer to motor manager
     */
    void configureMotor(MotorManager* motorManager) {
        // Single motor configuration
        MotorConfig motorConfig = DEFAULT_MOTOR_CONFIGS[0];
        motorConfig.maxVelocity = 3000.0f;       // 3000 steps/second
        motorConfig.maxAcceleration = 10000.0f;  // 10000 steps/second²
        motorConfig.maxDeceleration = 12000.0f;  // 12000 steps/second²
        motorConfig.maxJerk = 50000.0f;          // 50000 steps/second³
        motorConfig.pidKp = 0.8f;                // P gain for position control
        motorConfig.pidKi = 0.15f;               // I gain for steady-state error
        motorConfig.pidKd = 0.05f;               // D gain for dampening
        motorConfig.pidFf = 0.1f;                // Feed-forward for velocity tracking

        // Add motor to manager
        motorManager->addMotor(motorConfig);

        // Save the motor index for later reference
        m_motorIndex = motorConfig.index;
    }

    /**
     * Configure safety parameters
     */
    void configureSafety() {
        // Get safety monitor
        SafetyMonitor* safetyMonitor = m_systemManager->getSafetyMonitor();
        if (safetyMonitor == nullptr) {
            return;
        }

        // Configure position tolerance
        safetyMonitor->setPositionTolerances(20, 100);  // 20 steps warning, 100 steps error

        // Configure velocity tolerance
        safetyMonitor->setVelocityTolerances(50.0f,
                                             300.0f);  // 50 steps/s warning, 300 steps/s error

        // Configure emergency stop pin
        safetyMonitor->setEmergencyStopPin(15, LOW);  // Emergency stop on pin 15, active LOW
    }

    /**
     * Configure custom commands
     */
    void configureCommands() {
        // Get serial command processor
        SerialCommand* serialCommand = new SerialCommand(m_systemManager);

        // Initialize command processor
        serialCommand->initialize();

        // Add custom commands for single motor control
        serialCommand->addCommand(
            "home", "[direction]", "Home the motor",
            [this](const String& params, String& response) { return homeMotor(params, response); });

        serialCommand->addCommand("move_rel", "<distance> [speed]",
                                  "Move motor by relative distance",
                                  [this](const String& params, String& response) {
                                      return moveRelative(params, response);
                                  });

        serialCommand->addCommand("move_to", "<position> [speed]",
                                  "Move motor to absolute position",
                                  [this](const String& params, String& response) {
                                      return moveAbsolute(params, response);
                                  });

        serialCommand->addCommand("oscillate", "<amplitude> <frequency> [cycles]",
                                  "Oscillate motor",
                                  [this](const String& params, String& response) {
                                      return oscillateMotor(params, response);
                                  });

        // Start command processing
        serialCommand->begin();
    }

    /**
     * Process system updates
     */
    void processSystem() {
        // Update system metrics periodically
        uint32_t currentTimeMs = millis();
        if (currentTimeMs - m_lastUpdateTimeMs >= 1000) {
            m_systemManager->updateSystemMetrics();
            m_lastUpdateTimeMs = currentTimeMs;
        }
    }

    /**
     * Process application-specific logic
     */
    void processApplication() {
        // Check system state
        SystemState state = m_systemManager->getSystemState();
        if (state == SystemState::ERROR || state == SystemState::EMERGENCY_STOP) {
            // Handle error state
            handleErrorState();
        } else if (state == SystemState::READY || state == SystemState::RUNNING) {
            // Handle normal operation
            handleNormalOperation();
        }
    }

    /**
     * Handle system error state
     */
    void handleErrorState() {
        // Get safety monitor
        SafetyMonitor* safetyMonitor = m_systemManager->getSafetyMonitor();
        if (safetyMonitor == nullptr) {
            return;
        }

        // Check if in emergency stop
        if (safetyMonitor->isEmergencyStop()) {
            // Emergency stop condition
            static uint32_t lastAttemptMs = 0;
            uint32_t currentTimeMs = millis();

            // Try to reset emergency stop every 5 seconds
            if (currentTimeMs - lastAttemptMs >= 5000) {
                safetyMonitor->resetEmergencyStop();
                lastAttemptMs = currentTimeMs;
            }
        }

        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            return;
        }

        // Get motor
        Motor* motor = motorManager->getMotor(m_motorIndex);
        if (motor == nullptr) {
            return;
        }

        // Disable motor in error state
        if (motor->isEnabled()) {
            motor->disable();
        }
    }

    /**
     * Handle normal system operation
     */
    void handleNormalOperation() {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            return;
        }

        // Get motor
        Motor* motor = motorManager->getMotor(m_motorIndex);
        if (motor == nullptr) {
            return;
        }

        // Monitor motor status
        if (motor->isMoving()) {
            // Motor is currently moving
            // You could add specific monitoring here if needed
        } else {
            // Motor is idle
            // You could initiate automated sequences here if needed
        }
    }

    // Custom command handlers

    /**
     * Home the motor
     *
     * @param params Command parameters
     * @param response Command response
     * @return True if successful, false otherwise
     */
    bool homeMotor(const String& params, String& response) {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            response = "Motor manager not available";
            return false;
        }

        // Get motor
        Motor* motor = motorManager->getMotor(m_motorIndex);
        if (motor == nullptr) {
            response = "Motor not available";
            return false;
        }

        // Parse direction
        int direction = 1;  // Default to positive direction
        if (params.length() > 0) {
            direction = params.toInt();
            direction = direction < 0 ? -1 : 1;
        }

        // Enable motor if not enabled
        if (!motor->isEnabled()) {
            motor->enable();
        }

        // Start homing
        motor->startHoming(direction, CONFIG_DEFAULT_MAX_VELOCITY / 2.0f);

        response =
            "Homing motor in " + String(direction > 0 ? "positive" : "negative") + " direction";
        return true;
    }

    /**
     * Move motor by relative distance
     *
     * @param params Command parameters
     * @param response Command response
     * @return True if successful, false otherwise
     */
    bool moveRelative(const String& params, String& response) {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            response = "Motor manager not available";
            return false;
        }

        // Get motor
        Motor* motor = motorManager->getMotor(m_motorIndex);
        if (motor == nullptr) {
            response = "Motor not available";
            return false;
        }

        // Parse parameters
        String paramsStr = params;
        paramsStr.trim();

        // Split parameters
        int spaceIndex = paramsStr.indexOf(' ');
        String distanceStr, speedStr;

        if (spaceIndex < 0) {
            // Just distance
            distanceStr = paramsStr;
            speedStr = "1000";  // Default speed
        } else {
            // Distance and speed
            distanceStr = paramsStr.substring(0, spaceIndex);
            speedStr = paramsStr.substring(spaceIndex + 1);
            speedStr.trim();
        }

        // Convert parameters
        if (distanceStr.length() == 0) {
            response = "Distance parameter required";
            return false;
        }

        int32_t distance = distanceStr.toInt();
        float speed = speedStr.length() > 0 ? speedStr.toFloat() : 1000.0f;

        // Enable motor if not enabled
        if (!motor->isEnabled()) {
            motor->enable();
        }

        // Get current position
        int32_t currentPos = motor->getCurrentPosition();

        // Calculate target position
        int32_t targetPos = currentPos + distance;

        // Move to target position
        motor->moveToPosition(targetPos, speed, speed * 2.0f, speed * 2.0f);

        response = "Moving motor by " + distanceStr + " steps at speed " + speedStr;
        return true;
    }

    /**
     * Move motor to absolute position
     *
     * @param params Command parameters
     * @param response Command response
     * @return True if successful, false otherwise
     */
    bool moveAbsolute(const String& params, String& response) {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            response = "Motor manager not available";
            return false;
        }

        // Get motor
        Motor* motor = motorManager->getMotor(m_motorIndex);
        if (motor == nullptr) {
            response = "Motor not available";
            return false;
        }

        // Parse parameters
        String paramsStr = params;
        paramsStr.trim();

        // Split parameters
        int spaceIndex = paramsStr.indexOf(' ');
        String positionStr, speedStr;

        if (spaceIndex < 0) {
            // Just position
            positionStr = paramsStr;
            speedStr = "1000";  // Default speed
        } else {
            // Position and speed
            positionStr = paramsStr.substring(0, spaceIndex);
            speedStr = paramsStr.substring(spaceIndex + 1);
            speedStr.trim();
        }

        // Convert parameters
        if (positionStr.length() == 0) {
            response = "Position parameter required";
            return false;
        }

        int32_t position = positionStr.toInt();
        float speed = speedStr.length() > 0 ? speedStr.toFloat() : 1000.0f;

        // Enable motor if not enabled
        if (!motor->isEnabled()) {
            motor->enable();
        }

        // Move to target position
        motor->moveToPosition(position, speed, speed * 2.0f, speed * 2.0f);

        response = "Moving motor to position " + positionStr + " at speed " + speedStr;
        return true;
    }

    /**
     * Oscillate motor with specified amplitude and frequency
     *
     * @param params Command parameters
     * @param response Command response
     * @return True if successful, false otherwise
     */
    bool oscillateMotor(const String& params, String& response) {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            response = "Motor manager not available";
            return false;
        }

        // Get motor
        Motor* motor = motorManager->getMotor(m_motorIndex);
        if (motor == nullptr) {
            response = "Motor not available";
            return false;
        }

        // Parse parameters
        String paramsStr = params;
        paramsStr.trim();

        // Split parameters
        int firstSpaceIndex = paramsStr.indexOf(' ');
        if (firstSpaceIndex < 0) {
            response = "Insufficient parameters. Usage: oscillate <amplitude> <frequency> [cycles]";
            return false;
        }

        String amplitudeStr = paramsStr.substring(0, firstSpaceIndex);
        paramsStr = paramsStr.substring(firstSpaceIndex + 1);
        paramsStr.trim();

        int secondSpaceIndex = paramsStr.indexOf(' ');
        String frequencyStr, cyclesStr;

        if (secondSpaceIndex < 0) {
            // Just frequency, no cycles
            frequencyStr = paramsStr;
            cyclesStr = "5";  // Default 5 cycles
        } else {
            // Frequency and cycles
            frequencyStr = paramsStr.substring(0, secondSpaceIndex);
            cyclesStr = paramsStr.substring(secondSpaceIndex + 1);
            cyclesStr.trim();
        }

        // Convert parameters
        if (amplitudeStr.length() == 0 || frequencyStr.length() == 0) {
            response = "Amplitude and frequency parameters required";
            return false;
        }

        int32_t amplitude = amplitudeStr.toInt();
        float frequency = frequencyStr.toFloat();
        int cycles = cyclesStr.length() > 0 ? cyclesStr.toInt() : 5;

        if (amplitude <= 0 || frequency <= 0.0f || cycles <= 0) {
            response = "Invalid parameters. Amplitude, frequency and cycles must be positive";
            return false;
        }

        // Enable motor if not enabled
        if (!motor->isEnabled()) {
            motor->enable();
        }

        // Get current position as center point
        int32_t centerPosition = motor->getCurrentPosition();

        // Calculate motion parameters
        float speed = 2.0f * frequency * amplitude;  // Speed needed for given frequency
        float accel = speed * 10.0f;                 // High acceleration for sinusoidal motion

        response = "Oscillating motor with amplitude " + amplitudeStr + ", frequency " +
                   frequencyStr + "Hz, " + cyclesStr + " cycles";

        // Execute oscillation in a separate task to avoid blocking
        TaskScheduler* scheduler = m_systemManager->getTaskScheduler();
        if (scheduler != nullptr) {
            scheduler->registerAuxiliaryTask(
                [this, motor, centerPosition, amplitude, speed, accel, cycles]() {
                    executeOscillation(motor, centerPosition, amplitude, speed, accel, cycles);
                },
                0, TaskTimingMode::ADAPTIVE);
        } else {
            // Manual oscillation execution (will block)
            executeOscillation(motor, centerPosition, amplitude, speed, accel, cycles);
        }

        return true;
    }

    /**
     * Execute oscillation motion
     *
     * @param motor Pointer to motor
     * @param centerPos Center position
     * @param amplitude Oscillation amplitude
     * @param speed Motion speed
     * @param accel Motion acceleration
     * @param cycles Number of cycles
     */
    void executeOscillation(Motor* motor, int32_t centerPos, int32_t amplitude, float speed,
                            float accel, int cycles) {
        for (int i = 0; i < cycles; i++) {
            // Move to positive peak
            motor->moveToPosition(centerPos + amplitude, speed, accel, accel);

            // Wait for motion completion
            while (motor->isMoving()) {
                delay(10);
            }

            // Move to negative peak
            motor->moveToPosition(centerPos - amplitude, speed, accel, accel);

            // Wait for motion completion
            while (motor->isMoving()) {
                delay(10);
            }
        }

        // Return to center
        motor->moveToPosition(centerPos, speed, accel, accel);
    }
};

#endif  // EXAMPLE_ENABLED

#endif  // SINGLE_MOTOR_EXAMPLE_H
        // End of Code