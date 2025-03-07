/*
 * ESP32 High-Precision Motion Control System
 * Custom Application Example
 *
 * This file demonstrates how to configure and use the motion control
 * system for a specific application with multiple motors.
 */

#ifndef CUSTOM_APPLICATION_H
#define CUSTOM_APPLICATION_H

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
 * Custom application class for a specific use case
 */
class CustomApplication {
   public:
    /**
     * Constructor
     */
    CustomApplication() : m_initialized(false) {
        // Initialize system components
        m_systemManager = new SystemManager();
    }

    /**
     * Destructor
     */
    ~CustomApplication() {
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

        // Configure motors for this application
        configureMotors(motorManager);

        // Configure safety parameters
        configureSafety();

        // Configure serial commands
        configureCommands();

        // Load configuration from EEPROM
        m_systemManager->loadSystemConfiguration();

        m_initialized = true;
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

    /**
     * Configure motors for this application
     *
     * @param motorManager Pointer to motor manager
     */
    void configureMotors(MotorManager* motorManager) {
        // Example: Configure 2 stepper motors with specific parameters

        // Motor 0 configuration (X-axis)
        MotorConfig xAxisConfig = DEFAULT_MOTOR_CONFIGS[0];
        xAxisConfig.maxVelocity = 5000.0f;       // 5000 steps/second
        xAxisConfig.maxAcceleration = 20000.0f;  // 20000 steps/second²
        xAxisConfig.maxDeceleration = 25000.0f;  // 25000 steps/second²
        xAxisConfig.maxJerk = 100000.0f;         // 100000 steps/second³
        xAxisConfig.pidKp = 0.5f;                // Lower P gain for smoother response
        xAxisConfig.pidKi = 0.1f;
        xAxisConfig.pidKd = 0.05f;
        xAxisConfig.pidFf = 0.01f;  // Small feed-forward for better tracking

        // Add X-axis motor
        motorManager->addMotor(xAxisConfig);

        // Motor 1 configuration (Y-axis)
        MotorConfig yAxisConfig = DEFAULT_MOTOR_CONFIGS[1];
        yAxisConfig.maxVelocity = 4000.0f;       // 4000 steps/second
        yAxisConfig.maxAcceleration = 15000.0f;  // 15000 steps/second²
        yAxisConfig.maxDeceleration = 20000.0f;  // 20000 steps/second²
        yAxisConfig.maxJerk = 80000.0f;          // 80000 steps/second³
        yAxisConfig.pidKp = 0.6f;                // Higher P gain for better position holding
        yAxisConfig.pidKi = 0.12f;
        yAxisConfig.pidKd = 0.06f;
        yAxisConfig.pidFf = 0.01f;

        // Add Y-axis motor
        motorManager->addMotor(yAxisConfig);

        // Optional: Configure additional motors as needed
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
        safetyMonitor->setPositionTolerances(50, 200);  // 50 steps warning, 200 steps error

        // Configure velocity tolerance
        safetyMonitor->setVelocityTolerances(100.0f,
                                             500.0f);  // 100 steps/s warning, 500 steps/s error

        // Configure temperature tolerance
        safetyMonitor->setTemperatureTolerances(60.0f, 80.0f);  // 60°C warning, 80°C error

        // Configure emergency stop pin
        safetyMonitor->setEmergencyStopPin(23, LOW);  // Emergency stop on pin 23, active LOW
    }

    /**
     * Configure custom commands
     */
    void configureCommands() {
        // Get serial command processor
        SerialCommand* serialCommand = new SerialCommand(m_systemManager);

        // Initialize command processor
        serialCommand->initialize();

        // Add custom commands
        serialCommand->addCommand("home_all", "", "Home all axes",
                                  [this](const String& params, String& response) {
                                      return homeAllAxes(params, response);
                                  });

        serialCommand->addCommand("square", "<size> [speed]", "Move in square pattern",
                                  [this](const String& params, String& response) {
                                      return moveSquarePattern(params, response);
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
        // Example: Application-specific processing

        // Check system state
        SystemState state = m_systemManager->getSystemState();
        if (state == SystemState::ERROR || state == SystemState::EMERGENCY_STOP) {
            // System in error state, handle accordingly
            handleErrorState();
        } else if (state == SystemState::READY || state == SystemState::RUNNING) {
            // System ready/running, perform normal operations
            performNormalOperations();
        }
    }

    /**
     * Handle system error state
     */
    void handleErrorState() {
        // Example: Error handling logic

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
    }

    /**
     * Perform normal operations
     */
    void performNormalOperations() {
        // Example: Normal operation logic

        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            return;
        }

        // Check if all motors are idle
        if (!areAllMotorsIdle(motorManager)) {
            // Motors are moving, monitor progress
            monitorMotionProgress();
        } else {
            // All motors idle, can start new motions
            // (Application-specific logic would go here)
        }
    }

    /**
     * Check if all motors are idle
     *
     * @param motorManager Pointer to motor manager
     * @return True if all motors are idle, false otherwise
     */
    bool areAllMotorsIdle(MotorManager* motorManager) {
        for (uint8_t i = 0; i < motorManager->getMotorCount(); i++) {
            Motor* motor = motorManager->getMotor(i);
            if (motor != nullptr && motor->isMoving()) {
                return false;
            }
        }
        return true;
    }

    /**
     * Monitor motion progress
     */
    void monitorMotionProgress() {
        // Example: Monitor and handle motion progress
        // (Application-specific logic would go here)
    }

    // Custom command handlers

    /**
     * Home all axes
     *
     * @param params Command parameters
     * @param response Command response
     * @return True if successful, false otherwise
     */
    bool homeAllAxes(const String& params, String& response) {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            response = "Motor manager not available";
            return false;
        }

        // Parse direction
        int direction = 1;  // Default to positive direction
        if (params.length() > 0) {
            direction = params.toInt();
            direction = direction < 0 ? -1 : 1;
        }

        // Start homing sequence for all motors
        for (uint8_t i = 0; i < motorManager->getMotorCount(); i++) {
            Motor* motor = motorManager->getMotor(i);
            if (motor != nullptr) {
                // Enable motor if not enabled
                if (!motor->isEnabled()) {
                    motor->enable();
                }

                // Start homing
                motor->startHoming(direction, CONFIG_DEFAULT_MAX_VELOCITY / 2.0f);
            }
        }

        response =
            "Homing all axes in " + String(direction > 0 ? "positive" : "negative") + " direction";
        return true;
    }

    /**
     * Move in square pattern
     *
     * @param params Command parameters
     * @param response Command response
     * @return True if successful, false otherwise
     */
    bool moveSquarePattern(const String& params, String& response) {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            response = "Motor manager not available";
            return false;
        }

        // Check motor count
        if (motorManager->getMotorCount() < 2) {
            response = "Need at least 2 motors for square pattern";
            return false;
        }

        // Parse parameters
        String paramsStr = params;
        paramsStr.trim();

        // Split parameters
        int spaceIndex = paramsStr.indexOf(' ');
        String sizeStr, speedStr;

        if (spaceIndex < 0) {
            // Just size
            sizeStr = paramsStr;
            speedStr = "1000";  // Default speed
        } else {
            // Size and speed
            sizeStr = paramsStr.substring(0, spaceIndex);
            speedStr = paramsStr.substring(spaceIndex + 1);
            speedStr.trim();
        }

        // Convert parameters
        if (sizeStr.length() == 0) {
            response = "Size parameter required";
            return false;
        }

        int32_t size = sizeStr.toInt();
        float speed = speedStr.length() > 0 ? speedStr.toFloat() : 1000.0f;

        // Get motors
        Motor* xMotor = motorManager->getMotor(0);
        Motor* yMotor = motorManager->getMotor(1);

        if (xMotor == nullptr || yMotor == nullptr) {
            response = "X or Y motor not available";
            return false;
        }

        // Enable motors if not enabled
        if (!xMotor->isEnabled()) {
            xMotor->enable();
        }

        if (!yMotor->isEnabled()) {
            yMotor->enable();
        }

        // Get current positions
        int32_t xStart = xMotor->getCurrentPosition();
        int32_t yStart = yMotor->getCurrentPosition();

        // Calculate square corner positions
        int32_t x1 = xStart;
        int32_t y1 = yStart;
        int32_t x2 = xStart + size;
        int32_t y2 = yStart;
        int32_t x3 = xStart + size;
        int32_t y3 = yStart + size;
        int32_t x4 = xStart;
        int32_t y4 = yStart + size;

        // Move to first corner (top-left)
        xMotor->moveToPosition(x1, speed, speed * 2.0f, speed * 2.0f);
        yMotor->moveToPosition(y1, speed, speed * 2.0f, speed * 2.0f);

        // Wait for motors to complete
        while (xMotor->isMoving() || yMotor->isMoving()) {
            delay(10);
        }

        // Move to second corner (top-right)
        xMotor->moveToPosition(x2, speed, speed * 2.0f, speed * 2.0f);

        // Wait for X motor to complete
        while (xMotor->isMoving()) {
            delay(10);
        }

        // Move to third corner (bottom-right)
        yMotor->moveToPosition(y3, speed, speed * 2.0f, speed * 2.0f);

        // Wait for Y motor to complete
        while (yMotor->isMoving()) {
            delay(10);
        }

        // Move to fourth corner (bottom-left)
        xMotor->moveToPosition(x4, speed, speed * 2.0f, speed * 2.0f);

        // Wait for X motor to complete
        while (xMotor->isMoving()) {
            delay(10);
        }

        // Move back to start (top-left)
        yMotor->moveToPosition(y1, speed, speed * 2.0f, speed * 2.0f);

        response = "Moving in square pattern of size " + sizeStr + " at speed " + speedStr;
        return true;
    }
};

#endif  // CUSTOM_APPLICATION_H