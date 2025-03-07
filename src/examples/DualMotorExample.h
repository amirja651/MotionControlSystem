/*
 * ESP32 High-Precision Motion Control System
 * Dual Motor Example
 *
 * This file demonstrates how to configure and use the motion control
 * system for a dual-axis application with coordinated movement.
 */

#ifndef DUAL_MOTOR_EXAMPLE_H
#define DUAL_MOTOR_EXAMPLE_H

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
 * Dual Motor Example class for coordinated dual-axis control
 */
class DualMotorExample {
   public:
    /**
     * Constructor
     */
    DualMotorExample() : m_initialized(false), m_xAxisIndex(0), m_yAxisIndex(1) {
        // Initialize system components
        m_systemManager = new SystemManager();
    }

    /**
     * Destructor
     */
    ~DualMotorExample() {
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

        // Configure motors
        configureMotors(motorManager);

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
        m_systemManager->getLogger()->logInfo("Dual Motor Example initialized");

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
    uint8_t m_xAxisIndex;
    uint8_t m_yAxisIndex;

    /**
     * Configure motors for this application
     *
     * @param motorManager Pointer to motor manager
     */
    void configureMotors(MotorManager* motorManager) {
        // X-axis motor configuration
        MotorConfig xAxisConfig = DEFAULT_MOTOR_CONFIGS[0];
        xAxisConfig.maxVelocity = 4000.0f;       // 4000 steps/second
        xAxisConfig.maxAcceleration = 15000.0f;  // 15000 steps/second²
        xAxisConfig.maxDeceleration = 20000.0f;  // 20000 steps/second²
        xAxisConfig.maxJerk = 80000.0f;          // 80000 steps/second³
        xAxisConfig.pidKp = 0.7f;                // P gain for position control
        xAxisConfig.pidKi = 0.1f;                // I gain for steady-state error
        xAxisConfig.pidKd = 0.05f;               // D gain for dampening
        xAxisConfig.pidFf = 0.08f;               // Feed-forward for velocity tracking
        xAxisConfig.encoderPPR = 2000;           // 2000 pulses per revolution encoder

        // Add X-axis motor
        motorManager->addMotor(xAxisConfig);

        // Y-axis motor configuration
        MotorConfig yAxisConfig = DEFAULT_MOTOR_CONFIGS[1];
        yAxisConfig.maxVelocity = 3500.0f;       // 3500 steps/second
        yAxisConfig.maxAcceleration = 12000.0f;  // 12000 steps/second²
        yAxisConfig.maxDeceleration = 15000.0f;  // 15000 steps/second²
        yAxisConfig.maxJerk = 60000.0f;          // 60000 steps/second³
        yAxisConfig.pidKp = 0.65f;               // P gain for position control
        yAxisConfig.pidKi = 0.12f;               // I gain for steady-state error
        yAxisConfig.pidKd = 0.04f;               // D gain for dampening
        yAxisConfig.pidFf = 0.075f;              // Feed-forward for velocity tracking
        yAxisConfig.encoderPPR = 2000;           // 2000 pulses per revolution encoder

        // Add Y-axis motor
        motorManager->addMotor(yAxisConfig);

        // Save the motor indices for later reference
        m_xAxisIndex = xAxisConfig.index;
        m_yAxisIndex = yAxisConfig.index;
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
        safetyMonitor->setPositionTolerances(30, 150);  // 30 steps warning, 150 steps error

        // Configure velocity tolerance
        safetyMonitor->setVelocityTolerances(80.0f,
                                             400.0f);  // 80 steps/s warning, 400 steps/s error

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

        // Add custom commands for dual motor control
        serialCommand->addCommand("home_all", "[direction]", "Home all axes",
                                  [this](const String& params, String& response) {
                                      return homeAllAxes(params, response);
                                  });

        serialCommand->addCommand(
            "home_axis", "<axis> [direction]", "Home specific axis (x or y)",
            [this](const String& params, String& response) { return homeAxis(params, response); });

        serialCommand->addCommand("move_to", "<x> <y> [speed]", "Move to absolute position",
                                  [this](const String& params, String& response) {
                                      return moveToPosition(params, response);
                                  });

        serialCommand->addCommand("rectangle", "<width> <height> [speed]",
                                  "Move in rectangular pattern",
                                  [this](const String& params, String& response) {
                                      return moveRectanglePattern(params, response);
                                  });

        serialCommand->addCommand("circle", "<radius> [speed] [segments]",
                                  "Move in circular pattern",
                                  [this](const String& params, String& response) {
                                      return moveCirclePattern(params, response);
                                  });

        serialCommand->addCommand("diagonal", "<distance> [speed]", "Move diagonally",
                                  [this](const String& params, String& response) {
                                      return moveDiagonal(params, response);
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

        // Get motors
        Motor* xMotor = motorManager->getMotor(m_xAxisIndex);
        Motor* yMotor = motorManager->getMotor(m_yAxisIndex);

        // Disable motors in error state
        if (xMotor != nullptr && xMotor->isEnabled()) {
            xMotor->disable();
        }

        if (yMotor != nullptr && yMotor->isEnabled()) {
            yMotor->disable();
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

        // Get motors
        Motor* xMotor = motorManager->getMotor(m_xAxisIndex);
        Motor* yMotor = motorManager->getMotor(m_yAxisIndex);

        if (xMotor == nullptr || yMotor == nullptr) {
            return;
        }

        // Check if any motor is moving
        bool anyMotorMoving = xMotor->isMoving() || yMotor->isMoving();

        if (anyMotorMoving) {
            // Motors are moving, monitor progress
            // You could add specific monitoring logic here
        } else {
            // All motors idle
            // You could initiate automated sequences here if needed
        }
    }

    /**
     * Check if all motors are idle
     *
     * @return True if all motors are idle, false otherwise
     */
    bool areAllMotorsIdle() {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            return false;
        }

        // Get motors
        Motor* xMotor = motorManager->getMotor(m_xAxisIndex);
        Motor* yMotor = motorManager->getMotor(m_yAxisIndex);

        if (xMotor == nullptr || yMotor == nullptr) {
            return false;
        }

        // Check if any motor is moving
        return !xMotor->isMoving() && !yMotor->isMoving();
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

        // Get motors
        Motor* xMotor = motorManager->getMotor(m_xAxisIndex);
        Motor* yMotor = motorManager->getMotor(m_yAxisIndex);

        if (xMotor == nullptr || yMotor == nullptr) {
            response = "One or more motors not available";
            return false;
        }

        // Enable motors if not enabled
        if (!xMotor->isEnabled()) {
            xMotor->enable();
        }

        if (!yMotor->isEnabled()) {
            yMotor->enable();
        }

        // Start homing
        xMotor->startHoming(direction, CONFIG_DEFAULT_MAX_VELOCITY / 2.0f);
        yMotor->startHoming(direction, CONFIG_DEFAULT_MAX_VELOCITY / 2.0f);

        response =
            "Homing all axes in " + String(direction > 0 ? "positive" : "negative") + " direction";
        return true;
    }

    /**
     * Home specific axis (x or y)
     *
     * @param params Command parameters
     * @param response Command response
     * @return True if successful, false otherwise
     */
    bool homeAxis(const String& params, String& response) {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            response = "Motor manager not available";
            return false;
        }

        // Parse parameters
        String paramsStr = params;
        paramsStr.trim();

        // Split parameters
        int spaceIndex = paramsStr.indexOf(' ');
        String axisStr, directionStr;

        if (spaceIndex < 0) {
            // Just axis
            axisStr = paramsStr;
            directionStr = "1";  // Default to positive direction
        } else {
            // Axis and direction
            axisStr = paramsStr.substring(0, spaceIndex);
            directionStr = paramsStr.substring(spaceIndex + 1);
            directionStr.trim();
        }

        // Convert parameters
        if (axisStr.length() == 0) {
            response = "Axis parameter required (x or y)";
            return false;
        }

        // Determine which axis to home
        uint8_t motorIndex;

        if (axisStr.equalsIgnoreCase("x")) {
            motorIndex = m_xAxisIndex;
        } else if (axisStr.equalsIgnoreCase("y")) {
            motorIndex = m_yAxisIndex;
        } else {
            response = "Invalid axis. Use 'x' or 'y'";
            return false;
        }

        // Parse direction
        int direction = 1;  // Default to positive direction
        if (directionStr.length() > 0) {
            direction = directionStr.toInt();
            direction = direction < 0 ? -1 : 1;
        }

        // Get motor
        Motor* motor = motorManager->getMotor(motorIndex);
        if (motor == nullptr) {
            response = "Motor not available";
            return false;
        }

        // Enable motor if not enabled
        if (!motor->isEnabled()) {
            motor->enable();
        }

        // Start homing
        motor->startHoming(direction, CONFIG_DEFAULT_MAX_VELOCITY / 2.0f);

        response = "Homing " + axisStr + " axis in " +
                   String(direction > 0 ? "positive" : "negative") + " direction";
        return true;
    }

    /**
     * Move to absolute position
     *
     * @param params Command parameters
     * @param response Command response
     * @return True if successful, false otherwise
     */
    bool moveToPosition(const String& params, String& response) {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            response = "Motor manager not available";
            return false;
        }

        // Get motors
        Motor* xMotor = motorManager->getMotor(m_xAxisIndex);
        Motor* yMotor = motorManager->getMotor(m_yAxisIndex);

        if (xMotor == nullptr || yMotor == nullptr) {
            response = "One or more motors not available";
            return false;
        }

        // Parse parameters
        String paramsStr = params;
        paramsStr.trim();

        // Find first space
        int firstSpaceIndex = paramsStr.indexOf(' ');
        if (firstSpaceIndex < 0) {
            response = "Insufficient parameters. Usage: move_to <x> <y> [speed]";
            return false;
        }

        // Extract X position
        String xPosStr = paramsStr.substring(0, firstSpaceIndex);
        paramsStr = paramsStr.substring(firstSpaceIndex + 1);
        paramsStr.trim();

        // Find second space
        int secondSpaceIndex = paramsStr.indexOf(' ');
        String yPosStr, speedStr;

        if (secondSpaceIndex < 0) {
            // Just Y position, no speed
            yPosStr = paramsStr;
            speedStr = "1000";  // Default speed
        } else {
            // Y position and speed
            yPosStr = paramsStr.substring(0, secondSpaceIndex);
            speedStr = paramsStr.substring(secondSpaceIndex + 1);
            speedStr.trim();
        }

        // Convert parameters
        if (xPosStr.length() == 0 || yPosStr.length() == 0) {
            response = "X and Y position parameters required";
            return false;
        }

        int32_t xPos = xPosStr.toInt();
        int32_t yPos = yPosStr.toInt();
        float speed = speedStr.length() > 0 ? speedStr.toFloat() : 1000.0f;

        // Enable motors if not enabled
        if (!xMotor->isEnabled()) {
            xMotor->enable();
        }

        if (!yMotor->isEnabled()) {
            yMotor->enable();
        }

        // Move to target positions with synchronized motion
        uint8_t motorIndices[2] = {m_xAxisIndex, m_yAxisIndex};
        int32_t positions[2] = {xPos, yPos};

        motorManager->moveMultipleMotors(positions, motorIndices, 2, speed, speed * 2.0f,
                                         speed * 2.0f);

        response = "Moving to position X:" + xPosStr + ", Y:" + yPosStr + " at speed " + speedStr;
        return true;
    }

    /**
     * Move in rectangular pattern
     *
     * @param params Command parameters
     * @param response Command response
     * @return True if successful, false otherwise
     */
    bool moveRectanglePattern(const String& params, String& response) {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            response = "Motor manager not available";
            return false;
        }

        // Get motors
        Motor* xMotor = motorManager->getMotor(m_xAxisIndex);
        Motor* yMotor = motorManager->getMotor(m_yAxisIndex);

        if (xMotor == nullptr || yMotor == nullptr) {
            response = "One or more motors not available";
            return false;
        }

        // Parse parameters
        String paramsStr = params;
        paramsStr.trim();

        // Find first space
        int firstSpaceIndex = paramsStr.indexOf(' ');
        if (firstSpaceIndex < 0) {
            response = "Insufficient parameters. Usage: rectangle <width> <height> [speed]";
            return false;
        }

        // Extract width
        String widthStr = paramsStr.substring(0, firstSpaceIndex);
        paramsStr = paramsStr.substring(firstSpaceIndex + 1);
        paramsStr.trim();

        // Find second space
        int secondSpaceIndex = paramsStr.indexOf(' ');
        String heightStr, speedStr;

        if (secondSpaceIndex < 0) {
            // Just height, no speed
            heightStr = paramsStr;
            speedStr = "1000";  // Default speed
        } else {
            // Height and speed
            heightStr = paramsStr.substring(0, secondSpaceIndex);
            speedStr = paramsStr.substring(secondSpaceIndex + 1);
            speedStr.trim();
        }

        // Convert parameters
        if (widthStr.length() == 0 || heightStr.length() == 0) {
            response = "Width and height parameters required";
            return false;
        }

        int32_t width = widthStr.toInt();
        int32_t height = heightStr.toInt();
        float speed = speedStr.length() > 0 ? speedStr.toFloat() : 1000.0f;

        if (width <= 0 || height <= 0) {
            response = "Width and height must be positive";
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

        // Calculate rectangle corner positions
        int32_t x1 = xStart;
        int32_t y1 = yStart;
        int32_t x2 = xStart + width;
        int32_t y2 = yStart;
        int32_t x3 = xStart + width;
        int32_t y3 = yStart + height;
        int32_t x4 = xStart;
        int32_t y4 = yStart + height;

        response = "Moving in rectangle pattern of width " + widthStr + ", height " + heightStr +
                   " at speed " + speedStr;

        // Execute rectangle motion in a separate task to avoid blocking
        TaskScheduler* scheduler = m_systemManager->getTaskScheduler();
        if (scheduler != nullptr) {
            scheduler->registerAuxiliaryTask(
                [this, xMotor, yMotor, x1, y1, x2, y2, x3, y3, x4, y4, speed]() {
                    executeRectangleMotion(xMotor, yMotor, x1, y1, x2, y2, x3, y3, x4, y4, speed);
                },
                0, TaskTimingMode::ADAPTIVE);
        } else {
            // Manual execution (will block)
            executeRectangleMotion(xMotor, yMotor, x1, y1, x2, y2, x3, y3, x4, y4, speed);
        }

        return true;
    }

    /**
     * Execute rectangle motion
     */
    void executeRectangleMotion(Motor* xMotor, Motor* yMotor, int32_t x1, int32_t y1, int32_t x2,
                                int32_t y2, int32_t x3, int32_t y3, int32_t x4, int32_t y4,
                                float speed) {
        float accel = speed * 2.0f;

        // Move to first corner (if not already there)
        xMotor->moveToPosition(x1, speed, accel, accel);
        yMotor->moveToPosition(y1, speed, accel, accel);

        // Wait for motors to complete
        while (xMotor->isMoving() || yMotor->isMoving()) {
            delay(10);
        }

        // Move to second corner
        xMotor->moveToPosition(x2, speed, accel, accel);

        // Wait for X motor to complete
        while (xMotor->isMoving()) {
            delay(10);
        }

        // Move to third corner
        yMotor->moveToPosition(y3, speed, accel, accel);

        // Wait for Y motor to complete
        while (yMotor->isMoving()) {
            delay(10);
        }

        // Move to fourth corner
        xMotor->moveToPosition(x4, speed, accel, accel);

        // Wait for X motor to complete
        while (xMotor->isMoving()) {
            delay(10);
        }

        // Move back to first corner
        yMotor->moveToPosition(y1, speed, accel, accel);
    }

    /**
     * Move in circular pattern
     *
     * @param params Command parameters
     * @param response Command response
     * @return True if successful, false otherwise
     */
    bool moveCirclePattern(const String& params, String& response) {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            response = "Motor manager not available";
            return false;
        }

        // Get motors
        Motor* xMotor = motorManager->getMotor(m_xAxisIndex);
        Motor* yMotor = motorManager->getMotor(m_yAxisIndex);

        if (xMotor == nullptr || yMotor == nullptr) {
            response = "One or more motors not available";
            return false;
        }

        // Parse parameters
        String paramsStr = params;
        paramsStr.trim();

        // Split parameters
        int firstSpaceIndex = paramsStr.indexOf(' ');
        String radiusStr, speedStr, segmentsStr;

        if (firstSpaceIndex < 0) {
            // Just radius
            radiusStr = paramsStr;
            speedStr = "1000";   // Default speed
            segmentsStr = "36";  // Default 36 segments (10° per segment)
        } else {
            // Radius and more
            radiusStr = paramsStr.substring(0, firstSpaceIndex);
            paramsStr = paramsStr.substring(firstSpaceIndex + 1);
            paramsStr.trim();

            int secondSpaceIndex = paramsStr.indexOf(' ');
            if (secondSpaceIndex < 0) {
                // Radius and speed only
                speedStr = paramsStr;
                segmentsStr = "36";  // Default 36 segments
            } else {
                // Radius, speed, and segments
                speedStr = paramsStr.substring(0, secondSpaceIndex);
                segmentsStr = paramsStr.substring(secondSpaceIndex + 1);
                segmentsStr.trim();
            }
        }

        // Convert parameters
        if (radiusStr.length() == 0) {
            response = "Radius parameter required";
            return false;
        }

        int32_t radius = radiusStr.toInt();
        float speed = speedStr.length() > 0 ? speedStr.toFloat() : 1000.0f;
        int segments = segmentsStr.length() > 0 ? segmentsStr.toInt() : 36;

        if (radius <= 0 || segments < 6) {
            response = "Radius must be positive and segments must be at least 6";
            return false;
        }

        // Enable motors if not enabled
        if (!xMotor->isEnabled()) {
            xMotor->enable();
        }

        if (!yMotor->isEnabled()) {
            yMotor->enable();
        }

        // Get current position as circle center
        int32_t xCenter = xMotor->getCurrentPosition();
        int32_t yCenter = yMotor->getCurrentPosition();

        response = "Moving in circular pattern with radius " + radiusStr + " at speed " + speedStr +
                   " using " + segmentsStr + " segments";

        // Execute circle motion in a separate task to avoid blocking
        TaskScheduler* scheduler = m_systemManager->getTaskScheduler();
        if (scheduler != nullptr) {
            scheduler->registerAuxiliaryTask(
                [this, xMotor, yMotor, xCenter, yCenter, radius, speed, segments]() {
                    executeCircleMotion(xMotor, yMotor, xCenter, yCenter, radius, speed, segments);
                },
                0, TaskTimingMode::ADAPTIVE);
        } else {
            // Manual execution (will block)
            executeCircleMotion(xMotor, yMotor, xCenter, yCenter, radius, speed, segments);
        }

        return true;
    }

    /**
     * Execute circular motion
     */
    void executeCircleMotion(Motor* xMotor, Motor* yMotor, int32_t xCenter, int32_t yCenter,
                             int32_t radius, float speed, int segments) {
        float accel = speed * 2.0f;

        // First move to starting point on circle
        int32_t xStart = xCenter + radius;
        int32_t yStart = yCenter;

        // Move to start point
        xMotor->moveToPosition(xStart, speed, accel, accel);
        yMotor->moveToPosition(yStart, speed, accel, accel);

        // Wait for motors to complete
        while (xMotor->isMoving() || yMotor->isMoving()) {
            delay(10);
        }

        // Calculate segment angle in radians
        float angleIncrement = 2.0f * PI / segments;

        // Move through each segment of the circle
        for (int i = 1; i <= segments; i++) {
            float angle = i * angleIncrement;

            // Calculate next point on circle
            int32_t xPos = xCenter + round(radius * cos(angle));
            int32_t yPos = yCenter + round(radius * sin(angle));

            // Move to next point
            xMotor->moveToPosition(xPos, speed, accel, accel);
            yMotor->moveToPosition(yPos, speed, accel, accel);

            // Wait for motors to complete
            while (xMotor->isMoving() || yMotor->isMoving()) {
                delay(10);
            }
        }
    }

    /**
     * Move diagonally
     *
     * @param params Command parameters
     * @param response Command response
     * @return True if successful, false otherwise
     */
    bool moveDiagonal(const String& params, String& response) {
        // Get motor manager
        MotorManager* motorManager = m_systemManager->getMotorManager();
        if (motorManager == nullptr) {
            response = "Motor manager not available";
            return false;
        }

        // Get motors
        Motor* xMotor = motorManager->getMotor(m_xAxisIndex);
        Motor* yMotor = motorManager->getMotor(m_yAxisIndex);

        if (xMotor == nullptr || yMotor == nullptr) {
            response = "One or more motors not available";
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

        // Calculate target positions
        int32_t xTarget = xStart + distance;
        int32_t yTarget = yStart + distance;

        // Move to target positions with synchronized motion
        uint8_t motorIndices[2] = {m_xAxisIndex, m_yAxisIndex};
        int32_t positions[2] = {xTarget, yTarget};

        motorManager->moveMultipleMotors(positions, motorIndices, 2, speed, speed * 2.0f,
                                         speed * 2.0f);

        response = "Moving diagonally by " + distanceStr + " steps at speed " + speedStr;
        return true;
    }
};

#endif  // DUAL_MOTOR_EXAMPLE_H
        // End of Code