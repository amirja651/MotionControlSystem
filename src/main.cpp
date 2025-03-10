/*
 * ESP32 High-Precision Motion Control System
 * Main sketch file
 *
 * This file initializes the motion control system and sets up the dual-core
 * task distribution for optimal real-time performance.
 */

#include <Arduino.h>

#include "Configuration.h"
#include "MotorManager.h"
#include "SystemManager.h"
#include "communication/SerialCommand.h"
#include "communication/StatusReporter.h"
#include "utils/Logger.h"
#include "utils/TaskScheduler.h"

// System components
SystemManager systemManager;
Logger logger;
MotorManager motorManager(CONFIG_MAX_MOTORS, &logger);
TaskScheduler taskScheduler;
SerialCommand serialCommand;
StatusReporter statusReporter(&systemManager, CONFIG_STATUS_UPDATE_FREQUENCY_HZ);

// Core 0 task handle
TaskHandle_t auxiliaryTaskHandle = NULL;

/**
 * Core 0 Task - Handles auxiliary operations
 * - Command processing
 * - Status reporting
 * - Logging
 * - Parameter management
 * - Thermal monitoring
 */
void auxiliaryTask(void *parameter) {
    // Initialize auxiliary systems
    serialCommand.begin();
    statusReporter.begin();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 100Hz update rate for non-critical tasks

    // Main auxiliary task loop
    while (true) {
        // Process incoming commands with priority
        serialCommand.processCommands();

        // Execute scheduled auxiliary tasks
        taskScheduler.executeAuxiliaryTasks();

        // Update system status at regular intervals
        statusReporter.updateStatus();

        // Log system information
        logger.processPendingLogs();

        // Yield to allow other Core 0 tasks to run
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    // Initialize serial communication for debugging and commands
    Serial.begin(CONFIG_SERIAL_BAUD_RATE);

    // Wait a moment for serial to stabilize
    delay(100);

    Serial.println(ANSI_COLOR_MAGENTA SYSTEM_NAME " starting..." ANSI_COLOR_RESET);

    // Initialize logger first for better debug output
    if (!logger.initialize()) {
        Serial.println(ANSI_COLOR_RED "Logger initialization failed" ANSI_COLOR_RESET);
        while (1) {
            delay(1000);
        }
    }

    // Initialize system components with proper error handling
    if (!systemManager.initialize()) {
        logger.logError("System initialization failed");
        while (1) {
            delay(1000);
        }  // Safety halt if critical initialization fails
    }

    // Make sure the serialCommand knows about the systemManager
    serialCommand.setSystemManager(&systemManager);

    // Initialize the motor manager with the configured motors
    if (!motorManager.initialize()) {
        logger.logError("Motor manager initialization failed");
        while (1) {
            delay(1000);
        }
    }

    // The SystemManager should store a reference to this instance:
    systemManager.setStatusReporter(&statusReporter);

    // Initialize the status reporter after the system manager is ready
    if (!statusReporter.initialize()) {
        logger.logError("Status reporter initialization failed");
        while (1) {
            delay(1000);
        }
    }

    // to ensure it's ready to handle commands
    if (!serialCommand.initialize()) {
        logger.logError("Serial command initialization failed");
        while (1) {
            delay(1000);
        }
    }

// Check for power failure recovery only if power monitoring is enabled
#if CONFIG_POWER_MONITORING_ENABLED
    if (!systemManager.wasNormalShutdown()) {
        // Power interruption detected
        logger.logInfo("Power interruption detected. Restoring previous positions...");
        systemManager.restoreMotorPositions();
    }
#else
    logger.logWarning("Power monitoring disabled. Skipping power failure recovery.");
#endif

    // Reset the shutdown flag for next time
    systemManager.setNormalShutdown(false);

    // Configure the task scheduler
    taskScheduler.initialize();

    // Register scheduled tasks for Core 1 (control loop)
    for (uint8_t i = 0; i < motorManager.getMotorCount(); i++) {
        Motor *motor = motorManager.getMotor(i);

        // Register high-priority motor control tasks
        taskScheduler.registerControlTask(
            [motor]() {
                motor->updateControl();  // Runs the PID loop and control logic
            },
            motor->getControlInterval());

        // Register motion profile updates (slightly lower priority)
        taskScheduler.registerControlTask(
            [motor]() {
                motor->updateTrajectory();  // Updates position/velocity profiles
            },
            motor->getTrajectoryUpdateInterval());
    }

    // Register system monitoring tasks
    taskScheduler.registerControlTask([]() { systemManager.getSafetyMonitor()->checkSafety(); },
                                      CONFIG_SAFETY_CHECK_INTERVAL_US);

    // Create auxiliary task on Core 0
    xTaskCreatePinnedToCore(auxiliaryTask, "AuxiliaryTask", CONFIG_AUXILIARY_TASK_STACK_SIZE, NULL,
                            CONFIG_AUXILIARY_TASK_PRIORITY, &auxiliaryTaskHandle,
                            0  // Core 0
    );

    // Log successful initialization
    logger.logInfo("System initialized successfully");

    // Make sure serial command interface is properly started
    serialCommand.begin();

    // Print welcome message
    logger.logInfo(SYSTEM_NAME " initialized");
    delay(100);
    Serial.println("Type " ANSI_COLOR_BLUE "'help'" ANSI_COLOR_RESET " for available commands");
    Serial.print("> ");
}

/**
 * Main loop runs on Core 1 and handles critical real-time control tasks
 * This is where the highest priority motion control happens
 */
void loop() {
    // Execute all scheduled high-priority control tasks
    taskScheduler.executeControlTasks();

    // Make sure the status reporter is updating
    // statusReporter.updateStatus();

    // Periodically save motor positions (every 10 seconds)
    static uint32_t lastSaveTimeMs = 0;
    uint32_t currentTimeMs = millis();

    if (currentTimeMs - lastSaveTimeMs >= 10000) {
        systemManager.saveMotorPositions();
        lastSaveTimeMs = currentTimeMs;
    }

    // Allow a very short yield for watchdog feeding and critical system tasks
    // This is carefully tuned to not impact real-time performance
    delayMicroseconds(CONFIG_CONTROL_LOOP_YIELD_US);
}