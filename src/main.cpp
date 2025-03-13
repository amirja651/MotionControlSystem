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
  systemManager.getSerialCommand()->begin();
  systemManager.getStatusReporter()->begin();

  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency =
      pdMS_TO_TICKS(10);  // 100Hz update rate for non-critical tasks

  // Main auxiliary task loop
  while (true) {
    // Process incoming commands with priority
    systemManager.getSerialCommand()->processCommands();

    // Execute scheduled auxiliary tasks
    systemManager.getTaskScheduler()->executeAuxiliaryTasks();

    // Update system status at regular intervals
    systemManager.getStatusReporter()->updateStatus();

    // Log system information
    systemManager.getLogger()->processPendingLogs();

    // Yield to allow other Core 0 tasks to run
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  // Initialize serial communication for debugging and commands
  Serial.begin(CONFIG_SERIAL_BAUD_RATE);

  // Wait a moment for serial to stabilize
  delay(100);

  String output = ANSI_COLOR_MAGENTA;
  output += SYSTEM_NAME;
  output += ANSI_COLOR_RESET;
  output += " starting...";
  Serial.println(output);

  // Initialize system components with proper error handling
  if (!systemManager.initialize()) {
    output = ANSI_COLOR_RED;
    output += "System initialization failed";
    output += ANSI_COLOR_RESET;
    Serial.println(output);

    // Safety halt if critical initialization fails
    while (1) {
      delay(1000);
    }
  }

// Check for power failure recovery only if power monitoring is enabled
#if CONFIG_POWER_MONITORING_ENABLED
  if (!systemManager.wasNormalShutdown()) {
    // Power interruption detected
    systemManager.getLogger()->logInfo(
        "Power interruption detected. Restoring previous positions...",
        LogModule::SYSTEM);
    systemManager.restoreMotorPositions();
  }
#else
  systemManager.getLogger()->logWarning(
      "Power monitoring disabled. Skipping power failure recovery.",
      LogModule::SYSTEM);
#endif

  // Reset the shutdown flag for next time
  systemManager.setNormalShutdown(false);

  // Configure the task scheduler
  // Register scheduled tasks for Core 1 (control loop)
  for (uint8_t i = 0; i < systemManager.getMotorManager()->getMotorCount(); i++) {
    Motor *motor = systemManager.getMotorManager()->getMotor(i);

    systemManager.getLogger()->logInfo("REGISTERING MOTOR TASK", LogModule::SYSTEM);

    // Register high-priority motor control tasks
    systemManager.getTaskScheduler()->registerControlTask(
        [motor]() {
          motor->updateControl();  // Runs the PID loop and control logic
        },
        2000);  // Increase interval from 1000μs to 2000μs
                // motor->getControlInterval());

    // Register motion profile updates (slightly lower priority)
    systemManager.getTaskScheduler()->registerControlTask(
        [motor]() {
          motor->updateTrajectory();  // Updates position/velocity profiles
        },
        motor->getTrajectoryUpdateInterval());
  }

  // Register system monitoring tasks
  systemManager.getTaskScheduler()->registerControlTask(
      []() { systemManager.getSafetyMonitor()->checkSafety(); },
      CONFIG_SAFETY_CHECK_INTERVAL_US);

  // Create auxiliary task on Core 0
  xTaskCreatePinnedToCore(auxiliaryTask,
                          "AuxiliaryTask",
                          CONFIG_AUXILIARY_TASK_STACK_SIZE,
                          NULL,
                          CONFIG_AUXILIARY_TASK_PRIORITY,
                          &auxiliaryTaskHandle,
                          0  // Core 0
  );

  // Log successful initialization
  systemManager.getLogger()->logInfo("System initialized successfully",
                                     LogModule::SYSTEM);

  // Make sure serial command interface is properly started
  systemManager.getSerialCommand()->begin();

  // Print welcome message
  systemManager.getLogger()->logInfo(SYSTEM_NAME " initialized", LogModule::SYSTEM);
  delay(100);
  output = "Type ";
  output += ANSI_COLOR_BLUE;
  output += "'help'";
  output += ANSI_COLOR_RESET;
  output += " for available commands";
  Serial.println(output);
  Serial.print("> ");
}

/**
 * Main loop runs on Core 1 and handles critical real-time control tasks
 * This is where the highest priority motion control happens
 */
void loop() {
  // Execute all scheduled high-priority control tasks
  systemManager.getTaskScheduler()->executeControlTasks();

  // Make sure the status reporter is updating
  // statusReporter.updateStatus();

  // Periodically save motor positions (every 10 seconds)
  static uint32_t lastSaveTimeMs = 0;
  uint32_t        currentTimeMs  = millis();

  if (currentTimeMs - lastSaveTimeMs >= 10000) {
    systemManager.saveMotorPositions();
    lastSaveTimeMs = currentTimeMs;
  }

  // Allow a very short yield for watchdog feeding and critical system tasks
  // This is carefully tuned to not impact real-time performance
  delayMicroseconds(CONFIG_CONTROL_LOOP_YIELD_US);
}
// End of Code