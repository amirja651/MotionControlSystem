/*
 * ESP32 High-Precision Motion Control System
 * Main sketch file
 * 
 * This file initializes the motion control system and sets up the dual-core
 * task distribution for optimal real-time performance.
 */

 #include <Arduino.h>
 #include "Configuration.h"
 #include "SystemManager.h"
 #include "MotorManager.h"
 #include "utils/TaskScheduler.h"
 #include "utils/Logger.h"
 #include "communication/SerialCommand.h"
 #include "communication/StatusReporter.h"
 
 // System components
 SystemManager systemManager;
 MotorManager motorManager;
 TaskScheduler taskScheduler;
 Logger logger;
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
   const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz update rate for non-critical tasks
   
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
   
   // Initialize system components with proper error handling
   if (!systemManager.initialize()) {
     logger.logError("System initialization failed");
     while (1) { delay(1000); } // Safety halt if critical initialization fails
   }
   
   // Initialize the motor manager with the configured motors
   if (!motorManager.initialize()) {
     logger.logError("Motor manager initialization failed");
     while (1) { delay(1000); }
   }
   
   // Configure the task scheduler
   taskScheduler.initialize();
   
   // Register scheduled tasks for Core 1 (control loop)
   for (uint8_t i = 0; i < motorManager.getMotorCount(); i++) {
     Motor* motor = motorManager.getMotor(i);
     
     // Register high-priority motor control tasks
     taskScheduler.registerControlTask([motor]() {
       motor->updateControl(); // Runs the PID loop and control logic
     }, motor->getControlInterval());
     
     // Register motion profile updates (slightly lower priority)
     taskScheduler.registerControlTask([motor]() {
       motor->updateTrajectory(); // Updates position/velocity profiles
     }, motor->getTrajectoryUpdateInterval());
   }
   
   // Register system monitoring tasks
   taskScheduler.registerControlTask([]() {
     systemManager.getSafetyMonitor()->checkSafety();
   }, CONFIG_SAFETY_CHECK_INTERVAL_US);
   
   // Create auxiliary task on Core 0
   xTaskCreatePinnedToCore(
     auxiliaryTask,
     "AuxiliaryTask",
     CONFIG_AUXILIARY_TASK_STACK_SIZE,
     NULL,
     CONFIG_AUXILIARY_TASK_PRIORITY,
     &auxiliaryTaskHandle,
     0 // Core 0
   );
   
   // Log successful initialization
   logger.logInfo("System initialized successfully");
   
   // Print welcome message
   Serial.println("ESP32 Motion Control System initialized");
   Serial.println("Type 'help' for available commands");
 }
 
 /**
  * Main loop runs on Core 1 and handles critical real-time control tasks
  * This is where the highest priority motion control happens
  */
 void loop() {
   // Execute all scheduled high-priority control tasks
   taskScheduler.executeControlTasks();
   
   // Allow a very short yield for watchdog feeding and critical system tasks
   // This is carefully tuned to not impact real-time performance
   delayMicroseconds(CONFIG_CONTROL_LOOP_YIELD_US);
 }