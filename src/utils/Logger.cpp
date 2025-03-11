/*
 * ESP32 High-Precision Motion Control System
 * Logger Implementation
 */

 #include "Logger.h"

 Logger::Logger()
     : m_globalLogLevel(static_cast<LogLevel>(CONFIG_LOG_LEVEL)),
       m_serialOutputEnabled(true),
       m_serialBaudRate(CONFIG_SERIAL_BAUD_RATE),
       m_taskEnabled(false),
       m_initialized(false),
       m_taskHandle(nullptr),
       m_logQueue(nullptr),
       m_bufferSize(CONFIG_MAX_LOG_ENTRIES),
       m_bufferIndex(0) {
     
     // Initialize module levels
     for (size_t i = 0; i < static_cast<size_t>(LogModule::MAX_MODULES); i++) {
         m_moduleLevels[i] = m_globalLogLevel;
         m_moduleEnabled[i] = true;
     }
 
     // Set module-specific enabled flags based on config
     m_moduleEnabled[static_cast<size_t>(LogModule::SYSTEM)] = CONFIG_LOG_SYSTEM_ENABLED;
     m_moduleEnabled[static_cast<size_t>(LogModule::MOTOR_MANAGER)] = CONFIG_LOG_MOTORMANAGER_ENABLED;
     m_moduleEnabled[static_cast<size_t>(LogModule::STEPPER_DRIVER)] = CONFIG_LOG_STEPPERDRIVER_ENABLED;
     m_moduleEnabled[static_cast<size_t>(LogModule::PID_CONTROLLER)] = CONFIG_LOG_PIDCONTROLLER_ENABLED;
     m_moduleEnabled[static_cast<size_t>(LogModule::ENCODER)] = CONFIG_LOG_ENCODER_ENABLED;
     m_moduleEnabled[static_cast<size_t>(LogModule::SAFETY_MONITOR)] = CONFIG_LOG_SAFETYMONITOR_ENABLED;
     m_moduleEnabled[static_cast<size_t>(LogModule::COMMAND_HANDLER)] = CONFIG_LOG_COMMANDHANDLER_ENABLED;
 
     // Pre-allocate buffer to avoid reallocations
     m_logBuffer.reserve(m_bufferSize);
 }
 
 Logger::~Logger() {
     // Clean up FreeRTOS resources
     if (m_taskEnabled && m_taskHandle != nullptr) {
         vTaskDelete(m_taskHandle);
         m_taskHandle = nullptr;
     }
 
     if (m_logQueue != nullptr) {
         vQueueDelete(m_logQueue);
         m_logQueue = nullptr;
     }
 }
 
 bool Logger::initialize(uint32_t serialBaudRate, const LoggerTaskParams& taskParams) {
     // If already initialized, don't re-initialize
     if (m_initialized) {
         return true;
     }
 
     // If serial baud rate specified, use it; otherwise use default
     if (serialBaudRate > 0) {
         m_serialBaudRate = serialBaudRate;
     }
 
     // If serial isn't already initialized, initialize it
     if (!Serial) {
         Serial.begin(m_serialBaudRate);
 
         // Wait for serial to connect (max 500ms)
         uint32_t startTime = millis();
         while (!Serial && (millis() - startTime < 500)) {
             delay(10);
         }
     }
 
     // Initialize FreeRTOS task
     if (!initializeTask(taskParams)) {
         return false;
     }
 
     m_initialized = true;
 
     // Log initialization
     logInfo("Logger initialized");
 
     return true;
 }
 
 bool Logger::initializeTask(const LoggerTaskParams& taskParams) {
     // Store task parameters
     m_taskParams = taskParams;
     
     // Create log queue
     m_logQueue = xQueueCreate(m_taskParams.queueSize, sizeof(LogEntry*));
     if (m_logQueue == nullptr) {
         return false;
     }
 
     // Create task
     BaseType_t result = xTaskCreatePinnedToCore(
         loggerTask,
         "LoggerTask",
         m_taskParams.taskStackSize,
         this,
         m_taskParams.taskPriority,
         &m_taskHandle,
         m_taskParams.taskCoreId
     );
 
     if (result != pdPASS) {
         // Clean up queue if task creation failed
         vQueueDelete(m_logQueue);
         m_logQueue = nullptr;
         return false;
     }
 
     m_taskEnabled = true;
     return true;
 }
 
 void Logger::loggerTask(void* pvParameters) {
     Logger* logger = static_cast<Logger*>(pvParameters);
     LogEntry* entry = nullptr;
     TickType_t waitTime = pdMS_TO_TICKS(100); // 100ms wait time
 
     for(;;) {
         // Wait for an entry from the queue
         if (xQueueReceive(logger->m_logQueue, &entry, waitTime) == pdTRUE) {
             if (entry != nullptr) {
                 // Process the log entry
                 logger->outputLogEntry(*entry);
                 
                 // Add to buffer for history
                 logger->m_logBuffer.push_back(*entry);
                 if (logger->m_logBuffer.size() > logger->m_bufferSize) {
                     logger->m_logBuffer.erase(logger->m_logBuffer.begin());
                 }
                 
                 // Clean up the dynamically allocated entry
                 delete entry;
             }
         }
         
         // Process any pending logs that might have come in via other methods
         logger->processPendingLogs();
         
         // Give other tasks a chance to run
         taskYIELD();
     }
 }
 
 void Logger::setLogLevel(LogLevel level) {
     m_globalLogLevel = level;
     logInfo("Log level set to " + logLevelToString(level));
 }
 
 LogLevel Logger::getLogLevel() const {
     return m_globalLogLevel;
 }
 
 void Logger::setModuleLogLevel(LogModule module, LogLevel level) {
     size_t moduleIndex = static_cast<size_t>(module);
     if (moduleIndex < static_cast<size_t>(LogModule::MAX_MODULES)) {
         m_moduleLevels[moduleIndex] = level;
         logInfo("Module " + moduleToString(module) + " log level set to " + logLevelToString(level));
     }
 }
 
 LogLevel Logger::getModuleLogLevel(LogModule module) const {
     size_t moduleIndex = static_cast<size_t>(module);
     if (moduleIndex < static_cast<size_t>(LogModule::MAX_MODULES)) {
         return m_moduleLevels[moduleIndex];
     }
     return m_globalLogLevel;
 }
 
 void Logger::enableModuleLogging(LogModule module, bool enable) {
     size_t moduleIndex = static_cast<size_t>(module);
     if (moduleIndex < static_cast<size_t>(LogModule::MAX_MODULES)) {
         m_moduleEnabled[moduleIndex] = enable;
         logInfo("Module " + moduleToString(module) + " logging " + (enable ? "enabled" : "disabled"));
     }
 }
 
 bool Logger::isModuleLoggingEnabled(LogModule module) const {
     size_t moduleIndex = static_cast<size_t>(module);
     if (moduleIndex < static_cast<size_t>(LogModule::MAX_MODULES)) {
         return m_moduleEnabled[moduleIndex];
     }
     return false;
 }
 
 void Logger::logError(const String& message, LogModule module) {
     log(LogLevel::ERROR, message, module);
 }
 
 void Logger::logWarning(const String& message, LogModule module) {
     log(LogLevel::WARNING, message, module);
 }
 
 void Logger::logInfo(const String& message, LogModule module) {
     log(LogLevel::INFO, message, module);
 }
 
 void Logger::logDebug(const String& message, LogModule module) {
     log(LogLevel::DEBUG, message, module);
 }
 
 void Logger::logVerbose(const String& message, LogModule module) {
     log(LogLevel::VERBOSE, message, module);
 }
 
 void Logger::processPendingLogs() {
     // This method is already handled by the FreeRTOS task
     // but exists for compatibility and manual processing if needed
 }
 
 void Logger::clearLogBuffer() {
     m_logBuffer.clear();
     m_bufferIndex = 0;
 }
 
 void Logger::enableSerialOutput(bool enable) {
     m_serialOutputEnabled = enable;
 }
 
 bool Logger::addLogOutput(LogOutputCallback callback) {
     if (callback) {
         m_logOutputs.push_back(callback);
         return true;
     }
     return false;
 }
 
 void Logger::clearLogOutputs() {
     m_logOutputs.clear();
 }
 
 size_t Logger::getLogCount() const {
     return m_logBuffer.size();
 }
 
 const LogEntry* Logger::getLogEntry(size_t index) const {
     if (index < m_logBuffer.size()) {
         return &m_logBuffer[index];
     }
     return nullptr;
 }
 
 String Logger::logLevelToString(LogLevel level) {
     switch (level) {
         case LogLevel::OFF:
             return "OFF";
         case LogLevel::ERROR:
             return "ERROR";
         case LogLevel::WARNING:
             return "WARN";
         case LogLevel::INFO:
             return "INFO";
         case LogLevel::DEBUG:
             return "DEBUG";
         case LogLevel::VERBOSE:
             return "VERB";
         default:
             return "????";
     }
 }
 
 String Logger::moduleToString(LogModule module) {
     switch (module) {
         case LogModule::SYSTEM:
             return "SYS";
         case LogModule::MOTOR_MANAGER:
             return "MOT";
         case LogModule::STEPPER_DRIVER:
             return "DRV";
         case LogModule::PID_CONTROLLER:
             return "PID";
         case LogModule::ENCODER:
             return "ENC";
         case LogModule::SAFETY_MONITOR:
             return "SAF";
         case LogModule::COMMAND_HANDLER:
             return "CMD";
         case LogModule::CUSTOM_1:
             return "CUS1";
         case LogModule::CUSTOM_2:
             return "CUS2";
         case LogModule::CUSTOM_3:
             return "CUS3";
         default:
             return "???";
     }
 }
 
 void Logger::log(LogLevel level, const String& message, LogModule module) {
     // Skip if logger is not initialized
     if (!m_initialized) {
         return;
     }
 
     // Check module log level and enabled status
     size_t moduleIndex = static_cast<size_t>(module);
     if (moduleIndex >= static_cast<size_t>(LogModule::MAX_MODULES) ||
         !m_moduleEnabled[moduleIndex] || 
         level == LogLevel::OFF || 
         level > m_moduleLevels[moduleIndex]) {
         return;
     }
 
     // Create a log entry (dynamically allocated for queue)
     LogEntry* entry = new LogEntry();
     entry->timestamp = millis();
     entry->level = level;
     entry->module = module;
     entry->message = message;
 
     // If critical (error), output immediately regardless of task
     if (level == LogLevel::ERROR) {
         outputLogEntry(*entry);
     }
 
     // Queue for async processing
     bool queued = queueLogEntry(*entry);
     
     // If queueing failed, handle immediately
     if (!queued) {
         outputLogEntry(*entry);
         
         // Add to buffer for history
         if (m_logBuffer.size() >= m_bufferSize) {
             m_logBuffer.erase(m_logBuffer.begin());
         }
         m_logBuffer.push_back(*entry);
 
         // Clean up if not queued
         delete entry;
     }
 }
 
 bool Logger::queueLogEntry(const LogEntry& entry) {
     if (!m_taskEnabled || m_logQueue == nullptr) {
         return false;
     }
 
     // Create a copy of the entry for the queue
     LogEntry* entryCopy = new LogEntry(entry);
     
     // Send to queue with timeout (don't block indefinitely)
     if (xQueueSend(m_logQueue, &entryCopy, 0) != pdPASS) {
         // Failed to queue, clean up
         delete entryCopy;
         return false;
     }
     
     return true;
 }
 
 void Logger::outputLogEntry(const LogEntry& entry) {
     String formattedMessage = formatLogEntry(entry);
 
     // Output to serial if enabled
     if (m_serialOutputEnabled && Serial) {
         Serial.println(formattedMessage);
     }
 
     // Output to custom outputs
     for (const auto& callback : m_logOutputs) {
         if (callback) {
             callback(entry);
         }
     }
 }
 
 String Logger::formatLogEntry(const LogEntry& entry) const {
     // Add appropriate color based on log level
     String colorCode = "";
     String resetCode = ANSI_COLOR_RESET;
 
     switch (entry.level) {
         case LogLevel::ERROR:
             colorCode = ANSI_COLOR_RED;
             break;
         case LogLevel::WARNING:
             colorCode = ANSI_COLOR_YELLOW;
             break;
         case LogLevel::INFO:
             colorCode = ANSI_COLOR_GREEN;
             break;
         case LogLevel::DEBUG:
             colorCode = ANSI_COLOR_CYAN;
             break;
         case LogLevel::VERBOSE:
             colorCode = ANSI_COLOR_BLUE;
             break;
         default:
             // No color for other levels
             colorCode = "";
             resetCode = "";
             break;
     }
 
     // Format: [timestamp] LEVEL [MODULE]: message
     String formatted = colorCode + "[";
     formatted += String(entry.timestamp);
     formatted += "] ";
     formatted += logLevelToString(entry.level);
     formatted += " [";
     formatted += moduleToString(entry.module);
     formatted += "]: ";
     formatted += entry.message;
     formatted += resetCode;  // Reset color at the end
 
     return formatted;
 }
 // End of Code