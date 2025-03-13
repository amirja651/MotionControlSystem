/*
 * ESP32 High-Precision Motion Control System
 * Logger
 *
 * Provides diagnostic logging with configurable log levels per module,
 * FreeRTOS task-based logging, and multiple output destinations.
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <functional>
#include <unordered_map>
#include <vector>

#include "../Configuration.h"

/**
 * Log levels
 */
enum class LogLevel {
    OFF     = 0,  // No logging
    ERROR   = 1,  // Critical errors only
    WARNING = 2,  // Warnings and errors
    INFO    = 3,  // Informational messages, warnings, and errors
    DEBUG   = 4,  // Debug messages and below
    VERBOSE = 5   // All messages
};

/**
 * Module identifiers for module-specific logging control
 */
enum class LogModule : uint8_t {
    SYSTEM          = 0,   // System-wide logs
    MOTOR_MANAGER   = 1,   // Motor manager logs
    STEPPER_DRIVER  = 2,   // Stepper driver logs
    PID_CONTROLLER  = 3,   // PID controller logs
    ENCODER         = 4,   // Encoder logs
    SAFETY_MONITOR  = 5,   // Safety monitor logs
    COMMAND_HANDLER = 6,   // Command handler logs
    CUSTOM_1        = 7,   // Custom module 1
    CUSTOM_2        = 8,   // Custom module 2
    CUSTOM_3        = 9,   // Custom module 3
    MAX_MODULES     = 10,  // Maximum number of modules
    MOTOR           = 11   // Motor logs
};

/**
 * Log entry structure
 */
struct LogEntry {
    uint32_t  timestamp;  // Timestamp in milliseconds
    LogLevel  level;      // Log level
    LogModule module;     // Source module
    String message;  // Log message (using reference counting to avoid copies)
};

/**
 * Log output callback function
 * Called when a log entry needs to be output
 */
using LogOutputCallback = std::function<void(const LogEntry&)>;

/**
 * FreeRTOS task parameters
 */
struct LoggerTaskParams {
    int taskStackSize = 2048;  // Default stack size
    int taskPriority  = 1;     // Default priority (low)
    int taskCoreId    = 0;     // Default core (0)
    int queueSize     = 20;    // Default queue size
};

/**
 * Logger class for diagnostic logging
 */
class Logger {
public:
    /**
     * Constructor
     */
    Logger();

    /**
     * Destructor
     */
    ~Logger();

    /**
     * Initialize the logger
     *
     * @param serialBaudRate Serial baud rate for log output
     * @param taskParams FreeRTOS task parameters
     * @return True if initialization successful, false otherwise
     */
    virtual bool initialize(
        uint32_t                serialBaudRate = 0,
        const LoggerTaskParams& taskParams     = LoggerTaskParams());

    /**
     * Set global log level
     * Only messages at this level or higher priority will be logged
     *
     * @param level Log level
     */
    void setLogLevel(LogLevel level);

    /**
     * Get global log level
     *
     * @return Current log level
     */
    LogLevel getLogLevel() const;

    /**
     * Set module-specific log level
     *
     * @param module Module identifier
     * @param level Log level for this module
     */
    void setModuleLogLevel(LogModule module, LogLevel level);

    /**
     * Get module-specific log level
     *
     * @param module Module identifier
     * @return Log level for this module
     */
    LogLevel getModuleLogLevel(LogModule module) const;

    /**
     * Enable or disable logging for a specific module
     *
     * @param module Module identifier
     * @param enable True to enable, false to disable
     */
    void enableModuleLogging(LogModule module, bool enable);

    /**
     * Check if logging is enabled for a specific module
     *
     * @param module Module identifier
     * @return True if enabled, false if disabled
     */
    bool isModuleLoggingEnabled(LogModule module) const;

    /**
     * Log an error message
     *
     * @param message Message to log
     * @param module Source module (default: SYSTEM)
     */
    virtual void logError(const String& message,
                          LogModule     module = LogModule::SYSTEM);

    /**
     * Log a warning message
     *
     * @param message Message to log
     * @param module Source module (default: SYSTEM)
     */
    virtual void logWarning(const String& message,
                            LogModule     module = LogModule::SYSTEM);

    /**
     * Log an info message
     *
     * @param message Message to log
     * @param module Source module (default: SYSTEM)
     */
    virtual void logInfo(const String& message,
                         LogModule     module = LogModule::SYSTEM);

    /**
     * Log a debug message
     *
     * @param message Message to log
     * @param module Source module (default: SYSTEM)
     */
    virtual void logDebug(const String& message,
                          LogModule     module = LogModule::SYSTEM);

    /**
     * Log a verbose message
     *
     * @param message Message to log
     * @param module Source module (default: SYSTEM)
     */
    virtual void logVerbose(const String& message,
                            LogModule     module = LogModule::SYSTEM);

    /**
     * Process any pending log entries
     * This is called automatically by the task, but can be called
     * manually if task-based logging is disabled
     */
    void processPendingLogs();

    /**
     * Clear the log buffer
     */
    void clearLogBuffer();

    /**
     * Enable/disable serial output
     *
     * @param enable True to enable, false to disable
     */
    void enableSerialOutput(bool enable);

    /**
     * Add a custom log output
     *
     * @param callback Callback function to handle log output
     * @return True if added successfully, false otherwise
     */
    bool addLogOutput(LogOutputCallback callback);

    /**
     * Remove all custom log outputs
     */
    void clearLogOutputs();

    /**
     * Get number of entries in log buffer
     *
     * @return Number of log entries
     */
    size_t getLogCount() const;

    /**
     * Get a log entry by index
     *
     * @param index Log entry index
     * @return Log entry or nullptr if index out of range
     */
    const LogEntry* getLogEntry(size_t index) const;

    /**
     * Convert log level to string
     *
     * @param level Log level
     * @return String representation of log level
     */
    static String logLevelToString(LogLevel level);

    /**
     * Convert module to string
     *
     * @param module Module identifier
     * @return String representation of module
     */
    static String moduleToString(LogModule module);

private:
    // Logger state
    LogLevel m_globalLogLevel;
    bool     m_serialOutputEnabled;
    uint32_t m_serialBaudRate;
    bool     m_taskEnabled;
    bool     m_initialized;

    // Per-module log levels and state
    LogLevel m_moduleLevels[static_cast<size_t>(LogModule::MAX_MODULES)];
    bool     m_moduleEnabled[static_cast<size_t>(LogModule::MAX_MODULES)];

    // FreeRTOS task and queue
    TaskHandle_t     m_taskHandle;
    QueueHandle_t    m_logQueue;
    LoggerTaskParams m_taskParams;

    // Log buffer for viewing
    std::vector<LogEntry> m_logBuffer;
    size_t                m_bufferSize;
    size_t                m_bufferIndex;

    // Custom log outputs
    std::vector<LogOutputCallback> m_logOutputs;

    /**
     * Log a message with the specified level
     *
     * @param level Log level
     * @param message Message to log
     * @param module Source module
     */
    void log(LogLevel level, const String& message, LogModule module);

    /**
     * Queue a log entry for processing
     *
     * @param entry Log entry
     * @return True if queued successfully, false otherwise
     */
    bool queueLogEntry(const LogEntry& entry);

    /**
     * Output a log entry to all enabled outputs
     *
     * @param entry Log entry
     */
    void outputLogEntry(const LogEntry& entry);

    /**
     * Format a log entry for output
     *
     * @param entry Log entry
     * @return Formatted log message
     */
    String formatLogEntry(const LogEntry& entry) const;

    /**
     * FreeRTOS task function
     *
     * @param pvParameters Task parameters
     */
    static void loggerTask(void* pvParameters);

    /**
     * Initialize the logger task
     *
     * @param taskParams Task parameters
     * @return True if successful, false otherwise
     */
    bool initializeTask(const LoggerTaskParams& taskParams);
};

#endif  // LOGGER_H
        // End of Code