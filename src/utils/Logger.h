/*
 * ESP32 High-Precision Motion Control System
 * Logger
 * 
 * Provides diagnostic logging with configurable log levels,
 * buffering, and multiple output destinations.
 */

 #ifndef LOGGER_H
 #define LOGGER_H
 
 #include <Arduino.h>
 #include <functional>
 #include <vector>
 #include "../Configuration.h"
 
 /**
  * Log levels
  */
 enum class LogLevel {
     OFF = 0,       // No logging
     ERROR = 1,     // Critical errors only
     WARNING = 2,   // Warnings and errors
     INFO = 3,      // Informational messages, warnings, and errors
     DEBUG = 4,     // Debug messages and below
     VERBOSE = 5    // All messages
 };
 
 /**
  * Log entry structure
  */
 struct LogEntry {
     uint32_t timestamp;     // Timestamp in milliseconds
     LogLevel level;         // Log level
     String message;         // Log message
     uint8_t source;         // Source component (optional)
 };
 
 /**
  * Log output callback function
  * Called when a log entry needs to be output
  */
 using LogOutputCallback = std::function<void(const LogEntry&)>;
 
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
      * Initialize the logger
      * 
      * @param serialBaudRate Serial baud rate for log output
      * @return True if initialization successful, false otherwise
      */
     bool initialize(uint32_t serialBaudRate = 0);
     
     /**
      * Set log level
      * Only messages at this level or higher priority will be logged
      * 
      * @param level Log level
      */
     void setLogLevel(LogLevel level);
     
     /**
      * Get current log level
      * 
      * @return Current log level
      */
     LogLevel getLogLevel() const;
     
     /**
      * Log an error message
      * 
      * @param message Message to log
      * @param source Source component (optional)
      */
     void logError(const String& message, uint8_t source = 0);
     
     /**
      * Log a warning message
      * 
      * @param message Message to log
      * @param source Source component (optional)
      */
     void logWarning(const String& message, uint8_t source = 0);
     
     /**
      * Log an info message
      * 
      * @param message Message to log
      * @param source Source component (optional)
      */
     void logInfo(const String& message, uint8_t source = 0);
     
     /**
      * Log a debug message
      * 
      * @param message Message to log
      * @param source Source component (optional)
      */
     void logDebug(const String& message, uint8_t source = 0);
     
     /**
      * Log a verbose message
      * 
      * @param message Message to log
      * @param source Source component (optional)
      */
     void logVerbose(const String& message, uint8_t source = 0);
     
     /**
      * Process any pending log entries
      * Should be called periodically from non-critical code
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
     
 private:
     // Logger state
     LogLevel m_logLevel;
     bool m_serialOutputEnabled;
     uint32_t m_serialBaudRate;
     
     // Log buffer
     std::vector<LogEntry> m_logBuffer;
     size_t m_bufferSize;
     size_t m_bufferIndex;
     
     // Custom log outputs
     std::vector<LogOutputCallback> m_logOutputs;
     
     /**
      * Log a message with the specified level
      * 
      * @param level Log level
      * @param message Message to log
      * @param source Source component (optional)
      */
     void log(LogLevel level, const String& message, uint8_t source);
     
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
 };
 
 #endif // LOGGER_H