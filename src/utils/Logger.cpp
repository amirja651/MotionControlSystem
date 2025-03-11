/*
 * ESP32 High-Precision Motion Control System
 * Logger Implementation
 */

#include "Logger.h"

Logger::Logger()
    : m_logLevel(static_cast<LogLevel>(CONFIG_LOG_LEVEL)),
      m_serialOutputEnabled(true),
      m_serialBaudRate(CONFIG_SERIAL_BAUD_RATE),
      m_bufferSize(CONFIG_MAX_LOG_ENTRIES),
      m_bufferIndex(0) {
    // Pre-allocate buffer to avoid reallocations
    m_logBuffer.reserve(m_bufferSize);
}

bool Logger::initialize(uint32_t serialBaudRate) {
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

    // Log initialization
    logInfo("Logger initialized");

    return true;
}

void Logger::setLogLevel(LogLevel level) {
    m_logLevel = level;
    logInfo("Log level set to " + logLevelToString(level));
}

LogLevel Logger::getLogLevel() const {
    return m_logLevel;
}

void Logger::logError(const String& message, uint8_t source) {
    log(LogLevel::ERROR, message, source);
}

void Logger::logWarning(const String& message, uint8_t source) {
    log(LogLevel::WARNING, message, source);
}

void Logger::logInfo(const String& message, uint8_t source) {
    log(LogLevel::INFO, message, source);
}

void Logger::logDebug(const String& message, uint8_t source) {
    log(LogLevel::DEBUG, message, source);
}

void Logger::logVerbose(const String& message, uint8_t source) {
    log(LogLevel::VERBOSE, message, source);
}

void Logger::processPendingLogs() {
    // Process all pending logs in the buffer
    for (size_t i = 0; i < m_logBuffer.size(); i++) {
        outputLogEntry(m_logBuffer[i]);
    }

    // Clear the buffer after processing
    m_logBuffer.clear();
    m_bufferIndex = 0;
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
            return "WARNING";
        case LogLevel::INFO:
            return "INFO";
        case LogLevel::DEBUG:
            return "DEBUG";
        case LogLevel::VERBOSE:
            return "VERBOSE";
        default:
            return "UNKNOWN";
    }
}

void Logger::log(LogLevel level, const String& message, uint8_t source) {
    // Check if this message should be logged based on level
    if (level == LogLevel::OFF || level > m_logLevel) {
        return;
    }

    // Create a log entry
    LogEntry entry;
    entry.timestamp = millis();
    entry.level = level;
    entry.message = message;
    entry.source = source;

    // If critical (error), output immediately
    if (level == LogLevel::ERROR) {
        outputLogEntry(entry);
    }

    // Add to buffer for later processing
    if (m_logBuffer.size() < m_bufferSize) {
        m_logBuffer.push_back(entry);
    } else {
        // Buffer full, replace oldest entry
        if (m_bufferIndex >= m_bufferSize) {
            m_bufferIndex = 0;
        }

        if (m_bufferIndex < m_logBuffer.size()) {
            m_logBuffer[m_bufferIndex] = entry;
            m_bufferIndex++;
        } else {
            // Should never happen, but just in case
            m_logBuffer.push_back(entry);
        }
    }
}

void Logger::outputLogEntry(const LogEntry& entry) {
    String formattedMessage = "";

#if CONFIG_COLOR_OUTPUT_ENABLED
    // Format the log entry
    formattedMessage = formatLogEntry(entry);

#endif

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

    // Format: [timestamp] LEVEL [source]: message
    String formatted = colorCode + "[";
    formatted += String(entry.timestamp);
    formatted += "] ";
    formatted += logLevelToString(entry.level);

    if (entry.source > 0) {
        formatted += " [";
        formatted += String(entry.source);
        formatted += "]";
    }

    formatted += ": ";
    formatted += entry.message;
    formatted += resetCode;  // Reset color at the end

    return formatted;
}