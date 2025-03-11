/*
 * ESP32 High-Precision Motion Control System
 * Status Reporter Implementation
 */

#include "StatusReporter.h"
#include "../SystemManager.h" // Include the complete definition here

StatusReporter::StatusReporter(SystemManager *systemManager, uint8_t updateFrequencyHz)
    : m_systemManager(systemManager),
      m_updateFrequencyHz(updateFrequencyHz > 0 ? updateFrequencyHz
                                                : CONFIG_STATUS_UPDATE_FREQUENCY_HZ),
      m_lastUpdateMs(0),
      m_serialOutputEnabled(false) {
    // Initialize status structure
    memset(&m_status, 0, sizeof(SystemStatus));
}

bool StatusReporter::initialize() {
    // Validate system manager
    if (m_systemManager == nullptr) {
        return false;
    }

    // Initial status update
    collectStatusData();

    return true;
}

void StatusReporter::begin() {
    // Reset last update time
    m_lastUpdateMs = millis();

    // Update status immediately
    updateStatus();
}

void StatusReporter::setUpdateFrequency(uint8_t frequencyHz) {
    m_updateFrequencyHz = frequencyHz > 0 ? frequencyHz : CONFIG_STATUS_UPDATE_FREQUENCY_HZ;
}

void StatusReporter::updateStatus() {
    uint32_t currentTimeMs = millis();

    // Check if it's time to update
    if (m_updateFrequencyHz > 0) {
        uint32_t updateIntervalMs = 1000 / m_updateFrequencyHz;

        if (currentTimeMs - m_lastUpdateMs >= updateIntervalMs) {
            // Collect current status data
            collectStatusData();

            // Notify callbacks
            notifyStatusCallbacks();

            // Output to serial if enabled
            if (m_serialOutputEnabled && Serial) {
                Serial.println(generateStatusJson());
            }

            // Update last update time
            m_lastUpdateMs = currentTimeMs;
        }
    }
}

const SystemStatus &StatusReporter::getStatus() const {
    return m_status;
}

bool StatusReporter::addStatusCallback(StatusUpdateCallback callback) {
    if (callback) {
        m_statusCallbacks.push_back(callback);
        return true;
    }
    return false;
}

void StatusReporter::clearStatusCallbacks() {
    m_statusCallbacks.clear();
}

void StatusReporter::enableSerialOutput(bool enable) {
    m_serialOutputEnabled = enable;
}

String StatusReporter::generateStatusJson() const {
    // Build JSON string with system status
    String json = "{";

    // System info
    json += "\"timestamp\":" + String(m_status.timestamp) + ",";
    json += "\"systemState\":" + String(m_status.systemState) + ",";
    json += "\"emergencyStop\":" + String(m_status.emergencyStop ? "true" : "false") + ",";
    json += "\"uptime\":" + String(m_status.uptimeMs) + ",";

    // CPU and memory info
    json += "\"cpu0\":" + String(m_status.cpuUsageCore0, 1) + ",";
    json += "\"cpu1\":" + String(m_status.cpuUsageCore1, 1) + ",";
    json += "\"memory\":" + String(m_status.freeMemory) + ",";

    // Control loop info
    json += "\"controlLoop\":" + String(m_status.controlLoopTimeUs) + ",";
    json += "\"missedDeadlines\":" + String(m_status.missedDeadlines) + ",";

    // Motors
    json += "\"motors\":[";
    for (uint8_t i = 0; i < m_status.motorCount; i++) {
        if (i > 0) {
            json += ",";
        }

        json += "{";
        json += "\"index\":" + String(i) + ",";
        json += "\"position\":" + String(m_status.motorPositions[i]) + ",";
        json += "\"velocity\":" + String(m_status.motorVelocities[i], 2) + ",";
        json += "\"state\":" + String(m_status.motorStates[i]);
        json += "}";
    }
    json += "]";

    json += "}";

    return json;
}

void StatusReporter::collectStatusData() {
    // Get current timestamp
    m_status.timestamp = millis();

    // System state
    m_status.systemState = static_cast<uint8_t>(m_systemManager->getSystemState());
    m_status.emergencyStop = m_systemManager->isEmergencyStop();
    m_status.uptimeMs = m_systemManager->getUptimeMs();

    // CPU and memory
    m_status.cpuUsageCore0 = m_systemManager->getCPUUsage(0);
    m_status.cpuUsageCore1 = m_systemManager->getCPUUsage(1);
    m_status.freeMemory = m_systemManager->getFreeMemory();

    // Control loop stats
    TaskScheduler *taskScheduler = m_systemManager->getTaskScheduler();
    if (taskScheduler != nullptr) {
        uint32_t controlTaskCount, auxiliaryTaskCount, totalMissedDeadlines,
            averageControlLoopTimeUs;
        taskScheduler->getSchedulerStats(controlTaskCount, auxiliaryTaskCount, totalMissedDeadlines,
                                         averageControlLoopTimeUs);

        m_status.controlLoopTimeUs = averageControlLoopTimeUs;
        m_status.missedDeadlines = totalMissedDeadlines;
    }

    // Motor data
    MotorManager *motorManager = m_systemManager->getMotorManager();
    if (motorManager != nullptr) {
        m_status.motorCount = motorManager->getMotorCount();

        // Limit to max motors supported in status structure
        if (m_status.motorCount > CONFIG_MAX_MOTORS) {
            m_status.motorCount = CONFIG_MAX_MOTORS;
        }

        // Collect data for each motor
        for (uint8_t i = 0; i < m_status.motorCount; i++) {
            Motor *motor = motorManager->getMotor(i);
            if (motor != nullptr) {
                const MotorState &motorState = motor->getState();

                m_status.motorPositions[i] = motorState.currentPosition;
                m_status.motorVelocities[i] = motorState.currentVelocity;
                m_status.motorStates[i] = static_cast<uint8_t>(motorState.status);
            } else {
                // Motor not available
                m_status.motorPositions[i] = 0;
                m_status.motorVelocities[i] = 0.0f;
                m_status.motorStates[i] = 0;
            }
        }
    } else {
        m_status.motorCount = 0;
    }
}

void StatusReporter::notifyStatusCallbacks() {
    // Call all registered callbacks with current status
    for (const auto &callback : m_statusCallbacks) {
        if (callback) {
            callback(m_status);
        }
    }
}
// End of Code