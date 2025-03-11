/*
 * ESP32 High-Precision Motion Control System
 * Status Reporter
 *
 * Provides system-wide status reporting, including motor positions,
 * velocities, system state, and diagnostic information.
 */

#ifndef STATUS_REPORTER_H
#define STATUS_REPORTER_H

#include <Arduino.h>

#include <functional>
#include <vector>

#include "../Configuration.h"
#include "../MotorManager.h"
#include "../core/Motor.h"
#include "../core/SafetyMonitor.h"
#include "../utils/TaskScheduler.h"

// Forward declarations
class SystemManager;

/**
 * Status report structure
 */
struct SystemStatus {
    uint32_t timestamp;                         // Timestamp in milliseconds
    uint8_t systemState;                        // System state
    bool emergencyStop;                         // Emergency stop status
    uint8_t motorCount;                         // Number of motors
    int32_t motorPositions[CONFIG_MAX_MOTORS];  // Motor positions
    float motorVelocities[CONFIG_MAX_MOTORS];   // Motor velocities
    uint8_t motorStates[CONFIG_MAX_MOTORS];     // Motor states
    float cpuUsageCore0;                        // CPU usage for Core 0
    float cpuUsageCore1;                        // CPU usage for Core 1
    uint32_t freeMemory;                        // Free memory in bytes
    uint32_t uptimeMs;                          // System uptime in milliseconds
    float controlLoopTimeUs;                    // Control loop execution time
    uint32_t missedDeadlines;                   // Number of missed control loop deadlines
};

/**
 * Status update callback function
 */
using StatusUpdateCallback = std::function<void(const SystemStatus &)>;

/**
 * Status Reporter class
 */
class StatusReporter {
   public:
    /**
     * Constructor
     *
     * @param systemManager Pointer to system manager
     * @param updateFrequencyHz Status update frequency in Hz
     */
    StatusReporter(SystemManager *systemManager,
                   uint8_t updateFrequencyHz = CONFIG_STATUS_UPDATE_FREQUENCY_HZ);

    /**
     * Initialize the status reporter
     *
     * @return True if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * Begin status reporting
     */
    void begin();

    /**
     * Set update frequency
     *
     * @param frequencyHz Update frequency in Hz
     */
    void setUpdateFrequency(uint8_t frequencyHz);

    /**
     * Update system status
     * Should be called periodically
     */
    void updateStatus();

    /**
     * Get current system status
     *
     * @return System status structure
     */
    const SystemStatus &getStatus() const;

    /**
     * Add status update callback
     *
     * @param callback Callback function
     * @return True if callback added successfully, false otherwise
     */
    bool addStatusCallback(StatusUpdateCallback callback);

    /**
     * Clear all status callbacks
     */
    void clearStatusCallbacks();

    /**
     * Enable/disable JSON status output over serial
     *
     * @param enable True to enable, false to disable
     */
    void enableSerialOutput(bool enable);

    /**
     * Generate JSON status string
     *
     * @return JSON-formatted status string
     */
    String generateStatusJson() const;

   private:
    // References
    SystemManager *m_systemManager;

    // Status reporting state
    SystemStatus m_status;
    uint8_t m_updateFrequencyHz;
    uint32_t m_lastUpdateMs;
    bool m_serialOutputEnabled;

    // Status callbacks
    std::vector<StatusUpdateCallback> m_statusCallbacks;

    /**
     * Collect system status data
     */
    void collectStatusData();

    /**
     * Notify status callbacks
     */
    void notifyStatusCallbacks();
};

#endif  // STATUS_REPORTER_H
// End of Code