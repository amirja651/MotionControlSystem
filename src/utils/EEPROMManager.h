/*
 * ESP32 High-Precision Motion Control System
 * EEPROM Manager
 *
 * Manages storage and retrieval of configuration parameters in non-volatile memory,
 * providing persistent storage for motor calibration, PID tuning, and system settings.
 */

#ifndef EEPROM_MANAGER_H
#define EEPROM_MANAGER_H

#include <Arduino.h>
#include <EEPROM.h>

#include "../Configuration.h"

/**
 * EEPROM address map for system parameters
 */
enum EEPROMAddresses {
    ADDR_MAGIC_MARKER = 0,           // 2 bytes - magic marker for validation
    ADDR_VERSION = 2,                // 1 byte - configuration version
    ADDR_MOTOR_CONFIG_START = 4,     // Start of motor configuration blocks
    ADDR_MOTOR_CONFIG_SIZE = 64,     // Size of each motor configuration block
    ADDR_SYSTEM_CONFIG_START = 260,  // Start of system configuration block
    ADDR_SYSTEM_CONFIG_SIZE = 64,    // Size of system configuration block
    ADDR_SAFETY_CONFIG_START = 324,  // Start of safety configuration block
    ADDR_SAFETY_CONFIG_SIZE = 32,    // Size of safety configuration block
    ADDR_USER_DATA_START = 356,      // Start of user data area
    ADDR_USER_DATA_SIZE = 128        // Size of user data area
};

/**
 * EEPROM Manager class for parameter storage
 */
class EEPROMManager {
   public:
    /**
     * Constructor
     */
    EEPROMManager();

    /**
     * Initialize the EEPROM manager
     *
     * @return True if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * Check if EEPROM contains valid configuration data
     *
     * @return True if valid data found, false otherwise
     */
    bool isConfigValid() const;

    /**
     * Commit changes to EEPROM
     * This must be called after making changes to save them permanently
     *
     * @return True if commit successful, false otherwise
     */
    bool commit();

    /**
     * Reset EEPROM to default values
     *
     * @return True if reset successful, false otherwise
     */
    bool resetToDefaults();

    /**
     * Load motor PID parameters
     *
     * @param motorIndex Motor index
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param ff Feed-forward gain
     * @return True if parameters loaded successfully, false otherwise
     */
    bool loadPIDParameters(uint8_t motorIndex, float& kp, float& ki, float& kd, float& ff);

    /**
     * Save motor PID parameters
     *
     * @param motorIndex Motor index
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param ff Feed-forward gain
     * @return True if parameters saved successfully, false otherwise
     */
    bool savePIDParameters(uint8_t motorIndex, float kp, float ki, float kd, float ff);

    /**
     * Load motor profile parameters
     *
     * @param motorIndex Motor index
     * @param maxVelocity Maximum velocity
     * @param acceleration Acceleration
     * @param deceleration Deceleration
     * @param jerk Jerk limit
     * @return True if parameters loaded successfully, false otherwise
     */
    bool loadProfileParameters(uint8_t motorIndex, float& maxVelocity, float& acceleration,
                               float& deceleration, float& jerk);

    /**
     * Save motor profile parameters
     *
     * @param motorIndex Motor index
     * @param maxVelocity Maximum velocity
     * @param acceleration Acceleration
     * @param deceleration Deceleration
     * @param jerk Jerk limit
     * @return True if parameters saved successfully, false otherwise
     */
    bool saveProfileParameters(uint8_t motorIndex, float maxVelocity, float acceleration,
                               float deceleration, float jerk);

    /**
     * Load motor soft limits
     *
     * @param motorIndex Motor index
     * @param minLimit Minimum position limit
     * @param maxLimit Maximum position limit
     * @param enabled Whether soft limits are enabled
     * @return True if limits loaded successfully, false otherwise
     */
    bool loadSoftLimits(uint8_t motorIndex, int32_t& minLimit, int32_t& maxLimit, bool& enabled);

    /**
     * Save motor soft limits
     *
     * @param motorIndex Motor index
     * @param minLimit Minimum position limit
     * @param maxLimit Maximum position limit
     * @param enabled Whether soft limits are enabled
     * @return True if limits saved successfully, false otherwise
     */
    bool saveSoftLimits(uint8_t motorIndex, int32_t minLimit, int32_t maxLimit, bool enabled);

    /**
     * Load system configuration
     *
     * @param logLevel Log level
     * @param debugEnabled Whether debug mode is enabled
     * @param statusUpdateFrequency Status update frequency
     * @return True if configuration loaded successfully, false otherwise
     */
    bool loadSystemConfig(uint8_t& logLevel, bool& debugEnabled, uint8_t& statusUpdateFrequency);

    /**
     * Save system configuration
     *
     * @param logLevel Log level
     * @param debugEnabled Whether debug mode is enabled
     * @param statusUpdateFrequency Status update frequency
     * @return True if configuration saved successfully, false otherwise
     */
    bool saveSystemConfig(uint8_t logLevel, bool debugEnabled, uint8_t statusUpdateFrequency);

    /**
     * Load safety configuration
     *
     * @param positionErrorThreshold Position error threshold
     * @param velocityErrorThreshold Velocity error threshold
     * @param maxTemperature Maximum temperature
     * @return True if configuration loaded successfully, false otherwise
     */
    bool loadSafetyConfig(uint32_t& positionErrorThreshold, float& velocityErrorThreshold,
                          float& maxTemperature);

    /**
     * Save safety configuration
     *
     * @param positionErrorThreshold Position error threshold
     * @param velocityErrorThreshold Velocity error threshold
     * @param maxTemperature Maximum temperature
     * @return True if configuration saved successfully, false otherwise
     */
    bool saveSafetyConfig(uint32_t positionErrorThreshold, float velocityErrorThreshold,
                          float maxTemperature);

    /**
     * Load user data
     *
     * @param data Pointer to data buffer
     * @param size Size of data buffer
     * @return Number of bytes read, or -1 if error
     */
    int loadUserData(void* data, size_t size);

    /**
     * Save user data
     *
     * @param data Pointer to data buffer
     * @param size Size of data to save
     * @return Number of bytes written, or -1 if error
     */
    int saveUserData(const void* data, size_t size);

   private:
    // EEPROM state
    bool m_initialized;
    bool m_configValid;
    uint8_t m_configVersion;

    /**
     * Calculate address for motor parameters
     *
     * @param motorIndex Motor index
     * @param offset Offset within motor configuration block
     * @return EEPROM address
     */
    uint16_t getMotorAddress(uint8_t motorIndex, uint16_t offset) const;

    /**
     * Write value to EEPROM
     *
     * @param address EEPROM address
     * @param value Value to write
     */
    template <typename T>
    void writeValue(uint16_t address, const T& value);

    /**
     * Read value from EEPROM
     *
     * @param address EEPROM address
     * @param value Reference to store read value
     */
    template <typename T>
    void readValue(uint16_t address, T& value);

    /**
     * Validate motor index
     *
     * @param motorIndex Motor index
     * @return True if index is valid, false otherwise
     */
    bool isValidMotorIndex(uint8_t motorIndex) const;
};

#endif  // EEPROM_MANAGER_H