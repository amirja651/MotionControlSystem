#include <unity.h>
#include <Arduino.h>
#include "utils/EEPROMManager.h"
#include "../include/test_mocks.h"

// Global mock EEPROM for testing
MockEEPROM mockEEPROM;

// Mocking EEPROM.h functions that EEPROMManager will call
namespace EEPROM {
    uint8_t read(int address) {
        return mockEEPROM.read(address);
    }
    
    void write(int address, uint8_t value) {
        mockEEPROM.write(address, value);
    }
    
    template <typename T>
    void get(int address, T& data) {
        mockEEPROM.get(address, &data, sizeof(T));
    }
    
    template <typename T>
    void put(int address, const T& data) {
        mockEEPROM.put(address, &data, sizeof(T));
    }
    
    bool begin(size_t size) {
        return mockEEPROM.begin(size);
    }
    
    bool commit() {
        return mockEEPROM.commit();
    }
    
    size_t length() {
        return mockEEPROM.length();
    }
}

void setUp(void) {
    // Reset the mock EEPROM before each test
    mockEEPROM.begin(CONFIG_EEPROM_SIZE);
}

void tearDown(void) {
    // Cleanup after each test
}

void test_eeprom_manager_initialization() {
    EEPROMManager manager;
    
    // Test initialization
    bool result = manager.initialize();
    TEST_ASSERT_TRUE(result);
    
    // Config should not be valid until we write the magic marker
    TEST_ASSERT_FALSE(manager.isConfigValid());
}

void test_eeprom_manager_reset_to_defaults() {
    EEPROMManager manager;
    manager.initialize();
    
    // Reset to defaults
    bool result = manager.resetToDefaults();
    TEST_ASSERT_TRUE(result);
    
    // Config should now be valid
    TEST_ASSERT_TRUE(manager.isConfigValid());
    
    // Verify magic marker was written
    uint16_t magicMarker;
    EEPROM::get(ADDR_MAGIC_MARKER, magicMarker);
    TEST_ASSERT_EQUAL(CONFIG_EEPROM_MAGIC_MARKER, magicMarker);
    
    // Verify version was written
    uint8_t version;
    EEPROM::get(ADDR_VERSION, version);
    TEST_ASSERT_EQUAL(CONFIG_EEPROM_VERSION, version);
}

void test_eeprom_manager_pid_parameters() {
    EEPROMManager manager;
    manager.initialize();
    manager.resetToDefaults();
    
    // Test saving PID parameters
    const uint8_t motorIndex = 0;
    const float kp = 1.5f;
    const float ki = 0.2f;
    const float kd = 0.3f;
    const float ff = 0.1f;
    
    bool saveResult = manager.savePIDParameters(motorIndex, kp, ki, kd, ff);
    TEST_ASSERT_TRUE(saveResult);
    
    // Commit changes
    bool commitResult = manager.commit();
    TEST_ASSERT_TRUE(commitResult);
    
    // Test loading PID parameters
    float loadedKp = 0.0f;
    float loadedKi = 0.0f;
    float loadedKd = 0.0f;
    float loadedFf = 0.0f;
    
    bool loadResult = manager.loadPIDParameters(motorIndex, loadedKp, loadedKi, loadedKd, loadedFf);
    TEST_ASSERT_TRUE(loadResult);
    
    // Check that loaded values match saved values
    TEST_ASSERT_FLOAT_WITHIN(0.001f, kp, loadedKp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, ki, loadedKi);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, kd, loadedKd);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, ff, loadedFf);
}

void test_eeprom_manager_profile_parameters() {
    EEPROMManager manager;
    manager.initialize();
    manager.resetToDefaults();
    
    // Test saving profile parameters
    const uint8_t motorIndex = 0;
    const float maxVelocity = 5000.0f;
    const float acceleration = 10000.0f;
    const float deceleration = 15000.0f;
    const float jerk = 20000.0f;
    
    bool saveResult = manager.saveProfileParameters(motorIndex, maxVelocity, acceleration, deceleration, jerk);
    TEST_ASSERT_TRUE(saveResult);
    
    // Commit changes
    bool commitResult = manager.commit();
    TEST_ASSERT_TRUE(commitResult);
    
    // Test loading profile parameters
    float loadedMaxVelocity = 0.0f;
    float loadedAcceleration = 0.0f;
    float loadedDeceleration = 0.0f;
    float loadedJerk = 0.0f;
    
    bool loadResult = manager.loadProfileParameters(motorIndex, loadedMaxVelocity, loadedAcceleration, 
                                                  loadedDeceleration, loadedJerk);
    TEST_ASSERT_TRUE(loadResult);
    
    // Check that loaded values match saved values
    TEST_ASSERT_FLOAT_WITHIN(0.001f, maxVelocity, loadedMaxVelocity);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, acceleration, loadedAcceleration);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, deceleration, loadedDeceleration);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, jerk, loadedJerk);
}

void test_eeprom_manager_soft_limits() {
    EEPROMManager manager;
    manager.initialize();
    manager.resetToDefaults();
    
    // Test saving soft limits
    const uint8_t motorIndex = 0;
    const int32_t minLimit = -10000;
    const int32_t maxLimit = 10000;
    const bool enabled = true;
    
    bool saveResult = manager.saveSoftLimits(motorIndex, minLimit, maxLimit, enabled);
    TEST_ASSERT_TRUE(saveResult);
    
    // Commit changes
    bool commitResult = manager.commit();
    TEST_ASSERT_TRUE(commitResult);
    
    // Test loading soft limits
    int32_t loadedMinLimit = 0;
    int32_t loadedMaxLimit = 0;
    bool loadedEnabled = false;
    
    bool loadResult = manager.loadSoftLimits(motorIndex, loadedMinLimit, loadedMaxLimit, loadedEnabled);
    TEST_ASSERT_TRUE(loadResult);
    
    // Check that loaded values match saved values
    TEST_ASSERT_EQUAL(minLimit, loadedMinLimit);
    TEST_ASSERT_EQUAL(maxLimit, loadedMaxLimit);
    TEST_ASSERT_EQUAL(enabled, loadedEnabled);
}

void test_eeprom_manager_system_config() {
    EEPROMManager manager;
    manager.initialize();
    manager.resetToDefaults();
    
    // Test saving system configuration
    const uint8_t logLevel = 4; // DEBUG
    const bool debugEnabled = true;
    const uint8_t statusUpdateFrequency = 5;
    
    bool saveResult = manager.saveSystemConfig(logLevel, debugEnabled, statusUpdateFrequency);
    TEST_ASSERT_TRUE(saveResult);
    
    // Commit changes
    bool commitResult = manager.commit();
    TEST_ASSERT_TRUE(commitResult);
    
    // Test loading system configuration
    uint8_t loadedLogLevel = 0;
    bool loadedDebugEnabled = false;
    uint8_t loadedStatusUpdateFrequency = 0;
    
    bool loadResult = manager.loadSystemConfig(loadedLogLevel, loadedDebugEnabled, loadedStatusUpdateFrequency);
    TEST_ASSERT_TRUE(loadResult);
    
    // Check that loaded values match saved values
    TEST_ASSERT_EQUAL(logLevel, loadedLogLevel);
    TEST_ASSERT_EQUAL(debugEnabled, loadedDebugEnabled);
    TEST_ASSERT_EQUAL(statusUpdateFrequency, loadedStatusUpdateFrequency);
}

void test_eeprom_manager_safety_config() {
    EEPROMManager manager;
    manager.initialize();
    manager.resetToDefaults();
    
    // Test saving safety configuration
    const uint32_t positionErrorThreshold = 200;
    const float velocityErrorThreshold = 500.0f;
    const float maxTemperature = 75.0f;
    
    bool saveResult = manager.saveSafetyConfig(positionErrorThreshold, velocityErrorThreshold, maxTemperature);
    TEST_ASSERT_TRUE(saveResult);
    
    // Commit changes
    bool commitResult = manager.commit();
    TEST_ASSERT_TRUE(commitResult);
    
    // Test loading safety configuration
    uint32_t loadedPositionErrorThreshold = 0;
    float loadedVelocityErrorThreshold = 0.0f;
    float loadedMaxTemperature = 0.0f;
    
    bool loadResult = manager.loadSafetyConfig(loadedPositionErrorThreshold, loadedVelocityErrorThreshold, loadedMaxTemperature);
    TEST_ASSERT_TRUE(loadResult);
    
    // Check that loaded values match saved values
    TEST_ASSERT_EQUAL(positionErrorThreshold, loadedPositionErrorThreshold);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, velocityErrorThreshold, loadedVelocityErrorThreshold);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, maxTemperature, loadedMaxTemperature);
}

void test_eeprom_manager_user_data() {
    EEPROMManager manager;
    manager.initialize();
    manager.resetToDefaults();
    
    // Test saving user data
    const uint16_t address = ADDR_USER_DATA_START;
    const int32_t testData = 0x12345678;
    
    int savedBytes = manager.saveUserData(&testData, sizeof(testData), address);
    TEST_ASSERT_EQUAL(sizeof(testData), savedBytes);
    
    // Commit changes
    bool commitResult = manager.commit();
    TEST_ASSERT_TRUE(commitResult);
    
    // Test loading user data
    int32_t loadedData = 0;
    
    int loadedBytes = manager.loadUserData(&loadedData, sizeof(loadedData), address);
    TEST_ASSERT_EQUAL(sizeof(loadedData), loadedBytes);
    
    // Check that loaded value matches saved value
    TEST_ASSERT_EQUAL(testData, loadedData);
}

void test_eeprom_manager_invalid_motor_index() {
    EEPROMManager manager;
    manager.initialize();
    manager.resetToDefaults();
    
    // Test with invalid motor index (out of range)
    const uint8_t invalidMotorIndex = CONFIG_MAX_MOTORS + 1;
    
    // Attempt to save PID parameters with invalid index
    bool saveResult = manager.savePIDParameters(invalidMotorIndex, 1.0f, 0.1f, 0.0f, 0.0f);
    TEST_ASSERT_FALSE(saveResult);
    
    // Attempt to load PID parameters with invalid index
    float kp, ki, kd, ff;
    bool loadResult = manager.loadPIDParameters(invalidMotorIndex, kp, ki, kd, ff);
    TEST_ASSERT_FALSE(loadResult);
}

void setup() {
    // Wait before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_eeprom_manager_initialization);
    RUN_TEST(test_eeprom_manager_reset_to_defaults);
    RUN_TEST(test_eeprom_manager_pid_parameters);
    RUN_TEST(test_eeprom_manager_profile_parameters);
    RUN_TEST(test_eeprom_manager_soft_limits);
    RUN_TEST(test_eeprom_manager_system_config);
    RUN_TEST(test_eeprom_manager_safety_config);
    RUN_TEST(test_eeprom_manager_user_data);
    RUN_TEST(test_eeprom_manager_invalid_motor_index);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}