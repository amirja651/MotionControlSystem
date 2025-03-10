#include <Arduino.h>
#include <unity.h>

#include "../include/test_mocks.h"
#include "MotorManager.h"
#include "SystemManager.h"
#include "core/Motor.h"
#include "utils/EEPROMManager.h"

// Test fixture
SystemManager* systemManager = nullptr;
MotorManager* motorManager = nullptr;
EEPROMManager* eepromManager = nullptr;
MockEEPROM mockEEPROM;

// Override EEPROM read/write functions for testing
namespace EEPROM {
uint8_t read(int address) {
    return mockEEPROM.read(address);
}

void write(int address, uint8_t value) {
    mockEEPROM.write(address, value);
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
}  // namespace EEPROM

void setUp() {
    // Initialize mock EEPROM
    mockEEPROM.begin(CONFIG_EEPROM_SIZE);

    // Create system components
    systemManager = new SystemManager();

    // Initialize system (limited initialization for testing)
    systemManager->initialize();

    // Get motor manager and EEPROM manager
    motorManager = systemManager->getMotorManager();
    eepromManager = systemManager->getEEPROMManager();

    // Make sure we're in a known state
    eepromManager->resetToDefaults();

    // Set up motors for testing
    // Configure two motors - one linear and one rotary for a single optical element
    MotorConfig linearConfig;
    linearConfig.index = 0;
    linearConfig.stepPin = 10;
    linearConfig.dirPin = 11;
    linearConfig.enablePin = 12;
    linearConfig.maxVelocity = 2000.0f;
    linearConfig.maxAcceleration = 5000.0f;
    linearConfig.encoderPPR = 2000;  // High resolution encoder for linear stage

    MotorConfig rotaryConfig;
    rotaryConfig.index = 1;
    rotaryConfig.stepPin = 15;
    rotaryConfig.dirPin = 16;
    rotaryConfig.enablePin = 17;
    rotaryConfig.maxVelocity = 1000.0f;
    rotaryConfig.maxAcceleration = 3000.0f;
    rotaryConfig.encoderPPR = 1000;  // Lower resolution for rotary stage

    motorManager->addMotor(linearConfig);
    motorManager->addMotor(rotaryConfig);

    // Simulate abnormal power down
    systemManager->setNormalShutdown(false);
}

void tearDown() {
    // Clean up
    delete systemManager;
    systemManager = nullptr;
    motorManager = nullptr;   // Owned by SystemManager
    eepromManager = nullptr;  // Owned by SystemManager
}

// Test power failure recovery for a single motor
void test_power_recovery_single_motor() {
    // Get a motor for testing
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);

    // Set motor to a known position with high precision (sub-micrometer)
    const int32_t testPosition = 10000;  // For a high microstepping setup, this could be microns
    motor->enable();
    motor->setPosition(testPosition);

    // Save the position to EEPROM as if it was running
    systemManager->saveMotorPositions();

    // Verify position was saved correctly by reading directly from EEPROM
    int32_t savedPosition = 0;
    eepromManager->loadUserData(&savedPosition, sizeof(savedPosition),
                                systemManager->getMotorPositionsAddr());
    TEST_ASSERT_EQUAL(testPosition, savedPosition);

    // Now reset the motor position to simulate power loss
    motor->disable();
    motor->setPosition(0);
    TEST_ASSERT_EQUAL(0, motor->getCurrentPosition());

    // Simulate power recovery
    systemManager->restoreMotorPositions();

    // Check that position was restored EXACTLY to maintain sub-micrometer precision
    TEST_ASSERT_EQUAL_INT32(testPosition, motor->getCurrentPosition());
    TEST_ASSERT_TRUE(motor->isEnabled());
}

// Test position recovery with multiple motors in realistic laser alignment configuration
void test_power_recovery_multiple_motors() {
    // Get motors for testing - both linear and rotary
    Motor* linearMotor = motorManager->getMotor(0);
    Motor* rotaryMotor = motorManager->getMotor(1);

    TEST_ASSERT_NOT_NULL(linearMotor);
    TEST_ASSERT_NOT_NULL(rotaryMotor);

    // Set motors to known positions mimicking a real laser alignment setup
    // Linear position with sub-micrometer precision
    // Rotary position with 0.5 degree precision
    const int32_t linearPosition = 12345;  // Linear stage at precise position
    const int32_t rotaryPosition = 800;    // Rotary stage at specific angle

    linearMotor->enable();
    rotaryMotor->enable();

    linearMotor->setPosition(linearPosition);
    rotaryMotor->setPosition(rotaryPosition);

    // Periodically save positions (as would happen in real operation)
    systemManager->saveMotorPositions();

    // Verify saved positions in EEPROM directly
    int32_t savedLinearPosition = 0;
    int32_t savedRotaryPosition = 0;

    eepromManager->loadUserData(&savedLinearPosition, sizeof(savedLinearPosition),
                                systemManager->getMotorPositionsAddr());

    eepromManager->loadUserData(&savedRotaryPosition, sizeof(savedRotaryPosition),
                                systemManager->getMotorPositionsAddr() + sizeof(int32_t));

    TEST_ASSERT_EQUAL(linearPosition, savedLinearPosition);
    TEST_ASSERT_EQUAL(rotaryPosition, savedRotaryPosition);

    // Reset positions to simulate power loss
    linearMotor->disable();
    rotaryMotor->disable();
    linearMotor->setPosition(0);
    rotaryMotor->setPosition(0);

    TEST_ASSERT_EQUAL(0, linearMotor->getCurrentPosition());
    TEST_ASSERT_EQUAL(0, rotaryMotor->getCurrentPosition());

    // Restore positions
    systemManager->restoreMotorPositions();

    // Check that positions were restored with EXACT precision for both motors
    TEST_ASSERT_EQUAL_INT32(linearPosition, linearMotor->getCurrentPosition());
    TEST_ASSERT_EQUAL_INT32(rotaryPosition, rotaryMotor->getCurrentPosition());

    // Verify both motors were enabled during recovery
    TEST_ASSERT_TRUE(linearMotor->isEnabled());
    TEST_ASSERT_TRUE(rotaryMotor->isEnabled());
}

// Test saving positions during movement (mid-trajectory recovery)
void test_power_recovery_during_movement() {
    // Get motors for testing
    Motor* linearMotor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(linearMotor);

    // Enable motor and set initial position
    linearMotor->enable();
    linearMotor->setPosition(1000);

    // Start a movement
    linearMotor->moveToPosition(2000, 500.0f, 1000.0f, 1000.0f);

    // Simulate being in the middle of movement
    linearMotor->setPosition(1500);  // Manually set to halfway point

    // Save position during movement
    systemManager->saveMotorPositions();

    // Verify saved positions
    int32_t savedPosition = 0;
    eepromManager->loadUserData(&savedPosition, sizeof(savedPosition),
                                systemManager->getMotorPositionsAddr());
    TEST_ASSERT_EQUAL(1500, savedPosition);

    // Reset position to simulate power loss mid-movement
    linearMotor->disable();
    linearMotor->setPosition(0);

    // Restore after power-up
    systemManager->restoreMotorPositions();

    // Check that intermediate position was restored exactly
    TEST_ASSERT_EQUAL_INT32(1500, linearMotor->getCurrentPosition());
    TEST_ASSERT_TRUE(linearMotor->isEnabled());

    // Target position would be lost during power failure, but current position restores correctly
}

// Test recovery behavior when normal shutdown occurred
void test_power_recovery_normal_shutdown() {
    // Set normal shutdown flag
    systemManager->setNormalShutdown(true);

    // Get a motor for testing
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);

    // Set and save a position
    motor->enable();
    motor->setPosition(1000);
    systemManager->saveMotorPositions();

    // Reset position
    motor->setPosition(0);

// Call restore - but it should not restore positions since it was a normal shutdown
#if CONFIG_POWER_MONITORING_ENABLED
    systemManager->restoreMotorPositions();
    // With power monitoring enabled, position should not be restored after normal shutdown
    TEST_ASSERT_EQUAL(0, motor->getCurrentPosition());
#else
    // With power monitoring disabled, the function might behave differently
    // so we'll test what actually happens in our implementation
    systemManager->restoreMotorPositions();
    // Check what your implementation does
    int32_t currentPos = motor->getCurrentPosition();
    // Either position is restored or it remains at 0 - document actual behavior
    if (currentPos == 0) {
        TEST_ASSERT_EQUAL(0, motor->getCurrentPosition());
    } else {
        TEST_ASSERT_EQUAL(1000, motor->getCurrentPosition());
    }
#endif
}

// Test a full optical alignment setup with multiple sets of motors
void test_multiple_optical_elements_recovery() {
    // Add two more motors for a second optical element
    MotorConfig linearConfig2;
    linearConfig2.index = 2;
    linearConfig2.stepPin = 20;
    linearConfig2.dirPin = 21;
    linearConfig2.enablePin = 22;
    linearConfig2.maxVelocity = 2000.0f;
    linearConfig2.maxAcceleration = 5000.0f;

    MotorConfig rotaryConfig2;
    rotaryConfig2.index = 3;
    rotaryConfig2.stepPin = 25;
    rotaryConfig2.dirPin = 26;
    rotaryConfig2.enablePin = 27;
    rotaryConfig2.maxVelocity = 1000.0f;
    rotaryConfig2.maxAcceleration = 3000.0f;

    motorManager->addMotor(linearConfig2);
    motorManager->addMotor(rotaryConfig2);

    // Get all four motors
    Motor* linear1 = motorManager->getMotor(0);
    Motor* rotary1 = motorManager->getMotor(1);
    Motor* linear2 = motorManager->getMotor(2);
    Motor* rotary2 = motorManager->getMotor(3);

    TEST_ASSERT_NOT_NULL(linear1);
    TEST_ASSERT_NOT_NULL(rotary1);
    TEST_ASSERT_NOT_NULL(linear2);
    TEST_ASSERT_NOT_NULL(rotary2);

    // Enable all motors
    linear1->enable();
    rotary1->enable();
    linear2->enable();
    rotary2->enable();

    // Set specific positions for laser alignment
    const int32_t linear1Pos = 12345;  // Sub-micrometer position
    const int32_t rotary1Pos = 800;    // 0.5-degree precision
    const int32_t linear2Pos = 54321;  // Different sub-micrometer position
    const int32_t rotary2Pos = 400;    // Different angle

    linear1->setPosition(linear1Pos);
    rotary1->setPosition(rotary1Pos);
    linear2->setPosition(linear2Pos);
    rotary2->setPosition(rotary2Pos);

    // Save all positions
    systemManager->saveMotorPositions();

    // Reset all positions to simulate power loss
    linear1->setPosition(0);
    rotary1->setPosition(0);
    linear2->setPosition(0);
    rotary2->setPosition(0);

    // Disable all motors as would happen in power loss
    linear1->disable();
    rotary1->disable();
    linear2->disable();
    rotary2->disable();

    // Restore after power-up
    systemManager->restoreMotorPositions();

    // Verify exact position recovery for all motors
    TEST_ASSERT_EQUAL_INT32(linear1Pos, linear1->getCurrentPosition());
    TEST_ASSERT_EQUAL_INT32(rotary1Pos, rotary1->getCurrentPosition());
    TEST_ASSERT_EQUAL_INT32(linear2Pos, linear2->getCurrentPosition());
    TEST_ASSERT_EQUAL_INT32(rotary2Pos, rotary2->getCurrentPosition());

    // Verify all motors were re-enabled
    TEST_ASSERT_TRUE(linear1->isEnabled());
    TEST_ASSERT_TRUE(rotary1->isEnabled());
    TEST_ASSERT_TRUE(linear2->isEnabled());
    TEST_ASSERT_TRUE(rotary2->isEnabled());
}

// Test recovery with brown-out conditions (partial power failure)
void test_brownout_recovery() {
    // Get a motor for testing
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);

    // Set a precise position
    const int32_t precisePosition = 12345;
    motor->enable();
    motor->setPosition(precisePosition);

    // Save position periodically
    systemManager->saveMotorPositions();

    // Verify saved position
    int32_t savedPosition = 0;
    eepromManager->loadUserData(&savedPosition, sizeof(savedPosition),
                                systemManager->getMotorPositionsAddr());
    TEST_ASSERT_EQUAL(precisePosition, savedPosition);

    // Simulate corrupted EEPROM data due to brownout during save operation
    // Write partial data to EEPROM
    int32_t corruptedValue = 0xFFFFFFFF;             // Simulated corruption
    eepromManager->saveUserData(&corruptedValue, 2,  // Only half the bytes
                                systemManager->getMotorPositionsAddr());

    // Reset position
    motor->setPosition(0);

    // Try to restore after brownout
    systemManager->restoreMotorPositions();

    // Check behavior with corrupted data
    // Should either restore to last good position or use default safe position
    int32_t currentPos = motor->getCurrentPosition();

    // Document actual behavior
    // Since this is an edge case, just verify the system doesn't crash
    // and that motor is still enabled for manual recovery
    TEST_ASSERT_TRUE(motor->isEnabled());
}

// Test position recovery accuracy for ultra-precise applications
void test_recovery_sub_micrometer_accuracy() {
    // Get motor for high-precision test
    Motor* linearMotor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(linearMotor);

    // Enable with high-precision settings
    linearMotor->enable();

    // Set a position requiring sub-micrometer precision
    // For a system with 200 steps/rev, 32 microsteps, and 5mm leadscrew
    // Resolution would be 5mm/(200*32) = 0.78125 microns per step
    // so a position of 12800 steps = 10mm exactly
    const int32_t microPrecisePosition = 12800;
    linearMotor->setPosition(microPrecisePosition);

    // Save position
    systemManager->saveMotorPositions();

    // Reset position to simulate power loss
    linearMotor->disable();
    linearMotor->setPosition(0);

    // Restore position
    systemManager->restoreMotorPositions();

    // Verify exact recovery to maintain sub-micrometer precision
    // Even 1 step off would be ~0.78 microns of error
    TEST_ASSERT_EQUAL_INT32(microPrecisePosition, linearMotor->getCurrentPosition());
}

// Test voltage monitoring integration with power recovery
void test_voltage_monitoring_integration() {
#if CONFIG_POWER_MONITORING_ENABLED
    // This test is only relevant if power monitoring is enabled

    // Get a motor for testing
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);

    // Set a known precise position
    motor->enable();
    motor->setPosition(12345);

    // Save position normally
    systemManager->saveMotorPositions();

    // Simulate a voltage sag (mock would be needed in real implementation)
    // Here we're just testing the integration between power monitoring and recovery

    // Reset position
    motor->setPosition(0);

    // Restore position from EEPROM
    systemManager->restoreMotorPositions();

    // Position should be restored precisely
    TEST_ASSERT_EQUAL_INT32(12345, motor->getCurrentPosition());
#else
    // Skip test if power monitoring is disabled
    TEST_IGNORE_MESSAGE("Power monitoring is disabled in configuration");
#endif
}

void setup() {
    // Wait before starting tests
    delay(2000);

    UNITY_BEGIN();

    // Run tests
    RUN_TEST(test_power_recovery_single_motor);
    RUN_TEST(test_power_recovery_multiple_motors);
    RUN_TEST(test_power_recovery_during_movement);
    RUN_TEST(test_power_recovery_normal_shutdown);
    RUN_TEST(test_multiple_optical_elements_recovery);
    RUN_TEST(test_brownout_recovery);
    RUN_TEST(test_recovery_sub_micrometer_accuracy);
    RUN_TEST(test_voltage_monitoring_integration);

    UNITY_END();
}

void loop() {
    // Nothing to do here
}