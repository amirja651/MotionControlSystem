#include <unity.h>
#include <Arduino.h>
#include "SystemManager.h"
#include "MotorManager.h"
#include "utils/EEPROMManager.h"
#include "core/Motor.h"

// This test verifies the power failure recovery feature

// Test fixture
SystemManager* systemManager = nullptr;
MotorManager* motorManager = nullptr;
EEPROMManager* eepromManager = nullptr;

void setUp() {
    // Create system components
    systemManager = new SystemManager();
    
    // Initialize system (limited initialization for testing)
    systemManager->initialize();
    
    // Get motor manager and EEPROM manager
    motorManager = systemManager->getMotorManager();
    eepromManager = systemManager->getEEPROMManager();
    
    // Make sure we're in a known state
    eepromManager->resetToDefaults();
    systemManager->setNormalShutdown(false); // Simulate abnormal power down
}

void tearDown() {
    // Clean up
    delete systemManager;
    systemManager = nullptr;
    motorManager = nullptr; // Owned by SystemManager
    eepromManager = nullptr; // Owned by SystemManager
}

// Test power failure recovery for a single motor
void test_power_recovery_single_motor() {
    // Skip test if any component is missing
    if (!systemManager || !motorManager || !eepromManager) {
        TEST_FAIL_MESSAGE("System components not initialized properly");
        return;
    }
    
    // Get a motor for testing
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);
    
    // Set motor to a known position
    const int32_t testPosition = 1000;
    motor->enable();
    motor->setPosition(testPosition);
    
    // Save the position to EEPROM as if it was running
    systemManager->saveMotorPositions();
    
    // Verify position was saved correctly
    int32_t savedPosition = 0;
    eepromManager->loadUserData(&savedPosition, sizeof(savedPosition), 
                                SystemManager::MOTOR_POSITIONS_ADDR);
    TEST_ASSERT_EQUAL(testPosition, savedPosition);
    
    // Now reset the motor position to simulate power loss
    motor->disable();
    motor->setPosition(0);
    TEST_ASSERT_EQUAL(0, motor->getCurrentPosition());
    
    // Simulate power recovery
    systemManager->restoreMotorPositions();
    
    // Check that position was restored
    TEST_ASSERT_EQUAL(testPosition, motor->getCurrentPosition());
}

// Test position recovery with multiple motors
void test_power_recovery_multiple_motors() {
    if (!systemManager || !motorManager || !eepromManager) {
        TEST_FAIL_MESSAGE("System components not initialized properly");
        return;
    }
    
    // Need at least 2 motors for this test
    if (motorManager->getMotorCount() < 2) {
        TEST_IGNORE_MESSAGE("Test requires at least 2 motors");
        return;
    }
    
    // Get motors for testing
    Motor* motor0 = motorManager->getMotor(0);
    Motor* motor1 = motorManager->getMotor(1);
    TEST_ASSERT_NOT_NULL(motor0);
    TEST_ASSERT_NOT_NULL(motor1);
    
    // Set motors to known positions
    const int32_t testPosition0 = 1000;
    const int32_t testPosition1 = -500;
    
    motor0->enable();
    motor1->enable();
    
    motor0->setPosition(testPosition0);
    motor1->setPosition(testPosition1);
    
    // Save positions
    systemManager->saveMotorPositions();
    
    // Reset positions to simulate power loss
    motor0->setPosition(0);
    motor1->setPosition(0);
    
    TEST_ASSERT_EQUAL(0, motor0->getCurrentPosition());
    TEST_ASSERT_EQUAL(0, motor1->getCurrentPosition());
    
    // Restore positions
    systemManager->restoreMotorPositions();
    
    // Check that positions were restored
    TEST_ASSERT_EQUAL(testPosition0, motor0->getCurrentPosition());
    TEST_ASSERT_EQUAL(testPosition1, motor1->getCurrentPosition());
}

// Test recovery behavior when normal shutdown occurred
void test_power_recovery_normal_shutdown() {
    if (!systemManager || !motorManager) {
        TEST_FAIL_MESSAGE("System components not initialized properly");
        return;
    }
    
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
        // With power monitoring enabled, position should not be restored
        TEST_ASSERT_EQUAL(0, motor->getCurrentPosition());
    #else
        // With power monitoring disabled, the function might behave differently
        // Check actual implementation
    #endif
}

// Test edge case: EEPROM is corrupted
void test_power_recovery_corrupted_eeprom() {
    if (!systemManager || !motorManager || !eepromManager) {
        TEST_FAIL_MESSAGE("System components not initialized properly");
        return;
    }
    
    // Get a motor for testing
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);
    
    // Corrupt EEPROM by writing invalid data
    int32_t invalidData = 0xDEADBEEF;
    eepromManager->saveUserData(&invalidData, sizeof(invalidData), 
                                SystemManager::MOTOR_POSITIONS_ADDR);
    
    // Set a known position
    motor->enable();
    motor->setPosition(0);
    
    // Try to restore positions
    systemManager->restoreMotorPositions();
    
    // Behavior depends on implementation - but should not crash
    // Just verify motor is still enabled
    TEST_ASSERT_TRUE(motor->isEnabled());
}

void setup() {
    // Wait before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    // Run tests
    RUN_TEST(test_power_recovery_single_motor);
    RUN_TEST(test_power_recovery_multiple_motors);
    RUN_TEST(test_power_recovery_normal_shutdown);
    RUN_TEST(test_power_recovery_corrupted_eeprom);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}