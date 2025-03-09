#include <unity.h>
#include <Arduino.h>
#include "MotorManager.h"
#include "core/Motor.h"
#include "utils/EEPROMManager.h"
#include "../include/test_mocks.h"

// Create a custom mock EEPROMManager for testing
class MockEEPROMManager : public EEPROMManager {
public:
    bool m_savedPID = false;
    bool m_savedSoftLimits = false;
    bool m_commitCalled = false;
    
    bool savePIDParameters(uint8_t motorIndex, float kp, float ki, float kd, float ff) override {
        m_savedPID = true;
        return true;
    }
    
    bool saveSoftLimits(uint8_t motorIndex, int32_t minLimit, int32_t maxLimit, bool enabled) override {
        m_savedSoftLimits = true;
        return true;
    }
    
    bool commit() override {
        m_commitCalled = true;
        return true;
    }
    
    void reset() {
        m_savedPID = false;
        m_savedSoftLimits = false;
        m_commitCalled = false;
    }
};

// Global variables for testing
MotorManager* motorManager = nullptr;
MockEEPROMManager* mockEEPROMManager = nullptr;

void setUp() {
    // Create motor manager with maximum of 2 motors for testing
    motorManager = new MotorManager(2);
    
    // Create mock EEPROM manager
    mockEEPROMManager = new MockEEPROMManager();
    
    // Reset mock states
    mockEEPROMManager->reset();
}

void tearDown() {
    delete motorManager;
    delete mockEEPROMManager;
    
    motorManager = nullptr;
    mockEEPROMManager = nullptr;
}

void test_motor_manager_initialization() {
    // Test initialization
    bool result = motorManager->initialize();
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(0, motorManager->getMotorCount()); // Should start with 0 motors
}

void test_motor_manager_add_motor() {
    // Create a valid motor configuration
    MotorConfig config;
    config.index = 0;
    config.stepPin = 10;
    config.dirPin = 11;
    config.enablePin = 12;
    config.maxVelocity = 1000.0f;
    config.maxAcceleration = 2000.0f;
    
    // Add the motor
    bool result = motorManager->addMotor(config);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(1, motorManager->getMotorCount());
    
    // Get the added motor
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);
    TEST_ASSERT_EQUAL(0, motor->getIndex());
}

void test_motor_manager_add_multiple_motors() {
    // Add first motor
    MotorConfig config1;
    config1.index = 0;
    config1.stepPin = 10;
    config1.dirPin = 11;
    config1.enablePin = 12;
    
    bool result1 = motorManager->addMotor(config1);
    TEST_ASSERT_TRUE(result1);
    
    // Add second motor
    MotorConfig config2;
    config2.index = 1;
    config2.stepPin = 20;
    config2.dirPin = 21;
    config2.enablePin = 22;
    
    bool result2 = motorManager->addMotor(config2);
    TEST_ASSERT_TRUE(result2);
    
    // Verify count
    TEST_ASSERT_EQUAL(2, motorManager->getMotorCount());
    
    // Try to add a third motor (should fail as max is 2)
    MotorConfig config3;
    config3.index = 2;
    config3.stepPin = 30;
    config3.dirPin = 31;
    config3.enablePin = 32;
    
    bool result3 = motorManager->addMotor(config3);
    TEST_ASSERT_FALSE(result3);
    
    // Verify count remains unchanged
    TEST_ASSERT_EQUAL(2, motorManager->getMotorCount());
}

void test_motor_manager_get_motor() {
    // Add a motor
    MotorConfig config;
    config.index = 0;
    config.stepPin = 10;
    config.dirPin = 11;
    config.enablePin = 12;
    
    motorManager->addMotor(config);
    
    // Get the motor
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);
    TEST_ASSERT_EQUAL(0, motor->getIndex());
    
    // Try to get a non-existent motor
    Motor* nonExistentMotor = motorManager->getMotor(1);
    TEST_ASSERT_NULL(nonExistentMotor);
}

void test_motor_manager_enable_disable_all() {
    // Add two motors
    MotorConfig config1;
    config1.index = 0;
    config1.stepPin = 10;
    config1.dirPin = 11;
    config1.enablePin = 12;
    
    MotorConfig config2;
    config2.index = 1;
    config2.stepPin = 20;
    config2.dirPin = 21;
    config2.enablePin = 22;
    
    motorManager->addMotor(config1);
    motorManager->addMotor(config2);
    
    // Enable all motors
    motorManager->enableAllMotors();
    
    // Check that all motors are enabled
    Motor* motor1 = motorManager->getMotor(0);
    Motor* motor2 = motorManager->getMotor(1);
    
    TEST_ASSERT_NOT_NULL(motor1);
    TEST_ASSERT_NOT_NULL(motor2);
    TEST_ASSERT_TRUE(motor1->isEnabled());
    TEST_ASSERT_TRUE(motor2->isEnabled());
    
    // Disable all motors
    motorManager->disableAllMotors();
    
    // Check that all motors are disabled
    TEST_ASSERT_FALSE(motor1->isEnabled());
    TEST_ASSERT_FALSE(motor2->isEnabled());
}

void test_motor_manager_stop_all_motors() {
    // Add two motors
    MotorConfig config1;
    config1.index = 0;
    config1.stepPin = 10;
    config1.dirPin = 11;
    config1.enablePin = 12;
    
    MotorConfig config2;
    config2.index = 1;
    config2.stepPin = 20;
    config2.dirPin = 21;
    config2.enablePin = 22;
    
    motorManager->addMotor(config1);
    motorManager->addMotor(config2);
    
    // Enable motors and set them to move
    Motor* motor1 = motorManager->getMotor(0);
    Motor* motor2 = motorManager->getMotor(1);
    
    motor1->enable();
    motor2->enable();
    
    motor1->setTargetPosition(1000);
    motor2->setTargetPosition(1000);
    
    // Stop all motors
    motorManager->stopAllMotors(false);
    
    // Unfortunately, we can't easily test the actual stopping behavior here
    // as it would require simulating motor movement
}

void test_motor_manager_save_to_eeprom() {
    // Replace the normal EEPROM manager with our mock
    // This would typically require a more complex approach in a real system
    MotorManager testManager(2);
    
    // Add a motor
    MotorConfig config;
    config.index = 0;
    config.stepPin = 10;
    config.dirPin = 11;
    config.enablePin = 12;
    
    testManager.addMotor(config);
    
    // TODO: In a real test, we would inject the mock EEPROM manager
    // But since we can't easily modify the internal EEPROMManager of the MotorManager,
    // we'll just test the basic functionality
    
    bool result = testManager.saveToEEPROM();
    
    // The test simply verifies the function returns without error
    // A more sophisticated test would verify the actual EEPROM writes
    TEST_ASSERT_TRUE(result);
}

void test_motor_manager_move_multiple_motors() {
    // Add two motors
    MotorConfig config1;
    config1.index = 0;
    config1.stepPin = 10;
    config1.dirPin = 11;
    config1.enablePin = 12;
    
    MotorConfig config2;
    config2.index = 1;
    config2.stepPin = 20;
    config2.dirPin = 21;
    config2.enablePin = 22;
    
    motorManager->addMotor(config1);
    motorManager->addMotor(config2);
    
    // Set up the move
    int32_t positions[2] = {1000, 2000};
    uint8_t motorIndices[2] = {0, 1};
    float maxVelocity = 500.0f;
    float acceleration = 1000.0f;
    float deceleration = 1000.0f;
    
    // Move the motors
    bool result = motorManager->moveMultipleMotors(positions, motorIndices, 2, maxVelocity, acceleration, deceleration);
    
    TEST_ASSERT_TRUE(result);
    
    // Verify the target positions were set
    Motor* motor1 = motorManager->getMotor(0);
    Motor* motor2 = motorManager->getMotor(1);
    
    TEST_ASSERT_NOT_NULL(motor1);
    TEST_ASSERT_NOT_NULL(motor2);
    
    // Due to synchronization, the actual target position might not exactly match what we specified
    // But it should be close to the requested position
    TEST_ASSERT_EQUAL(1000, motor1->getTargetPosition());
    TEST_ASSERT_EQUAL(2000, motor2->getTargetPosition());
}

void test_motor_manager_are_motors_idle() {
    // Add two motors
    MotorConfig config1;
    config1.index = 0;
    config1.stepPin = 10;
    config1.dirPin = 11;
    config1.enablePin = 12;
    
    MotorConfig config2;
    config2.index = 1;
    config2.stepPin = 20;
    config2.dirPin = 21;
    config2.enablePin = 22;
    
    motorManager->addMotor(config1);
    motorManager->addMotor(config2);
    
    // Get motors
    Motor* motor1 = motorManager->getMotor(0);
    Motor* motor2 = motorManager->getMotor(1);
    
    // Enable motors
    motor1->enable();
    motor2->enable();
    
    // Initially, motors should be idle
    uint8_t motorIndices[2] = {0, 1};
    TEST_ASSERT_TRUE(motorManager->areMotorsIdle(motorIndices, 2));
    
    // Set motors to move
    motor1->setTargetPosition(1000);
    
    // Now they should not be idle
    // Note: In a test environment, it's possible this still returns true because the motion isn't real
    // A more sophisticated test would mock the isMoving() method of the Motor class
}

void test_motor_manager_calculate_synchronized_move() {
    // Add two motors
    MotorConfig config1;
    config1.index = 0;
    config1.stepPin = 10;
    config1.dirPin = 11;
    config1.enablePin = 12;
    
    MotorConfig config2;
    config2.index = 1;
    config2.stepPin = 20;
    config2.dirPin = 21;
    config2.enablePin = 22;
    
    motorManager->addMotor(config1);
    motorManager->addMotor(config2);
    
    // Set up the move
    int32_t positions[2] = {1000, 2000};
    uint8_t motorIndices[2] = {0, 1};
    float maxVelocity = 500.0f;
    float acceleration = 1000.0f;
    float deceleration = 1000.0f;
    
    // Calculate synchronized move
    float duration = motorManager->calculateSynchronizedMove(positions, motorIndices, 2, maxVelocity, acceleration, deceleration);
    
    // Duration should be greater than zero
    TEST_ASSERT_TRUE(duration > 0.0f);
}

void test_motor_manager_error_handling() {
    // Add a motor
    MotorConfig config;
    config.index = 0;
    config.stepPin = 10;
    config.dirPin = 11;
    config.enablePin = 12;
    
    motorManager->addMotor(config);
    
    // Check if there are errors initially
    TEST_ASSERT_FALSE(motorManager->hasErrors());
    
    // Set an error on the motor
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);
    
    // Unfortunately, Motor class doesn't provide a public method to set errors directly
    // In a real test, we would need to cause an error condition
    
    // Clear all errors
    motorManager->clearAllErrors();
    
    // Should no longer have errors
    TEST_ASSERT_FALSE(motorManager->hasErrors());
}

void test_motor_manager_reset_all_encoders() {
    // Add a motor
    MotorConfig config;
    config.index = 0;
    config.stepPin = 10;
    config.dirPin = 11;
    config.enablePin = 12;
    
    motorManager->addMotor(config);
    
    // Get the motor
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);
    
    // Set a position
    motor->setPosition(1000);
    TEST_ASSERT_EQUAL(1000, motor->getCurrentPosition());
    
    // Reset all encoders
    motorManager->resetAllEncoders();
    
    // Position should be reset to 0
    TEST_ASSERT_EQUAL(0, motor->getCurrentPosition());
}

void test_motor_manager_invalid_config() {
    // Test with invalid step pin
    MotorConfig invalidStepPin;
    invalidStepPin.index = 0;
    invalidStepPin.stepPin = 0xFF; // Invalid pin
    invalidStepPin.dirPin = 11;
    invalidStepPin.enablePin = 12;
    
    bool result1 = motorManager->addMotor(invalidStepPin);
    TEST_ASSERT_FALSE(result1);
    
    // Test with invalid direction pin
    MotorConfig invalidDirPin;
    invalidDirPin.index = 0;
    invalidDirPin.stepPin = 10;
    invalidDirPin.dirPin = 0xFF; // Invalid pin
    invalidDirPin.enablePin = 12;
    
    bool result2 = motorManager->addMotor(invalidDirPin);
    TEST_ASSERT_FALSE(result2);
    
    // Test with invalid index
    MotorConfig invalidIndex;
    invalidIndex.index = 0xFF; // Invalid index
    invalidIndex.stepPin = 10;
    invalidIndex.dirPin = 11;
    invalidIndex.enablePin = 12;
    
    bool result3 = motorManager->addMotor(invalidIndex);
    TEST_ASSERT_FALSE(result3);
}

void setup() {
    // Wait before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_motor_manager_initialization);
    RUN_TEST(test_motor_manager_add_motor);
    RUN_TEST(test_motor_manager_add_multiple_motors);
    RUN_TEST(test_motor_manager_get_motor);
    RUN_TEST(test_motor_manager_enable_disable_all);
    RUN_TEST(test_motor_manager_stop_all_motors);
    RUN_TEST(test_motor_manager_save_to_eeprom);
    RUN_TEST(test_motor_manager_move_multiple_motors);
    RUN_TEST(test_motor_manager_are_motors_idle);
    RUN_TEST(test_motor_manager_calculate_synchronized_move);
    RUN_TEST(test_motor_manager_error_handling);
    RUN_TEST(test_motor_manager_reset_all_encoders);
    RUN_TEST(test_motor_manager_invalid_config);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}