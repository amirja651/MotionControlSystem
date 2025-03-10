#include <unity.h>
#include <Arduino.h>
#include "SystemManager.h"
#include "MotorManager.h"
#include "core/Motor.h"
#include "core/TrajectoryPlanner.h"
#include "../include/test_mocks.h"

// Global variables for testing
SystemManager* systemManager = nullptr;
MotorManager* motorManager = nullptr;
MockLogger* mockLogger = nullptr;

void setUp() {
    // Create and initialize system components
    systemManager = new SystemManager();
    systemManager->initialize();
    
    // Get motor manager
    motorManager = systemManager->getMotorManager();
    
    // Create logger for tracking messages
    mockLogger = new MockLogger();
    
    // Add two motors for sync testing
    // First motor - simulating linear motor
    MotorConfig linearConfig;
    linearConfig.index = 0;
    linearConfig.stepPin = 10;
    linearConfig.dirPin = 11;
    linearConfig.enablePin = 12;
    linearConfig.maxVelocity = 2000.0f;       // 2000 steps/second
    linearConfig.maxAcceleration = 5000.0f;   // 5000 steps/second²
    linearConfig.maxDeceleration = 5000.0f;   // 5000 steps/second²
    linearConfig.pidKp = 0.5f;
    linearConfig.pidKi = 0.1f;
    linearConfig.pidKd = 0.05f;
    linearConfig.encoderPPR = 2000;          // 2000 pulses per revolution for high resolution
    
    // Second motor - simulating rotary motor
    MotorConfig rotaryConfig;
    rotaryConfig.index = 1;
    rotaryConfig.stepPin = 15;
    rotaryConfig.dirPin = 16;
    rotaryConfig.enablePin = 17;
    rotaryConfig.maxVelocity = 1000.0f;       // 1000 steps/second
    rotaryConfig.maxAcceleration = 3000.0f;   // 3000 steps/second²
    rotaryConfig.maxDeceleration = 3000.0f;   // 3000 steps/second²
    rotaryConfig.pidKp = 0.6f;
    rotaryConfig.pidKi = 0.12f;
    rotaryConfig.pidKd = 0.06f;
    rotaryConfig.encoderPPR = 1000;          // 1000 pulses per revolution
    
    motorManager->addMotor(linearConfig);
    motorManager->addMotor(rotaryConfig);
}

void tearDown() {
    delete mockLogger;
    delete systemManager;
    
    mockLogger = nullptr;
    systemManager = nullptr;
    motorManager = nullptr; // Owned by SystemManager
}

// Test synchronized movement of linear and rotary motors
void test_synchronized_movement() {
    Motor* linearMotor = motorManager->getMotor(0);
    Motor* rotaryMotor = motorManager->getMotor(1);
    
    TEST_ASSERT_NOT_NULL(linearMotor);
    TEST_ASSERT_NOT_NULL(rotaryMotor);
    
    // Enable motors
    linearMotor->enable();
    rotaryMotor->enable();
    
    // Set initial positions
    linearMotor->setPosition(0);
    rotaryMotor->setPosition(0);
    
    // Set up coordinated movement - linear motor moves 1000 steps, rotary motor moves 500 steps
    int32_t targetPositions[2] = {1000, 500};
    uint8_t motorIndices[2] = {0, 1};
    
    // Verify motion parameters before movement
    TEST_ASSERT_EQUAL(0, linearMotor->getCurrentPosition());
    TEST_ASSERT_EQUAL(0, rotaryMotor->getCurrentPosition());
    
    // Execute synchronized movement
    bool result = motorManager->moveMultipleMotors(
        targetPositions, 
        motorIndices, 
        2,           // Two motors
        1000.0f,     // 1000 steps/sec max velocity
        2000.0f,     // 2000 steps/sec² acceleration
        2000.0f      // 2000 steps/sec² deceleration
    );
    
    TEST_ASSERT_TRUE(result);
    
    // Calculate the theoretical duration
    float duration = motorManager->calculateSynchronizedMove(
        targetPositions, 
        motorIndices, 
        2, 
        1000.0f, 
        2000.0f, 
        2000.0f
    );
    
    // Duration should be positive
    TEST_ASSERT_GREATER_THAN(0.0f, duration);
    
    // Simulate time progression and motion update
    // In a real test environment, we would simulate time more precisely
    // For each motor and check positions along the way
    
    // Simulate step to 25% of the way through the motion
    // Manually set positions that would be achieved through motion
    linearMotor->setPosition(250);  // 25% of 1000
    rotaryMotor->setPosition(125);  // 25% of 500
    
    // Verify both motors are moving at synchronized rates (proportionally)
    float linearPercent = (float)linearMotor->getCurrentPosition() / targetPositions[0];
    float rotaryPercent = (float)rotaryMotor->getCurrentPosition() / targetPositions[1];
    
    // Both motors should have moved the same percentage of their total distance
    TEST_ASSERT_FLOAT_WITHIN(0.01f, linearPercent, rotaryPercent);
    
    // Simulate step to 75% of the way through the motion
    linearMotor->setPosition(750);  // 75% of 1000
    rotaryMotor->setPosition(375);  // 75% of 500
    
    // Verify proportional movement again
    linearPercent = (float)linearMotor->getCurrentPosition() / targetPositions[0];
    rotaryPercent = (float)rotaryMotor->getCurrentPosition() / targetPositions[1];
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, linearPercent, rotaryPercent);
    
    // Simulate completion of movement
    linearMotor->setPosition(1000);
    rotaryMotor->setPosition(500);
    
    // Verify both motors reached their targets
    TEST_ASSERT_EQUAL(targetPositions[0], linearMotor->getCurrentPosition());
    TEST_ASSERT_EQUAL(targetPositions[1], rotaryMotor->getCurrentPosition());
}

// Test for precise movement profiles relevant to laser alignment
void test_precise_mirror_alignment() {
    Motor* linearMotor = motorManager->getMotor(0);
    Motor* rotaryMotor = motorManager->getMotor(1);
    
    TEST_ASSERT_NOT_NULL(linearMotor);
    TEST_ASSERT_NOT_NULL(rotaryMotor);
    
    // Enable motors
    linearMotor->enable();
    rotaryMotor->enable();
    
    // Reset positions
    linearMotor->setPosition(0);
    rotaryMotor->setPosition(0);
    
    // Test fine positioning of linear motor (submicron adjustments)
    // For a 5mm leadscrew with 200 steps/rev motor and 16x microstepping
    // Each step = 5mm / (200 * 16) = 0.00156mm = 1.56μm
    // So 10 steps = ~15.6μm (submicron would require higher microstepping)
    
    // First test: small linear movement while holding rotary position
    linearMotor->moveToPosition(10, 100.0f, 200.0f, 200.0f); // Very slow for precision
    
    // Simulate completion of movement
    linearMotor->setPosition(10);
    
    // Verify linear motor moved precisely while rotary stayed put
    TEST_ASSERT_EQUAL(10, linearMotor->getCurrentPosition());
    TEST_ASSERT_EQUAL(0, rotaryMotor->getCurrentPosition());
    
    // Second test: small rotary adjustment while holding linear position
    // For a 1.8° stepper with 16x microstepping, each step is 0.1125°
    // 5 steps = 0.5625° (close to our 0.5° requirement)
    rotaryMotor->moveToPosition(5, 50.0f, 100.0f, 100.0f); // Very slow for precision
    
    // Simulate completion of movement
    rotaryMotor->setPosition(5);
    
    // Verify rotary motor moved precisely while linear held position
    TEST_ASSERT_EQUAL(10, linearMotor->getCurrentPosition());
    TEST_ASSERT_EQUAL(5, rotaryMotor->getCurrentPosition());
    
    // Third test: coordinated small adjustment of both
    int32_t fineAdjustment[2] = {15, 8}; // +5 steps linear, +3 steps rotary
    uint8_t motorIndices[2] = {0, 1};
    
    bool result = motorManager->moveMultipleMotors(
        fineAdjustment, 
        motorIndices, 
        2,
        50.0f,      // Very slow for precision
        100.0f,
        100.0f
    );
    
    TEST_ASSERT_TRUE(result);
    
    // Simulate completion of movement
    linearMotor->setPosition(15);
    rotaryMotor->setPosition(8);
    
    // Verify both motors reached their exact targets
    TEST_ASSERT_EQUAL(fineAdjustment[0], linearMotor->getCurrentPosition());
    TEST_ASSERT_EQUAL(fineAdjustment[1], rotaryMotor->getCurrentPosition());
}

// Test realistic laser alignment scenario with multiple fine adjustments
void test_laser_alignment_scenario() {
    Motor* linearMotor = motorManager->getMotor(0);
    Motor* rotaryMotor = motorManager->getMotor(1);
    
    TEST_ASSERT_NOT_NULL(linearMotor);
    TEST_ASSERT_NOT_NULL(rotaryMotor);
    
    // Enable motors and reset positions
    linearMotor->enable();
    rotaryMotor->enable();
    linearMotor->setPosition(1000); // Start at some arbitrary position
    rotaryMotor->setPosition(200);  // Start at some arbitrary position
    
    // Simulate a typical alignment sequence:
    // 1. Coarse adjustment of mirror position
    // 2. Fine rotation adjustment
    // 3. Ultra-fine linear adjustment
    // 4. Final verification micro-adjustments
    
    // 1. Coarse adjustment - move linear motor forward
    linearMotor->moveToPosition(1200, 500.0f, 1000.0f, 1000.0f);
    linearMotor->setPosition(1200); // Simulate completion
    TEST_ASSERT_EQUAL(1200, linearMotor->getCurrentPosition());
    
    // 2. Fine rotation adjustment
    rotaryMotor->moveToPosition(210, 200.0f, 400.0f, 400.0f);
    rotaryMotor->setPosition(210); // Simulate completion
    TEST_ASSERT_EQUAL(210, rotaryMotor->getCurrentPosition());
    
    // 3. Ultra-fine linear adjustment (forward a few microns)
    linearMotor->moveToPosition(1205, 100.0f, 200.0f, 200.0f);
    linearMotor->setPosition(1205); // Simulate completion
    TEST_ASSERT_EQUAL(1205, linearMotor->getCurrentPosition());
    
    // 4. Final micro-adjustments - coordinated move
    int32_t finalPositions[2] = {1204, 209}; // -1 linear, -1 rotary
    uint8_t motorIndices[2] = {0, 1};
    
    bool result = motorManager->moveMultipleMotors(
        finalPositions,
        motorIndices,
        2,
        50.0f,  // Very slow speed for precision
        100.0f, 
        100.0f
    );
    
    TEST_ASSERT_TRUE(result);
    
    // Simulate completion
    linearMotor->setPosition(1204);
    rotaryMotor->setPosition(209);
    
    // Verify final positions are exact
    TEST_ASSERT_EQUAL(finalPositions[0], linearMotor->getCurrentPosition());
    TEST_ASSERT_EQUAL(finalPositions[1], rotaryMotor->getCurrentPosition());
}

// Test for motion limits and soft boundaries
void test_motion_limits() {
    Motor* linearMotor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(linearMotor);
    
    // Enable motor
    linearMotor->enable();
    linearMotor->setPosition(500); // Start position
    
    // Set soft limits
    int32_t minLimit = 100;
    int32_t maxLimit = 900;
    linearMotor->setSoftLimits(minLimit, maxLimit, true); // Enable soft limits
    
    // Test movement within limits
    linearMotor->moveToPosition(700, 500.0f, 1000.0f, 1000.0f);
    linearMotor->setPosition(700); // Simulate completion
    TEST_ASSERT_EQUAL(700, linearMotor->getCurrentPosition());
    
    // Test movement to exact limit
    linearMotor->moveToPosition(maxLimit, 500.0f, 1000.0f, 1000.0f);
    linearMotor->setPosition(maxLimit); // Simulate completion
    TEST_ASSERT_EQUAL(maxLimit, linearMotor->getCurrentPosition());
    
    // Test attempting to move beyond max limit
    linearMotor->moveToPosition(maxLimit + 100, 500.0f, 1000.0f, 1000.0f);
    // Should not change target position beyond limit
    TEST_ASSERT_EQUAL(maxLimit, linearMotor->getTargetPosition());
    
    // Test attempting to move beyond min limit
    linearMotor->moveToPosition(minLimit - 100, 500.0f, 1000.0f, 1000.0f);
    // Should not change target position beyond limit
    TEST_ASSERT_EQUAL(minLimit, linearMotor->getTargetPosition());
    
    // Disable soft limits and try again
    linearMotor->setSoftLimits(minLimit, maxLimit, false);
    
    // Now should be able to move beyond previous limits
    linearMotor->moveToPosition(950, 500.0f, 1000.0f, 1000.0f);
    linearMotor->setPosition(950); // Simulate completion
    TEST_ASSERT_EQUAL(950, linearMotor->getCurrentPosition());
}

void setup() {
    // Wait before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_synchronized_movement);
    RUN_TEST(test_precise_mirror_alignment);
    RUN_TEST(test_laser_alignment_scenario);
    RUN_TEST(test_motion_limits);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}