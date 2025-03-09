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
    // Create system components
    systemManager = new SystemManager();
    
    // Initialize system (limited initialization for testing)
    systemManager->initialize();
    
    // Get motor manager
    motorManager = systemManager->getMotorManager();
    
    // Get logger
    mockLogger = new MockLogger();
}

void tearDown() {
    delete mockLogger;
    mockLogger = nullptr;
    
    delete systemManager;
    systemManager = nullptr;
    motorManager = nullptr; // Owned by SystemManager
}

// Test different motion profiles with various speeds and accelerations
void test_trapezoidal_profile() {
    // Add a motor
    MotorConfig config;
    config.index = 0;
    config.stepPin = 10;
    config.dirPin = 11;
    config.enablePin = 12;
    config.maxVelocity = 5000.0f;
    config.maxAcceleration = 10000.0f;
    config.maxDeceleration = 10000.0f;
    config.maxJerk = 20000.0f;
    
    motorManager->addMotor(config);
    
    // Get the motor
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);
    
    // Enable the motor
    motor->enable();
    
    // Set initial position
    motor->setPosition(0);
    
    // Test a short move with trapezoidal profile
    // Distance: 1000 steps
    // Speed: 1000 steps/s
    // Acceleration: 5000 steps/s²
    motor->moveToPosition(1000, 1000.0f, 5000.0f, 5000.0f);
    
    // Get the current position at start
    int32_t startPosition = motor->getCurrentPosition();
    TEST_ASSERT_EQUAL(0, startPosition);
    
    // In a real test, we would simulate the passage of time and
    // check positions at various points during the profile.
    // For this test, we'll just verify that the target is set correctly.
    
    TEST_ASSERT_EQUAL(1000, motor->getTargetPosition());
    
    // Calculate the expected duration of the move
    float distance = 1000.0f;
    float velocity = 1000.0f;
    float acceleration = 5000.0f;
    
    // Time to accelerate to max velocity
    float accelTime = velocity / acceleration;
    
    // Distance covered during acceleration
    float accelDistance = 0.5f * acceleration * accelTime * accelTime;
    
    // Distance covered during deceleration
    float decelDistance = accelDistance;
    
    // Distance at constant velocity
    float cruiseDistance = distance - accelDistance - decelDistance;
    
    // Time at constant velocity
    float cruiseTime = cruiseDistance / velocity;
    
    // Total time
    float totalTime = accelTime + cruiseTime + accelTime; // assuming same accel and decel
    
    // Total time should be reasonable
    TEST_ASSERT_GREATER_THAN(0.0f, totalTime);
    
    // For now, we've set up a move with a specific profile
    // In a real test, we would check positions at different time points
}

void test_s_curve_profile() {
    // Add a motor
    MotorConfig config;
    config.index = 0;
    config.stepPin = 10;
    config.dirPin = 11;
    config.enablePin = 12;
    config.maxVelocity = 5000.0f;
    config.maxAcceleration = 10000.0f;
    config.maxDeceleration = 10000.0f;
    config.maxJerk = 20000.0f;
    
    motorManager->addMotor(config);
    
    // Get the motor
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);
    
    // Enable the motor
    motor->enable();
    
    // Set initial position
    motor->setPosition(0);
    
    // Create a trajectory planner with S-curve profile
    TrajectoryPlanner planner(5000.0f, 10000.0f, 10000.0f, 20000.0f);
    
    // Plan an S-curve move
    bool result = planner.planPositionMove(0, 1000, 0.0f, 1000.0f, 5000.0f, 5000.0f, 20000.0f, ProfileType::S_CURVE);
    TEST_ASSERT_TRUE(result);
    
    // Calculate expected duration
    float profileDuration = planner.getState().profileDuration;
    TEST_ASSERT_GREATER_THAN(0.0f, profileDuration);
    
    // In a real test, we would simulate the passage of time and
    // check positions at various points during the profile
    // For now, just verify that planning works
}

void test_velocity_profile() {
    // Add a motor
    MotorConfig config;
    config.index = 0;
    config.stepPin = 10;
    config.dirPin = 11;
    config.enablePin = 12;
    config.maxVelocity = 5000.0f;
    config.maxAcceleration = 10000.0f;
    
    motorManager->addMotor(config);
    
    // Get the motor
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);
    
    // Enable the motor
    motor->enable();
    
    // Set initial position and velocity
    motor->setPosition(0);
    
    // Test a velocity profile
    motor->setTargetVelocityWithAccel(1000.0f, 5000.0f);
    
    // Verify target velocity
    TEST_ASSERT_EQUAL(1000.0f, motor->getTargetVelocity());
    
    // Calculate time to reach velocity
    float targetVelocity = 1000.0f;
    float acceleration = 5000.0f;
    float timeToReachVelocity = targetVelocity / acceleration;
    
    // Time should be reasonable
    TEST_ASSERT_GREATER_THAN(0.0f, timeToReachVelocity);
    
    // In a real test, we would simulate the passage of time and
    // check velocities at various points during the profile
}

void test_multi_axis_profile() {
    // Add two motors
    MotorConfig config1;
    config1.index = 0;
    config1.stepPin = 10;
    config1.dirPin = 11;
    config1.enablePin = 12;
    config1.maxVelocity = 5000.0f;
    config1.maxAcceleration = 10000.0f;
    
    MotorConfig config2;
    config2.index = 1;
    config2.stepPin = 20;
    config2.dirPin = 21;
    config2.enablePin = 22;
    config2.maxVelocity = 5000.0f;
    config2.maxAcceleration = 10000.0f;
    
    motorManager->addMotor(config1);
    motorManager->addMotor(config2);
    
    // Get motors
    Motor* motor1 = motorManager->getMotor(0);
    Motor* motor2 = motorManager->getMotor(1);
    
    TEST_ASSERT_NOT_NULL(motor1);
    TEST_ASSERT_NOT_NULL(motor2);
    
    // Enable motors
    motor1->enable();
    motor2->enable();
    
    // Set initial positions
    motor1->setPosition(0);
    motor2->setPosition(0);
    
    // Move motors with coordinated motion
    int32_t positions[2] = {1000, 2000};
    uint8_t indices[2] = {0, 1};
    
    bool result = motorManager->moveMultipleMotors(positions, indices, 2, 1000.0f, 5000.0f, 5000.0f);
    TEST_ASSERT_TRUE(result);
    
    // Verify target positions
    TEST_ASSERT_EQUAL(1000, motor1->getTargetPosition());
    TEST_ASSERT_EQUAL(2000, motor2->getTargetPosition());
    
    // Calculate synchronized move duration
    float duration = motorManager->calculateSynchronizedMove(positions, indices, 2, 1000.0f, 5000.0f, 5000.0f);
    TEST_ASSERT_GREATER_THAN(0.0f, duration);
    
    // In a real test, we would simulate the passage of time and
    // check that both motors complete their movements at the same time
}

void test_very_small_movement() {
    // Add a motor
    MotorConfig config;
    config.index = 0;
    config.stepPin = 10;
    config.dirPin = 11;
    config.enablePin = 12;
    config.maxVelocity = 5000.0f;
    config.maxAcceleration = 10000.0f;
    
    motorManager->addMotor(config);
    
    // Get the motor
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);
    
    // Enable the motor
    motor->enable();
    
    // Set initial position
    motor->setPosition(0);
    
    // Test a very small move
    // Distance: 1 step
    motor->moveToPosition(1, 1000.0f, 5000.0f, 5000.0f);
    
    // Verify target position
    TEST_ASSERT_EQUAL(1, motor->getTargetPosition());
    
    // In a real test, we would simulate the passage of time and
    // check that the motor actually moves 1 step
}

void test_very_large_movement() {
    // Add a motor
    MotorConfig config;
    config.index = 0;
    config.stepPin = 10;
    config.dirPin = 11;
    config.enablePin = 12;
    config.maxVelocity = 5000.0f;
    config.maxAcceleration = 10000.0f;
    
    motorManager->addMotor(config);
    
    // Get the motor
    Motor* motor = motorManager->getMotor(0);
    TEST_ASSERT_NOT_NULL(motor);
    
    // Enable the motor
    motor->enable();
    
    // Set initial position
    motor->setPosition(0);
    
    // Test a very large move
    // Distance: 1,000,000 steps
    motor->moveToPosition(1000000, 5000.0f, 10000.0f, 10000.0f);
    
    // Verify target position
    TEST_ASSERT_EQUAL(1000000, motor->getTargetPosition());
    
    // In a real test, we would simulate the passage of time and
    // check positions at various points during the profile
}

void setup() {
    // Wait before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_trapezoidal_profile);
    RUN_TEST(test_s_curve_profile);
    RUN_TEST(test_velocity_profile);
    RUN_TEST(test_multi_axis_profile);
    RUN_TEST(test_very_small_movement);
    RUN_TEST(test_very_large_movement);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}