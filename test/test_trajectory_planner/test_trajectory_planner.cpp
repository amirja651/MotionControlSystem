#include <unity.h>
#include <Arduino.h>
#include "core/TrajectoryPlanner.h"
#include "../test_mocks.h"

// Define test cases for TrajectoryPlanner

void setUp(void) {
    // Setup code before each test
}

void tearDown(void) {
    // Cleanup code after each test
}

void test_trajectory_planner_initialization() {
    // Create TrajectoryPlanner with default parameters
    TrajectoryPlanner planner(1000.0f, 2000.0f, 2000.0f);
    
    // Test that it initializes correctly
    TEST_ASSERT_TRUE(planner.isComplete());
}

void test_trajectory_position_move_planning() {
    TrajectoryPlanner planner(1000.0f, 2000.0f, 2000.0f);
    
    // Plan a simple position move
    int32_t startPos = 0;
    int32_t targetPos = 1000;
    float startVel = 0.0f;
    float maxVel = 500.0f;
    float accel = 1000.0f;
    
    bool success = planner.planPositionMove(startPos, targetPos, startVel, maxVel, accel, accel);
    
    // Should plan successfully
    TEST_ASSERT_TRUE(success);
    
    // Should not be complete immediately
    TEST_ASSERT_FALSE(planner.isComplete());
    
    // Initial position and velocity should match what we set
    TEST_ASSERT_FLOAT_WITHIN(0.1f, static_cast<float>(startPos), planner.getCurrentPosition());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, startVel, planner.getCurrentVelocity());
}

void test_trajectory_velocity_move_planning() {
    TrajectoryPlanner planner(1000.0f, 2000.0f, 2000.0f);
    
    // Plan a velocity move
    float startVel = 0.0f;
    float targetVel = 500.0f;
    float accel = 1000.0f;
    
    bool success = planner.planVelocityMove(startVel, targetVel, accel);
    
    // Should plan successfully
    TEST_ASSERT_TRUE(success);
    
    // Should not be complete immediately
    TEST_ASSERT_FALSE(planner.isComplete());
    
    // Initial velocity should match what we set
    TEST_ASSERT_FLOAT_WITHIN(0.1f, startVel, planner.getCurrentVelocity());
}

void test_trajectory_update() {
    TrajectoryPlanner planner(1000.0f, 2000.0f, 2000.0f);
    
    // Plan a position move
    int32_t startPos = 0;
    int32_t targetPos = 1000;
    planner.planPositionMove(startPos, targetPos, 0.0f, 500.0f, 1000.0f, 1000.0f);
    
    // Update for a portion of the total time
    planner.update(500000); // 500ms
    
    // Position and velocity should have changed
    TEST_ASSERT_TRUE(planner.getCurrentPosition() > 0.0f);
    TEST_ASSERT_TRUE(planner.getCurrentPosition() < static_cast<float>(targetPos));
    TEST_ASSERT_TRUE(planner.getCurrentVelocity() > 0.0f);
}

void test_trajectory_completion() {
    TrajectoryPlanner planner(1000.0f, 2000.0f, 2000.0f);
    
    // Plan a short position move that will complete quickly
    planner.planPositionMove(0, 100, 0.0f, 1000.0f, 5000.0f, 5000.0f);
    
    // Create a large update that should complete the trajectory
    planner.update(1000000); // 1 second
    
    // Should be complete
    TEST_ASSERT_TRUE(planner.isComplete());
    
    // Final position should be the target
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, planner.getCurrentPosition());
    
    // Final velocity should be zero for a position move
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, planner.getCurrentVelocity());
}

void test_trajectory_reset() {
    TrajectoryPlanner planner(1000.0f, 2000.0f, 2000.0f);
    
    // Plan a position move
    planner.planPositionMove(0, 1000, 0.0f, 500.0f, 1000.0f, 1000.0f);
    
    // Reset the planner
    planner.reset();
    
    // Should be complete after reset
    TEST_ASSERT_TRUE(planner.isComplete());
    
    // Position and velocity should be zero after reset
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, planner.getCurrentPosition());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, planner.getCurrentVelocity());
}

void test_trajectory_time_remaining() {
    TrajectoryPlanner planner(1000.0f, 2000.0f, 2000.0f);
    
    // Plan a position move that takes about 2 seconds
    planner.planPositionMove(0, 1000, 0.0f, 1000.0f, 1000.0f, 1000.0f);
    
    // Get initial time remaining
    float initialTimeRemaining = planner.getTimeRemaining();
    
    // Update for half a second
    planner.update(500000); // 500ms
    
    // Get new time remaining
    float newTimeRemaining = planner.getTimeRemaining();
    
    // New time should be about 0.5 seconds less than initial time
    TEST_ASSERT_TRUE(initialTimeRemaining > newTimeRemaining);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.5f, initialTimeRemaining - newTimeRemaining);
}

void test_trajectory_parameter_setters() {
    TrajectoryPlanner planner(1000.0f, 2000.0f, 2000.0f);
    
    // Set new parameters
    float newMaxVel = 1500.0f;
    float newMaxAccel = 3000.0f;
    float newMaxDecel = 2500.0f;
    float newMaxJerk = 5000.0f;
    
    planner.setMaxVelocity(newMaxVel);
    planner.setMaxAcceleration(newMaxAccel);
    planner.setMaxDeceleration(newMaxDecel);
    planner.setMaxJerk(newMaxJerk);
    
    // Plan a move - the new parameters should be used
    planner.planPositionMove(0, 1000, 0.0f);
    
    // Update for some time
    planner.update(100000); // 100ms
    
    // Velocity should be ramping up under the new acceleration
    float expectedVel = 3000.0f * 0.1f; // acceleration * time
    TEST_ASSERT_FLOAT_WITHIN(50.0f, expectedVel, planner.getCurrentVelocity());
}

void test_s_curve_profile() {
    TrajectoryPlanner planner(1000.0f, 2000.0f, 2000.0f, 10000.0f);
    
    // Plan an S-curve move
    bool success = planner.planPositionMove(
        0, 1000, 0.0f, 500.0f, 1000.0f, 1000.0f, 5000.0f, ProfileType::S_CURVE);
    
    TEST_ASSERT_TRUE(success);
    
    // Update through the profile and check that velocity changes smoothly
    float prevVel = 0.0f;
    float prevAccel = 0.0f;
    
    // Sample a few points to see if velocity is changing smoothly
    for (int i = 0; i < 10; i++) {
        planner.update(100000); // 100ms
        
        float curVel = planner.getCurrentVelocity();
        float curAccel = planner.getCurrentAcceleration();
        
        // In an S-curve, acceleration should start small, increase, then decrease
        if (i > 0 && i < 9) {
            // Velocity should always be increasing in first half of move
            TEST_ASSERT_TRUE(curVel > prevVel);
        }
        
        prevVel = curVel;
        prevAccel = curAccel;
    }
}

void setup() {
    // Wait before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_trajectory_planner_initialization);
    RUN_TEST(test_trajectory_position_move_planning);
    RUN_TEST(test_trajectory_velocity_move_planning);
    RUN_TEST(test_trajectory_update);
    RUN_TEST(test_trajectory_completion);
    RUN_TEST(test_trajectory_reset);
    RUN_TEST(test_trajectory_time_remaining);
    RUN_TEST(test_trajectory_parameter_setters);
    RUN_TEST(test_s_curve_profile);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}