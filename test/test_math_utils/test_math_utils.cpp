#include <unity.h>
#include <Arduino.h>
#include "utils/MathUtils.h"

// Define test cases for MathUtils

void test_constrain_value() {
    // Test with integers
    TEST_ASSERT_EQUAL(5, MathUtils::constrainValue(5, 0, 10));
    TEST_ASSERT_EQUAL(0, MathUtils::constrainValue(-5, 0, 10));
    TEST_ASSERT_EQUAL(10, MathUtils::constrainValue(15, 0, 10));
    
    // Test with floats
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.5f, MathUtils::constrainValue(5.5f, 0.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, MathUtils::constrainValue(-5.5f, 0.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, MathUtils::constrainValue(15.5f, 0.0f, 10.0f));
}

void test_lerp() {
    // Test linear interpolation
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, MathUtils::lerp(0.0f, 10.0f, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, MathUtils::lerp(0.0f, 10.0f, 0.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, MathUtils::lerp(0.0f, 10.0f, 1.0f));
    
    // Test with t out of range
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, MathUtils::lerp(0.0f, 10.0f, -0.5f)); // Should be constrained to 0.0
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, MathUtils::lerp(0.0f, 10.0f, 1.5f)); // Should be constrained to 1.0
    
    // Test with different types
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, MathUtils::lerp(-5.0f, 15.0f, 0.5f));
}

void test_calculate_velocity() {
    // Test velocity calculation
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, MathUtils::calculateVelocity(10.0f, 1.0f)); // 10 units in 1 second = 10 units/sec
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, MathUtils::calculateVelocity(10.0f, 2.0f));  // 10 units in 2 seconds = 5 units/sec
    
    // Test edge case - zero time
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, MathUtils::calculateVelocity(10.0f, 0.0f)); // Should handle divide by zero
}

void test_calculate_acceleration() {
    // Test acceleration calculation
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, MathUtils::calculateAcceleration(10.0f, 2.0f)); // 10 velocity change in 2 seconds = 5 units/sec^2
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, MathUtils::calculateAcceleration(10.0f, 1.0f)); // 10 velocity change in 1 second = 10 units/sec^2
    
    // Test edge case - zero time
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, MathUtils::calculateAcceleration(10.0f, 0.0f)); // Should handle divide by zero
}

void test_apply_dead_band() {
    // Test dead band application
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, MathUtils::applyDeadBand(0.5f, 1.0f)); // Value below threshold, should be 0
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, MathUtils::applyDeadBand(1.5f, 1.0f)); // Value above threshold, should remain unchanged
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.5f, MathUtils::applyDeadBand(-1.5f, 1.0f)); // Negative value below -threshold, should remain unchanged
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, MathUtils::applyDeadBand(-0.5f, 1.0f)); // Negative value within -threshold, should be 0
}

void test_calculate_trapezoidal_profile_duration() {
    // Test trapezoidal profile duration calculation
    
    // Case 1: Triangle profile (can't reach max velocity)
    float distance1 = 100.0f;
    float maxVelocity1 = 100.0f;
    float acceleration1 = 50.0f;
    float expectedTime1 = 2.0f * sqrt(distance1 / acceleration1); // For triangle profile
    float actualTime1 = MathUtils::calculateTrapezoidalProfileDuration(distance1, maxVelocity1, acceleration1);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expectedTime1, actualTime1);
    
    // Case 2: Trapezoidal profile (can reach max velocity)
    float distance2 = 1000.0f;
    float maxVelocity2 = 100.0f;
    float acceleration2 = 50.0f;
    float timeToAccelerate = maxVelocity2 / acceleration2;
    float distanceAtConstantVelocity = distance2 - (acceleration2 * timeToAccelerate * timeToAccelerate);
    float timeAtConstantVelocity = distanceAtConstantVelocity / maxVelocity2;
    float expectedTime2 = 2.0f * timeToAccelerate + timeAtConstantVelocity;
    float actualTime2 = MathUtils::calculateTrapezoidalProfileDuration(distance2, maxVelocity2, acceleration2);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expectedTime2, actualTime2);
}

void test_calculate_trapezoidal_position() {
    // Test position calculation at different points in a trapezoidal profile
    
    // Setup a simple move: 100 units distance, 10 units/s velocity, 10 units/s^2 acceleration
    float distance = 100.0f;
    float maxVelocity = 10.0f;
    float acceleration = 10.0f;
    float totalTime = MathUtils::calculateTrapezoidalProfileDuration(distance, maxVelocity, acceleration);
    
    // Test at start (t=0)
    float posStart = MathUtils::calculateTrapezoidalPosition(0.0f, totalTime, distance, maxVelocity, acceleration);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, posStart);
    
    // Test at end (t=totalTime)
    float posEnd = MathUtils::calculateTrapezoidalPosition(totalTime, totalTime, distance, maxVelocity, acceleration);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, distance, posEnd);
    
    // Test at acceleration phase (t=0.5)
    float timeToAccelerate = maxVelocity / acceleration; // Time to reach max velocity
    float posAccel = MathUtils::calculateTrapezoidalPosition(0.5f, totalTime, distance, maxVelocity, acceleration);
    // Expected position during acceleration: s = 0.5 * a * t^2
    float expectedPosAccel = 0.5f * acceleration * 0.5f * 0.5f;
    TEST_ASSERT_FLOAT_WITHIN(0.5f, expectedPosAccel, posAccel); // Allow for more tolerance due to different calculation methods
}

void test_calculate_trapezoidal_velocity() {
    // Test velocity calculation at different points in a trapezoidal profile
    
    // Setup a simple move: 100 units distance, 10 units/s velocity, 10 units/s^2 acceleration
    float distance = 100.0f;
    float maxVelocity = 10.0f;
    float acceleration = 10.0f;
    float totalTime = MathUtils::calculateTrapezoidalProfileDuration(distance, maxVelocity, acceleration);
    
    // Test at start (t=0)
    float velStart = MathUtils::calculateTrapezoidalVelocity(0.0f, totalTime, distance, maxVelocity, acceleration);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, velStart);
    
    // Test at end (t=totalTime)
    float velEnd = MathUtils::calculateTrapezoidalVelocity(totalTime, totalTime, distance, maxVelocity, acceleration);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, velEnd);
    
    // Test at acceleration phase (t=0.5)
    float vel = MathUtils::calculateTrapezoidalVelocity(0.5f, totalTime, distance, maxVelocity, acceleration);
    // Expected velocity during acceleration: v = a * t
    float expectedVel = acceleration * 0.5f;
    TEST_ASSERT_FLOAT_WITHIN(0.5f, expectedVel, vel); // Allow for more tolerance
}

void test_calculate_s_curve_position() {
    // Test S-curve position calculation
    
    // Setup a simple move: 100 units distance, 10 units/s velocity, 10 units/s^2 acceleration, 50 units/s^3 jerk
    float distance = 100.0f;
    float maxVelocity = 10.0f;
    float maxAcceleration = 10.0f;
    float maxJerk = 50.0f;
    float totalTime = 12.0f; // Approximate time for this profile
    
    // Test at start (t=0)
    float posStart = MathUtils::calculateSCurvePosition(0.0f, totalTime, distance, maxVelocity, maxAcceleration, maxJerk);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, posStart);
    
    // Test at end (t=totalTime)
    float posEnd = MathUtils::calculateSCurvePosition(totalTime, totalTime, distance, maxVelocity, maxAcceleration, maxJerk);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, distance, posEnd);
    
    // Test at middle (t=totalTime/2)
    float posMid = MathUtils::calculateSCurvePosition(totalTime/2, totalTime, distance, maxVelocity, maxAcceleration, maxJerk);
    // S-curve should be approximately at half distance at half time
    TEST_ASSERT_FLOAT_WITHIN(distance * 0.3f, distance * 0.5f, posMid); // Allow for significant tolerance due to S-curve nature
}

void test_low_pass_filter() {
    // Test low-pass filter
    
    // Test with alpha = 1.0 (no filtering)
    float filtered = MathUtils::lowPassFilter(0.0f, 10.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, filtered);
    
    // Test with alpha = 0.0 (complete filtering)
    filtered = MathUtils::lowPassFilter(5.0f, 10.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, filtered);
    
    // Test with alpha = 0.5 (partial filtering)
    filtered = MathUtils::lowPassFilter(0.0f, 10.0f, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, filtered);
}

void test_steps_to_mm() {
    // Test steps to mm conversion
    float mm = MathUtils::stepsToMm(1000, 100.0f); // 100 steps per mm
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, mm);
    
    // Test with negative steps
    mm = MathUtils::stepsToMm(-1000, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -10.0f, mm);
}

void test_mm_to_steps() {
    // Test mm to steps conversion
    int32_t steps = MathUtils::mmToSteps(10.0f, 100.0f); // 100 steps per mm
    TEST_ASSERT_EQUAL(1000, steps);
    
    // Test with negative mm
    steps = MathUtils::mmToSteps(-10.0f, 100.0f);
    TEST_ASSERT_EQUAL(-1000, steps);
    
    // Test with fractional mm
    steps = MathUtils::mmToSteps(10.5f, 100.0f);
    TEST_ASSERT_EQUAL(1050, steps);
}

void setup() {
    // Wait before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_constrain_value);
    RUN_TEST(test_lerp);
    RUN_TEST(test_calculate_velocity);
    RUN_TEST(test_calculate_acceleration);
    RUN_TEST(test_apply_dead_band);
    RUN_TEST(test_calculate_trapezoidal_profile_duration);
    RUN_TEST(test_calculate_trapezoidal_position);
    RUN_TEST(test_calculate_trapezoidal_velocity);
    RUN_TEST(test_calculate_s_curve_position);
    RUN_TEST(test_low_pass_filter);
    RUN_TEST(test_steps_to_mm);
    RUN_TEST(test_mm_to_steps);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}