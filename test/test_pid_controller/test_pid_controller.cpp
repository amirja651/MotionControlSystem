#include <unity.h>
#include <Arduino.h>
#include "core/PIDController.h"

// Mock for MathUtils if needed
#include "utils/MathUtils.h"

// Define test cases for PIDController

void test_pid_controller_init(void) {
    PIDController pid(1.0f, 0.5f, 0.1f, 0.2f);
    
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.getProportionalTerm());
    TEST_ASSERT_EQUAL_FLOAT(0.5f, pid.getIntegralTerm());
    TEST_ASSERT_EQUAL_FLOAT(0.1f, pid.getDerivativeTerm());
    TEST_ASSERT_EQUAL_FLOAT(0.2f, pid.getFeedForwardTerm());
}

void test_pid_controller_set_gains(void) {
    PIDController pid(1.0f, 0.5f, 0.1f, 0.2f);
    
    pid.setGains(2.0f, 0.4f, 0.3f, 0.1f);
    
    TEST_ASSERT_EQUAL_FLOAT(2.0f, pid.getProportionalTerm());
    TEST_ASSERT_EQUAL_FLOAT(0.4f, pid.getIntegralTerm());
    TEST_ASSERT_EQUAL_FLOAT(0.3f, pid.getDerivativeTerm());
    TEST_ASSERT_EQUAL_FLOAT(0.1f, pid.getFeedForwardTerm());
}

void test_pid_controller_proportional(void) {
    // Create PID with only P term
    PIDController pid(2.0f, 0.0f, 0.0f, 0.0f);
    
    // Compute with 5.0 error (setpoint=10, process_var=5)
    float output = pid.compute(10.0f, 5.0f);
    
    // Output should equal error * P gain
    TEST_ASSERT_EQUAL_FLOAT(10.0f, output);
}

void test_pid_controller_reset(void) {
    PIDController pid(1.0f, 0.5f, 0.1f, 0.0f);
    
    // First compute to accumulate some values
    pid.compute(10.0f, 5.0f);
    
    // Reset controller
    pid.reset();
    
    // Last error should be reset
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.getLastError());
}

void test_pid_controller_with_integral(void) {
    // Create PID with only I term, and use a smaller dt (0.01) for predictable results
    PIDController pid(0.0f, 1.0f, 0.0f, 0.0f, 0.01f);
    
    // First call with constant error
    float output1 = pid.compute(10.0f, 9.0f);  // Error = 1.0
    
    // Second call with same error - integral should accumulate
    float output2 = pid.compute(10.0f, 9.0f);  // Error = 1.0
    
    // Test that output is increasing due to integral accumulation
    TEST_ASSERT_TRUE(output2 > output1);
}

void test_pid_controller_derivative(void) {
    // Create PID with only D term
    PIDController pid(0.0f, 0.0f, 1.0f, 0.0f, 0.1f);
    
    // First call to initialize
    pid.compute(10.0f, 9.0f);  // Error = 1.0
    
    // Second call with increasing error - derivative should be positive
    float output1 = pid.compute(10.0f, 8.0f);  // Error = 2.0, change = 1.0
    
    // Third call with decreasing error - derivative should be negative
    float output2 = pid.compute(10.0f, 9.5f);  // Error = 0.5, change = -1.5
    
    // Derivative term should be negative for second case (error is decreasing)
    TEST_ASSERT_TRUE(output1 > 0);
    TEST_ASSERT_TRUE(output2 < 0);
}

void test_pid_controller_feedforward(void) {
    // Create PID with only FF term
    PIDController pid(0.0f, 0.0f, 0.0f, 2.0f);
    
    // Compute with FF value
    float output = pid.compute(10.0f, 10.0f, 5.0f);  // No error, FF=5.0
    
    // Output should equal FF value * FF gain
    TEST_ASSERT_EQUAL_FLOAT(10.0f, output);
}

void test_pid_controller_output_limits(void) {
    PIDController pid(10.0f, 0.0f, 0.0f, 0.0f);
    
    // Set output limits
    pid.setOutputLimits(-5.0f, 5.0f);
    
    // Compute with large error (should saturate)
    float output1 = pid.compute(10.0f, 0.0f);  // Error = 10, P output = 100
    float output2 = pid.compute(0.0f, 10.0f);  // Error = -10, P output = -100
    
    // Output should be limited
    TEST_ASSERT_EQUAL_FLOAT(5.0f, output1);
    TEST_ASSERT_EQUAL_FLOAT(-5.0f, output2);
}

void test_pid_controller_anti_windup(void) {
    // Create PID with large I term
    PIDController pid(0.0f, 10.0f, 0.0f, 0.0f, 0.1f);
    
    // Set anti-windup limit
    pid.setAntiWindupLimits(1.0f);
    
    // Several iterations with a constant error to accumulate integral
    for (int i = 0; i < 10; i++) {
        pid.compute(10.0f, 9.0f);  // Error = 1.0
    }
    
    // Integral term should be limited
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(1.0f, pid.getIntegralTerm());
}

void setup() {
    // Wait 2 seconds before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_pid_controller_init);
    RUN_TEST(test_pid_controller_set_gains);
    RUN_TEST(test_pid_controller_proportional);
    RUN_TEST(test_pid_controller_reset);
    RUN_TEST(test_pid_controller_with_integral);
    RUN_TEST(test_pid_controller_derivative);
    RUN_TEST(test_pid_controller_feedforward);
    RUN_TEST(test_pid_controller_output_limits);
    RUN_TEST(test_pid_controller_anti_windup);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}