#include <unity.h>
#include <Arduino.h>
#include "core/Encoder.h"
#include "../include/test_mocks.h"

// Define test cases for Encoder

void setUp(void) {
    // Setup code before each test
}

void tearDown(void) {
    // Cleanup code after each test
}

void test_encoder_initialization() {
    // Create encoder with specific pins and PPR
    uint8_t encAPin = 18;
    uint8_t encBPin = 19;
    uint16_t ppr = 2000;
    bool invertDir = false;
    
    Encoder encoder(encAPin, encBPin, ppr, invertDir);
    
    // Initialize won't actually use hardware in test environment
    bool result = encoder.initialize();
    
    // Should initialize successfully
    TEST_ASSERT_TRUE(result);
    
    // Initial position and velocity should be zero
    TEST_ASSERT_EQUAL_INT32(0, encoder.getPosition());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, encoder.getVelocity());
}

void test_encoder_set_position() {
    Encoder encoder(18, 19, 2000);
    encoder.initialize();
    
    // Set a position
    int32_t testPos = 1000;
    encoder.setPosition(testPos);
    
    // Position should be updated
    TEST_ASSERT_EQUAL_INT32(testPos, encoder.getPosition());
}

void test_encoder_set_velocity() {
    Encoder encoder(18, 19, 2000);
    encoder.initialize();
    
    // Set a velocity
    float testVel = 500.0f;
    encoder.setVelocity(testVel);
    
    // Velocity should be updated
    TEST_ASSERT_FLOAT_WITHIN(0.1f, testVel, encoder.getVelocity());
}

void test_encoder_reset() {
    Encoder encoder(18, 19, 2000);
    encoder.initialize();
    
    // Set non-zero position and velocity
    encoder.setPosition(1000);
    encoder.setVelocity(500.0f);
    
    // Reset encoder
    encoder.reset();
    
    // Position and velocity should be reset to zero
    TEST_ASSERT_EQUAL_INT32(0, encoder.getPosition());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, encoder.getVelocity());
}

void test_encoder_counts_per_revolution() {
    // Test with different PPR values
    uint16_t ppr = 2000;
    Encoder encoder(18, 19, ppr);
    
    // Get counts per revolution
    uint32_t cpr = encoder.getCountsPerRevolution();
    
    // With quadrature encoding, CPR = PPR * 4 * interpolation
    uint32_t expectedCpr = ppr * 4 * CONFIG_ENCODER_INTERPOLATION_FACTOR;
    TEST_ASSERT_EQUAL_UINT32(expectedCpr, cpr);
}

void test_encoder_update() {
    Encoder encoder(18, 19, 2000);
    encoder.initialize();
    
    // Manually set a position
    encoder.setPosition(1000);
    
    // Call update with some time delta
    uint32_t deltaTimeUs = 100000; // 100ms
    encoder.update(deltaTimeUs);
    
    // Position should remain unchanged (since we're not simulating transitions)
    TEST_ASSERT_EQUAL_INT32(1000, encoder.getPosition());
    
    // Filtered position should be approximately the same
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, encoder.getFilteredPosition());
}

void test_encoder_process_interrupt() {
    // This test simulates quadrature encoder signals
    Encoder encoder(18, 19, 2000);
    encoder.initialize();
    
    // Simulate a full quadrature cycle (4 state transitions)
    // AB: 00 -> 01 -> 11 -> 10 -> 00
    encoder.processInterrupt(false, true);  // 01
    encoder.processInterrupt(true, true);   // 11
    encoder.processInterrupt(true, false);  // 10
    encoder.processInterrupt(false, false); // 00
    
    // One full cycle should increment position by 1
    TEST_ASSERT_EQUAL_INT32(1, encoder.getPosition());
    
    // Now simulate reverse direction
    // AB: 00 -> 10 -> 11 -> 01 -> 00
    encoder.processInterrupt(true, false);  // 10
    encoder.processInterrupt(true, true);   // 11
    encoder.processInterrupt(false, true);  // 01
    encoder.processInterrupt(false, false); // 00
    
    // After one reverse cycle, should be back at position 0
    TEST_ASSERT_EQUAL_INT32(0, encoder.getPosition());
}

void test_encoder_direction_inversion() {
    // Test with inverted direction
    Encoder encoder(18, 19, 2000, true);
    encoder.initialize();
    
    // Simulate forward quadrature cycle as before
    encoder.processInterrupt(false, true);  // 01
    encoder.processInterrupt(true, true);   // 11
    encoder.processInterrupt(true, false);  // 10
    encoder.processInterrupt(false, false); // 00
    
    // With inverted direction, position should decrement
    TEST_ASSERT_EQUAL_INT32(-1, encoder.getPosition());
}

void test_encoder_open_loop() {
    // Test encoder in open-loop mode (no actual encoder connected)
    // This is indicated by setting the pin numbers to 0xFF
    Encoder encoder(0xFF, 0xFF, 2000);
    encoder.initialize();
    
    // Set a position and velocity
    encoder.setPosition(1000);
    encoder.setVelocity(500.0f);
    
    // Update encoder
    encoder.update(100000); // 100ms
    
    // Position and velocity should remain as set
    TEST_ASSERT_EQUAL_INT32(1000, encoder.getPosition());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 500.0f, encoder.getVelocity());
}

void test_encoder_velocity_calculation() {
    Encoder encoder(18, 19, 2000);
    encoder.initialize();
    
    // Set initial position
    encoder.setPosition(0);
    
    // Call update to initialize timing
    encoder.update(1000); // 1ms
    
    // Simulate position change over time
    for (int i = 0; i < 10; i++) {
        // Move position forward by 100 counts
        int32_t newPos = encoder.getPosition() + 100;
        encoder.setPosition(newPos);
        
        // Update with a fixed time delta
        uint32_t deltaTimeUs = 100000; // 100ms
        encoder.update(deltaTimeUs);
    }
    
    // After moving 100 counts every 100ms, velocity should be approximately 1000 counts/sec
    TEST_ASSERT_FLOAT_WITHIN(100.0f, 1000.0f, encoder.getVelocity());
}

void test_encoder_interpolation_factor() {
    Encoder encoder(18, 19, 2000);
    encoder.initialize();
    
    // Get default interpolation factor
    uint32_t defaultCpr = encoder.getCountsPerRevolution();
    
    // Change interpolation factor
    uint8_t newFactor = 8; // Half of default
    encoder.setInterpolationFactor(newFactor);
    
    // Get new counts per revolution
    uint32_t newCpr = encoder.getCountsPerRevolution();
    
    // CPR should change by the ratio of interpolation factors
    uint32_t expectedCpr = 2000 * 4 * newFactor; // PPR * 4 states * interpolation
    TEST_ASSERT_EQUAL_UINT32(expectedCpr, newCpr);
}

void setup() {
    // Wait before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_encoder_initialization);
    RUN_TEST(test_encoder_set_position);
    RUN_TEST(test_encoder_set_velocity);
    RUN_TEST(test_encoder_reset);
    RUN_TEST(test_encoder_counts_per_revolution);
    RUN_TEST(test_encoder_update);
    RUN_TEST(test_encoder_process_interrupt);
    RUN_TEST(test_encoder_direction_inversion);
    RUN_TEST(test_encoder_open_loop);
    RUN_TEST(test_encoder_velocity_calculation);
    RUN_TEST(test_encoder_interpolation_factor);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}