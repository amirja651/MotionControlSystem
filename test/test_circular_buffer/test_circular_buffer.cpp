#include <unity.h>
#include <Arduino.h>
#include "utils/CircularBuffer.h"

// Define test cases for CircularBuffer

void test_circular_buffer_initial_state(void) {
    CircularBuffer<int, 8> buffer;
    
    TEST_ASSERT_TRUE(buffer.isEmpty());
    TEST_ASSERT_FALSE(buffer.isFull());
    TEST_ASSERT_EQUAL(0, buffer.count());
    TEST_ASSERT_EQUAL(8, buffer.capacity());
}

void test_circular_buffer_push_and_retrieve(void) {
    CircularBuffer<int, 8> buffer;
    
    // Push some elements
    buffer.push(10);
    buffer.push(20);
    buffer.push(30);
    
    // Check buffer state
    TEST_ASSERT_FALSE(buffer.isEmpty());
    TEST_ASSERT_FALSE(buffer.isFull());
    TEST_ASSERT_EQUAL(3, buffer.count());
    
    // Check element retrieval
    TEST_ASSERT_EQUAL(30, buffer.newest());
    TEST_ASSERT_EQUAL(10, buffer.oldest());
    TEST_ASSERT_EQUAL(10, buffer.at(0));
    TEST_ASSERT_EQUAL(20, buffer.at(1));
    TEST_ASSERT_EQUAL(30, buffer.at(2));
}

void test_circular_buffer_clear(void) {
    CircularBuffer<int, 8> buffer;
    
    // Add items
    buffer.push(10);
    buffer.push(20);
    buffer.push(30);
    
    // Clear buffer
    buffer.clear();
    
    // Check state after clear
    TEST_ASSERT_TRUE(buffer.isEmpty());
    TEST_ASSERT_EQUAL(0, buffer.count());
}

void test_circular_buffer_overflow(void) {
    CircularBuffer<int, 4> buffer;
    
    // Fill the buffer
    buffer.push(10);
    buffer.push(20);
    buffer.push(30);
    buffer.push(40);
    
    // Check state when full
    TEST_ASSERT_TRUE(buffer.isFull());
    TEST_ASSERT_EQUAL(4, buffer.count());
    TEST_ASSERT_EQUAL(10, buffer.oldest());
    TEST_ASSERT_EQUAL(40, buffer.newest());
    
    // Push another element (causing overflow)
    buffer.push(50);
    
    // Check state after overflow
    TEST_ASSERT_TRUE(buffer.isFull());
    TEST_ASSERT_EQUAL(4, buffer.count());
    TEST_ASSERT_EQUAL(20, buffer.oldest());  // 10 should be overwritten
    TEST_ASSERT_EQUAL(50, buffer.newest());
}

void test_circular_buffer_statistics(void) {
    CircularBuffer<int, 4> buffer;
    
    // Add elements
    buffer.push(10);
    buffer.push(20);
    buffer.push(30);
    buffer.push(40);
    
    // Test statistical functions
    TEST_ASSERT_EQUAL(25, buffer.average());
    TEST_ASSERT_EQUAL(10, buffer.minimum());
    TEST_ASSERT_EQUAL(40, buffer.maximum());
}

// Edge case: buffer of size 1
void test_circular_buffer_single_element(void) {
    CircularBuffer<int, 1> buffer;
    
    TEST_ASSERT_TRUE(buffer.isEmpty());
    
    buffer.push(42);
    
    TEST_ASSERT_FALSE(buffer.isEmpty());
    TEST_ASSERT_TRUE(buffer.isFull());
    TEST_ASSERT_EQUAL(1, buffer.count());
    TEST_ASSERT_EQUAL(42, buffer.newest());
    TEST_ASSERT_EQUAL(42, buffer.oldest());
    
    buffer.push(99);
    
    TEST_ASSERT_EQUAL(99, buffer.newest());
    TEST_ASSERT_EQUAL(99, buffer.oldest());
}

// Edge case: floating point values
void test_circular_buffer_float_values(void) {
    CircularBuffer<float, 4> buffer;
    
    buffer.push(1.5f);
    buffer.push(2.5f);
    buffer.push(3.5f);
    
    TEST_ASSERT_EQUAL_FLOAT(1.5f, buffer.oldest());
    TEST_ASSERT_EQUAL_FLOAT(3.5f, buffer.newest());
    TEST_ASSERT_EQUAL_FLOAT(2.5f, buffer.average());
}

void setup() {
    // Wait 2 seconds before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_circular_buffer_initial_state);
    RUN_TEST(test_circular_buffer_push_and_retrieve);
    RUN_TEST(test_circular_buffer_clear);
    RUN_TEST(test_circular_buffer_overflow);
    RUN_TEST(test_circular_buffer_statistics);
    RUN_TEST(test_circular_buffer_single_element);
    RUN_TEST(test_circular_buffer_float_values);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}