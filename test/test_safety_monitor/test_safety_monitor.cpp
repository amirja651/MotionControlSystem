#include <unity.h>
#include <Arduino.h>
#include "core/SafetyMonitor.h"
#include "MotorManager.h"
#include "../include/test_mocks.h"

// Global variables for testing
MotorManager* motorManager = nullptr;
MockLogger* mockLogger = nullptr;
SafetyMonitor* safetyMonitor = nullptr;

void setUp() {
    // Create mock objects
    motorManager = new MotorManager(2);
    mockLogger = new MockLogger();
    safetyMonitor = new SafetyMonitor(motorManager, mockLogger);
    
    // Initialize components
    motorManager->initialize();
    safetyMonitor->initialize();
    
    // Reset logger counts
    mockLogger->reset();
}

void tearDown() {
    delete safetyMonitor;
    delete mockLogger;
    delete motorManager;
    
    safetyMonitor = nullptr;
    mockLogger = nullptr;
    motorManager = nullptr;
}

void test_safety_monitor_initialization() {
    // Test that initialization works
    SafetyMonitor monitor(motorManager, mockLogger);
    bool result = monitor.initialize();
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(SystemSafetyStatus::NORMAL, monitor.getStatus());
    TEST_ASSERT_EQUAL(SafetyCode::NONE, monitor.getLastSafetyCode());
    TEST_ASSERT_FALSE(monitor.isEmergencyStop());
}

void test_safety_monitor_emergency_stop() {
    // Test triggering emergency stop
    safetyMonitor->triggerEmergencyStop(SafetyCode::EMERGENCY_STOP_PRESSED);
    
    // Check status
    TEST_ASSERT_EQUAL(SystemSafetyStatus::EMERGENCY_STOP, safetyMonitor->getStatus());
    TEST_ASSERT_EQUAL(SafetyCode::EMERGENCY_STOP_PRESSED, safetyMonitor->getLastSafetyCode());
    TEST_ASSERT_TRUE(safetyMonitor->isEmergencyStop());
    
    // Check that the logger was called
    TEST_ASSERT_GREATER_THAN(0, mockLogger->getErrorCount());
    
    // Reset emergency stop
    bool resetResult = safetyMonitor->resetEmergencyStop();
    TEST_ASSERT_TRUE(resetResult);
    
    // Check status after reset
    TEST_ASSERT_EQUAL(SystemSafetyStatus::NORMAL, safetyMonitor->getStatus());
    TEST_ASSERT_EQUAL(SafetyCode::NONE, safetyMonitor->getLastSafetyCode());
    TEST_ASSERT_FALSE(safetyMonitor->isEmergencyStop());
}

void test_safety_monitor_position_check() {
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
    
    // Enable the motor
    motor->enable();
    
    // Set position tolerances
    safetyMonitor->setPositionTolerances(50, 100);
    
    // Set a target position that's within warning threshold but below error threshold
    motor->setTargetPosition(1000);
    
    // In a real test, we would need to simulate a position error
    // by manipulating the encoder position vs. target position
    // This would typically require direct access to the internal state of the Motor object
    
    // For now, we'll just test that the safety check runs without errors
    SystemSafetyStatus status = safetyMonitor->checkSafety();
    
    // Should remain normal since we haven't actually created an error condition
    TEST_ASSERT_EQUAL(SystemSafetyStatus::NORMAL, status);
}

void test_safety_monitor_velocity_check() {
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
    
    // Enable the motor
    motor->enable();
    
    // Set velocity tolerances
    safetyMonitor->setVelocityTolerances(50.0f, 100.0f);
    
    // Set a target velocity
    motor->setTargetVelocity(500.0f);
    
    // In a real test, we would need to simulate a velocity error
    // by manipulating the encoder velocity vs. target velocity
    
    // For now, we'll just test that the safety check runs without errors
    SystemSafetyStatus status = safetyMonitor->checkSafety();
    
    // Should remain normal since we haven't actually created an error condition
    TEST_ASSERT_EQUAL(SystemSafetyStatus::NORMAL, status);
}

void test_safety_monitor_temperature_check() {
    // Set temperature tolerances
    safetyMonitor->setTemperatureTolerances(60.0f, 80.0f);
    
    // In a real test, we would need to simulate a temperature reading
    // but this is usually done through hardware sensors
    
    // For now, we'll just test that the safety check runs without errors
    SystemSafetyStatus status = safetyMonitor->checkSafety();
    
    // Should remain normal since we haven't actually created an error condition
    TEST_ASSERT_EQUAL(SystemSafetyStatus::NORMAL, status);
}

void test_safety_monitor_voltage_check() {
    // Set voltage tolerances
    safetyMonitor->setVoltageTolerances(3.0f, 3.6f, 0.1f);
    
    // In a real test, we would need to simulate a voltage reading
    // but this is usually done through hardware sensors
    
    // For now, we'll just test that the safety check runs without errors
    SystemSafetyStatus status = safetyMonitor->checkSafety();
    
    // Should remain normal since we haven't actually created an error condition
    TEST_ASSERT_EQUAL(SystemSafetyStatus::NORMAL, status);
}

void test_safety_monitor_emergency_stop_pin() {
    // Configure emergency stop pin
    // Note: In a test environment, pin state can't be easily manipulated
    safetyMonitor->setEmergencyStopPin(15, LOW);
    
    // In a real test, we would simulate a pin state change
    
    // For now, we'll just test that the safety check runs without errors
    SystemSafetyStatus status = safetyMonitor->checkSafety();
    
    // Should remain normal since we haven't actually triggered the emergency stop
    TEST_ASSERT_EQUAL(SystemSafetyStatus::NORMAL, status);
}

void test_safety_monitor_statistics() {
    // Get initial statistics
    uint32_t warningCount, errorCount, emergencyStopCount;
    safetyMonitor->getSafetyStats(warningCount, errorCount, emergencyStopCount);
    
    // All should be zero initially
    TEST_ASSERT_EQUAL(0, warningCount);
    TEST_ASSERT_EQUAL(0, errorCount);
    TEST_ASSERT_EQUAL(0, emergencyStopCount);
    
    // Trigger an emergency stop
    safetyMonitor->triggerEmergencyStop(SafetyCode::EMERGENCY_STOP_PRESSED);
    
    // Get statistics again
    safetyMonitor->getSafetyStats(warningCount, errorCount, emergencyStopCount);
    
    // Emergency stop count should be incremented
    TEST_ASSERT_EQUAL(0, warningCount);
    TEST_ASSERT_EQUAL(0, errorCount);
    TEST_ASSERT_EQUAL(1, emergencyStopCount);
    
    // Clear statistics
    safetyMonitor->clearSafetyStats();
    
    // Get statistics again
    safetyMonitor->getSafetyStats(warningCount, errorCount, emergencyStopCount);
    
    // All should be zero again
    TEST_ASSERT_EQUAL(0, warningCount);
    TEST_ASSERT_EQUAL(0, errorCount);
    TEST_ASSERT_EQUAL(0, emergencyStopCount);
}

void test_safety_monitor_multiple_emergency_stops() {
    // Trigger first emergency stop
    safetyMonitor->triggerEmergencyStop(SafetyCode::EMERGENCY_STOP_PRESSED);
    
    // Try to trigger another one (shouldn't increment count)
    safetyMonitor->triggerEmergencyStop(SafetyCode::POSITION_ERROR);
    
    // Get statistics
    uint32_t warningCount, errorCount, emergencyStopCount;
    safetyMonitor->getSafetyStats(warningCount, errorCount, emergencyStopCount);
    
    // Emergency stop count should still be 1
    TEST_ASSERT_EQUAL(1, emergencyStopCount);
    
    // Reset emergency stop
    safetyMonitor->resetEmergencyStop();
    
    // Trigger another emergency stop
    safetyMonitor->triggerEmergencyStop(SafetyCode::VELOCITY_ERROR);
    
    // Get statistics again
    safetyMonitor->getSafetyStats(warningCount, errorCount, emergencyStopCount);
    
    // Emergency stop count should now be 2
    TEST_ASSERT_EQUAL(2, emergencyStopCount);
}

void setup() {
    // Wait before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_safety_monitor_initialization);
    RUN_TEST(test_safety_monitor_emergency_stop);
    RUN_TEST(test_safety_monitor_position_check);
    RUN_TEST(test_safety_monitor_velocity_check);
    RUN_TEST(test_safety_monitor_temperature_check);
    RUN_TEST(test_safety_monitor_voltage_check);
    RUN_TEST(test_safety_monitor_emergency_stop_pin);
    RUN_TEST(test_safety_monitor_statistics);
    RUN_TEST(test_safety_monitor_multiple_emergency_stops);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}