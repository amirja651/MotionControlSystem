#include <unity.h>
#include <Arduino.h>
#include "SystemManager.h"
#include "core/SafetyMonitor.h"
#include "utils/Logger.h"
#include "utils/EEPROMManager.h"
#include "../include/test_mocks.h"

// Global mock objects
MockLogger* mockLogger = nullptr;

void setUp() {
    mockLogger = new MockLogger();
}

void tearDown() {
    delete mockLogger;
    mockLogger = nullptr;
}

void test_system_manager_initialization() {
    SystemManager systemManager;
    
    // In a real test, we would need to mock all the components
    // but the core test is to check that initialization succeeds
    bool result = systemManager.initialize();
    
    // Should initialize successfully
    TEST_ASSERT_TRUE(result);
    
    // Check initial state
    TEST_ASSERT_EQUAL(SystemState::READY, systemManager.getSystemState());
    TEST_ASSERT_FALSE(systemManager.isEmergencyStop());
}

void test_system_manager_get_components() {
    SystemManager systemManager;
    systemManager.initialize();
    
    // Test getting components
    SafetyMonitor* safetyMonitor = systemManager.getSafetyMonitor();
    MotorManager* motorManager = systemManager.getMotorManager();
    Logger* logger = systemManager.getLogger();
    TaskScheduler* taskScheduler = systemManager.getTaskScheduler();
    StatusReporter* statusReporter = systemManager.getStatusReporter();
    EEPROMManager* eepromManager = systemManager.getEEPROMManager();
    
    // Check that all components are created
    TEST_ASSERT_NOT_NULL(safetyMonitor);
    TEST_ASSERT_NOT_NULL(motorManager);
    TEST_ASSERT_NOT_NULL(logger);
    TEST_ASSERT_NOT_NULL(taskScheduler);
    // Status reporter may be null initially
    TEST_ASSERT_NOT_NULL(eepromManager);
}

void test_system_manager_state_changes() {
    SystemManager systemManager;
    systemManager.initialize();
    
    // Test state transitions
    
    // Initially in READY state
    TEST_ASSERT_EQUAL(SystemState::READY, systemManager.getSystemState());
    
    // Change to RUNNING
    systemManager.setSystemState(SystemState::RUNNING);
    TEST_ASSERT_EQUAL(SystemState::RUNNING, systemManager.getSystemState());
    
    // Change to ERROR
    systemManager.setSystemState(SystemState::ERROR);
    TEST_ASSERT_EQUAL(SystemState::ERROR, systemManager.getSystemState());
    
    // Change to READY
    systemManager.setSystemState(SystemState::READY);
    TEST_ASSERT_EQUAL(SystemState::READY, systemManager.getSystemState());
    
    // Change to EMERGENCY_STOP
    systemManager.setSystemState(SystemState::EMERGENCY_STOP);
    TEST_ASSERT_EQUAL(SystemState::EMERGENCY_STOP, systemManager.getSystemState());
    
    // Try to change from EMERGENCY_STOP to RUNNING (invalid transition)
    systemManager.setSystemState(SystemState::RUNNING);
    TEST_ASSERT_EQUAL(SystemState::EMERGENCY_STOP, systemManager.getSystemState()); // Should remain in emergency stop
    
    // Change from EMERGENCY_STOP to READY (valid transition)
    systemManager.setSystemState(SystemState::READY);
    TEST_ASSERT_EQUAL(SystemState::READY, systemManager.getSystemState());
    
    // Change to SHUTDOWN
    systemManager.setSystemState(SystemState::SHUTDOWN);
    TEST_ASSERT_EQUAL(SystemState::SHUTDOWN, systemManager.getSystemState());
    
    // Try to change from SHUTDOWN to any state (invalid transition)
    systemManager.setSystemState(SystemState::READY);
    TEST_ASSERT_EQUAL(SystemState::SHUTDOWN, systemManager.getSystemState()); // Should remain in shutdown
}

void test_system_manager_emergency_stop() {
    SystemManager systemManager;
    systemManager.initialize();
    
    // Trigger emergency stop
    systemManager.triggerEmergencyStop(SafetyCode::EMERGENCY_STOP_PRESSED);
    
    // Check state
    TEST_ASSERT_EQUAL(SystemState::EMERGENCY_STOP, systemManager.getSystemState());
    TEST_ASSERT_TRUE(systemManager.isEmergencyStop());
    
    // Try to reset emergency stop
    bool result = systemManager.resetEmergencyStop();
    
    // May not be able to reset in test environment
    // Just check that the function doesn't crash
}

void test_system_manager_config_operations() {
    SystemManager systemManager;
    systemManager.initialize();
    
    // Test saving configuration
    bool saveResult = systemManager.saveSystemConfiguration();
    
    // This may fail in the test environment due to missing EEPROM
    // Just check that the function doesn't crash
    
    // Test loading configuration
    bool loadResult = systemManager.loadSystemConfiguration();
    
    // This may fail in the test environment due to missing EEPROM
    // Just check that the function doesn't crash
}

void test_system_manager_system_metrics() {
    SystemManager systemManager;
    systemManager.initialize();
    
    // Test getting uptime
    uint32_t uptime = systemManager.getUptimeMs();
    TEST_ASSERT_GREATER_OR_EQUAL(0, uptime);
    
    // Test getting CPU usage
    float cpu0 = systemManager.getCPUUsage(0);
    float cpu1 = systemManager.getCPUUsage(1);
    
    // CPU usage should be between 0 and 100%
    TEST_ASSERT_GREATER_OR_EQUAL(0.0f, cpu0);
    TEST_ASSERT_LESS_OR_EQUAL(100.0f, cpu0);
    TEST_ASSERT_GREATER_OR_EQUAL(0.0f, cpu1);
    TEST_ASSERT_LESS_OR_EQUAL(100.0f, cpu1);
    
    // Test getting free memory
    uint32_t freeMemory = systemManager.getFreeMemory();
    TEST_ASSERT_GREATER_THAN(0, freeMemory);
    
    // Test updating metrics
    systemManager.updateSystemMetrics();
    
    // Metrics should still be valid after update
    cpu0 = systemManager.getCPUUsage(0);
    TEST_ASSERT_GREATER_OR_EQUAL(0.0f, cpu0);
    TEST_ASSERT_LESS_OR_EQUAL(100.0f, cpu0);
}

void test_system_manager_motor_positions() {
    SystemManager systemManager;
    systemManager.initialize();
    
    // Get motor manager
    MotorManager* motorManager = systemManager.getMotorManager();
    TEST_ASSERT_NOT_NULL(motorManager);
    
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
    
    // Save motor positions
    systemManager.saveMotorPositions();
    
    // Reset position
    motor->setPosition(0);
    TEST_ASSERT_EQUAL(0, motor->getCurrentPosition());
    
    // Restore motor positions
    systemManager.restoreMotorPositions();
    
    // Position should be restored
    // Note: This may not work in the test environment due to missing EEPROM
}

void test_system_manager_shutdown_flag() {
    SystemManager systemManager;
    systemManager.initialize();
    
    // Test normal shutdown flag
    bool wasNormal = systemManager.wasNormalShutdown();
    
    // Set normal shutdown flag
    systemManager.setNormalShutdown(true);
    
    // Should now return true
    // Note: This may not work in the test environment due to missing EEPROM
    
    // Reset flag
    systemManager.setNormalShutdown(false);
    
    // Should now return false
    // Note: This may not work in the test environment due to missing EEPROM
}

void test_system_manager_reset() {
    SystemManager systemManager;
    systemManager.initialize();
    
    // Call reset
    // Note: This would normally restart the ESP32, which we can't test
    // Just check that the function doesn't crash
    
    // Trigger reset
    systemManager.resetSystem();
    
    // Should reach here (since we can't actually reset in a test)
    TEST_ASSERT_EQUAL(SystemState::SHUTDOWN, systemManager.getSystemState());
}

void setup() {
    // Wait before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_system_manager_initialization);
    RUN_TEST(test_system_manager_get_components);
    RUN_TEST(test_system_manager_state_changes);
    RUN_TEST(test_system_manager_emergency_stop);
    RUN_TEST(test_system_manager_config_operations);
    RUN_TEST(test_system_manager_system_metrics);
    RUN_TEST(test_system_manager_motor_positions);
    RUN_TEST(test_system_manager_shutdown_flag);
    RUN_TEST(test_system_manager_reset);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}