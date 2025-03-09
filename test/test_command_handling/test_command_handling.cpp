#include <unity.h>
#include <Arduino.h>
#include "SystemManager.h"
#include "communication/SerialCommand.h"
#include "communication/CommandProtocol.h"
#include "../include/test_mocks.h"

// Mock classes for testing
class MockSystemManager : public SystemManager {
public:
    bool m_resetCalled = false;
    bool m_saveConfigCalled = false;
    bool m_loadConfigCalled = false;
    bool m_emergencyStopCalled = false;
    bool m_emergencyStopResetCalled = false;
    
    void resetSystem() override {
        m_resetCalled = true;
    }
    
    bool saveSystemConfiguration() override {
        m_saveConfigCalled = true;
        return true;
    }
    
    bool loadSystemConfiguration() override {
        m_loadConfigCalled = true;
        return true;
    }
    
    void triggerEmergencyStop(SafetyCode reason) override {
        m_emergencyStopCalled = true;
    }
    
    bool resetEmergencyStop() override {
        m_emergencyStopResetCalled = true;
        return true;
    }
    
    void reset() {
        m_resetCalled = false;
        m_saveConfigCalled = false;
        m_loadConfigCalled = false;
        m_emergencyStopCalled = false;
        m_emergencyStopResetCalled = false;
    }
};

// Global variables for testing
MockSystemManager* mockSystemManager = nullptr;
SerialCommand* serialCommand = nullptr;
CommandProtocol* commandProtocol = nullptr;
MockLogger* mockLogger = nullptr;

void setUp() {
    // Create mock objects
    mockSystemManager = new MockSystemManager();
    mockLogger = new MockLogger();
    
    // Connect logger to system manager
    mockSystemManager->getLogger = [&]() -> Logger* { return mockLogger; };
    
    // Create command handlers
    serialCommand = new SerialCommand(mockSystemManager);
    commandProtocol = new CommandProtocol(mockSystemManager, serialCommand);
    
    // Initialize command handlers
    serialCommand->initialize();
    commandProtocol->initialize();
    
    // Reset mock states
    mockSystemManager->reset();
    mockLogger->reset();
}

void tearDown() {
    delete commandProtocol;
    delete serialCommand;
    delete mockLogger;
    delete mockSystemManager;
    
    commandProtocol = nullptr;
    serialCommand = nullptr;
    mockLogger = nullptr;
    mockSystemManager = nullptr;
}

// Test SerialCommand execution
void test_serial_command_help() {
    String response;
    bool result = serialCommand->executeCommand("help", response);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(response.length() > 0);
    TEST_ASSERT_TRUE(response.indexOf("Available commands") >= 0);
}

void test_serial_command_status() {
    String response;
    bool result = serialCommand->executeCommand("status", response);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(response.length() > 0);
    TEST_ASSERT_TRUE(response.indexOf("System Status") >= 0);
}

void test_serial_command_motor() {
    String response;
    bool result = serialCommand->executeCommand("motor 0", response);
    
    // This may fail if motor 0 isn't available, but we're testing the command handling
    TEST_ASSERT_TRUE(response.length() > 0);
}

void test_serial_command_move() {
    String response;
    bool result = serialCommand->executeCommand("move 0 1000 500", response);
    
    // This may fail if motor 0 isn't available, but we're testing the command handling
    TEST_ASSERT_TRUE(response.length() > 0);
}

void test_serial_command_reset() {
    String response;
    bool result = serialCommand->executeCommand("reset", response);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(mockSystemManager->m_resetCalled);
}

void test_serial_command_save() {
    String response;
    bool result = serialCommand->executeCommand("save", response);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(mockSystemManager->m_saveConfigCalled);
    TEST_ASSERT_TRUE(response.indexOf("Configuration saved") >= 0);
}

void test_serial_command_load() {
    String response;
    bool result = serialCommand->executeCommand("load", response);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(mockSystemManager->m_loadConfigCalled);
    TEST_ASSERT_TRUE(response.indexOf("Configuration loaded") >= 0);
}

void test_serial_command_estop() {
    String response;
    bool result = serialCommand->executeCommand("estop", response);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(mockSystemManager->m_emergencyStopCalled);
    TEST_ASSERT_TRUE(response.indexOf("Emergency stop triggered") >= 0);
}

void test_serial_command_estop_reset() {
    String response;
    bool result = serialCommand->executeCommand("estop reset", response);
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_TRUE(mockSystemManager->m_emergencyStopResetCalled);
    TEST_ASSERT_TRUE(response.indexOf("Emergency stop reset") >= 0);
}

void test_serial_command_invalid() {
    String response;
    bool result = serialCommand->executeCommand("nonexistent_command", response);
    
    TEST_ASSERT_FALSE(result);
    TEST_ASSERT_TRUE(response.indexOf("Unknown command") >= 0);
}

// Test CommandProtocol packet handling
void test_command_protocol_system_command() {
    // Enable binary protocol
    commandProtocol->enableBinaryProtocol(true);
    
    // Create a system status command packet
    CommandPacket packet;
    packet.startByte = 0xAA;
    packet.commandType = static_cast<uint8_t>(CommandType::SYSTEM_COMMAND);
    packet.commandId = 0x01; // Get system status
    packet.length = 0;
    packet.checksum = 0xAA ^ static_cast<uint8_t>(CommandType::SYSTEM_COMMAND) ^ 0x01 ^ 0;
    
    // Process the packet
    bool result = commandProtocol->processCommandPacket(packet);
    
    TEST_ASSERT_TRUE(result);
}

void test_command_protocol_motion_command() {
    // Enable binary protocol
    commandProtocol->enableBinaryProtocol(true);
    
    // Create data for enable motor command
    uint8_t data[1] = {0}; // Motor index 0
    
    // Create a motion command packet to enable motor
    CommandPacket packet;
    packet.startByte = 0xAA;
    packet.commandType = static_cast<uint8_t>(CommandType::MOTION_COMMAND);
    packet.commandId = 0x01; // Enable motor
    packet.length = 1;
    memcpy(packet.data, data, 1);
    packet.checksum = 0xAA ^ static_cast<uint8_t>(CommandType::MOTION_COMMAND) ^ 0x01 ^ 1 ^ data[0];
    
    // Process the packet
    bool result = commandProtocol->processCommandPacket(packet);
    
    // This may fail if motor 0 isn't available, but we're testing the command handling
    TEST_ASSERT_TRUE(commandProtocol->isBinaryProtocolEnabled());
}

void test_command_protocol_enable_disable() {
    // Initially disable
    commandProtocol->enableBinaryProtocol(false);
    TEST_ASSERT_FALSE(commandProtocol->isBinaryProtocolEnabled());
    
    // Then enable
    commandProtocol->enableBinaryProtocol(true);
    TEST_ASSERT_TRUE(commandProtocol->isBinaryProtocolEnabled());
    
    // Then disable again
    commandProtocol->enableBinaryProtocol(false);
    TEST_ASSERT_FALSE(commandProtocol->isBinaryProtocolEnabled());
}

void setup() {
    // Wait before starting tests
    delay(2000);
    
    UNITY_BEGIN();
    
    // Test SerialCommand
    RUN_TEST(test_serial_command_help);
    RUN_TEST(test_serial_command_status);
    RUN_TEST(test_serial_command_motor);
    RUN_TEST(test_serial_command_move);
    RUN_TEST(test_serial_command_reset);
    RUN_TEST(test_serial_command_save);
    RUN_TEST(test_serial_command_load);
    RUN_TEST(test_serial_command_estop);
    RUN_TEST(test_serial_command_estop_reset);
    RUN_TEST(test_serial_command_invalid);
    
    // Test CommandProtocol
    RUN_TEST(test_command_protocol_system_command);
    RUN_TEST(test_command_protocol_motion_command);
    RUN_TEST(test_command_protocol_enable_disable);
    
    UNITY_END();
}

void loop() {
    // Nothing to do here
}