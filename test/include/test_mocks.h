#ifndef TEST_MOCKS_H
#define TEST_MOCKS_H

#include <Arduino.h>
#include "core/Motor.h"
#include "core/SafetyMonitor.h"
#include "utils/Logger.h"
#include "hardware/drivers/StepperDriver.h"

// Mock Logger for testing
class MockLogger : public Logger {
public:
    MockLogger() : m_errorCount(0), m_warningCount(0), m_infoCount(0) {}
    
    void logError(const String& message, uint8_t source = 0) override {
        m_lastError = message;
        m_errorCount++;
    }
    
    void logWarning(const String& message, uint8_t source = 0) override {
        m_lastWarning = message;
        m_warningCount++;
    }
    
    void logInfo(const String& message, uint8_t source = 0) override {
        m_lastInfo = message;
        m_infoCount++;
    }
    
    void logDebug(const String& message, uint8_t source = 0) override {
        // Add debug logging for completeness
        m_lastDebug = message;
        m_debugCount++;
    }
    
    void logVerbose(const String& message, uint8_t source = 0) override {
        // Add verbose logging for completeness
        m_lastVerbose = message;
        m_verboseCount++;
    }
    
    bool initialize(uint32_t serialBaudRate = 0) override {
        return true;
    }
    
    int getErrorCount() const { return m_errorCount; }
    int getWarningCount() const { return m_warningCount; }
    int getInfoCount() const { return m_infoCount; }
    int getDebugCount() const { return m_debugCount; }
    int getVerboseCount() const { return m_verboseCount; }
    
    String getLastError() const { return m_lastError; }
    String getLastWarning() const { return m_lastWarning; }
    String getLastInfo() const { return m_lastInfo; }
    String getLastDebug() const { return m_lastDebug; }
    String getLastVerbose() const { return m_lastVerbose; }
    
    void reset() {
        m_errorCount = 0;
        m_warningCount = 0;
        m_infoCount = 0;
        m_debugCount = 0;
        m_verboseCount = 0;
        m_lastError = "";
        m_lastWarning = "";
        m_lastInfo = "";
        m_lastDebug = "";
        m_lastVerbose = "";
    }
    
private:
    int m_errorCount;
    int m_warningCount;
    int m_infoCount;
    int m_debugCount;
    int m_verboseCount;
    String m_lastError;
    String m_lastWarning;
    String m_lastInfo;
    String m_lastDebug;
    String m_lastVerbose;
};

// Mock Stepper Driver for testing Motor class
class MockStepperDriver : public DriverInterface {
public:
    MockStepperDriver() 
        : m_enabled(false), m_direction(true), m_position(0), 
          m_moving(false), m_output(0.0f), m_fault(false), m_speed(0.0f) {}
    
    bool initialize() override { return true; }
    void enable() override { m_enabled = true; }
    void disable() override { m_enabled = false; }
    bool isEnabled() const override { return m_enabled; }
    void setDirection(bool direction) override { m_direction = direction; }
    void setSpeed(float speed) override { m_speed = speed; }
    
    int32_t step(int32_t steps) override { 
        if (steps != 0) {
            m_position += m_direction ? steps : -steps;
            m_moving = true;
        }
        return steps;
    }
    
    bool moveTo(int32_t position) override { 
        m_moving = position != m_position;
        m_position = position;
        return true;
    }
    
    void setOutput(float output) override { 
        m_output = output;
        // Simulate movement based on output
        if (fabs(output) > 0.01f) {
            m_moving = true;
            // In a real test we would update position based on output
        } else {
            m_moving = false;
        }
    }
    
    void stop(bool emergency = false) override { m_moving = false; }
    bool isMoving() const override { return m_moving; }
    DriverType getType() const override { return DriverType::STEPPER; }
    int32_t getCurrentPosition() const override { return m_position; }
    
    bool setCurrentPosition(int32_t position) override { 
        m_position = position;
        return true;
    }
    
    bool checkFault() override { return m_fault; }
    bool clearFault() override { m_fault = false; return true; }
    String getConfig() const override { return "{}"; }
    bool setConfig(const String& config) override { return true; }
    
    // Test helpers
    void setFault(bool fault) { m_fault = fault; }
    float getOutput() const { return m_output; }
    float getSpeed() const { return m_speed; }
    
    void simulateMovement(int32_t amount) { 
        m_position += amount; 
        if (amount != 0) m_moving = true;
    }
    
    void simulateStep() {
        if (m_moving) {
            m_position += m_direction ? 1 : -1;
        }
    }
    
private:
    bool m_enabled;
    bool m_direction;
    int32_t m_position;
    bool m_moving;
    float m_output;
    bool m_fault;
    float m_speed;
};

// Mock Encoder for testing
class MockEncoder : public Encoder {
public:
    MockEncoder() : Encoder(0xFF, 0xFF), // Use invalid pins to prevent actual hardware access
                   m_position(0), m_velocity(0.0f) {}
    
    bool initialize() override { return true; }
    
    void setPosition(int32_t position) override {
        m_position = position;
        Encoder::setPosition(position); // Call base class implementation
    }
    
    void setVelocity(float velocity) override {
        m_velocity = velocity;
        Encoder::setVelocity(velocity); // Call base class implementation
    }
    
    int32_t getPosition() const override {
        return m_position;
    }
    
    float getVelocity() const override {
        return m_velocity;
    }
    
    void update(uint32_t deltaTimeUs) override {
        // Mock implementation that just updates the base class
        Encoder::update(deltaTimeUs);
    }
    
    void simulateMovement(int32_t steps, float velocity) {
        m_position += steps;
        m_velocity = velocity;
    }
    
private:
    int32_t m_position;
    float m_velocity;
};

// Mock EEPROM for testing
class MockEEPROM {
public:
    MockEEPROM(size_t size = 4096) : m_size(size) {
        m_data = new uint8_t[size];
        memset(m_data, 0, size);
    }
    
    ~MockEEPROM() {
        delete[] m_data;
    }
    
    uint8_t read(size_t address) {
        if (address < m_size) {
            return m_data[address];
        }
        return 0;
    }
    
    void write(size_t address, uint8_t value) {
        if (address < m_size) {
            m_data[address] = value;
        }
    }
    
    void get(size_t address, void* data, size_t length) {
        if (address + length <= m_size) {
            memcpy(data, &m_data[address], length);
        }
    }
    
    void put(size_t address, const void* data, size_t length) {
        if (address + length <= m_size) {
            memcpy(&m_data[address], data, length);
        }
    }
    
    size_t length() const {
        return m_size;
    }
    
    bool begin(size_t size) {
        if (size != m_size) {
            delete[] m_data;
            m_data = new uint8_t[size];
            m_size = size;
        }
        memset(m_data, 0, m_size);
        return true;
    }
    
    bool commit() {
        return true;
    }
    
private:
    uint8_t* m_data;
    size_t m_size;
};

#endif // TEST_MOCKS_H