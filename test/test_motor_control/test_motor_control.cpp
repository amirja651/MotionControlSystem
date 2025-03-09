#include <unity.h>
#include <Arduino.h>
#include "core/Motor.h"
#include "Configuration.h"

// Create a mock driver for testing Motor class
class MockStepperDriver : public DriverInterface {
public:
    MockStepperDriver() 
        : m_enabled(false), m_direction(true), m_position(0), m_moving(false), 
          m_output(0.0f), m_fault(false) {}
    
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
            // In a real test we would keep track of position/velocity changes
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
    void simulateMovement(int32_t amount) { 
        m_position += amount; 
        if (amount != 0) m_moving = true;
    }
    
private:
    bool m_enabled;
    bool m_direction;
    int32_t m_position;
    float m_speed;
    bool m_moving;
    float m_output;
    bool m_fault;
};

// Global mock driver reference for use in the tests
MockStepperDriver* g_mockDriver = nullptr;

// Test override to create our mock driver
DriverInterface* createTestDriver() {
    if (g_mockDriver == nullptr) {
        g_mockDriver = new MockStepperDriver();
    }
    return g_mockDriver;
}

// Test cases for Motor control

void test_motor_initialization() {
    // Create motor configuration
    MotorConfig config;
    config.index = 0;
    config.stepPin = 5;
    config.dirPin = 6;
    config.enablePin = 7;
    config.maxVelocity = 1000.0f;
    config.maxAcceleration = 2000.0f;
    
    // Create motor with this config
    Motor motor(config);
    
    // Initialize motor
    bool result = motor.initialize();
    
    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL(0, motor.getIndex());
    TEST_ASSERT_FALSE(motor.isEnabled());
    TEST_ASSERT_FALSE(motor.isMoving());
}

void test_motor_enable_disable() {
    // Create motor configuration
    MotorConfig config;
    config.index = 1;
    config.stepPin = 10;
    config.dirPin = 11;
    config.enablePin = 12;
    
    // Create motor with this config
    Motor motor(config);
    motor.initialize();
    
    // Test enable/disable
    motor.enable();
    TEST_ASSERT_TRUE(motor.isEnabled());
    
    motor.disable();
    TEST_ASSERT_FALSE(motor.isEnabled());
}

void test_motor_position_control() {
    // Create motor configuration
    MotorConfig config;
    config.index = 2;
    config.stepPin = 15;
    config.dirPin = 16;
    config.enablePin = 17;
    config.maxVelocity = 1000.0f;
    config.maxAcceleration = 2000.0f;
    
    // Create motor with this config
    Motor motor(config);
    motor.initialize();
    motor.enable();
    
    // Set initial position
    motor.setPosition(0);
    TEST_ASSERT_EQUAL(0, motor.getCurrentPosition());
    
    // Set target position
    int32_t targetPos = 1000;
    motor.setTargetPosition(targetPos);
    
    // Target should be set
    TEST_ASSERT_EQUAL(targetPos, motor.getTargetPosition());
    TEST_ASSERT_EQUAL(MotorControlMode::POSITION, motor.getControlMode());
    
    // Simulate some movement
    if (g_mockDriver) {
        g_mockDriver->simulateMovement(500);
    }
    
    // Check if it's still moving
    TEST_ASSERT_TRUE(motor.isMoving());
    
    // Simulate reaching the target
    if (g_mockDriver) {
        g_mockDriver->simulateMovement(500);  // Now at position 1000
        g_mockDriver->setOutput(0.0f);  // Stop moving
    }
    
    // Should no longer be moving after reaching target
    TEST_ASSERT_EQUAL(targetPos, motor.getCurrentPosition());
    // In a real test, we'd have to call updateControl() to see motor.isMoving() become false
}

void test_motor_velocity_control() {
    // Create motor configuration
    MotorConfig config;
    config.index = 3;
    config.stepPin = 20;
    config.dirPin = 21;
    config.enablePin = 22;
    config.maxVelocity = 1000.0f;
    config.maxAcceleration = 2000.0f;
    
    // Create motor with this config
    Motor motor(config);
    motor.initialize();
    motor.enable();
    
    // Set target velocity
    float targetVel = 500.0f;
    motor.setTargetVelocity(targetVel);
    
    // Check mode and target
    TEST_ASSERT_EQUAL(MotorControlMode::VELOCITY, motor.getControlMode());
    TEST_ASSERT_EQUAL_FLOAT(targetVel, motor.getTargetVelocity());
    
    // Simulate movement
    if (g_mockDriver) {
        // In a real test, we would have the driver simulate actual velocity
        g_mockDriver->simulateMovement(50);
    }
    
    // Check movement
    TEST_ASSERT_TRUE(motor.isMoving());
    
    // Test stopping
    motor.setTargetVelocity(0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, motor.getTargetVelocity());
}

void test_motor_emergency_stop() {
    // Create motor configuration
    MotorConfig config;
    config.index = 4;
    config.stepPin = 25;
    config.dirPin = 26;
    config.enablePin = 27;
    
    // Create motor with this config
    Motor motor(config);
    motor.initialize();
    motor.enable();
    
    // Start moving
    motor.setTargetVelocity(100.0f);
    
    // Trigger emergency stop
    motor.emergencyStop();
    
    // Verify motion has stopped
    if (g_mockDriver) {
        TEST_ASSERT_FALSE(g_mockDriver->isMoving());
    }
}

void test_motor_soft_limits() {
    // Create motor configuration
    MotorConfig config;
    config.index = 5;
    config.stepPin = 30;
    config.dirPin = 31;
    config.enablePin = 32;
    config.maxVelocity = 1000.0f;
    
    // Create motor with this config
    Motor motor(config);
    motor.initialize();
    motor.enable();
    
    // Set soft limits
    int32_t minLimit = -500;
    int32_t maxLimit = 500;
    motor.setSoftLimits(minLimit, maxLimit, true);
    
    // Set initial position
    motor.setPosition(0);
    
    // Try to move beyond limits
    motor.setTargetPosition(600);  // Beyond max limit
    
    // Position command should be ignored or limited
    TEST_ASSERT_NOT_EQUAL(600, motor.getTargetPosition());
    
    // Try valid movement
    motor.setTargetPosition(400);
    TEST_ASSERT_EQUAL(400, motor.getTargetPosition());
}

void test_motor_pid_control() {
    // Create motor configuration
    MotorConfig config;
    config.index = 6;
    config.stepPin = 35;
    config.dirPin = 36;
    config.enablePin = 37;
    config.pidKp = 1.0f;
    config.pidKi = 0.1f;
    config.pidKd = 0.05f;
    
    // Create motor with this config
    Motor motor(config);
    motor.initialize();
    
    // Get PID controller and test parameters
    PIDController& pid = motor.getPIDController();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.getProportionalTerm());
    TEST_ASSERT_EQUAL_FLOAT(0.1f, pid.getIntegralTerm());
    TEST_ASSERT_EQUAL_FLOAT(0.05f, pid.getDerivativeTerm());
    
    // Test setting new PID parameters
    motor.setPIDParameters(2.0f, 0.2f, 0.1f, 0.5f);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, pid.getProportionalTerm());
    TEST_ASSERT_EQUAL_FLOAT(0.2f, pid.getIntegralTerm());
    TEST_ASSERT_EQUAL_FLOAT(0.1f, pid.getDerivativeTerm());
    TEST_ASSERT_EQUAL_FLOAT(0.5f, pid.getFeedForwardTerm());
}

void setup() {
    // Wait 2 seconds before starting tests
    delay(2000);
    
    // Create the global mock driver
    g_mockDriver = new MockStepperDriver();
    
    UNITY_BEGIN();
    
    RUN_TEST(test_motor_initialization);
    RUN_TEST(test_motor_enable_disable);
    RUN_TEST(test_motor_position_control);
    RUN_TEST(test_motor_velocity_control);
    RUN_TEST(test_motor_emergency_stop);
    RUN_TEST(test_motor_soft_limits);
    RUN_TEST(test_motor_pid_control);
    
    UNITY_END();
    
    // Clean up
    delete g_mockDriver;
    g_mockDriver = nullptr;
}

void loop() {
    // Nothing to do here
}