/*
 * ESP32 High-Precision Motion Control System
 * Motor Class
 *
 * Provides a high-level abstraction for motor control with position and velocity
 * control modes, utilizing closed-loop feedback from encoders.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#include "../Configuration.h"
#include "../hardware/DriverInterface.h"
#include "../hardware/GPIOManager.h"
#include "../hardware/TimerManager.h"
#include "../hardware/drivers/StepperDriver.h"
#include "../utils/CircularBuffer.h"
#include "../utils/Logger.h"
#include "Encoder.h"
#include "PIDController.h"
#include "TrajectoryPlanner.h"

/**
 * Motor control modes
 */
enum class MotorControlMode {
    DISABLED_,  // Motor disabled
    OPEN_LOOP,  // Open-loop control (no feedback)
    POSITION,   // Closed-loop position control
    VELOCITY,   // Closed-loop velocity control
    TORQUE,     // Closed-loop torque control (if supported)
    HOMING      // Homing sequence
};

/**
 * Motor status codes
 */
enum class MotorStatus {
    IDLE,      // Motor idle
    MOVING,    // Motor moving
    HOMING,    // Homing in progress
    HOLDING,   // Holding position
    ERROR,     // Error condition
    DISABLED_  // Motor disabled
};

/**
 * Motor error codes
 */
enum class MotorError {
    NONE,                    // No error
    POSITION_ERROR,          // Position error exceeded limit
    VELOCITY_ERROR,          // Velocity error exceeded limit
    LIMIT_SWITCH_TRIGGERED,  // Limit switch triggered unexpectedly
    DRIVER_FAULT,            // Motor driver reported a fault
    ENCODER_ERROR,           // Encoder reading error
    TIMEOUT,                 // Operation timed out
    TRAJECTORY_ERROR,        // Trajectory calculation error
    GENERAL_ERROR            // Unspecified error
};

/**
 * Motor configuration structure
 */
struct MotorState {
    int32_t targetPosition;     // Target position in encoder counts
    int32_t currentPosition;    // Current position in encoder counts
    float targetVelocity;       // Target velocity in counts/s
    float currentVelocity;      // Current velocity in counts/s
    float targetAcceleration;   // Target acceleration in counts/s²
    float currentAcceleration;  // Current acceleration in counts/s²
    float controlOutput;        // Current control output
    MotorStatus status;         // Current motor status
    MotorError error;           // Current error code
    bool limitMinTriggered;     // Min limit switch state
    bool limitMaxTriggered;     // Max limit switch state
    bool isHomed;               // Homing status
    uint32_t lastUpdateTimeUs;  // Last update time
};

/**
 * Motor class - abstracts motor control functionality
 */
class Motor {
   public:
    /**
     * Constructor
     *
     * @param config Motor configuration
     * @param logger Pointer to logger instance
     */
    Motor(const MotorConfig& config, Logger* logger = nullptr);

    /**
     * Initialize the motor
     *
     * @return True if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * Enable the motor
     */
    void enable();

    /**
     * Disable the motor
     */
    void disable();

    /**
     * Check if motor is enabled
     *
     * @return True if enabled, false otherwise
     */
    bool isEnabled() const;

    /**
     * Update the control loop
     * Should be called at fixed intervals for consistent control
     */
    void updateControl();

    /**
     * Update trajectory planner
     * Can be called at a lower frequency than updateControl
     */
    void updateTrajectory();

    /**
     * Set control mode
     *
     * @param mode Control mode
     */
    void setControlMode(MotorControlMode mode);

    /**
     * Get current control mode
     *
     * @return Current control mode
     */
    MotorControlMode getControlMode() const;

    /**
     * Set target position
     *
     * @param position Target position in encoder counts
     */
    void setTargetPosition(int32_t position);

    /**
     * Set target position with motion profile parameters
     *
     * @param position Target position in encoder counts
     * @param maxVelocity Maximum velocity for the move
     * @param acceleration Acceleration for the move
     * @param deceleration Deceleration for the move
     * @param jerk Jerk limit (if supported)
     */
    void moveToPosition(int32_t position, float maxVelocity, float acceleration, float deceleration,
                        float jerk = 0.0f);

    /**
     * Set target velocity
     *
     * @param velocity Target velocity in encoder counts per second
     */
    void setTargetVelocity(float velocity);

    /**
     * Set target velocity with acceleration limit
     *
     * @param velocity Target velocity in encoder counts per second
     * @param acceleration Acceleration limit
     * @param jerk Jerk limit (if supported)
     */
    void setTargetVelocityWithAccel(float velocity, float acceleration, float jerk = 0.0f);

    /**
     * Emergency stop
     * Immediately stops the motor with maximum deceleration
     */
    void emergencyStop();

    /**
     * Start homing sequence
     *
     * @param direction Direction to move during homing (1 = positive, -1 = negative)
     * @param velocity Homing velocity
     */
    void startHoming(int8_t direction, float velocity);

    /**
     * Abort current operation
     * Gracefully stops the current motion
     */
    void abort();

    /**
     * Get current motor state
     *
     * @return Motor state structure
     */
    const MotorState& getState() const;

    /**
     * Get motor index
     *
     * @return Motor index
     */
    uint8_t getIndex() const;

    /**
     * Get current position
     *
     * @return Current position in encoder counts
     */
    int32_t getCurrentPosition() const;

    /**
     * Get current velocity
     *
     * @return Current velocity in encoder counts per second
     */
    float getCurrentVelocity() const;

    /**
     * Get target position
     *
     * @return Target position in encoder counts
     */
    int32_t getTargetPosition() const;

    /**
     * Get target velocity
     *
     * @return Target velocity in encoder counts per second
     */
    float getTargetVelocity() const;

    /**
     * Check if motor is moving
     *
     * @return True if motor is moving, false otherwise
     */
    bool isMoving() const;

    /**
     * Check if position is reached
     *
     * @return True if position reached, false otherwise
     */
    bool isPositionReached() const;

    /**
     * Check if target velocity is reached
     *
     * @return True if velocity reached, false otherwise
     */
    bool isVelocityReached() const;

    /**
     * Check if homing is complete
     *
     * @return True if homed, false otherwise
     */
    bool isHomed() const;

    /**
     * Reset position to zero
     * Typically used after homing
     */
    void resetPosition();

    /**
     * Set position
     *
     * @param position New position value
     */
    void setPosition(int32_t position);

    /**
     * Set PID parameters
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param ff Feed-forward gain
     */
    void setPIDParameters(float kp, float ki, float kd, float ff);

    /**
     * Get PID controller
     *
     * @return Reference to PID controller
     */
    PIDController& getPIDController();

    /**
     * Get encoder
     *
     * @return Reference to encoder
     */
    Encoder& getEncoder();

    /**
     * Set soft limits
     *
     * @param min Minimum position limit
     * @param max Maximum position limit
     * @param enable Whether to enable soft limits
     */
    void setSoftLimits(int32_t min, int32_t max, bool enable = true);

    /**
     * Set hard limit switch configuration
     *
     * @param invertMin Whether to invert min limit logic
     * @param invertMax Whether to invert max limit logic
     * @param enable Whether to enable limit switches
     */
    void setLimitSwitchConfig(bool invertMin, bool invertMax, bool enable = true);

    /**
     * Get control interval in microseconds
     *
     * @return Control interval
     */
    uint32_t getControlInterval() const;

    /**
     * Get trajectory update interval in microseconds
     *
     * @return Trajectory update interval
     */
    uint32_t getTrajectoryUpdateInterval() const;

    /**
     * Clear error state
     */
    void clearError();

    /**
     * Check for errors
     *
     * @return Current error code
     */
    MotorError checkErrors();

   private:
    // Motor configuration
    MotorConfig m_config;

    bool m_invertEnable;  // Flag to invert enable pin logic

    // Motor components
    Encoder m_encoder;
    PIDController m_controller;
    TrajectoryPlanner m_trajectoryPlanner;
    DriverInterface* m_driver;

    /**
     * Logger instance
     */
    Logger* m_logger;

    // Motor state
    MotorState m_state;
    MotorControlMode m_controlMode;

    // Control parameters
    uint32_t m_controlIntervalUs;
    uint32_t m_trajectoryIntervalUs;
    float m_positionTolerance;
    float m_velocityTolerance;

    // Safety parameters
    bool m_softLimitsEnabled;
    int32_t m_softLimitMin;
    int32_t m_softLimitMax;
    bool m_limitSwitchesEnabled;
    bool m_invertLimitMin;
    bool m_invertLimitMax;
    bool m_emergencyStopActive;

    // Pin states
    bool m_limitMinState;
    bool m_limitMaxState;

    // Additional state tracking
    uint32_t m_lastControlUpdateUs;
    uint32_t m_lastTrajectoryUpdateUs;
    CircularBuffer<float, 8> m_controlOutputBuffer;

    /**
     * Read limit switch states
     *
     * @return True if a limit switch is triggered, false otherwise
     */
    bool readLimitSwitches();

    /**
     * Check if position is within soft limits
     *
     * @param position Position to check
     * @return True if within limits, false otherwise
     */
    bool isWithinSoftLimits(int32_t position) const;

    /**
     * Apply control output to motor
     *
     * @param output Control output value
     */
    void applyControlOutput(float output);

    /**
     * Process position control mode
     */
    void processPositionControl();

    /**
     * Process velocity control mode
     */
    void processVelocityControl();

    /**
     * Process homing sequence
     */
    void processHoming();

    /**
     * Process motor state transitions
     */
    void updateMotorState();

    /**
     * Set error state
     *
     * @param error Error code
     */
    void setError(MotorError error);

    /**
     * Convert control mode to string
     *
     * @param mode Control mode
     * @return String representation
     */
    String controlModeToString(MotorControlMode mode) const;

    /**
     * Convert motor status to string
     *
     * @param status Motor status
     * @return String representation
     */
    String motorStatusToString(MotorStatus status) const;

    /**
     * Convert motor error to string
     *
     * @param error Motor error
     * @return String representation
     */
    String motorErrorToString(MotorError error) const;
};
#endif  // MOTOR_H