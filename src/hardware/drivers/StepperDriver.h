/*
 * ESP32 High-Precision Motion Control System
 * Stepper Motor Driver
 *
 * Controls stepper motors using step/direction interface with
 * precise timing and microstepping support.
 */

#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <Arduino.h>

#include "../../Configuration.h"
#include "../../utils/Logger.h"
#include "../DriverInterface.h"
#include "../TimerManager.h"

/**
 * Microstepping modes
 */
enum class MicrostepMode {
    FULL_STEP = 1,          // 1 microstep per full step
    HALF_STEP = 2,          // 2 microsteps per full step
    QUARTER_STEP = 4,       // 4 microsteps per full step
    EIGHTH_STEP = 8,        // 8 microsteps per full step
    SIXTEENTH_STEP = 16,    // 16 microsteps per full step
    THIRTYSECOND_STEP = 32  // 32 microsteps per full step
};

/**
 * Stepper motor driver class
 */
class StepperDriver : public DriverInterface {
   public:
    /**
     * Constructor
     *
     * @param stepPin Step pin number
     * @param dirPin Direction pin number
     * @param enablePin Enable pin number
     * @param invertDir Whether to invert direction pin logic
     * @param invertEnable Whether to invert enable pin logic
     * @param logger Pointer to logger instance
     */
    StepperDriver(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, bool invertDir = false,
                  bool invertEnable = false, Logger* logger = nullptr);

    /**
     * Destructor
     */
    virtual ~StepperDriver();

    /**
     * Initialize the driver
     *
     * @return True if initialization successful, false otherwise
     */
    bool initialize() override;

    /**
     * Enable the motor driver
     */
    void enable() override;

    /**
     * Disable the motor driver
     */
    void disable() override;

    /**
     * Check if driver is enabled
     *
     * @return True if enabled, false otherwise
     */
    bool isEnabled() const override;

    /**
     * Set motor direction
     *
     * @param direction Direction (true = forward, false = reverse)
     */
    void setDirection(bool direction) override;

    /**
     * Set motor speed/velocity
     *
     * @param speed Speed in steps per second
     */
    void setSpeed(float speed) override;

    /**
     * Step the motor
     *
     * @param steps Number of steps to move (positive or negative)
     * @return Actual number of steps moved
     */
    int32_t step(int32_t steps) override;

    /**
     * Move to absolute position
     *
     * @param position Target position
     * @return True if command accepted, false otherwise
     */
    bool moveTo(int32_t position) override;

    /**
     * Set control output directly
     * This maps to speed for stepper drivers
     *
     * @param output Control output value (-1.0 to 1.0)
     */
    void setOutput(float output) override;

    /**
     * Stop the motor
     *
     * @param emergency Whether to perform emergency stop
     */
    void stop(bool emergency = false) override;

    /**
     * Check if motor is moving
     *
     * @return True if moving, false otherwise
     */
    bool isMoving() const override;

    /**
     * Get driver type
     *
     * @return Driver type
     */
    DriverType getType() const override;

    /**
     * Get current position
     *
     * @return Current position in steps
     */
    int32_t getCurrentPosition() const override;

    /**
     * Set current position
     *
     * @param position New position value
     * @return True if successful, false otherwise
     */
    bool setCurrentPosition(int32_t position) override;

    /**
     * Check for driver faults
     *
     * @return True if fault detected, false otherwise
     */
    bool checkFault() override;

    /**
     * Clear driver fault
     *
     * @return True if fault cleared, false otherwise
     */
    bool clearFault() override;

    /**
     * Get driver configuration
     *
     * @return Driver-specific configuration string
     */
    String getConfig() const override;

    /**
     * Set driver configuration
     *
     * @param config Driver-specific configuration string
     * @return True if configuration applied, false otherwise
     */
    bool setConfig(const String& config) override;

    /**
     * Set microstepping mode
     *
     * @param mode Microstepping mode
     * @return True if successful, false otherwise
     */
    bool setMicrostepMode(MicrostepMode mode);

    /**
     * Get microstepping mode
     *
     * @return Current microstepping mode
     */
    MicrostepMode getMicrostepMode() const;

    /**
     * Set maximum steps per second
     *
     * @param maxStepsPerSecond Maximum steps per second
     */
    void setMaxStepsPerSecond(float maxStepsPerSecond);

    /**
     * Get maximum steps per second
     *
     * @return Maximum steps per second
     */
    float getMaxStepsPerSecond() const;

    /**
     * Timer interrupt handler
     * This is called by the timer ISR
     */
    void handleTimerInterrupt();

   private:
    // Pin configuration
    uint8_t m_stepPin;
    uint8_t m_dirPin;
    uint8_t m_enablePin;
    bool m_invertDir;
    bool m_invertEnable;

    // Driver state
    bool m_enabled;
    bool m_direction;          // Current direction (true = forward, false = reverse)
    float m_speed;             // Current speed in steps per second
    int32_t m_position;        // Current position in steps
    int32_t m_targetPosition;  // Target position in steps
    bool m_isMoving;           // Whether motor is currently moving

    // Microstepping
    MicrostepMode m_microstepMode;
    uint8_t m_microsteps;  // Number of microsteps per full step

    // Speed control
    float m_maxStepsPerSecond;
    uint32_t m_stepIntervalUs;     // Interval between steps in microseconds
    uint32_t m_lastStepTimeUs;     // Time of last step in microseconds
    volatile int32_t m_stepsToGo;  // Steps remaining to move

    // Timer control
    TimerManager* m_timerManager;
    bool m_usingTimer;

    // Logger instance
    Logger* m_logger;

    /**
     * Generate a step pulse
     */
    void pulseStep();

    /**
     * Calculate step interval based on speed
     *
     * @param speed Speed in steps per second
     * @return Step interval in microseconds
     */
    uint32_t calculateStepInterval(float speed);

    /**
     * Start the timer for step generation
     *
     * @param intervalUs Interval between steps in microseconds
     * @return True if timer started successfully, false otherwise
     */
    bool startTimer(uint32_t intervalUs);

    /**
     * Stop the timer
     */
    void stopTimer();

    /**
     * Convert microstepping mode to string for logging
     *
     * @param mode Microstepping mode
     * @return String representation of mode
     */
    String microstepModeToString(MicrostepMode mode) const;
};

#endif  // STEPPER_DRIVER_H