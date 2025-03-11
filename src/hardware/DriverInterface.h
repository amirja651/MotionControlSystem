/*
 * ESP32 High-Precision Motion Control System
 * Driver Interface
 *
 * Abstract interface for motor drivers, providing a common API
 * for different motor types (stepper, DC, servo, etc.).
 */

#ifndef DRIVER_INTERFACE_H
#define DRIVER_INTERFACE_H

#include <Arduino.h>

#include "../Configuration.h"

/**
 * Driver types enumeration
 */
enum class DriverType {
    STEPPER,   // Stepper motor driver
    DC_MOTOR,  // DC motor driver
    SERVO,     // Servo motor driver
    BLDC       // Brushless DC motor driver
};

/**
 * Abstract driver interface class
 */
class DriverInterface {
   public:
    /**
     * Virtual destructor
     */
    virtual ~DriverInterface() {
    }

    /**
     * Initialize the driver
     *
     * @return True if initialization successful, false otherwise
     */
    virtual bool initialize() = 0;

    /**
     * Enable the motor driver
     */
    virtual void enable() = 0;

    /**
     * Disable the motor driver
     */
    virtual void disable() = 0;

    /**
     * Check if driver is enabled
     *
     * @return True if enabled, false otherwise
     */
    virtual bool isEnabled() const = 0;

    /**
     * Set motor direction
     *
     * @param direction Direction (true = forward, false = reverse)
     */
    virtual void setDirection(bool direction) = 0;

    /**
     * Set motor speed/velocity
     *
     * @param speed Speed/velocity value (0.0 - 1.0 or absolute value depending on driver)
     */
    virtual void setSpeed(float speed) = 0;

    /**
     * Step the motor (for stepper motors)
     *
     * @param steps Number of steps to move (positive or negative)
     * @return Actual number of steps moved
     */
    virtual int32_t step(int32_t steps) = 0;

    /**
     * Move to absolute position (if supported)
     *
     * @param position Target position
     * @return True if command accepted, false otherwise
     */
    virtual bool moveTo(int32_t position) = 0;

    /**
     * Set control output directly
     *
     * @param output Control output value (-1.0 to 1.0)
     */
    virtual void setOutput(float output) = 0;

    /**
     * Stop the motor
     *
     * @param emergency Whether to perform emergency stop
     */
    virtual void stop(bool emergency = false) = 0;

    /**
     * Check if motor is moving
     *
     * @return True if moving, false otherwise
     */
    virtual bool isMoving() const = 0;

    /**
     * Get driver type
     *
     * @return Driver type
     */
    virtual DriverType getType() const = 0;

    /**
     * Get current position (if available)
     *
     * @return Current position or 0 if not available
     */
    virtual int32_t getCurrentPosition() const = 0;

    /**
     * Set current position (if supported)
     *
     * @param position New position value
     * @return True if successful, false otherwise
     */
    virtual bool setCurrentPosition(int32_t position) = 0;

    /**
     * Check for driver faults
     *
     * @return True if fault detected, false otherwise
     */
    virtual bool checkFault() = 0;

    /**
     * Clear driver fault
     *
     * @return True if fault cleared, false otherwise
     */
    virtual bool clearFault() = 0;

    /**
     * Get driver configuration
     *
     * @return Driver-specific configuration string
     */
    virtual String getConfig() const = 0;

    /**
     * Set driver configuration
     *
     * @param config Driver-specific configuration string
     * @return True if configuration applied, false otherwise
     */
    virtual bool setConfig(const String& config) = 0;
};

#endif  // DRIVER_INTERFACE_H
        // End of Code