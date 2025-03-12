/*
 * ESP32 High-Precision Motion Control System
 * Encoder Class
 *
 * Provides high-resolution position feedback with advanced signal processing,
 * including quadrature decoding, digital filtering, and interpolation.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#include "../Configuration.h"
#include "../utils/CircularBuffer.h"
#include "../utils/Logger.h"
#include "../utils/MathUtils.h"

class Encoder {
   public:
    /**
     * Constructor
     *
     * @param encAPin Encoder A pin
     * @param encBPin Encoder B pin
     * @param pulsesPerRev Encoder pulses per revolution
     * @param invertDirection Whether to invert the direction
     * @param logger Pointer to logger instance
     */
    Encoder(uint8_t  encAPin,
            uint8_t  encBPin,
            uint16_t pulsesPerRev    = CONFIG_ENCODER_DEFAULT_PPR,
            bool     invertDirection = false,
            Logger*  logger          = nullptr);

    /**
     * Initialize the encoder
     *
     * @return True if initialization successful, false otherwise
     */
    virtual bool initialize();

    /**
     * Reset encoder position to zero
     */
    void reset();

    /**
     * Set encoder position to a specific value
     *
     * @param position New position value
     */
    virtual void setPosition(int32_t position);

    /**
     * Set encoder velocity to a specific value
     * Used for manual override or initialization
     *
     * @param velocity New velocity value
     */
    virtual void setVelocity(float velocity);

    /**
     * Get current encoder position
     *
     * @return Current position in encoder counts
     */
    virtual int32_t getPosition() const;

    /**
     * Get filtered encoder position
     *
     * @return Filtered position
     */
    float getFilteredPosition() const;

    /**
     * Get current encoder velocity
     *
     * @return Current velocity in counts per second
     */
    virtual float getVelocity() const;

    /**
     * Get filtered encoder velocity
     *
     * @return Filtered velocity
     */
    float getFilteredVelocity() const;

    /**
     * Get encoder acceleration
     *
     * @return Current acceleration in counts per second squared
     */
    float getAcceleration() const;

    /**
     * Set the interpolation factor
     *
     * @param factor Interpolation factor (1, 2, 4, or 8)
     */
    void setInterpolationFactor(uint8_t factor);

    /**
     * Set the velocity filter time constant
     *
     * @param alpha Filter coefficient (0.0 - 1.0, higher = less filtering)
     */
    void setVelocityFilterAlpha(float alpha);

    /**
     * Set the position filter time constant
     *
     * @param alpha Filter coefficient (0.0 - 1.0, higher = less filtering)
     */
    void setPositionFilterAlpha(float alpha);

    /**
     * Update encoder position and velocity
     * Must be called at regular intervals for accurate velocity calculation
     *
     * @param deltaTimeUs Time since last update in microseconds
     */
    virtual void update(uint32_t deltaTimeUs);

    /**
     * Process encoder A/B pin state change
     * Called from interrupt handler
     *
     * @param pinA State of encoder A pin
     * @param pinB State of encoder B pin
     */
    void processInterrupt(bool pinA, bool pinB);

    /**
     * Get encoder resolution in counts per revolution
     * Includes interpolation factor
     *
     * @return Counts per revolution
     */
    uint32_t getCountsPerRevolution() const;

   private:
    // Logger instance
    Logger* m_logger;

    // Pin configuration
    uint8_t m_encoderAPin;
    uint8_t m_encoderBPin;

    // Encoder specifications
    uint16_t m_pulsesPerRev;
    uint8_t  m_interpolationFactor;
    bool     m_invertDirection;

    // Current state
    volatile int32_t  m_position;
    volatile uint8_t  m_stateAB;      // Current A/B state (bits 0-1)
    volatile uint32_t m_transitions;  // Total transitions count for diagnostics

    // Position and velocity tracking
    float m_filteredPosition;
    float m_velocity;  // Counts per second
    float m_filteredVelocity;
    float m_acceleration;  // Counts per second squared

    // Timing
    uint32_t m_lastUpdateTimeUs;
    uint32_t m_timeSinceLastTransitionUs;
    uint32_t m_deltaTimeUs;  // Time since last update

    // Filter parameters
    float m_velocityFilterAlpha;
    float m_positionFilterAlpha;

    // Circular buffers for filtering
    CircularBuffer<int32_t, CONFIG_ENCODER_BUFFER_SIZE> m_positionBuffer;
    CircularBuffer<float, CONFIG_ENCODER_BUFFER_SIZE>   m_velocityBuffer;

    // Direction state for velocity calculation
    int8_t m_direction;

    /**
     * Calculate the encoder velocity based on position change and time
     *
     * @param positionChange Change in position
     * @param deltaTimeUs Time since last update in microseconds
     * @return Velocity in counts per second
     */
    float calculateVelocity(int32_t positionChange, uint32_t deltaTimeUs);

    /**
     * Apply interpolation to improve position resolution
     *
     * @param stateAB Current A/B state
     * @param prevStateAB Previous A/B state
     * @return Interpolated position increment
     */
    float interpolatePosition(uint8_t stateAB, uint8_t prevStateAB);

    /**
     * Determine direction of rotation based on state transition
     *
     * @param stateAB Current A/B state
     * @param prevStateAB Previous A/B state
     * @return Direction: 1 for forward, -1 for reverse, 0 for invalid
     */
    int8_t determineDirection(uint8_t stateAB, uint8_t prevStateAB);
};

#endif  // ENCODER_H