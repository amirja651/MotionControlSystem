/*
 * ESP32 High-Precision Motion Control System
 * Encoder Implementation
 */

#include "Encoder.h"

Encoder::Encoder(uint8_t encAPin, uint8_t encBPin, uint16_t pulsesPerRev, bool invertDirection)
    : m_encoderAPin(encAPin),
      m_encoderBPin(encBPin),
      m_pulsesPerRev(pulsesPerRev),
      m_interpolationFactor(CONFIG_ENCODER_INTERPOLATION_FACTOR),
      m_invertDirection(invertDirection),
      m_position(0),
      m_stateAB(0),
      m_transitions(0),
      m_filteredPosition(0.0f),
      m_velocity(0.0f),
      m_filteredVelocity(0.0f),
      m_acceleration(0.0f),
      m_lastUpdateTimeUs(0),
      m_timeSinceLastTransitionUs(0),
      m_deltaTimeUs(0),
      m_velocityFilterAlpha(0.2f)  // Default: moderate filtering
      ,
      m_positionFilterAlpha(0.5f)  // Default: less filtering for position
      ,
      m_direction(0) {
}

bool Encoder::initialize() {
    // Configure encoder pins as inputs with pull-ups
    pinMode(m_encoderAPin, INPUT_PULLUP);
    pinMode(m_encoderBPin, INPUT_PULLUP);

    // Read initial state
    bool pinA = digitalRead(m_encoderAPin);
    bool pinB = digitalRead(m_encoderBPin);
    m_stateAB = (pinA ? 1 : 0) | (pinB ? 2 : 0);

    // Initialize timing
    m_lastUpdateTimeUs = micros();

    return true;
}

void Encoder::reset() {
    m_position = 0;
    m_filteredPosition = 0.0f;
    m_velocity = 0.0f;
    m_filteredVelocity = 0.0f;
    m_acceleration = 0.0f;
    m_positionBuffer.clear();
    m_velocityBuffer.clear();
}

void Encoder::setPosition(int32_t position) {
    m_position = position;
    m_filteredPosition = static_cast<float>(position);
    m_positionBuffer.clear();
    m_positionBuffer.push(position);
}

void Encoder::setVelocity(float velocity) {
    m_velocity = velocity;
    m_filteredVelocity = velocity;
    m_velocityBuffer.clear();
    m_velocityBuffer.push(velocity);
}

int32_t Encoder::getPosition() const {
    return m_position;
}

float Encoder::getFilteredPosition() const {
    return m_filteredPosition;
}

float Encoder::getVelocity() const {
    return m_velocity;
}

float Encoder::getFilteredVelocity() const {
    return m_filteredVelocity;
}

float Encoder::getAcceleration() const {
    return m_acceleration;
}

void Encoder::setInterpolationFactor(uint8_t factor) {
    // Only allow valid interpolation factors: 1, 2, 4, or 8
    if (factor == 1 || factor == 2 || factor == 4 || factor == 8) {
        m_interpolationFactor = factor;
    }
}

void Encoder::setVelocityFilterAlpha(float alpha) {
    m_velocityFilterAlpha = MathUtils::constrainValue(alpha, 0.01f, 1.0f);
}

void Encoder::setPositionFilterAlpha(float alpha) {
    m_positionFilterAlpha = MathUtils::constrainValue(alpha, 0.01f, 1.0f);
}

void Encoder::update(uint32_t deltaTimeUs) {
    m_deltaTimeUs = deltaTimeUs;
    m_timeSinceLastTransitionUs += deltaTimeUs;

    // Measure time between updates for velocity calculation
    uint32_t currentTimeUs = micros();

    // Calculate elapsed time since last update
    uint32_t elapsedTimeUs = currentTimeUs - m_lastUpdateTimeUs;
    m_lastUpdateTimeUs = currentTimeUs;

    if (elapsedTimeUs > 0) {
        // Calculate position change
        static int32_t lastPosition = m_position;
        int32_t positionChange = m_position - lastPosition;
        lastPosition = m_position;

        // Calculate raw velocity
        float newVelocity = calculateVelocity(positionChange, elapsedTimeUs);

        // Apply low-pass filter to velocity
        float prevFilteredVelocity = m_filteredVelocity;
        m_filteredVelocity =
            MathUtils::lowPassFilter(m_filteredVelocity, newVelocity, m_velocityFilterAlpha);

        // Store velocity in circular buffer
        m_velocityBuffer.push(m_filteredVelocity);

        // Calculate acceleration
        m_acceleration = (m_filteredVelocity - prevFilteredVelocity) / (elapsedTimeUs * 1e-6f);

        // Apply low-pass filter to position
        m_filteredPosition = MathUtils::lowPassFilter(
            m_filteredPosition, static_cast<float>(m_position), m_positionFilterAlpha);

        // Store position in circular buffer
        int32_t position_copy = m_position;
        m_positionBuffer.push(position_copy);
    }

    // Auto-reset velocity to zero if no transitions for a while
    // This helps detect "stopped" state more quickly
    if (m_timeSinceLastTransitionUs > 50000) {  // 50ms timeout
        if (fabs(m_velocity) < 0.1f) {
            m_velocity = 0.0f;
            m_filteredVelocity = 0.0f;
        }
    }
}

void Encoder::processInterrupt(bool pinA, bool pinB) {
    // Get previous state
    uint8_t prevStateAB = m_stateAB;

    // Determine current state
    uint8_t stateAB = (pinA ? 1 : 0) | (pinB ? 2 : 0);
    m_stateAB = stateAB;

    // Only update if state has changed
    if (stateAB != prevStateAB) {
        // Reset transition timer
        m_timeSinceLastTransitionUs = 0;
        m_transitions++;

        // Get direction of rotation from the state transition
        int8_t direction = determineDirection(stateAB, prevStateAB);

        if (direction != 0) {
            m_direction = direction;

            // Apply direction inversion if configured
            if (m_invertDirection) {
                direction = -direction;
            }

            // Update position counter
            m_position += direction;

            // Calculate instantaneous velocity from time between transitions
            // This provides a responsive velocity signal
            if (m_deltaTimeUs > 0) {
                // Convert to counts per second
                float instantVelocity = direction * (1000000.0f / m_deltaTimeUs);

                // Apply some minimal filtering to smooth out noise
                m_velocity = MathUtils::lowPassFilter(m_velocity, instantVelocity, 0.3f);
            }
        }
    }
}

uint32_t Encoder::getCountsPerRevolution() const {
    // Return counts per revolution including interpolation factor
    return m_pulsesPerRev * 4 * m_interpolationFactor;  // 4 counts per pulse in quadrature
}

float Encoder::calculateVelocity(int32_t positionChange, uint32_t deltaTimeUs) {
    // Convert microseconds to seconds
    float deltaTimeSeconds = static_cast<float>(deltaTimeUs) / 1000000.0f;

    // Avoid division by zero
    if (deltaTimeSeconds < 0.000001f) {
        return m_velocity;  // Return previous velocity
    }

    // Calculate counts per second
    return static_cast<float>(positionChange) / deltaTimeSeconds;
}

float Encoder::interpolatePosition(uint8_t stateAB, uint8_t prevStateAB) {
    // Simple implementation - just returns integer steps
    // Could be enhanced with real interpolation based on timing
    return 1.0f;
}

int8_t Encoder::determineDirection(uint8_t stateAB, uint8_t prevStateAB) {
    // Quadrature state transition table:
    // Previous -> Current : Direction
    // 0 -> 1 : +1 (CW)
    // 0 -> 2 : -1 (CCW)
    // 1 -> 3 : +1 (CW)
    // 1 -> 0 : -1 (CCW)
    // 2 -> 0 : +1 (CW)
    // 2 -> 3 : -1 (CCW)
    // 3 -> 2 : +1 (CW)
    // 3 -> 1 : -1 (CCW)

    // A more compact way to check is using XOR and comparing bits
    // If XOR of states is 1 or 2, check if bit 0 of previous state matches bit 1 of XOR result
    // If they match, direction is CW (+1), otherwise CCW (-1)

    uint8_t xorResult = prevStateAB ^ stateAB;

    // Only handle valid transitions (single bit changes)
    if (xorResult == 1 || xorResult == 2) {
        return (((prevStateAB & 0x01) ^ ((xorResult & 0x02) >> 1)) == 0) ? 1 : -1;
    }

    // Invalid transition (both bits changed or no change)
    return 0;
}
// End of Code