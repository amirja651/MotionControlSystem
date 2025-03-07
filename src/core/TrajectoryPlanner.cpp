/*
 * ESP32 High-Precision Motion Control System
 * Trajectory Planner Implementation
 */

#include "TrajectoryPlanner.h"

TrajectoryPlanner::TrajectoryPlanner(float maxVelocity, float maxAcceleration,
                                     float maxDeceleration, float maxJerk)
    : m_defaultMaxVelocity(maxVelocity),
      m_defaultMaxAcceleration(maxAcceleration),
      m_defaultMaxDeceleration(maxDeceleration),
      m_defaultMaxJerk(maxJerk) {
    initialize();
}

void TrajectoryPlanner::initialize() {
    reset();
}

void TrajectoryPlanner::reset() {
    // Initialize trajectory state
    m_state.initialPosition = 0;
    m_state.targetPosition = 0;
    m_state.initialVelocity = 0.0f;
    m_state.targetVelocity = 0.0f;
    m_state.currentPosition = 0.0f;
    m_state.currentVelocity = 0.0f;
    m_state.currentAcceleration = 0.0f;
    m_state.maxVelocity = m_defaultMaxVelocity;
    m_state.maxAcceleration = m_defaultMaxAcceleration;
    m_state.maxDeceleration = m_defaultMaxDeceleration;
    m_state.maxJerk = m_defaultMaxJerk;
    m_state.profileDuration = 0.0f;
    m_state.elapsedTime = 0.0f;
    m_state.isComplete = true;
    m_state.profileType = ProfileType::NONE;
}

bool TrajectoryPlanner::planPositionMove(int32_t currentPosition, int32_t targetPosition,
                                         float currentVelocity, float maxVelocity,
                                         float maxAcceleration, float maxDeceleration,
                                         float maxJerk, ProfileType profileType) {
    // Use default parameters if not specified
    if (maxVelocity <= 0.0f) {
        maxVelocity = m_defaultMaxVelocity;
    }

    if (maxAcceleration <= 0.0f) {
        maxAcceleration = m_defaultMaxAcceleration;
    }

    if (maxDeceleration <= 0.0f) {
        maxDeceleration = m_defaultMaxDeceleration;
    }

    if (maxJerk <= 0.0f) {
        maxJerk = m_defaultMaxJerk;
    }

    // Calculate distance to travel
    float distance = static_cast<float>(targetPosition - currentPosition);

    // If distance is zero and velocity is zero, no need to move
    if (distance == 0.0f && currentVelocity == 0.0f) {
        m_state.isComplete = true;
        return true;
    }

    // Save initial state
    m_state.initialPosition = currentPosition;
    m_state.targetPosition = targetPosition;
    m_state.initialVelocity = currentVelocity;
    m_state.targetVelocity = 0.0f;  // Target velocity is zero for a position move
    m_state.maxVelocity = maxVelocity;
    m_state.maxAcceleration = maxAcceleration;
    m_state.maxDeceleration = maxDeceleration;
    m_state.maxJerk = maxJerk;
    m_state.profileType = profileType;

    // Reset current state
    m_state.currentPosition = static_cast<float>(currentPosition);
    m_state.currentVelocity = currentVelocity;
    m_state.currentAcceleration = 0.0f;
    m_state.elapsedTime = 0.0f;
    m_state.isComplete = false;

    // Calculate profile parameters based on profile type
    if (profileType == ProfileType::S_CURVE) {
        m_state.profileDuration =
            calculateSCurveProfile(distance, currentVelocity, 0.0f, maxVelocity, maxAcceleration,
                                   maxDeceleration, maxJerk);
    } else {
        // Default to trapezoidal
        m_state.profileDuration = calculateTrapezoidalProfile(
            distance, currentVelocity, 0.0f, maxVelocity, maxAcceleration, maxDeceleration);
    }

    // Check if valid duration was calculated
    if (m_state.profileDuration <= 0.0f) {
        // Invalid profile
        m_state.isComplete = true;
        return false;
    }

    return true;
}

bool TrajectoryPlanner::planVelocityMove(float currentVelocity, float targetVelocity,
                                         float maxAcceleration, float maxJerk,
                                         ProfileType profileType) {
    // Use default parameters if not specified
    if (maxAcceleration <= 0.0f) {
        maxAcceleration = m_defaultMaxAcceleration;
    }

    if (maxJerk <= 0.0f) {
        maxJerk = m_defaultMaxJerk;
    }

    // If current velocity is already at target velocity, no need to move
    if (currentVelocity == targetVelocity) {
        m_state.isComplete = true;
        return true;
    }

    // Save initial state
    m_state.initialPosition = 0;
    m_state.targetPosition = 0;  // Not applicable for velocity move
    m_state.initialVelocity = currentVelocity;
    m_state.targetVelocity = targetVelocity;
    m_state.maxVelocity =
        fabs(targetVelocity);  // For velocity move, max velocity is target velocity
    m_state.maxAcceleration = maxAcceleration;
    m_state.maxDeceleration = maxAcceleration;  // Use same value for deceleration
    m_state.maxJerk = maxJerk;
    m_state.profileType = profileType;

    // Reset current state
    m_state.currentPosition = 0.0f;  // Not relevant for velocity move
    m_state.currentVelocity = currentVelocity;
    m_state.currentAcceleration = 0.0f;
    m_state.elapsedTime = 0.0f;
    m_state.isComplete = false;

    // Calculate velocity change
    float velocityChange = targetVelocity - currentVelocity;

    // For velocity move, we're interested in acceleration only, not distance
    if (profileType == ProfileType::S_CURVE) {
        // For S-curve, we need jerk
        // Simple approximation: time to accelerate with jerk limits
        float accelTime = maxAcceleration / maxJerk;  // Time to reach max acceleration
        float cruiseTime = (fabs(velocityChange) - maxAcceleration * accelTime) / maxAcceleration;

        if (cruiseTime < 0.0f) {
            // We never reach max acceleration
            accelTime = sqrt(fabs(velocityChange) / maxJerk);
            cruiseTime = 0.0f;
        }

        m_state.profileDuration = 2.0f * accelTime + cruiseTime;
    } else {
        // For trapezoidal (or triangular), just time to reach velocity with constant acceleration
        m_state.profileDuration = fabs(velocityChange) / maxAcceleration;
    }

    // Check if valid duration was calculated
    if (m_state.profileDuration <= 0.0f) {
        // Invalid profile
        m_state.isComplete = true;
        return false;
    }

    return true;
}

void TrajectoryPlanner::update(uint32_t deltaTimeUs) {
    // If trajectory is complete, nothing to do
    if (m_state.isComplete) {
        return;
    }

    // Convert time delta to seconds
    float deltaTimeSeconds = static_cast<float>(deltaTimeUs) / 1000000.0f;

    // Update elapsed time
    m_state.elapsedTime += deltaTimeSeconds;

    // Check if we've reached the end of the profile
    if (m_state.elapsedTime >= m_state.profileDuration) {
        // Reached the end of the profile
        m_state.currentPosition = static_cast<float>(m_state.targetPosition);
        m_state.currentVelocity = m_state.targetVelocity;
        m_state.currentAcceleration = 0.0f;
        m_state.isComplete = true;
        return;
    }

    // Update trajectory based on profile type
    if (m_state.profileType == ProfileType::S_CURVE) {
        updateSCurveProfile(m_state.elapsedTime);
    } else {
        // Default to trapezoidal
        updateTrapezoidalProfile(m_state.elapsedTime);
    }
}

const TrajectoryState& TrajectoryPlanner::getState() const {
    return m_state;
}

bool TrajectoryPlanner::isComplete() const {
    return m_state.isComplete;
}

float TrajectoryPlanner::getCurrentPosition() const {
    return m_state.currentPosition;
}

float TrajectoryPlanner::getCurrentVelocity() const {
    return m_state.currentVelocity;
}

float TrajectoryPlanner::getCurrentAcceleration() const {
    return m_state.currentAcceleration;
}

float TrajectoryPlanner::getTimeRemaining() const {
    if (m_state.isComplete) {
        return 0.0f;
    }

    return m_state.profileDuration - m_state.elapsedTime;
}

void TrajectoryPlanner::setMaxVelocity(float maxVelocity) {
    m_defaultMaxVelocity = maxVelocity > 0.0f ? maxVelocity : CONFIG_DEFAULT_MAX_VELOCITY;
}

void TrajectoryPlanner::setMaxAcceleration(float maxAcceleration) {
    m_defaultMaxAcceleration =
        maxAcceleration > 0.0f ? maxAcceleration : CONFIG_DEFAULT_ACCELERATION;
}

void TrajectoryPlanner::setMaxDeceleration(float maxDeceleration) {
    m_defaultMaxDeceleration =
        maxDeceleration > 0.0f ? maxDeceleration : CONFIG_DEFAULT_DECELERATION;
}

void TrajectoryPlanner::setMaxJerk(float maxJerk) {
    m_defaultMaxJerk = maxJerk > 0.0f ? maxJerk : CONFIG_DEFAULT_MAX_JERK;
}

float TrajectoryPlanner::calculateTrapezoidalProfile(float distance, float initialVelocity,
                                                     float targetVelocity, float maxVelocity,
                                                     float maxAcceleration, float maxDeceleration) {
    // If this is a velocity move, not a position move
    if (m_state.initialPosition == m_state.targetPosition) {
        // Simple velocity ramp
        float velocityChange = targetVelocity - initialVelocity;
        return fabs(velocityChange) / maxAcceleration;
    }

    // For position moves:
    // Calculate time to accelerate to max velocity
    float accelTime = (maxVelocity - initialVelocity) / maxAcceleration;
    accelTime = accelTime < 0.0f ? 0.0f : accelTime;

    // Calculate time to decelerate from max velocity to target velocity (typically 0)
    float decelTime = (maxVelocity - targetVelocity) / maxDeceleration;
    decelTime = decelTime < 0.0f ? 0.0f : decelTime;

    // Calculate distances covered during acceleration and deceleration
    float accelDistance =
        initialVelocity * accelTime + 0.5f * maxAcceleration * accelTime * accelTime;
    float decelDistance = maxVelocity * decelTime - 0.5f * maxDeceleration * decelTime * decelTime;

    // Calculate remaining distance at constant velocity
    float cruiseDistance = distance - accelDistance - decelDistance;

    // Check if we can reach max velocity (triangle vs. trapezoid)
    if (cruiseDistance < 0.0f) {
        // Triangle profile - we never reach max velocity
        // Recalculate with peak velocity we can actually reach
        // v² = v₀² + 2a₁s₁ = v₂² + 2a₂s₂
        // Solve for peak velocity and recalculate times

        // For simplicity, let's use a numerical method
        // Iteratively reduce max velocity until profile is valid
        float adjustedMaxVelocity = maxVelocity;
        float step = maxVelocity * 0.1f;

        while (cruiseDistance < 0.0f && step > 0.001f) {
            adjustedMaxVelocity -= step;

            // Recalculate times and distances
            accelTime = (adjustedMaxVelocity - initialVelocity) / maxAcceleration;
            accelTime = accelTime < 0.0f ? 0.0f : accelTime;

            decelTime = (adjustedMaxVelocity - targetVelocity) / maxDeceleration;
            decelTime = decelTime < 0.0f ? 0.0f : decelTime;

            accelDistance =
                initialVelocity * accelTime + 0.5f * maxAcceleration * accelTime * accelTime;
            decelDistance =
                adjustedMaxVelocity * decelTime - 0.5f * maxDeceleration * decelTime * decelTime;

            cruiseDistance = distance - accelDistance - decelDistance;

            // Reduce step if we went too far
            if (cruiseDistance > 0.0f) {
                step *= 0.5f;
                adjustedMaxVelocity += step;
            }
        }

        // If we still couldn't find a valid profile, use a direct calculation
        if (cruiseDistance < 0.0f) {
            // Special case for starting and ending with zero velocity
            if (initialVelocity == 0.0f && targetVelocity == 0.0f) {
                float peakVelocity = sqrt(2.0f * maxAcceleration * maxDeceleration * distance /
                                          (maxAcceleration + maxDeceleration));
                accelTime = peakVelocity / maxAcceleration;
                decelTime = peakVelocity / maxDeceleration;
                return accelTime + decelTime;
            }

            // For other cases, use a simplified approximation
            return 2.0f * sqrt(distance / maxAcceleration);
        }

        // No cruise phase
        return accelTime + decelTime;
    }

    // Trapezoidal profile with cruise phase
    float cruiseTime = cruiseDistance / maxVelocity;

    // Total duration
    return accelTime + cruiseTime + decelTime;
}

float TrajectoryPlanner::calculateSCurveProfile(float distance, float initialVelocity,
                                                float targetVelocity, float maxVelocity,
                                                float maxAcceleration, float maxDeceleration,
                                                float maxJerk) {
    // This is a simplified S-curve implementation
    // A complete implementation would be much more complex

    // First, calculate a trapezoidal profile as a base
    float trapezoidalDuration = calculateTrapezoidalProfile(
        distance, initialVelocity, targetVelocity, maxVelocity, maxAcceleration, maxDeceleration);

    // Add time for jerk limitations
    // For each acceleration/deceleration phase, we add time for jerk smoothing
    float jerkTime = maxAcceleration / maxJerk;

    // Total jerk phases: beginning of accel, end of accel, beginning of decel, end of decel
    return trapezoidalDuration + 4.0f * jerkTime;
}

void TrajectoryPlanner::updateTrapezoidalProfile(float elapsedTimeSeconds) {
    // If doing a velocity move
    if (m_state.initialPosition == m_state.targetPosition) {
        // Simple velocity ramp
        float velocityChange = m_state.targetVelocity - m_state.initialVelocity;
        float accelSign = velocityChange >= 0.0f ? 1.0f : -1.0f;
        float acceleration = accelSign * m_state.maxAcceleration;

        // Calculate current velocity based on elapsed time
        if (elapsedTimeSeconds >= m_state.profileDuration) {
            m_state.currentVelocity = m_state.targetVelocity;
            m_state.currentAcceleration = 0.0f;
        } else {
            m_state.currentVelocity = m_state.initialVelocity + acceleration * elapsedTimeSeconds;
            m_state.currentAcceleration = acceleration;
        }

        return;
    }

    // For position moves, use the MathUtils function to calculate position and velocity
    // This ensures consistent trajectory calculation
    float distance = static_cast<float>(m_state.targetPosition - m_state.initialPosition);

    m_state.currentPosition = MathUtils::calculateTrapezoidalPosition(
                                  elapsedTimeSeconds, m_state.profileDuration, distance,
                                  m_state.maxVelocity, m_state.maxAcceleration) +
                              m_state.initialPosition;

    m_state.currentVelocity = MathUtils::calculateTrapezoidalVelocity(
        elapsedTimeSeconds, m_state.profileDuration, distance, m_state.maxVelocity,
        m_state.maxAcceleration);

    // Estimate current acceleration
    // In a trapezoidal profile, acceleration is either maxAcceleration, -maxDeceleration, or 0
    float accelTime = m_state.maxVelocity / m_state.maxAcceleration;
    float decelTime = m_state.maxVelocity / m_state.maxDeceleration;

    if (elapsedTimeSeconds < accelTime) {
        // Acceleration phase
        m_state.currentAcceleration = m_state.maxAcceleration;
    } else if (elapsedTimeSeconds > (m_state.profileDuration - decelTime)) {
        // Deceleration phase
        m_state.currentAcceleration = -m_state.maxDeceleration;
    } else {
        // Constant velocity phase
        m_state.currentAcceleration = 0.0f;
    }
}

void TrajectoryPlanner::updateSCurveProfile(float elapsedTimeSeconds) {
    // Use simplified S-curve approximation based on MathUtils
    float distance = static_cast<float>(m_state.targetPosition - m_state.initialPosition);

    m_state.currentPosition = MathUtils::calculateSCurvePosition(
                                  elapsedTimeSeconds, m_state.profileDuration, distance,
                                  m_state.maxVelocity, m_state.maxAcceleration, m_state.maxJerk) +
                              m_state.initialPosition;

    // For velocity, we need to calculate the derivative of the position function
    // For simplicity, use finite difference approximation
    const float dt = 0.001f;  // 1ms
    float positionNext = MathUtils::calculateSCurvePosition(
        elapsedTimeSeconds + dt, m_state.profileDuration, distance, m_state.maxVelocity,
        m_state.maxAcceleration, m_state.maxJerk);

    float positionCurrent = MathUtils::calculateSCurvePosition(
        elapsedTimeSeconds, m_state.profileDuration, distance, m_state.maxVelocity,
        m_state.maxAcceleration, m_state.maxJerk);

    m_state.currentVelocity = (positionNext - positionCurrent) / dt;

    // For acceleration, use second derivative approximation
    float positionPrev = MathUtils::calculateSCurvePosition(
        elapsedTimeSeconds - dt, m_state.profileDuration, distance, m_state.maxVelocity,
        m_state.maxAcceleration, m_state.maxJerk);

    m_state.currentAcceleration =
        (positionNext - 2.0f * positionCurrent + positionPrev) / (dt * dt);
}
// End of Code