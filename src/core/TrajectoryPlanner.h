/*
 * ESP32 High-Precision Motion Control System
 * Trajectory Planner
 *
 * Generates smooth motion profiles for position and velocity control,
 * supporting trapezoidal and S-curve profiles with jerk limiting.
 */

#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <Arduino.h>

#include "../Configuration.h"
#include "../utils/MathUtils.h"

/**
 * Profile types for trajectory generation
 */
enum class ProfileType {
    NONE,         // No profile (immediate)
    TRAPEZOIDAL,  // Trapezoidal velocity profile
    S_CURVE       // S-curve profile with jerk limiting
};

/**
 * Trajectory state data
 */
struct TrajectoryState {
    int32_t initialPosition;    // Initial position
    int32_t targetPosition;     // Target position
    float initialVelocity;      // Initial velocity
    float targetVelocity;       // Target velocity
    float currentPosition;      // Current position
    float currentVelocity;      // Current velocity
    float currentAcceleration;  // Current acceleration
    float maxVelocity;          // Maximum velocity
    float maxAcceleration;      // Maximum acceleration
    float maxDeceleration;      // Maximum deceleration
    float maxJerk;              // Maximum jerk (for S-curve)
    float profileDuration;      // Total duration of the profile
    float elapsedTime;          // Elapsed time since start
    bool isComplete;            // Whether profile is complete
    ProfileType profileType;    // Type of profile
};

/**
 * Trajectory planner class for generating motion profiles
 */
class TrajectoryPlanner {
   public:
    /**
     * Constructor
     *
     * @param maxVelocity Default maximum velocity
     * @param maxAcceleration Default maximum acceleration
     * @param maxDeceleration Default maximum deceleration
     * @param maxJerk Default maximum jerk
     */
    TrajectoryPlanner(float maxVelocity = CONFIG_DEFAULT_MAX_VELOCITY,
                      float maxAcceleration = CONFIG_DEFAULT_ACCELERATION,
                      float maxDeceleration = CONFIG_DEFAULT_DECELERATION,
                      float maxJerk = CONFIG_DEFAULT_MAX_JERK);

    /**
     * Initialize the trajectory planner
     */
    void initialize();

    /**
     * Reset the trajectory planner
     */
    void reset();

    /**
     * Plan a velocity move with constraints
     *
     * @param currentVelocity Current velocity
     * @param targetVelocity Target velocity
     * @param maxAcceleration Maximum acceleration
     * @param maxJerk Maximum jerk (for S-curve)
     * @param profileType Type of profile to generate
     * @return True if planning successful, false otherwise
     */
    bool planPositionMove(int32_t currentPosition, int32_t targetPosition,
                          float currentVelocity = 0.0f, float maxVelocity = 0.0f,
                          float maxAcceleration = 0.0f, float maxDeceleration = 0.0f,
                          float maxJerk = 0.0f, ProfileType profileType = ProfileType::TRAPEZOIDAL);

    /**
     * Plan a position move with constraints
     *
     * @param currentPosition Current position
     * @param targetPosition Target position
     * @param currentVelocity Current velocity
     * @param maxVelocity Maximum velocity
     * @param maxAcceleration Maximum acceleration
     * @param maxDeceleration Maximum deceleration
     * @param maxJerk Maximum jerk (for S-curve)
     * @param profileType Type of profile to generate
     * @return True if planning successful, false otherwise
     */
    bool planVelocityMove(float currentVelocity, float targetVelocity, float maxAcceleration = 0.0f,
                          float maxJerk = 0.0f, ProfileType profileType = ProfileType::TRAPEZOIDAL);

    /**
     * Update the trajectory state
     * Must be called at regular intervals
     *
     * @param deltaTimeUs Time since last update in microseconds
     */
    void update(uint32_t deltaTimeUs);

    /**
     * Get current trajectory state
     *
     * @return Current trajectory state
     */
    const TrajectoryState& getState() const;

    /**
     * Check if trajectory is complete
     *
     * @return True if complete, false otherwise
     */
    bool isComplete() const;

    /**
     * Get current position
     *
     * @return Current position
     */
    float getCurrentPosition() const;

    /**
     * Get current velocity
     *
     * @return Current velocity
     */
    float getCurrentVelocity() const;

    /**
     * Get current acceleration
     *
     * @return Current acceleration
     */
    float getCurrentAcceleration() const;

    /**
     * Get time remaining
     *
     * @return Time remaining in seconds
     */
    float getTimeRemaining() const;

    /**
     * Set default maximum velocity
     *
     * @param maxVelocity Maximum velocity
     */
    void setMaxVelocity(float maxVelocity);

    /**
     * Set default maximum acceleration
     *
     * @param maxAcceleration Maximum acceleration
     */
    void setMaxAcceleration(float maxAcceleration);

    /**
     * Set default maximum deceleration
     *
     * @param maxDeceleration Maximum deceleration
     */
    void setMaxDeceleration(float maxDeceleration);

    /**
     * Set default maximum jerk
     *
     * @param maxJerk Maximum jerk
     */
    void setMaxJerk(float maxJerk);

   private:
    // Trajectory state
    TrajectoryState m_state;

    // Default parameters
    float m_defaultMaxVelocity;
    float m_defaultMaxAcceleration;
    float m_defaultMaxDeceleration;
    float m_defaultMaxJerk;

    // Internal calculation methods

    /**
     * Calculate trapezoidal profile parameters
     *
     * @param distance Distance to travel
     * @param initialVelocity Initial velocity
     * @param targetVelocity Target velocity (0 for position move)
     * @param maxVelocity Maximum velocity
     * @param maxAcceleration Maximum acceleration
     * @param maxDeceleration Maximum deceleration
     * @return Duration of the profile in seconds
     */
    float calculateTrapezoidalProfile(float distance, float initialVelocity, float targetVelocity,
                                      float maxVelocity, float maxAcceleration,
                                      float maxDeceleration);

    /**
     * Calculate S-curve profile parameters
     *
     * @param distance Distance to travel
     * @param initialVelocity Initial velocity
     * @param targetVelocity Target velocity (0 for position move)
     * @param maxVelocity Maximum velocity
     * @param maxAcceleration Maximum acceleration
     * @param maxDeceleration Maximum deceleration
     * @param maxJerk Maximum jerk
     * @return Duration of the profile in seconds
     */
    float calculateSCurveProfile(float distance, float initialVelocity, float targetVelocity,
                                 float maxVelocity, float maxAcceleration, float maxDeceleration,
                                 float maxJerk);

    /**
     * Update trapezoidal profile
     *
     * @param elapsedTimeSeconds Elapsed time in seconds
     */
    void updateTrapezoidalProfile(float elapsedTimeSeconds);

    /**
     * Update S-curve profile
     *
     * @param elapsedTimeSeconds Elapsed time in seconds
     */
    void updateSCurveProfile(float elapsedTimeSeconds);
};

#endif  // TRAJECTORY_PLANNER_H
        // End of Code