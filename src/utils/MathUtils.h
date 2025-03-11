/*
 * ESP32 High-Precision Motion Control System
 * Math Utilities
 *
 * Optimized mathematical functions for real-time control applications,
 * focusing on performance and precision on the ESP32 platform.
 */

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <Arduino.h>

#include <cmath>

class MathUtils {
   public:
    /**
     * Constrain a value between minimum and maximum bounds
     *
     * @tparam T Data type
     * @param value Value to constrain
     * @param min Minimum bound
     * @param max Maximum bound
     * @return Constrained value
     */
    template <typename T>
    static inline T constrainValue(T value, T min, T max) {
        return (value < min) ? min : ((value > max) ? max : value);
    }

    /**
     * Linear interpolation between two values
     *
     * @tparam T Data type
     * @param a First value
     * @param b Second value
     * @param t Interpolation factor (0.0 - 1.0)
     * @return Interpolated value
     */
    template <typename T>
    static inline T lerp(T a, T b, float t) {
        t = constrainValue<float>(t, 0.0f, 1.0f);
        return a + (b - a) * t;
    }

    /**
     * Calculate velocity given displacement and time
     *
     * @param displacement Change in position
     * @param timeSeconds Time in seconds
     * @return Velocity
     */
    static inline float calculateVelocity(float displacement, float timeSeconds) {
        return timeSeconds > 0.0f ? displacement / timeSeconds : 0.0f;
    }

    /**
     * Calculate acceleration given velocity change and time
     *
     * @param velocityChange Change in velocity
     * @param timeSeconds Time in seconds
     * @return Acceleration
     */
    static inline float calculateAcceleration(float velocityChange, float timeSeconds) {
        return timeSeconds > 0.0f ? velocityChange / timeSeconds : 0.0f;
    }

    /**
     * Apply dead band to a value
     *
     * @param value Input value
     * @param threshold Dead band threshold
     * @return Value with dead band applied
     */
    static inline float applyDeadBand(float value, float threshold) {
        if (fabs(value) < threshold) {
            return 0.0f;
        }
        return value;
    }

    /**
     * Calculate the time required to reach a target position with trapezoidal profile
     *
     * @param distance Total distance to travel
     * @param maxVelocity Maximum velocity
     * @param acceleration Acceleration/deceleration rate
     * @return Time in seconds
     */
    static float calculateTrapezoidalProfileDuration(float distance, float maxVelocity,
                                                     float acceleration) {
        // Calculate the distance required to accelerate to max velocity and decelerate to stop
        float accelDistance = maxVelocity * maxVelocity / acceleration;

        // Check if we can reach max velocity
        if (accelDistance > (distance / 2.0f)) {
            // Triangle profile (can't reach max velocity)
            // Calculate the peak velocity we can reach
            float peakVelocity = sqrt(distance * acceleration / 2.0f);

            // Time is just acceleration to peak and deceleration to stop
            return 2.0f * peakVelocity / acceleration;
        } else {
            // Trapezoidal profile
            // Time to accelerate to max velocity
            float accelTime = maxVelocity / acceleration;

            // Distance at constant velocity
            float constDistance = distance - accelDistance;

            // Time at constant velocity
            float constTime = constDistance / maxVelocity;

            // Total time is accelerate + constant + decelerate
            return 2.0f * accelTime + constTime;
        }
    }

    /**
     * Calculate the position at a specific time in a trapezoidal velocity profile
     *
     * @param time Current time in seconds
     * @param totalTime Total profile duration in seconds
     * @param distance Total distance to travel
     * @param maxVelocity Maximum velocity
     * @param acceleration Acceleration/deceleration rate
     * @return Position at the specified time
     */
    static float calculateTrapezoidalPosition(float time, float totalTime, float distance,
                                              float maxVelocity, float acceleration) {
        if (time <= 0.0f) {
            return 0.0f;
        }

        if (time >= totalTime) {
            return distance;
        }

        // Calculate the time to accelerate to max velocity
        float accelTime = maxVelocity / acceleration;

        // Calculate the time to decelerate from max velocity to stop
        float decelTime = accelTime;

        // Calculate the time at constant velocity
        float constTime = totalTime - accelTime - decelTime;

        if (constTime < 0.0f) {
            // Triangle profile (acceleration directly followed by deceleration)
            float peakTime = totalTime / 2.0f;
            float peakVelocity = peakTime * acceleration;

            if (time <= peakTime) {
                // Acceleration phase
                return 0.5f * acceleration * time * time;
            } else {
                // Deceleration phase
                float timeFromPeak = time - peakTime;
                return distance - 0.5f * acceleration * (totalTime - time) * (totalTime - time);
            }
        } else {
            // Trapezoidal profile
            if (time <= accelTime) {
                // Acceleration phase
                return 0.5f * acceleration * time * time;
            } else if (time <= accelTime + constTime) {
                // Constant velocity phase
                return 0.5f * acceleration * accelTime * accelTime +
                       maxVelocity * (time - accelTime);
            } else {
                // Deceleration phase
                float timeFromDecel = time - (accelTime + constTime);
                return distance - 0.5f * acceleration * (totalTime - time) * (totalTime - time);
            }
        }
    }

    /**
     * Calculate the velocity at a specific time in a trapezoidal velocity profile
     *
     * @param time Current time in seconds
     * @param totalTime Total profile duration in seconds
     * @param distance Total distance to travel
     * @param maxVelocity Maximum velocity
     * @param acceleration Acceleration/deceleration rate
     * @return Velocity at the specified time
     */
    static float calculateTrapezoidalVelocity(float time, float totalTime, float distance,
                                              float maxVelocity, float acceleration) {
        if (time <= 0.0f || time >= totalTime) {
            return 0.0f;
        }

        // Calculate the time to accelerate to max velocity
        float accelTime = maxVelocity / acceleration;

        // Calculate the time to decelerate from max velocity to stop
        float decelTime = accelTime;

        // Calculate the time at constant velocity
        float constTime = totalTime - accelTime - decelTime;

        if (constTime < 0.0f) {
            // Triangle profile (acceleration directly followed by deceleration)
            float peakTime = totalTime / 2.0f;

            if (time <= peakTime) {
                // Acceleration phase
                return acceleration * time;
            } else {
                // Deceleration phase
                return acceleration * (totalTime - time);
            }
        } else {
            // Trapezoidal profile
            if (time <= accelTime) {
                // Acceleration phase
                return acceleration * time;
            } else if (time <= accelTime + constTime) {
                // Constant velocity phase
                return maxVelocity;
            } else {
                // Deceleration phase
                return acceleration * (totalTime - time);
            }
        }
    }

    /**
     * Calculate S-curve position based on jerk-limited profile
     *
     * @param time Current time in seconds
     * @param totalTime Total profile duration in seconds
     * @param distance Total distance to travel
     * @param maxVelocity Maximum velocity
     * @param maxAcceleration Maximum acceleration
     * @param maxJerk Maximum jerk
     * @return Position at the specified time
     */
    static float calculateSCurvePosition(float time, float totalTime, float distance,
                                         float maxVelocity, float maxAcceleration, float maxJerk) {
        // Simplified S-curve implementation
        // This is a very basic approximation - a full implementation would be much more complex

        if (time <= 0.0f) {
            return 0.0f;
        }

        if (time >= totalTime) {
            return distance;
        }

        // Normalized time (0.0 to 1.0)
        float t = time / totalTime;

        // Simple S-curve approximation using 5th-order polynomial
        // This is smoother than the trapezoidal profile but not a true jerk-limited profile
        float s = 10.0f * pow(t, 3.0f) - 15.0f * pow(t, 4.0f) + 6.0f * pow(t, 5.0f);

        return s * distance;
    }

    /**
     * Apply a low-pass filter to a value
     *
     * @param currentValue Current filtered value
     * @param newValue New input value
     * @param alpha Filter coefficient (0.0 - 1.0, higher = less filtering)
     * @return Filtered value
     */
    static inline float lowPassFilter(float currentValue, float newValue, float alpha) {
        alpha = constrainValue<float>(alpha, 0.0f, 1.0f);
        return currentValue * (1.0f - alpha) + newValue * alpha;
    }

    /**
     * Convert steps to millimeters
     *
     * @param steps Number of steps
     * @param stepsPerMm Steps per millimeter
     * @return Position in millimeters
     */
    static inline float stepsToMm(int32_t steps, float stepsPerMm) {
        return static_cast<float>(steps) / stepsPerMm;
    }

    /**
     * Convert millimeters to steps
     *
     * @param mm Position in millimeters
     * @param stepsPerMm Steps per millimeter
     * @return Number of steps
     */
    static inline int32_t mmToSteps(float mm, float stepsPerMm) {
        return static_cast<int32_t>(mm * stepsPerMm);
    }
};

#endif  // MATH_UTILS_H
        // End of Code