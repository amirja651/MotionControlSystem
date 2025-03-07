/*
 * ESP32 High-Precision Motion Control System
 * Advanced PID Controller Implementation
 * 
 * Features:
 * - Proportional, Integral, Derivative control
 * - Feed-forward compensation
 * - Anti-windup protection
 * - Derivative filtering
 * - Adaptive gains
 * - Output limiting
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "../Configuration.h"
#include "../utils/CircularBuffer.h"
#include "../utils/MathUtils.h"

class PIDController {
public:
    /**
     * Constructor
     * 
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param ff Feed-forward gain
     * @param dt Control loop time step in seconds
     */
    PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float ff = 0.0f, float dt = 0.001f);
    
    /**
     * Initialize the PID controller
     */
    void initialize();
    
    /**
     * Reset the PID controller state
     */
    void reset();
    
    /**
     * Compute PID output
     * 
     * @param setpoint Desired position/velocity
     * @param processVariable Current position/velocity
     * @param feedforwardValue Optional feed-forward value
     * @return Control output
     */
    float compute(float setpoint, float processVariable, float feedforwardValue = 0.0f);
    
    /**
     * Set PID gains
     */
    void setGains(float kp, float ki, float kd, float ff);
    
    /**
     * Set proportional gain
     */
    void setKp(float kp);
    
    /**
     * Set integral gain
     */
    void setKi(float ki);
    
    /**
     * Set derivative gain
     */
    void setKd(float kd);
    
    /**
     * Set feed-forward gain
     */
    void setFf(float ff);
    
    /**
     * Set output limits
     */
    void setOutputLimits(float min, float max);
    
    /**
     * Set integral anti-windup limits
     */
    void setAntiWindupLimits(float limit);
    
    /**
     * Set derivative low-pass filter coefficient
     * 
     * @param alpha Filter coefficient (0-1, lower = more filtering)
     */
    void setDerivativeFilterAlpha(float alpha);
    
    /**
     * Enable/disable adaptive gains based on error magnitude
     */
    void enableAdaptiveGains(bool enable);
    
    /**
     * Set adaptive gain parameters
     * 
     * @param errorThreshold Error threshold for gain adaptation
     * @param adaptationRate Rate of gain adjustment (0-1)
     */
    void setAdaptiveGainParameters(float errorThreshold, float adaptationRate);
    
    /**
     * Get current proportional term
     */
    float getProportionalTerm() const;
    
    /**
     * Get current integral term
     */
    float getIntegralTerm() const;
    
    /**
     * Get current derivative term
     */
    float getDerivativeTerm() const;
    
    /**
     * Get current feed-forward term
     */
    float getFeedForwardTerm() const;
    
    /**
     * Get last computed error
     */
    float getLastError() const;
    
    /**
     * Get last output value
     */
    float getLastOutput() const;
    
private:
    // PID gains
    float m_kp;                   // Proportional gain
    float m_ki;                   // Integral gain
    float m_kd;                   // Derivative gain
    float m_ff;                   // Feed-forward gain
    
    // Adaptive gain parameters
    bool m_adaptiveGainsEnabled;  // Whether adaptive gains are enabled
    float m_adaptiveErrorThreshold; // Error threshold for gain adaptation
    float m_adaptiveRate;         // Rate of gain adaptation
    float m_kpBase;               // Base proportional gain
    float m_kiBase;               // Base integral gain
    float m_kdBase;               // Base derivative gain
    
    // PID state variables
    float m_lastError;            // Previous error value
    float m_integralTerm;         // Accumulated integral term
    float m_lastProcessVariable;  // Previous process variable
    float m_filteredDerivative;   // Filtered derivative value
    float m_dtSeconds;            // Time step in seconds
    
    // PID terms for diagnostics
    float m_proportionalTerm;     // Current P term
    float m_derivativeTerm;       // Current D term
    float m_feedForwardTerm;      // Current FF term
    float m_lastOutput;           // Last computed output
    
    // Filter parameters
    float m_derivativeFilterAlpha; // Derivative low-pass filter coefficient
    
    // Limits
    float m_minOutput;            // Minimum output value
    float m_maxOutput;            // Maximum output value
    float m_antiWindupLimit;      // Anti-windup limit for integral term
    
    // Error circular buffer for advanced filtering
    CircularBuffer<float, 4> m_errorBuffer;
    
    /**
     * Update adaptive gains based on error magnitude
     * 
     * @param error Current error value
     */
    void updateAdaptiveGains(float error);
    
    /**
     * Apply anti-windup to integral term
     */
    void applyAntiWindup();
    
    /**
     * Apply output limits
     * 
     * @param output Raw output value
     * @return Limited output value
     */
    float applyOutputLimits(float output);
};

#endif // PID_CONTROLLER_H