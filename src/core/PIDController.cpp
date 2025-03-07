/*
 * ESP32 High-Precision Motion Control System
 * Advanced PID Controller Implementation
 */

 #include "PIDController.h"

 PIDController::PIDController(float kp, float ki, float kd, float ff, float dt)
     : m_kp(kp)
     , m_ki(ki)
     , m_kd(kd)
     , m_ff(ff)
     , m_adaptiveGainsEnabled(false)
     , m_adaptiveErrorThreshold(100.0f)
     , m_adaptiveRate(0.1f)
     , m_kpBase(kp)
     , m_kiBase(ki)
     , m_kdBase(kd)
     , m_lastError(0.0f)
     , m_integralTerm(0.0f)
     , m_lastProcessVariable(0.0f)
     , m_filteredDerivative(0.0f)
     , m_dtSeconds(dt)
     , m_proportionalTerm(0.0f)
     , m_derivativeTerm(0.0f)
     , m_feedForwardTerm(0.0f)
     , m_lastOutput(0.0f)
     , m_derivativeFilterAlpha(CONFIG_PID_DERIVATIVE_FILTER_ALPHA)
     , m_minOutput(-CONFIG_PID_OUTPUT_LIMIT)
     , m_maxOutput(CONFIG_PID_OUTPUT_LIMIT)
     , m_antiWindupLimit(CONFIG_PID_ANTI_WINDUP_LIMIT)
 {
     initialize();
 }
 
 void PIDController::initialize() {
     reset();
 }
 
 void PIDController::reset() {
     m_lastError = 0.0f;
     m_integralTerm = 0.0f;
     m_lastProcessVariable = 0.0f;
     m_filteredDerivative = 0.0f;
     m_proportionalTerm = 0.0f;
     m_derivativeTerm = 0.0f;
     m_feedForwardTerm = 0.0f;
     m_lastOutput = 0.0f;
     m_errorBuffer.clear();
 }
 
 float PIDController::compute(float setpoint, float processVariable, float feedforwardValue) {
     // Calculate error
     float error = setpoint - processVariable;
     
     // Store error in circular buffer for advanced filtering if needed
     m_errorBuffer.push(error);
     
     // Update adaptive gains if enabled
     if (m_adaptiveGainsEnabled) {
         updateAdaptiveGains(error);
     }
     
     // Calculate proportional term
     m_proportionalTerm = m_kp * error;
     
     // Calculate integral term with anti-windup protection
     if (m_ki > 0.0f) {
         m_integralTerm += m_ki * error * m_dtSeconds;
         applyAntiWindup();
     }
     
     // Calculate derivative term with filtering
     if (m_kd > 0.0f) {
         // Calculate raw derivative (negative because we want derivative of process variable, not error)
         float derivative = (processVariable - m_lastProcessVariable) / m_dtSeconds;
         
         // Apply low-pass filter to derivative
         m_filteredDerivative = m_filteredDerivative * (1.0f - m_derivativeFilterAlpha) + 
                               derivative * m_derivativeFilterAlpha;
         
         // Calculate derivative term (negative because we want derivative of PV, not error)
         m_derivativeTerm = -m_kd * m_filteredDerivative;
     } else {
         m_derivativeTerm = 0.0f;
     }
     
     // Calculate feed-forward term
     m_feedForwardTerm = m_ff * feedforwardValue;
     
     // Sum all terms
     float output = m_proportionalTerm + m_integralTerm + m_derivativeTerm + m_feedForwardTerm;
     
     // Apply output limits
     output = applyOutputLimits(output);
     
     // Update state variables
     m_lastError = error;
     m_lastProcessVariable = processVariable;
     m_lastOutput = output;
     
     return output;
 }
 
 void PIDController::setGains(float kp, float ki, float kd, float ff) {
     m_kp = kp;
     m_ki = ki;
     m_kd = kd;
     m_ff = ff;
     
     // Update base gains for adaptive control
     m_kpBase = kp;
     m_kiBase = ki;
     m_kdBase = kd;
 }
 
 void PIDController::setKp(float kp) {
     m_kp = kp;
     m_kpBase = kp;
 }
 
 void PIDController::setKi(float ki) {
     m_ki = ki;
     m_kiBase = ki;
 }
 
 void PIDController::setKd(float kd) {
     m_kd = kd;
     m_kdBase = kd;
 }
 
 void PIDController::setFf(float ff) {
     m_ff = ff;
 }
 
 void PIDController::setOutputLimits(float min, float max) {
     if (min < max) {
         m_minOutput = min;
         m_maxOutput = max;
     }
 }
 
 void PIDController::setAntiWindupLimits(float limit) {
     m_antiWindupLimit = limit > 0.0f ? limit : 0.0f;
 }
 
 void PIDController::setDerivativeFilterAlpha(float alpha) {
     m_derivativeFilterAlpha = MathUtils::constrainValue(alpha, 0.0f, 1.0f);
 }
 
 void PIDController::enableAdaptiveGains(bool enable) {
     m_adaptiveGainsEnabled = enable;
     
     // Reset to base gains when disabling
     if (!enable) {
         m_kp = m_kpBase;
         m_ki = m_kiBase;
         m_kd = m_kdBase;
     }
 }
 
 void PIDController::setAdaptiveGainParameters(float errorThreshold, float adaptationRate) {
     m_adaptiveErrorThreshold = errorThreshold > 0.0f ? errorThreshold : 1.0f;
     m_adaptiveRate = MathUtils::constrainValue(adaptationRate, 0.0f, 1.0f);
 }
 
 float PIDController::getProportionalTerm() const {
     return m_proportionalTerm;
 }
 
 float PIDController::getIntegralTerm() const {
     return m_integralTerm;
 }
 
 float PIDController::getDerivativeTerm() const {
     return m_derivativeTerm;
 }
 
 float PIDController::getFeedForwardTerm() const {
     return m_feedForwardTerm;
 }
 
 float PIDController::getLastError() const {
     return m_lastError;
 }
 
 float PIDController::getLastOutput() const {
     return m_lastOutput;
 }
 
 void PIDController::updateAdaptiveGains(float error) {
     // Calculate error ratio (how much error exceeds threshold)
     float absError = fabs(error);
     float errorRatio = absError / m_adaptiveErrorThreshold;
     
     if (errorRatio > 1.0f) {
         // Large error: increase P, decrease I
         float adaptFactor = 1.0f + (errorRatio - 1.0f) * m_adaptiveRate;
         adaptFactor = MathUtils::constrainValue(adaptFactor, 1.0f, 2.0f);
         
         m_kp = m_kpBase * adaptFactor;
         m_ki = m_kiBase / adaptFactor;
         
         // For very large errors, increase D to dampen oscillations
         if (errorRatio > 2.0f) {
             m_kd = m_kdBase * adaptFactor;
         } else {
             m_kd = m_kdBase;
         }
     } else {
         // Small error: restore base gains for fine control
         m_kp = m_kpBase;
         m_ki = m_kiBase;
         m_kd = m_kdBase;
     }
 }
 
 void PIDController::applyAntiWindup() {
     // Clamp integral term to prevent windup
     m_integralTerm = MathUtils::constrainValue(m_integralTerm, -m_antiWindupLimit, m_antiWindupLimit);
 }
 
 float PIDController::applyOutputLimits(float output) {
     // Clamp output to min/max limits
     return MathUtils::constrainValue(output, m_minOutput, m_maxOutput);
 }
 // End of Code