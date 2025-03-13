/*
 * ESP32 High-Precision Motion Control System
 * Advanced PID Controller Implementation
 */

#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float ff, float dt, Logger* logger)
    : m_kp(kp),
      m_ki(ki),
      m_kd(kd),
      m_ff(ff),
      m_dtSeconds(dt),
      m_logger(logger),
      m_adaptiveGainsEnabled(false),
      m_adaptiveErrorThreshold(100.0f),
      m_adaptiveRate(0.1f),
      m_kpBase(kp),
      m_kiBase(ki),
      m_kdBase(kd),
      m_lastError(0.0f),
      m_integralTerm(0.0f),
      m_lastProcessVariable(0.0f),
      m_filteredDerivative(0.0f),
      m_proportionalTerm(0.0f),
      m_derivativeTerm(0.0f),
      m_feedForwardTerm(0.0f),
      m_lastOutput(0.0f),
      m_derivativeFilterAlpha(CONFIG_PID_DERIVATIVE_FILTER_ALPHA),
      m_minOutput(-CONFIG_PID_OUTPUT_LIMIT),
      m_maxOutput(CONFIG_PID_OUTPUT_LIMIT),
      m_antiWindupLimit(CONFIG_PID_ANTI_WINDUP_LIMIT) {
    initialize();
}

void PIDController::initialize() {
    reset();

    m_logger->logInfo(String("PID controller initialized with gains: ") + "Kp=" + String(m_kp)
                          + ", Ki=" + String(m_ki) + ", Kd=" + String(m_kd) + ", Ff=" + String(m_ff)
                          + ", dt=" + String(m_dtSeconds) + "s",
                      LogModule::PID_CONTROLLER);
}

void PIDController::reset() {
    m_lastError           = 0.0f;
    m_integralTerm        = 0.0f;
    m_lastProcessVariable = 0.0f;
    m_filteredDerivative  = 0.0f;
    m_proportionalTerm    = 0.0f;
    m_derivativeTerm      = 0.0f;
    m_feedForwardTerm     = 0.0f;
    m_lastOutput          = 0.0f;
    m_errorBuffer.clear();

    m_logger->logDebug("PID controller reset", LogModule::PID_CONTROLLER);
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
        // Calculate raw derivative (negative because we want derivative of process variable, not
        // error)
        float derivative = (processVariable - m_lastProcessVariable) / m_dtSeconds;

        // Apply low-pass filter to derivative
        m_filteredDerivative = m_filteredDerivative * (1.0f - m_derivativeFilterAlpha)
                               + derivative * m_derivativeFilterAlpha;

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
    m_lastError           = error;
    m_lastProcessVariable = processVariable;
    m_lastOutput          = output;

    // Log PID calculation periodically to avoid flooding
    static uint32_t lastLogTime = 0;
    if (m_logger && (millis() - lastLogTime > 500)) {  // Log every 500ms
        lastLogTime = millis();
        m_logger->logDebug("PID: SP=" + String(setpoint) + ", PV=" + String(processVariable)
                               + ", Err=" + String(error) + ", P=" + String(m_proportionalTerm)
                               + ", I=" + String(m_integralTerm) + ", D=" + String(m_derivativeTerm)
                               + ", FF=" + String(m_feedForwardTerm) + ", Out=" + String(output),
                           LogModule::PID_CONTROLLER);
    }

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

    m_logger->logInfo("PID gains set: Kp=" + String(kp) + ", Ki=" + String(ki)
                          + ", Kd=" + String(kd) + ", Ff=" + String(ff),
                      LogModule::PID_CONTROLLER);
}

void PIDController::setKp(float kp) {
    m_kp     = kp;
    m_kpBase = kp;

    m_logger->logInfo("PID Kp set to " + String(kp), LogModule::PID_CONTROLLER);
}

void PIDController::setKi(float ki) {
    m_ki     = ki;
    m_kiBase = ki;

    m_logger->logInfo("PID Ki set to " + String(ki), LogModule::PID_CONTROLLER);
}

void PIDController::setKd(float kd) {
    m_kd     = kd;
    m_kdBase = kd;

    m_logger->logInfo("PID Kd set to " + String(kd), LogModule::PID_CONTROLLER);
}

void PIDController::setFf(float ff) {
    m_ff = ff;

    m_logger->logInfo("PID Ff set to " + String(ff), LogModule::PID_CONTROLLER);
}

void PIDController::setOutputLimits(float min, float max) {
    if (min < max) {
        m_minOutput = min;
        m_maxOutput = max;

        m_logger->logInfo("PID output limits set: min=" + String(min) + ", max=" + String(max),
                          LogModule::PID_CONTROLLER);
    } else {
        m_logger->logError("Invalid output limits: min must be less than max",
                           LogModule::PID_CONTROLLER);
    }
}

void PIDController::setAntiWindupLimits(float limit) {
    m_antiWindupLimit = limit > 0.0f ? limit : 0.0f;

    m_logger->logInfo("PID anti-windup limit set to " + String(m_antiWindupLimit),
                      LogModule::PID_CONTROLLER);
}

void PIDController::setDerivativeFilterAlpha(float alpha) {
    m_derivativeFilterAlpha = MathUtils::constrainValue(alpha, 0.0f, 1.0f);

    m_logger->logInfo("PID derivative filter alpha set to " + String(m_derivativeFilterAlpha),
                      LogModule::PID_CONTROLLER);
}

void PIDController::enableAdaptiveGains(bool enable) {
    m_adaptiveGainsEnabled = enable;

    // Reset to base gains when disabling
    if (!enable) {
        m_kp = m_kpBase;
        m_ki = m_kiBase;
        m_kd = m_kdBase;
    }

    m_logger->logInfo("PID adaptive gains " + String(enable ? "enabled" : "disabled"),
                      LogModule::PID_CONTROLLER);
}

void PIDController::setAdaptiveGainParameters(float errorThreshold, float adaptationRate) {
    m_adaptiveErrorThreshold = errorThreshold > 0.0f ? errorThreshold : 1.0f;
    m_adaptiveRate           = MathUtils::constrainValue(adaptationRate, 0.0f, 1.0f);

    m_logger->logInfo(
        "PID adaptive gain parameters set: errorThreshold=" + String(m_adaptiveErrorThreshold)
            + ", adaptationRate=" + String(m_adaptiveRate),
        LogModule::PID_CONTROLLER);
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
    float absError   = fabs(error);
    float errorRatio = absError / m_adaptiveErrorThreshold;

    if (errorRatio > 1.0f) {
        // Large error: increase P, decrease I
        float adaptFactor = 1.0f + (errorRatio - 1.0f) * m_adaptiveRate;
        adaptFactor       = MathUtils::constrainValue(adaptFactor, 1.0f, 2.0f);

        m_kp = m_kpBase * adaptFactor;
        m_ki = m_kiBase / adaptFactor;

        // For very large errors, increase D to dampen oscillations
        if (errorRatio > 2.0f) {
            m_kd = m_kdBase * adaptFactor;
        } else {
            m_kd = m_kdBase;
        }

        if (m_logger && errorRatio > 2.0f) {
            m_logger->logDebug("PID adaptive gains adjusted: errorRatio=" + String(errorRatio)
                                   + ", adaptFactor=" + String(adaptFactor) + ", Kp=" + String(m_kp)
                                   + ", Ki=" + String(m_ki) + ", Kd=" + String(m_kd),
                               LogModule::PID_CONTROLLER);
        }
    } else {
        // Small error: restore base gains for fine control
        m_kp = m_kpBase;
        m_ki = m_kiBase;
        m_kd = m_kdBase;
    }
}

void PIDController::applyAntiWindup() {
    // Check if integral term exceeds limits
    if (m_integralTerm > m_antiWindupLimit || m_integralTerm < -m_antiWindupLimit) {
        // Log anti-windup activation
        m_logger->logDebug("PID anti-windup activated: integralTerm=" + String(m_integralTerm)
                               + " constrained to +/-" + String(m_antiWindupLimit),
                           LogModule::PID_CONTROLLER);
    }

    // Clamp integral term to prevent windup
    m_integralTerm =
        MathUtils::constrainValue(m_integralTerm, -m_antiWindupLimit, m_antiWindupLimit);
}

float PIDController::applyOutputLimits(float output) {
    // Check if output exceeds limits
    if (output > m_maxOutput || output < m_minOutput) {
        // Log output limiting
        m_logger->logDebug("PID output limited: " + String(output) + " constrained to ["
                               + String(m_minOutput) + ", " + String(m_maxOutput) + "]",
                           LogModule::PID_CONTROLLER);
    }

    // Clamp output to min/max limits
    return MathUtils::constrainValue(output, m_minOutput, m_maxOutput);
}
// End of Code