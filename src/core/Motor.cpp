/*
 * ESP32 High-Precision Motion Control System
 * Motor Implementation
 */

#include "Motor.h"

Motor::Motor(const MotorConfig& config, Logger* logger)
    : m_config(config),
      m_logger(logger),
      m_encoder(nullptr),
      m_driver(nullptr),
      m_trajectoryPlanner(nullptr),
      m_controller(nullptr),
      m_controlMode(MotorControlMode::DISABLED_),
      m_controlIntervalUs(1000000 / CONFIG_CONTROL_LOOP_FREQUENCY_HZ),
      m_trajectoryIntervalUs(1000000 / CONFIG_TRAJECTORY_UPDATE_FREQUENCY_HZ),
      m_positionTolerance(10.0f),
      m_velocityTolerance(10.0f),
      m_softLimitsEnabled(config.useSoftLimits),
      m_softLimitMin(config.softLimitMin),
      m_softLimitMax(config.softLimitMax),
      m_limitSwitchesEnabled(true),
      m_invertLimitMin(false),
      m_invertLimitMax(false),
      m_emergencyStopActive(false),
      m_limitMinState(false),
      m_limitMaxState(false),
      m_lastControlUpdateUs(0),
      m_lastTrajectoryUpdateUs(0),
      m_invertEnable(config.invertEnable) {
  // Initialize motor state
  memset(&m_state, 0, sizeof(MotorState));
  m_state.status = MotorStatus::DISABLED_;
  m_state.error  = MotorError::NONE;
}

bool Motor::initialize() {
  // Initialize encoder
  m_encoder = new Encoder(m_config.encoderAPin,
                          m_config.encoderBPin,
                          m_config.encoderPPR,
                          m_config.invertEncoder,
                          m_config.index,
                          m_logger);

  if (!m_encoder->initialize()) {
    m_logger->logError(
        "Failed to initialize encoder for motor " + String(m_config.index),
        LogModule::MOTOR);
    return false;
  }

  // Initialize driver
  m_driver = new StepperDriver(m_config.stepPin,
                               m_config.dirPin,
                               m_config.enablePin,
                               m_config.invertDirection,
                               m_config.invertEnable,
                               m_config.index,
                               m_logger);

  if (!m_driver->initialize()) {
    m_logger->logError(
        "Failed to initialize driver for motor " + String(m_config.index),
        LogModule::MOTOR);
    return false;
  }

  // Check and configure the limit pins only if they're valid
  if (m_config.limitMinPin != 0xFF && m_config.limitMinPin <= 39) {
    pinMode(m_config.limitMinPin, INPUT_PULLUP);
  }

  if (m_config.limitMaxPin != 0xFF && m_config.limitMaxPin <= 39) {
    pinMode(m_config.limitMaxPin, INPUT_PULLUP);
  }

  m_trajectoryPlanner = new TrajectoryPlanner(m_config.maxVelocity,
                                              m_config.maxAcceleration,
                                              m_config.maxDeceleration,
                                              m_config.maxJerk,
                                              m_logger);
  // Initialize trajectory planner
  m_trajectoryPlanner->initialize();

  m_controller = new PIDController(m_config.pidKp,
                                   m_config.pidKi,
                                   m_config.pidKd,
                                   m_config.pidFf,
                                   1.0f / CONFIG_CONTROL_LOOP_FREQUENCY_HZ,
                                   m_logger);
  // Initialize PID controller
  m_controller->initialize();

  // Configure limits
  setSoftLimits(m_softLimitMin, m_softLimitMax, m_softLimitsEnabled);

  // Read initial limit switch states
  readLimitSwitches();

  // Motor initialized, but disabled
  m_state.status = MotorStatus::IDLE;

  m_logger->logInfo("Motor " + String(m_config.index) + " initialized successfully",
                    LogModule::MOTOR);

  return true;
}

void Motor::enable() {
  if (m_driver != nullptr) {
    m_driver->enable();
    m_controlMode = MotorControlMode::POSITION;  // Default to position
                                                 // mode when enabled
    m_state.status = MotorStatus::IDLE;

    m_logger->logInfo("Motor " + String(m_config.index) + " enabled",
                      LogModule::MOTOR);
  }
}

void Motor::disable() {
  if (m_driver != nullptr) {
    m_driver->disable();
    m_controlMode  = MotorControlMode::DISABLED_;
    m_state.status = MotorStatus::DISABLED_;

    m_logger->logInfo("Motor " + String(m_config.index) + " disabled",
                      LogModule::MOTOR);
  }
}

bool Motor::isEnabled() const {
  return m_driver != nullptr && m_driver->isEnabled();
}

void Motor::updateControl() {
  // Check if motor is disabled
  if (m_controlMode == MotorControlMode::DISABLED_) {
    // return;
  }

  // Get current time
  uint32_t currentTimeUs = micros();

  // Calculate time since last update
  uint32_t deltaTimeUs;
  if (currentTimeUs < m_lastControlUpdateUs) {
    // Handle timer overflow

    deltaTimeUs = (UINT32_MAX - m_lastControlUpdateUs) + currentTimeUs + 1;
  } else {
    deltaTimeUs = currentTimeUs - m_lastControlUpdateUs;
  }

  // Only update at specified interval
  if (deltaTimeUs < m_controlIntervalUs) {
    return;
  }

  // Update last control time
  m_lastControlUpdateUs = currentTimeUs;

  // Read limit switches
  readLimitSwitches();

  // Update encoder
  m_logger->logError("CALLING ENCODER UPDATE", LogModule::MOTOR);
  m_encoder->update(deltaTimeUs);

  // Update motor state
  m_state.currentPosition     = m_encoder->getPosition();
  m_state.currentVelocity     = m_encoder->getVelocity();
  m_state.currentAcceleration = m_encoder->getAcceleration();

  // Process based on control mode
  switch (m_controlMode) {
    case MotorControlMode::POSITION: processPositionControl(); break;

    case MotorControlMode::VELOCITY: processVelocityControl(); break;

    case MotorControlMode::HOMING: processHoming(); break;

    default:
      // Nothing to do for other modes
      break;
  }

  // Update motor state
  updateMotorState();

  // Log motor state periodically to avoid flooding logs
  static uint32_t lastLogTime = 0;
  if (m_logger && (millis() - lastLogTime > 1000)) {  // Log every second
    lastLogTime = millis();
    m_logger->logDebug("Motor " + String(m_config.index)
                           + " state: " + "pos=" + String(m_state.currentPosition)
                           + ", vel=" + String(m_state.currentVelocity)
                           + ", mode=" + controlModeToString(m_controlMode)
                           + ", status=" + motorStatusToString(m_state.status),
                       LogModule::MOTOR);
  }
}

void Motor::updateTrajectory() {
  // Existing implementation remains largely the same, but add
  // logging...

  // Check if motor is disabled
  if (m_controlMode == MotorControlMode::DISABLED_) {
    return;
  }

  // Get current time
  uint32_t currentTimeUs = micros();

  // Calculate time since last update
  uint32_t deltaTimeUs;
  if (currentTimeUs < m_lastTrajectoryUpdateUs) {
    // Handle timer overflow
    deltaTimeUs = (UINT32_MAX - m_lastTrajectoryUpdateUs) + currentTimeUs + 1;
  } else {
    deltaTimeUs = currentTimeUs - m_lastTrajectoryUpdateUs;
  }

  // Only update at specified interval
  if (deltaTimeUs < m_trajectoryIntervalUs) {
    return;
  }

  // Update last trajectory time
  m_lastTrajectoryUpdateUs = currentTimeUs;

  // Update trajectory
  m_trajectoryPlanner->update(deltaTimeUs);

  // If trajectory is complete and was in position mode, update the
  // target position
  if (m_trajectoryPlanner->isComplete()
      && m_controlMode == MotorControlMode::POSITION) {
    m_state.targetPosition =
        static_cast<int32_t>(m_trajectoryPlanner->getCurrentPosition());

    m_logger->logDebug("Motor " + String(m_config.index)
                           + " trajectory complete, target position set to "
                           + String(m_state.targetPosition),
                       LogModule::MOTOR);
  }
}

void Motor::setControlMode(MotorControlMode mode) {
  // Don't change mode if disabled
  if (m_controlMode == MotorControlMode::DISABLED_
      && mode != MotorControlMode::DISABLED_) {
    enable();
  }

  m_logger->logInfo("Motor " + String(m_config.index) + " control mode changed from "
                        + controlModeToString(m_controlMode) + " to "
                        + controlModeToString(mode),
                    LogModule::MOTOR);

  // Set new mode
  m_controlMode = mode;

  // Reset controller when changing modes
  m_controller->reset();
}

// Continue with similar changes to all methods, adding appropriate
// logging...

// For larger methods like setTargetPosition, moveToPosition, etc.,
// add logging at key points

void Motor::setTargetPosition(int32_t position) {
  // Check if within soft limits
  if (m_softLimitsEnabled && !isWithinSoftLimits(position)) {
    m_logger->logWarning("Motor " + String(m_config.index) + " target position "
                             + String(position) + " outside soft limits ["
                             + String(m_softLimitMin) + ", " + String(m_softLimitMax)
                             + "]",
                         LogModule::MOTOR);
    return;
  }

  // Set target position
  m_state.targetPosition = position;

  // Set control mode to position
  setControlMode(MotorControlMode::POSITION);

  // Plan trajectory
  m_trajectoryPlanner->planPositionMove(m_state.currentPosition,
                                        position,
                                        m_state.currentVelocity,
                                        m_config.maxVelocity,
                                        m_config.maxAcceleration,
                                        m_config.maxDeceleration,
                                        m_config.maxJerk);

  // Update state
  m_state.status = MotorStatus::MOVING;

  m_logger->logInfo("Motor " + String(m_config.index) + " target position set to "
                        + String(position)
                        + " (current: " + String(m_state.currentPosition) + ")",
                    LogModule::MOTOR);
}

void Motor::moveToPosition(int32_t position,
                           float   maxVelocity,
                           float   acceleration,
                           float   deceleration,
                           float   jerk) {
  // Check if within soft limits
  if (m_softLimitsEnabled && !isWithinSoftLimits(position)) {
    m_logger->logWarning("Motor " + String(m_config.index) + " target position "
                             + String(position) + " outside soft limits ["
                             + String(m_softLimitMin) + ", " + String(m_softLimitMax)
                             + "]",
                         LogModule::MOTOR);
    return;
  }

  // Set target position
  m_state.targetPosition = position;

  // Set control mode to position
  setControlMode(MotorControlMode::POSITION);

  // Plan trajectory
  m_trajectoryPlanner->planPositionMove(
      m_state.currentPosition,
      position,
      m_state.currentVelocity,
      maxVelocity > 0 ? maxVelocity : m_config.maxVelocity,
      acceleration > 0 ? acceleration : m_config.maxAcceleration,
      deceleration > 0 ? deceleration : m_config.maxDeceleration,
      jerk > 0 ? jerk : m_config.maxJerk);

  // Update state
  m_state.status = MotorStatus::MOVING;

  m_logger->logInfo(
      "Motor " + String(m_config.index) + " moving to position " + String(position)
          + " with v_max="
          + String(maxVelocity > 0 ? maxVelocity : m_config.maxVelocity) + ", a="
          + String(acceleration > 0 ? acceleration : m_config.maxAcceleration)
          + ", d="
          + String(deceleration > 0 ? deceleration : m_config.maxDeceleration),
      LogModule::MOTOR);
}

void Motor::emergencyStop() {
  // Set emergency stop flag
  m_emergencyStopActive = true;

  // Stop trajectory
  m_trajectoryPlanner->reset();

  // Set target velocity to zero
  m_state.targetVelocity = 0.0f;

  // Reset controller
  m_controller->reset();

  // Send stop command to driver
  if (m_driver != nullptr) {
    // Use fastest possible deceleration
    m_driver->setOutput(0.0f);
  }

  // Update state
  m_state.status = MotorStatus::IDLE;

  m_logger->logError("Motor " + String(m_config.index) + " emergency stop",
                     LogModule::MOTOR);
}

void Motor::abort() {
  // Stop trajectory with normal deceleration
  m_trajectoryPlanner->reset();

  // Set target velocity to zero
  m_state.targetVelocity = 0.0f;

  // Plan deceleration trajectory
  m_trajectoryPlanner->planVelocityMove(
      m_state.currentVelocity, 0.0f, m_config.maxDeceleration, m_config.maxJerk);

  // Update state
  m_state.status = MotorStatus::MOVING;

  m_logger->logInfo("Motor " + String(m_config.index) + " abort requested",
                    LogModule::MOTOR);
}

// Add the utility methods for converting enums to strings

String Motor::controlModeToString(MotorControlMode mode) const {
  switch (mode) {
    case MotorControlMode::DISABLED_: return "DISABLED";
    case MotorControlMode::OPEN_LOOP: return "OPEN_LOOP";
    case MotorControlMode::POSITION: return "POSITION";
    case MotorControlMode::VELOCITY: return "VELOCITY";
    case MotorControlMode::TORQUE: return "TORQUE";
    case MotorControlMode::HOMING: return "HOMING";
    default: return "UNKNOWN";
  }
}

String Motor::motorStatusToString(MotorStatus status) const {
  switch (status) {
    case MotorStatus::IDLE: return "IDLE";
    case MotorStatus::MOVING: return "MOVING";
    case MotorStatus::HOMING: return "HOMING";
    case MotorStatus::HOLDING: return "HOLDING";
    case MotorStatus::ERROR: return "ERROR";
    case MotorStatus::DISABLED_: return "DISABLED";
    default: return "UNKNOWN";
  }
}

String Motor::motorErrorToString(MotorError error) const {
  switch (error) {
    case MotorError::NONE: return "NONE";
    case MotorError::POSITION_ERROR: return "POSITION_ERROR";
    case MotorError::VELOCITY_ERROR: return "VELOCITY_ERROR";
    case MotorError::LIMIT_SWITCH_TRIGGERED: return "LIMIT_SWITCH_TRIGGERED";
    case MotorError::DRIVER_FAULT: return "DRIVER_FAULT";
    case MotorError::ENCODER_ERROR: return "ENCODER_ERROR";
    case MotorError::TIMEOUT: return "TIMEOUT";
    case MotorError::TRAJECTORY_ERROR: return "TRAJECTORY_ERROR";
    case MotorError::GENERAL_ERROR: return "GENERAL_ERROR";
    default: return "UNKNOWN";
  }
}

MotorControlMode Motor::getControlMode() const {
  return m_controlMode;
}

void Motor::setTargetVelocity(float velocity) {
  // Set target velocity
  m_state.targetVelocity = velocity;

  // Set control mode to velocity
  setControlMode(MotorControlMode::VELOCITY);

  // No need to plan trajectory for simple velocity control

  // Update state
  if (velocity != 0.0f) {
    m_state.status = MotorStatus::MOVING;
  } else {
    m_state.status = MotorStatus::IDLE;
  }
}

void Motor::setTargetVelocityWithAccel(float velocity,
                                       float acceleration,
                                       float jerk) {
  // Set target velocity
  m_state.targetVelocity = velocity;

  // Set control mode to velocity
  setControlMode(MotorControlMode::VELOCITY);

  // Plan velocity trajectory
  m_trajectoryPlanner->planVelocityMove(
      m_state.currentVelocity,
      velocity,
      acceleration > 0 ? acceleration : m_config.maxAcceleration,
      jerk > 0 ? jerk : m_config.maxJerk);

  // Update state
  if (velocity != 0.0f) {
    m_state.status = MotorStatus::MOVING;
  } else {
    m_state.status = MotorStatus::IDLE;
  }
}

void Motor::startHoming(int8_t direction, float velocity) {
  // Set homing direction and velocity
  int8_t homingDirection = direction > 0 ? 1 : -1;
  float  homingVelocity  = fabs(velocity);

  // Set control mode to homing
  setControlMode(MotorControlMode::HOMING);

  // Set target velocity based on direction
  m_state.targetVelocity = homingDirection * homingVelocity;

  // Update state
  m_state.isHomed = false;
  m_state.status  = MotorStatus::HOMING;
}

const MotorState& Motor::getState() const {
  return m_state;
}

uint8_t Motor::getIndex() const {
  return m_config.index;
}

int32_t Motor::getCurrentPosition() const {
  return m_state.currentPosition;
}

float Motor::getCurrentVelocity() const {
  return m_state.currentVelocity;
}

int32_t Motor::getTargetPosition() const {
  return m_state.targetPosition;
}

float Motor::getTargetVelocity() const {
  return m_state.targetVelocity;
}

bool Motor::isMoving() const {
  return m_state.status == MotorStatus::MOVING;
}

bool Motor::isPositionReached() const {
  return fabs(m_state.currentPosition - m_state.targetPosition)
         <= m_positionTolerance;
}

bool Motor::isVelocityReached() const {
  return fabs(m_state.currentVelocity - m_state.targetVelocity)
         <= m_velocityTolerance;
}

bool Motor::isHomed() const {
  return m_state.isHomed;
}

void Motor::resetPosition() {
  m_encoder->reset();
  m_state.currentPosition = 0;
  m_state.targetPosition  = 0;
}

void Motor::setPosition(int32_t position) {
  m_encoder->setPosition(position);
  m_state.currentPosition = position;
  m_state.targetPosition  = position;
}

void Motor::setPIDParameters(float kp, float ki, float kd, float ff) {
  m_controller->setGains(kp, ki, kd, ff);
}

PIDController* Motor::getPIDController() {
  return m_controller;
}

Encoder* Motor::getEncoder() {
  return m_encoder;
}

void Motor::setSoftLimits(int32_t min, int32_t max, bool enable) {
  m_softLimitMin      = min;
  m_softLimitMax      = max;
  m_softLimitsEnabled = enable;
}

void Motor::setLimitSwitchConfig(bool invertMin, bool invertMax, bool enable) {
  m_invertLimitMin       = invertMin;
  m_invertLimitMax       = invertMax;
  m_limitSwitchesEnabled = enable;
}

uint32_t Motor::getControlInterval() const {
  return m_controlIntervalUs;
}

uint32_t Motor::getTrajectoryUpdateInterval() const {
  return m_trajectoryIntervalUs;
}

void Motor::clearError() {
  m_state.error = MotorError::NONE;
}

MotorError Motor::checkErrors() {
  // Check for position error
  if (m_controlMode == MotorControlMode::POSITION
      && abs(m_state.currentPosition - m_state.targetPosition)
             > CONFIG_SAFETY_POSITION_TOLERANCE) {
    m_state.error = MotorError::POSITION_ERROR;
  }

  // Check for velocity error
  if (m_controlMode == MotorControlMode::VELOCITY
      && fabs(m_state.currentVelocity - m_state.targetVelocity)
             > CONFIG_SAFETY_VELOCITY_TOLERANCE) {
    m_state.error = MotorError::VELOCITY_ERROR;
  }

  // Check for limit switch errors
  if (m_limitSwitchesEnabled) {
    if (m_state.limitMinTriggered && m_state.currentVelocity < 0) {
      m_state.error = MotorError::LIMIT_SWITCH_TRIGGERED;
    }

    if (m_state.limitMaxTriggered && m_state.currentVelocity > 0) {
      m_state.error = MotorError::LIMIT_SWITCH_TRIGGERED;
    }
  }

  // Check for driver faults
  if (m_driver != nullptr && m_driver->checkFault()) {
    m_state.error = MotorError::DRIVER_FAULT;
  }

  return m_state.error;
}

bool Motor::readLimitSwitches() {
  bool limitTriggered = false;

  if (m_limitSwitchesEnabled) {
    // Read limit switch pins if configured and valid
    if (m_config.limitMinPin != 0xFF && m_config.limitMinPin <= 39) {
      m_limitMinState = digitalRead(m_config.limitMinPin) == LOW;
      if (m_invertLimitMin) {
        m_limitMinState = !m_limitMinState;
      }

      // Update state
      m_state.limitMinTriggered = m_limitMinState;

      if (m_limitMinState) {
        limitTriggered = true;
      }
    }

    if (m_config.limitMaxPin != 0xFF && m_config.limitMaxPin <= 39) {
      m_limitMaxState = digitalRead(m_config.limitMaxPin) == LOW;
      if (m_invertLimitMax) {
        m_limitMaxState = !m_limitMaxState;
      }

      // Update state
      m_state.limitMaxTriggered = m_limitMaxState;

      if (m_limitMaxState) {
        limitTriggered = true;
      }
    }
  }

  return limitTriggered;
}

bool Motor::isWithinSoftLimits(int32_t position) const {
  if (!m_softLimitsEnabled) {
    return true;
  }

  return position >= m_softLimitMin && position <= m_softLimitMax;
}

void Motor::applyControlOutput(float output) {
  if (m_driver != nullptr) {
    m_driver->setOutput(output);
  }

  // Store control output
  m_state.controlOutput = output;
  m_controlOutputBuffer.push(output);
}

void Motor::processPositionControl() {
  // If we have an active trajectory, use it for position reference
  float targetPosition;
  float targetVelocity;

  if (!m_trajectoryPlanner->isComplete()) {
    // Get position and velocity from trajectory
    targetPosition = m_trajectoryPlanner->getCurrentPosition();
    targetVelocity = m_trajectoryPlanner->getCurrentVelocity();
  } else {
    // Use direct target position
    targetPosition = static_cast<float>(m_state.targetPosition);
    targetVelocity = 0.0f;
  }

  // Perform PID control for position
  float controlOutput =
      m_controller->compute(targetPosition,
                            static_cast<float>(m_state.currentPosition),
                            targetVelocity  // Feed-forward with target velocity
      );

  // Apply control output to motor
  applyControlOutput(controlOutput);
}

void Motor::processVelocityControl() {
  // If we have an active trajectory, use it for velocity reference
  float targetVelocity;
  float targetAcceleration;

  if (!m_trajectoryPlanner->isComplete()) {
    // Get velocity and acceleration from trajectory
    targetVelocity     = m_trajectoryPlanner->getCurrentVelocity();
    targetAcceleration = m_trajectoryPlanner->getCurrentAcceleration();
  } else {
    // Use direct target velocity
    targetVelocity     = m_state.targetVelocity;
    targetAcceleration = 0.0f;
  }

  // Perform PID control for velocity
  float controlOutput = m_controller->compute(
      targetVelocity,
      m_state.currentVelocity,
      targetAcceleration  // Feed-forward with target acceleration
  );

  // Apply control output to motor
  applyControlOutput(controlOutput);
}

void Motor::processHoming() {
  // Check if we've hit a limit switch
  bool limitTriggered = false;

  if (m_state.targetVelocity > 0 && m_state.limitMaxTriggered) {
    // Homing in positive direction and hit max limit
    limitTriggered = true;
  } else if (m_state.targetVelocity < 0 && m_state.limitMinTriggered) {
    // Homing in negative direction and hit min limit
    limitTriggered = true;
  }

  if (limitTriggered) {
    // Stop movement
    m_state.targetVelocity = 0.0f;

    // Set position to zero or limit position
    if (m_state.limitMinTriggered) {
      setPosition(m_softLimitMin);
    } else if (m_state.limitMaxTriggered) {
      setPosition(m_softLimitMax);
    } else {
      // Should never happen
      setPosition(0);
    }

    // Mark as homed
    m_state.isHomed = true;

    // Switch to position control mode
    setControlMode(MotorControlMode::POSITION);

    // Update state
    m_state.status = MotorStatus::IDLE;
  } else {
    // Continue homing - use velocity control
    float controlOutput = m_controller->compute(m_state.targetVelocity,
                                                m_state.currentVelocity,
                                                0.0f  // No feed-forward
    );

    // Apply control output to motor
    applyControlOutput(controlOutput);
  }
}

void Motor::updateMotorState() {
  // Check if we need to change the state
  if (m_state.status == MotorStatus::MOVING) {
    // Check if we've reached the target
    if (m_controlMode == MotorControlMode::POSITION) {
      if (isPositionReached()
          && fabs(m_state.currentVelocity) < m_velocityTolerance) {
        m_state.status = MotorStatus::IDLE;
      }
    } else if (m_controlMode == MotorControlMode::VELOCITY) {
      if (m_state.targetVelocity == 0.0f
          && fabs(m_state.currentVelocity) < m_velocityTolerance) {
        m_state.status = MotorStatus::IDLE;
      }
    }
  }

  // Check for errors
  checkErrors();

  // If error detected, update state
  if (m_state.error != MotorError::NONE) {
    m_state.status = MotorStatus::ERROR;
  }
}

void Motor::setError(MotorError error) {
  m_state.error  = error;
  m_state.status = MotorStatus::ERROR;
}

void Motor::updateSensors(uint32_t deltaTimeUs) {
  // Update encoder
  m_encoder->update(deltaTimeUs);

  // Update motor state with encoder data
  m_state.currentPosition     = m_encoder->getPosition();
  m_state.currentVelocity     = m_encoder->getVelocity();
  m_state.currentAcceleration = m_encoder->getAcceleration();
}
// End of Code