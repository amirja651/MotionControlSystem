/*
 * ESP32 High-Precision Motion Control System
 * Command Protocol Implementation
 */

#include "CommandProtocol.h"

CommandProtocol::CommandProtocol(SystemManager* systemManager,
                                 SerialCommand* serialCommand,
                                 Logger*        logger)
    : m_systemManager(systemManager),
      m_serialCommand(serialCommand),
      m_logger(logger),
      m_binaryProtocolEnabled(false),
      m_receiveBufferIndex(0) {
    // Clear receive buffer
    memset(m_receiveBuffer, 0, sizeof(m_receiveBuffer));

    m_logger->logInfo("Command protocol instance created",
                      LogModule::COMMAND_HANDLER);
}

bool CommandProtocol::initialize() {
    m_logger->logInfo("Command protocol initialized",
                      LogModule::COMMAND_HANDLER);

    // Initialization successful
    return true;
}

void CommandProtocol::processCommands() {
    // Only process if binary protocol is enabled
    if (!m_binaryProtocolEnabled) {
        return;
    }

    // Check for available serial data
    while (Serial.available() > 0) {
        // Read a byte
        uint8_t byte = Serial.read();

        // Add to buffer if space available
        if (m_receiveBufferIndex < sizeof(m_receiveBuffer)) {
            m_receiveBuffer[m_receiveBufferIndex++] = byte;
        }

        // Check for complete packet
        CommandPacket packet;
        if (findCommandPacket(packet)) {
            m_logger->logDebug(
                "Command packet found: type="
                    + commandTypeToString(
                        static_cast<CommandType>(packet.commandType))
                    + ", id=" + String(packet.commandId)
                    + ", length=" + String(packet.length),
                LogModule::COMMAND_HANDLER);

            // Process the packet
            processCommandPacket(packet);

            // Remove processed packet from buffer
            uint8_t packetSize =
                5 + packet.length;  // Header (4) + data + checksum (1)

            if (packetSize < m_receiveBufferIndex) {
                // Shift remaining data to start of buffer
                memmove(m_receiveBuffer,
                        m_receiveBuffer + packetSize,
                        m_receiveBufferIndex - packetSize);
                m_receiveBufferIndex -= packetSize;
            } else {
                // Buffer empty
                m_receiveBufferIndex = 0;
            }
        }
    }
}

bool CommandProtocol::processCommandPacket(const CommandPacket& packet) {
    // Prepare response packet
    ResponsePacket response;
    response.startByte   = 0xBB;
    response.commandType = packet.commandType;
    response.commandId   = packet.commandId;
    response.status      = 0;  // Success by default
    response.length      = 0;

    bool success = false;

    // Process command based on type
    switch (static_cast<CommandType>(packet.commandType)) {
        case CommandType::SYSTEM_COMMAND:
            m_logger->logInfo(
                "Processing system command: id=" + String(packet.commandId),
                LogModule::COMMAND_HANDLER);
            success = processSystemCommand(
                packet.commandId, packet.data, packet.length, response);
            break;

        case CommandType::MOTION_COMMAND:
            m_logger->logInfo(
                "Processing motion command: id=" + String(packet.commandId),
                LogModule::COMMAND_HANDLER);
            success = processMotionCommand(
                packet.commandId, packet.data, packet.length, response);
            break;

        case CommandType::STATUS_COMMAND:
            m_logger->logDebug(
                "Processing status command: id=" + String(packet.commandId),
                LogModule::COMMAND_HANDLER);
            success = processStatusCommand(
                packet.commandId, packet.data, packet.length, response);
            break;

        case CommandType::CONFIG_COMMAND:
            m_logger->logInfo(
                "Processing config command: id=" + String(packet.commandId),
                LogModule::COMMAND_HANDLER);
            success = processConfigCommand(
                packet.commandId, packet.data, packet.length, response);
            break;

        case CommandType::DEBUG_COMMAND:
            m_logger->logInfo(
                "Processing debug command: id=" + String(packet.commandId),
                LogModule::COMMAND_HANDLER);
            success = processDebugCommand(
                packet.commandId, packet.data, packet.length, response);
            break;

        default:
            // Unknown command type
            response.status = 1;  // General error
            m_logger->logError(
                "Unknown command type: " + String(packet.commandType),
                LogModule::COMMAND_HANDLER);
            break;
    }

    // Set status code
    if (!success) {
        response.status = 1;  // General error
        m_logger->logWarning(
            "Command processing failed: type="
                + commandTypeToString(
                    static_cast<CommandType>(packet.commandType))
                + ", id=" + String(packet.commandId),
            LogModule::COMMAND_HANDLER);
    }

    // Calculate checksum
    uint8_t* responseData = reinterpret_cast<uint8_t*>(&response);
    response.checksum = calculateChecksum(responseData, 4 + response.length);

    // Send response
    sendResponse(response);

    return success;
}

void CommandProtocol::sendResponse(const ResponsePacket& packet) {
    // Send packet bytes
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&packet);

    // Calculate total packet size
    uint8_t totalSize = 5 + packet.length;  // Header (4) + data + checksum (1)

    // Send all bytes
    Serial.write(data, totalSize);

    m_logger->logDebug("Sent response: type=" + String(packet.commandType)
                           + ", id=" + String(packet.commandId)
                           + ", status=" + String(packet.status)
                           + ", length=" + String(packet.length),
                       LogModule::COMMAND_HANDLER);
}

void CommandProtocol::enableBinaryProtocol(bool enable) {
    m_binaryProtocolEnabled = enable;

    // Clear receive buffer when enabling/disabling
    m_receiveBufferIndex = 0;
    memset(m_receiveBuffer, 0, sizeof(m_receiveBuffer));

    m_logger->logInfo(
        "Binary protocol " + String(enable ? "enabled" : "disabled"),
        LogModule::COMMAND_HANDLER);
}

bool CommandProtocol::isBinaryProtocolEnabled() const {
    return m_binaryProtocolEnabled;
}

bool CommandProtocol::processSystemCommand(uint8_t         commandId,
                                           const uint8_t*  data,
                                           uint8_t         length,
                                           ResponsePacket& response) {
    // System commands:
    // 0x01: Get system status
    // 0x02: Reset system
    // 0x03: Save configuration
    // 0x04: Load configuration
    // 0x05: Emergency stop
    // 0x06: Reset emergency stop

    switch (commandId) {
        case 0x01: {  // Get system status
            // Check if system manager is available
            if (m_systemManager == nullptr) {
                response.status = 2;  // System not available
                m_logger->logError(
                    "System command 0x01 failed: System manager not available",
                    LogModule::COMMAND_HANDLER);
                return false;
            }

            // Get system state
            SystemState state = m_systemManager->getSystemState();

            // Prepare response data
            response.data[0] = static_cast<uint8_t>(state);
            response.data[1] = m_systemManager->isEmergencyStop() ? 1 : 0;

            // Get uptime (32-bit value)
            uint32_t uptime = m_systemManager->getUptimeMs();
            memcpy(&response.data[2], &uptime, sizeof(uint32_t));

            response.length = 6;

            m_logger->logDebug("System status command: state="
                                   + String(static_cast<int>(state))
                                   + ", emergency="
                                   + String(m_systemManager->isEmergencyStop())
                                   + ", uptime=" + String(uptime) + "ms",
                               LogModule::COMMAND_HANDLER);

            return true;
        }

        case 0x02: {  // Reset system
            // Check if system manager is available
            if (m_systemManager == nullptr) {
                response.status = 2;  // System not available
                m_logger->logError(
                    "System command 0x02 failed: System manager not available",
                    LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logInfo("System reset command received",
                              LogModule::COMMAND_HANDLER);

            // Reset the system
            m_systemManager->resetSystem();

            // No response data
            response.length = 0;
            return true;
        }

        case 0x03: {  // Save configuration
            // Check if system manager is available
            if (m_systemManager == nullptr) {
                response.status = 2;  // System not available
                m_logger->logError(
                    "System command 0x03 failed: System manager not available",
                    LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logInfo("Save configuration command received",
                              LogModule::COMMAND_HANDLER);

            // Save configuration
            bool success = m_systemManager->saveSystemConfiguration();

            // Response data: success flag
            response.data[0] = success ? 1 : 0;
            response.length  = 1;

            m_logger->logInfo(
                "Save configuration " + String(success ? "success" : "failed"),
                LogModule::COMMAND_HANDLER);

            return success;
        }

        case 0x04: {  // Load configuration
            // Check if system manager is available
            if (m_systemManager == nullptr) {
                response.status = 2;  // System not available
                m_logger->logError(
                    "System command 0x04 failed: System manager not available",
                    LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logInfo("Load configuration command received",
                              LogModule::COMMAND_HANDLER);

            // Load configuration
            bool success = m_systemManager->loadSystemConfiguration();

            // Response data: success flag
            response.data[0] = success ? 1 : 0;
            response.length  = 1;

            m_logger->logInfo(
                "Load configuration " + String(success ? "success" : "failed"),
                LogModule::COMMAND_HANDLER);

            return success;
        }

        case 0x05: {  // Emergency stop
            // Check if system manager is available
            if (m_systemManager == nullptr) {
                response.status = 2;  // System not available
                m_logger->logError(
                    "System command 0x05 failed: System manager not available",
                    LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logWarning("Emergency stop command received",
                                 LogModule::COMMAND_HANDLER);

            // Trigger emergency stop
            m_systemManager->triggerEmergencyStop(
                SafetyCode::EMERGENCY_STOP_PRESSED);

            // No response data
            response.length = 0;
            return true;
        }

        case 0x06: {  // Reset emergency stop
            // Check if system manager is available
            if (m_systemManager == nullptr) {
                response.status = 2;  // System not available
                m_logger->logError(
                    "System command 0x06 failed: System manager not available",
                    LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logInfo("Reset emergency stop command received",
                              LogModule::COMMAND_HANDLER);

            // Reset emergency stop
            bool success = m_systemManager->resetEmergencyStop();

            // Response data: success flag
            response.data[0] = success ? 1 : 0;
            response.length  = 1;

            m_logger->logInfo("Reset emergency stop "
                                  + String(success ? "success" : "failed"),
                              LogModule::COMMAND_HANDLER);

            return success;
        }

        default:
            // Unknown command ID
            response.status = 3;  // Unknown command
            m_logger->logWarning(
                "Unknown system command ID: 0x" + String(commandId, HEX),
                LogModule::COMMAND_HANDLER);
            return false;
    }
}

bool CommandProtocol::processMotionCommand(uint8_t         commandId,
                                           const uint8_t*  data,
                                           uint8_t         length,
                                           ResponsePacket& response) {
    // Motion commands:
    // 0x01: Enable motor
    // 0x02: Disable motor
    // 0x03: Move to position
    // 0x04: Set velocity
    // 0x05: Stop motor
    // 0x06: Emergency stop motor
    // 0x07: Home motor
    // 0x08: Reset position

    // Check if system manager is available
    if (m_systemManager == nullptr) {
        response.status = 2;  // System not available
        m_logger->logError(
            "Motion command failed: System manager not available",
            LogModule::COMMAND_HANDLER);
        return false;
    }

    // Get motor manager
    MotorManager* motorManager = m_systemManager->getMotorManager();
    if (motorManager == nullptr) {
        response.status = 2;  // Motor manager not available
        m_logger->logError(
            "Motion command failed: Motor manager not available",
            LogModule::COMMAND_HANDLER);
        return false;
    }

    switch (commandId) {
        case 0x01: {  // Enable motor
            // Check data length
            if (length < 1) {
                response.status = 4;  // Invalid parameters
                m_logger->logError("Enable motor command: Invalid parameters",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            // Get motor index
            uint8_t motorIndex = data[0];

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                m_logger->logError("Enable motor command: Motor "
                                       + String(motorIndex) + " not found",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logInfo("Enabling motor " + String(motorIndex),
                              LogModule::COMMAND_HANDLER);

            // Enable motor
            motor->enable();

            // No response data
            response.length = 0;
            return true;
        }

        case 0x02: {  // Disable motor
            // Check data length
            if (length < 1) {
                response.status = 4;  // Invalid parameters
                m_logger->logError("Disable motor command: Invalid parameters",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            // Get motor index
            uint8_t motorIndex = data[0];

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                m_logger->logError("Disable motor command: Motor "
                                       + String(motorIndex) + " not found",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logInfo("Disabling motor " + String(motorIndex),
                              LogModule::COMMAND_HANDLER);

            // Disable motor
            motor->disable();

            // No response data
            response.length = 0;
            return true;
        }

        case 0x03: {  // Move to position
            // Check data length
            if (length < 13) {
                response.status = 4;  // Invalid parameters
                m_logger->logError(
                    "Move to position command: Invalid parameters (length="
                        + String(length) + ", expected at least 13)",
                    LogModule::COMMAND_HANDLER);
                return false;
            }

            // Get parameters
            uint8_t motorIndex = data[0];

            int32_t position;
            memcpy(&position, &data[1], sizeof(int32_t));

            float velocity, acceleration, deceleration;
            memcpy(&velocity, &data[5], sizeof(float));
            memcpy(&acceleration, &data[9], sizeof(float));

            // Deceleration is optional, use acceleration if not provided
            if (length >= 17) {
                memcpy(&deceleration, &data[13], sizeof(float));
            } else {
                deceleration = acceleration;
            }

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                m_logger->logError("Move to position command: Motor "
                                       + String(motorIndex) + " not found",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logInfo("Moving motor " + String(motorIndex)
                                  + " to position " + String(position)
                                  + " with velocity=" + String(velocity)
                                  + ", accel=" + String(acceleration)
                                  + ", decel=" + String(deceleration),
                              LogModule::COMMAND_HANDLER);

            // Enable motor if not enabled
            if (!motor->isEnabled()) {
                motor->enable();
            }

            // Move to position
            motor->moveToPosition(
                position, velocity, acceleration, deceleration);

            // No response data
            response.length = 0;
            return true;
        }

        case 0x04: {  // Set velocity
            // Check data length
            if (length < 9) {
                response.status = 4;  // Invalid parameters
                m_logger->logError("Set velocity command: Invalid parameters",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            // Get parameters
            uint8_t motorIndex = data[0];

            float velocity, acceleration;
            memcpy(&velocity, &data[1], sizeof(float));
            memcpy(&acceleration, &data[5], sizeof(float));

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                m_logger->logError("Set velocity command: Motor "
                                       + String(motorIndex) + " not found",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logInfo("Setting motor " + String(motorIndex)
                                  + " velocity to " + String(velocity)
                                  + " with acceleration "
                                  + String(acceleration),
                              LogModule::COMMAND_HANDLER);

            // Enable motor if not enabled
            if (!motor->isEnabled()) {
                motor->enable();
            }

            // Set velocity
            motor->setTargetVelocityWithAccel(velocity, acceleration);

            // No response data
            response.length = 0;
            return true;
        }

        case 0x05: {  // Stop motor
            // Check data length
            if (length < 1) {
                response.status = 4;  // Invalid parameters
                m_logger->logError("Stop motor command: Invalid parameters",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            // Get motor index
            uint8_t motorIndex = data[0];

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                m_logger->logError("Stop motor command: Motor "
                                       + String(motorIndex) + " not found",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logInfo("Stopping motor " + String(motorIndex),
                              LogModule::COMMAND_HANDLER);

            // Stop motor
            motor->emergencyStop();  // stop(false);

            // No response data
            response.length = 0;
            return true;
        }

        case 0x06: {  // Emergency stop motor
            // Check data length
            if (length < 1) {
                response.status = 4;  // Invalid parameters
                m_logger->logError(
                    "Emergency stop motor command: Invalid parameters",
                    LogModule::COMMAND_HANDLER);
                return false;
            }

            // Get motor index
            uint8_t motorIndex = data[0];

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                m_logger->logError("Emergency stop motor command: Motor "
                                       + String(motorIndex) + " not found",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logWarning(
                "Emergency stopping motor " + String(motorIndex),
                LogModule::COMMAND_HANDLER);

            // Emergency stop motor
            motor->emergencyStop();

            // No response data
            response.length = 0;
            return true;
        }

        case 0x07: {  // Home motor
            // Check data length
            if (length < 2) {
                response.status = 4;  // Invalid parameters
                m_logger->logError("Home motor command: Invalid parameters",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            // Get parameters
            uint8_t motorIndex = data[0];
            int8_t  direction  = static_cast<int8_t>(data[1]);

            // Default velocity
            float velocity = CONFIG_DEFAULT_MAX_VELOCITY / 2.0f;

            // Optional velocity parameter
            if (length >= 6) {
                memcpy(&velocity, &data[2], sizeof(float));
            }

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                m_logger->logError("Home motor command: Motor "
                                       + String(motorIndex) + " not found",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logInfo("Homing motor " + String(motorIndex)
                                  + " with direction " + String(direction)
                                  + " at velocity " + String(velocity),
                              LogModule::COMMAND_HANDLER);

            // Enable motor if not enabled
            if (!motor->isEnabled()) {
                motor->enable();
            }

            // Start homing
            motor->startHoming(direction, velocity);

            // No response data
            response.length = 0;
            return true;
        }

        case 0x08: {  // Reset position
            // Check data length
            if (length < 5) {
                response.status = 4;  // Invalid parameters
                m_logger->logError(
                    "Reset position command: Invalid parameters",
                    LogModule::COMMAND_HANDLER);
                return false;
            }

            // Get parameters
            uint8_t motorIndex = data[0];

            int32_t position = 0;
            if (length >= 5) {
                memcpy(&position, &data[1], sizeof(int32_t));
            }

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                m_logger->logError("Reset position command: Motor "
                                       + String(motorIndex) + " not found",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            m_logger->logInfo("Resetting motor " + String(motorIndex)
                                  + " position to " + String(position),
                              LogModule::COMMAND_HANDLER);

            // Set position
            motor->setPosition(position);

            // No response data
            response.length = 0;
            return true;
        }

        default:
            // Unknown command ID
            response.status = 3;  // Unknown command
            m_logger->logWarning(
                "Unknown motion command ID: 0x" + String(commandId, HEX),
                LogModule::COMMAND_HANDLER);
            return false;
    }
}

bool CommandProtocol::processStatusCommand(uint8_t         commandId,
                                           const uint8_t*  data,
                                           uint8_t         length,
                                           ResponsePacket& response) {
    // Status commands:
    // 0x01: Get motor status
    // 0x02: Get motor position
    // 0x03: Get motor velocity
    // 0x04: Get all motor positions
    // 0x05: Get system metrics

    // Check if system manager is available
    if (m_systemManager == nullptr) {
        response.status = 2;  // System not available
        m_logger->logError(
            "Status command failed: System manager not available",
            LogModule::COMMAND_HANDLER);
        return false;
    }

    // Get motor manager
    MotorManager* motorManager = m_systemManager->getMotorManager();
    if (motorManager == nullptr) {
        response.status = 2;  // Motor manager not available
        m_logger->logError(
            "Status command failed: Motor manager not available",
            LogModule::COMMAND_HANDLER);
        return false;
    }

    switch (commandId) {
        case 0x01: {  // Get motor status
            // Check data length
            if (length < 1) {
                response.status = 4;  // Invalid parameters
                m_logger->logError(
                    "Get motor status command: Invalid parameters",
                    LogModule::COMMAND_HANDLER);
                return false;
            }

            // Get motor index
            uint8_t motorIndex = data[0];

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                m_logger->logError("Get motor status command: Motor "
                                       + String(motorIndex) + " not found",
                                   LogModule::COMMAND_HANDLER);
                return false;
            }

            // Get motor state
            const MotorState& state = motor->getState();

            // Prepare response data
            response.data[0] = motorIndex;
            response.data[1] = static_cast<uint8_t>(state.status);
            response.data[2] = static_cast<uint8_t>(state.error);
            response.data[3] = motor->isEnabled() ? 1 : 0;
            response.data[4] = motor->isMoving() ? 1 : 0;
            response.data[5] = motor->isHomed() ? 1 : 0;
            response.data[6] = state.limitMinTriggered ? 1 : 0;
            response.data[7] = state.limitMaxTriggered ? 1 : 0;

            response.length = 8;
            return true;
        }

        case 0x02: {  // Get motor position
            // Check data length
            if (length < 1) {
                response.status = 4;  // Invalid parameters
                return false;
            }

            // Get motor index
            uint8_t motorIndex = data[0];

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                return false;
            }

            // Get motor position
            int32_t position = motor->getCurrentPosition();

            // Prepare response data
            response.data[0] = motorIndex;
            memcpy(&response.data[1], &position, sizeof(int32_t));

            response.length = 5;
            return true;
        }

        case 0x03: {  // Get motor velocity
            // Check data length
            if (length < 1) {
                response.status = 4;  // Invalid parameters
                return false;
            }

            // Get motor index
            uint8_t motorIndex = data[0];

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                return false;
            }

            // Get motor velocity
            float velocity = motor->getCurrentVelocity();

            // Prepare response data
            response.data[0] = motorIndex;
            memcpy(&response.data[1], &velocity, sizeof(float));

            response.length = 5;
            return true;
        }

        case 0x04: {  // Get all motor positions
            // Get motor count
            uint8_t motorCount = motorManager->getMotorCount();

            // Limit to max response data size
            if (motorCount > 6) {
                motorCount =
                    6;  // 6 motors * (1 index + 4 position bytes) = 30 bytes
            }

            // Prepare response data
            response.data[0] = motorCount;

            uint8_t dataIndex = 1;
            for (uint8_t i = 0; i < motorCount; i++) {
                // Get motor
                Motor* motor = motorManager->getMotor(i);
                if (motor != nullptr) {
                    // Add motor index
                    response.data[dataIndex++] = i;

                    // Add motor position
                    int32_t position = motor->getCurrentPosition();
                    memcpy(
                        &response.data[dataIndex], &position, sizeof(int32_t));
                    dataIndex += sizeof(int32_t);
                }
            }

            response.length = dataIndex;
            return true;
        }

        case 0x05: {  // Get system metrics
            // Prepare response data

            // CPU usage
            float cpu0 = m_systemManager->getCPUUsage(0);
            float cpu1 = m_systemManager->getCPUUsage(1);
            memcpy(&response.data[0], &cpu0, sizeof(float));
            memcpy(&response.data[4], &cpu1, sizeof(float));

            // Memory
            uint32_t freeMemory = m_systemManager->getFreeMemory();
            memcpy(&response.data[8], &freeMemory, sizeof(uint32_t));

            // Uptime
            uint32_t uptime = m_systemManager->getUptimeMs();
            memcpy(&response.data[12], &uptime, sizeof(uint32_t));

            response.length = 16;
            return true;
        }

        default:
            // Unknown command ID
            response.status = 3;  // Unknown command
            return false;
    }
}

bool CommandProtocol::processConfigCommand(uint8_t         commandId,
                                           const uint8_t*  data,
                                           uint8_t         length,
                                           ResponsePacket& response) {
    // Configuration commands:
    // 0x01: Get PID parameters
    // 0x02: Set PID parameters
    // 0x03: Get motor profile parameters
    // 0x04: Set motor profile parameters
    // 0x05: Get soft limits
    // 0x06: Set soft limits

    // Check if system manager is available
    if (m_systemManager == nullptr) {
        response.status = 2;  // System not available
        return false;
    }

    // Get motor manager
    MotorManager* motorManager = m_systemManager->getMotorManager();
    if (motorManager == nullptr) {
        response.status = 2;  // Motor manager not available
        return false;
    }

    switch (commandId) {
        case 0x01: {  // Get PID parameters
            // Check data length
            if (length < 1) {
                response.status = 4;  // Invalid parameters
                return false;
            }

            // Get motor index
            uint8_t motorIndex = data[0];

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                return false;
            }

            // Get PID controller
            PIDController* pid = motor->getPIDController();

            // Prepare response data
            response.data[0] = motorIndex;

            float kp = pid->getProportionalTerm();
            float ki = pid->getIntegralTerm();
            float kd = pid->getDerivativeTerm();
            float ff = pid->getFeedForwardTerm();

            memcpy(&response.data[1], &kp, sizeof(float));
            memcpy(&response.data[5], &ki, sizeof(float));
            memcpy(&response.data[9], &kd, sizeof(float));
            memcpy(&response.data[13], &ff, sizeof(float));

            response.length = 17;
            return true;
        }

        case 0x02: {  // Set PID parameters
            // Check data length
            if (length < 17) {
                response.status = 4;  // Invalid parameters
                return false;
            }

            // Get parameters
            uint8_t motorIndex = data[0];

            float kp, ki, kd, ff;
            memcpy(&kp, &data[1], sizeof(float));
            memcpy(&ki, &data[5], sizeof(float));
            memcpy(&kd, &data[9], sizeof(float));
            memcpy(&ff, &data[13], sizeof(float));

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                return false;
            }

            // Set PID parameters
            PIDController* pid = motor->getPIDController();
            pid->setGains(kp, ki, kd, ff);

            // No response data
            response.length = 0;
            return true;
        }

        case 0x03: {  // Get motor profile parameters
            // Check data length
            if (length < 1) {
                response.status = 4;  // Invalid parameters
                return false;
            }

            // Get motor index
            uint8_t motorIndex = data[0];

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                return false;
            }

            // Get motor state
            const MotorState& state = motor->getState();

            // Prepare response data
            response.data[0] = motorIndex;

            float maxVelocity  = state.targetVelocity;
            float acceleration = state.targetAcceleration;

            memcpy(&response.data[1], &maxVelocity, sizeof(float));
            memcpy(&response.data[5], &acceleration, sizeof(float));

            response.length = 9;
            return true;
        }

        case 0x04: {  // Set motor profile parameters
            // Check data length
            if (length < 9) {
                response.status = 4;  // Invalid parameters
                return false;
            }

            // Get parameters
            uint8_t motorIndex = data[0];

            float maxVelocity, acceleration;
            memcpy(&maxVelocity, &data[1], sizeof(float));
            memcpy(&acceleration, &data[5], sizeof(float));

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                return false;
            }

            // TODO: Update motor profile parameters
            // This would require adding a method to the Motor class

            // No response data
            response.length = 0;
            return true;
        }

        case 0x05: {  // Get soft limits
            // Check data length
            if (length < 1) {
                response.status = 4;  // Invalid parameters
                return false;
            }

            // Get motor index
            uint8_t motorIndex = data[0];

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                return false;
            }

            // TODO: Get soft limits
            // This would require adding a method to the Motor class

            // For now, return placeholder data
            response.data[0] = motorIndex;

            int32_t minLimit = -1000000;
            int32_t maxLimit = 1000000;
            uint8_t enabled  = 1;

            memcpy(&response.data[1], &minLimit, sizeof(int32_t));
            memcpy(&response.data[5], &maxLimit, sizeof(int32_t));
            response.data[9] = enabled;

            response.length = 10;
            return true;
        }

        case 0x06: {  // Set soft limits
            // Check data length
            if (length < 10) {
                response.status = 4;  // Invalid parameters
                return false;
            }

            // Get parameters
            uint8_t motorIndex = data[0];

            int32_t minLimit, maxLimit;
            uint8_t enabled;

            memcpy(&minLimit, &data[1], sizeof(int32_t));
            memcpy(&maxLimit, &data[5], sizeof(int32_t));
            enabled = data[9];

            // Get motor
            Motor* motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response.status = 5;  // Motor not found
                return false;
            }

            // Set soft limits
            motor->setSoftLimits(minLimit, maxLimit, enabled != 0);

            // No response data
            response.length = 0;
            return true;
        }

        default:
            // Unknown command ID
            response.status = 3;  // Unknown command
            return false;
    }
}

bool CommandProtocol::processDebugCommand(uint8_t         commandId,
                                          const uint8_t*  data,
                                          uint8_t         length,
                                          ResponsePacket& response) {
    // Debug commands:
    // 0x01: Set log level
    // 0x02: Get log buffer
    // 0x03: Clear log buffer
    // 0x04: Enable binary protocol
    // 0x05: Disable binary protocol

    // Check if system manager is available
    if (m_systemManager == nullptr) {
        response.status = 2;  // System not available
        return false;
    }

    // Get logger
    Logger* logger = m_systemManager->getLogger();
    if (logger == nullptr) {
        response.status = 2;  // Logger not available
        return false;
    }

    switch (commandId) {
        case 0x01: {  // Set log level
            // Check data length
            if (length < 1) {
                response.status = 4;  // Invalid parameters
                return false;
            }

            // Get log level
            uint8_t level = data[0];

            // Validate level
            if (level > 5) {
                response.status = 4;  // Invalid parameters
                return false;
            }

            // Set log level
            logger->setLogLevel(static_cast<LogLevel>(level));

            // No response data
            response.length = 0;
            return true;
        }

        case 0x02: {  // Get log buffer
            // Get log count
            size_t logCount = logger->getLogCount();

            // Limit to max response data size
            if (logCount > 5) {
                logCount = 5;  // Limit to 5 log entries
            }

            // Prepare response data
            response.data[0] = static_cast<uint8_t>(logCount);

            uint8_t dataIndex = 1;
            for (size_t i = 0; i < logCount; i++) {
                const LogEntry* entry = logger->getLogEntry(i);
                if (entry != nullptr) {
                    // Add log level
                    response.data[dataIndex++] =
                        static_cast<uint8_t>(entry->level);

                    // Add timestamp (32-bit)
                    memcpy(&response.data[dataIndex],
                           &entry->timestamp,
                           sizeof(uint32_t));
                    dataIndex += sizeof(uint32_t);

                    // Add message (truncated to fit)
                    uint8_t messageLength      = entry->message.length() > 20
                                                     ? 20
                                                     : entry->message.length();
                    response.data[dataIndex++] = messageLength;

                    memcpy(&response.data[dataIndex],
                           entry->message.c_str(),
                           messageLength);
                    dataIndex += messageLength;
                }
            }

            response.length = dataIndex;
            return true;
        }

        case 0x03: {  // Clear log buffer
            // Clear log buffer
            logger->clearLogBuffer();

            // No response data
            response.length = 0;
            return true;
        }

        case 0x04: {  // Enable binary protocol
            // Enable binary protocol
            enableBinaryProtocol(true);

            // No response data
            response.length = 0;
            return true;
        }

        case 0x05: {  // Disable binary protocol
            // Disable binary protocol
            enableBinaryProtocol(false);

            // No response data
            response.length = 0;
            return true;
        }

        default:
            // Unknown command ID
            response.status = 3;  // Unknown command
            return false;
    }
}

uint8_t CommandProtocol::calculateChecksum(const uint8_t* data,
                                           uint8_t        length) {
    uint8_t checksum = 0;

    // XOR all bytes
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }

    return checksum;
}

bool CommandProtocol::findCommandPacket(CommandPacket& packet) {
    // Search for start byte
    for (uint8_t i = 0; i < m_receiveBufferIndex; i++) {
        if (m_receiveBuffer[i] == 0xAA) {
            // Check if we have enough bytes for a complete packet header
            if (i + 4 > m_receiveBufferIndex) {
                return false;  // Not enough bytes
            }

            // Parse header
            uint8_t commandType = m_receiveBuffer[i + 1];
            uint8_t commandId   = m_receiveBuffer[i + 2];
            uint8_t length      = m_receiveBuffer[i + 3];

            // Check if we have enough bytes for complete packet
            if (i + 4 + length + 1 > m_receiveBufferIndex) {
                return false;  // Not enough bytes
            }

            // Extract data
            memcpy(packet.data, &m_receiveBuffer[i + 4], length);

            // Extract checksum
            uint8_t checksum = m_receiveBuffer[i + 4 + length];

            // Verify checksum
            uint8_t calculatedChecksum =
                calculateChecksum(&m_receiveBuffer[i], 4 + length);

            if (checksum == calculatedChecksum) {
                // Valid packet
                packet.startByte   = 0xAA;
                packet.commandType = commandType;
                packet.commandId   = commandId;
                packet.length      = length;
                packet.checksum    = checksum;

                return true;
            }
        }
    }

    return false;
}
// End of Code