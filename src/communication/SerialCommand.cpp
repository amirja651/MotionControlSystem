/*
 * ESP32 High-Precision Motion Control System
 * Serial Command Interface Implementation
 */

#include "SerialCommand.h"

SerialCommand::SerialCommand(SystemManager *systemManager)
    : m_systemManager(systemManager), m_commandBufferIndex(0), m_lastCommandTimeMs(0) {
    // Clear command buffer
    memset(m_commandBuffer, 0, CONFIG_COMMAND_BUFFER_SIZE);

    // Pre-allocate space for commands to avoid reallocation issues
    m_commands.reserve(20);
}

bool SerialCommand::initialize() {
    // Register built-in commands
    registerBuiltInCommands();
    return true;
}

void SerialCommand::begin(uint32_t baudRate) {
    // Initialize serial if not already initialized
    if (!Serial) {
        Serial.begin(baudRate);
        delay(100);  // Give serial time to initialize
    }
}

void SerialCommand::processCommands() {
    // Check for available serial data
    while (Serial.available() > 0) {
        // Read a byte
        char c = Serial.read();

        // Process the byte
        if (c == '\r' || c == '\n') {
            // End of command
            if (m_commandBufferIndex > 0) {
                // Null-terminate the command
                m_commandBuffer[m_commandBufferIndex] = 0;

                // Execute the command
                String response;
                bool success = executeCommand(String(m_commandBuffer), response);

                // Print response
                if (response.length() > 0) {
                    Serial.println(response);
                }

                // Reset command buffer
                m_commandBufferIndex = 0;
                memset(m_commandBuffer, 0, CONFIG_COMMAND_BUFFER_SIZE);

                // Print prompt
                Serial.print("> ");
            }
        } else if (c == 8 || c == 127) {
            // Backspace/Delete
            if (m_commandBufferIndex > 0) {
                m_commandBufferIndex--;
                m_commandBuffer[m_commandBufferIndex] = 0;
                // Echo backspace
                Serial.print("\b \b");
            }
        } else if (m_commandBufferIndex < CONFIG_COMMAND_BUFFER_SIZE - 1) {
            // Add to buffer if space available
            m_commandBuffer[m_commandBufferIndex++] = c;
            // Echo character
            Serial.print(c);
        }
    }
}

bool SerialCommand::addCommand(const String &name, const String &params, const String &description,
                               CommandHandler handler) {
    // Check if command already exists
    auto it = findCommand(name);
    if (it != m_commands.end()) {
        // Command already exists, replace it
        it->params = params;
        it->description = description;
        it->handler = handler;
    } else {
        // Add new command
        Command cmd;
        cmd.name = name;
        cmd.params = params;
        cmd.description = description;
        cmd.handler = handler;
        m_commands.push_back(cmd);
    }

    return true;
}

void SerialCommand::setSystemManager(SystemManager *systemManager) {
    m_systemManager = systemManager;
}

bool SerialCommand::executeCommand(const String &command, String &response) {
    // Record command execution time
    m_lastCommandTimeMs = millis();

    // Parse command and parameters
    String cmdName, params;
    parseCommand(command, cmdName, params);

    // Convert command to lowercase
    cmdName.toLowerCase();

    // Find command
    auto it = findCommand(cmdName);
    if (it != m_commands.end()) {
        // Execute command handler
        bool success = it->handler(params, response);

        if (!success && response.isEmpty()) {
            response = "Error executing command";
        }

        if (response.length() > 0 && response[0] != '\n') {
            response = "\n" + response;
        }

        return success;
    } else {
        // Command not found
        response = "\nUnknown command '" + cmdName + "'. Type ";
        response += ANSI_COLOR_BLUE;
        response += "'help'";
        response += ANSI_COLOR_RESET;
        response += " for available commands.";
        return false;
    }
}

void SerialCommand::registerBuiltInCommands() {
    // Add all built-in commands
    addCommand(
        "help", "[command]", "Display help information",
        std::bind(&SerialCommand::handleHelp, this, std::placeholders::_1, std::placeholders::_2));

    addCommand("status", "", "Display system status",
               std::bind(&SerialCommand::handleStatus, this, std::placeholders::_1,
                         std::placeholders::_2));

    addCommand(
        "motor", "<index> [enable|disable]", "Get or set motor state",
        std::bind(&SerialCommand::handleMotor, this, std::placeholders::_1, std::placeholders::_2));

    addCommand(
        "move", "<index> <position> [velocity] [accel]", "Move motor to position",
        std::bind(&SerialCommand::handleMove, this, std::placeholders::_1, std::placeholders::_2));

    addCommand(
        "stop", "[index] [emergency]", "Stop one or all motors",
        std::bind(&SerialCommand::handleStop, this, std::placeholders::_1, std::placeholders::_2));

    addCommand(
        "home", "<index> [direction]", "Home a motor",
        std::bind(&SerialCommand::handleHome, this, std::placeholders::_1, std::placeholders::_2));

    addCommand(
        "pid", "<index> [kp] [ki] [kd] [ff]", "Get or set PID parameters",
        std::bind(&SerialCommand::handlePID, this, std::placeholders::_1, std::placeholders::_2));

    addCommand(
        "reset", "[config]", "Reset system or configuration",
        std::bind(&SerialCommand::handleReset, this, std::placeholders::_1, std::placeholders::_2));

    addCommand(
        "save", "", "Save configuration to EEPROM",
        std::bind(&SerialCommand::handleSave, this, std::placeholders::_1, std::placeholders::_2));

    addCommand(
        "load", "", "Load configuration from EEPROM",
        std::bind(&SerialCommand::handleLoad, this, std::placeholders::_1, std::placeholders::_2));

    addCommand(
        "estop", "[reset]", "Trigger or reset emergency stop",
        std::bind(&SerialCommand::handleEStop, this, std::placeholders::_1, std::placeholders::_2));

    addCommand(
        "debug", "<level>", "Set debug log level",
        std::bind(&SerialCommand::handleDebug, this, std::placeholders::_1, std::placeholders::_2));

    addCommand("shutdown", "", "Shutdown system normally and save positions",
               std::bind(&SerialCommand::handleShutdown, this, std::placeholders::_1,
                         std::placeholders::_2));

    addCommand("status_output", "<on|off>", "Enable or disable status JSON output",
               std::bind(&SerialCommand::handleStatusOutput, this, std::placeholders::_1,
                         std::placeholders::_2));

    addCommand("gpio_state", "<pin>", "Show GPIO pin state",
               [](const String &params, String &response) {
                   int pin = params.toInt();
                   if (pin >= 0 && pin <= 39) {
                       pinMode(pin, INPUT);
                       int state = digitalRead(pin);
                       response = "Pin " + String(pin) + " state: " + String(state);
                       return true;
                   }
                   response = "Invalid pin number";
                   return false;
               });
}

void SerialCommand::parseCommand(const String &commandString, String &command, String &params) {
    // Find first space
    int spaceIndex = commandString.indexOf(' ');

    if (spaceIndex < 0) {
        // No space, entire string is command
        command = commandString;
        params = "";
    } else {
        // Split command and parameters
        command = commandString.substring(0, spaceIndex);
        params = commandString.substring(spaceIndex + 1);
        params.trim();  // Remove leading/trailing whitespace
    }
}

std::vector<Command>::iterator SerialCommand::findCommand(const String &name) {
    for (auto it = m_commands.begin(); it != m_commands.end(); ++it) {
        if (it->name.equalsIgnoreCase(name)) {
            return it;
        }
    }

    return m_commands.end();
}

bool SerialCommand::handleHelp(const String &params, String &response) {
    if (params.length() == 0) {
        // Show list of all commands
        response = "Available commands:\n";

        for (const auto &cmd : m_commands) {
            response += "  " + cmd.name;
            if (cmd.params.length() > 0) {
                response += " " + cmd.params;
            }
            response += "\n";
        }

        response += "Type 'help <command>' for more information on a specific command.";
    } else {
        // Show help for specific command
        auto it = findCommand(params);
        if (it != m_commands.end()) {
            response = "Command: " + it->name + "\n";
            response += "Usage: " + it->name;
            if (it->params.length() > 0) {
                response += " " + it->params;
            }
            response += "\n";
            response += "Description: " + it->description;
        } else {
            response = "Unknown command '" + params + "'.";
            return false;
        }
    }

    return true;
}

bool SerialCommand::handleStatus(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    // Get system state
    SystemState state = m_systemManager->getSystemState();
    String stateStr;

    switch (state) {
        case SystemState::INITIALIZING:
            stateStr = "Initializing";
            break;
        case SystemState::READY:
            stateStr = "Ready";
            break;
        case SystemState::RUNNING:
            stateStr = "Running";
            break;
        case SystemState::ERROR:
            stateStr = "Error";
            break;
        case SystemState::EMERGENCY_STOP:
            stateStr = "Emergency Stop";
            break;
        case SystemState::SHUTDOWN:
            stateStr = "Shutdown";
            break;
        default:
            stateStr = "Unknown";
            break;
    }

    // Build status response
    response = "System Status:\n";
    response += "  State: " + stateStr + "\n";
    response += "  Uptime: " + String(m_systemManager->getUptimeMs() / 1000) + " s\n";
    response += "  CPU: " + String(m_systemManager->getCPUUsage(0), 1) + "% / " +
                String(m_systemManager->getCPUUsage(1), 1) + "%\n";
    response += "  Memory: " + String(m_systemManager->getFreeMemory() / 1024) + " KB free\n";

    // Motor status
    MotorManager *motorManager = m_systemManager->getMotorManager();
    if (motorManager != nullptr) {
        uint8_t motorCount = motorManager->getMotorCount();
        response += "  Motors: " + String(motorCount) + "\n";

        for (uint8_t i = 0; i < motorCount; i++) {
            Motor *motor = motorManager->getMotor(i);
            if (motor != nullptr) {
                response += "    Motor " + String(i) + ": ";
                response += "Pos=" + String(motor->getCurrentPosition()) + ", ";
                response += "Vel=" + String(motor->getCurrentVelocity(), 1) + ", ";
                response += "Enabled=" + String(motor->isEnabled() ? "Yes" : "No") + ", ";
                response += "Moving=" + String(motor->isMoving() ? "Yes" : "No") + "\n";
            }
        }
    }

    // Safety status
    SafetyMonitor *safetyMonitor = m_systemManager->getSafetyMonitor();
    if (safetyMonitor != nullptr) {
        SystemSafetyStatus safetyStatus = safetyMonitor->getStatus();
        String safetyStr;

        switch (safetyStatus) {
            case SystemSafetyStatus::NORMAL:
                safetyStr = "Normal";
                break;
            case SystemSafetyStatus::WARNING:
                safetyStr = "Warning";
                break;
            case SystemSafetyStatus::ERROR:
                safetyStr = "Error";
                break;
            case SystemSafetyStatus::EMERGENCY_STOP:
                safetyStr = "E-Stop";
                break;
            default:
                safetyStr = "Unknown";
                break;
        }

        response += "  Safety: " + safetyStr + "\n";

        if (safetyStatus != SystemSafetyStatus::NORMAL) {
            SafetyCode code = safetyMonitor->getLastSafetyCode();
            response += "  Safety Code: " + String(static_cast<int>(code)) + "\n";
        }
    }

    return true;
}

bool SerialCommand::handleMotor(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    MotorManager *motorManager = m_systemManager->getMotorManager();
    if (motorManager == nullptr) {
        response = "Motor manager not available";
        return false;
    }

    // Parse parameters
    int spaceIndex = params.indexOf(' ');
    String indexStr, command;

    if (spaceIndex < 0) {
        // No command, just index
        indexStr = params;
        command = "";
    } else {
        // Split index and command
        indexStr = params.substring(0, spaceIndex);
        command = params.substring(spaceIndex + 1);
        command.trim();
    }

    // Convert index to integer
    if (indexStr.length() == 0) {
        response = "Motor index required";
        return false;
    }

    int motorIndex = indexStr.toInt();

    // Get motor
    Motor *motor = motorManager->getMotor(motorIndex);
    if (motor == nullptr) {
        response = "Motor " + indexStr + " not found";
        return false;
    }

    // Process command
    if (command.length() == 0) {
        // No command, show motor status
        const MotorState &state = motor->getState();

        response = "Motor " + indexStr + " Status:\n";
        response += "  Enabled: " + String(motor->isEnabled() ? "Yes" : "No") + "\n";
        response += "  Position: " + String(state.currentPosition) + "\n";
        response += "  Target: " + String(state.targetPosition) + "\n";
        response += "  Velocity: " + String(state.currentVelocity, 1) + "\n";
        response += "  Mode: " + String(static_cast<int>(motor->getControlMode())) + "\n";
        response += "  Moving: " + String(motor->isMoving() ? "Yes" : "No") + "\n";
        response += "  Homed: " + String(motor->isHomed() ? "Yes" : "No") + "\n";
        response += "  Error: " + String(static_cast<int>(state.error));
    } else if (command.equalsIgnoreCase("enable")) {
        // Enable motor
        motor->enable();
        response = "Motor " + indexStr + " enabled";
    } else if (command.equalsIgnoreCase("disable")) {
        // Disable motor
        motor->disable();
        response = "Motor " + indexStr + " disabled";
    } else {
        // Unknown command
        response = "Unknown motor command: " + command;
        return false;
    }

    return true;
}

bool SerialCommand::handleMove(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    MotorManager *motorManager = m_systemManager->getMotorManager();
    if (motorManager == nullptr) {
        response = "Motor manager not available";
        return false;
    }

    // Parse parameters
    String paramsStr = params;
    paramsStr.trim();

    // Split parameters
    int spaceIndex = paramsStr.indexOf(' ');
    if (spaceIndex < 0) {
        response = "Insufficient parameters. Usage: move <index> <position> [velocity] [accel]";
        return false;
    }

    String indexStr = paramsStr.substring(0, spaceIndex);
    paramsStr = paramsStr.substring(spaceIndex + 1);
    paramsStr.trim();

    spaceIndex = paramsStr.indexOf(' ');
    String positionStr;
    String velocityStr;
    String accelStr;

    if (spaceIndex < 0) {
        // Just position
        positionStr = paramsStr;
        velocityStr = "";
        accelStr = "";
    } else {
        // Position and more
        positionStr = paramsStr.substring(0, spaceIndex);
        paramsStr = paramsStr.substring(spaceIndex + 1);
        paramsStr.trim();

        spaceIndex = paramsStr.indexOf(' ');
        if (spaceIndex < 0) {
            // Position and velocity only
            velocityStr = paramsStr;
            accelStr = "";
        } else {
            // Position, velocity, and accel
            velocityStr = paramsStr.substring(0, spaceIndex);
            accelStr = paramsStr.substring(spaceIndex + 1);
            accelStr.trim();
        }
    }

    // Convert parameters to values
    int motorIndex = indexStr.toInt();
    int32_t position = positionStr.toInt();
    float velocity = velocityStr.length() > 0 ? velocityStr.toFloat() : 0.0f;
    float accel = accelStr.length() > 0 ? accelStr.toFloat() : 0.0f;

    // Get motor
    Motor *motor = motorManager->getMotor(motorIndex);
    if (motor == nullptr) {
        response = "Motor " + indexStr + " not found";
        return false;
    }

    // Enable motor if not enabled
    if (!motor->isEnabled()) {
        motor->enable();
    }

    // Move the motor
    if (velocity > 0.0f && accel > 0.0f) {
        // Use moveToPosition with specified velocity and acceleration
        motor->moveToPosition(position, velocity, accel, accel, 0.0f);
    } else if (velocity > 0.0f) {
        // Use moveToPosition with specified velocity and default acceleration
        motor->moveToPosition(position, velocity, CONFIG_DEFAULT_ACCELERATION,
                              CONFIG_DEFAULT_DECELERATION, 0.0f);
    } else {
        // Use setTargetPosition with default velocity and acceleration
        motor->setTargetPosition(position);
    }

    response = "\nMoving motor " + indexStr + " to position " + positionStr;

    return true;
}

bool SerialCommand::handleStop(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    MotorManager *motorManager = m_systemManager->getMotorManager();
    if (motorManager == nullptr) {
        response = "Motor manager not available";
        return false;
    }

    // Parse parameters
    String paramsStr = params;
    paramsStr.trim();

    if (paramsStr.length() == 0) {
        // Stop all motors
        motorManager->stopAllMotors(false);
        response = "Stopped all motors";
    } else {
        // Check for emergency flag
        bool emergency = paramsStr.endsWith("emergency");
        if (emergency) {
            // Remove "emergency" from params
            int emergencyIndex = paramsStr.lastIndexOf("emergency");
            paramsStr = paramsStr.substring(0, emergencyIndex);
            paramsStr.trim();
        }

        if (paramsStr.length() == 0) {
            // Emergency stop all motors
            motorManager->stopAllMotors(true);
            response = "Emergency stopped all motors";
        } else {
            // Stop specific motor
            int motorIndex = paramsStr.toInt();

            // Get motor
            Motor *motor = motorManager->getMotor(motorIndex);
            if (motor == nullptr) {
                response = "Motor " + paramsStr + " not found";
                return false;
            }

            // Stop the motor
            motor->emergencyStop();  // amir stop(emergency);

            response =
                emergency ? "Emergency stopped motor " + paramsStr : "Stopped motor " + paramsStr;
        }
    }

    return true;
}

bool SerialCommand::handleHome(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    MotorManager *motorManager = m_systemManager->getMotorManager();
    if (motorManager == nullptr) {
        response = "Motor manager not available";
        return false;
    }

    // Parse parameters
    String paramsStr = params;
    paramsStr.trim();

    int spaceIndex = paramsStr.indexOf(' ');
    String indexStr;
    String directionStr;

    if (spaceIndex < 0) {
        // Just index
        indexStr = paramsStr;
        directionStr = "1";  // Default to positive direction
    } else {
        // Index and direction
        indexStr = paramsStr.substring(0, spaceIndex);
        directionStr = paramsStr.substring(spaceIndex + 1);
        directionStr.trim();
    }

    // Convert parameters
    if (indexStr.length() == 0) {
        response = "Motor index required";
        return false;
    }

    int motorIndex = indexStr.toInt();
    int8_t direction = directionStr.length() > 0 ? directionStr.toInt() : 1;

    // Ensure direction is -1 or 1
    direction = direction < 0 ? -1 : 1;

    // Get motor
    Motor *motor = motorManager->getMotor(motorIndex);
    if (motor == nullptr) {
        response = "Motor " + indexStr + " not found";
        return false;
    }

    // Enable motor if not enabled
    if (!motor->isEnabled()) {
        motor->enable();
    }

    // Start homing
    motor->startHoming(direction, CONFIG_DEFAULT_MAX_VELOCITY / 2.0f);

    response = "Homing motor " + indexStr + " in " + (direction > 0 ? "positive" : "negative") +
               " direction";

    return true;
}

bool SerialCommand::handlePID(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    MotorManager *motorManager = m_systemManager->getMotorManager();
    if (motorManager == nullptr) {
        response = "Motor manager not available";
        return false;
    }

    // Parse parameters
    String paramsStr = params;
    paramsStr.trim();

    // Split parameters
    int spaceIndex = paramsStr.indexOf(' ');
    String indexStr;
    String remainingParams;

    if (spaceIndex < 0) {
        // Just index
        indexStr = paramsStr;
        remainingParams = "";
    } else {
        // Index and PID values
        indexStr = paramsStr.substring(0, spaceIndex);
        remainingParams = paramsStr.substring(spaceIndex + 1);
        remainingParams.trim();
    }

    // Convert index
    if (indexStr.length() == 0) {
        response = "Motor index required";
        return false;
    }

    int motorIndex = indexStr.toInt();

    // Get motor
    Motor *motor = motorManager->getMotor(motorIndex);
    if (motor == nullptr) {
        response = "Motor " + indexStr + " not found";
        return false;
    }

    // Get PID controller
    PIDController &pid = motor->getPIDController();

    if (remainingParams.length() == 0) {
        // No parameters, show current PID values
        response = "Motor " + indexStr + " PID Parameters:\n";
        response += "  Kp: " + String(pid.getProportionalTerm()) + "\n";
        response += "  Ki: " + String(pid.getIntegralTerm()) + "\n";
        response += "  Kd: " + String(pid.getDerivativeTerm()) + "\n";
        response += "  FF: " + String(pid.getFeedForwardTerm());
    } else {
        // Parse PID values
        float kp = 0.0f, ki = 0.0f, kd = 0.0f, ff = 0.0f;

        // Extract values from parameters
        // Format: "kp ki kd ff"
        char *str = (char *)remainingParams.c_str();
        char *token = strtok(str, " ");
        int i = 0;

        while (token != NULL && i < 4) {
            switch (i) {
                case 0:
                    kp = atof(token);
                    break;
                case 1:
                    ki = atof(token);
                    break;
                case 2:
                    kd = atof(token);
                    break;
                case 3:
                    ff = atof(token);
                    break;
            }
            token = strtok(NULL, " ");
            i++;
        }

        // Set PID parameters
        pid.setGains(kp, ki, kd, ff);

        response = "Set motor " + indexStr + " PID to Kp=" + String(kp) + ", Ki=" + String(ki) +
                   ", Kd=" + String(kd) + ", FF=" + String(ff);
    }

    return true;
}

bool SerialCommand::handleReset(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    String paramsStr = params;
    paramsStr.trim();

    if (paramsStr.equalsIgnoreCase("config")) {
        // Reset configuration to defaults
        EEPROMManager *eepromManager = m_systemManager->getEEPROMManager();
        if (eepromManager != nullptr) {
            if (eepromManager->resetToDefaults()) {
                response = "Configuration reset to defaults";
                return true;
            } else {
                response = "Failed to reset configuration";
                return false;
            }
        } else {
            response = "EEPROM manager not available";
            return false;
        }
    } else {
        // Reset system
        response = "Resetting system...";

        // Allow time for response to be sent
        delay(100);

        // Reset the system
        m_systemManager->resetSystem();

        return true;
    }
}

bool SerialCommand::handleSave(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    // Save system configuration
    if (m_systemManager->saveSystemConfiguration()) {
        response = "Configuration saved";
        return true;
    } else {
        response = "Failed to save configuration";
        return false;
    }
}

bool SerialCommand::handleLoad(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    // Load system configuration
    if (m_systemManager->loadSystemConfiguration()) {
        response = "Configuration loaded";
        return true;
    } else {
        response = "Failed to load configuration";
        return false;
    }
}

bool SerialCommand::handleEStop(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    String paramsStr = params;
    paramsStr.trim();

    if (paramsStr.equalsIgnoreCase("reset")) {
        // Reset emergency stop
        if (m_systemManager->resetEmergencyStop()) {
            response = "Emergency stop reset";
            return true;
        } else {
            response = "Failed to reset emergency stop";
            return false;
        }
    } else {
        // Trigger emergency stop
        m_systemManager->triggerEmergencyStop(SafetyCode::EMERGENCY_STOP_PRESSED);
        response = "Emergency stop triggered";
        return true;
    }
}

bool SerialCommand::handleDebug(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    Logger *logger = m_systemManager->getLogger();
    if (logger == nullptr) {
        response = "Logger not available";
        return false;
    }

    String paramsStr = params;
    paramsStr.trim();

    if (paramsStr.length() == 0) {
        // Show current log level
        LogLevel level = logger->getLogLevel();
        response = "Current log level: " + Logger::logLevelToString(level);
        return true;
    } else {
        // Set log level
        int level = paramsStr.toInt();
        if (level >= 0 && level <= 5) {
            logger->setLogLevel(static_cast<LogLevel>(level));
            response = "Log level set to " + Logger::logLevelToString(static_cast<LogLevel>(level));
            return true;
        } else {
            response =
                "Invalid log level. Valid levels: 0=OFF, 1=ERROR, 2=WARNING, 3=INFO, 4=DEBUG, "
                "5=VERBOSE";
            return false;
        }
    }
}

/**
 * Handle shutdown command
 * Saves motor positions and sets normal shutdown flag
 *
 * @param params Command parameters
 * @param response Command response
 * @return True if successful, false otherwise
 */
bool SerialCommand::handleShutdown(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    // Set normal shutdown flag
    m_systemManager->setNormalShutdown(true);

    // Save current motor positions
    m_systemManager->saveMotorPositions();

    response = "System shutting down normally. Positions saved.";
    return true;
}

/**
 * Handle status output command
 * Enables or disables status reporter serial output
 *
 * @param params Command parameters
 * @param response Command response
 * @return True if successful, false otherwise
 */
bool SerialCommand::handleStatusOutput(const String &params, String &response) {
    if (m_systemManager == nullptr) {
        response = "System manager not available";
        return false;
    }

    StatusReporter *statusReporter = m_systemManager->getStatusReporter();
    if (statusReporter == nullptr) {
        response = "Status reporter not available";
        return false;
    }

    String paramsStr = params;
    paramsStr.trim();
    paramsStr.toLowerCase();

    if (paramsStr == "on" || paramsStr == "true" || paramsStr == "1") {
        statusReporter->enableSerialOutput(true);
        statusReporter->setUpdateFrequency(1);  // Default to 1Hz
        response = "Status output enabled at 1Hz";
        return true;
    } else if (paramsStr == "off" || paramsStr == "false" || paramsStr == "0") {
        statusReporter->enableSerialOutput(false);
        response = "Status output disabled";
        return true;
    } else {
        // Try to parse as a frequency value
        float frequency = paramsStr.toFloat();
        if (frequency > 0) {
            statusReporter->enableSerialOutput(true);
            statusReporter->setUpdateFrequency(frequency);
            response = "Status output enabled at " + String(frequency) + "Hz";
            return true;
        } else {
            response = "Invalid parameter. Usage: status_output <on|off|frequency>";
            return false;
        }
    }
}
// End of Code