/*
 * ESP32 High-Precision Motion Control System
 * Command Protocol
 *
 * Implements a higher-level command protocol for communication
 * with external systems, supporting binary and text-based formats.
 */

#ifndef COMMAND_PROTOCOL_H
#define COMMAND_PROTOCOL_H

#include <Arduino.h>

#include "../Configuration.h"
#include "../MotorManager.h"
#include "../SystemManager.h"
#include "../core/Motor.h"
#include "../utils/Logger.h"
#include "SerialCommand.h"

// Forward declarations
class SystemManager;
class SerialCommand;

/**
 * Command types
 */
enum class CommandType : uint8_t {
    SYSTEM_COMMAND = 0x01,  // System commands
    MOTION_COMMAND = 0x02,  // Motion control commands
    STATUS_COMMAND = 0x03,  // Status request commands
    CONFIG_COMMAND = 0x04,  // Configuration commands
    DEBUG_COMMAND  = 0x05   // Debug commands
};

/**
 * Command packet structure
 */
struct CommandPacket {
    uint8_t startByte;    // Start byte (0xAA)
    uint8_t commandType;  // Command type
    uint8_t commandId;    // Command ID
    uint8_t length;       // Data length
    uint8_t data[32];     // Command data (variable length, max 32 bytes)
    uint8_t checksum;     // Checksum (XOR of all previous bytes)
};

/**
 * Response packet structure
 */
struct ResponsePacket {
    uint8_t startByte;    // Start byte (0xBB)
    uint8_t commandType;  // Original command type
    uint8_t commandId;    // Original command ID
    uint8_t status;       // Status code (0=success, other=error)
    uint8_t length;       // Data length
    uint8_t data[32];     // Response data (variable length, max 32 bytes)
    uint8_t checksum;     // Checksum (XOR of all previous bytes)
};

/**
 * Command Protocol class
 */
class CommandProtocol {
   public:
    /**
     * Constructor
     *
     * @param systemManager Pointer to system manager
     * @param serialCommand Pointer to serial command processor
     * @param logger Pointer to logger instance
     */
    CommandProtocol(SystemManager* systemManager,
                    SerialCommand* serialCommand,
                    Logger*        logger = nullptr);

    /**
     * Initialize the command protocol
     *
     * @return True if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * Process incoming commands
     * Checks for complete command packets in buffer
     */
    void processCommands();

    /**
     * Process a binary command packet
     *
     * @param packet Command packet
     * @return True if command processed successfully, false otherwise
     */
    bool processCommandPacket(const CommandPacket& packet);

    /**
     * Send a response packet
     *
     * @param packet Response packet to send
     */
    void sendResponse(const ResponsePacket& packet);

    /**
     * Enable/disable binary protocol
     *
     * @param enable True to enable, false to disable
     */
    void enableBinaryProtocol(bool enable);

    /**
     * Check if binary protocol is enabled
     *
     * @return True if enabled, false otherwise
     */
    bool isBinaryProtocolEnabled() const;

   private:
    // System components
    SystemManager* m_systemManager;
    SerialCommand* m_serialCommand;
    Logger*        m_logger;

    // Protocol state
    bool m_binaryProtocolEnabled;

    // Receive buffer
    uint8_t m_receiveBuffer[64];
    uint8_t m_receiveBufferIndex;

    /**
     * Process a system command
     *
     * @param commandId Command ID
     * @param data Command data
     * @param length Data length
     * @param response Response packet
     * @return True if command processed successfully, false otherwise
     */
    bool processSystemCommand(uint8_t         commandId,
                              const uint8_t*  data,
                              uint8_t         length,
                              ResponsePacket& response);

    /**
     * Process a motion command
     *
     * @param commandId Command ID
     * @param data Command data
     * @param length Data length
     * @param response Response packet
     * @return True if command processed successfully, false otherwise
     */
    bool processMotionCommand(uint8_t         commandId,
                              const uint8_t*  data,
                              uint8_t         length,
                              ResponsePacket& response);

    /**
     * Process a status command
     *
     * @param commandId Command ID
     * @param data Command data
     * @param length Data length
     * @param response Response packet
     * @return True if command processed successfully, false otherwise
     */
    bool processStatusCommand(uint8_t         commandId,
                              const uint8_t*  data,
                              uint8_t         length,
                              ResponsePacket& response);

    /**
     * Process a configuration command
     *
     * @param commandId Command ID
     * @param data Command data
     * @param length Data length
     * @param response Response packet
     * @return True if command processed successfully, false otherwise
     */
    bool processConfigCommand(uint8_t         commandId,
                              const uint8_t*  data,
                              uint8_t         length,
                              ResponsePacket& response);

    /**
     * Process a debug command
     *
     * @param commandId Command ID
     * @param data Command data
     * @param length Data length
     * @param response Response packet
     * @return True if command processed successfully, false otherwise
     */
    bool processDebugCommand(uint8_t         commandId,
                             const uint8_t*  data,
                             uint8_t         length,
                             ResponsePacket& response);

    /**
     * Calculate packet checksum
     *
     * @param data Packet data
     * @param length Data length
     * @return Checksum value
     */
    uint8_t calculateChecksum(const uint8_t* data, uint8_t length);

    /**
     * Find command packet in receive buffer
     *
     * @param packet Reference to store command packet
     * @return True if packet found, false otherwise
     */
    bool findCommandPacket(CommandPacket& packet);

    /**
     * Convert command type to string for logging
     *
     * @param type Command type
     * @return String representation of command type
     */
    String commandTypeToString(CommandType type) const;
};

#endif  // COMMAND_PROTOCOL_H