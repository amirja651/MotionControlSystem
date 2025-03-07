/*
 * ESP32 High-Precision Motion Control System
 * Serial Command Interface
 *
 * Processes commands received over serial interface, providing
 * a simple command-line interface for system control and configuration.
 */

#ifndef SERIAL_COMMAND_H
#define SERIAL_COMMAND_H

#include <Arduino.h>

#include <functional>
#include <vector>

#include "../Configuration.h"

class SystemManager;

/**
 * Command handler function type
 */
using CommandHandler = std::function<bool(const String &params, String &response)>;

/**
 * Command definition structure
 */
struct Command {
    String name;             // Command name
    String params;           // Parameter format
    String description;      // Command description
    CommandHandler handler;  // Command handler function
};

/**
 * Serial Command class for processing command-line interface
 */
class SerialCommand {
   public:
    /**
     * Constructor
     *
     * @param systemManager Pointer to system manager
     */
    SerialCommand(SystemManager *systemManager = nullptr);

    /**
     * Initialize the command interface
     *
     * @return True if initialization successful, false otherwise
     */
    bool initialize();

    /**
     * Begin command processing
     * Sets up serial interface if needed
     *
     * @param baudRate Serial baud rate
     */
    void begin(uint32_t baudRate = CONFIG_SERIAL_BAUD_RATE);

    /**
     * Process incoming commands
     * Should be called regularly from the main loop
     */
    void processCommands();

    /**
     * Add a custom command
     *
     * @param name Command name
     * @param params Parameter format string
     * @param description Command description
     * @param handler Command handler function
     * @return True if command added successfully, false otherwise
     */
    bool addCommand(const String &name, const String &params, const String &description,
                    CommandHandler handler);

    /**
     * Set system manager
     *
     * @param systemManager Pointer to system manager
     */
    void setSystemManager(SystemManager *systemManager);

    /**
     * Execute a command directly
     *
     * @param command Command string
     * @param response Reference to string for response
     * @return True if command executed successfully, false otherwise
     */
    bool executeCommand(const String &command, String &response);

   private:
    // System manager
    SystemManager *m_systemManager;

    // Command buffer
    char m_commandBuffer[CONFIG_COMMAND_BUFFER_SIZE];
    size_t m_commandBufferIndex;

    // Command list
    std::vector<Command> m_commands;

    // Last command execution time
    uint32_t m_lastCommandTimeMs;

    /**
     * Register built-in commands
     */
    void registerBuiltInCommands();

    /**
     * Parse command string
     *
     * @param commandString Full command string
     * @param command Reference to store command name
     * @param params Reference to store parameters
     */
    void parseCommand(const String &commandString, String &command, String &params);

    /**
     * Find command by name
     *
     * @param name Command name
     * @return Iterator to command or end() if not found
     */
    std::vector<Command>::iterator findCommand(const String &name);

    // Built-in command handlers
    bool handleHelp(const String &params, String &response);
    bool handleStatus(const String &params, String &response);
    bool handleMotor(const String &params, String &response);
    bool handleMove(const String &params, String &response);
    bool handleStop(const String &params, String &response);
    bool handleHome(const String &params, String &response);
    bool handlePID(const String &params, String &response);
    bool handleReset(const String &params, String &response);
    bool handleSave(const String &params, String &response);
    bool handleLoad(const String &params, String &response);
    bool handleEStop(const String &params, String &response);
    bool handleDebug(const String &params, String &response);

    /**
     * Handle shutdown command
     * Saves motor positions and sets normal shutdown flag
     *
     * @param params Command parameters
     * @param response Command response
     * @return True if successful, false otherwise
     */
    bool handleShutdown(const String &params, String &response);
};

#endif  // SERIAL_COMMAND_H