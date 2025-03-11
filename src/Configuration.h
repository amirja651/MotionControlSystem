/*
 * ESP32 High-Precision Motion Control System
 * Configuration Header
 *
 * Central configuration file defining system-wide parameters,
 * pinouts, and operational constants.
 */

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>

// System Identification
#define SYSTEM_VERSION                          "1.0.0"
#define SYSTEM_NAME                             "ESP32 Motion Control System"

// Example folder
#define EXAMPLE_ENABLED false

// Debug and Logging
#define CONFIG_DEBUG_ENABLED                    true    // Master switch for debug mode
#define CONFIG_LOG_LEVEL                        3       // 0=OFF, 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG, 5=VERBOSE
#define CONFIG_SERIAL_BAUD_RATE                 115200
#define CONFIG_MAX_LOG_ENTRIES                  50
#define CONFIG_COLOR_OUTPUT_ENABLED             true    // Enable/disable colored output

// Module-specific logging flags
#define CONFIG_LOG_MOTORMANAGER_ENABLED         true
#define CONFIG_LOG_STEPPERDRIVER_ENABLED        true
#define CONFIG_LOG_PIDCONTROLLER_ENABLED        true
#define CONFIG_LOG_ENCODER_ENABLED              true
#define CONFIG_LOG_SAFETYMONITOR_ENABLED        true
#define CONFIG_LOG_SYSTEM_ENABLED               true
#define CONFIG_LOG_COMMANDHANDLER_ENABLED       true

// Color output for terminal
#define CONFIG_COLOR_OUTPUT_ENABLED             true 

// ANSI Color codes for terminal output - conditional based on CONFIG_COLOR_OUTPUT_ENABLED
#if CONFIG_COLOR_OUTPUT_ENABLED
    #define ANSI_COLOR_RED                      "\x1b[31m"
    #define ANSI_COLOR_GREEN                    "\x1b[32m"
    #define ANSI_COLOR_YELLOW                   "\x1b[33m"
    #define ANSI_COLOR_BLUE                     "\x1b[34m"
    #define ANSI_COLOR_MAGENTA                  "\x1b[35m"
    #define ANSI_COLOR_CYAN                     "\x1b[36m"
    #define ANSI_COLOR_RESET                    "\x1b[0m"
#else
    #define ANSI_COLOR_RED                      ""
    #define ANSI_COLOR_GREEN                    ""
    #define ANSI_COLOR_YELLOW                   ""
    #define ANSI_COLOR_BLUE                     ""
    #define ANSI_COLOR_MAGENTA                  ""
    #define ANSI_COLOR_CYAN                     ""
    #define ANSI_COLOR_RESET                    ""
#endif

// Task Configuration
#define CONFIG_CONTROL_LOOP_FREQUENCY_HZ        1000    // 1kHz control loop
#define CONFIG_TRAJECTORY_UPDATE_FREQUENCY_HZ   250     // 250Hz trajectory updates
#define CONFIG_STATUS_UPDATE_FREQUENCY_HZ       1       // 1Hz status reporting
#define CONFIG_SAFETY_CHECK_INTERVAL_US         500     // 500us safety check interval
#define CONFIG_CONTROL_LOOP_YIELD_US            50      // 50us yield in main loop
#define CONFIG_AUXILIARY_TASK_STACK_SIZE        4096
#define CONFIG_AUXILIARY_TASK_PRIORITY          1
#define CONFIG_CONTROL_TASK_PRIORITY            10

// EEPROM Configuration
#define CONFIG_EEPROM_SIZE                      4096
#define CONFIG_EEPROM_MAGIC_MARKER              0xE5FC  // For validating EEPROM data
#define CONFIG_EEPROM_VERSION                   1

// Motion Control Parameters
#define CONFIG_MAX_MOTORS                       1
#define CONFIG_MAX_ENCODERS                     1
#define CONFIG_DEFAULT_ACCELERATION             1000.0f  // steps/s²
#define CONFIG_DEFAULT_DECELERATION             1000.0f  // steps/s²
#define CONFIG_DEFAULT_MAX_VELOCITY             5000.0f  // steps/s
#define CONFIG_DEFAULT_MAX_JERK                 10000.0f // steps/s³
#define CONFIG_MIN_STEP_INTERVAL_US             10       // Minimum 10us between steps

// PID Control Parameters
#define CONFIG_DEFAULT_PID_KP                   1.0f
#define CONFIG_DEFAULT_PID_KI                   0.1f
#define CONFIG_DEFAULT_PID_KD                   0.05f
#define CONFIG_DEFAULT_PID_FF                   0.0f
#define CONFIG_PID_ANTI_WINDUP_LIMIT            1000.0f
#define CONFIG_PID_OUTPUT_LIMIT                 10000.0f
#define CONFIG_PID_DERIVATIVE_FILTER_ALPHA      0.1f    // Low-pass filter for D term

// Encoder Parameters
#define CONFIG_ENCODER_BUFFER_SIZE              16      // Size of circular buffer for filtering
#define CONFIG_ENCODER_DEFAULT_PPR              2000    // Pulses per revolution
#define CONFIG_ENCODER_INTERPOLATION_FACTOR     4       // 4x interpolation

// Safety Parameters
#define CONFIG_SAFETY_POSITION_TOLERANCE       100      // Max position error in steps
#define CONFIG_SAFETY_VELOCITY_TOLERANCE       1000     // Max velocity error in steps/s
#define CONFIG_SAFETY_MAX_TEMPERATURE_C        80       // Maximum motor temperature
#define CONFIG_EMERGENCY_STOP_DECELERATION     5000.0f  // Emergency stop decel rate

// Timer Configuration
#define CONFIG_TIMER_GROUP                     TIMER_GROUP_0
#define CONFIG_CONTROL_TIMER                   TIMER_0
#define CONFIG_STEP_TIMER                      TIMER_1

// Voltage sensing configuration
#define CONFIG_POWER_MONITORING_ENABLED        false    // Set to false to disable power monitoring
#define CONFIG_VOLTAGE_SENSE_PIN               36       // ADC1_CH0
#define CONFIG_VOLTAGE_MIN                     3.1f     // Minimum safe voltage
#define CONFIG_VOLTAGE_MAX                     3.3f     // Maximum safe voltage
#define CONFIG_VOLTAGE_WARNING_THRESHOLD       0.5f     // 50% deviation tolerance

// Pin Assignments (can be overridden in MotorConfig instances)
// Default pins for Motor 0
#define CONFIG_MOTOR0_STEP_PIN                 32
#define CONFIG_MOTOR0_DIR_PIN                  33
#define CONFIG_MOTOR0_ENABLE_PIN               0xFF     // 0xFF indicates no pin assigned
#define CONFIG_MOTOR0_ENCODER_A_PIN            23
#define CONFIG_MOTOR0_ENCODER_B_PIN            22
#define CONFIG_MOTOR0_LIMIT_MIN_PIN            0xFF     // 0xFF indicates no pin assigned
#define CONFIG_MOTOR0_LIMIT_MAX_PIN            0xFF     // 0xFF indicates no pin assigned

// Command Interface
#define CONFIG_MAX_COMMAND_LENGTH              64
#define CONFIG_COMMAND_BUFFER_SIZE             20

// Structure for motor configuration
struct MotorConfig {
    uint8_t index;
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t enablePin;
    uint8_t encoderAPin;
    uint8_t encoderBPin;
    uint8_t limitMinPin;
    uint8_t limitMaxPin;
    float maxVelocity;
    float maxAcceleration;
    float maxDeceleration;
    float maxJerk;
    float pidKp;
    float pidKi;
    float pidKd;
    float pidFf;
    uint16_t encoderPPR;
    bool invertDirection;
    bool invertEncoder;
    bool useSoftLimits;
    int32_t softLimitMin;
    int32_t softLimitMax;
    bool invertEnable;  // Invert enable pin logic
};

// Default configurations for static allocation
const MotorConfig DEFAULT_MOTOR_CONFIGS[CONFIG_MAX_MOTORS] = {
    {
        // Motor 0
        0,
        CONFIG_MOTOR0_STEP_PIN,
        CONFIG_MOTOR0_DIR_PIN,
        CONFIG_MOTOR0_ENABLE_PIN,
        CONFIG_MOTOR0_ENCODER_A_PIN,
        CONFIG_MOTOR0_ENCODER_B_PIN,
        CONFIG_MOTOR0_LIMIT_MIN_PIN,
        CONFIG_MOTOR0_LIMIT_MAX_PIN,
        CONFIG_DEFAULT_MAX_VELOCITY,
        CONFIG_DEFAULT_ACCELERATION,
        CONFIG_DEFAULT_DECELERATION,
        CONFIG_DEFAULT_MAX_JERK,
        CONFIG_DEFAULT_PID_KP,
        CONFIG_DEFAULT_PID_KI,
        CONFIG_DEFAULT_PID_KD,
        CONFIG_DEFAULT_PID_FF,
        CONFIG_ENCODER_DEFAULT_PPR,
        false,
        false,
        true,
        -1000000,
        1000000,
        false  // invertEnable default: false
    },
    // Additional motor configurations can be added here as needed
};

#endif  // CONFIGURATION_H