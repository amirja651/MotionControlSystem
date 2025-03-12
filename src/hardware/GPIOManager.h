/*
 * ESP32 High-Precision Motion Control System
 * GPIO Manager
 *
 * Centralizes GPIO pin management, ensuring proper configuration
 * and avoiding pin conflicts.
 */

#ifndef GPIO_MANAGER_H
#define GPIO_MANAGER_H

#include <Arduino.h>

#include <vector>

#include "../Configuration.h"
#include "../utils/Logger.h"

/**
 * GPIO pin modes
 */
enum class PinMode {
    INPUT_PIN,              // Digital input
    INPUT_PULLUP_PIN,       // Digital input with pullup
    INPUT_PULLDOWN_PIN,     // Digital input with pulldown
    OUTPUT_PIN,             // Digital output
    ANALOG_INPUT_PIN,       // Analog input
    ANALOG_OUTPUT_PIN,      // Analog output (DAC)
    INTERRUPT_RISING_PIN,   // Interrupt on rising edge
    INTERRUPT_FALLING_PIN,  // Interrupt on falling edge
    INTERRUPT_CHANGE_PIN    // Interrupt on change
};

/**
 * Pin allocation structure
 */
struct PinAllocation {
    uint8_t pin;   // GPIO pin number
    PinMode mode;  // Pin mode
    String owner;  // Component that owns the pin
    bool inUse;    // Whether pin is currently in use
};

/**
 * GPIO manager singleton class
 */
class GPIOManager {
   public:
    /**
     * Get singleton instance
     *
     * @param logger Optional logger instance
     * @return GPIOManager instance
     */
    static GPIOManager* getInstance(Logger* logger = nullptr);

    /**
     * Allocate a GPIO pin
     *
     * @param pin GPIO pin number
     * @param mode Pin mode
     * @param owner Component requesting the pin
     * @return True if allocation successful, false otherwise
     */
    bool allocatePin(uint8_t pin, PinMode mode, const String& owner);

    /**
     * Release a GPIO pin
     *
     * @param pin GPIO pin number
     * @param owner Component releasing the pin
     * @return True if release successful, false otherwise
     */
    bool releasePin(uint8_t pin, const String& owner);

    /**
     * Check if a pin is available
     *
     * @param pin GPIO pin number
     * @return True if pin is available, false otherwise
     */
    bool isPinAvailable(uint8_t pin) const;

    /**
     * Get current pin mode
     *
     * @param pin GPIO pin number
     * @return Pin mode or INPUT_PIN if pin not allocated
     */
    PinMode getPinMode(uint8_t pin) const;

    /**
     * Get pin owner
     *
     * @param pin GPIO pin number
     * @return Owner name or empty string if pin not allocated
     */
    String getPinOwner(uint8_t pin) const;

    /**
     * Configure a pin for interrupt
     *
     * @param pin GPIO pin number
     * @param mode Interrupt mode
     * @param callback Interrupt service routine
     * @return True if configuration successful, false otherwise
     */
    bool configureInterrupt(uint8_t pin, PinMode mode, void (*callback)());

    /**
     * Disable interrupt on a pin
     *
     * @param pin GPIO pin number
     * @return True if successful, false otherwise
     */
    bool disableInterrupt(uint8_t pin);

    /**
     * Get pin allocation status for all pins
     *
     * @return Vector of pin allocations
     */
    std::vector<PinAllocation> getPinAllocations() const;

    /**
     * Reset all pin allocations
     * This should be used with caution as it resets all pin states
     */
    void resetAllocations();

   private:
    /**
     * Private constructor for singleton
     *
     * @param logger Pointer to logger instance
     */
    GPIOManager(Logger* logger = nullptr);

    // Add logger member
    Logger* m_logger;
    
    /**
     * Destructor
     */
    ~GPIOManager();

    // Singleton instance
    static GPIOManager* s_instance;

    // Pin allocations
    std::vector<PinAllocation> m_pinAllocations;

    /**
     * Dump pin allocations for debugging
     */
    void dumpPinAllocations();

    /**
     * Debug pin state for debugging
     *
     * @param pin GPIO pin number
     */
    void debugPinState(uint8_t pin);

    /**
     * Find pin allocation by pin number
     *
     * @param pin GPIO pin number
     * @return Iterator to pin allocation or end() if not found
     */
    std::vector<PinAllocation>::iterator findPinAllocation(uint8_t pin);

    /**
     * Find pin allocation by pin number (const version)
     *
     * @param pin GPIO pin number
     * @return Const iterator to pin allocation or end() if not found
     */
    std::vector<PinAllocation>::const_iterator findPinAllocation(uint8_t pin) const;

    /**
     * Configure Arduino pin mode
     *
     * @param pin GPIO pin number
     * @param mode Pin mode
     * @return True if configuration successful, false otherwise
     */
    bool configurePinMode(uint8_t pin, PinMode mode);

    /**
     * Convert pin mode to string
     *
     * @param mode Pin mode
     * @return String representation of pin mode
     */
    String pinModeToString(PinMode mode) const;
};

#endif  // GPIO_MANAGER_H