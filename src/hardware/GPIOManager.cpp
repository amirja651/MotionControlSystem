/*
 * ESP32 High-Precision Motion Control System
 * GPIO Manager Implementation
 */

#include "GPIOManager.h"

// Static instance for singleton
GPIOManager* GPIOManager::s_instance = nullptr;

GPIOManager* GPIOManager::getInstance(Logger* logger) {
    if (s_instance == nullptr) {
        s_instance = new GPIOManager(logger);
    }
    return s_instance;
}

GPIOManager::GPIOManager(Logger* logger) : m_logger(logger) {
    // Reserve space for pin allocations to avoid reallocations
    m_pinAllocations.reserve(40);  // ESP32 has up to 40 GPIO pins

    m_logger->logInfo("GPIO Manager initialized", LogModule::SYSTEM);
}

GPIOManager::~GPIOManager() {
    // Release all pins
    resetAllocations();

    m_logger->logInfo("GPIO Manager destroyed", LogModule::SYSTEM);
}

bool GPIOManager::allocatePin(uint8_t pin, PinMode mode, const String& owner) {
    // Check if pin is already allocated
    auto it = findPinAllocation(pin);

    if (it != m_pinAllocations.end()) {
        // Pin is already allocated
        if (it->inUse) {
            // Only allow if the same owner is reconfiguring
            if (it->owner != owner) {
                // Pin is in use by someone else
                m_logger->logError(
                    "Pin " + String(pin) + " allocation failed: already in use by " + it->owner,
                    LogModule::SYSTEM);
                return false;
            }
        }

        // Owner is reconfiguring or pin was previously allocated but not in use
        it->mode  = mode;
        it->owner = owner;
        it->inUse = true;

        m_logger->logInfo("Pin " + String(pin) + " reconfigured to mode " + pinModeToString(mode)
                              + " by " + owner,
                          LogModule::SYSTEM);
    } else {
        // Pin is not allocated, add new allocation
        PinAllocation allocation;
        allocation.pin   = pin;
        allocation.mode  = mode;
        allocation.owner = owner;
        allocation.inUse = true;
        m_pinAllocations.push_back(allocation);

        m_logger->logInfo(
            "Pin " + String(pin) + " allocated in mode " + pinModeToString(mode) + " to " + owner,
            LogModule::SYSTEM);
    }

    // Configure pin mode in hardware
    bool result = configurePinMode(pin, mode);

    if (!result && m_logger) {
        m_logger->logError(
            "Failed to configure pin " + String(pin) + " in mode " + pinModeToString(mode),
            LogModule::SYSTEM);
    }

    return result;
}

bool GPIOManager::releasePin(uint8_t pin, const String& owner) {
    // Find pin allocation
    auto it = findPinAllocation(pin);

    if (it != m_pinAllocations.end() && it->inUse) {
        // Check if the owner matches
        if (it->owner == owner) {
            // Release the pin
            it->inUse = false;

            // Remove interrupt if it was configured
            if (it->mode == PinMode::INTERRUPT_RISING_PIN
                || it->mode == PinMode::INTERRUPT_FALLING_PIN
                || it->mode == PinMode::INTERRUPT_CHANGE_PIN) {
                disableInterrupt(pin);
            }

            m_logger->logInfo("Pin " + String(pin) + " released by " + owner, LogModule::SYSTEM);

            return true;
        } else {
            m_logger->logWarning("Pin " + String(pin) + " release failed: owned by " + it->owner
                                     + ", release attempted by " + owner,
                                 LogModule::SYSTEM);
        }
    } else {
        m_logger->logWarning("Pin " + String(pin) + " release failed: not allocated or not in use",
                             LogModule::SYSTEM);
    }

    return false;
}

bool GPIOManager::isPinAvailable(uint8_t pin) const {
    // Find pin allocation
    auto it = findPinAllocation(pin);

    // Pin is available if not allocated or not in use
    return (it == m_pinAllocations.end() || !it->inUse);
}

PinMode GPIOManager::getPinMode(uint8_t pin) const {
    // Find pin allocation
    auto it = findPinAllocation(pin);

    if (it != m_pinAllocations.end() && it->inUse) {
        return it->mode;
    }

    // Default mode if pin not allocated
    return PinMode::INPUT_PIN;
}

String GPIOManager::getPinOwner(uint8_t pin) const {
    // Find pin allocation
    auto it = findPinAllocation(pin);

    if (it != m_pinAllocations.end() && it->inUse) {
        return it->owner;
    }

    return "";
}

bool GPIOManager::configureInterrupt(uint8_t pin, PinMode mode, void (*callback)()) {
    // Validate interrupt mode
    if (mode != PinMode::INTERRUPT_RISING_PIN && mode != PinMode::INTERRUPT_FALLING_PIN
        && mode != PinMode::INTERRUPT_CHANGE_PIN) {
        m_logger->logError(
            "Interrupt configuration failed for pin " + String(pin) + ": invalid mode",
            LogModule::SYSTEM);
        return false;
    }

    // Configure pin mode first
    if (!configurePinMode(pin, mode)) {
        m_logger->logError("Interrupt configuration failed for pin " + String(pin)
                               + ": pin mode configuration failed",
                           LogModule::SYSTEM);
        return false;
    }

    // Determine interrupt mode
    int interruptMode;
    switch (mode) {
        case PinMode::INTERRUPT_RISING_PIN:
            interruptMode = RISING;
            break;
        case PinMode::INTERRUPT_FALLING_PIN:
            interruptMode = FALLING;
            break;
        case PinMode::INTERRUPT_CHANGE_PIN:
            interruptMode = CHANGE;
            break;
        default:
            return false;  // Should never happen due to validation above
    }

    // Attach interrupt
    attachInterrupt(digitalPinToInterrupt(pin), callback, interruptMode);

    String modeStr;
    switch (mode) {
        case PinMode::INTERRUPT_RISING_PIN:
            modeStr = "RISING";
            break;
        case PinMode::INTERRUPT_FALLING_PIN:
            modeStr = "FALLING";
            break;
        case PinMode::INTERRUPT_CHANGE_PIN:
            modeStr = "CHANGE";
            break;
        default:
            modeStr = "UNKNOWN";

            m_logger->logInfo("Interrupt configured for pin " + String(pin) + " in mode " + modeStr,
                              LogModule::SYSTEM);
    }

    return true;
}

bool GPIOManager::disableInterrupt(uint8_t pin) {
    // Detach interrupt
    detachInterrupt(digitalPinToInterrupt(pin));

    m_logger->logInfo("Interrupt disabled for pin " + String(pin), LogModule::SYSTEM);

    return true;
}

std::vector<PinAllocation> GPIOManager::getPinAllocations() const {
    return m_pinAllocations;
}

void GPIOManager::resetAllocations() {
    // Disable all interrupts first
    for (const auto& allocation : m_pinAllocations) {
        if (allocation.inUse
            && (allocation.mode == PinMode::INTERRUPT_RISING_PIN
                || allocation.mode == PinMode::INTERRUPT_FALLING_PIN
                || allocation.mode == PinMode::INTERRUPT_CHANGE_PIN)) {
            disableInterrupt(allocation.pin);
        }
    }

    m_logger->logInfo(
        "Resetting all pin allocations (" + String(m_pinAllocations.size()) + " pins)",
        LogModule::SYSTEM);

    // Clear all allocations
    m_pinAllocations.clear();
}

std::vector<PinAllocation>::iterator GPIOManager::findPinAllocation(uint8_t pin) {
    for (auto it = m_pinAllocations.begin(); it != m_pinAllocations.end(); ++it) {
        if (it->pin == pin) {
            return it;
        }
    }

    return m_pinAllocations.end();
}

std::vector<PinAllocation>::const_iterator GPIOManager::findPinAllocation(uint8_t pin) const {
    for (auto it = m_pinAllocations.begin(); it != m_pinAllocations.end(); ++it) {
        if (it->pin == pin) {
            return it;
        }
    }

    return m_pinAllocations.end();
}

bool GPIOManager::configurePinMode(uint8_t pin, PinMode mode) {
    switch (mode) {
        case PinMode::INPUT_PIN:
            pinMode(pin, INPUT);
            break;

        case PinMode::INPUT_PULLUP_PIN:
            pinMode(pin, INPUT_PULLUP);
            break;

        case PinMode::INPUT_PULLDOWN_PIN:
            pinMode(pin, INPUT_PULLDOWN);
            break;

        case PinMode::OUTPUT_PIN:
            pinMode(pin, OUTPUT);
            break;

        case PinMode::ANALOG_INPUT_PIN:
            // ESP32 doesn't need pinMode for analog input
            break;

        case PinMode::ANALOG_OUTPUT_PIN:
            // Check if pin is DAC capable (ESP32 DAC: GPIO25, GPIO26)
            if (pin != 25 && pin != 26) {
                m_logger->logError("Pin " + String(pin) + " does not support analog output",
                                   LogModule::SYSTEM);
                return false;
            }
            break;

        case PinMode::INTERRUPT_RISING_PIN:
        case PinMode::INTERRUPT_FALLING_PIN:
        case PinMode::INTERRUPT_CHANGE_PIN:
            // Check if pin supports interrupts (ESP32 supports interrupts on all pins)
            pinMode(pin, INPUT);
            break;

        default:
            m_logger->logError("Invalid pin mode for pin " + String(pin), LogModule::SYSTEM);
            return false;
    }

    return true;
}

// Add helper method to convert PinMode to string for easier logging
String GPIOManager::pinModeToString(PinMode mode) const {
    switch (mode) {
        case PinMode::INPUT_PIN:
            return "INPUT";
        case PinMode::INPUT_PULLUP_PIN:
            return "INPUT_PULLUP";
        case PinMode::INPUT_PULLDOWN_PIN:
            return "INPUT_PULLDOWN";
        case PinMode::OUTPUT_PIN:
            return "OUTPUT";
        case PinMode::ANALOG_INPUT_PIN:
            return "ANALOG_INPUT";
        case PinMode::ANALOG_OUTPUT_PIN:
            return "ANALOG_OUTPUT";
        case PinMode::INTERRUPT_RISING_PIN:
            return "INTERRUPT_RISING";
        case PinMode::INTERRUPT_FALLING_PIN:
            return "INTERRUPT_FALLING";
        case PinMode::INTERRUPT_CHANGE_PIN:
            return "INTERRUPT_CHANGE";
        default:
            return "UNKNOWN";
    }
}

// Add utility method to dump pin allocations for debugging
void GPIOManager::dumpPinAllocations() {
    if (!m_logger) {
        return;
    }

    m_logger->logInfo("=== GPIO Pin Allocations ===", LogModule::SYSTEM);

    if (m_pinAllocations.empty()) {
        m_logger->logInfo("No pins allocated", LogModule::SYSTEM);
        return;
    }

    for (const auto& allocation : m_pinAllocations) {
        String status = allocation.inUse ? "IN_USE" : "FREE";
        m_logger->logInfo("Pin " + String(allocation.pin) + ": " + status + ", Mode: "
                              + pinModeToString(allocation.mode) + ", Owner: " + allocation.owner,
                          LogModule::SYSTEM);
    }

    m_logger->logInfo("===========================", LogModule::SYSTEM);
}

void GPIOManager::debugPinState(uint8_t pin) {
    int state = digitalRead(pin);
    m_logger->logDebug("Pin " + String(pin) + " state: " + String(state), LogModule::SYSTEM);
}
// End of Code