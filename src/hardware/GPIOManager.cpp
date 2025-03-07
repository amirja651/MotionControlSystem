/*
 * ESP32 High-Precision Motion Control System
 * GPIO Manager Implementation
 */

#include "GPIOManager.h"

// Static instance for singleton
GPIOManager* GPIOManager::s_instance = nullptr;

GPIOManager* GPIOManager::getInstance() {
    if (s_instance == nullptr) {
        s_instance = new GPIOManager();
    }
    return s_instance;
}

GPIOManager::GPIOManager() {
    // Reserve space for pin allocations to avoid reallocations
    m_pinAllocations.reserve(40);  // ESP32 has up to 40 GPIO pins
}

GPIOManager::~GPIOManager() {
    // Release all pins
    resetAllocations();
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
                return false;
            }
        }

        // Owner is reconfiguring or pin was previously allocated but not in use
        it->mode = mode;
        it->owner = owner;
        it->inUse = true;
    } else {
        // Pin is not allocated, add new allocation
        PinAllocation allocation;
        allocation.pin = pin;
        allocation.mode = mode;
        allocation.owner = owner;
        allocation.inUse = true;
        m_pinAllocations.push_back(allocation);
    }

    // Configure pin mode in hardware
    return configurePinMode(pin, mode);
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
            if (it->mode == PinMode::INTERRUPT_RISING_PIN ||
                it->mode == PinMode::INTERRUPT_FALLING_PIN ||
                it->mode == PinMode::INTERRUPT_CHANGE_PIN) {
                disableInterrupt(pin);
            }

            return true;
        }
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
    if (mode != PinMode::INTERRUPT_RISING_PIN && mode != PinMode::INTERRUPT_FALLING_PIN &&
        mode != PinMode::INTERRUPT_CHANGE_PIN) {
        return false;
    }

    // Configure pin mode first
    if (!configurePinMode(pin, mode)) {
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

    return true;
}

bool GPIOManager::disableInterrupt(uint8_t pin) {
    // Detach interrupt
    detachInterrupt(digitalPinToInterrupt(pin));

    return true;
}

std::vector<PinAllocation> GPIOManager::getPinAllocations() const {
    return m_pinAllocations;
}

void GPIOManager::resetAllocations() {
    // Disable all interrupts first
    for (const auto& allocation : m_pinAllocations) {
        if (allocation.inUse && (allocation.mode == PinMode::INTERRUPT_RISING_PIN ||
                                 allocation.mode == PinMode::INTERRUPT_FALLING_PIN ||
                                 allocation.mode == PinMode::INTERRUPT_CHANGE_PIN)) {
            disableInterrupt(allocation.pin);
        }
    }

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
            return false;
    }

    return true;
}