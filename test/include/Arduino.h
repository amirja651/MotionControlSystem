#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <cstdint>
#include <cstring>
#include <string>
#include "WString.h" 

// Add these if not already present
void interrupts();
void noInterrupts();

class SerialClass {
public:
    void begin(long baudRate);
    void print(const std::string& str);
    void println(const std::string& str);
    int available();
    int read();
};

extern SerialClass Serial;

// Pin mode definitions
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);

// Interrupt modes
#define RISING 0x01
#define FALLING 0x02
#define CHANGE 0x03

// ESP32-specific placeholder
namespace esp_random {
    uint32_t get();
}

inline void delay(unsigned long ms) {
    // In a native test environment, this is a no-op
}

inline void delayMicroseconds(unsigned int us) {
    // In a native test environment, this is a no-op
}

#endif // MOCK_ARDUINO_H