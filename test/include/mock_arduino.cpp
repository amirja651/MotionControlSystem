#include "Arduino.h"
#include <chrono>
#include <thread>
#include <random>

static auto start_time = std::chrono::steady_clock::now();

unsigned long millis() {
    auto current_time = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - start_time
    ).count();
}

unsigned long micros() {
    auto current_time = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - start_time
    ).count();
}

void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void delayMicroseconds(unsigned int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

SerialClass Serial;

void SerialClass::begin(long baudRate) {
    // Stub implementation
}

void SerialClass::print(const std::string& str) {
    // Stub implementation
}

void SerialClass::println(const std::string& str) {
    // Stub implementation
}

int SerialClass::available() {
    return 0; // Stub
}

int SerialClass::read() {
    return -1; // Stub
}

void pinMode(uint8_t pin, uint8_t mode) {
    // Stub implementation
}

void digitalWrite(uint8_t pin, uint8_t val) {
    // Stub implementation
}

int digitalRead(uint8_t pin) {
    return 0; // Stub
}

namespace esp_random {
    uint32_t get() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(0, UINT32_MAX);
        return dis(gen);
    }
}