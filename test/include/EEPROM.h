#ifndef MOCK_EEPROM_H
#define MOCK_EEPROM_H

#include <cstddef>
#include <cstdint>

class EEPROMClass {
public:
    bool begin(size_t size) { return true; }
    uint8_t read(int address) { return 0; }
    void write(int address, uint8_t value) {}
    bool commit() { return true; }
    size_t length() { return 4096; }

    template <typename T>
    void get(int address, T& data) {
        // Minimal implementation
        memset(&data, 0, sizeof(T));
    }

    template <typename T>
    void put(int address, const T& data) {}
};

extern EEPROMClass EEPROM;

#endif // MOCK_EEPROM_H