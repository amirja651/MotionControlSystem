#ifndef MOCK_ESP_TIMER_H
#define MOCK_ESP_TIMER_H

#include <cstdint>

// Minimal mock implementation for esp_timer
inline int esp_timer_get_time() {
    return 0; // Return a dummy value
}

#endif // MOCK_ESP_TIMER_H