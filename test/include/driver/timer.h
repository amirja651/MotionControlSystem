#ifndef MOCK_TIMER_H
#define MOCK_TIMER_H

#include <cstdint>

// Enum and type definitions to match ESP-IDF timer headers
typedef enum {
    TIMER_GROUP_0 = 0,
    TIMER_GROUP_1 = 1
} timer_group_t;

typedef enum {
    TIMER_0 = 0,
    TIMER_1 = 1
} timer_idx_t;

typedef enum {
    TIMER_ALARM_EN = 1,
    TIMER_PAUSE = 0,
    TIMER_INTR_LEVEL = 0,
    TIMER_COUNT_UP = 0,
    TIMER_AUTORELOAD_EN = 1,
    TIMER_AUTORELOAD_DIS = 0
} timer_config_enum_t;

// Simplified timer configuration structure
typedef struct {
    uint32_t alarm_en;
    uint32_t counter_en;
    uint32_t intr_type;
    uint32_t counter_dir;
    uint32_t auto_reload;
    uint32_t divider;
} timer_config_t;

// Mock function declarations to match expected usage in TimerManager
inline void timer_init(timer_group_t group, timer_idx_t timer, timer_config_t* config) {}
inline void timer_set_counter_value(timer_group_t group, timer_idx_t timer, uint64_t value) {}
inline void timer_set_alarm_value(timer_group_t group, timer_idx_t timer, uint64_t value) {}
inline void timer_enable_intr(timer_group_t group, timer_idx_t timer) {}
inline void timer_group_enable_alarm_in_isr(timer_group_t group, timer_idx_t timer) {}
inline void timer_group_clr_intr_status_in_isr(timer_group_t group, timer_idx_t timer) {}
inline void timer_start(timer_group_t group, timer_idx_t timer) {}
inline void timer_pause(timer_group_t group, timer_idx_t timer) {}
inline void timer_isr_register(timer_group_t group, timer_idx_t timer, void* handler, void* arg, int flags, void** handle) {}

#endif // MOCK_TIMER_H