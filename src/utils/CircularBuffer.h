/*
 * ESP32 High-Precision Motion Control System
 * Circular Buffer Implementation
 *
 * A templated circular buffer optimized for ESP32's limited memory,
 * providing efficient storage and retrieval of recent data samples.
 */

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <Arduino.h>

/**
 * Templated circular buffer class
 *
 * @tparam T Data type to store
 * @tparam SIZE Buffer size (must be power of 2 for efficient modulo operations)
 */
template <typename T, size_t SIZE>
class CircularBuffer {
   private:
    // Static assertion to ensure SIZE is a power of 2
    static_assert((SIZE & (SIZE - 1)) == 0, "CircularBuffer SIZE must be a power of 2");

    T m_buffer[SIZE];  // Data storage
    size_t m_head;     // Head index (where to write next)
    size_t m_count;    // Number of items in buffer

    // Fast modulo for powers of 2
    inline size_t modulo(size_t index) const {
        return index & (SIZE - 1);
    }

   public:
    /**
     * Constructor
     */
    CircularBuffer() : m_head(0), m_count(0) {
        // Initialize all elements to zero
        memset(m_buffer, 0, sizeof(m_buffer));
    }

    /**
     * Reset the buffer to empty state
     */
    void clear() {
        m_head = 0;
        m_count = 0;
        // Clear buffer contents to prevent stale data
        memset(m_buffer, 0, sizeof(m_buffer));
    }

    /**
     * Add an item to the buffer
     *
     * @param item Item to add
     */
    void push(const T& item) {
        m_buffer[m_head] = item;

        // Move head to next position
        m_head = modulo(m_head + 1);

        // Increment count up to maximum size
        if (m_count < SIZE) {
            m_count++;
        }
    }

    /**
     * Get the most recently added item
     *
     * @return Most recent item or default T if empty
     */
    T newest() const {
        if (m_count == 0) {
            return T();
        }

        // Head points to next write, so newest is one before
        return m_buffer[modulo(m_head - 1)];
    }

    /**
     * Get the oldest item in the buffer
     *
     * @return Oldest item or default T if empty
     */
    T oldest() const {
        if (m_count == 0) {
            return T();
        }

        // If buffer is full, oldest is at head (will be overwritten next)
        // Otherwise, oldest is at index 0
        size_t oldestIndex = (m_count == SIZE) ? m_head : 0;
        return m_buffer[oldestIndex];
    }

    /**
     * Get an item relative to newest (-1 = previous, -2 = two ago, etc.)
     *
     * @param offset Negative offset from newest item
     * @return Item at offset or default T if invalid offset
     */
    T get(int offset) const {
        if (m_count == 0 || offset >= 0 || -offset > (int)m_count) {
            return T();
        }

        // Calculate the actual index
        size_t index = modulo(m_head + offset);
        return m_buffer[index];
    }

    /**
     * Get item at specific index (0 = oldest available, count-1 = newest)
     *
     * @param index Index from oldest (0) to newest (count-1)
     * @return Item at index or default T if invalid index
     */
    T at(size_t index) const {
        if (index >= m_count) {
            return T();
        }

        // Calculate the actual buffer index
        size_t startIndex = (m_count == SIZE) ? m_head : 0;
        size_t actualIndex = modulo(startIndex + index);
        return m_buffer[actualIndex];
    }

    /**
     * Get current number of items in buffer
     *
     * @return Item count
     */
    size_t count() const {
        return m_count;
    }

    /**
     * Check if buffer is empty
     *
     * @return True if empty, false otherwise
     */
    bool isEmpty() const {
        return m_count == 0;
    }

    /**
     * Check if buffer is full
     *
     * @return True if full, false otherwise
     */
    bool isFull() const {
        return m_count == SIZE;
    }

    /**
     * Get buffer capacity
     *
     * @return Buffer size
     */
    size_t capacity() const {
        return SIZE;
    }

    /**
     * Calculate average of all items in buffer
     *
     * @return Average value or default T if empty
     */
    T average() const {
        if (m_count == 0) {
            return T();
        }

        T sum = T();
        for (size_t i = 0; i < m_count; i++) {
            sum += at(i);
        }

        return sum / static_cast<T>(m_count);
    }

    /**
     * Find min value in buffer
     *
     * @return Minimum value or default T if empty
     */
    T minimum() const {
        if (m_count == 0) {
            return T();
        }

        T min = at(0);
        for (size_t i = 1; i < m_count; i++) {
            T val = at(i);
            if (val < min) {
                min = val;
            }
        }

        return min;
    }

    /**
     * Find max value in buffer
     *
     * @return Maximum value or default T if empty
     */
    T maximum() const {
        if (m_count == 0) {
            return T();
        }

        T max = at(0);
        for (size_t i = 1; i < m_count; i++) {
            T val = at(i);
            if (val > max) {
                max = val;
            }
        }

        return max;
    }
};

#endif  // CIRCULAR_BUFFER_H
        // End of Code