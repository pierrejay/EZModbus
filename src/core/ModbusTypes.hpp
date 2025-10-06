/**
 * @file ModbusTypes.hpp
 * @brief Header includes & general-purpose types used across EZModbus library
 */

#pragma once

#include <cstring>
#include <string>
#include <cstdint>
#include <cstddef>
#include <array>
#include <vector>
#include <functional>
#include <algorithm>
#include <atomic>
#include <limits>

// Do not include timing macros & FreeRTOS types if NATIVE_TEST is defined
#ifndef NATIVE_TEST

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/portmacro.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"

// ===================================================================================
// TIMING MACROS
// ===================================================================================

inline uint32_t TIME_MS()           { return (xTaskGetTickCount() * portTICK_PERIOD_MS); }
inline uint64_t TIME_US()           { return esp_timer_get_time(); }
inline void WAIT_MS(uint32_t ms)    { vTaskDelay(pdMS_TO_TICKS(ms)); }
inline void WAIT_US(uint32_t us)    { esp_rom_delay_us(us); }

// ===================================================================================
// FREERTOS SYNCHRONIZATION
// ===================================================================================

/* @brief RAII wrapper for a FreeRTOS mutex
* @note Offers a try-lock mechanism
*/
class Mutex {
public:
    Mutex() {
        _sem = xSemaphoreCreateMutexStatic(&_semBuf);
        configASSERT(_sem);
    }
    ~Mutex() {
        vSemaphoreDelete(_sem);
    }
    // Non-copiable
    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;

    /* @brief Try to lock the mutex
    * @return True if the mutex was locked, false otherwise
    */
    bool tryLock() {
        return xSemaphoreTake(_sem, 0) == pdTRUE;
    }

    /* @brief Lock the mutex
    * @param wait The time to wait for the lock (in ticks)
    * @return True if the mutex was locked, false otherwise
    */
    bool lock(TickType_t wait = portMAX_DELAY) {
        return xSemaphoreTake(_sem, wait) == pdTRUE;
    }

    /* @brief Unlock the mutex
    */
    void unlock() {
        BaseType_t ok = xSemaphoreGive(_sem);
        configASSERT(ok == pdTRUE);
    }

private:
    StaticSemaphore_t _semBuf;
    SemaphoreHandle_t _sem; // Handle to the mutex
};


/* @brief RAII wrapper for a FreeRTOS lock (Mutex lock/unlock)
* @param m The Mutex to lock/unlock
* @param wait The time to wait for the lock (in ticks)
 */
class Lock {
public:
    explicit Lock(Mutex& m, TickType_t wait = portMAX_DELAY)
        : _m(m), _locked(_m.lock(wait)) {}

    ~Lock() { if (_locked) _m.unlock(); }

    bool isLocked() const { return _locked; }

private:
    Mutex& _m;
    volatile bool _locked;
};

/* @brief RAII wrapper for a FreeRTOS binary semaphore
*/
class BinarySemaphore {
public:
    BinarySemaphore() { 
        _sem = xSemaphoreCreateBinaryStatic(&_semBuf); 
        configASSERT(_sem);
        // Give the semaphore initially so first take() succeeds
        xSemaphoreGive(_sem);
    }
    ~BinarySemaphore() {} // Static Semaphore : no call to vSemaphoreDelete needed
    BinarySemaphore(const BinarySemaphore&) = delete;
    BinarySemaphore& operator=(const BinarySemaphore&) = delete;
    
    inline bool take(TickType_t wait = portMAX_DELAY) { return xSemaphoreTake(_sem, wait) == pdTRUE; }
    inline bool tryTake() { return xSemaphoreTake(_sem, 0) == pdTRUE; }
    inline void give() const { BaseType_t ok = xSemaphoreGive(_sem); configASSERT(ok == pdTRUE); }
    inline void giveForce() const { (void)xSemaphoreGive(_sem); }
    __attribute__((always_inline)) inline void giveFromISR(BaseType_t* pxHigherPriorityTaskWoken) { 
        xSemaphoreGiveFromISR(_sem, pxHigherPriorityTaskWoken); 
    }

private:
    StaticSemaphore_t _semBuf;
    SemaphoreHandle_t _sem;
};

#endif // NATIVE_TEST


// ===================================================================================
// BYTEBUFFER
// ===================================================================================

/* @brief "Vector-like" minimalistic facade on a raw static buffer
 * @note This class is used to handle binary data in the library.
 * @note It doesn't own the buffer, it only references it (similar to std::span in C++20).
 */
class ByteBuffer {
public:
    ByteBuffer() noexcept
    : _data(nullptr), _size(0), _cap(0) {}

    /* @brief Construct a read/write ByteBuffer.
     * @param ptr The pointer to the buffer.
     * @param capacity The capacity of the buffer.
     */
    ByteBuffer(uint8_t* ptr, size_t capacity) noexcept
        : _data(ptr), _size(0), _cap(capacity) {
        memset(_data, 0, _cap);
    }

    /* @brief Construct a READ-ONLY ByteBuffer.
     * @param ptr The pointer to the buffer.
     * @param size The size of the buffer.
     */
    ByteBuffer(const uint8_t* ptr, size_t size) noexcept
        : _data(const_cast<uint8_t*>(ptr)), _size(size), _cap(size) {
    }

    // No copy allowed
    ByteBuffer(const ByteBuffer&) = delete;
    ByteBuffer& operator=(const ByteBuffer&) = delete;
    // Move allowed
    ByteBuffer(ByteBuffer&&) noexcept = default;
    ByteBuffer& operator=(ByteBuffer&&) noexcept = default;

    // READ-ONLY ACCESSORS

    const uint8_t* data() const { return _data; }
    size_t         size() const { return _size; }
    size_t     capacity() const { return _cap; }
    bool          empty() const { return _size == 0; }
    size_t   free_space() const { return _cap - _size; }
    const uint8_t& operator[](size_t i) const { return _data[i]; }
    bool at(size_t i, uint8_t& out) const {
        if (i >= _size || !_data) return false;
        out = _data[i];
        return true;
    }

    // ITERATORS
    //
    // WARNING: the non-const iterators directly access the underlying buffer
    // but do not manage capacity. USE WITH CAUTION! They are meant to be used
    // with APIs that only take a raw pointer and write directly to it (e.g., UART read).
    // 
    // USAGE PATTERN:
    // 1. Reserve space first: buffer.resize(expected_max_size)
    // 2. Use iterator to get raw pointer: uint8_t* ptr = buffer.begin() [+ offset]
    // 3. Pass ptr to the function that writes to the buffer
    // 4. Adjust size to actual data: buffer.trim(offset + actual_bytes_read)

    uint8_t*       begin()       { return _data; }
    uint8_t*       end()         { return _data + _size; }
    const uint8_t* begin() const { return _data; }
    const uint8_t* end()   const { return _data + _size; }

    // READ-ONLY SLICE

    ByteBuffer slice(size_t offset, size_t length) const {
        if (!_data || offset > _size) return ByteBuffer();
        if (offset + length > _size) length = _size - offset;
        return ByteBuffer(static_cast<const uint8_t*>(_data + offset), length);  // <- read-only ctor!
    }

    // BASIC WRITING

    void clear() { 
        _size = 0; 
        // memset(_data, 0, _cap); // No need to set the bytes to 0 as we will overwrite them anyway
    }
    bool resize(size_t newSize) {
        if (newSize > _cap || !_data) return false;
        if (newSize > _size) memset(_data + _size, 0, newSize - _size); // Set the new bytes to 0 to avoid using garbage data before overwriting them
        _size = newSize;
        return true;
    }
    bool trim(size_t new_size) {
        if (new_size > _cap || !_data) return false;
        if (new_size > _size) return false;
        _size = new_size;
        return true;
    }
    bool push_back(uint8_t b) {
        if (_size >= _cap || !_data) return false;
        _data[_size++] = b;
        return true;
    }
    bool push_back(const uint8_t* buf, size_t len) { // Atomic operation: all bytes in buf will be written or none
        if (_size + len > _cap || !_data) return false;
        memcpy(_data + _size, buf, len);
        _size += len;
        return true;
    }

    // ARBITRARY POSITION WRITING

    bool write_at(size_t pos, uint8_t b) {
        if (pos >= _cap || !_data) return false;
        _data[pos] = b;
        if (pos >= _size) _size = pos + 1;
        return true;
    }

    bool write_at(size_t pos, const uint8_t* buf, size_t len) {
        if (pos + len > _cap || !_data) return false;
        memcpy(_data + pos, buf, len);
        if (pos + len > _size) _size = pos + len;
        return true;
    }

    // CONSUME THE FIRST/LAST N BYTES WHILE KEEPING THE CAPACITY

    bool pop_front(size_t n) {
        if (n > _size) return false;
        if (n) {
            memmove(_data, _data + n, _size - n);
            _size -= n;
        }
        return true;
    }

    bool pop_back(size_t n) {
        if (n > _size) return false;
        _size -= n;
        return true;
    }

private:
    uint8_t* _data;
    size_t   _size;
    size_t   _cap;
};

