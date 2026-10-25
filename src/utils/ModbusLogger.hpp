/**
 * @file ModbusLogger.hpp
 * @brief Thread-safe & non-blocking log sink implementation for EZModbus debug output
 */

#pragma once

#ifndef NATIVE_TEST

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <string.h>
#include <stdarg.h>
#include <cstdio>
#include <cstdarg>
#include <atomic>

// Driver includes for log output
#ifdef ARDUINO
    #include <Arduino.h>
#endif

#ifndef EZMODBUS_LOG_Q_SIZE // Log queue size (# of messages)
    #define EZMODBUS_LOG_Q_SIZE 16
#endif
#ifndef EZMODBUS_LOG_MAX_MSG_SIZE // Maximum length for a formatted debug message (including null terminator)
    #define EZMODBUS_LOG_MAX_MSG_SIZE 256
#endif
#ifndef EZMODBUS_LOG_TASK_PRIORITY // Log task priority
    #define EZMODBUS_LOG_TASK_PRIORITY 1
#endif
#ifndef EZMODBUS_LOG_TASK_STACK_SIZE // Log task stack size (bytes)
    #define EZMODBUS_LOG_TASK_STACK_SIZE 4096
#endif
#ifndef EZMODBUS_LOG_TASK_CORE // Log task core affinity (single-core targets like ESP32-C6 have no core 1)
    #if defined(CONFIG_FREERTOS_UNICORE)
        #define EZMODBUS_LOG_TASK_CORE 0
    #else
        #define EZMODBUS_LOG_TASK_CORE 1
    #endif
#endif


// =============================================================================
// LOG DESTINATION & CHUNK SIZE CONFIGURATION
// =============================================================================

// Default log destination (add user define flag to override)
#ifndef EZMODBUS_LOG_OUTPUT
    #ifdef ARDUINO
        #define EZMODBUS_LOG_OUTPUT Serial
    #endif
#endif

// Default log chunk size
#ifndef EZMODBUS_LOG_CHUNK_SIZE
    #ifdef ARDUINO
        #define EZMODBUS_LOG_CHUNK_SIZE 64
    #endif
#endif

// Default flush function
#ifndef EZMODBUS_LOG_FLUSH
    #ifdef ARDUINO
        #define EZMODBUS_LOG_FLUSH() EZMODBUS_LOG_OUTPUT.flush()
    #elif defined(ESP_IDF_VERSION)
        #define EZMODBUS_LOG_FLUSH() fflush(stdout)
    #else
        #define EZMODBUS_LOG_FLUSH() fflush(stdout)
    #endif
#endif

namespace Modbus {

class Logger {

public:
    static constexpr size_t QUEUE_SIZE = (size_t)EZMODBUS_LOG_Q_SIZE;
    static constexpr size_t MAX_MSG_SIZE = (size_t)EZMODBUS_LOG_MAX_MSG_SIZE;
    static constexpr size_t TASK_PRIORITY = (size_t)EZMODBUS_LOG_TASK_PRIORITY;
    static constexpr uint32_t STACK_SIZE = (uint32_t)EZMODBUS_LOG_TASK_STACK_SIZE;
    static constexpr BaseType_t TASK_CORE = (BaseType_t)EZMODBUS_LOG_TASK_CORE;
    static constexpr uint32_t CHECK_INTERVAL_MS = 100;
    
    // Event bits for queue status
    static constexpr EventBits_t QUEUE_EMPTY_BIT = BIT0;

    struct LogMessage {
        char msg[MAX_MSG_SIZE];
    };

    // Automatic initialization: the first caller does it, concurrent ones wait for it
    static void begin() {
        uint32_t expected = STATE_UNINIT;
        if (!state.compare_exchange_strong(expected, STATE_INITIALIZING)) {
            while (state == STATE_INITIALIZING) vTaskDelay(1);
            return;
        }

        // Create the message queue & the event group for queue status, before the task using them
        logQueue = xQueueCreate(QUEUE_SIZE, sizeof(LogMessage));
        queueEventGroup = xEventGroupCreate();
        if (!logQueue || !queueEventGroup) {
            releaseResources();
            return;
        }
        xEventGroupSetBits(queueEventGroup, QUEUE_EMPTY_BIT); // Initially queue is empty

        // Create the logging task
        BaseType_t taskCreated = xTaskCreatePinnedToCore(
            logTask,
            "LogTask",
            STACK_SIZE,
            NULL,
            TASK_PRIORITY,
            &logTaskHandle,
            TASK_CORE
        );
        if (taskCreated != pdPASS) {
            releaseResources();
            return;
        }
        state = STATE_READY;
    }

    // Simple logging methods
    static void logln(const char* message = "") {
        char buffer[MAX_MSG_SIZE];
        if (*message == '\0') {
            // Empty line → CRLF for it to be visible
            strcpy(buffer, "\r\n");
        } else {
            snprintf(buffer, sizeof(buffer), "%s\r\n", message);
        }
        sendToQueue(buffer);
    }

    template<typename... Args>
    static void logf(const char* format, Args&&... args) {
        char buffer[MAX_MSG_SIZE];
        int len = snprintf(buffer, sizeof(buffer), format, std::forward<Args>(args)...);
        
        // Ensure we don't exceed buffer bounds, add ellipsis if truncated
        if (len >= MAX_MSG_SIZE) {
            len = MAX_MSG_SIZE - 1;
            buffer[len] = '\0';

            // Place ellipsis at the end (before potential newline) → indices -5, -4, -3
            if (MAX_MSG_SIZE >= 5) {
                buffer[MAX_MSG_SIZE - 5] = '.';
                buffer[MAX_MSG_SIZE - 4] = '.';
                buffer[MAX_MSG_SIZE - 3] = '.';
            }
        }
        
        // Strip any trailing newlines to ensure consistent formatting
        char* end = buffer + len - 1;
        while (end >= buffer && (*end == '\n' || *end == '\r')) {
            *end = '\0';
            end--;
            len--;
        }
        
        // Ensure exactly one newline at the end even when the message was truncated
        // If there is room, append it; otherwise overwrite the last printable character
        if (len < MAX_MSG_SIZE - 2) {            // Standard case → we still have space
            buffer[len] = '\n';
            buffer[len + 1] = '\0';
        } else {                                 // Buffer was filled → force newline at the end
            buffer[MAX_MSG_SIZE - 2] = '\n';   // Reserve last char for null terminator
            buffer[MAX_MSG_SIZE - 1] = '\0';
        }
        
        sendToQueue(buffer);
    }
    
    // Wait for all messages to be flushed from the queue
    static void waitQueueFlushed() {
        if (state != STATE_READY) return; // Consider empty if not initialized
        
        // Wait for the queue to be empty
        EventBits_t bits = xEventGroupWaitBits(
            queueEventGroup,
            QUEUE_EMPTY_BIT,
            pdFALSE,  // Don't clear the bit
            pdTRUE,   // Wait for all bits (only one here)
            portMAX_DELAY
        );

        EZMODBUS_LOG_FLUSH(); // Flush the UART buffer
        return;
    }

private:
    static constexpr uint32_t STATE_UNINIT = 0, STATE_INITIALIZING = 1, STATE_READY = 2;
    inline static std::atomic<uint32_t> state{STATE_UNINIT};
    inline static QueueHandle_t logQueue = nullptr;
    inline static TaskHandle_t logTaskHandle = nullptr;
    inline static EventGroupHandle_t queueEventGroup = nullptr;

    // Initialization failed: release what was created, a later log retries
    static void releaseResources() {
        if (logQueue) vQueueDelete(logQueue);
        if (queueEventGroup) vEventGroupDelete(queueEventGroup);
        logQueue = nullptr;
        queueEventGroup = nullptr;
        state = STATE_UNINIT;
    }

    // Send a message to the queue
    static void sendToQueue(const char* message) {
        if (state != STATE_READY) begin(); // Initialize if not already initialized
        if (state != STATE_READY) return;  // Abort if initialization failed
        
        LogMessage msg;
        strncpy(msg.msg, message, MAX_MSG_SIZE - 1);
        msg.msg[MAX_MSG_SIZE - 1] = '\0';
        
        // Clear empty bit when adding message
        xEventGroupClearBits(queueEventGroup, QUEUE_EMPTY_BIT);
        xQueueSend(logQueue, &msg, 0); // Send without waiting if queue is full
    }
    
    // Task that reads the queue and writes the logs
    static void logTask(void* parameter) {
        LogMessage msg;
        while (true) {
            if (xQueueReceive(logQueue, &msg, CHECK_INTERVAL_MS) == pdTRUE) {
                writeOutput(msg.msg, strlen(msg.msg));
                
                // Check if queue is now empty and set bit accordingly
                if (uxQueueMessagesWaiting(logQueue) == 0) {
                    xEventGroupSetBits(queueEventGroup, QUEUE_EMPTY_BIT);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

// =============================================================================
// PLATFORM-SPECIFIC IMPLEMENTATIONS OF writeOutput()
// =============================================================================

    #if defined(ARDUINO)
        static void writeOutput(const char* data, size_t len) {
            const char* ptr = data;
            size_t remaining = len;
            
            while (remaining > 0) {
                size_t chunk = (remaining > EZMODBUS_LOG_CHUNK_SIZE) ? EZMODBUS_LOG_CHUNK_SIZE : remaining;
                size_t written = EZMODBUS_LOG_OUTPUT.write((const uint8_t*)ptr, chunk);
                
                if (written == 0) {
                    vTaskDelay(pdMS_TO_TICKS(5)); // Wait for the buffer to be flushed
                    continue;
                }
                
                ptr += written;
                remaining -= written;
                
                if (remaining > 0) {
                    vTaskDelay(pdMS_TO_TICKS(1)); // Yield before next chunk
                }
            }
            
            vTaskDelay(pdMS_TO_TICKS(5)); // Wait for the buffer to be flushed
        }
        
    #else
        // Use printf by default
        static void writeOutput(const char* data, size_t len) {
            printf("%s", data);
        }
        
    #endif
};

} // namespace Modbus

#endif // NATIVE_TEST
