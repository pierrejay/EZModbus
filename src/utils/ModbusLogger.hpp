/**
 * @file ModbusLogger.hpp
 * @brief Thread-safe & non-blocking log sink implementation for EZModbus debug output
 */

#pragma once

#ifndef NATIVE_TEST

#include "core/ModbusTypes.hpp"
#include "core/ModbusCore.h"

// Forward declarations for STM32 UART handle
#if defined(STM32_HAL)
    #include "main.h"
    extern UART_HandleTypeDef huart1;
    extern UART_HandleTypeDef huart2;
    extern UART_HandleTypeDef huart3;
    extern UART_HandleTypeDef huart4;
    extern UART_HandleTypeDef huart5;
    extern UART_HandleTypeDef huart6;
#endif

#ifndef EZMODBUS_LOG_Q_SIZE // Log queue size (# of messages)
    #define EZMODBUS_LOG_Q_SIZE 32
#endif
#ifndef EZMODBUS_LOG_MAX_MSG_SIZE // Maximum length for a formatted debug message (including null terminator)
    #define EZMODBUS_LOG_MAX_MSG_SIZE 256
#endif
#ifndef EZMODBUS_LOG_TASK_PRIORITY // Log task priority
    #define EZMODBUS_LOG_TASK_PRIORITY 1
#endif
#ifndef EZMODBUS_LOG_TASK_STACK_SIZE // Log task stack size (bytes)
    #define EZMODBUS_LOG_TASK_STACK_SIZE BYTES_TO_STACK_SIZE(4096)
#endif

// =============================================================================
// LOG DESTINATION & CHUNK SIZE CONFIGURATION
// =============================================================================

// RP2040 USB stdio constant
#if defined(PICO_SDK)
    #define USB_STDIO ((uart_inst_t*)0xFFFFFFFF)  // Special marker for USB stdio
#endif

// Default log destination (add user define flag to override)
#ifndef EZMODBUS_LOG_OUTPUT
    #ifdef ARDUINO
        #define EZMODBUS_LOG_OUTPUT Serial
    #elif defined(ESP_PLATFORM)
        #define EZMODBUS_LOG_OUTPUT UART_NUM_MAX  // Default: use console (UART or USB-CDC)
    #elif defined(STM32_HAL)
        #define EZMODBUS_LOG_OUTPUT huart2
    #elif defined(PICO_SDK)
        #define EZMODBUS_LOG_OUTPUT USB_STDIO  // Default: USB stdio (printf)
    #endif
#endif

// Default log chunk size
#ifndef EZMODBUS_LOG_CHUNK_SIZE
    #ifdef ARDUINO
        #define EZMODBUS_LOG_CHUNK_SIZE 128
    #elif defined(ESP_PLATFORM)
        #define EZMODBUS_LOG_CHUNK_SIZE 128
    #elif defined(STM32_HAL)
        #define EZMODBUS_LOG_CHUNK_SIZE 64
    #elif defined(PICO_SDK)
        #define EZMODBUS_LOG_CHUNK_SIZE 64
    #endif
#endif

// Default flush function
#ifndef EZMODBUS_LOG_FLUSH
    #ifdef ARDUINO
        #define EZMODBUS_LOG_FLUSH() EZMODBUS_LOG_OUTPUT.flush()
    #elif defined(ESP_PLATFORM)
        #define EZMODBUS_LOG_FLUSH() do {                                   \
            if (EZMODBUS_LOG_OUTPUT == UART_NUM_MAX) {                      \
                fflush(stdout);                                             \
            } else {                                                        \
                uart_wait_tx_done(EZMODBUS_LOG_OUTPUT, pdMS_TO_TICKS(500)); \
            }                                                               \
        } while(0)
    #elif defined(STM32_HAL)
        #define EZMODBUS_LOG_FLUSH() do {} while(0)  // No-op: HAL_UART_Transmit is blocking
    #elif defined(PICO_SDK)
        #define EZMODBUS_LOG_FLUSH() do {} while(0)  // No-op for USB stdio or direct UART
    #endif
#endif

namespace Modbus {

class Logger {

public:
    static constexpr size_t QUEUE_SIZE = (size_t)EZMODBUS_LOG_Q_SIZE;
    static constexpr size_t MAX_MSG_SIZE = (size_t)EZMODBUS_LOG_MAX_MSG_SIZE;
    static constexpr size_t TASK_PRIORITY = (size_t)EZMODBUS_LOG_TASK_PRIORITY;
    static constexpr uint32_t STACK_SIZE = (uint32_t)EZMODBUS_LOG_TASK_STACK_SIZE;
    static constexpr uint32_t CHECK_INTERVAL_MS = 100;
    static constexpr uint32_t EVT_BIT = 0x00000001;
    
    // Event bits for queue status
    static constexpr EventBits_t QUEUE_EMPTY_BIT = EVT_BIT;

    struct LogMessage {
        char msg[MAX_MSG_SIZE];
    };

    // Automatic initialization
    static void begin() {
        if (!initialized) {
            // Create the message queue
            logQueue = xQueueCreate(QUEUE_SIZE, sizeof(LogMessage));
            
            // Create event group for queue status
            queueEventGroup = xEventGroupCreate();
            
            // Create the logging task
            BaseType_t taskCreated = xTaskCreate(
                logTask,
                "LogTask",
                STACK_SIZE,
                NULL,
                TASK_PRIORITY,
                &logTaskHandle
            );
            
            if (logQueue && queueEventGroup && taskCreated == pdPASS) {
                // Initially queue is empty
                xEventGroupSetBits(queueEventGroup, QUEUE_EMPTY_BIT);
                initialized = true;
            }
        }
    }

    // Simple logging methods
    static void logln(const char* message = "") {
        // Work directly in LogMessage structure (single buffer allocation)
        LogMessage msg;
        if (*message == '\0') {
            // Empty line → CRLF for it to be visible
            strcpy(msg.msg, "\r\n");
        } else {
            // Use strcpy/strcat instead of snprintf for better performance
            size_t msg_len = strlen(message);
            if (msg_len < MAX_MSG_SIZE - 3) { // Reserve space for "\r\n\0"
                strcpy(msg.msg, message);
                strcat(msg.msg, "\r\n");
            } else {
                // Message too long → truncate with "..." and add \r\n
                strncpy(msg.msg, message, MAX_MSG_SIZE - 6); // Reserve space for "...\r\n"
                msg.msg[MAX_MSG_SIZE - 6] = '\0';
                strcat(msg.msg, "...\r\n");
            }
        }
        sendToQueue(msg);
    }

    template<typename... Args>
    static void logf(const char* format, Args&&... args) {
        // Work directly in LogMessage structure (single buffer allocation)
        LogMessage msg;
        int len = snprintf(msg.msg, MAX_MSG_SIZE, format, std::forward<Args>(args)...);
        
        // STEP 1: Strip all existing \r\n and \n from the end
        char* end = msg.msg + strlen(msg.msg) - 1;
        while (end >= msg.msg && (*end == '\n' || *end == '\r')) {
            *end = '\0';
            end--;
        }
        
        // STEP 2: Check if we have room for \r\n
        size_t current_len = strlen(msg.msg);
        if (current_len + 2 < MAX_MSG_SIZE) {
            // We have room → add \r\n normally
            msg.msg[current_len] = '\r';
            msg.msg[current_len + 1] = '\n';
            msg.msg[current_len + 2] = '\0';
        } else {
            // Not enough room → force "...\r\n" from end of buffer
            // Truncate message and use strcat like logln()
            msg.msg[MAX_MSG_SIZE - 6] = '\0'; // Truncate to leave space
            strcat(msg.msg, "...\r\n");
        }
        
        sendToQueue(msg);
    }
    
    // Wait for all messages to be flushed from the queue
    static void waitQueueFlushed() {
        if (!initialized) return; // Consider empty if not initialized
        
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
    inline static bool initialized = false;
    inline static QueueHandle_t logQueue = nullptr;
    inline static TaskHandle_t logTaskHandle = nullptr;
    inline static EventGroupHandle_t queueEventGroup = nullptr;

    // Send a message to the queue (overload for LogMessage)
    static void sendToQueue(const LogMessage& msg) {
        #if defined(STM32_HAL)
            // Write output directly if scheduler has not yet started
            if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
                writeOutput(msg.msg, strlen(msg.msg));
                return;
            }
        #endif
        
        if (!initialized) begin(); // Initialize if not already initialized
        if (!initialized) return; // Abort if initialization failed
        
        // Send without waiting if queue is full → discard the message
        if (xQueueSend(logQueue, &msg, 0) != pdPASS) {
            return;
        }
        // Clear empty bit when adding message
        xEventGroupClearBits(queueEventGroup, QUEUE_EMPTY_BIT);
    }

    // Send a message to the queue (overload for const char*)
    static void sendToQueue(const char* message) {
        LogMessage msg;
        strncpy(msg.msg, message, MAX_MSG_SIZE - 1);
        msg.msg[MAX_MSG_SIZE - 1] = '\0';
        sendToQueue(msg);
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
            vTaskDelay(pdMS_TO_TICKS(5)); // Yield before processing next message
        }
    }

// =============================================================================
// UTILITY FUNCTION FOR STM32
// =============================================================================

static void waitIfSchedulerRunning(uint32_t ms) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        WAIT_MS(ms);
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
                    WAIT_MS(5); // Wait for the buffer to be flushed
                    continue;
                }
                
                ptr += written;
                remaining -= written;
                
                if (remaining > 0) {
                    WAIT_MS(1); // Yield before next chunk
                }
            }
        }
        
    #elif defined(ESP_PLATFORM)
        static void writeOutput(const char* data, size_t len) {
            if (EZMODBUS_LOG_OUTPUT == UART_NUM_MAX) {
                // Console output (follows menuconfig: UART or USB-CDC)
                printf("%.*s", (int)len, data);
                WAIT_MS(5); // Wait for the buffer to be flushed
            } else {
                // Direct UART output
                const char* ptr = data;
                size_t remaining = len;
                
                while (remaining > 0) {
                    size_t chunk = (remaining > EZMODBUS_LOG_CHUNK_SIZE) ? EZMODBUS_LOG_CHUNK_SIZE : remaining;
                    int written = uart_write_bytes(EZMODBUS_LOG_OUTPUT, ptr, chunk);
                    
                    if (written <= 0) {
                        WAIT_MS(5); // Wait for the buffer to be flushed
                        continue;
                    }
                    
                    ptr += written;
                    remaining -= written;
                    
                    if (remaining > 0) {
                        WAIT_MS(1); // Yield before next chunk
                    }
                }
            }
        }

    #elif defined(STM32_HAL)
        static void writeOutput(const char* data, size_t len) {
            const char* ptr = data;
            size_t remaining = len;
            
            while (remaining > 0) {
                size_t chunk = (remaining > EZMODBUS_LOG_CHUNK_SIZE) ? EZMODBUS_LOG_CHUNK_SIZE : remaining;
                HAL_StatusTypeDef status = HAL_UART_Transmit(&EZMODBUS_LOG_OUTPUT, (uint8_t*)ptr, chunk, 100);
                
                if (status != HAL_OK) {
                    break; // Exit on error, no more delays
                }
                
                ptr += chunk;
                remaining -= chunk;
                if (remaining > 0 && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
                    waitIfSchedulerRunning(1); // Yield before next chunk
                }
            }
        }

    #elif defined(PICO_SDK)
        static void writeOutput(const char* data, size_t len) {
            // Check if user wants direct UART output or USB stdio
            if (EZMODBUS_LOG_OUTPUT == USB_STDIO) {
                // USB stdio via puts_raw()
                for (size_t i = 0; i < len; i++) {
                    putchar_raw(data[i]);
                }
                WAIT_MS(5); // Wait for the buffer to be flushed
            } else {
                // Direct UART output (uart0 or uart1) via uart_putc_raw()
                uart_inst_t* uart = (uart_inst_t*)EZMODBUS_LOG_OUTPUT;
                const char* ptr = data;
                size_t remaining = len;
                
                while (remaining > 0) {
                    size_t chunk = (remaining > EZMODBUS_LOG_CHUNK_SIZE) ? EZMODBUS_LOG_CHUNK_SIZE : remaining;
                    
                    for (size_t i = 0; i < chunk; i++) {
                        uart_putc_raw(uart, ptr[i]);
                    }
                    
                    ptr += chunk;
                    remaining -= chunk;
                    
                    if (remaining > 0) {
                        WAIT_MS(1); // Yield before next chunk
                    }
                }
            }
        }
        
    #else
        // Fallback for unknown platforms
        static void writeOutput(const char* data, size_t len) {
            printf("%.*s", (int)len, data);
        }
        
    #endif
};

} // namespace Modbus

#endif // NATIVE_TEST