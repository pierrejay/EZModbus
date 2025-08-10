/**
 * @file ModbusLogSink.cpp
 * @brief Thread-safe & non-blocking log sink implementation for EZModbus debug output
 */

#include "ModbusLogSink.hpp"
#include "ModbusDebug.hpp" // For printLog()
#include <cstdarg>  // For va_list, va_start, va_end

#ifndef NATIVE_TEST

namespace Modbus {

// Using declarations to avoid long type names
using Lock = ModbusTypeDef::Lock;
using Mutex = ModbusTypeDef::Mutex;

// Static member definitions
bool LogSink::initialized = false;
QueueHandle_t LogSink::logQueue = nullptr;
TaskHandle_t LogSink::logTaskHandle = nullptr;

/* @brief Initialize the log sink
 * @brief Creates the message queue and the logging task
 * @brief Sets the initialized flag to true
 */
void LogSink::begin() {
    if (!initialized) {
        // Create the message queue
        logQueue = xQueueCreate(QUEUE_SIZE, sizeof(LogMessage));
        
        // Create the logging task
        BaseType_t taskCreated = xTaskCreate(
            logTask,
            "LogTask",
            STACK_SIZE,
            NULL,
            TASK_PRIORITY,
            &logTaskHandle
        );
        
        if (logQueue && taskCreated == pdPASS) {
            initialized = true;
        }
    }
}

/* @brief Log a message with line ending
 * @brief This function is used to log a message with line ending
 * @param buffer The buffer to log
 * @note This method sanitizes (modifies) the caller's buffer. Its size must be at least MAX_MSG_SIZE bytes to prevent overflow.
 */
void LogSink::logln(char* buffer) {
    // Caller provides the buffer, we just normalize line endings
    
    if (buffer == nullptr || *buffer == '\0') {
        // Empty message case - need a local buffer for this special case
        LogMessage localBuffer;
        strcpy(localBuffer.msg, "\r\n");
        sendToQueue(localBuffer);
        return;
    }
    
    // Strip trailing \r and \n characters from caller's buffer
    size_t len = strlen(buffer);
    while (len > 0 && (buffer[len - 1] == '\r' || buffer[len - 1] == '\n')) {
        len--;
    }
    
    // Add exactly one \r\n to caller's buffer (with safety check)
    if (len + 2 < MAX_MSG_SIZE) {
        buffer[len] = '\r';
        buffer[len + 1] = '\n';
        buffer[len + 2] = '\0';
    } else {
        // No space for \r\n, truncate and add it anyway at the end
        buffer[MAX_MSG_SIZE - 3] = '\r';
        buffer[MAX_MSG_SIZE - 2] = '\n';
        buffer[MAX_MSG_SIZE - 1] = '\0';
    }
    
    // Create LogMessage that wraps the caller's buffer
    LogMessage msg;
    size_t actualLen = strlen(buffer); // Get actual length after \r\n addition
    size_t copyLen = (actualLen < MAX_MSG_SIZE - 1) ? actualLen : MAX_MSG_SIZE - 1;
    memcpy(msg.msg, buffer, copyLen);
    msg.msg[copyLen] = '\0';
    
    sendToQueue(msg);
}

/* @brief Log a formatted message
 * @brief This function is used to log a formatted message
 * @param buffer The buffer to log
 * @param format The format string
 * @note This method sanitizes (modifies) the caller's buffer. Its size must be at least MAX_MSG_SIZE bytes to prevent overflow.
 */
void LogSink::logf(char* buffer, const char* format, ...) {
    // Format the message directly in caller's buffer
    va_list args;
    va_start(args, format);
    int written = vsnprintf(buffer, MAX_MSG_SIZE, format, args);
    va_end(args);
    
    // Ensure null termination
    if (written >= (int)MAX_MSG_SIZE) {
        buffer[MAX_MSG_SIZE - 1] = '\0';
        written = MAX_MSG_SIZE - 1;
    } else if (written < 0) {
        // Error in formatting, use empty message
        buffer[0] = '\0';
        written = 0;
    }
    
    // Strip trailing \r and \n characters from caller's buffer
    size_t pos = written;
    while (pos > 0 && (buffer[pos - 1] == '\r' || buffer[pos - 1] == '\n')) {
        pos--;
    }
    
    // Add exactly one \r\n to caller's buffer (with safety check)
    if (pos + 2 < MAX_MSG_SIZE) {
        buffer[pos] = '\r';
        buffer[pos + 1] = '\n';
        buffer[pos + 2] = '\0';
    } else {
        // No space for \r\n, truncate and add it anyway at the end
        buffer[MAX_MSG_SIZE - 3] = '\r';
        buffer[MAX_MSG_SIZE - 2] = '\n';
        buffer[MAX_MSG_SIZE - 1] = '\0';
    }
    
    // Create LogMessage and copy caller's buffer (single copy to queue)
    LogMessage msg;
    size_t actualLen = strlen(buffer); // Get actual length after \r\n addition
    size_t copyLen = (actualLen < MAX_MSG_SIZE - 1) ? actualLen : MAX_MSG_SIZE - 1;
    memcpy(msg.msg, buffer, copyLen);
    msg.msg[copyLen] = '\0';
    
    sendToQueue(msg);
}

/* @brief Wait for all messages to be flushed from the queue
 */
void LogSink::waitQueueFlushed() {
    // Simple delay to allow queue to flush
    // Much simpler than event bits and avoids race conditions
    vTaskDelay(pdMS_TO_TICKS(100));
}

/* @brief Send a message to the queue (overload for LogMessage)
 * @brief This function is used to send a message to the queue
 * @param msg The message to send
 */
void LogSink::sendToQueue(const LogMessage& msg) {
    // If scheduler not started, use direct output
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        // Direct call to user function before scheduler starts
        // Use TIME_US() as TIME_MS() may depend on FreeRTOS
        const char* ptr = msg.msg;
        size_t remaining = strlen(msg.msg);
        uint32_t startTimeUs = TIME_US();
        
        while (remaining > 0) {
            int result = Modbus::Debug::printLog(ptr, remaining);
            
            if (result < 0) {
                // Error, skip this message
                break;
            } else if (result == 0) {
                // Busy, check timeout (do NOT reload)
                uint32_t currentTimeUs = TIME_US();
                uint32_t elapsedMs = (currentTimeUs >= startTimeUs) ? 
                    ((currentTimeUs - startTimeUs) / 1000) : 
                    ((UINT32_MAX - startTimeUs + currentTimeUs + 1) / 1000); // Handle wraparound
                if (elapsedMs > LOG_PRINT_TIMEOUT_MS) {
                    // Timeout exceeded, abandon message
                    break;
                }
                // No delay possible without scheduler, just retry immediately
                continue;
            } else {
                // Success, advance pointer and RELOAD timeout
                // Security check: ensure result is not greater than remaining
                size_t bytes_written = (result > 0) ? static_cast<size_t>(result) : 0;
                if (bytes_written > remaining) {
                    // User function returned invalid value, abort
                    break;
                }
                ptr += bytes_written;
                remaining -= bytes_written;
                startTimeUs = TIME_US(); // Reload timeout on progress
            }
        }
        // Note: writeOutput() is now legacy code, not used anymore
        return;
    }
    
    if (!initialized) {
        begin();
        if (!initialized) return; // Abort if initialization failed
    }
    
    // Send without waiting if queue is full → discard the message
    xQueueSend(logQueue, &msg, 0);
}

/* @brief Task that reads the queue and writes the logs
 * @brief This function is used to read the queue and write the logs
 * @param parameter The parameter to the task (unused)
 */
void LogSink::logTask(void*) {
    LogMessage msg;
    while (true) {
        if (xQueueReceive(logQueue, &msg, portMAX_DELAY) == pdTRUE) {
            const char* ptr = msg.msg;
            size_t remaining = strlen(msg.msg);
            
            // User-provided function with retry logic and timeout protection
            uint32_t startTime = TIME_MS();
            while (remaining > 0) {
                int result = Modbus::Debug::printLog(ptr, remaining);
                
                if (result < 0) {
                    // Error, skip this message
                    break;
                } else if (result == 0) {
                    // Busy, check timeout (do NOT reload)
                    uint32_t currentTime = TIME_MS();
                    uint32_t elapsed = (currentTime >= startTime) ? 
                        (currentTime - startTime) : 
                        (UINT32_MAX - startTime + currentTime + 1); // Handle wraparound
                    if (elapsed > LOG_PRINT_TIMEOUT_MS) {
                        // Timeout exceeded, abandon message
                        break;
                    }
                    // Retry after a short delay
                    vTaskDelay(pdMS_TO_TICKS(10));
                } else {
                    // Success, advance pointer and RELOAD timeout
                    // Security check: ensure result is not greater than remaining
                    size_t bytes_written = (result > 0) ? static_cast<size_t>(result) : 0;
                    if (bytes_written > remaining) {
                        // User function returned invalid value, abort
                        break;
                    }
                    ptr += bytes_written;
                    remaining -= bytes_written;
                    startTime = TIME_MS(); // Reload timeout on progress
                }
            }
        }
        vTaskDelay(5); // Yield before processing next message
    }
}

} // namespace Modbus

#endif // NATIVE_TEST