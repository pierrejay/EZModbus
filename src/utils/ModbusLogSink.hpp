/**
 * @file ModbusLogSink.hpp
 * @brief Thread-safe & non-blocking log sink implementation for EZModbus debug output
 */

#pragma once

#ifndef NATIVE_TEST

#include "core/ModbusTypes.hpp"
#include "core/ModbusCore.h"

#ifndef EZMODBUS_LOG_Q_SIZE // Log queue size (# of messages)
    #define EZMODBUS_LOG_Q_SIZE 32
#endif
#ifndef EZMODBUS_LOG_MAX_MSG_SIZE // Maximum length for a formatted debug message (including null terminator)
    #define EZMODBUS_LOG_MAX_MSG_SIZE 256
#endif
#ifndef EZMODBUS_LOG_TASK_PRIORITY // Log task priority
    #define EZMODBUS_LOG_TASK_PRIORITY tskIDLE_PRIORITY + 1
#endif
#ifndef EZMODBUS_LOG_TASK_STACK_SIZE // Log task stack size (bytes)
    #define EZMODBUS_LOG_TASK_STACK_SIZE BYTES_TO_STACK_SIZE(4096)
#endif

namespace Modbus {

class LogSink {

public:
    static constexpr size_t QUEUE_SIZE = (size_t)EZMODBUS_LOG_Q_SIZE;
    static constexpr size_t MAX_MSG_SIZE = (size_t)EZMODBUS_LOG_MAX_MSG_SIZE;
    static constexpr UBaseType_t TASK_PRIORITY = (UBaseType_t)EZMODBUS_LOG_TASK_PRIORITY;
    static constexpr uint32_t STACK_SIZE = (uint32_t)EZMODBUS_LOG_TASK_STACK_SIZE;
    static constexpr uint32_t LOG_PRINT_TIMEOUT_MS = 500;
    
    /**
     * User-provided print function for EZModbus logs
     * @param msg Message to print (null-terminated)
     * @param len Length of message (excluding null terminator)
     * @return int Status code:
     *   -1 : Error occurred, skip this message
     *    0 : Busy/would block, retry later
     *   >0 : Success, number of characters printed
     *        If less than 'len' characters were printed, the logger
     *        will call the function again with the remaining portion
     *        until the entire message is sent.
     */
    using PrintFunction = int(*)(const char* msg, size_t len);

    struct LogMessage {
        char msg[MAX_MSG_SIZE];
    };
    
    // Ensure buffer is large enough for truncation logic
    static_assert(MAX_MSG_SIZE >= 8, "Log buffer size must be at least 8 bytes");

    // Automatic initialization
    static void begin();
    
    // Set user-provided print function (internal API, use Modbus::Debug::setPrintFunction instead)
    static void setPrintFunction(PrintFunction fn);

    // Low-copy logging methods - caller provides buffer, no internal buffering
    // WARNING: These methods MODIFY the provided buffer (normalize line endings, formatting)
    // CRITICAL: buffer MUST be at least MAX_MSG_SIZE bytes to prevent overflow
    static void logln(char* buffer);  // Normalize line endings in caller's buffer, send to queue
    static void logf(char* buffer, const char* format, ...);  // Format in caller's buffer, send to queue
    
    // Wait for all messages to be flushed from the queue
    static void waitQueueFlushed();

private:
    // Static members for logging system
    static bool initialized;
    static QueueHandle_t logQueue;
    static TaskHandle_t logTaskHandle;
    
    // User-provided print function (nullptr = use platform-specific implementation)
    static PrintFunction userPrintFn;

    // Send a message to the queue
    static void sendToQueue(const LogMessage& msg);
    
    // Task that reads the queue and writes the logs
    static void logTask(void* parameter);
};

} // namespace Modbus

#endif // NATIVE_TEST