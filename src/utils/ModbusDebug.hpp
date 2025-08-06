/**
 * @file ModbusDebug.hpp
 * @brief Modbus debug utilities
 */

#pragma once

#include "core/ModbusCore.h"
#include "core/ModbusFrame.hpp"
#include "utils/ModbusEventBus.hpp" // Moved here from ModbusCore.h to break circular dependency

#ifdef EZMODBUS_DEBUG

#include "utils/ModbusLogSink.hpp"

namespace Modbus {
namespace Debug {

/* @brief Maximum size for a formatted debug message (including null terminator) */
constexpr size_t MAX_DEBUG_MSG_SIZE = (size_t)EZMODBUS_LOG_MAX_MSG_SIZE;

} // namespace Debug
} // namespace Modbus

namespace Modbus {
namespace Debug {

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

/**
 * Set user-provided print function for EZModbus debug output
 * @param fn Print function implementing the PrintFunction contract
 */
void setPrintFunction(PrintFunction fn);

/**
 * RAII helper for early print function registration (can be used as global variable)
 * Usage: `static Modbus::Debug::PrintFunctionSetter func(MyPrintFunction);`
 */
class PrintFunctionSetter {
public:
    // Accept function by reference, convert to pointer automatically
    template<typename Fn>
    explicit PrintFunctionSetter(Fn&& fn) {
        setPrintFunction(fn);
    }
};

/* @brief Copy the prefix into dst and return the number of characters written.
 * @brief Reduces calls to snprintf in logs (heavy overhead on RP2040)
 * @param dst Destination buffer
 * @param dstSize Size of the destination buffer
 * @param ctx Call context (file, function, line)
 * @return Number of characters written
 */
size_t buildPrefix(char* dst, size_t dstSize, const CallCtx& ctx);

/* @brief Log a simple debug message with context information
 * @param message Message to log
 * @param ctx Call context (file, function, line)
 */
void LOG_MSG(const char* message = "", CallCtx ctx = CallCtx());

/* @brief Format and log a debug message with printf-style formatting
 * @param ctx Call context (file, function, line)
 * @param userFmt Printf-style format string
 * @param ... Arguments for the format string
 */
void LOG_MSGF_CTX(CallCtx ctx, const char* userFmt, ...);

/* @brief Macro to automatically capture call context
 * @param format Printf-style format string
 * @param args Arguments for the format string
 */
#define LOG_MSGF(format, ...) LOG_MSGF_CTX(CallCtx(), format, ##__VA_ARGS__)

/* @brief Log a hexdump of a byte buffer with context information
 * @param bytes Byte buffer to log
 * @param ctx Call context (file, function, line)
 */
void LOG_HEXDUMP(const ByteBuffer& bytes, CallCtx ctx = CallCtx());

/* @brief Log a Modbus frame with context information
 * @param frame Modbus frame to log
 * @param desc Description of the frame (optional)
 * @param ctx Call context (file, function, line)
 */
void LOG_FRAME(const Modbus::Frame& frame, const char* desc = nullptr, CallCtx ctx = CallCtx());

} // namespace Debug
} // namespace Modbus


#else // EZMODBUS_DEBUG

namespace Modbus {
namespace Debug {

    /* @brief Maximum size for a formatted debug message (including null terminator) */
    constexpr size_t MAX_DEBUG_MSG_SIZE = 256;

    // If EZMODBUS_DEBUG is not defined, use no-op templates to totally
    // disable calls to LOG_xxx functions (will not evaluate args)

    // No-op type alias for PrintFunction
    using PrintFunction = int(*)(const char*, size_t);

    template<typename... Args>
    inline void setPrintFunction(Args&&...) {}

    // No-op RAII helper when EZMODBUS_DEBUG disabled
    class PrintFunctionSetter {
    public:
        template<typename... Args>
        explicit PrintFunctionSetter(Args&&...) {}
    };

    template<typename... Args>
    inline void LOG_MSG(Args&&...) {}
        
    template<typename... Args>
    inline void LOG_HEXDUMP(Args&&...) {}

    template<typename... Args>
    inline void LOG_FRAME(Args&&...) {}

    template<typename... Args>
    inline void LOG_MSGF(Args&&...) {}

} // namespace Debug
} // namespace Modbus


#endif // EZMODBUS_DEBUG