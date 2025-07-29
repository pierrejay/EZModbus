/**
 * @file ModbusDebug.hpp
 * @brief Modbus debug utilities
 */

#pragma once

#include "core/ModbusCore.h"
#include "core/ModbusFrame.hpp"
#include "utils/ModbusEventBus.hpp" // Moved here from ModbusCore.h to break circular dependency

#ifndef EZMODBUS_MAX_DEBUG_MSG_SIZE // Maximum length for a formatted debug message (including \r\n\0 at the end)
    #define EZMODBUS_MAX_DEBUG_MSG_SIZE 256
#endif

namespace Modbus {
namespace Debug {

/* @brief Maximum size for a formatted debug message (including null terminator) */
constexpr size_t MAX_DEBUG_MSG_SIZE = (size_t)EZMODBUS_MAX_DEBUG_MSG_SIZE;

} // namespace Debug
} // namespace Modbus

#ifdef EZMODBUS_DEBUG

#include "utils/ModbusLogger.hpp"

namespace Modbus {
namespace Debug {

/* @brief Copy the prefix into dst and return the number of characters written.
 * @brief Reduces calls to snprintf in logs (heavy overhead on RP2040)
 * @param dst Destination buffer
 * @param dstSize Size of the destination buffer
 * @param ctx Call context (file, function, line)
 * @return Number of characters written
 */
inline size_t buildPrefix(char* dst,
                          size_t dstSize,
                          const CallCtx& ctx)
{
    size_t i = 0;
    auto put = [&](char c) {
        if (i + 1 < dstSize) dst[i++] = c;   // +1 to leave space for '\0'
    };

    put('[');

    // File name
    for (const char* p = ModbusTypeDef::getBasename(ctx.file); *p; ++p) put(*p);

    put(':'); put(':');

    // Function name
    for (const char* p = ctx.function; *p; ++p) put(*p);

    put(':');

    // Line number (minimal itoa)
    char num[10];                       // max 4 294 967 295
    int  len = 0, n = ctx.line;
    do {
        num[len++] = char('0' + (n % 10));
        n /= 10;
    } while (n && len < 10);
    while (len) put(num[--len]);

    put(']'); put(' ');

    dst[i] = '\0';
    return i;
}

/* @brief Log a simple debug message with context information
 * @param message Message to log
 * @param ctx Call context (file, function, line)
 */
inline void LOG_MSG(const char* message = "", CallCtx ctx = CallCtx()) {
    Modbus::Logger::logf("[%s::%s:%d] %s\n", ModbusTypeDef::getBasename(ctx.file), ctx.function, ctx.line, message);
}

/* @brief Format and log a debug message with printf-style formatting
 * @param ctx Call context (file, function, line)
 * @param userFmt Printf-style format string
 * @param args Arguments for the format string
 */
template<typename... Args>
inline void LOG_MSGF_CTX(CallCtx ctx,
                         const char* userFmt,
                         Args&&... args)
{
    // Format the prefix (call context info: file, function, line)
    char fmtBuf[Modbus::Logger::MAX_MSG_SIZE];
    size_t idx = buildPrefix(fmtBuf, sizeof(fmtBuf), ctx);

    // Concatenate userFmt respecting the max size
    size_t copy = strnlen(userFmt, sizeof(fmtBuf) - idx - 1);
    memcpy(fmtBuf + idx, userFmt, copy);
    idx += copy;
    fmtBuf[idx] = '\0';

    // Send to logger with user args
    Modbus::Logger::logf(fmtBuf, std::forward<Args>(args)...);
}

/* @brief Macro to automatically capture call context
 * @param format Printf-style format string
 * @param args Arguments for the format string
 */
#define LOG_MSGF(format, ...) LOG_MSGF_CTX(CallCtx(), format, ##__VA_ARGS__)

/* @brief Log a hexdump of a byte buffer with context information
 * @param bytes Byte buffer to log
 * @param ctx Call context (file, function, line)
 */
inline void LOG_HEXDUMP(const ByteBuffer& bytes, CallCtx ctx = CallCtx()) {
    char buf[MAX_DEBUG_MSG_SIZE];
    size_t idx = buildPrefix(buf, sizeof(buf), ctx);

    /* "Hexdump: " */
    static constexpr char header[] = "Hexdump: ";
    memcpy(buf + idx, header, sizeof(header) - 1);
    idx += sizeof(header) - 1;

    if (bytes.empty()) {
        // Add "<empty>" then log if void buffer
        memcpy(buf + idx, "<empty>", 7);
        idx += 7;
        buf[idx] = '\0';
        Modbus::Logger::logln(buf);
        return;
    }

    static constexpr char LUT[] = "0123456789ABCDEF";

    for (uint8_t b : bytes) {
        // 3 characters per byte ("XX ") + "...\r\n\0" = 6
        if (idx + 6 > sizeof(buf) - 1) {
            buf[idx++] = '.';
            buf[idx++] = '.';
            buf[idx++] = '.';
            break;
        }
        buf[idx++] = LUT[b >> 4];
        buf[idx++] = LUT[b & 0x0F];
        buf[idx++] = ' ';
    }

    buf[idx] = '\0'; // Logger will add "\r\n" automatically
    Modbus::Logger::logln(buf);
}

/* @brief Log a Modbus frame with context information
 * @param frame Modbus frame to log
 * @param desc Description of the frame (optional)
 * @param ctx Call context (file, function, line)
 */
inline void LOG_FRAME(const Modbus::Frame& frame, const char* desc = nullptr, CallCtx ctx = CallCtx()) {
    // Log header with file/function/line information
    Modbus::Logger::logf("[%s::%s:%d] %s:\n", ModbusTypeDef::getBasename(ctx.file), ctx.function, ctx.line, desc);
    
    // Body
    Modbus::Logger::logf("> Type           : %s\n", frame.type == Modbus::REQUEST ? "REQUEST" : "RESPONSE");
    Modbus::Logger::logf("> Function code  : 0x%02X (%s)\n", frame.fc, Modbus::toString(frame.fc));
    Modbus::Logger::logf("> Slave ID       : %d\n", frame.slaveId);
    Modbus::Logger::logf("> Register Addr  : %d\n", frame.regAddress);
    Modbus::Logger::logf("> Register Count : %d\n", frame.regCount);
    
    // Frame data (only if not empty)
    if (frame.regCount > 0) {
        // Pre-format data using manual concatenation (no snprintf in loop)
        constexpr size_t bufSize = MAX_DEBUG_MSG_SIZE;
        char dataStr[bufSize];
        size_t idx = 0;
        
        // Initial prefix
        const char prefix[] = "> Data           : ";
        memcpy(dataStr, prefix, sizeof(prefix) - 1);
        idx = sizeof(prefix) - 1;
        
        // Hex conversion using lookup table
        static constexpr char LUT[] = "0123456789ABCDEF";
        
        for (int i = 0; i < frame.regCount; i++) {
            // Check space for next "0xXXXX " (7 chars) + "..." (3 chars) + future "\r\n" from ModbusLogger (2 chars) + "\0" (1 char)
            if (idx + 13 > bufSize) { // 7 + 3 + 2 + 1 = 13
                // Not enough space for next register → add "..." and stop
                idx = bufSize - 13;
                dataStr[idx++] = '.';
                dataStr[idx++] = '.';
                dataStr[idx++] = '.';
                break;
            }
            
            // Manual hex conversion: much faster than snprintf("0x%04X ", frame.data[i])
            uint16_t reg = frame.data[i];
            
            // "0x" prefix
            dataStr[idx++] = '0';
            dataStr[idx++] = 'x';
            
            // Convert 16-bit value to 4 hex digits (big-endian: MSB first)
            dataStr[idx++] = LUT[(reg >> 12) & 0x0F];  // Bits 15-12
            dataStr[idx++] = LUT[(reg >> 8)  & 0x0F];  // Bits 11-8
            dataStr[idx++] = LUT[(reg >> 4)  & 0x0F];  // Bits 7-4
            dataStr[idx++] = LUT[(reg >> 0)  & 0x0F];  // Bits 3-0
            
            // Space separator
            dataStr[idx++] = ' ';
        }
        
        // Add null terminator
        dataStr[idx] = '\0';
        
        // Single logln call (handles \r\n automatically + truncation)
        Modbus::Logger::logln(dataStr);
    }
    
    // Exception code (if present)
    if (frame.exceptionCode != Modbus::NULL_EXCEPTION) {
        Modbus::Logger::logf("> Exception     : 0x%02X\n", frame.exceptionCode);
    }
    
}

} // namespace Debug
} // namespace Modbus


#else // EZMODBUS_DEBUG


namespace Modbus {
namespace Debug {

    // If EZMODBUS_DEBUG is not defined, use no-op templates to totally
    // disable calls to LOG_xxx functions (will not evaluate args)

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