/**
 * @file ModbusDebug.hpp
 * @brief Modbus debug utilities
 */

#pragma once

#include "core/ModbusCore.h"
#include "core/ModbusFrame.hpp"

#ifndef EZMODBUS_MAX_DEBUG_MSG_SIZE // Maximum length for a formatted debug message (including null terminator)
    #define EZMODBUS_MAX_DEBUG_MSG_SIZE 256
#endif

namespace Modbus {
namespace Debug {

/* @brief Context structure to capture call location information
 */
struct CallCtx {
    const char* file;
    const char* function;
    int line;
    
    CallCtx(const char* f = __builtin_FILE(), 
            const char* func = __builtin_FUNCTION(), 
            int l = __builtin_LINE()) 
        : file(f), function(func), line(l) {}
};

/* @brief Maximum size for a formatted debug message (including null terminator) */
constexpr size_t MAX_DEBUG_MSG_SIZE = (size_t)EZMODBUS_MAX_DEBUG_MSG_SIZE;
static_assert(MAX_DEBUG_MSG_SIZE >= 32, "EZMODBUS_MAX_DEBUG_MSG_SIZE too small");

} // namespace Debug
} // namespace Modbus

#ifdef EZMODBUS_DEBUG

#include "utils/ModbusLogger.hpp"

namespace Modbus {
namespace Debug {

/* @brief Utility function to extract the filename from a full path
 * @param path The full path to extract the filename from
 * @return The filename
 */
static const char* getBasename(const char* path) {
    const char* basename = path;
    
    // Search for the last occurrence of '/' (Unix/Linux/macOS)
    const char* lastSlash = strrchr(path, '/');
    if (lastSlash) basename = lastSlash + 1;
    
    // Search for the last occurrence of '\' (Windows)
    const char* lastBackslash = strrchr(path, '\\');
    if (lastBackslash && lastBackslash > basename) basename = lastBackslash + 1;
    
    return basename;
}

/* @brief Log a simple debug message with context information
 * @param message Message to log
 * @param ctx Call context (file, function, line)
 */
inline void LOG_MSG(const char* message = "", CallCtx ctx = CallCtx()) {
    Modbus::Logger::logf("[%s::%s:%d] %s\n", getBasename(ctx.file), ctx.function, ctx.line, message);
}

/* @brief Format and log a debug message with printf-style formatting
 * @param ctx Call context (file, function, line)
 * @param format Printf-style format string
 * @param args Arguments for the format string
 */
template<typename... Args>
inline void LOG_MSGF_CTX(CallCtx ctx, const char* format, Args&&... args) {
    // Format directly into a fixed-size stack buffer and truncate if necessary
    char buffer[MAX_DEBUG_MSG_SIZE];

    // snprintf guarantees null-termination if buffer size > 0
    int written = snprintf(buffer, sizeof(buffer), format, std::forward<Args>(args)...);
    if (written < 0) return; // snprintf error

    // Determine suffix to indicate truncation if needed
    const char* suffix = (written >= static_cast<int>(sizeof(buffer))) ? " ..." : "";

    // Single logf call with context information
    Modbus::Logger::logf("[%s::%s:%d] %s%s",
                         getBasename(ctx.file), ctx.function, ctx.line,
                         buffer, suffix);
}

/* @brief Macro to automatically capture call context
 * @param format Printf-style format string
 * @param args Arguments for the format string
 */
#define LOG_MSGF(format, ...) LOG_MSGF_CTX(Modbus::Debug::CallCtx(), format, ##__VA_ARGS__)

inline void LOG_HEXDUMP(const ByteBuffer& bytes, CallCtx ctx = CallCtx()) {
    if (bytes.empty()) {
        Modbus::Logger::logf("[%s::%s:%d] Hexdump:<empty>\n", getBasename(ctx.file), ctx.function, ctx.line);
        return;
    }

    char buffer[MAX_DEBUG_MSG_SIZE];
    size_t idx = 0;

    // Prefix
    idx += snprintf(buffer + idx, sizeof(buffer) - idx, "Hexdump: ");

    // Hex bytes
    for (uint8_t b : bytes) {
        if (idx + 4 >= sizeof(buffer)) { // 3 chars for "..." + null terminator reserve
            idx += snprintf(buffer + idx, sizeof(buffer) - idx, "...");
            break;
        }
        idx += snprintf(buffer + idx, sizeof(buffer) - idx, "%02X ", b);
    }

    // Newline termination (ensure room for 1 char + null)
    if (idx + 2 < sizeof(buffer)) {
        buffer[idx++] = '\n';
        buffer[idx] = '\0';
    } else {
        buffer[sizeof(buffer) - 2] = '\n';
        buffer[sizeof(buffer) - 1] = '\0';
    }

    // Single logf call for everything
    Modbus::Logger::logf("[%s::%s:%d] %s", getBasename(ctx.file), ctx.function, ctx.line, buffer);
}

/* @brief Log a Modbus frame with context information
 * @param frame Modbus frame to log
 * @param desc Description of the frame (optional)
 * @param ctx Call context (file, function, line)
 */
inline void LOG_FRAME(const Modbus::Frame& frame, const char* desc = nullptr, CallCtx ctx = CallCtx()) {
    // Log header with file/function/line information
    Modbus::Logger::logf("[%s::%s:%d] %s:\n", getBasename(ctx.file), ctx.function, ctx.line, desc);
    
    // Body
    Modbus::Logger::logf("> Type           : %s\n", frame.type == Modbus::REQUEST ? "REQUEST" : "RESPONSE");
    Modbus::Logger::logf("> Function code  : 0x%02X (%s)\n", frame.fc, Modbus::toString(frame.fc));
    Modbus::Logger::logf("> Slave ID       : %d\n", frame.slaveId);
    Modbus::Logger::logf("> Register Addr  : %d\n", frame.regAddress);
    Modbus::Logger::logf("> Register Count : %d\n", frame.regCount);
    
    // Frame data (only if not empty)
    if (frame.regCount > 0) {
        constexpr size_t bufSize = MAX_DEBUG_MSG_SIZE;
        char dataStr[bufSize];
        strcpy(dataStr, "> Data           : ");
        
        const size_t count = std::min<size_t>(frame.regCount, Modbus::FRAME_DATASIZE); // Coils: regCount > data size
        for (size_t i = 0; i < count; i++) {
            char buffer[10];
            int written = snprintf(buffer, sizeof(buffer), "0x%04X ", frame.data[i]);

            // Bytes still available in dataStr (leave 4 chars: "..." + null)
            size_t remaining = bufSize - strlen(dataStr) - 1;

            if (written >= 0 && static_cast<size_t>(written) < remaining) {
                // Enough room → append normally
                strncat(dataStr, buffer, remaining);
            } else {
                // Not enough room → append truncation suffix and stop
                strncat(dataStr, "...", remaining);
                break;
            }
        }
        Modbus::Logger::logf("%s\n", dataStr);
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