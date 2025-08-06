/**
 * @file ModbusDebug.cpp
 * @brief Modbus debug utilities implementation
 */

#include "ModbusDebug.hpp"
#include <cstdarg>  // For va_list, va_start, va_end

#ifdef EZMODBUS_DEBUG

namespace Modbus {
namespace Debug {

/* @brief Set user-provided print function for EZModbus debug output
 * @param fn Print function implementing the PrintFunction contract
 */
void setPrintFunction(PrintFunction fn) {
    Modbus::LogSink::setPrintFunction(fn);
}

/* @brief Copy the prefix into dst and return the number of characters written.
 * @brief Reduces calls to snprintf in logs (heavy overhead on RP2040)
 * @param dst Destination buffer
 * @param dstSize Size of the destination buffer
 * @param ctx Call context (file, function, line)
 * @return Number of characters written
 * @note Manual string formatting - no snprintf (low-resource MCU friendly)
 */
size_t buildPrefix(char* dst, size_t dstSize, const CallCtx& ctx) {
    if (dstSize == 0) return 0;  // Prevent underflow in dstSize - 1
    
    size_t i = 0;
    auto put = [&](char c) {
        if (i < dstSize - 1) dst[i++] = c;   // Leave space for '\0'
    };

    put('[');

    // File name
    const char* fileName = ctx.file ? ModbusTypeDef::getBasename(ctx.file) : "unknown";
    for (const char* p = fileName; *p; ++p) put(*p);

    put(':'); put(':');

    // Function name
    const char* funcName = ctx.function ? ctx.function : "unknown";
    for (const char* p = funcName; *p; ++p) put(*p);

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
 * @note Direct memcpy - no snprintf (low-resource MCU friendly)
 */
void LOG_MSG(const char* message, CallCtx ctx) {
    char buf[MAX_DEBUG_MSG_SIZE];
    size_t idx = buildPrefix(buf, sizeof(buf), ctx);
    
    // Add message
    size_t remaining = (idx < sizeof(buf) - 1) ? (sizeof(buf) - idx - 1) : 0;
    size_t msgLen = strnlen(message, remaining);
    memcpy(buf + idx, message, msgLen);
    idx += msgLen;
    buf[idx] = '\0';
    
    Modbus::LogSink::logln(buf);
}

/* @brief Format and log a debug message with printf-style formatting
 * @param ctx Call context (file, function, line)
 * @param userFmt Printf-style format string
 * @param ... Arguments for the format string
 * @note Uses snprintf internally
 */
void LOG_MSGF_CTX(CallCtx ctx, const char* userFmt, ...) {
    // Format the prefix (call context info: file, function, line)
    char fmtBuf[Modbus::LogSink::MAX_MSG_SIZE];
    size_t idx = buildPrefix(fmtBuf, sizeof(fmtBuf), ctx);

    // Concatenate userFmt respecting the max size
    size_t remaining = (idx < sizeof(fmtBuf) - 1) ? (sizeof(fmtBuf) - idx - 1) : 0;
    size_t copy = strnlen(userFmt, remaining);
    memcpy(fmtBuf + idx, userFmt, copy);
    idx += copy;
    fmtBuf[idx] = '\0';

    // Format user args with vsnprintf
    char tempBuf[Modbus::LogSink::MAX_MSG_SIZE];
    va_list args;
    va_start(args, userFmt);
    vsnprintf(tempBuf, sizeof(tempBuf), fmtBuf, args);
    va_end(args);
    
    // Send to logger
    Modbus::LogSink::logln(tempBuf);
}

/* @brief Log a hexdump of a byte buffer with context information
 * @param bytes Byte buffer to log
 * @param ctx Call context (file, function, line)
 * @note Manual string formatting - no snprintf (low-resource MCU friendly)
 */
void LOG_HEXDUMP(const ByteBuffer& bytes, CallCtx ctx) {
    char buf[MAX_DEBUG_MSG_SIZE];
    size_t idx = buildPrefix(buf, sizeof(buf), ctx);

    /* "Hexdump: " */
    static constexpr char header[] = "Hexdump: ";
    memcpy(buf + idx, header, sizeof(header) - 1);
    idx += sizeof(header) - 1;

    if (bytes.empty()) {
        // Add "<empty>" then log if void buffer
        if (idx + 8 <= sizeof(buf)) { // 7 chars + null terminator
            memcpy(buf + idx, "<empty>", 7);
            idx += 7;
        }
        buf[idx] = '\0';
        Modbus::LogSink::logln(buf);
        return;
    }

    static constexpr char LUT[] = "0123456789ABCDEF";

    for (uint8_t b : bytes) {
        // Need space for "XX " (3 chars) + '\0' (1 char) = 4 total
        if (idx + 4 > sizeof(buf)) {
            break;
        }
        // Write hex byte: "XX "
        buf[idx] = LUT[b >> 4];
        buf[idx + 1] = LUT[b & 0x0F];
        buf[idx + 2] = ' ';
        idx += 3;
    }

    // Ensure safe null termination
    if (idx >= sizeof(buf)) {
        idx = sizeof(buf) - 1;
    }
    buf[idx] = '\0'; // LogSink will add "\r\n" automatically
    Modbus::LogSink::logln(buf);
}

/* @brief Log a Modbus frame with context information
 * @param frame Modbus frame to log
 * @param desc Description of the frame (optional)
 * @param ctx Call context (file, function, line)
 * @note Manual string formatting - no snprintf (low-resource MCU friendly)
 */
void LOG_FRAME(const Modbus::Frame& frame, const char* desc, CallCtx ctx) {
    char buf[MAX_DEBUG_MSG_SIZE];
    
    // Log header with file/function/line information
    size_t idx = buildPrefix(buf, sizeof(buf), ctx);
    
    // Add description with colon
    const char* safeDesc = desc ? desc : "Frame";
    size_t remaining = (idx < sizeof(buf) - 2) ? (sizeof(buf) - idx - 2) : 0; // -2 for ":\0"
    size_t descLen = strnlen(safeDesc, remaining);
    memcpy(buf + idx, safeDesc, descLen);
    idx += descLen;
    if (idx < sizeof(buf) - 1) {
        buf[idx] = ':';
        idx += 1;
    }
    buf[idx] = '\0';
    Modbus::LogSink::logln(buf);
    
    // Type
    const char* typeStr = (frame.type == Modbus::REQUEST) ? "REQUEST" : "RESPONSE";
    memcpy(buf, "> Type           : ", 19);
    idx = 19;
    size_t typeLen = strlen(typeStr); // Safe because typeStr is always "REQUEST" or "RESPONSE"
    memcpy(buf + idx, typeStr, typeLen);
    idx += typeLen;
    buf[idx] = '\0';
    Modbus::LogSink::logln(buf);
    
    // Function code - manual hex formatting
    memcpy(buf, "> Function code  : 0x", 21);
    idx = 21;
    // Convert FC to hex (2 digits)
    static constexpr char LUT[] = "0123456789ABCDEF";
    if (idx + 5 < sizeof(buf)) { // Need space for "XX (" + at least 1 char for name + ")\0"
        buf[idx] = LUT[(frame.fc >> 4) & 0x0F];
        buf[idx + 1] = LUT[frame.fc & 0x0F];
        buf[idx + 2] = ' ';
        buf[idx + 3] = '(';
        idx += 4;
        // Add function name
        const char* fcName = Modbus::toString(frame.fc);
        size_t fcNameLen = strlen(fcName); // fcName is always a compile-time constant string literal
        if (idx + fcNameLen < sizeof(buf) - 1) { // Ensure space for name + ')' + '\0'
            memcpy(buf + idx, fcName, fcNameLen);
            idx += fcNameLen;
        }
        if (idx < sizeof(buf) - 1) {
            buf[idx] = ')';
            idx += 1;
        }
    }
    buf[idx] = '\0';
    Modbus::LogSink::logln(buf);
    
    // Slave ID - manual decimal conversion
    memcpy(buf, "> Slave ID       : ", 19);
    idx = 19;
    // Simple decimal conversion with bounds checking
    if (idx + 4 < sizeof(buf)) { // Max 3 digits + null terminator
        if (frame.slaveId >= 100) {
            buf[idx] = '0' + (frame.slaveId / 100);
            buf[idx + 1] = '0' + ((frame.slaveId / 10) % 10);
            buf[idx + 2] = '0' + (frame.slaveId % 10);
            idx += 3;
        } else if (frame.slaveId >= 10) {
            buf[idx] = '0' + (frame.slaveId / 10);
            buf[idx + 1] = '0' + (frame.slaveId % 10);
            idx += 2;
        } else {
            buf[idx] = '0' + frame.slaveId;
            idx += 1;
        }
    }
    buf[idx] = '\0';
    Modbus::LogSink::logln(buf);
    
    // Register Address
    memcpy(buf, "> Register Addr  : ", 19);
    idx = 19;
    // Manual decimal conversion for 16-bit values
    uint16_t addr = frame.regAddress;
    char numBuf[6]; // max 65535 + \0
    int numLen = 0;
    do {
        numBuf[numLen++] = '0' + (addr % 10);
        addr /= 10;
    } while (addr > 0);
    // Reverse digits with bounds checking - NO idx++ notation!
    while (numLen > 0 && idx < sizeof(buf) - 1) {
        numLen -= 1;
        buf[idx] = numBuf[numLen];
        idx += 1;
    }
    buf[idx] = '\0';
    Modbus::LogSink::logln(buf);
    
    // Register Count
    memcpy(buf, "> Register Count : ", 19);
    idx = 19;
    uint16_t count = frame.regCount;
    numLen = 0;
    do {
        numBuf[numLen++] = '0' + (count % 10);
        count /= 10;
    } while (count > 0);
    // Reverse digits with bounds checking - NO idx++ notation!
    while (numLen > 0 && idx < sizeof(buf) - 1) {
        numLen -= 1;
        buf[idx] = numBuf[numLen];
        idx += 1;
    }
    buf[idx] = '\0';
    Modbus::LogSink::logln(buf);
    
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
                if (bufSize >= 16) { // Ensure we have at least room for prefix + "..."
                    idx = bufSize - 13;
                    dataStr[idx++] = '.';
                    dataStr[idx++] = '.';
                    dataStr[idx++] = '.';
                }
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
        Modbus::LogSink::logln(dataStr);
    }
    
    // Exception code (if present)
    if (frame.exceptionCode != Modbus::NULL_EXCEPTION) {
        memcpy(buf, "> Exception     : 0x", 20);
        idx = 20;
        // Convert exception code to hex (2 digits) with bounds checking
        if (idx + 3 < sizeof(buf)) { // 2 hex chars + null terminator
            buf[idx] = LUT[(frame.exceptionCode >> 4) & 0x0F];
            buf[idx + 1] = LUT[frame.exceptionCode & 0x0F];
            idx += 2;
        }
        buf[idx] = '\0';
        Modbus::LogSink::logln(buf);
    }
}

} // namespace Debug
} // namespace Modbus

#endif // EZMODBUS_DEBUG