/**
 * @file ModbusDebug.hpp
 * @brief Modbus debug utilities
 */

#pragma once

#include "core/ModbusCore.h"
#include "core/ModbusFrame.hpp"
#include "utils/ModbusLogSink.hpp"

// Moved here from ModbusCore.h to break circular dependency
// All files that include ModbusDebug.hpp will have access to direct & EventBus logs
#include "utils/ModbusEventBus.hpp"

// ===================================================================================
// USER PRINT FUNCTION
// ===================================================================================

namespace Modbus {
namespace Debug {

    /* @brief Maximum size for a formatted debug message (including null terminator) */
    constexpr size_t MAX_DEBUG_MSG_SIZE = (size_t)Modbus::LogSink::MAX_MSG_SIZE;

    /**
     * @brief User-provided print function for EZModbus logs
     * @param msg Message to print (null-terminated)
     * @param len Length of message (excluding null terminator)
     * @return int Status code:
     *   -1 : Error occurred, skip this message
     *    0 : Busy/would block, retry later
     *   >0 : Success, number of characters printed
     *        If less than 'len' characters were printed, the logger
     *        will call the function again with the remaining portion
     *        until the entire message is sent.
     * @note If EZMODBUS_DEBUG is not defined, the function will never
     *       be called, but we still declare it to avoid link errors
     *       as it might still be implemented in user code
     */
    int printLog(const char* msg, size_t len);

} // namespace Debug
} // namespace Modbus


// ===================================================================================
// LOG FUNCTIONS - EZMODBUS_DEBUG ENABLED
// ===================================================================================

#ifdef EZMODBUS_DEBUG

namespace Modbus {
namespace Debug {

// Helper to decode file name
size_t buildPrefix(char* dst, size_t dstSize, const CallCtx& ctx);

// Log functions to be called in library code
void LOG_MSG(const char* message = "", CallCtx ctx = CallCtx());
void LOG_MSGF_CTX(CallCtx ctx, const char* userFmt, ...);
#define LOG_MSGF(format, ...) LOG_MSGF_CTX(CallCtx(), format, ##__VA_ARGS__)
void LOG_HEXDUMP(const ByteBuffer& bytes, CallCtx ctx = CallCtx());
void LOG_FRAME(const Modbus::Frame& frame, const char* desc = nullptr, CallCtx ctx = CallCtx());

} // namespace Debug
} // namespace Modbus


#else // EZMODBUS_DEBUG

// ===================================================================================
// NO-OP LOG FUNCTIONS - EZMODBUS_DEBUG DISABLED
// ===================================================================================

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