// test/test_codec/mock/ModbusDebug.hpp
#ifndef MODBUS_DEBUG_H  
#define MODBUS_DEBUG_H

#include <stdio.h>

#ifdef __cplusplus
#include <iostream>
#include <string>

#include "core/ModbusTypes.hpp" // CallCtx definition

#ifdef EZMODBUS_DEBUG

namespace Modbus   {
namespace Debug    {

    inline const char* getBasename(const char* p) {
        const char *b = p;
        const char *s = strrchr(p, '/');  if (s) b = s + 1;
        s = strrchr(p, '\\');             if (s && s > b) b = s + 1;
        return b;
    }
    inline void LOG_MSG(const std::string& msg,
                        const Modbus::CallCtx& ctx) {
        std::printf("[%s::%s:%d] %s\n",
                    ctx.file, ctx.function, ctx.line, msg.c_str());
    }

    // Lightweight printf-style logger with context (variadic template)
    template<typename... Args>
    inline void LOG_MSGF_CTX(const Modbus::CallCtx& ctx, const char* fmt, Args&&... args) {
        std::printf("[%s::%s:%d] ", getBasename(ctx.file), ctx.function, ctx.line);
        std::printf(fmt, std::forward<Args>(args)...);
        std::printf("\n");
    }

    // Convenience macro matching production header
    #define LOG_MSGF(format, ...) Modbus::Debug::LOG_MSGF_CTX(Modbus::CallCtx(), format, ##__VA_ARGS__)

    // Hexdump stub (prints size only to keep it simple)
    inline void LOG_HEXDUMP(const std::vector<uint8_t>& bytes, const char* desc = "", Modbus::CallCtx ctx = Modbus::CallCtx()) {
        std::printf("[%s::%s:%d] %s <hexdump len=%u>\n", getBasename(ctx.file), ctx.function, ctx.line,
                    desc ? desc : "", (uint32_t)bytes.size());
    }

    // Frame stub – prints basic metadata, avoids full dependency on Frame structure
    template<typename T>
    inline void LOG_FRAME(const T&, const char* desc = "", Modbus::CallCtx ctx = Modbus::CallCtx()) {
        std::printf("[%s::%s:%d] %s <frame log suppressed in test>\n",
                    getBasename(ctx.file), ctx.function, ctx.line, desc ? desc : "");
    }

    inline void LOG_HEXDUMP(const std::vector<uint8_t>& bytes, Modbus::CallCtx ctx = Modbus::CallCtx()) {
        std::printf("[%s::%s:%d] Hexdump: <len=%u>\n", getBasename(ctx.file), ctx.function, ctx.line, (uint32_t)bytes.size());
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
    inline void LOG_MSGF_CTX(Args&&...) {}
    template<typename... Args>
    inline void LOG_MSGF(Args&&...) {}
    template<typename... Args>
    inline void LOG_HEXDUMP(const std::vector<uint8_t>&, Modbus::CallCtx = Modbus::CallCtx()) {}
    template<typename... Args>
    inline void LOG_HEXDUMP(Args&&...) {}
    template<typename... Args>
    inline void LOG_FRAME(Args&&...) {}

} // namespace Debug
} // namespace Modbus

#endif // EZMODBUS_DEBUG

#endif  // __cplusplus

#endif  // MODBUS_DEBUG_H
