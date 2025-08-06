// @file ModbusResultHelpers.inl
// @brief Helper functions for Modbus results (cast Success/Error)
// @brief This file is directly included in all header files that use a Result type

// ===================================================================================
// ACTUAL HELPER FUNCTIONS - STANDARD VERSION
// ===================================================================================

#ifndef EZMODBUS_USE_STATIC_RESULT_HELPERS

// Helper to cast an error
// - Returns a Result
// - Captures point of call context & prints a log message when debug 
// is enabled. No overhead when debug is disabled.
// - Also pushes an event to the event bus if enabled
inline Result Error(Result res, const char* desc = nullptr
                    #if (defined(EZMODBUS_DEBUG) || defined(EZMODBUS_EVENTBUS))
                    , CallCtx ctx = CallCtx()
                    #endif
                    ) const {
    #if defined(EZMODBUS_DEBUG)
        if (desc && *desc != '\0') {
            Modbus::Debug::LOG_MSGF_CTX(ctx, "Error: %s (%s)", toString(res), desc);
        } else {
            Modbus::Debug::LOG_MSGF_CTX(ctx, "Error: %s", toString(res));
        }
    #endif
    #if defined(EZMODBUS_EVENTBUS)
        // Capture address when calling from a class instance
        Modbus::EventBus::pushResult(static_cast<uint16_t>(res), toString(res), this, ctx); 
    #endif
    return res;
}

// Helper to cast a success
// - Returns Result::SUCCESS
// - Captures point of call context & prints a log message when debug 
// is enabled. No overhead when debug is disabled.
// - No event is pushed to the event bus (we only log error events to KISS)
inline Result Success(const char* desc = nullptr
                        #if defined(EZMODBUS_DEBUG)
                        , CallCtx ctx = CallCtx()
                        #endif
                        ) const {
    #if defined(EZMODBUS_DEBUG)
        if (desc && *desc != '\0') {
            Modbus::Debug::LOG_MSGF_CTX(ctx, "Success: %s", desc);
        }
    #endif
    return SUCCESS;
}

// ===================================================================================
// ACTUAL HELPER FUNCTIONS - STATIC VERSION
// ===================================================================================

#else // EZMODBUS_USE_STATIC_RESULT_HELPERS

// Helper to cast an error
// - Returns a Result
// - Captures point of call context & prints a log message when debug 
// is enabled. No overhead when debug is disabled.
// - Also pushes an event to the event bus if enabled
static inline Result Error(Result res, const char* desc = nullptr
                    #if (defined(EZMODBUS_DEBUG) || defined(EZMODBUS_EVENTBUS))
                    , CallCtx ctx = CallCtx()
                    #endif
                    ) {
    #if defined(EZMODBUS_DEBUG)
        if (desc && *desc != '\0') {
            Modbus::Debug::LOG_MSGF_CTX(ctx, "Error: %s (%s)", toString(res), desc);
        } else {
            Modbus::Debug::LOG_MSGF_CTX(ctx, "Error: %s", toString(res));
        }
    #endif
    #if defined(EZMODBUS_EVENTBUS)
        // Use the raw method with nullptr instance address when calling from a static class/function (e.g. ModbusCodec)
        Modbus::EventBus::pushResult(static_cast<uint16_t>(res), toString(res), nullptr, ctx);
    #endif
    return res;
}

// Helper to cast a success
// - Returns Result::SUCCESS
// - Captures point of call context & prints a log message when debug 
// is enabled. No overhead when debug is disabled.
// - No event is pushed to the event bus (we only log error events to KISS)
static inline Result Success(const char* desc = nullptr
                        #if defined(EZMODBUS_DEBUG)
                        , CallCtx ctx = CallCtx()
                        #endif
                        ) {
    #if defined(EZMODBUS_DEBUG)
        if (desc && *desc != '\0') {
            Modbus::Debug::LOG_MSGF_CTX(ctx, "Success: %s", desc);
        }
    #endif
    return SUCCESS;
}

#endif // EZMODBUS_USE_STATIC_RESULT_HELPERS
