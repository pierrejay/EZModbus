/**
 * @file ModbusEventBus.hpp
 * @brief Centralized diagnostic event system for EZModbus
 * @brief Provides a compile-time solution to catch all errors at runtime
 */

#pragma once

#include "core/ModbusTypes.hpp"
#include "core/ModbusCore.h"
#include "core/ModbusFrame.hpp"

#ifndef EZMODBUS_EVENTBUS_Q_SIZE
    #define EZMODBUS_EVENTBUS_Q_SIZE 16
#endif

#ifndef EZMODBUS_EVENTBUS_INSTANCE_FILTER_SIZE
    #define EZMODBUS_EVENTBUS_INSTANCE_FILTER_SIZE 8
#endif

namespace Modbus {

class EventBus {
public:

    // ===================================================================================
    // TYPES
    // ===================================================================================

    /* @brief Event type for EventBus records */
    enum EventType : uint8_t {
        EVT_RESULT = 0,   // Error/exception event
        EVT_REQUEST = 1  // Successful request event
    };
    
    /* @brief Struct storing event data */
    struct Record {
    // Event classification
        EventType eventType;            // EVT_RESULT or EVT_REQUEST
    // Payload
        uint16_t result;                // Error/result code (enum casted)
        const char* resultStr;          // toString(enum) - static string
        Modbus::FrameMeta requestInfo;  // Request metadata (POD) - only for EVT_REQUEST
    // Context / origin
        uintptr_t instance;             // Caller instance address
        uint64_t timestampUs;           // TIME_US()
        const char* fileName;           // Basename
        uint16_t lineNo;                // Line number
    };

    
    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    static constexpr size_t QUEUE_SIZE = EZMODBUS_EVENTBUS_Q_SIZE;
    static constexpr size_t INSTANCE_FILTER_SIZE = EZMODBUS_EVENTBUS_INSTANCE_FILTER_SIZE;
    static constexpr size_t EVT_SIZE = sizeof(Record);


// Methods only defined if EventBus is enabled (and not in native test)
#if defined(EZMODBUS_EVENTBUS) && !defined(NATIVE_TEST)

    // ===================================================================================
    // "PRODUCER" PUBLIC METHODS (used internally in lib)
    // ===================================================================================

    static void pushResult(uint16_t res, 
                              const char* resultStr,
                              const void* instance = nullptr, 
                              ModbusTypeDef::CallCtx ctx = ModbusTypeDef::CallCtx());
    

    static void pushResultFromISR(uint16_t res, 
                                     const char* resultStr,
                                     const void* instance = nullptr, 
                                     ModbusTypeDef::CallCtx ctx = ModbusTypeDef::CallCtx());


    static void pushRequest(const Modbus::FrameMeta& request,
                               uint16_t res,
                               const char* resultStr,
                               const void* instance = nullptr,
                               ModbusTypeDef::CallCtx ctx = ModbusTypeDef::CallCtx());

    static bool begin();


    // ===================================================================================
    // "CONSUMER" PUBLIC METHODS (USER API)
    // ===================================================================================

    static bool pop(Record& rcd, uint32_t timeoutMs = 0);
    static bool filterOut(void* instance);
    static uint32_t getDroppedCount();

private:

    // ===================================================================================
    // PRIVATE METHODS
    // ===================================================================================

    static bool _isFiltered(uintptr_t instanceAddr);
    static void _send(const Record& rcd);
    static void _sendFromISR(const Record& rcd);

    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    // State
    static volatile bool _initialized;
    static ModbusTypeDef::Mutex _initMutex; // Mutex to prevent race condition on initialization
    static volatile uint32_t _droppedCount;
    
    // Static FreeRTOS objects
    static StaticQueue_t _queueBuffer;
    static Record _queueStorage[QUEUE_SIZE];
    static QueueHandle_t _queue;
    
    // Instance filtering
    static uintptr_t _filteredInstances[INSTANCE_FILTER_SIZE];
    

#else // EZMODBUS_EVENTBUS not defined or NATIVE_TEST

    // If EZMODBUS_EVENTBUS is not defined, use no-op templates to totally
    // disable calls to class methods (will not evaluate args)

    template<typename... Args>
    static void pushResult(Args&&...) {}
    
    template<typename... Args>
    static void pushResultFromISR(Args&&...) {}

    template<typename... Args>
    static void pushRequest(Args&&...) {}
    
    template<typename... Args>
    static bool begin(Args&&...) { return true; }
    
    template<typename... Args>
    static bool pop(Args&&...) { return false; }
    
    template<typename... Args>
    static bool filterOut(Args&&...) { return true; }
    
    template<typename... Args>
    static uint32_t getDroppedCount(Args&&...) { return 0; }

#endif // EZMODBUS_EVENTBUS && !NATIVE_TEST

}; // class EventBus

} // namespace Modbus