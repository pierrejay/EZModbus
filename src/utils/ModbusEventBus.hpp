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

/* @brief Event type for EventBus records */
enum EventType : uint8_t {
    EVENT_ERROR = 0,   // Error/exception event
    EVENT_REQUEST = 1  // Successful request event
};

class EventBus {
public:

    // ===================================================================================
    // TYPES
    // ===================================================================================
    
    struct Record {
        // Event classification
        EventType eventType;     // ERROR or REQUEST
        // Payload
        uint16_t result;         // Error/result code (enum casted)
        const char* resultStr;   // toString(enum) - static string
        const char* desc;        // Additional description - static string
        // Modbus request info (valid only if eventType == EVENT_REQUEST)
        Modbus::FrameMeta requestInfo; // Request metadata (POD struct)
        // Context / origin
        uintptr_t instance;      // Caller instance address
        uint64_t timestampUs;    // TIME_US()
        const char* fileName;    // Basename
        uint16_t lineNo;         // Line number
    };

    
    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    static constexpr size_t QUEUE_SIZE = EZMODBUS_EVENTBUS_Q_SIZE;
    static constexpr size_t INSTANCE_FILTER_SIZE = EZMODBUS_EVENTBUS_INSTANCE_FILTER_SIZE;
    static constexpr size_t EVT_SIZE = sizeof(Record);


    // ===================================================================================
    // "PRODUCER" PUBLIC METHODS (used internally in lib)
    // ===================================================================================

#if defined(EZMODBUS_EVENTBUS) && !defined(NATIVE_TEST)

    /* @brief Push an event to the event bus
     * @param res Result code
     * @param desc Additional description
     * @param instance Instance of caller class
     * @param ctx Call context (optional - hidden parameter)
     * @note This version is used for calls from class instances that have a Result type & toString() method
     */
    template<typename T>
    static inline void push(typename T::Result res, 
                            const char* desc, 
                            const T* instance, 
                            ModbusTypeDef::CallCtx ctx = ModbusTypeDef::CallCtx()) {
        if (!begin()) return;
        
        uintptr_t instanceAddr = reinterpret_cast<uintptr_t>(instance);
        if (_isFiltered(instanceAddr)) return;
        
        Record rcd = {
            .eventType = EVENT_ERROR,
            .result = static_cast<uint16_t>(res),
            .resultStr = T::toString(res),
            .desc = desc,
            .requestInfo = {}, // Default (invalid) RequestInfo
            .instance = instanceAddr,
            .timestampUs = TIME_US(),
            .fileName = ModbusTypeDef::getBasename(ctx.file),
            .lineNo = static_cast<uint16_t>(ctx.line)
        };
        
        _send(rcd);
    }
    
    /* @brief Push an event to the event bus from ISR
     * @param res Result code
     * @param desc Additional description
     * @param instance Instance of caller class
     * @param ctx Call context (optional - hidden parameter)
     * @note This version is used for calls from class instances that have a Result type & toString() method
     */
    template<typename T>
    static inline void pushFromISR(typename T::Result res, 
                                    const char* desc, 
                                    const T* instance, 
                                    ModbusTypeDef::CallCtx ctx) {
        if (!begin()) return;
        
        uintptr_t instanceAddr = reinterpret_cast<uintptr_t>(instance);
        if (_isFiltered(instanceAddr)) return;
        
        Record rcd = {
            .eventType = EVENT_ERROR,
            .result = static_cast<uint16_t>(res),
            .resultStr = T::toString(res),
            .desc = desc,
            .requestInfo = {}, // Default (invalid) RequestInfo
            .instance = instanceAddr,
            .timestampUs = TIME_US(),
            .fileName = ModbusTypeDef::getBasename(ctx.file),
            .lineNo = static_cast<uint16_t>(ctx.line)
        };
        
        _sendFromISR(rcd);
    }

    /* @brief Push an event to the event bus
     * @param res Result code
     * @param resultStr Result string
     * @param desc Additional description
     * @param instance Instance of caller class
     * @param ctx Call context (optional - hidden parameter)
     * @note This version is used for calls from static classes/functions (e.g. ModbusCodec)
     */
    static inline void pushRaw(uint16_t res, 
                            const char* resultStr,
                            const char* desc, 
                            const void* instance, 
                            ModbusTypeDef::CallCtx ctx = ModbusTypeDef::CallCtx()) {
        if (!begin()) return;
        
        uintptr_t instanceAddr = reinterpret_cast<uintptr_t>(instance);
        if (_isFiltered(instanceAddr)) return;
        
        Record rcd = {
            .eventType = EVENT_ERROR,
            .result = res,
            .resultStr = resultStr,
            .desc = desc,
            .requestInfo = {}, // Default (invalid) RequestInfo
            .instance = instanceAddr,
            .timestampUs = TIME_US(),
            .fileName = ModbusTypeDef::getBasename(ctx.file),
            .lineNo = static_cast<uint16_t>(ctx.line)
        };
        
        _send(rcd);
    }
    
    /* @brief Push an event to the event bus from ISR
     * @param res Result code
     * @param resultStr Result string
     * @param desc Additional description
     * @param instance Instance of caller class
     * @param ctx Call context (optional - hidden parameter)
     * @note This version is used for calls from static classes/functions (e.g. ModbusCodec)
     */
    static inline void pushRawFromISR(uint16_t res, 
                                    const char* resultStr,
                                    const char* desc, 
                                    const void* instance, 
                                    ModbusTypeDef::CallCtx ctx) {
        if (!begin()) return;
        
        uintptr_t instanceAddr = reinterpret_cast<uintptr_t>(instance);
        if (_isFiltered(instanceAddr)) return;
        
        Record rcd = {
            .eventType = EVENT_ERROR,
            .result = res,
            .resultStr = resultStr,
            .desc = desc,
            .requestInfo = {}, // Default (invalid) RequestInfo
            .instance = instanceAddr,
            .timestampUs = TIME_US(),
            .fileName = ModbusTypeDef::getBasename(ctx.file),
            .lineNo = static_cast<uint16_t>(ctx.line)
        };
        
        _sendFromISR(rcd);
    }

    /* @brief Push a successful request event to the event bus (optimized for success cases)
     * @param request Modbus request frame (converted to metadata internally)
     * @param instance Instance of caller class
     * @param ctx Call context (optional - hidden parameter)
     * @note This version is optimized for successful request logging - no string formatting overhead
     */
    template<typename T>
    static inline void pushRequest(const Modbus::Frame& request,
                                   const T* instance,
                                   ModbusTypeDef::CallCtx ctx = ModbusTypeDef::CallCtx()) {
        if (!begin()) return;
        
        uintptr_t instanceAddr = reinterpret_cast<uintptr_t>(instance);
        if (_isFiltered(instanceAddr)) return;
        
        Record rcd = {
            .eventType = EVENT_REQUEST,
            .result = static_cast<uint16_t>(T::SUCCESS),
            .resultStr = T::toString(T::SUCCESS),
            .desc = nullptr, // No description needed for success
            .requestInfo = Modbus::FrameMeta(request), // Convert Frame to FrameMeta
            .instance = instanceAddr,
            .timestampUs = TIME_US(),
            .fileName = ModbusTypeDef::getBasename(ctx.file),
            .lineNo = static_cast<uint16_t>(ctx.line)
        };
        
        _send(rcd);
    }

    // ===================================================================================
    // "CONSUMER" PUBLIC METHODS (USER API)
    // ===================================================================================

    /* @brief Initialize the event bus, or check if it is already initialized
     * @return True if initialization was successful, false otherwise
     */
    static bool begin() {
        if (_initialized) return true; // Already initialized

        ModbusTypeDef::Lock lock(_initMutex);         // Not initialized: try to lock the mutex
        if (_initialized) return true; // Double check if another thread already did the job in the meantime
        
        _queue = xQueueCreateStatic(QUEUE_SIZE, sizeof(Record), (uint8_t*)_queueStorage, &_queueBuffer);
        if (!_queue) return false;
        
        _droppedCount = 0;
        
        // Clear filter instances
        for (size_t i = 0; i < INSTANCE_FILTER_SIZE; i++) {
            _filteredInstances[i] = 0;
        }
        
        _initialized = true;
        return true;
    }
    
    static bool pop(Record& rcd, uint32_t timeoutMs = 0) {
        if (!_initialized) return false;
        return xQueueReceive(_queue, &rcd, pdMS_TO_TICKS(timeoutMs)) == pdPASS;
    }
    
    static bool filterOut(void* instance) {
        uintptr_t instanceAddr = (uintptr_t)instance;
        
        // Find first empty slot
        for (size_t i = 0; i < INSTANCE_FILTER_SIZE; i++) {
            if (_filteredInstances[i] == 0) {
                _filteredInstances[i] = instanceAddr;
                return true;
            }
        }
        return false; // No slot left
    }
    
    /* @brief Get the number of dropped events
     * @return Number of dropped events
     */
    static uint32_t getDroppedCount() {
        return _droppedCount;
    }

private:

    // ===================================================================================
    // PRIVATE METHODS
    // ===================================================================================

    /* @brief Check if an instance is filtered out
     * @param instanceAddr Instance address
     * @return True if the instance is filtered out, false otherwise
     */
    static inline bool _isFiltered(uintptr_t instanceAddr) {
        for (size_t i = 0; i < INSTANCE_FILTER_SIZE; i++) {
            if (_filteredInstances[i] == instanceAddr) return true;
        }
        return false;
    }
    
    /* @brief Send a record to the event bus
     * @param rcd Record to send
     */
    static inline void _send(const Record& rcd) {
        if (xQueueSend(_queue, &rcd, 0) != pdPASS) {
            _droppedCount = _droppedCount + 1;
        }
    }
    
    /* @brief Send a record to the event bus from ISR
     * @param rcd Record to send
     */
    static inline void _sendFromISR(const Record& rcd) {
        // Do not yield to higher priority task, logs are not time-critical.
        // Let the library be able to wake its own HP tasks in the ISR, they
        // are probably more important than waking the logs consumer...
        if (xQueueSendFromISR(_queue, &rcd, NULL) != pdPASS) {
            _droppedCount = _droppedCount + 1;
        }
    }

    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    // State
    inline static volatile bool _initialized = false;
    inline static ModbusTypeDef::Mutex _initMutex; // Mutex to prevent race condition on initialization
    inline static volatile uint32_t _droppedCount = 0;
    
    // Static FreeRTOS objects
    inline static StaticQueue_t _queueBuffer;
    inline static Record _queueStorage[QUEUE_SIZE];
    inline static QueueHandle_t _queue = nullptr;
    
    // Instance filtering
    inline static uintptr_t _filteredInstances[INSTANCE_FILTER_SIZE];
    

#else // EZMODBUS_EVENTBUS not defined or NATIVE_TEST

    // If EZMODBUS_EVENTBUS is not defined, use no-op templates to totally
    // disable calls to class methods (will not evaluate args)

    template<typename... Args>
    static void push(Args&&...) {}
    
    template<typename... Args>
    static void pushFromISR(Args&&...) {}
    
    template<typename... Args>
    static bool begin(Args&&...) { return true; }
    
    template<typename... Args>
    static bool pop(Args&&...) { return false; }
    
    template<typename... Args>
    static bool filterOut(Args&&...) { return true; }
    
    template<typename... Args>
    static uint32_t getDroppedCount(Args&&...) { return 0; }

#endif // EZMODBUS_EVENTBUS && !NATIVE_TEST
};

} // namespace Modbus