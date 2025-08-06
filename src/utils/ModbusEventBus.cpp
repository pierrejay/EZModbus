#include "ModbusEventBus.hpp"

#if defined(EZMODBUS_EVENTBUS) && !defined(NATIVE_TEST)

namespace Modbus {

// ===================================================================================
// STATIC MEMBERS DEFINITION
// ===================================================================================

volatile bool           EventBus::_initialized = false;
ModbusTypeDef::Mutex    EventBus::_initMutex;
volatile uint32_t       EventBus::_droppedCount = 0;
StaticQueue_t           EventBus::_queueBuffer;
EventBus::Record        EventBus::_queueStorage[EventBus::QUEUE_SIZE];
QueueHandle_t           EventBus::_queue = nullptr;
uintptr_t               EventBus::_filteredInstances[EventBus::INSTANCE_FILTER_SIZE];


// ===================================================================================
// "PRODUCER" PUBLIC METHODS (used internally in lib)
// ===================================================================================

/* @brief Push an event to the event bus
 * @param res Result code (casted to uint16_t)
 * @param resultStr Plaintext Result string
 * @param instance Instance of caller class
 * @param ctx Call context (optional - hidden parameter)
 */
void EventBus::pushResult(uint16_t res, 
                             const char* resultStr,
                             const void* instance, 
                             ModbusTypeDef::CallCtx ctx) {
    if (!begin()) return;
    
    uintptr_t instanceAddr = reinterpret_cast<uintptr_t>(instance);
    if (_isFiltered(instanceAddr)) return;
    
    Record rcd = {
        .eventType = EVT_RESULT,
        .result = res,
        .resultStr = resultStr,
        .requestInfo = {}, // Default (invalid) RequestInfo
        .instance = instanceAddr,
        .timestampUs = TIME_US(),
        .fileName = ModbusTypeDef::getBasename(ctx.file),
        .lineNo = static_cast<uint16_t>(ctx.line)
    };
    
    _send(rcd);
}

/* @brief Push an event to the event bus from ISR
 * @param res Result code (casted to uint16_t)
 * @param resultStr Plaintext Result string
 * @param instance Instance of caller class
 * @param ctx Call context (optional - hidden parameter)
 * @note This version is used for calls from static classes/functions (e.g. ModbusCodec)
 */
void EventBus::pushResultFromISR(uint16_t res, 
                                    const char* resultStr,
                                    const void* instance, 
                                    ModbusTypeDef::CallCtx ctx) {
    if (!begin()) return;
    
    uintptr_t instanceAddr = reinterpret_cast<uintptr_t>(instance);
    if (_isFiltered(instanceAddr)) return;
    
    Record rcd = {
        .eventType = EVT_RESULT,
        .result = res,
        .resultStr = resultStr,
        .requestInfo = {}, // Default (invalid) RequestInfo
        .instance = instanceAddr,
        .timestampUs = TIME_US(),
        .fileName = ModbusTypeDef::getBasename(ctx.file),
        .lineNo = static_cast<uint16_t>(ctx.line)
    };
    
    _sendFromISR(rcd);
}

/* @brief Push a request event to the event bus
 * @param request Modbus request frame converted to FrameMeta
 * @param res Result code for processing the request (casted to uint16_t)
 * @param resultStr Plaintext Result string
 * @param instance Instance of caller class
 * @param ctx Call context (optional - hidden parameter)
 */
void EventBus::pushRequest(const Modbus::FrameMeta& request,
                              uint16_t res,
                              const char* resultStr,
                              const void* instance,
                              ModbusTypeDef::CallCtx ctx) {
    if (!begin()) return;
    
    uintptr_t instanceAddr = reinterpret_cast<uintptr_t>(instance);
    if (_isFiltered(instanceAddr)) return;
    
    Record rcd = {
        .eventType = EVT_REQUEST,
        .result = res,
        .resultStr = resultStr,
        .requestInfo = request,
        .instance = instanceAddr,
        .timestampUs = TIME_US(),
        .fileName = ModbusTypeDef::getBasename(ctx.file),
        .lineNo = static_cast<uint16_t>(ctx.line)
    };
    
    _send(rcd);
}

/* @brief Initialize the event bus, or check if it is already initialized
 * @return True if initialization was successful, false otherwise
 */
bool EventBus::begin() {
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

// ===================================================================================
// "CONSUMER" PUBLIC METHODS (USER API)
// ===================================================================================

bool EventBus::pop(Record& rcd, uint32_t timeoutMs) {
    if (!_initialized) return false;
    return xQueueReceive(_queue, &rcd, pdMS_TO_TICKS(timeoutMs)) == pdPASS;
}

bool EventBus::filterOut(void* instance) {
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
uint32_t EventBus::getDroppedCount() {
    return _droppedCount;
}

// ===================================================================================
// PRIVATE METHODS
// ===================================================================================

/* @brief Check if an instance is filtered out
 * @param instanceAddr Instance address
 * @return True if the instance is filtered out, false otherwise
 */
bool EventBus::_isFiltered(uintptr_t instanceAddr) {
    for (size_t i = 0; i < INSTANCE_FILTER_SIZE; i++) {
        if (_filteredInstances[i] == instanceAddr) return true;
    }
    return false;
}

/* @brief Send a record to the event bus
 * @param rcd Record to send
 */
void EventBus::_send(const Record& rcd) {
    if (xQueueSend(_queue, &rcd, 0) != pdPASS) {
        _droppedCount = _droppedCount + 1;
    }
}

/* @brief Send a record to the event bus from ISR
 * @param rcd Record to send
 */
void EventBus::_sendFromISR(const Record& rcd) {
    // Do not yield to higher priority task, logs are not time-critical.
    // Let the library be able to wake its own HP tasks in the ISR, they
    // are probably more important than waking the logs consumer...
    if (xQueueSendFromISR(_queue, &rcd, NULL) != pdPASS) {
        _droppedCount = _droppedCount + 1;
    }
}

} // namespace Modbus

#endif // EZMODBUS_EVENTBUS && !NATIVE_TEST