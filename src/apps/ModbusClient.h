/**
 * @file ModbusClient.h
 * @brief Modbus client class header
 */

#pragma once

#include "core/ModbusCore.h"
#include "interfaces/ModbusInterface.hpp"
#include "utils/ModbusDebug.hpp"
#include <cmath>

#ifndef EZMODBUS_CLIENT_REQ_TIMEOUT // Client request timeout (ms)
    #define EZMODBUS_CLIENT_REQ_TIMEOUT 1000
#endif

namespace Modbus {

class Client {
public:
    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    static constexpr uint32_t DEFAULT_REQUEST_TIMEOUT_MS = (uint32_t)EZMODBUS_CLIENT_REQ_TIMEOUT; // Max RTT before aborting the current request
    static constexpr EventBits_t SYNC_COMPLETION_BIT = 0x01; // Bitmask to set for sync path in sendRequest()


    // ===================================================================================
    // RESULT TYPES
    // ===================================================================================

    enum Result {
        SUCCESS,
        NODATA,
        ERR_INVALID_FRAME,
        ERR_BUSY,
        ERR_TX_FAILED,
        ERR_TIMEOUT,
        ERR_INVALID_RESPONSE,
        ERR_NOT_INITIALIZED,
        ERR_INIT_FAILED,
        ERR_TIMER_FAILURE
    };
    static constexpr const char* toString(const Result result) {
        switch (result) {
            case SUCCESS: return "success";
            case NODATA: return "no data (ongoing transaction)";
            case ERR_INVALID_FRAME: return "invalid frame";
            case ERR_BUSY: return "busy";
            case ERR_TX_FAILED: return "tx failed";
            case ERR_TIMEOUT: return "timeout";
            case ERR_INVALID_RESPONSE: return "invalid response";
            case ERR_NOT_INITIALIZED: return "client not initialized";
            case ERR_INIT_FAILED: return "init failed";
            case ERR_TIMER_FAILURE: return "FreeRTOS timer failure";
            default: return "unknown result";
        }
    }

     // Helper to cast an error
    // - Returns a Result
    // - Captures point of call context & prints a log message when debug 
    // is enabled. No overhead when debug is disabled (except for
    // the desc string, if any)
    static inline Result Error(Result res, const char* desc = nullptr
                        #ifdef EZMODBUS_DEBUG
                        , Modbus::Debug::CallCtx ctx = Modbus::Debug::CallCtx()
                        #endif
                        ) {
        #ifdef EZMODBUS_DEBUG
            if (desc && *desc != '\0') {
                Modbus::Debug::LOG_MSGF_CTX(ctx, "Error: %s (%s)", toString(res), desc);
            } else {
                Modbus::Debug::LOG_MSGF_CTX(ctx, "Error: %s", toString(res));
            }
        #endif
        return res;
    }

    // Helper to cast a success
    // - Returns Result::SUCCESS
    // - Captures point of call context & prints a log message when debug 
    // is enabled. No overhead when debug is disabled (except for
    // the desc string, if any)
    static inline Result Success(const char* desc = nullptr
                          #ifdef EZMODBUS_DEBUG
                          , Modbus::Debug::CallCtx ctx = Modbus::Debug::CallCtx()
                          #endif
                          ) {
        #ifdef EZMODBUS_DEBUG
            if (desc && *desc != '\0') {
                Modbus::Debug::LOG_MSGF_CTX(ctx, "Success: %s", desc);
            }
        #endif
        return SUCCESS;
    }

    // ===================================================================================
    // TYPE ALIAS FOR ASYNCHRONOUS CALLBACKS
    // ===================================================================================

    using ResponseCallback = void (*)(Result result,
                                     const Modbus::Frame* response, // nullptr if no response
                                     void* userCtx);

    // ===================================================================================
    // CONSTRUCTOR & PUBLIC METHODS
    // ===================================================================================

    Client(ModbusInterface::IInterface& interface, uint32_t timeoutMs = DEFAULT_REQUEST_TIMEOUT_MS);
    ~Client();

    Result begin();
    Result sendRequest(const Modbus::Frame& request, 
                       Modbus::Frame& response, Result* userTracker = nullptr);
    Result sendRequest(const Modbus::Frame& request,
                       ResponseCallback cb, void* userCtx = nullptr);
    bool isReady();

    // ===================================================================================
    // CONVENIENCE HELPER METHODS
    // ===================================================================================

    // Read helper template (works for all register types with any arithmetic type)
    template<typename T>
    Result read(uint8_t slaveId, Modbus::RegisterType regType, uint16_t startAddr,
                uint16_t qty, T* dst, Modbus::ExceptionCode* rspExcep = nullptr);

    // Write helper template (works for COIL and HOLDING_REGISTER with any arithmetic type)
    template<typename T>
    Result write(uint8_t slaveId, Modbus::RegisterType regType, uint16_t startAddr,
                 uint16_t qty, const T* src, Modbus::ExceptionCode* rspExcep = nullptr);


private:
    // ===================================================================================
    // PRIVATE DATA STRUCTURES
    // ===================================================================================
    
    /*
    * @class Modbus::Client::PendingRequest
    * @brief The single request in flight: one state machine, one mutex
    *
    * The request state is changed from several contexts: the interface task (response, TX
    * result), the timer daemon (timeout), the calling task (start, synchronous wait fallback)
    * and the destructor. Two invariants keep them consistent:
    *
    * 1. Each transition is a single critical section under _mutex, which checks its condition
    *    on the current state: no decision is taken on a copy of the state read beforehand.
    * 2. Each event carries the condition tying it to its request (no generation counter):
    *    - response: applied only if it matches the active request (slave ID, FC), checked &
    *      applied in the same critical section;
    *    - timeout: ends the active request only if its own deadline is reached, so a timer event
    *      can never end a request early. The timer is never stopped when a request ends: its
    *      late event is harmless, and start() re-arms it (stopping it after the request ends
    *      could cancel the timer of the next one);
    *    - TX result: a new request can't start until the TX result of the previous one came back
    *      (_txPending), so a TX result always belongs to the active request, or to none. This
    *      relies on the IInterface contract: one TX callback per sendFrame() call.
    *
    * Transitions: start() (calling task), onTxResult() (interface task, or calling task on a
    * synchronous sendFrame() failure), onResponse() (interface task), timeoutCallback() (timer
    * daemon), finishSyncWait() (calling task), deleteTimer() (destruction).
    * The interface abort is posted under _mutex: before a newer request can start, so that it
    * can't cancel it (abortCurrentTransaction() is short & never calls the application back).
    * Completion callbacks are called outside _mutex: a new request may start meanwhile.
    */
    class PendingRequest {
    public:
        PendingRequest(Client* client, TickType_t timeoutTicks) : _client(client), _timeoutTicks(timeoutTicks) {}
        ~PendingRequest();

        // Transitions
        Result start(const Modbus::Frame& request, Modbus::Frame* response, Result* tracker,
                     ResponseCallback cb, void* cbCtx, EventGroupHandle_t waiter);
        void onTxResult(ModbusInterface::IInterface::Result result);
        bool onResponse(const Modbus::Frame& response);
        void finishSyncWait(EventGroupHandle_t waiter);
        void deleteTimer();
        void detach();

        // State
        bool isIdle();   // No active request & no TX result awaited
        bool isActive();
        bool reserveSyncWaiter();  // The synchronous waiter storage is in use from reservation...
        void releaseSyncWaiter();  // ... until the caller deleted its event group
        TickType_t getTimeoutTicks() const { return _timeoutTicks; }

    private:
        // Outcome to deliver outside _mutex
        struct Completion {
            ResponseCallback cb = nullptr;
            void* cbCtx = nullptr;
            Result result = NODATA;
            const Modbus::Frame* response = nullptr;
            void deliver() const { if (cb) cb(result, response, cbCtx); }
        };

        Client* _client;                              // Parent Client (interface & response buffer)
        const TickType_t _timeoutTicks;               // Request timeout (at least 1 tick)
        Mutex _mutex;                                 // Protects all the fields below

        // Active request
        bool _active = false;                         // A request waits for its outcome
        bool _txPending = false;                      // The TX result of the last request didn't come back yet
        bool _syncWaiterInUse = false;                // _waiterEventGroupBuf is used by a synchronous call
        Modbus::FrameMeta _meta;                      // Request metadata (no payload)
        TickType_t _startTick = 0;                    // Deadline = _startTick + _timeoutTicks

        // Outputs of the active request
        Modbus::Frame* _pResponse = nullptr;          // Response storage (sync or async w/ tracker)
        Result* _tracker = nullptr;                   // Result tracker (sync or async w/ tracker)
        ResponseCallback _cb = nullptr;               // Completion callback (async w/ callback)
        void* _cbCtx = nullptr;                       // Completion callback context
        EventGroupHandle_t _waiterEventGroup = nullptr; // Synchronous waiter

        // Timeout timer
        StaticTimer_t _timerBuf;                      // Static timer buffer
        TimerHandle_t _timer = nullptr;               // One-shot timer, re-armed by start()
        BinarySemaphore _timerDeleteFence;            // Signals that the timer daemon processed the timer deletion

        // Under _mutex
        bool deadlineReachedLocked() const {
            return (TickType_t)(xTaskGetTickCount() - _startTick) >= _timeoutTicks; // Unsigned: wrap-safe
        }
        Completion finishLocked(Result result, const Modbus::Frame* response);
        static void timeoutCallback(TimerHandle_t timer);
    };

    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    ModbusInterface::IInterface& _interface;
    PendingRequest _pendingRequest;
    Modbus::Frame _responseBuffer;
    bool _isInitialized = false;
    StaticEventGroup_t _waiterEventGroupBuf{}; // Event group buffer for synchronous wait (sync mode)

    // Helper methods members
    Modbus::Frame _helperBuffer;             // Reusable buffer for helper methods
    Mutex _helperMutex;                      // Mutex to protect helper buffer access

    // ===================================================================================
    // PRIVATE METHODS
    // ===================================================================================

    Result handleResponse(const Modbus::Frame& response);

    // ===================================================================================
    // TX RESULT CALLBACK
    // ===================================================================================

    static void staticHandleTxResult(ModbusInterface::IInterface::Result result, void* pClient);

};

// ===================================================================================
// TEMPLATE IMPLEMENTATIONS FOR READ/WRITE HELPERS
// ===================================================================================

/* @brief Send read request and write response data & exception code (if any) to provided buffer
 * @tparam T Arithmetic type for the user buffer (bool, uint16_t, etc.)
 * @param slaveId Target slave ID
 * @param regType Register type (COIL, DISCRETE_INPUT, HOLDING_REGISTER, INPUT_REGISTER)
 * @param startAddr Starting address
 * @param qty Number of registers/coils to read
 * @param dst Buffer to store the read values (should have at least `qty` elements)
 * @param rspExcep Pointer to exception response
 * @return Result code (SUCCESS = transaction succeeded with slave response, check rspExcep for Modbus exceptions)
 * @note For coils: values are 0 or 1. For registers: values are clamped to T's max if sizeof(T) < 2 bytes.
 * @note Floating-point types: Values are truncated (not rounded) when converting from uint16_t. E.g., 123 → 123.0
 * @note IMPORTANT: make sure your buffer size is at least equal to `qty`! The method has no way to enforce this.
 */
template<typename T>
Client::Result Client::read(uint8_t slaveId, Modbus::RegisterType regType, uint16_t startAddr,
                            uint16_t qty, T* dst, Modbus::ExceptionCode* rspExcep) {
    static_assert(std::is_arithmetic<T>::value, "Client::read() only works with arithmetic types (bool, int, float, etc.)");

    // Validate parameters
    if (!dst || qty == 0) {
        return Error(ERR_INVALID_FRAME, "invalid buffer or quantity");
    }

    // Thread-safe access to helper buffer
    Lock guard(_helperMutex);

    // Build request in helper buffer
    _helperBuffer.clear();
    _helperBuffer.type = Modbus::REQUEST;
    _helperBuffer.slaveId = slaveId;
    _helperBuffer.regAddress = startAddr;
    _helperBuffer.regCount = qty;

    // Set function code based on register type
    switch (regType) {
        case Modbus::COIL:
            _helperBuffer.fc = Modbus::READ_COILS;
            break;
        case Modbus::DISCRETE_INPUT:
            _helperBuffer.fc = Modbus::READ_DISCRETE_INPUTS;
            break;
        case Modbus::HOLDING_REGISTER:
            _helperBuffer.fc = Modbus::READ_HOLDING_REGISTERS;
            break;
        case Modbus::INPUT_REGISTER:
            _helperBuffer.fc = Modbus::READ_INPUT_REGISTERS;
            break;
        default:
            return Error(ERR_INVALID_FRAME, "invalid register type");
    }

    // Send request synchronously
    Result res = sendRequest(_helperBuffer, _helperBuffer);

    // Handle transport result
    if (res != SUCCESS) {
        if (rspExcep) *rspExcep = Modbus::NULL_EXCEPTION;
        return Error(res);
    }

    // Transport success - check for Modbus exception
    if (_helperBuffer.exceptionCode != Modbus::NULL_EXCEPTION) {
        if (rspExcep) *rspExcep = _helperBuffer.exceptionCode;
        return Success(); // Transport OK, but Modbus exception
    }

    // Valid response with data - extract based on type
    if (regType == Modbus::COIL || regType == Modbus::DISCRETE_INPUT) {
        // For coils: extract each bit as T (0 or 1)
        for (uint16_t i = 0; i < qty; i++) {
            dst[i] = _helperBuffer.getCoil(i) ? static_cast<T>(1) : static_cast<T>(0);
        }
    } else {
        // For registers: copy with clamping if needed
        for (uint16_t i = 0; i < qty; i++) {
            uint16_t val = _helperBuffer.getRegister(i);

            // Clamp if T cannot contain uint16_t
            if constexpr (std::numeric_limits<T>::max() < std::numeric_limits<uint16_t>::max()) {
                if (val > std::numeric_limits<T>::max()) {
                    dst[i] = std::numeric_limits<T>::max();
                    continue;
                }
            }

            dst[i] = static_cast<T>(val);
        }
    }

    if (rspExcep) *rspExcep = Modbus::NULL_EXCEPTION;
    return Success();
}

/* @brief Send write request from provided buffer, write exception code (if any) to provided buffer
 * @tparam T Arithmetic type for the buffer (bool, int8_t, uint16_t, float, etc.)
 * @param slaveId Target slave ID
 * @param regType Register type (COIL or HOLDING_REGISTER only)
 * @param startAddr Starting address
 * @param qty Number of registers/coils to write
 * @param src Buffer containing values to write (should have at least `qty` elements - values will be clamped to bool/uint16_t range if needed)
 * @param rspExcep Optional pointer to store exception code from response
 * @return Result code (SUCCESS = transaction succeeded with slave response, check rspExcep for Modbus exceptions)
 * @note For coils: values > 0 are considered as `true` state. For registers: values are clamped to UINT16_MAX if needed.
 * @note If using signed numbers, negative write values will be clamped to 0.
 * @note Returns ERR_INVALID_FRAME for a quantity above the Modbus maximum, or a NaN value.
 * @note Floating-point types: Values are truncated (not rounded) when converting to uint16_t. E.g., 123.7 → 123
 * @note IMPORTANT: make sure your buffer size is at least equal to `qty`! The method has no way to enforce this.
 */
template<typename T>
Client::Result Client::write(uint8_t slaveId, Modbus::RegisterType regType, uint16_t startAddr,
                             uint16_t qty, const T* src, Modbus::ExceptionCode* rspExcep) {
    static_assert(std::is_arithmetic<T>::value, "Client::write() only works with arithmetic types (bool, int, float, etc.)");

    // Validate parameters
    if (!src || qty == 0) {
        return Error(ERR_INVALID_FRAME, "invalid buffer or quantity");
    }

    // Only COIL and HOLDING_REGISTER can be written
    if (regType != Modbus::COIL && regType != Modbus::HOLDING_REGISTER) {
        return Error(ERR_INVALID_FRAME, "register type not writable");
    }

    // Quantity within the Modbus maximum (values are then written straight to the frame)
    const size_t maxQty = (regType == Modbus::COIL) ? Modbus::MAX_COILS_WRITE : Modbus::MAX_REGISTERS_WRITE;
    if (qty > maxQty) {
        return Error(ERR_INVALID_FRAME, "quantity exceeds the Modbus maximum");
    }

    // NaN has no register or coil value
    if constexpr (std::is_floating_point<T>::value) {
        for (uint16_t i = 0; i < qty; i++) {
            if (std::isnan(src[i])) return Error(ERR_INVALID_FRAME, "NaN value");
        }
    }

    // Thread-safe access to helper buffer
    Lock guard(_helperMutex);

    // Build request in helper buffer
    _helperBuffer.clear();
    _helperBuffer.type = Modbus::REQUEST;
    _helperBuffer.slaveId = slaveId;
    _helperBuffer.regAddress = startAddr;
    _helperBuffer.regCount = qty;

    // Set function code and data based on quantity and type
    if (regType == Modbus::COIL) {
        if (qty == 1) {
            _helperBuffer.fc = Modbus::WRITE_COIL;
        } else {
            _helperBuffer.fc = Modbus::WRITE_MULTIPLE_COILS;
        }

        // Convert T to coil states (0 or 1) - no cast, just check != 0
        for (uint16_t i = 0; i < qty; i++) {
            const bool coil = src[i] != 0;
            _helperBuffer.setCoils(&coil, 1, i);
        }
    } else { // HOLDING_REGISTER
        if (qty == 1) {
            _helperBuffer.fc = Modbus::WRITE_REGISTER;
        } else {
            _helperBuffer.fc = Modbus::WRITE_MULTIPLE_REGISTERS;
        }

        // Convert T to uint16_t for registers with clamping
        for (uint16_t i = 0; i < qty; i++) {
            T val = src[i];
            uint16_t& reg = _helperBuffer.data[i];

            // Clamp negative signed numbers to 0
            if constexpr (std::is_signed<T>::value) {
                if (val < 0) {
                    reg = 0;
                    continue;
                }
            }

            // Clamp to UINT16_MAX for > 16-bit types
            if constexpr (std::numeric_limits<T>::max() > std::numeric_limits<uint16_t>::max()) {
                if (val > std::numeric_limits<uint16_t>::max()) {
                    reg = std::numeric_limits<uint16_t>::max();
                    continue;
                }
            }

            reg = static_cast<uint16_t>(val);
        }
    }

    // Send request synchronously
    Result res = sendRequest(_helperBuffer, _helperBuffer);

    // Handle transport result
    if (res != SUCCESS) {
        if (rspExcep) *rspExcep = Modbus::NULL_EXCEPTION;
        return Error(res);
    }

    // Transport success - check for Modbus exception
    if (_helperBuffer.exceptionCode != Modbus::NULL_EXCEPTION) {
        if (rspExcep) *rspExcep = _helperBuffer.exceptionCode;
        return Success(); // Transport OK, but Modbus exception
    }

    // Write successful
    if (rspExcep) *rspExcep = Modbus::NULL_EXCEPTION;
    return Success();
}

} // namespace Modbus