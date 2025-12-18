/**
 * @file ModbusClient.h
 * @brief Modbus client class header
 */

#pragma once

#include "core/ModbusCore.h"
#include "interfaces/ModbusInterface.hpp"
#include "utils/ModbusDebug.hpp"

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
    static constexpr TickType_t TIMER_CMD_TOUT_TICKS = pdMS_TO_TICKS(5); // Bounded wait for FreeRTOS timer commands


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
    * @brief Lifecycle of a single in-flight request with timeout management & race handling
    *
    * This class coordinates a request/response transaction with timeout driven by a FreeRTOS 
    * one-shot timer, while preventing all practical race conditions between:
    * - the normal path (response or TX-result completion),
    * - and the timeout callback (Timer Service Task).
    * 
    * Concurrency challenge
    * ─────────────────────
    * The challenge lies in that a FreeRTOS timer is managed asynchronously. Despite what the
    * name of the API methods suggest (xTimerStop, xTimerReset, etc.), timers are controlled by
    * sending commands to the Timer Service Task (daemon) command queue. There is no guarantee
    * whether or when those commands execute! We can just be sure that if our command was queued,
    * it will be executed at some point in the future.
    * 
    * The race window is extremely thin but nevertheless exists. The problematic situation is where
    * a request is to be cleaned up by the timeout timer, and at the same time, a response arrives.
    * The naive approach (implemented in earlier versions) cannot guarantee that, if the client is
    * very fast, either the timer callback or the normal response path won't prematurely close a
    * NEW request since from their own context, they have no clue about the "creation epoch" of
    * this new request.
    * 
    * Despite the naive implementation working well in 99.9% of cases, with very limited practical
    * consequences in case of a race (no data or memory corruption, just a client request closed 
    * too early), I wasn't confident with a solution that has conceptual flaws.
    * I decided to address all possible "failure modes", which makes up for a quite sophisticated 
    * solution (though not that complicated) given the absence of native FreeRTOS primitives capable 
    * of solving this specific case - read further below for considered alternative approaches.
    *
    * The design is deliberately epoch-free and single-timer to stay as simple as possible.
    * It relies on four layers of defense that together prevent every realistic race window:
    *
    * Four Lines of Defense
    * ─────────────────────
    * 1. Close Gates (two atomics)
    * Two atomic flags prevent a new request from being armed while the previous one is closing:
    * - _respClosing - set by the response path before attempting to neutralize the timer;
    * - _timerClosing - set by the timeout callback at entry.
    * set() proceeds to start a new request only if !_active && !_respClosing() && !_timerClosing.
    * (checked once pre-mutex for no-lock fail-fast and again under _mutex to avoid TOCTOU).
    *
    * 2. Physical Neutralization (killTimer)
    * Before response path finalization, we try to stop the timer and fence-ACK the daemon queue
    * (bounded wait, max 5 ticks by default at each step) with 3 possible outcomes:
    * - KILLED         -> STOP posted and fence ACK'd: guarantee of no future timeout.
    * - FAILED         -> couldn't post STOP: do not finalize here, timer will finish.
    * - STOPPED_NOACK  -> STOP posted but no fence ACK within budget: indeterminate.
    * In 99.9% of cases, the scheduler will switch smoothly between timer daemon and request
    * context, so that not even one full tick might elapse until the timer is fully neutralized
    * (10 to 100µs-ish latency -> negligible impact on the real-time performance)
    *
    * 3. Logical Disarm (disarmTimerCb / rearmTimerCb)
    * If STOPPED_NOACK, we disarm the timeout callback to prevent any future surprise invocation.
    * The callback checks isTimerCbDisarmed() before any lock/abort and becomes a no-op immediately.
    * This guarantees that a late timeout cannot touch current or future requests.
    *
    * 4. Mutual Exclusion (single _mutex)
    * All state mutations occur under _mutex in setResult(), setResponse(), clear() (via resetUnsafe()),
    * preventing double-finalization of the same request and ensuring atomic signaling of waiters.
    * Both setResult() and setResponse() use RAII closers to guarantee atomic gates are
    * always cleared, even on early returns paths.
    *
    * Centralization (single point of truth)
    * ──────────────────────────────────────
    * Call sites never manage timers directly. They only:
    * - take a read-only snapshot for validation (to avoid long critical sections or torn reads),
    * - then call one of:
    *   - setResponse(frame, finalize) -> only called from response path (not from timer)
    *   - setResult(result, finalize, fromTimer=false)
    *
    * These methods internally:
    * 1. set _respClosing=true (via RAII closer),
    * 2. run killTimer(),
    *    - on FAILED: return immediately (RAII clears _respClosing, timer will finalize),
    *    - on STOPPED_NOACK: disarmTimerCb() (logical neutralization),
    * 3. finalize under _mutex (write tracker/response, _active=false, signal waiters),
    * 4. resetUnsafe() clears _active (gates cleared by RAII automatically).
    * The timeout path calls setResult(..., fromTimer=true) and does not invoke killTimer() nor disarm.
    *
    * Execution Paths (precise order)
    * ───────────────────────────────
    * 1. Normal response (handleResponse)
    * 1.1. Snapshot request metadata (no locks held during validation).
    * 1.2. setResponse(..., finalize=true, fromTimer=false) -> triggers 4-layer policy above.
    *
    * 2. TX result (staticHandleTxResult)
    * 2.1. On TX error or broadcast success: setResult(..., finalize=true, fromTimer=false).
    * 2.2. On TX success (non-broadcast): wait for response.
    *
    * 3. Early fail in sendRequest()
    * If sendFrame() fails: setResult(ERR_TX_FAILED, finalize=true, fromTimer=false).
    *
    * 4. Timeout callback (timeoutCallback)
    * 4.1. _timerClosing = true (atomic, with RAII closer),
    * 4.2. if isTimerCbDisarmed() -> return (RAII clears _timerClosing),
    * 4.3. lock _mutex; if !_active -> return (RAII clears _timerClosing),
    * 4.4. abort transport (abortCurrentTransaction()),
    * 4.5. setResult(ERR_TIMEOUT, true, fromTimer=true) -> resetUnsafe() clears _active.
    *
    * 5. Synchronous wait
    * The EventGroup handle is registered inside set(...) (or just before arming) to avoid a small 
    * window where completion arrives before the waiter is installed.
    *
    * Invariants & Guarantees
    * ───────────────────────
    * - Single in-flight: at most one active request (_active==true) at a time.
    * - No phantom timeout: a late timeout cannot affect a future request:
    *   - either timer is physically killed (KILLED), or
    *   - callback is logically disarmed (STOPPED_NOACK), or
    *   - timer is the sole finalizer (FAILED).
    * - No double-finalization of the same request (guarded by _mutex).
    * - No new request during closure (gated by _respClosing/_timerClosing).
    * - Callbacks invoked outside critical sections.
    * - No epochs, no timer pools; a single timer handle is reused safely.
    *
    * Corner Cases (exhaustive)
    * ─────────────────────────
    * 1. Callback fires while normal path is killing timer
    * - If fence ACK arrives (KILLED): callback either ran before (then _active=false, normal path 
    *   no-ops) or cannot run after; safe.
    * - If no ACK (STOPPED_NOACK): we disarmCallback() before finalizing; any late callback 
    *   becomes a no-op.
    *
    * 2. Callback already past the disarm check and blocked on _mutex
    * - Atomic gates prevent arming N+1 while N is closing.
    * - When the blocker releases the mutex:
    *   - if normal path already finalized N => _active=false -> callback exits,
    *   - otherwise timer finalizes N and clears both gates; only then can N+1 start.
    *
    * 3. STOP cannot be posted (FAILED)
    * - Normal path must not finalize; returns early (RAII clears _respClosing).
    * - Timer will finalize (single-authority); gates cleared by respective RAII closers.
    *
    * 4. Broadcast completion
    * Treated as normal-path completion -> same centralization and defenses apply.
    *
    * 5. Synchronous path immediate completion
    * EventGroup is set before arming; notifySyncWaiterUnsafe() signals atomically under _mutex.
    * 
    * Alternative Approaches Considered
    * ─────────────────────────────────
    * 1. Timer pool with epochs → More memory, complex lifecycle
    * 2. Dedicated timer task → Extra task overhead, worse latency  
    * 3. Delegate timeout management to interface → architectural mismatch, logic duplication
    * Current solution balances correctness, performance, and maintainability.
    */
    class PendingRequest {
    private:
        // Pending Request state variables
        Client* _client;                              // Pointer to parent Client for transport access in timeout callback
        Modbus::FrameMeta _reqMetadata;               // Lightweight metadata without data payload (~240 bytes saved)
        Modbus::Frame* _pResponse = nullptr;          // Pointer to response buffer (using sync or async w/ tracker)
        Result* _tracker = nullptr;                   // Pointer to user tracker (using async w/ tracker)
        ResponseCallback _cb = nullptr;               // Pointer to user callback (using async w/ callback)
        void* _cbCtx = nullptr;                       // Pointer to user context (using async w/ callback)
        uint32_t _timestampMs = 0;                    // Timestamp of request creation
        volatile bool _active = false;                // Whether the request is active
        Mutex _mutex;                                 // Mutex to protect the pending request data
        EventGroupHandle_t _waiterEventGroup = nullptr; // Event group handle for synchronous wait (sync mode)

        // Request timeout cleanup timer management
        StaticTimer_t _timerBuf;                      // Static timer buffer
        TimerHandle_t _timer = nullptr;               // FreeRTOS timer handle
        BinarySemaphore _timerKillSem;                // Signals timer flush completion
        std::atomic<uint32_t> _timerCbDisarmed;       // Flag to indicate whether the timer callback is disarmed

        // Atomic gates for closing race protection
        std::atomic<uint32_t> _respClosing;           // Indicates whether the request is being closed via the "normal" path
        std::atomic<uint32_t> _timerClosing;          // Indicates whether the request is being closed via the "timeout callback" path


    public:
        // Constructor
        PendingRequest(Client* client) : _client(client), _timerCbDisarmed(0), _respClosing(0), _timerClosing(0) {}
        
        // Helper methods
        bool set(const Modbus::Frame& request, Modbus::Frame* response,
                 Result* tracker, uint32_t timeoutMs, EventGroupHandle_t waiterEventGroup = nullptr);
        bool set(const Modbus::Frame& request, ResponseCallback cb, 
                 void* userCtx, uint32_t timeoutMs);
        void clear();
        // Lock-free methods
        bool isActive() const;
        bool hasResponse() const;
        uint32_t getTimestampMs() const;
        const Modbus::FrameMeta& getRequestMetadata() const;
        // Locked methods
        void setResult(Result result, bool finalize = true, bool fromTimer = false);
        void setResultFromTimer(Result result); // For use from timer context (same, with safe args)
        void setResponse(const Modbus::Frame& response, bool finalize = true);

        // Snapshot helper for lock-free validation in handleResponse
        struct PendingSnapshot {
            Modbus::FrameMeta reqMetadata;
        };
        bool snapshotIfActive(PendingSnapshot& out);

        // Timer neutralization methods (used by Client)
        enum class KillOutcome { KILLED, FAILED, STOPPED_NOACK };
        KillOutcome killTimer(TickType_t maxWaitTicks);
        inline void disarmTimerCb()   { _timerCbDisarmed.store(1, std::memory_order_release); }
        inline void rearmTimerCb()    { _timerCbDisarmed.store(0, std::memory_order_release); }
        inline bool isTimerCbDisarmed() const { return _timerCbDisarmed.load(std::memory_order_acquire); }

        // Atomic gates for closing race protection
        inline void timerClosing(bool wip) { _timerClosing.store(wip ? 1 : 0, std::memory_order_release); }
        inline void respClosing(bool wip) { _respClosing.store(wip ? 1 : 0, std::memory_order_release); }
        inline bool closingInProgress() const { return _respClosing.load(std::memory_order_acquire) 
                                                       || _timerClosing.load(std::memory_order_acquire); }

        // Destructor
        ~PendingRequest();

    private:
        // Unsafe methods (to be called under mutex)
        inline void notifySyncWaiterUnsafe();
        inline void resetUnsafe();
        // Timeout callback & helper
        static void timeoutCallback(TimerHandle_t timer);
    };

    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    ModbusInterface::IInterface& _interface;
    uint32_t _requestTimeoutMs;
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

        // Convert T to uint16_t for coils (0 or 1) - no cast, just check != 0
        uint16_t coilBuf[qty];
        for (uint16_t i = 0; i < qty; i++) {
            coilBuf[i] = src[i] != 0 ? 1 : 0;
        }
        if (!_helperBuffer.setCoils(coilBuf, qty)) {
            return Error(ERR_INVALID_FRAME, "failed to pack coils");
        }
    } else { // HOLDING_REGISTER
        if (qty == 1) {
            _helperBuffer.fc = Modbus::WRITE_REGISTER;
        } else {
            _helperBuffer.fc = Modbus::WRITE_MULTIPLE_REGISTERS;
        }

        // Convert T to uint16_t for registers with clamping
        uint16_t regBuf[qty];
        for (uint16_t i = 0; i < qty; i++) {
            T val = src[i];

            // Clamp negative signed numbers to 0
            if constexpr (std::is_signed<T>::value) {
                if (val < 0) {
                    regBuf[i] = 0;
                    continue;
                }
            }

            // Clamp to UINT16_MAX for > 16-bit types
            if constexpr (std::numeric_limits<T>::max() > std::numeric_limits<uint16_t>::max()) {
                if (val > std::numeric_limits<uint16_t>::max()) {
                    regBuf[i] = std::numeric_limits<uint16_t>::max();
                    continue;
                }
            }

            regBuf[i] = static_cast<uint16_t>(val);
        }
        if (!_helperBuffer.setRegisters(regBuf, qty)) {
            return Error(ERR_INVALID_FRAME, "failed to set registers");
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