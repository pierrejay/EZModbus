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
    * The challenges lies in that a FreeRTOS timer is managed asynchronously. Despite what the
    * name of the API methods suggest (xTimerStop, xTimerReset, etc.), timers are controlled by
    * sending commands to the Timer Service Task (daemon) command queue. There is no guarantee
    * that those commands have been executed! We can just be sure that if our command was queued,
    * it may be executed at some point in the future.
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
    * - _respClosing - set by the normal path before attempting to neutralize the timer;
    * - _timerClosing - set by the timeout callback at entry.
    * set() proceeds to start a new request only if !_active && !closingInProgress()
    * (checked once pre-mutex for no-lock fail-fast and again under _mutex to remove TOCTOU).
    *
    * 2. Physical Neutralization (killTimer)
    * Before normal-path finalization, we try to stop the timer and fence-ACK the daemon queue:
    * enum class KillOutcome { KILLED, FAILED, STOPPED_NOACK };
    * - KILLED         - STOP posted and fence ACK'd -> no future timeout.
    * - FAILED         - couldn't post STOP -> do not finalize here; timer will finish.
    * - STOPPED_NOACK  - STOP posted but no fence ACK within budget -> indeterminate.
    * The only way to make sure there won't be surprise invocation of the callback is to "force flush" 
    * the timer daemon command queue by using xTimerPendFunctionCall() with a signaling mechanism
    * (in our case, a binary semaphore). Here, we use bounded waits in all blocking calls, i.e. a
    * 5-tick timeout. In 99.9% of cases, the scheduler should switch smoothly so that not even one
    * full tick elapses up to fully neutralizing the timer (-> 10 to 100µs-ish latency)
    *
    * 3. Logical Disarm (disarmTimerCb / rearmTimerCb)
    * If STOPPED_NOACK, we disarm the timeout callback.
    * The callback checks isTimerCbDisarmed() before any lock/abort and becomes a no-op immediately.
    * This guarantees that a late timeout cannot touch current or future requests.
    *
    * 4. Mutual Exclusion (single _mutex)
    * All state mutations occur under _mutex in setResult(), setResponse(), clear() (via resetUnsafe()),
    * preventing double-finalization of the same request and ensuring atomic signaling of waiters.
    *
    * Centralization (single point of truth)
    * ──────────────────────────────────────
    * Call sites never manage timers directly. They only:
    * - take a read-only snapshot for validation (to avoid long critical sections),
    * - then call one of:
    *   - setResponse(frame, finalize, fromTimer=false)
    *   - setResult(result, finalize, fromTimer=false)
    *
    * These methods internally:
    * 1. set _respClosing=true,
    * 2. run killTimer(),
    *    - on FAILED: clear _respClosing and return (timer will finalize),
    *    - on STOPPED_NOACK: disarmTimerCb() (logical neutralization),
    * 3. finalize under _mutex (write tracker/response, _active=false, signal waiters),
    * 4. clear both gates in resetUnsafe().
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
    * 4.1. _timerClosing = true (atomic),
    * 4.2. if isCallbackDisabled() -> _timerClosing=false; return;,
    * 4.3. lock _mutex; if !_active -> _timerClosing=false; return;,
    * 4.4. abort transport (abortCurrentTransaction()),
    * 4.5. setResult(ERR_TIMEOUT, true, fromTimer=true) -> resetUnsafe() clears gates.
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
    * - Normal path must not finalize; returns early.
    * - Timer will finalize (single-authority); gates are cleared in resetUnsafe().
    *
    * 4. Broadcast completion
    * Treated as normal-path completion -> same centralization and defenses apply.
    *
    * 5. Synchronous path immediate completion
    * EventGroup is set before arming; notifySyncWaiterUnsafe() signals atomically under _mutex.
    * 
    * Alternative Approaches Considered
    * ─────────────────────────────────
    * 1. Timer pools with epochs → More memory, complex lifecycle
    * 2. Dedicated timer task → Extra task overhead, worse latency  
    * 3. Delegate timeout management to interface → architectural mismatch, logic duplication
    * 4. Lock-free only → Cannot solve all race windows reliably
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
        EventGroupHandle_t _syncEventGroup = nullptr; // Event group handle for synchronous wait (sync mode)

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
                 Result* tracker, uint32_t timeoutMs);
        bool set(const Modbus::Frame& request, ResponseCallback cb, 
                 void* userCtx, uint32_t timeoutMs);
        void clear();
        // Lock-free methods
        bool isActive() const;
        bool hasResponse() const;
        uint32_t getTimestampMs() const;
        const Modbus::FrameMeta& getRequestMetadata() const;
        // Locked methods
        void setResult(Result result, bool finalize, bool fromTimer = false);
        void setResponse(const Modbus::Frame& response, bool finalize, bool fromTimer = false);
        void setSyncEventGroup(EventGroupHandle_t group);

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
    StaticEventGroup_t _syncEventGroupBuf; // Event group buffer for synchronous wait (sync mode)
    
    // ===================================================================================
    // PRIVATE METHODS
    // ===================================================================================

    Result handleResponse(const Modbus::Frame& response);

    // ===================================================================================
    // TX RESULT CALLBACK
    // ===================================================================================

    static void staticHandleTxResult(ModbusInterface::IInterface::Result result, void* pClient);

};

} // namespace Modbus