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
    
    /* @brief Encapsulates the management of a pending request lifecycle with a thread-safe API
    *
    * @note PendingRequest can only be set() if inactive (clear() must be called first).
    *
    * Timer Race Protection – Current Design:
    *   FreeRTOS xTimerStop() is asynchronous: the STOP command is enqueued and executed
    *   later by the Timer Service Task. Without precautions, a timeout “in flight” could
    *   fire while we are handling a response, or just after cleanup, and accidentally
    *   close a new request.
    *
    *   To avoid that, the timer is explicitly neutralized BEFORE finalizing a request,
    *   using a bounded sequence that never blocks the daemon:
    *
    *     killTimer(maxWaitTicks):
    *       1) xTimerStop(timer, ≤maxWaitTicks) – posts the stop command into the daemon queue.
    *       2) Fence via xTimerPendFunctionCall(...) – when the Timer Service Task processes
    *          the fence, it ACKs us (semaphore) ⇒ guarantees that all pending commands have
    *          been flushed and no further timeout callback can arrive.
    *       3) Fallback if the fence cannot be posted: poll xTimerIsTimerActive() for
    *          ≤maxWaitTicks to detect that the timer has no further expiry scheduled
    *          (STOP processed OR callback already executed). Bounded wait with vTaskDelay(1).
    *
    *   If killTimer() fails (returns false), the function exits EARLY WITHOUT finalizing
    *   the request: the Timer Service Task will then complete it by timeout. This is rare
    *   and only results in a transient ERR_BUSY, without leaving inconsistent state.
    *
    * Integration points:
    *   - handleResponse() and staticHandleTxResult() always call killTimer() before any
    *     cleanup (setResponse / setResult). On failure, they return early without finalizing.
    *   - timeoutCallback() calls setResult(ERR_TIMEOUT,finalize=true,fromTimer=true)
    *     and does not perform heavy synchronization: it is the natural “timer wins” path.
    *   - clear() calls killTimer() first, then resets state under the mutex.
    *
    * FreeRTOS API notes:
    *   - On creation: xTimerStart() is called once.
    *   - To (re)arm an existing timer in set(): xTimerChangePeriod(...). If the timer was
    *     dormant, the call updates expiry and (re)starts it; if it was active, it is
    *     re-evaluated and restarted with the new period.
    *   - killTimer() must never be called from within the Timer Service Task context.
    *
    * Invariants:
    *   - Only one request active at a time (_active).
    *   - No epoch, no multiple timers: a single timer handle, neutralized properly at
    *     termination points.
    *   - All request state modifications are protected by _mutex.
    *   - User callbacks are always invoked outside of critical sections.
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

    public:
        // Constructor
        PendingRequest(Client* client) : _client(client) {}
        
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

        // Timer neutralization method (used by Client)
        bool killTimer(TickType_t maxWaitTicks);

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