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
        ERR_INIT_FAILED
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
     * @note PendingRequest can only be set() if inactive (clear() must be called first)
     *
     * Timer Race Protection Design:
     * This class uses a double-buffer timer approach with epoch guards to prevent
     * race conditions between timer callbacks and response handling.
     *
     * Problem: xTimerStop() is asynchronous - it queues a command to the Timer
     * Service Task rather than stopping immediately. This creates races where:
     * 1. A late STOP from request N can kill the timer already restarted for N+1
     * 2. A late timeout callback from N can close request N+1 if not filtered
     *
     * Solution:
     * - Multiple timers: several timer handles alternate (_tim[0], _tim[1]...)
     *   so old STOP commands cannot affect the new timer (different handle)
     * - Epoch guards: Each request gets a unique _timGen ID copied into the
     *   timer's ID field. Late timeout callbacks are ignored if epoch mismatches
     * - No blocking stops: No xTimerStop() synchronization needed in response
     *   handling - safety is guaranteed by epochs, not by stopping timers
     * 
     * Key Invariants:
     * - Only one request active at a time (_active flag)
     * - Each transaction identified by unique epoch
     * - No blocking timer synchronization in critical paths
     * - Late callbacks filtered by epoch mismatch and _active state
     */
    class PendingRequest {
    private:
        // Pending Request state variables
        Client* _client;                              // Pointer to parent Client for transport access in timeout callback
        Modbus::Frame _reqMetadata;                   // Does not store request data (only fc, slaveId, regAddress, regCount)
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
        volatile bool _timerActive = false;           // Timer active flag
        volatile uint32_t _timerStartMs = false;      // Start timestamp of last timer
        BinarySemaphore _timerKillSem;                // Signals timer flush completion
        static constexpr uint32_t TIMER_WAIT_MS = 5;  // Max wait time in timer ops path
        static constexpr uint32_t TIMER_EXP_MS = 100; // Grace period after _requestTimeoutMs to safely declare last timer expired (last-resort mechanism)

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
        const Modbus::Frame& getRequestMetadata() const;
        // Locked methods
        void setResult(Result result, bool finalize, bool fromTimer = false);
        void setResponse(const Modbus::Frame& response, bool finalize, bool fromTimer = false);
        void setSyncEventGroup(EventGroupHandle_t group);

        // Snapshot helper for lock-free validation in handleResponse
        struct PendingSnapshot {
            Modbus::Frame reqMetadata;
        };
        bool snapshotIfActive(PendingSnapshot& out);

        // Timer neutralization method (used by Client)
        bool killTimer(TickType_t maxWaitTicks = pdMS_TO_TICKS(TIMER_WAIT_MS));

        // Destructor
        ~PendingRequest();

    private:
        // Unsafe methods (to be called under mutex)
        inline void notifySyncWaiterUnsafe();
        inline void resetUnsafe();
        // Timeout callback & helper
        static void timeoutCallback(TimerHandle_t timer);
        static inline bool inTimerDaemon();
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