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
        ERR_EXCEPTION_RESPONSE,
        ERR_NOT_INITIALIZED,
        ERR_INIT_FAILED
    };
    static constexpr const char* toString(const Result result) {
        switch (result) {
            case SUCCESS: return "Success";
            case NODATA: return "Waiting for response...";
            case ERR_INVALID_FRAME: return "Invalid frame";
            case ERR_BUSY: return "Busy";
            case ERR_TX_FAILED: return "TX failed";
            case ERR_TIMEOUT: return "Timeout";
            case ERR_INVALID_RESPONSE: return "Invalid response";
            case ERR_EXCEPTION_RESPONSE: return "Modbus exception received";
            case ERR_NOT_INITIALIZED: return "Client not initialized";
            case ERR_INIT_FAILED: return "Init failed";
            default: return "Unknown result";
        }
    }

    // Include Error() and Success() definitions
    // (helpers to cast a Result)
    #include "core/ModbusResultHelpers.inl"

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
    // HELPER METHODS FOR COMMON OPERATIONS
    // ===================================================================================
    
    /**
     * @brief Application-level helper methods for common Modbus operations
     * 
     * These methods provide a higher-level API compared to sendRequest():
     * - sendRequest(): Transport-level API - returns SUCCESS if valid Modbus frame received
     *   (caller must check response.exceptionCode separately)
     * - read()/write(): Application-level API - returns SUCCESS only if operation succeeded
     *   (returns ERR_EXCEPTION_RESPONSE if Modbus exception received)
     * 
     * @param slaveId Target slave ID
     * @param regType Register type (COIL, DISCRETE_INPUT, HOLDING_REGISTER, INPUT_REGISTER)
     * @param startAddr Starting register/coil address
     * @param qty Number of registers/coils to read/write
     * @param toBuf/fromBuf Buffer for data (uint16_t* for registers, bool* for coils)
     * @param rspExcep Optional pointer to store exception code if ERR_EXCEPTION_RESPONSE returned
     * @return SUCCESS if operation completed successfully,
     *         ERR_EXCEPTION_RESPONSE if Modbus exception received,
     *         other error codes for transport/validation failures
     */

    // Read/write helpers with uint16_t buffer (all register types)
    Result read(uint8_t slaveId, RegisterType regType, uint16_t startAddr, 
                uint16_t qty, uint16_t* toBuf, ExceptionCode* rspExcep = nullptr);
    Result write(uint8_t slaveId, RegisterType regType, uint16_t startAddr, 
                 uint16_t qty, const uint16_t* fromBuf, ExceptionCode* rspExcep = nullptr);

    // Read helpers with bool buffer (COIL/DISCRETE_INPUT only)  
    Result read(uint8_t slaveId, RegisterType regType, uint16_t startAddr, 
                uint16_t qty, bool* toBuf, ExceptionCode* rspExcep = nullptr);
    Result write(uint8_t slaveId, RegisterType regType, uint16_t startAddr, 
                 uint16_t qty, const bool* fromBuf, ExceptionCode* rspExcep = nullptr);

private:
    // ===================================================================================
    // PRIVATE DATA STRUCTURES
    // ===================================================================================
    
    /* @brief Encapsulates the management of a pending request lifecycle with a thread-safe API
     * @note PendingRequest can only be set() if inactive (clear() must be called first)
     */
    class PendingRequest {
    private:
        Client* _client;                         // Pointer to parent Client for transport access in timeout callback
        Modbus::FrameMeta _reqMetadata; // Lightweight metadata without data payload (saves 250 bytes)
        Modbus::Frame* _pResponse = nullptr;     // Pointer to response buffer (using sync or async w/ tracker)
        Result* _tracker = nullptr;              // Pointer to user tracker (using async w/ tracker)
        ResponseCallback _cb = nullptr;          // Pointer to user callback (using async w/ callback)
        void* _cbCtx = nullptr;                  // Pointer to user context (using async w/ callback)
        uint32_t _timestampMs = 0;               // Timestamp of request creation
        volatile bool _active = false;           // Whether the request is active
        Mutex _mutex;                            // Mutex to protect the pending request data
        StaticTimer_t _timeoutTimerBuf;          // Timer buffer for request timeout
        TimerHandle_t _timeoutTimer = nullptr;   // Timer handle for request timeout
        EventGroupHandle_t _syncEventGroup = nullptr; // Event group handle for synchronous wait (sync mode)

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
        void setResult(Result result, bool finalize);
        void setResponse(const Modbus::Frame& response, bool finalize);
        void setSyncEventGroup(EventGroupHandle_t group);
        void stopTimer();
        // Usafe methods (to be called in mutex)
        inline void notifySyncWaiterUnsafe();
        inline void resetUnsafe();
        // Timeout callback
        static void timeoutCallback(TimerHandle_t timer);
        // Destructor
        ~PendingRequest();
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

    // Single buffer & mutex for the "helper API" methods
    // This buffer is used for both request and response (safe because sendFrame copies the request)
    Modbus::Frame _helperBuffer;    // Buffer for request AND response helpers
    Mutex _helperMutex;             // Protects _helperBuffer for thread safety
    
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