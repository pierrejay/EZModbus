/**
 * @file ModbusServer.h
 * @brief Modbus server class header
 */

#pragma once

#include <initializer_list>
#include <array>
#include "core/ModbusCore.h"
#include "core/ModbusWord.hpp"
#include "interfaces/ModbusInterface.hpp"
#include "utils/ModbusDebug.hpp"

#ifndef EZMODBUS_SERVER_MAX_WORD_SIZE // Server max word size (# Modbus registers per Word)
    #define EZMODBUS_SERVER_MAX_WORD_SIZE 8
#endif

#ifndef EZMODBUS_SERVER_MAX_INTERFACES // Server max simultaneous interfaces
    #define EZMODBUS_SERVER_MAX_INTERFACES 2
#endif

#ifndef EZMODBUS_SERVER_REQ_MUTEX_TIMEOUT_MS // Server request mutex timeout (ms)
    #define EZMODBUS_SERVER_REQ_MUTEX_TIMEOUT_MS UINT32_MAX // Infinite wait by default
#endif


namespace Modbus {

class Server {
public:
    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    static constexpr uint32_t MAX_REGISTERS = 65535;
    static constexpr size_t MAX_WORD_SIZE = (size_t)EZMODBUS_SERVER_MAX_WORD_SIZE; // max no. registers per word
    static constexpr size_t MAX_INTERFACES = (size_t)EZMODBUS_SERVER_MAX_INTERFACES; // max simultaneous interfaces
    static constexpr uint32_t DEFAULT_REQ_MUTEX_TIMEOUT_MS = (uint32_t)EZMODBUS_SERVER_REQ_MUTEX_TIMEOUT_MS; // default request mutex timeout

    // ===================================================================================
    // RESULT TYPES
    // ===================================================================================

    enum Result {
        SUCCESS,
        NODATA,
        ERR_WORD_BUSY,
        ERR_WORD_OVERFLOW,
        ERR_WORD_INVALID,
        ERR_WORD_DIRECT_PTR,
        ERR_WORD_HANDLER,
        ERR_WORD_OVERLAP,
        ERR_RCV_UNKNOWN_WORD,
        ERR_RCV_BUSY,
        ERR_RCV_INVALID_TYPE,
        ERR_RCV_WRONG_SLAVE_ID,
        ERR_RCV_ILLEGAL_FUNCTION,
        ERR_RCV_ILLEGAL_DATA_ADDRESS,
        ERR_RCV_ILLEGAL_DATA_VALUE,
        ERR_RCV_SLAVE_DEVICE_FAILURE,
        ERR_RSP_TX_FAILED,
        ERR_NOT_INITIALIZED,
        ERR_INIT_FAILED
    };
    static const char* toString(const Result res) {
        switch (res) {
            case SUCCESS: return "success";
            case NODATA: return "no data";
            case ERR_WORD_BUSY: return "busy word store";
            case ERR_WORD_OVERFLOW: return "stored too many words";
            case ERR_WORD_INVALID: return "invalid word";
            case ERR_WORD_DIRECT_PTR: return "forbidden direct pointer";
            case ERR_WORD_HANDLER: return "malformed handlers";
            case ERR_WORD_OVERLAP: return "word overlaps with existing word";
            case ERR_RCV_UNKNOWN_WORD: return "unknown word";
            case ERR_RCV_BUSY: return "incoming request: busy";
            case ERR_RCV_INVALID_TYPE: return "received invalid request";
            case ERR_RCV_WRONG_SLAVE_ID: return "wrong slave ID in rcvd frame";
            case ERR_RCV_ILLEGAL_FUNCTION: return "illegal function in rcvd frame";
            case ERR_RCV_ILLEGAL_DATA_ADDRESS: return "illegal data address in rcvd frame";
            case ERR_RCV_ILLEGAL_DATA_VALUE: return "illegal data value in rcvd frame";
            case ERR_RCV_SLAVE_DEVICE_FAILURE: return "slave device failure on rcvd frame";
            case ERR_RSP_TX_FAILED: return "transmit response failed";
            case ERR_NOT_INITIALIZED: return "server not initialized";
            case ERR_INIT_FAILED: return "init failed";
            default: return "unknown error";
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
    // CONSTRUCTOR & PUBLIC METHODS
    // ===================================================================================

    // Single-interface ctor
    Server(ModbusInterface::IInterface& interface, IWordStore& store,
           uint8_t slaveId = 1, bool rejectUndefined = true,
           uint32_t reqMutexTimeoutMs = DEFAULT_REQ_MUTEX_TIMEOUT_MS);

    // Multi-interface ctor
    Server(std::initializer_list<ModbusInterface::IInterface*> interfaces, IWordStore& store,
           uint8_t slaveId = 1, bool rejectUndefined = true,
           uint32_t reqMutexTimeoutMs = DEFAULT_REQ_MUTEX_TIMEOUT_MS);

    ~Server();

    Result begin();
    
    Result clearAllWords();
    
    // Word management methods
    Result addWord(const Word& word);
    Result addWords(const std::vector<Word>& words);
    Result addWords(const Word* words, size_t count);
    Word getWord(Modbus::RegisterType type, uint16_t startAddr);
    
    bool isBusy();

private:
    // ===================================================================================
    // PRIVATE TYPES
    // ===================================================================================

    // Callback context for interface routing
    struct RcvCallbackCtx {
        Server* server;
        ModbusInterface::IInterface* sourceInterface;
    };

    // ===================================================================================
    // PRIVATE MEMBERS
    // ===================================================================================

    // Interfaces management
    std::array<ModbusInterface::IInterface*, MAX_INTERFACES> _interfaces;
    std::array<RcvCallbackCtx, MAX_INTERFACES> _rcvCbContexts;  // Callback contexts (must stay valid for lifetime)
    size_t _interfaceCount = 0;

    // Server configuration
    uint8_t _serverId;
    bool _rejectUndefined; // If false, undefined registers will be silently ignored (no exception returned)
    bool _isInitialized = false;
    uint32_t _reqMutexTimeoutMs; // Request mutex timeout in milliseconds

    // WordStore & buffer for word operations
    IWordStore& _wordStore;
    uint16_t _wordBuffer[MAX_WORD_SIZE];
    
    // Global mutex : protects both requests and WordStore modifications
    Mutex _serverMutex;

    // ===================================================================================
    // PRIVATE METHODS
    // ===================================================================================

    // Request handlers
    Result handleRequest(const Modbus::Frame& request, ModbusInterface::IInterface& sourceInterface);
    Result handleRead(const Modbus::Frame& request, Modbus::Frame& response);
    Result handleWrite(const Modbus::Frame& request, Modbus::Frame& response);
    Result sendResponse(const Modbus::Frame& response, ModbusInterface::IInterface& targetInterface);

    // Find helpers
    
    // Word helpers (simplified - no more legacy WordStore methods)
    Word* findWordEntry(const Modbus::RegisterType type, const uint16_t address);
    bool wordExists(const Word& word);
    bool wordOverlaps(const Word& word);
    bool validateNoOverlapsInStore(Modbus::RegisterType type);
    Result addWordInternal(const Word& word);
    Result isValidWordEntry(const Word& word);
    
    static inline bool isWrite(const Modbus::FunctionCode fc);
    static inline bool isReadOnly(const Modbus::RegisterType type);
};

} // namespace Modbus 