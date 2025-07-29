/**
 * @file ModbusServer.h
 * @brief Modbus server class header
 */

#pragma once

#include "core/ModbusCore.h"
#include "core/ModbusWord.hpp"
#include "interfaces/ModbusInterface.hpp"
#include "utils/ModbusDebug.hpp"

#ifndef EZMODBUS_SERVER_MAX_WORD_SIZE // Server max word size (# Modbus registers per Word)
    #define EZMODBUS_SERVER_MAX_WORD_SIZE 8
#endif


namespace Modbus {

class Server {
public:
    // ===================================================================================
    // CONSTANTS
    // ===================================================================================

    static constexpr uint32_t MAX_REGISTERS = 65535;
    static constexpr size_t MAX_WORD_SIZE = (size_t)EZMODBUS_SERVER_MAX_WORD_SIZE; // max no. registers per word

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
            case SUCCESS: return "Success";
            case NODATA: return "No data";
            case ERR_WORD_BUSY: return "Word store busy";
            case ERR_WORD_OVERFLOW: return "Stored too many words";
            case ERR_WORD_INVALID: return "Invalid word";
            case ERR_WORD_DIRECT_PTR: return "Forbidden direct pointer";
            case ERR_WORD_HANDLER: return "Malformed handlers";
            case ERR_WORD_OVERLAP: return "Word overlaps with existing word";
            case ERR_RCV_UNKNOWN_WORD: return "Unknown word";
            case ERR_RCV_BUSY: return "Incoming request while busy";
            case ERR_RCV_INVALID_TYPE: return "Received invalid request";
            case ERR_RCV_WRONG_SLAVE_ID: return "Wrong slave ID in received frame";
            case ERR_RCV_ILLEGAL_FUNCTION: return "Illegal function in received frame";
            case ERR_RCV_ILLEGAL_DATA_ADDRESS: return "Illegal data address in received frame";
            case ERR_RCV_ILLEGAL_DATA_VALUE: return "Illegal data value in received frame";
            case ERR_RCV_SLAVE_DEVICE_FAILURE: return "Slave device failure on received frame";
            case ERR_RSP_TX_FAILED: return "Transmit response failed";
            case ERR_NOT_INITIALIZED: return "Server not initialized";
            case ERR_INIT_FAILED: return "Init failed";
            default: return "Unknown error";
        }
    }

    // Include Error() and Success() definitions
    // (helpers to cast a Result)
    #include "core/ModbusResultHelpers.inl"

    // ===================================================================================
    // CONSTRUCTOR & PUBLIC METHODS
    // ===================================================================================

    // Constructor with WordStore (now mandatory)
    Server(ModbusInterface::IInterface& interface, IWordStore& store, uint8_t slaveId = 1, bool rejectUndefined = true);
    
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
    // PRIVATE MEMBERS
    // ===================================================================================

    // Configuration
    ModbusInterface::IInterface& _interface;
    uint8_t _serverId;
    bool _rejectUndefined; // If false, undefined registers will be silently ignored (no exception returned)
    bool _isInitialized = false;
    Modbus::Frame _responseBuffer;
    
    // WordStore & buffer for word operations
    IWordStore& _wordStore;
    uint16_t _wordBuffer[MAX_WORD_SIZE];
    
    // Global mutex : protects both requests and WordStore modifications
    Mutex _serverMutex;

    // ===================================================================================
    // PRIVATE METHODS
    // ===================================================================================

    // Request handlers
    Result handleRequest(const Modbus::Frame& request);
    Result handleRead(const Modbus::Frame& request, Modbus::Frame& response);
    Result handleWrite(const Modbus::Frame& request, Modbus::Frame& response);
    Result sendResponse(const Modbus::Frame& response);

    // Find helpers
    
    // Word helpers (simplified - no more legacy WordStore methods)
    Word* findWordEntry(const Modbus::RegisterType type, const uint16_t address);
    bool wordExists(const Word& word);
    bool wordOverlaps(const Word& word);
    bool validateNoOverlapsInStore(Modbus::RegisterType type);
    Result addWordInternal(const Word& word);
    Result isValidWordEntry(const Word& word);
    
    static bool isWrite(const Modbus::FunctionCode fc);
    static bool isReadOnly(const Modbus::RegisterType type);
};

} // namespace Modbus 