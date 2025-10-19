/**
 * @file ModbusServer.cpp
 * @brief Modbus server implementation
 */

#include "ModbusServer.h"

namespace Modbus {

// ===================================================================================
// PUBLIC METHODS
// ===================================================================================

/* @brief Single interface constructor
 * @param interface Reference to the TCP or RTU interface to bind
 * @param store Reference to the WordStore exposing Modbus registers
 * @param slaveId Server Slave/Unit ID (defaults to 1)
 * @param rejectUndefined Whether to reject requests containing read/writes to undefined registers (defaults to true)
 */
Server::Server(ModbusInterface::IInterface& interface, IWordStore& store,
               uint8_t slaveId, bool rejectUndefined, uint32_t reqMutexTimeoutMs)
    : _serverId(slaveId), _rejectUndefined(rejectUndefined), _reqMutexTimeoutMs(reqMutexTimeoutMs), _wordStore(store) {
    // Always valid, we assume MAX_INTERFACES >= 1 even if defined otherwise
    _interfaces.fill(nullptr);
    _interfaces[0] = &interface;
    _interfaceCount = 1;
    _rcvCbContexts[0] = {this, &interface};
    _isInitialized = false;
}

/* @brief Multi-interface constructor
 * @param interfaces List of pointers to interfaces to bind
 * @param store Reference to the WordStore exposing Modbus registers
 * @param slaveId Server Slave/Unit ID (defaults to 1)
 * @param rejectUndefined Whether to reject requests containing read/writes to undefined registers (defaults to true)
 * @note No more than MAX_INTERFACES should be provided, otherwise begin() will return an error
 */
Server::Server(std::initializer_list<ModbusInterface::IInterface*> interfaces, IWordStore& store,
               uint8_t slaveId, bool rejectUndefined, uint32_t reqMutexTimeoutMs)
    : _serverId(slaveId), _rejectUndefined(rejectUndefined), _reqMutexTimeoutMs(reqMutexTimeoutMs), _wordStore(store) {
    _interfaces.fill(nullptr);
    _interfaceCount = 0;
    for (auto* iface : interfaces) {
        if (_interfaceCount >= MAX_INTERFACES) {
            // Overflow detected - reset everything to signal error in begin()
            _interfaces.fill(nullptr);
            _interfaceCount = 0;
            break;
        }
        _interfaces[_interfaceCount] = iface;
        _rcvCbContexts[_interfaceCount] = {this, iface};
        _interfaceCount++;
    }
    _isInitialized = false;
}

Server::~Server() {
    _isInitialized = false;
}

/* @brief Initialize the server
 * @return Result::SUCCESS if the server was initialized successfully
 */
Server::Result Server::begin() {
    if (_isInitialized) return Success();

    // Validate that we have at least one interface, otherwise it means
    // the ctor aborted filling the _interfaces array because the user
    // provided more interfaces than authorized by MAX_INTERFACES
    if (_interfaceCount == 0 || !_interfaces[0]) {
        return Error(Server::ERR_INIT_FAILED, "no valid interface (or more than MAX_INTERFACES provided in ctor)");
    }

    // Initialize all interfaces with proper context routing
    for (size_t i = 0; i < _interfaceCount; i++) {
        if (_interfaces[i]->getRole() != Modbus::SERVER) {
            return Error(Server::ERR_INIT_FAILED, "interface must be SERVER");
        }

        if (_interfaces[i]->begin() != ModbusInterface::IInterface::SUCCESS) {
            // No cleanup needed - user can initialize interfaces manually
            // and retry server.begin() if desired
            return Error(Server::ERR_INIT_FAILED, "interface init failed");
        }

        // Build secure callback with interface identification
        auto rcvCb = [](const Modbus::Frame& frame, void* ctx) {
            // Defensive: validate context
            if (!ctx) {
                Modbus::Debug::LOG_MSG("ERROR: null context in receive callback");
                return;
            }

            auto* callbackCtx = static_cast<RcvCallbackCtx*>(ctx);

            // Validate context structure
            if (!callbackCtx->server || !callbackCtx->sourceInterface) {
                Modbus::Debug::LOG_MSG("ERROR: invalid context in receive callback");
                return;
            }

            Modbus::Debug::LOG_MSG("Received request from interface");
            // Route to handleRequest with proper source interface
            callbackCtx->server->handleRequest(frame, *callbackCtx->sourceInterface);
        };

        auto setRcvCbRes = _interfaces[i]->setRcvCallback(rcvCb, &_rcvCbContexts[i]);
        if (setRcvCbRes != ModbusInterface::IInterface::SUCCESS) {
            return Error(Server::ERR_INIT_FAILED, "cannot set receive callback on interface");
        }
    }

    // Sort WordStore for efficient lookups and validate overlaps
    _wordStore.sortAll();
    
    // Validate no overlaps exist after bulk insertion and sorting
    for (auto type : {Modbus::COIL, Modbus::DISCRETE_INPUT, 
                     Modbus::INPUT_REGISTER, Modbus::HOLDING_REGISTER}) {
        if (!validateNoOverlapsInStore(type)) {
            return Error(Server::ERR_WORD_OVERLAP, "overlapping words detected after sorting");
        }
    }

    _isInitialized = true;
    return Success();
}


/* @brief Clear all words from the server
 * @return Result::SUCCESS if the words were cleared successfully
 * @note Atomic operation: all words will be cleared or none!
 */
Server::Result Server::clearAllWords() {
    Lock guard(_serverMutex, 0);  // Try-lock global mutex (timeout=0)
    if (!guard.isLocked()) return Error(Server::ERR_WORD_BUSY);

    // Use WordStore (unified API)
    _wordStore.clearAll();
    
    return Success();
}

/* @brief Check if the server is busy
 * @return true if the server is busy (adding a word or processing a request)
 */
bool Server::isBusy() {
    if (!_isInitialized) return true;
    Lock guard(_serverMutex, 0);
    return !guard.isLocked();
}


// ===================================================================================
// PRIVATE METHODS
// ===================================================================================

/* @brief Process a Modbus request
 * @param request The request to process
 * @return The result of the request
 */
Server::Result Server::handleRequest(const Modbus::Frame& request,
                                     ModbusInterface::IInterface& sourceInterface) {
    if (!_isInitialized) {
        return Error(Server::ERR_NOT_INITIALIZED, "server not initialized");
    }

    Modbus::Frame response;
    bool shouldReply = false;
    Server::Result finalRes = SUCCESS;
    const bool isBroadcast = Modbus::isBroadcastId(request.slaveId);

    { // Scope for mutex protection of WordStore access
        TickType_t timeout = (_reqMutexTimeoutMs == UINT32_MAX)
                             ? portMAX_DELAY 
                             : pdMS_TO_TICKS(_reqMutexTimeoutMs);
        Lock guard(_serverMutex, timeout);

        // Single while(0) loop: check all guard clauses and exit at each step if needed
        // Set finalRes with Error() helper to log the exact location where it failed
        do {
            // Send busy exception if we couldn't lock the mutex
            if (!guard.isLocked()) {
                finalRes = Error(Server::ERR_RCV_BUSY, "server busy");
                if (!isBroadcast) {
                    Modbus::makeException(request, response, Modbus::SLAVE_DEVICE_BUSY);
                    shouldReply = true;
                }
                break;
            }

            // Ignore requests not addressed to us
            const bool catchAll = Modbus::isBroadcastId(_serverId) || sourceInterface.checkCatchAllSlaveIds();
            if (!catchAll && (request.slaveId != _serverId) && !isBroadcast) {
                finalRes = Error(Server::ERR_RCV_WRONG_SLAVE_ID);
                break;
            }

            // Ignore unsolicited responses from clients
            if (request.type != Modbus::REQUEST) {
                finalRes = Error(Server::ERR_RCV_INVALID_TYPE);
                break;
            }

            // Ignore broadcast read requests (do not send an exception!)
            const bool isWriteReq = Server::isWrite(request.fc);
            if (isBroadcast && !isWriteReq) {
                finalRes = Error(Server::ERR_RCV_ILLEGAL_FUNCTION, "read on broadcast");
                break;
            }

            // Send illegal function exception upon invalid FC
            if (!ModbusCodec::isValidFunctionCode(static_cast<uint8_t>(request.fc))) {
                finalRes = Error(Server::ERR_RCV_ILLEGAL_FUNCTION, "invalid function code");
                if (!isBroadcast) {
                    Modbus::makeException(request, response, Modbus::ILLEGAL_FUNCTION);
                    shouldReply = true;
                }
                break;
            }

            // If we reach here, the request is valid and ready to be processed
            response.clear();
            response.type           = Modbus::RESPONSE;
            response.fc             = request.fc;
            response.slaveId        = request.slaveId;
            response.regAddress     = request.regAddress;
            response.regCount       = request.regCount;
            response.exceptionCode  = Modbus::NULL_EXCEPTION;

            finalRes = isWriteReq ? handleWrite(request, response)
                                  : handleRead(request, response);

            shouldReply = !isBroadcast;

        } while (0);

    } // Mutex released here, now safe to reply

    // Single send point, outside critical section
    if (shouldReply) {
        Server::Result txRes = sendResponse(response, sourceInterface);
        // Prioritize transmission error if it exists
        if (txRes != SUCCESS) {
            finalRes = txRes; // sendResponse already logged the failure
        }
    }

    return finalRes;
}

/* @brief Handle a read request with streaming approach (NO dynamic allocation)
 * @param request The request to handle
 * @param response The response to send
 * @return The result of the request
 */
Server::Result Server::handleRead(const Modbus::Frame& request, Modbus::Frame& response) {
    response.clearData(false);

    // Step 1: Basic request validation
    if (request.regCount > 0 && request.regAddress > (Modbus::MAX_REG_ADDR - request.regCount + 1)) {
        response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
        return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "final address exceeds valid range");
    }

    Modbus::RegisterType regType = Modbus::toRegisterType(request.fc);

    uint32_t startAddr = request.regAddress;
    uint32_t endAddr = request.regAddress + request.regCount;
    
    // Step 2: Full validation scan - NO STORAGE, just validation of ALL Words
    uint32_t currentAddr = startAddr;
    while (currentAddr < endAddr) {
        Word* wordEntry = _wordStore.findExact(regType, currentAddr);
        
        if (!wordEntry) {
            if (_rejectUndefined) {
                response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
                return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "no word found at address");
            } else {
                // Handle gap: find next Word and skip gap
                Word* nextWord = _wordStore.findNext(regType, currentAddr);
                uint32_t gapEnd = (nextWord && nextWord->startAddr < endAddr) ? 
                                   nextWord->startAddr : endAddr;
                currentAddr = gapEnd;
                continue;
            }
        }
        
        // Validate complete Word access
        if ((currentAddr + wordEntry->nbRegs) > endAddr) {
            response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
            return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "Partial Word access not allowed");
        }
        
        // Validate handler exists (direct access - simplified architecture)
        if (!wordEntry->readHandler && (!wordEntry->value || wordEntry->nbRegs != 1)) {
            response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
            return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "Word has no read handler");
        }
        
        // Advance to next address after this Word
        currentAddr += wordEntry->nbRegs;
    }
    
    // Step 3: Stream process each address range directly to response buffer
    currentAddr = startAddr;
    while (currentAddr < endAddr) {
        Word* wordEntry = _wordStore.findExact(regType, currentAddr);
        uint32_t responseIndex = currentAddr - startAddr;  // Position in response buffer
        
        if (!wordEntry) {
            // Step 4a: Handle gaps (when rejectUndefined=false, validation already passed)
            // Find next Word and fill gap with 0s
            Word* nextWord = _wordStore.findNext(regType, currentAddr);
            uint32_t gapEnd = (nextWord && nextWord->startAddr < endAddr) ? 
                               nextWord->startAddr : endAddr;
            
            // Fill gap addresses with 0s
            while (currentAddr < gapEnd) {
                uint32_t gapResponseIndex = currentAddr - startAddr;
                
                if (request.fc == Modbus::READ_HOLDING_REGISTERS || request.fc == Modbus::READ_INPUT_REGISTERS) {
                    // Gap in register: write 0 register directly to response
                    uint16_t gapValue = 0;
                    if (!response.setRegisters(&gapValue, 1, gapResponseIndex)) {
                        response.exceptionCode = Modbus::SLAVE_DEVICE_FAILURE;
                        return Error(Server::ERR_RCV_SLAVE_DEVICE_FAILURE, "Failed to write gap register to response");
                    }
                }
                else if (request.fc == Modbus::READ_COILS || request.fc == Modbus::READ_DISCRETE_INPUTS) {
                    // Gap in coil/discrete input: write 0 bit directly to response
                    bool gapValue = false;
                    if (!response.setCoils(&gapValue, 1, gapResponseIndex)) {
                        response.exceptionCode = Modbus::SLAVE_DEVICE_FAILURE;
                        return Error(Server::ERR_RCV_SLAVE_DEVICE_FAILURE, "Failed to write gap coil to response");
                    }
                }
                currentAddr++;
            }
            continue;
        }
        
        // Step 4b: Execute Word handler and stream result directly to response (direct access)
        memset(_wordBuffer, 0, sizeof(_wordBuffer));
        
        // Execute read operation - either via handler or direct value access
        Modbus::ExceptionCode handlerResult = Modbus::NULL_EXCEPTION;
        if (wordEntry->readHandler) {
            // Use handler for multi-register Words or handler-based single Words
            handlerResult = wordEntry->readHandler(*wordEntry, _wordBuffer, wordEntry->userCtx);
        } else if (wordEntry->value && wordEntry->nbRegs == 1) {
            // Direct access for single-register Words with value pointer
            _wordBuffer[0] = *(wordEntry->value);
        } else {
            // Should not reach here due to validation phase
            handlerResult = Modbus::SLAVE_DEVICE_FAILURE;
        }
        
        // Step 5: Check handler result and abort on error
        if (handlerResult != Modbus::NULL_EXCEPTION) {
            response.clearData(false);  // Clear partial response
            response.exceptionCode = handlerResult;
            return Error(Server::ERR_RCV_SLAVE_DEVICE_FAILURE, "Word read handler failed");
        }
        
        // Step 6: Stream handler result directly to response buffer at correct position
        if (request.fc == Modbus::READ_HOLDING_REGISTERS || request.fc == Modbus::READ_INPUT_REGISTERS) {
            // Write registers directly to response at calculated position
            if (!response.setRegisters(_wordBuffer, wordEntry->nbRegs, responseIndex)) {
                response.clearData(false);
                response.exceptionCode = Modbus::SLAVE_DEVICE_FAILURE;
                return Error(Server::ERR_RCV_SLAVE_DEVICE_FAILURE, "Failed to write registers to response");
            }
        }
        else if (request.fc == Modbus::READ_COILS || request.fc == Modbus::READ_DISCRETE_INPUTS) {
            // Convert register values to bool array and write coils directly to response
            for (uint16_t i = 0; i < wordEntry->nbRegs; ++i) {
                bool coilValue = (_wordBuffer[i] != 0);
                if (!response.setCoils(&coilValue, 1, responseIndex + i)) {
                    response.clearData(false);
                    response.exceptionCode = Modbus::SLAVE_DEVICE_FAILURE;
                    return Error(Server::ERR_RCV_SLAVE_DEVICE_FAILURE, "Failed to write coil to response");
                }
            }
        }
        
        // Advance to next address after this Word
        currentAddr += wordEntry->nbRegs;
    }

    // Step 7: Set final response metadata and return success
    response.regCount = request.regCount;
    return Success();
}


/* @brief Handle a write request with streaming approach (NO dynamic allocation)
 * @param request The request to handle
 * @param response The response to send
 * @return The result of the request
 */
Server::Result Server::handleWrite(const Modbus::Frame& request, Modbus::Frame& response) {
    // Basic request validation
    if (request.regCount == 0) {
        response.exceptionCode = Modbus::ILLEGAL_DATA_VALUE;
        return Error(Server::ERR_RCV_ILLEGAL_DATA_VALUE, "Invalid data size");
    }

    uint16_t writeCount = request.regCount ? request.regCount : 1;

    // Check address range validity
    if (writeCount > 0 && request.regAddress > (Modbus::MAX_REG_ADDR - writeCount + 1)) {
        response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
        return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "Final address exceeds valid range");
    }

    // Check if register type is read-only
    if (isReadOnly(Modbus::toRegisterType(request.fc))) {
        response.exceptionCode = Modbus::ILLEGAL_FUNCTION;
        return Error(Server::ERR_RCV_ILLEGAL_FUNCTION, "Cannot write to read-only register type");
    }

    Modbus::RegisterType regType = Modbus::toRegisterType(request.fc);
    
    // Validation scan: iterate through all addresses to ensure complete validity
    // This scan does NOT store any operations, just validates the entire request
    uint32_t scanAddr = request.regAddress;
    uint32_t endAddr = request.regAddress + writeCount;
    
    while (scanAddr < endAddr) {
        Word* wordEntry = _wordStore.findExact(regType, scanAddr);
        
        if (!wordEntry) {
            if (_rejectUndefined) {
                response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
                return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "No Word found at address during validation");
            } else {
                // Handle gap: find next Word and skip gap
                Word* nextWord = _wordStore.findNext(regType, scanAddr);
                uint32_t gapEnd = (nextWord && nextWord->startAddr < endAddr) ? 
                                   nextWord->startAddr : endAddr;
                scanAddr = gapEnd;
                continue;
            }
        }
        
        // Validate complete Word access
        if ((scanAddr + wordEntry->nbRegs) > endAddr) {
            response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
            return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "Partial Word access not allowed during validation");
        }
        
        // Validate that Word has required handlers for write operation (direct access)
        if (!wordEntry->writeHandler && (!wordEntry->value || wordEntry->nbRegs != 1 || isReadOnly(wordEntry->type))) {
            response.exceptionCode = Modbus::ILLEGAL_DATA_ADDRESS;
            return Error(Server::ERR_RCV_ILLEGAL_DATA_ADDRESS, "Word has no write handler during validation");
        }
        
        // Move to next Word in validation scan
        scanAddr += wordEntry->nbRegs;
    }
    
    // Streaming execution: iterate through addresses again and process each Word
    // This time we execute the writes immediately without storing operations
    uint32_t currentAddr = request.regAddress;
    Modbus::ExceptionCode firstException = Modbus::NULL_EXCEPTION;
    
    while (currentAddr < endAddr) {
        Word* wordEntry = _wordStore.findExact(regType, currentAddr);
        
        if (!wordEntry) {
            if (!_rejectUndefined) {
                // Handle gap: find next Word and skip gap (same logic as validation)
                Word* nextWord = _wordStore.findNext(regType, currentAddr);
                uint32_t gapEnd = (nextWord && nextWord->startAddr < endAddr) ? 
                                   nextWord->startAddr : endAddr;
                currentAddr = gapEnd;
                continue;
            }
            // Should not reach here due to validation phase, but handle gracefully
            break;
        }
        
        // Calculate request offset for this Word
        uint32_t requestOffset = currentAddr - request.regAddress;
        
        // Prepare write data buffer for this specific Word
        memset(_wordBuffer, 0, sizeof(_wordBuffer));
        
        // Extract write data based on function code (NO dynamic allocation)
        if (request.fc == Modbus::WRITE_REGISTER || request.fc == Modbus::WRITE_MULTIPLE_REGISTERS) {
            // Direct register write: copy from request data
            for (uint16_t i = 0; i < wordEntry->nbRegs && (requestOffset + i) < Modbus::FRAME_DATASIZE; ++i) {
                _wordBuffer[i] = request.data[requestOffset + i];
            }
        }
        else if (request.fc == Modbus::WRITE_COIL || request.fc == Modbus::WRITE_MULTIPLE_COILS) {
            // Coil write: extract bits directly from request data (NO vector allocation)
            for (uint16_t i = 0; i < wordEntry->nbRegs && (requestOffset + i) < request.regCount; ++i) {
                // Extract coil bit directly from packed data using getCoil()
                bool coilValue = request.getCoil(requestOffset + i);
                _wordBuffer[i] = coilValue ? 1 : 0;
            }
        }
        
        // Execute write operation immediately - either via handler or direct value access
        Modbus::ExceptionCode writeResult = Modbus::NULL_EXCEPTION;
        if (wordEntry->writeHandler) {
            // Use handler for multi-register Words or handler-based single Words
            writeResult = wordEntry->writeHandler(_wordBuffer, *wordEntry, wordEntry->userCtx);
        } else if (wordEntry->value && wordEntry->nbRegs == 1 && !isReadOnly(wordEntry->type)) {
            // Direct access for single-register Words with value pointer
            *(wordEntry->value) = _wordBuffer[0];
        } else {
            // Should not reach here due to validation phase
            writeResult = Modbus::SLAVE_DEVICE_FAILURE;
        }
        
        // Record first exception but continue processing (Modbus behavior)
        if (writeResult != Modbus::NULL_EXCEPTION && firstException == Modbus::NULL_EXCEPTION) {
            firstException = writeResult;
        }
        
        // Move to next Word in execution stream
        currentAddr += wordEntry->nbRegs;
    }
    
    // Report the first exception encountered during execution phase
    if (firstException != Modbus::NULL_EXCEPTION) {
        response.exceptionCode = firstException;
        return Error(Server::ERR_RCV_SLAVE_DEVICE_FAILURE, "One or more Word handlers failed during streaming execution");
    }

    // Generate appropriate write response
    if (request.fc == Modbus::WRITE_REGISTER || request.fc == Modbus::WRITE_COIL) {
        // Echo data for single writes
        memcpy(response.data.data(), request.data.data(), writeCount * sizeof(uint16_t));
    } else {
        // For multiple writes, return only the address and count
        response.regAddress = request.regAddress;
        response.regCount = writeCount;
    }
    
    return Success();
}

Server::Result Server::sendResponse(const Modbus::Frame& response, ModbusInterface::IInterface& targetInterface) {
    auto sendResult = targetInterface.sendFrame(response, nullptr, nullptr);
    if (sendResult != ModbusInterface::IInterface::SUCCESS) {
        return Error(Server::ERR_RSP_TX_FAILED);
    }
    return Success();
}


/* @brief Check if a function code is a write request
 * @param fc The function code to check
 * @return true if the function code is a write request, false otherwise
 */
bool Server::isWrite(const Modbus::FunctionCode fc) {
    return fc == Modbus::WRITE_REGISTER ||
            fc == Modbus::WRITE_COIL ||
            fc == Modbus::WRITE_MULTIPLE_REGISTERS ||
            fc == Modbus::WRITE_MULTIPLE_COILS;
}

/* @brief Check if a register type is read-only
 * @param type The register type to check
 * @return true if the register type is read-only, false otherwise
 */
bool Server::isReadOnly(const Modbus::RegisterType type) {
    return type == Modbus::DISCRETE_INPUT ||
            type == Modbus::INPUT_REGISTER;
}

// ===================================================================================
// WORD MANAGEMENT METHODS
// ===================================================================================

/* @brief Add a word to the server
 * @param word The word to add
 * @return Result::SUCCESS if the word was added successfully
 * @note Will reject if any overlapping word already exists
 */
Server::Result Server::addWord(const Word& word) {
    Lock guard(_serverMutex, 0);  // Try-lock global mutex
    if (!guard.isLocked()) return Error(Server::ERR_WORD_BUSY);
    
    Server::Result res = isValidWordEntry(word);
    if (res != Server::SUCCESS) {
        Modbus::Debug::LOG_MSGF("Error (%s) : %s %d-%d", toString(res), Modbus::toString(word.type), 
                                word.startAddr, word.startAddr + word.nbRegs - 1);
        return Error(res, "failed to add word");
    }

    // FAST PATH: Skip overlap checks for bulk insertion - will be validated in begin()
    // Only check overlaps if server is already initialized (runtime additions)
    if (_isInitialized && wordOverlaps(word)) {
        Modbus::Debug::LOG_MSGF("Error (%s) : %s %d-%d", toString(ERR_WORD_OVERLAP), Modbus::toString(word.type), 
                                word.startAddr, word.startAddr + word.nbRegs - 1);
        return Error(Server::ERR_WORD_OVERLAP, "word overlaps with existing word");
    }

    return addWordInternal(word);
}

/* @brief Add a set of words to the server
 * @param words The vector containing the words to add
 * @return Result::SUCCESS if the words were added successfully
 * @note Atomic operation: all words will be added or none!
 */
Server::Result Server::addWords(const std::vector<Word>& words) {
    Lock guard(_serverMutex, 0);  // Try-lock global mutex
    if (!guard.isLocked()) return Error(Server::ERR_WORD_BUSY);

    // Quick capacity check (atomicity guarantee)
    if ((_wordStore.totalCapacity() - _wordStore.totalSize()) < words.size()) {
        return Error(Server::ERR_WORD_OVERFLOW, "WordStore capacity exceeded");
    }

    // Check validity of each word (basic validation only)
    for (const auto& word : words) {
        Server::Result res = isValidWordEntry(word);
        if (res != Server::SUCCESS) {
            Modbus::Debug::LOG_MSGF("Error (%s) : %s %d-%d", toString(res), Modbus::toString(word.type), 
                                    word.startAddr, word.startAddr + word.nbRegs - 1);
            return Error(res, "failed to add word");
        }
    }

    // FAST PATH: Skip expensive overlap checks for bulk insertion
    // Overlaps will be validated once in begin() after sorting
    if (_isInitialized) {
        // Only do overlap checks if server is already running (runtime additions)
        for (const auto& word : words) {
            if (wordOverlaps(word)) {
                Modbus::Debug::LOG_MSGF("Error (%s) : %s %d-%d", toString(ERR_WORD_OVERLAP), Modbus::toString(word.type), 
                                        word.startAddr, word.startAddr + word.nbRegs - 1);
                return Error(Server::ERR_WORD_OVERLAP, "word overlaps with existing word");
            }
        }

        // Check for overlaps within the batch itself
        for (size_t i = 0; i < words.size(); ++i) {
            for (size_t j = i + 1; j < words.size(); ++j) {
                const auto& word1 = words[i];
                const auto& word2 = words[j];
                
                // Only check words of the same type
                if (word1.type == word2.type) {
                    uint16_t word1End = word1.startAddr + word1.nbRegs;
                    uint16_t word2End = word2.startAddr + word2.nbRegs;
                    
                    // Check for overlap
                    if ((word1.startAddr < word2End) && (word2.startAddr < word1End)) {
                        Modbus::Debug::LOG_MSGF("Error (%s) : %s %d-%d overlaps with %d-%d", toString(ERR_WORD_OVERLAP), 
                                                Modbus::toString(word1.type), word1.startAddr, word1.startAddr + word1.nbRegs - 1,
                                                word2.startAddr, word2.startAddr + word2.nbRegs - 1);
                        return Error(Server::ERR_WORD_OVERLAP, "words overlap within batch");
                    }
                }
            }
        }
    }

    // Add each word to the store
    for (const auto& word : words) {
        Result res = addWordInternal(word);
        if (res != SUCCESS) {
            return res;  // Return first error encountered
        }
    }
    
    return Success();
}

/* @brief Add a set of words to the server
 * @param words Pointer to the Word buffer
 * @param count Number of words to add
 * @return Result::SUCCESS if the words were added successfully
 * @note Atomic operation: all words will be added or none!
 */
Server::Result Server::addWords(const Word* words, size_t count) {
    if (!words || count == 0) {
        return Error(Server::ERR_WORD_INVALID, "null ptr / zero count");
    }

    Lock guard(_serverMutex, 0);
    if (!guard.isLocked()) return Error(Server::ERR_WORD_BUSY);

    // Capacity check first
    if ((_wordStore.totalCapacity() - _wordStore.totalSize()) < count) {
        return Error(Server::ERR_WORD_OVERFLOW, "WordStore capacity exceeded");
    }

    // 1) Validate each word
    for (size_t i = 0; i < count; ++i) {
        Result res = isValidWordEntry(words[i]);
        if (res != SUCCESS) {
            return Error(res, "failed to add word");
        }
    }

    // 2) Overlap checks if server already running
    if (_isInitialized) {
        for (size_t i = 0; i < count; ++i) {
            if (wordOverlaps(words[i])) {
                return Error(ERR_WORD_OVERLAP, "word overlaps with existing word");
            }
        }

        // intra-batch overlap check (same logic as vector version)
        for (size_t i = 0; i + 1 < count; ++i) {
            for (size_t j = i + 1; j < count; ++j) {
                if (words[i].type == words[j].type) {
                    uint32_t endI = words[i].startAddr + words[i].nbRegs;
                    uint32_t endJ = words[j].startAddr + words[j].nbRegs;
                    if (words[i].startAddr < endJ && words[j].startAddr < endI) {
                        return Error(ERR_WORD_OVERLAP, "words overlap within batch");
                    }
                }
            }
        }
    }

    // 3) Insert each word
    for (size_t i = 0; i < count; ++i) {
        Result res = addWordInternal(words[i]);
        if (res != SUCCESS) return res;
    }

    return Success();
}

/* @brief Get a word from the server
 * @param type The type of word to get
 * @param startAddr The starting address of the word to get
 * @return The copy of the word if found, otherwise an empty word
 */
Word Server::getWord(Modbus::RegisterType type, uint16_t startAddr) {
    auto word = findWordEntry(type, startAddr);
    if (!word) return Word();
    return *word;
}

// ===================================================================================
// WORD HELPER METHODS
// ===================================================================================


/* @brief Find a word entry by type and address
 * @param type The type of word to find
 * @param address The address to search for (must be within the word range)
 * @return The pointer to the word entry
 */
Word* Server::findWordEntry(const Modbus::RegisterType type, const uint16_t address) {
    // Use WordStore (unified API)
    return _wordStore.findContaining(type, address);
}

/* @brief Check if a word exists (exact match)
 * @param word The word to check
 * @return true if the word exists, false otherwise
 */
bool Server::wordExists(const Word& word) {
    // Use WordStore unified API
    auto* foundWord = _wordStore.findExact(word.type, word.startAddr);
    return (foundWord != nullptr && foundWord->nbRegs == word.nbRegs);
}

/* @brief Check if a word overlaps with existing words (optimized O(log n))
 * @param word The word to check
 * @return true if the word overlaps, false otherwise
 * @note Uses binary search on sorted store for O(log n) complexity instead of O(n)
 */
bool Server::wordOverlaps(const Word& word) {
    // Use WordStore unified API
    return _wordStore.overlaps(word);
}

/* @brief Validate that no overlaps exist in a WordStore for a given type
 * @param type The register type to validate
 * @return True if no overlaps found, false if overlaps detected
 */
bool Server::validateNoOverlapsInStore(Modbus::RegisterType type) {
    // Check overlaps in WordStore (words should be sorted after sortAll())
    size_t count = _wordStore.size(type);
    if (count <= 1) return true;  // No overlaps possible with 0 or 1 words
    
    // Get first word to start iteration
    Word* prevWord = nullptr;
    Word* currentWord = _wordStore.findNext(type, 0);  // Find first word
    
    while (currentWord != nullptr) {
        if (prevWord != nullptr) {
            // Check if previous word overlaps with current word
            uint32_t prevEnd = prevWord->startAddr + prevWord->nbRegs;
            if (prevEnd > currentWord->startAddr) {
                Modbus::Debug::LOG_MSGF("Overlap detected: word at %d-%d overlaps with word at %d-%d", 
                                     prevWord->startAddr, prevWord->startAddr + prevWord->nbRegs - 1, 
                                     currentWord->startAddr, currentWord->startAddr + currentWord->nbRegs - 1);
                return false;
            }
        }
        
        prevWord = currentWord;
        currentWord = _wordStore.findNext(type, currentWord->startAddr);
    }
    return true;
}

/* @brief Add a word to the word store
 * @param word The word to add
 * @note Assumes overlap check has already been performed
 */
Server::Result Server::addWordInternal(const Word& word) {
    // Use WordStore (unified API)
    if (!_wordStore.insert(word)) {
        return Error(Server::ERR_WORD_OVERFLOW, "WordStore insert failed - capacity exceeded");
    }
    return Success("Word added to WordStore");
}

/* @brief Validate a word entry
 * @param word The word to validate
 * @return Result::SUCCESS if valid, error code otherwise
 */
Server::Result Server::isValidWordEntry(const Word& word) {
    // Check that the word type is valid (all register types now supported)
    if (!Modbus::isValid(word.type)) {
        return Error(Server::ERR_WORD_INVALID, "invalid register type for Word");
    }

    // Check that the word has at least one register and doesn't exceed maximum size
    if (word.nbRegs == 0) {
        return Error(Server::ERR_WORD_INVALID, "word must have at least 1 register");
    }
    if (word.nbRegs > MAX_WORD_SIZE) {
        return Error(Server::ERR_WORD_INVALID, "word exceeds maximum size");
    }

    // Check address range validity - prevent integer overflow
    if (word.nbRegs > 0 && word.startAddr > (Modbus::MAX_REG_ADDR - word.nbRegs + 1)) {
        return Error(Server::ERR_WORD_INVALID, "word address range exceeds maximum");
    }

    // UNIFIED VALIDATION: Thread safety rule for multi-register Words
    // - nbRegs = 1: Can use direct pointer access OR handlers
    // - nbRegs > 1: Must use handlers only (thread safety requirement)
    
    bool hasDirectPointer = (word.value != nullptr);
    bool hasHandlers = (word.readHandler != nullptr);
    
    // Multi-register Words cannot use direct pointer access (thread safety)
    if (word.nbRegs > 1 && hasDirectPointer) {
        return Error(Server::ERR_WORD_DIRECT_PTR, "multi-register Words cannot use direct pointer access (thread safety)");
    }
    
    // Multi-register Words must use handlers
    if (word.nbRegs > 1 && !hasHandlers) {
        return Error(Server::ERR_WORD_HANDLER, "multi-register Words must use handlers");
    }
    
    // Single-register Words need either direct pointer OR handlers
    if (word.nbRegs == 1 && !hasDirectPointer && !hasHandlers) {
        return Error(Server::ERR_WORD_HANDLER, "single-register Words need direct pointer or handlers");
    }

    // If using handlers, check that read handler is defined
    if (hasHandlers && word.readHandler == nullptr) {
        return Error(Server::ERR_WORD_HANDLER, "Word missing read handler");
    }

    // If using handlers and not read-only, check write handler
    if (hasHandlers && !isReadOnly(word.type) && word.writeHandler == nullptr) {
        return Error(Server::ERR_WORD_HANDLER, "Word missing write handler");
    }

    // Check that read-only types don't have write handlers or direct pointer for write access
    if (isReadOnly(word.type)) {
        if (word.writeHandler != nullptr) {
            return Error(Server::ERR_WORD_HANDLER, "read-only word should not have write handler");
        }
        // Note: Direct pointer for read-only types is allowed (read access only via static callback)
    }

    return Success();
}

} // namespace Modbus