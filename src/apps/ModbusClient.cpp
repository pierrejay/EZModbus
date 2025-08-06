/**
 * @file ModbusClient.cpp
 * @brief Modbus client implementation
 */

#include "ModbusClient.h"

namespace Modbus {

// ===================================================================================
// DATA STRUCTURES METHODS
// ===================================================================================

/* @brief Timeout callback for pending request
 * @param timer The timer handle
 */
void Client::PendingRequest::timeoutCallback(TimerHandle_t timer) {
    auto* pendingReq = static_cast<PendingRequest*>(pvTimerGetTimerID(timer));
    if (pendingReq && pendingReq->isActive()) {
        // Signal transport to cleanup transaction
        pendingReq->_client->_interface.abortCurrentTransaction();
        
        pendingReq->setResult(ERR_TIMEOUT, true);
        Modbus::Debug::LOG_MSG("Request timed out via timer");
    }
}

/* @brief Set the pending request (response & tracker version)
 * @param request The request to set
 * @param response Where to store the response (response.data will be resized to match its size)
 * @param tracker Pointer to the transfer result tracker
 * @param timeoutMs Timeout in milliseconds
 * @return true if the pending request was set successfully, false is already active (clear it first)
 */
bool Client::PendingRequest::set(const Modbus::Frame& request, Modbus::Frame* response, 
                                 Result* tracker, uint32_t timeoutMs) {
    if (isActive()) return false; // The pending request must be cleared first
    Lock guard(_mutex);
    _reqMetadata = Modbus::FrameMeta(request);
    _pResponse = response;
    _tracker = tracker;
    _cb = nullptr; 
    _cbCtx = nullptr;
    _timestampMs = TIME_MS();
    _syncEventGroup = nullptr; // Start from clean slate
    _active = true;
    
    // Create and start timeout timer
    if (!_timeoutTimer) {
        _timeoutTimer = xTimerCreateStatic(
            "ModbusTimeout",
            pdMS_TO_TICKS(timeoutMs),
            pdFALSE,  // one-shot
            this,     // timer ID = this PendingRequest
            timeoutCallback,
            &_timeoutTimerBuf
        );
    } else {
        // Update timer period and restart
        xTimerChangePeriod(_timeoutTimer, pdMS_TO_TICKS(timeoutMs), 0);
    }
    
    if (_timeoutTimer) {
        xTimerStart(_timeoutTimer, 0);
    }
    
    return true;
}

/* @brief Set the pending request (callback version)
 * @param request  The request to set
 * @param cb       Callback to invoke on completion
 * @param userCtx  User context pointer passed back to callback
 * @param timeoutMs Timeout in milliseconds
 * @return true if the pending request was set successfully, false if already active
 */
bool Client::PendingRequest::set(const Modbus::Frame& request, Client::ResponseCallback cb,
                                 void* userCtx, uint32_t timeoutMs) {
    if (isActive()) return false;
    Lock guard(_mutex);
    _reqMetadata = Modbus::FrameMeta(request);
    _pResponse = nullptr;     // No external response storage in callback mode
    _tracker   = nullptr;     // Not used in callback mode
    _cb        = cb;
    _cbCtx     = userCtx;
    _timestampMs = TIME_MS();
    _active = true;

    // Create / restart timeout timer
    if (!_timeoutTimer) {
        _timeoutTimer = xTimerCreateStatic(
            "ModbusTimeout",
            pdMS_TO_TICKS(timeoutMs),
            pdFALSE,
            this,
            timeoutCallback,
            &_timeoutTimerBuf);
    } else {
        xTimerChangePeriod(_timeoutTimer, pdMS_TO_TICKS(timeoutMs), 0);
    }
    if (_timeoutTimer) {
        xTimerStart(_timeoutTimer, 0);
    }
    return true;
}

/* @brief Clear the pending request
 */
void Client::PendingRequest::clear() {
    Lock guard(_mutex);
    
    // Stop timeout timer
    if (_timeoutTimer) {
        xTimerStop(_timeoutTimer, 0);
    }
    
    _reqMetadata.clear();
    _tracker = nullptr;
    _pResponse = nullptr;
    _timestampMs = 0;
    _syncEventGroup = nullptr; // Clear sync waiter
    _cb = nullptr;
    _cbCtx = nullptr;
    _active = false;
}

/* @brief Check if the pending request is active
 * @return true if the pending request is active
 */
bool Client::PendingRequest::isActive() const { return _active; }

/* @brief Check if the pending request has a response pointer
 * @return true if the pending request has a response pointer
 */
bool Client::PendingRequest::hasResponse() const { return _pResponse != nullptr; }

/* @brief Get the pending request timestamp
 * @return The timestamp of the pending request (creation time)
 */
uint32_t Client::PendingRequest::getTimestampMs() const { return _timestampMs; }

/* @brief Get the pending request
 * @return Copy of the pending request
 */
const Modbus::FrameMeta& Client::PendingRequest::getRequestMetadata() const { return _reqMetadata; }

/* @brief Update the pending request result tracker
 * @param result The result to set
 */
void Client::PendingRequest::setResult(Result result, bool finalize) {
    // Initialize callback variables to null
    Client::ResponseCallback cbSnapshot = nullptr;
    void* ctxSnapshot = nullptr;

    // Handle result & tracker under critical section
    { 
        Lock guard(_mutex);
        if (_tracker) *_tracker = result;
        cbSnapshot = _cb;
        ctxSnapshot = _cbCtx;
        if (finalize) {
            EventBus::pushRequest(_reqMetadata, result, toString(result), _client); // Log request processing result (only if finalized)
            resetUnsafe(); // also sets _active=false
        }
        notifySyncWaiterUnsafe(); // still inside mutex
        if (!cbSnapshot) return; // if no callback registered, exit now
    }

    // Invoke callback outside of critical section
    // Since we don't expect a response (failure or broadcast), we pass nullptr as response
    cbSnapshot(result, nullptr, ctxSnapshot);
}

/* @brief Set the response for the pending request & update the result tracker
 * @param response The response to set
 */
void Client::PendingRequest::setResponse(const Modbus::Frame& response, bool finalize) {
    Client::ResponseCallback cbSnapshot = nullptr;
    void* ctxSnapshot = nullptr;

    // Determine result based on exception code
    Result result = (response.exceptionCode != Modbus::NULL_EXCEPTION) ? ERR_EXCEPTION_RESPONSE : SUCCESS;

    // Handle response & tracker under critical section
    {
        Lock guard(_mutex);
        if (_pResponse) *_pResponse = response;
        if (_tracker) *_tracker = result;
        cbSnapshot = _cb;
        ctxSnapshot = _cbCtx;
        if (finalize) {
            EventBus::pushRequest(_reqMetadata, result, toString(result), this); // Log request processing result (only if finalized)
            resetUnsafe(); // also sets _active=false
        }
        notifySyncWaiterUnsafe(); // still inside mutex
    }

    // Invoke callback outside of critical section
    // (safe: see comments in handleResponse() & staticHandleTxResult())
    if (cbSnapshot) {
        cbSnapshot(result, &response, ctxSnapshot);
    }
}

/* @brief Set the event group for synchronous waiting
 * @param group The event group handle waiting for completion
 */
void Client::PendingRequest::setSyncEventGroup(EventGroupHandle_t group) {
    Lock guard(_mutex);
    _syncEventGroup = group;
}

/* @brief Notify the synchronous waiting task
 * @note This method MUST be called while holding _mutex to ensure atomicity
 */
void Client::PendingRequest::notifySyncWaiterUnsafe() {
    // This method should be called while holding the mutex
    if (_syncEventGroup) {
        xEventGroupSetBits(_syncEventGroup, SYNC_COMPLETION_BIT);
        _syncEventGroup = nullptr;
    }
}

/* @brief Stop the timeout timer
 * @note Called to neutralize timer when response received
 */
void Client::PendingRequest::stopTimer() {
    if (_timeoutTimer) {
        xTimerStop(_timeoutTimer, 0);
    }
}

/* @brief Destructor for PendingRequest - cleanup timer
 */
Client::PendingRequest::~PendingRequest() {
    if (_timeoutTimer) {
        xTimerDelete(_timeoutTimer, 0);
        _timeoutTimer = nullptr;
    }
}


/* @brief Reset the pending request to its initial state
 * @note This method MUST be called while holding _mutex to ensure atomicity
 */
void Client::PendingRequest::resetUnsafe() {
    // Stop timeout timer
    if (_timeoutTimer) {
        xTimerStop(_timeoutTimer, 0);
    }

    _reqMetadata.clear();
    _tracker = nullptr;
    _pResponse = nullptr;
    _timestampMs = 0;
    _cb = nullptr;
    _cbCtx = nullptr;
    _active = false;
}

// ===================================================================================
// PUBLIC METHODS
// ===================================================================================

Client::Client(ModbusInterface::IInterface& interface, uint32_t timeoutMs) : 
    _interface(interface),
    _requestTimeoutMs(timeoutMs),
    _pendingRequest(this),
    _isInitialized(false)
{}

Client::~Client() {
    // Cleanup any active pending request
    _pendingRequest.clear();
    _isInitialized = false;
}


/* @brief Initialize the client
 * @return Success if the client was initialized successfully
 */
Client::Result Client::begin() {
    if (_isInitialized) return Success();

    if (_interface.getRole() != Modbus::CLIENT) {
        return Error(ERR_INIT_FAILED, "interface must be CLIENT");
    }

    if (_interface.begin() != ModbusInterface::IInterface::SUCCESS) {
        return Error(ERR_INIT_FAILED, "interface init failed");
    }

    auto rcvCb = [](const Modbus::Frame& frame, void* ctx) {
        static_cast<Client*>(ctx)->handleResponse(frame);
    };

    auto setRcvCbRes = _interface.setRcvCallback(rcvCb, this);
    if (setRcvCbRes != ModbusInterface::IInterface::SUCCESS) {
        return Error(ERR_INIT_FAILED, "cannot set receive callback on interface");
    }

    _isInitialized = true;
    return Success();
}

/* @brief Check if the client is ready to accept a new request
 * @return true if interface ready & no active pending request
 */
bool Client::isReady() {
    if (!_isInitialized) return false;
    return (_interface.isReady() && !_pendingRequest.isActive());
}

/* @brief Send a request to the interface (synchronous or asynchronous with tracker)
 * @param request The request to send
 * @param response Where to store the response (response.data will be resized to match its size)
 * @param userTracker nullptr for blocking (sync) mode, or pointer to transfer result tracker for async mode
 * @return - In async mode: Success if the transfer was started successfully (check the tracker to follow up)
 * @return - In sync mode: Success if the transfer completed successfully
 */
Client::Result Client::sendRequest(const Modbus::Frame& request, 
                         Modbus::Frame& response,
                         Result* userTracker) {
   // Reject if the frame is not a request, invalid, or another request is already in progress
    if (request.type != Modbus::REQUEST) {
        return Error(ERR_INVALID_FRAME, "non-request frame");
    }
    auto validResult = ModbusCodec::isValidFrame(request);
    if (validResult != ModbusCodec::SUCCESS) {
        return Error(ERR_INVALID_FRAME, "invalid fields");
    }
    if (!isReady()) {
        return Error(ERR_BUSY, "interface busy or active pending request");
    }

    // Choose tracker to use : if userTracker is not provided ("sync mode"), we create a local one
    Result localResult;
    Result* tracker = userTracker ? userTracker : &localResult;

    // Basic checks passed, we try to initiate the pending request
    if (!_pendingRequest.set(request, &response, tracker, _requestTimeoutMs)) {
        return Error(ERR_BUSY, "request already in progress");
    }

    _pendingRequest.setResult(NODATA, false); // Update result tracker immediately

    // Send the frame on the interface using callback
    auto sendRes = _interface.sendFrame(request, staticHandleTxResult, this);
    if (sendRes != ModbusInterface::IInterface::SUCCESS) {
        _pendingRequest.setResult(ERR_TX_FAILED, true);
        return Error(ERR_TX_FAILED);
    }

    // ---------- Synchronous mode (userTracker == nullptr) ----------
    if (!userTracker) {
        // Create an event group for this synchronous transaction
        EventGroupHandle_t syncEvtGrp = xEventGroupCreateStatic(&_syncEventGroupBuf);
        if (!syncEvtGrp) {
            _pendingRequest.setResult(ERR_TIMEOUT, true);
            return Error(ERR_BUSY, "cannot create event group");
        }

        // Register the event group inside the pending request so that the worker can signal completion
        _pendingRequest.setSyncEventGroup(syncEvtGrp);

        // Wait for completion or timeout (response, TX error, or timeout timer)
        EventBits_t bits = xEventGroupWaitBits(
            syncEvtGrp,
            SYNC_COMPLETION_BIT, // bit 0 reserved for completion
            pdTRUE,               // clear on exit
            pdFALSE,              // wait for any bit
            pdMS_TO_TICKS(_requestTimeoutMs + 100) // little margin
        );

        // We're done with the event group
        vEventGroupDelete(syncEvtGrp);

        // Check if we got the expected bit (otherwise timeout)
        if ((bits & SYNC_COMPLETION_BIT) == 0) {
            _pendingRequest.setResult(ERR_TIMEOUT, true);
            return Error(ERR_TIMEOUT, "sync wait timeout");
        }

        // The localResult variable now contains the outcome
        bool ok = (*tracker == SUCCESS);
        if (!ok) {
            // The result tracker indicates the outcome of the request
            // (TX failure or timeout -> detected by the timer or TX task)
            return Error(*tracker);
        }
        return Success();
    }

    // ---------- Asynchronous mode (userTracker != nullptr) ----------
    return Success();
}

/* @brief Send a request (asynchronous with callback)
 * @param request   The request to send
 * @param cb        Callback invoked on completion
 * @param userCtx   User context pointer (may be nullptr)
 * @return Success if the request was queued, or error code
 */
Client::Result Client::sendRequest(const Modbus::Frame& request,
                         Client::ResponseCallback cb,
                         void* userCtx) {
    // Reject invalid cases first
    if (request.type != Modbus::REQUEST) {
        return Error(ERR_INVALID_FRAME, "non-request frame");
    }
    auto validResult = ModbusCodec::isValidFrame(request);
    if (validResult != ModbusCodec::SUCCESS) {
        return Error(ERR_INVALID_FRAME, "invalid fields");
    }
    if (!isReady()) {
        return Error(ERR_BUSY, "interface busy or active pending request");
    }

    // Initiate the pending request (callback mode)
    if (!_pendingRequest.set(request, cb, userCtx, _requestTimeoutMs)) {
        return Error(ERR_BUSY, "request already in progress");
    }

    // Send the frame on the interface using callback
    auto sendRes = _interface.sendFrame(request, staticHandleTxResult, this);
    if (sendRes != ModbusInterface::IInterface::SUCCESS) {
        _pendingRequest.setResult(ERR_TX_FAILED, true);
        return Error(ERR_TX_FAILED);
    }

    // In callback mode we just return immediately
    return Success();
}

// ===================================================================================
// PRIVATE METHODS
// ===================================================================================

/* @brief Handle a response from the interface
 * @param response The response to handle
 */
Client::Result Client::handleResponse(const Modbus::Frame& response)
{
    // TAKE CONTROL: Stop timer to prevent race condition
    _pendingRequest.stopTimer();
    
    // Check if request was still active (timer might have won)
    if (!_pendingRequest.isActive()) {
        return Error(ERR_INVALID_RESPONSE, "no request in progress");
    }

    // Reject any response to a broadcast request (safety check, probably unnecessary
    // as the request will be closed before any chance for the slave to respond)
    if (Modbus::isBroadcastId(_pendingRequest.getRequestMetadata().slaveId)) {
        return Error(ERR_INVALID_RESPONSE, "response to broadcast");
    }

    // Check if the response is from the right slave (unless catch-all is enabled)
    if (!_interface.checkCatchAllSlaveIds() 
        && response.slaveId != _pendingRequest.getRequestMetadata().slaveId) {
        return Error(ERR_INVALID_RESPONSE, "response from wrong slave");
    }

    // Check if response matches the expected FC
    if (response.type != Modbus::RESPONSE ||
        response.fc   != _pendingRequest.getRequestMetadata().fc) {
        return Error(ERR_INVALID_RESPONSE, "unexpected frame");
    }

    // Copy the response and re-inject the original metadata
    _responseBuffer.clear();
    _responseBuffer = response;
    const Modbus::FrameMeta& req = _pendingRequest.getRequestMetadata();
    _responseBuffer.regAddress = req.regAddress;   // <- original address
    _responseBuffer.regCount   = req.regCount;     // <- actual number of registers / coils

    // Propagate the response to the user and clean up the state
    // As long as this function call hasn't returned, no other request can be sent in parallel
    // (interface context is locked) which means the _responseBuffer is safe to be accessed
    // for the duration of user callback execution.
    _pendingRequest.setResponse(_responseBuffer, true);

    return Success();
}



/* @brief Static callback for TX result from interface layer
 * @param result The result of the TX operation
 * @param ctx Pointer to ModbusClient instance
 * @note Called from interface rxTxTask context when TX operation completes
 * @note Handles TX errors and broadcast request completion
 */
void Client::staticHandleTxResult(ModbusInterface::IInterface::Result result, void* pClient) {
    Client* client = static_cast<Client*>(pClient);
    
    // Ignore TX result if no request is active (might happen on late callbacks after timeout)
    if (!client->_pendingRequest.isActive()) {
        Modbus::Debug::LOG_MSG("Received TX result while no request in progress, ignoring");
        return;
    }

    // If the TX failed, it's an error - SUCCESS will be set by handleResponse()
    if (result != ModbusInterface::IInterface::SUCCESS) {
        // Something went wrong at the interface/queue/encoding step
        client->_pendingRequest.setResult(ERR_TX_FAILED, true);
        return;
    }
    
    // For broadcast successful TX, we create a dummy response and complete the request
    // (no response is expected for broadcasts)
    if (Modbus::isBroadcastId(client->_pendingRequest.getRequestMetadata().slaveId)) {
        // Use the shared response buffer rather than a local stack frame
        // (safe thanks to timing + safety checks in handleResponse())
        const Modbus::FrameMeta& reqMeta = client->_pendingRequest.getRequestMetadata();
        client->_responseBuffer.clear();
        client->_responseBuffer.type = Modbus::RESPONSE;
        client->_responseBuffer.fc = reqMeta.fc;
        client->_responseBuffer.slaveId = reqMeta.slaveId;
        client->_responseBuffer.regAddress = reqMeta.regAddress;
        client->_responseBuffer.regCount = reqMeta.regCount;
        client->_responseBuffer.exceptionCode = Modbus::NULL_EXCEPTION;
        // As long as this function call hasn't returned, no other request can be sent in parallel
        // (interface context is locked) which means the _responseBuffer is safe to be accessed
        // for the duration of user callback execution.
        client->_pendingRequest.setResponse(client->_responseBuffer, true);
        return;
    }
    
    // For non-broadcast successful TX, we just wait for the response in handleResponse()
}

// ===================================================================================
// HELPER METHODS IMPLEMENTATION
// ===================================================================================

/* @brief Read registers/coils with uint16_t buffer
 * @param slaveId Target slave ID
 * @param regType Register type (COIL, DISCRETE_INPUT, HOLDING_REGISTER, INPUT_REGISTER)
 * @param startAddr Starting address
 * @param qty Number of registers/coils to read
 * @param toBuf Buffer to store the read values
 * @param rspExcep Optional pointer to store exception code from response
 * @return Result code
 */
Client::Result Client::read(uint8_t slaveId, RegisterType regType, uint16_t startAddr, 
                            uint16_t qty, uint16_t* toBuf, ExceptionCode* rspExcep) {
    // Validate parameters
    if (!toBuf || qty == 0) {
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
        case COIL:
            _helperBuffer.fc = READ_COILS;
            break;
        case DISCRETE_INPUT:
            _helperBuffer.fc = READ_DISCRETE_INPUTS;
            break;
        case HOLDING_REGISTER:
            _helperBuffer.fc = READ_HOLDING_REGISTERS;
            break;
        case INPUT_REGISTER:
            _helperBuffer.fc = READ_INPUT_REGISTERS;
            break;
        default:
            return Error(ERR_INVALID_FRAME, "invalid register type");
    }
    
    // Send request synchronously - THE TRICK: reuse same buffer for response!
    Result res = sendRequest(_helperBuffer, _helperBuffer);
    
    // Handle response
    if (res == SUCCESS) {
        // Valid response with data
        if (regType == COIL || regType == DISCRETE_INPUT) {
            // For coils in uint16_t buffer, each coil takes one uint16_t (0 or 1)
            _helperBuffer.getCoils(toBuf, qty);
        } else {
            // For registers, direct copy
            _helperBuffer.getRegisters(toBuf, qty);
        }
        
        if (rspExcep) *rspExcep = NULL_EXCEPTION;
        return Success();
    } else if (res == ERR_EXCEPTION_RESPONSE) {
        // Exception response - extract exception code but no data
        if (rspExcep) *rspExcep = _helperBuffer.exceptionCode;
        return Error(ERR_EXCEPTION_RESPONSE, "Modbus exception received");
    } else {
        // Transmission or other error
        return Error(res, "Request failed");
    }
}

/* @brief Write registers/coils with uint16_t buffer
 * @param slaveId Target slave ID
 * @param regType Register type (COIL or HOLDING_REGISTER only)
 * @param startAddr Starting address
 * @param qty Number of registers/coils to write
 * @param fromBuf Buffer containing values to write
 * @param rspExcep Optional pointer to store exception code from response
 * @return Result code
 */
Client::Result Client::write(uint8_t slaveId, RegisterType regType, uint16_t startAddr, 
                             uint16_t qty, const uint16_t* fromBuf, ExceptionCode* rspExcep) {
    // Validate parameters
    if (!fromBuf || qty == 0) {
        return Error(ERR_INVALID_FRAME, "invalid buffer or quantity");
    }
    
    // Only COIL and HOLDING_REGISTER can be written
    if (regType != COIL && regType != HOLDING_REGISTER) {
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
    if (regType == COIL) {
        if (qty == 1) {
            _helperBuffer.fc = WRITE_COIL;
        } else {
            _helperBuffer.fc = WRITE_MULTIPLE_COILS;
        }
        if (!_helperBuffer.setCoils(fromBuf, qty)) {
            return Error(ERR_INVALID_FRAME, "failed to pack coils");
        }
    } else { // HOLDING_REGISTER
        if (qty == 1) {
            _helperBuffer.fc = WRITE_REGISTER;
        } else {
            _helperBuffer.fc = WRITE_MULTIPLE_REGISTERS;
        }
        if (!_helperBuffer.setRegisters(fromBuf, qty)) {
            return Error(ERR_INVALID_FRAME, "failed to set registers");
        }
    }
    
    // Send request synchronously - THE TRICK: reuse same buffer for response!
    Result res = sendRequest(_helperBuffer, _helperBuffer);
    
    // Handle response
    if (res == SUCCESS) {
        // Write successful
        if (rspExcep) *rspExcep = NULL_EXCEPTION;
        return Success();
    } else if (res == ERR_EXCEPTION_RESPONSE) {
        // Exception response - extract exception code
        if (rspExcep) *rspExcep = _helperBuffer.exceptionCode;
        return Error(ERR_EXCEPTION_RESPONSE, "Modbus exception received");
    } else {
        // Transmission or other error
        return Error(res, "Request failed");
    }
}

/* @brief Read coils/discrete inputs with bool buffer
 * @param slaveId Target slave ID
 * @param regType Register type (COIL or DISCRETE_INPUT only)
 * @param startAddr Starting address
 * @param qty Number of coils to read
 * @param toBuf Buffer to store the read values
 * @param rspExcep Optional pointer to store exception code from response
 * @return Result code
 */
Client::Result Client::read(uint8_t slaveId, RegisterType regType, uint16_t startAddr, 
                            uint16_t qty, bool* toBuf, ExceptionCode* rspExcep) {
    // Validate parameters
    if (!toBuf || qty == 0) {
        return Error(ERR_INVALID_FRAME, "invalid buffer or quantity");
    }
    
    // Only COIL and DISCRETE_INPUT can use bool buffer
    if (regType != COIL && regType != DISCRETE_INPUT) {
        return Error(ERR_INVALID_FRAME, "bool buffer requires COIL or DISCRETE_INPUT");
    }
    
    // Thread-safe access to helper buffer
    Lock guard(_helperMutex);
    
    // Build request in helper buffer
    _helperBuffer.clear();
    _helperBuffer.type = Modbus::REQUEST;
    _helperBuffer.slaveId = slaveId;
    _helperBuffer.regAddress = startAddr;
    _helperBuffer.regCount = qty;
    _helperBuffer.fc = (regType == COIL) ? READ_COILS : READ_DISCRETE_INPUTS;
    
    // Send request synchronously - THE TRICK: reuse same buffer for response!
    Result res = sendRequest(_helperBuffer, _helperBuffer);
    
    // Handle response
    if (res == SUCCESS) {
        // Valid response with data
        _helperBuffer.getCoils(toBuf, qty);
        
        if (rspExcep) *rspExcep = NULL_EXCEPTION;
        return Success();
    } else if (res == ERR_EXCEPTION_RESPONSE) {
        // Exception response - extract exception code but no data
        if (rspExcep) *rspExcep = _helperBuffer.exceptionCode;
        return Error(ERR_EXCEPTION_RESPONSE, "Modbus exception received");
    } else {
        // Transmission or other error
        return Error(res, "Request failed");
    }
}

/* @brief Write coils with bool buffer
 * @param slaveId Target slave ID
 * @param regType Register type (COIL only)
 * @param startAddr Starting address
 * @param qty Number of coils to write
 * @param fromBuf Buffer containing values to write
 * @param rspExcep Optional pointer to store exception code from response
 * @return Result code
 */
Client::Result Client::write(uint8_t slaveId, RegisterType regType, uint16_t startAddr, 
                             uint16_t qty, const bool* fromBuf, ExceptionCode* rspExcep) {
    // Validate parameters
    if (!fromBuf || qty == 0) {
        return Error(ERR_INVALID_FRAME, "invalid buffer or quantity");
    }
    
    // Only COIL can be written with bool buffer
    if (regType != COIL) {
        return Error(ERR_INVALID_FRAME, "bool buffer requires COIL type");
    }
    
    // Thread-safe access to helper buffer
    Lock guard(_helperMutex);
    
    // Build request in helper buffer
    _helperBuffer.clear();
    _helperBuffer.type = Modbus::REQUEST;
    _helperBuffer.slaveId = slaveId;
    _helperBuffer.regAddress = startAddr;
    _helperBuffer.regCount = qty;
    
    // Set function code and data based on quantity
    if (qty == 1) {
        _helperBuffer.fc = WRITE_COIL;
        _helperBuffer.data[0] = fromBuf[0] ? 0xFF00 : 0x0000;
    } else {
        _helperBuffer.fc = WRITE_MULTIPLE_COILS;
        if (!_helperBuffer.setCoils(fromBuf, qty)) {
            return Error(ERR_INVALID_FRAME, "failed to pack coils");
        }
    }
    
    // Send request synchronously - THE TRICK: reuse same buffer for response!
    Result res = sendRequest(_helperBuffer, _helperBuffer);
    
    // Handle response
    if (res == SUCCESS) {
        // Write successful
        if (rspExcep) *rspExcep = NULL_EXCEPTION;
        return Success();
    } else if (res == ERR_EXCEPTION_RESPONSE) {
        // Exception response - extract exception code
        if (rspExcep) *rspExcep = _helperBuffer.exceptionCode;
        return Error(ERR_EXCEPTION_RESPONSE, "Modbus exception received");
    } else {
        // Transmission or other error
        return Error(res, "Request failed");
    }
}


} // namespace Modbus
