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
    _reqMetadata.fc = request.fc;
    _reqMetadata.slaveId = request.slaveId;
    _reqMetadata.regAddress = request.regAddress;
    _reqMetadata.regCount = request.regCount;
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
    _reqMetadata.fc = request.fc;
    _reqMetadata.slaveId = request.slaveId;
    _reqMetadata.regAddress = request.regAddress;
    _reqMetadata.regCount = request.regCount;
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
const Modbus::Frame& Client::PendingRequest::getRequestMetadata() const { return _reqMetadata; }

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

    // Handle response & tracker under critical section
    {
        Lock guard(_mutex);
        if (_pResponse) *_pResponse = response;
        if (_tracker) *_tracker = SUCCESS;
        cbSnapshot = _cb;
        ctxSnapshot = _cbCtx;
        if (finalize) {
            resetUnsafe(); // also sets _active=false
        }
        notifySyncWaiterUnsafe(); // still inside mutex
    }

    // Invoke callback outside of critical section
    if (cbSnapshot) {
        cbSnapshot(SUCCESS, &response, ctxSnapshot);
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
    const Modbus::Frame& req = _pendingRequest.getRequestMetadata();
    _responseBuffer.regAddress = req.regAddress;   // <- original address
    _responseBuffer.regCount   = req.regCount;     // <- actual number of registers / coils

    // Propagate the response to the user and clean up the state
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
        client->_responseBuffer.clear();
        client->_responseBuffer = client->_pendingRequest.getRequestMetadata();
        client->_responseBuffer.type = Modbus::RESPONSE;
        client->_responseBuffer.exceptionCode = Modbus::NULL_EXCEPTION;
        client->_responseBuffer.clearData(false);
        client->_pendingRequest.setResponse(client->_responseBuffer, true);
        return;
    }
    
    // For non-broadcast successful TX, we just wait for the response in handleResponse()
}


} // namespace Modbus
