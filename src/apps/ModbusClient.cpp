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
 * @param timer The timer handle containing TimerTag
 *
 * This callback filters out late/obsolete timeout events using epoch comparison.
 * Only timeouts matching the current active transaction epoch are processed.
 */
void Client::PendingRequest::timeoutCallback(TimerHandle_t timer) {
    auto* tag = static_cast<TimerTag*>(pvTimerGetTimerID(timer));
    if (!tag || !tag->self) {
        return; // Invalid timer tag
    }

    auto* pendingReq = tag->self;

    // Race protection: Filter by epoch + active state
    {
        Lock guard(pendingReq->_mutex);
        if (!pendingReq->_active) {
            Modbus::Debug::LOG_MSG("Timeout callback: request already inactive, ignoring");
            return;
        }
        if (tag->gen != pendingReq->_timGen) {
            Modbus::Debug::LOG_MSGF("Timeout callback: epoch mismatch (timer=%u, current=%u), ignoring late timeout",
                                  tag->gen, pendingReq->_timGen);
            return;
        }
    }

    // Valid timeout for current transaction
    pendingReq->_client->_interface.abortCurrentTransaction();
    pendingReq->setResult(ERR_TIMEOUT, true, true);
    Modbus::Debug::LOG_MSGF("Request timed out via timer (epoch %u)", tag->gen);
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
    if (isActive()) return false; // Fail-fast if pending request already active
    Lock guard(_mutex);
    if (isActive()) return false; // Avoid corruption if another set() call won the lock race
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

    // Timer race protection
    ++_timGen;                        // Increment current generation
    _timIdx = ++_timIdx % TIM_SLOTS;  // Alternate between timer slots (0...TIM_SLOTS-1)

    // Create timer for current slot if needed
    if (!_tim[_timIdx]) {
        _tim[_timIdx] = xTimerCreateStatic(
            "ModbusTimeout",
            pdMS_TO_TICKS(timeoutMs),
            pdFALSE,  // one-shot
            &_timTag[_timIdx],  // timer ID points to tag (will be updated below)
            timeoutCallback,
            &_timBuf[_timIdx]
        );
    }

    // Set immutable tag for this transaction (epoch + self pointer)
    _timTag[_timIdx] = { this, _timGen };
    vTimerSetTimerID(_tim[_timIdx], &_timTag[_timIdx]);

    // Configure and start timer
    if (_tim[_timIdx]) {
        xTimerChangePeriod(_tim[_timIdx], pdMS_TO_TICKS(timeoutMs), 0);
        xTimerStart(_tim[_timIdx], 0);
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
    if (isActive()) return false; // Fail-fast if pending request already active
    Lock guard(_mutex);
    if (isActive()) return false; // Avoid corruption if another set() call won the lock race
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

    // Race protection: increment epoch & toggle timer handle
    ++_timGen;
    _timIdx ^= 1;  // Alternate between timer slots (0 <-> 1)

    // Create timer for current slot if needed
    if (!_tim[_timIdx]) {
        _tim[_timIdx] = xTimerCreateStatic(
            "ModbusTimeout",
            pdMS_TO_TICKS(timeoutMs),
            pdFALSE,  // one-shot
            &_timTag[_timIdx],  // timer ID points to tag (will be updated below)
            timeoutCallback,
            &_timBuf[_timIdx]
        );
    }

    // Set immutable tag for this transaction (epoch + self pointer)
    _timTag[_timIdx] = { this, _timGen };
    vTimerSetTimerID(_tim[_timIdx], &_timTag[_timIdx]);

    // Configure and start timer
    if (_tim[_timIdx]) {
        xTimerChangePeriod(_tim[_timIdx], pdMS_TO_TICKS(timeoutMs), 0);
        xTimerStart(_tim[_timIdx], 0);
    }
    return true;
}

/* @brief Clear the pending request
 */
void Client::PendingRequest::clear() {  
    TimerHandle_t timerToStop = nullptr;  
    {
        Lock guard(_mutex);
        if (!_active) return;
        timerToStop = currentTimer();
        _reqMetadata.clear();
        _tracker = nullptr;
        _pResponse = nullptr;
        _timestampMs = 0;
        _syncEventGroup = nullptr; // Clear sync waiter
        _cb = nullptr;
        _cbCtx = nullptr;
        _active = false; // Disarms any remaining timeout callbacks
    }

    if (timerToStop) xTimerStop(timerToStop,0);
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
void Client::PendingRequest::setResult(Result result, bool finalize, bool fromTimer) {
    Client::ResponseCallback cbSnapshot = nullptr;
    void* ctxSnapshot = nullptr;
    TimerHandle_t timerToStop = nullptr;

    // Handle result & tracker under critical section
    { 
        Lock guard(_mutex);
        if (!isActive()) return; // Exit if already cleared
        if (_tracker) *_tracker = result;
        cbSnapshot = _cb;
        ctxSnapshot = _cbCtx;
        if (finalize) {
            if (!fromTimer) timerToStop = currentTimer();
            resetUnsafe(); // also sets _active=false
        }
        notifySyncWaiterUnsafe(); // still inside mutex
    }

    // Kill timer (best-effort) & invoke callback outside of critical section
    if (timerToStop) xTimerStop(timerToStop, 0);
    // Since we don't expect a response (failure or broadcast), we pass nullptr as response
    if (cbSnapshot) cbSnapshot(result, nullptr, ctxSnapshot);
}

/* @brief Set the response for the pending request & update the result tracker
 * @param response The response to set
 */
void Client::PendingRequest::setResponse(const Modbus::Frame& response, bool finalize, bool fromTimer) {
    Client::ResponseCallback cbSnapshot = nullptr;
    void* ctxSnapshot = nullptr;
    TimerHandle_t timerToStop = nullptr;

    // Handle response & tracker under critical section
    {
        Lock guard(_mutex);
        if (!isActive()) return; // Exit if already cleared
        if (_pResponse) *_pResponse = response;
        if (_tracker) *_tracker = SUCCESS;
        cbSnapshot = _cb;
        ctxSnapshot = _cbCtx;
        if (finalize) {
            if (!fromTimer) timerToStop = currentTimer();
            resetUnsafe(); // also sets _active=false
        }
        notifySyncWaiterUnsafe(); // still inside mutex
    }

    // Kill timer (best-effort) & invoke callback outside of critical section
    if (timerToStop) xTimerStop(timerToStop, 0);
    if (cbSnapshot) cbSnapshot(SUCCESS, &response, ctxSnapshot);
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

/* @brief Snapshot request metadata if still active (thread-safe)
 * @param out Output structure to fill with metadata snapshot
 * @return true if request was active and snapshot captured, false otherwise
 *
 * This provides a lock-free way for handleResponse to validate responses
 * without holding the mutex during the entire validation process.
 */
bool Client::PendingRequest::snapshotIfActive(PendingSnapshot& out) {
    Lock guard(_mutex);
    if (!_active) {
        return false;
    }
    out.reqMetadata = _reqMetadata;
    return true;
}

/* @brief Destructor for PendingRequest - cleanup both timers
 */
Client::PendingRequest::~PendingRequest() {
    // Clean up both timer slots
    for (int i = 0; i < 2; ++i) {
        if (_tim[i]) {
            xTimerDelete(_tim[i], 0);
            _tim[i] = nullptr;
        }
    }
}


/* @brief Reset the pending request to its initial state
 * @note This method MUST be called while holding _mutex to ensure atomicity
 */
void Client::PendingRequest::resetUnsafe() {
    _reqMetadata.clear();
    _tracker = nullptr;
    _pResponse = nullptr;
    _timestampMs = 0;
    _cb = nullptr;
    _cbCtx = nullptr;
    _active = false; // This disarms any remaining timeout callbacks
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
 *
 * Uses snapshot approach to avoid holding mutex during validation.
 * No timer stop synchronization needed - epoch guards provide safety.
 */
Client::Result Client::handleResponse(const Modbus::Frame& response)
{
    // Timer race protection: take atomic snapshot
    PendingRequest::PendingSnapshot snapshot;
    if (!_pendingRequest.snapshotIfActive(snapshot)) {
        return Error(ERR_INVALID_RESPONSE, "no request in progress");
    }

    // Reject any response to a broadcast request (safety check, probably unnecessary
    // as the request will be closed before any chance for the slave to respond)
    if (Modbus::isBroadcastId(snapshot.reqMetadata.slaveId)) {
        return Error(ERR_INVALID_RESPONSE, "response to broadcast");
    }

    // Check if the response is from the right slave (unless catch-all is enabled)
    if (!_interface.checkCatchAllSlaveIds()
        && response.slaveId != snapshot.reqMetadata.slaveId) {
        return Error(ERR_INVALID_RESPONSE, "response from wrong slave");
    }

    // Check if response matches the expected FC
    if (response.type != Modbus::RESPONSE ||
        response.fc   != snapshot.reqMetadata.fc) {
        return Error(ERR_INVALID_RESPONSE, "unexpected frame");
    }

    // Copy the response and re-inject the original metadata
    _responseBuffer.clear();
    _responseBuffer = response;
    _responseBuffer.regAddress = snapshot.reqMetadata.regAddress;   // <- original address
    _responseBuffer.regCount   = snapshot.reqMetadata.regCount;     // <- actual number of registers / coils

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

    // Timer race protection: take atomic snapshot
    PendingRequest::PendingSnapshot snapshot;
    if (!client->_pendingRequest.snapshotIfActive(snapshot)) {
        Modbus::Debug::LOG_MSG("Received TX result while no request in progress, ignoring");
        return;
    }

    // If the TX failed, it's an error - SUCCESS will be set by handleResponse()
    if (result != ModbusInterface::IInterface::SUCCESS) {
        // Something went wrong at the interface/queue/encoding step: finalize request
        client->_pendingRequest.setResult(ERR_TX_FAILED, true);
        Modbus::Debug::LOG_MSG("Received TX error");
        return;
    }

    // For broadcast successful TX, we create a dummy response and complete the request
    // (no response is expected for broadcasts)
    if (Modbus::isBroadcastId(snapshot.reqMetadata.slaveId)) {
        // Use the shared response buffer with snapshot data (thread-safe)
        client->_responseBuffer.clear();
        client->_responseBuffer = snapshot.reqMetadata;  // Based on safe snapshot
        client->_responseBuffer.type = Modbus::RESPONSE;
        client->_responseBuffer.exceptionCode = Modbus::NULL_EXCEPTION;
        client->_responseBuffer.clearData(false);
        client->_pendingRequest.setResponse(client->_responseBuffer, true);
        return;
    }

    // For non-broadcast successful TX, we just wait for the response in handleResponse()
}


} // namespace Modbus
