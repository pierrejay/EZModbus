/**
 * @file ModbusClient.cpp
 * @brief Modbus client implementation
 */

#include "ModbusClient.h"

namespace Modbus {

// ===================================================================================
// DATA STRUCTURES METHODS
// ===================================================================================

/* @brief Tries to kill the current timer
 * @param maxWaitTicks Max wait time at each operation
 * @return true if the timer is stopped (won't hit afterwards), false if queuing command failed
 */
Client::PendingRequest::KillOutcome Client::PendingRequest::killTimer(TickType_t maxWaitTicks) {

    // Early return if no timer or already stopped
    if (!_timer) return KillOutcome::KILLED;
    if (xTimerIsTimerActive(_timer) == pdFALSE) {
        Modbus::Debug::LOG_MSG("Timer already inactive, OK");
        return KillOutcome::KILLED;
    }

    // Abort if we're in timer task
    if (xTaskGetCurrentTaskHandle() == xTimerGetTimerDaemonTaskHandle()) {
        Modbus::Debug::LOG_MSG("killTimer() called from timer daemon — forbidden");
        return KillOutcome::FAILED;
    }

    const TickType_t wait = (maxWaitTicks ? maxWaitTicks : 1);

    if (xTimerStop(_timer, wait) != pdPASS) {
        Modbus::Debug::LOG_MSG("xTimerStop enqueue failed, failed to kill timer");
        return KillOutcome::FAILED; // Nothing queued -> no guarantee, timer will clean up the request
    }

    while (_timerKillSem.tryTake()) {} // purge the semaphore

    // Function executed in timer daemon
    auto fenceFn = [](void* ctx, uint32_t){
        auto* pr = static_cast<PendingRequest*>(ctx);
        pr->_timerKillSem.giveForce();
    };

    // Try & post the fence w/ non-null delay (best-effort)
    if (xTimerPendFunctionCall(fenceFn, this, 0, wait) == pdPASS) {
        // Bounded wait for confirmation
        if (_timerKillSem.take(maxWaitTicks ? maxWaitTicks : 1)) {
            Modbus::Debug::LOG_MSG("Timer successfully killed (fence)");
            return KillOutcome::KILLED;
        }
        Modbus::Debug::LOG_MSG("Fence posted but no ACK yet");
    } else {
        Modbus::Debug::LOG_MSG("Fence enqueue failed");
    }

    return KillOutcome::STOPPED_NOACK;
}

/* @brief Timeout callback for pending request
 * @param timer The timer handle containing TimerTag
 */
void Client::PendingRequest::timeoutCallback(TimerHandle_t timer) {
    if (!timer) return;
    auto* pendingReq = static_cast<PendingRequest*>(pvTimerGetTimerID(timer));
    if (!pendingReq) return;

    pendingReq->timerClosing(true);
    // One-liner RAII closing guard: ensure we call timerClosing(false) in all return paths
    struct Closer { PendingRequest* p; ~Closer(){ p->timerClosing(false);} }
    closer{pendingReq};

    // Check if callback has been disabled by killTimer()
    if (pendingReq->isTimerCbDisarmed()) {
        Modbus::Debug::LOG_MSG("Timeout callback disabled, ignoring");
        return;
    }

    {
        Lock guard(pendingReq->_mutex);
        if (!pendingReq->_active) {
            Modbus::Debug::LOG_MSG("Timeout callback: request already inactive, ignoring");
            return;
        }
    }

    // Valid timeout for current transaction
    pendingReq->_client->_interface.abortCurrentTransaction();

    // Use dedicated timer method
    pendingReq->setResultFromTimer(ERR_TIMEOUT);
    Modbus::Debug::LOG_MSG("Request timed out via timer");
}

/* @brief Set the pending request (response & tracker version)
 * @param request The request to set
 * @param response Where to store the response (response.data will be resized to match its size)
 * @param tracker Pointer to the transfer result tracker
 * @param timeoutMs Timeout in milliseconds
 * @return true if the pending request was set successfully, false is already active (clear it first)
 */
bool Client::PendingRequest::set(const Modbus::Frame& request, Modbus::Frame* response,
                                 Result* tracker, uint32_t timeoutMs, EventGroupHandle_t waiterEventGroup) {
    if (isActive() || closingInProgress()) return false; // Fail-fast

    Lock guard(_mutex);

    if (isActive() || closingInProgress()) return false; // Avoid corruption & timer/response race

    _reqMetadata.type = Modbus::REQUEST;
    _reqMetadata.fc = request.fc;
    _reqMetadata.slaveId = request.slaveId;
    _reqMetadata.regAddress = request.regAddress;
    _reqMetadata.regCount = request.regCount;
    _pResponse = response;
    _tracker = tracker;
    _cb = nullptr; 
    _cbCtx = nullptr;
    _timestampMs = TIME_MS();
    _waiterEventGroup = waiterEventGroup;
    _active = true;

    // Rearm the timer callback & create timer if necessary
    rearmTimerCb();
    if (!_timer) {
        _timer = xTimerCreateStatic(
            "ModbusTimeout",
            pdMS_TO_TICKS(timeoutMs),
            pdFALSE,
            this,
            timeoutCallback,
            &_timerBuf
        );
        // Check if creation succeeded, and if Start command was sent to timer Q
        if (!_timer || xTimerStart(_timer, TIMER_CMD_TOUT_TICKS) != pdTRUE) {
            _active = false;
            return false;
        }
    } else {
        // Check if ChangePeriod command was sent to timer Q (will rearm the timer)
        if (xTimerChangePeriod(_timer, pdMS_TO_TICKS(timeoutMs), TIMER_CMD_TOUT_TICKS) != pdTRUE) {
            _active = false;
            return false;
        };
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
    if (isActive() || closingInProgress()) return false; // Fail-fast

    Lock guard(_mutex);
    
    if (isActive() || closingInProgress()) return false; // Avoid corruption & timer/response race

    _reqMetadata.type = Modbus::REQUEST;
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

    // Rearm the timer callback & create timer if necessary
    rearmTimerCb();
    if (!_timer) {
        _timer = xTimerCreateStatic(
            "ModbusTimeout",
            pdMS_TO_TICKS(timeoutMs),
            pdFALSE,
            this,
            timeoutCallback,
            &_timerBuf
        );
        // Check if creation succeeded, and if Start command was sent to timer Q
        if (!_timer || xTimerStart(_timer, TIMER_CMD_TOUT_TICKS) != pdTRUE) {
            _active = false;
            return false;
        }
    } else {
        // Check if ChangePeriod command was sent to timer Q (will rearm the timer)
        if (xTimerChangePeriod(_timer, pdMS_TO_TICKS(timeoutMs), TIMER_CMD_TOUT_TICKS) != pdTRUE) {
            _active = false;
            return false;
        };
    }
    
    return true;
}

/* @brief Clear the pending request
 */
void Client::PendingRequest::clear() {
    killTimer(portMAX_DELAY); // Only called in DTOR: wait indefinitely
    Lock guard(_mutex);
    if (!_active) return;
    _reqMetadata.clear();
    _tracker = nullptr;
    _pResponse = nullptr;
    _timestampMs = 0;
    _waiterEventGroup = nullptr; // Clear sync waiter
    _cb = nullptr;
    _cbCtx = nullptr;
    _active = false; // Disarms any remaining timeout callbacks
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
 * @param finalize Whether to finalize the request (default: true)
 * @param fromTimer Internal flag - DO NOT USE from timer callback, use setResultFromTimer() instead
 * @note ⚠️ DO NOT call this method from timer context! Use setResultFromTimer() instead
 */
void Client::PendingRequest::setResult(Result result, bool finalize, bool fromTimer) {
    Client::ResponseCallback cbSnapshot = nullptr;
    void* ctxSnapshot = nullptr;

    // Atomic gate : we are closing the request if finalize is true and call origins from response
    if (finalize && !fromTimer) respClosing(true);

    // One-liner RAII closing guard: ensure we call respClosing(false) in all return paths
    struct Closer { PendingRequest* p; bool en; ~Closer(){ if (en) p->respClosing(false);} } 
    closer{this, finalize && !fromTimer};

    // Kill timer first
    if (finalize && !fromTimer) {
        switch (killTimer(TIMER_CMD_TOUT_TICKS)) {
            case KillOutcome::KILLED:
                // safe to continue
                break;
            case KillOutcome::FAILED:
                Modbus::Debug::LOG_MSG("Failed to kill timer");
                return; // do not finalize here
            case KillOutcome::STOPPED_NOACK:
                Modbus::Debug::LOG_MSG("Timer kill command sent but delayed, disarming callback...");
                disarmTimerCb(); // logical neutralization of any late callback
                break;
        }
    }

    // Handle result & tracker under critical section
    {
        Lock guard(_mutex);
        if (!isActive()) return; // Exit if already cleared

        cbSnapshot = _cb;
        ctxSnapshot = _cbCtx;

        // Re-open the gates: it's now safe to publish & let API callers launch a new
        // request (the _active flag & _mutex protect us until we leave this function)
        if (fromTimer) timerClosing(false);
        else if (finalize) respClosing(false);
        
        if (_tracker) *_tracker = result;
        if (finalize) {
            resetUnsafe();
            notifySyncWaiterUnsafe();
        }
    }

    // Important - always call callback outside of mutex
    // Since we don't expect a response (failure or broadcast), we pass nullptr as response
    if (cbSnapshot) cbSnapshot(result, nullptr, ctxSnapshot);
}

/* @brief Set result from timer callback
 * @param result The timeout result (typically ERR_TIMEOUT)
 * @note This method should ONLY be called from timeoutCallback()
 */
void Client::PendingRequest::setResultFromTimer(Result result) {
    setResult(result, true, true); // Forces finalize=true, fromTimer=true
}

/* @brief Set the response for the pending request & update the result tracker
 * @param response The response to set
 * @param finalize Whether to finalize the request (default: true)
 * @param fromTimer Internal flag - should always be false for responses
 * @note ⚠️ DO NOT call this method from timer context!
 */
void Client::PendingRequest::setResponse(const Modbus::Frame& response, bool finalize) {
    Client::ResponseCallback cbSnapshot = nullptr;
    void* ctxSnapshot = nullptr;

    // Atomic gate : we are closing the request if finalize is true and call origins from response
    if (finalize) respClosing(true);
    
    // One-liner RAII closing guard: ensure we call respClosing(false) in all return paths
    struct Closer { PendingRequest* p; bool en; ~Closer(){ if (en) p->respClosing(false);} } 
    closer{this, finalize};

    // Kill timer first
    if (finalize) {
        switch (killTimer(TIMER_CMD_TOUT_TICKS)) {
            case KillOutcome::KILLED:
                // safe to continue
                break;
            case KillOutcome::FAILED:
                Modbus::Debug::LOG_MSG("Failed to kill timer");
                return; // do not finalize here
            case KillOutcome::STOPPED_NOACK:
                Modbus::Debug::LOG_MSG("Timer kill command sent but delayed, disarming callback...");
                disarmTimerCb(); // logical neutralization of any late callback
                break;
        }
    }

    // Handle response & tracker under critical section
    {
        Lock guard(_mutex);
        if (!isActive()) return; // Exit if already cleared

        cbSnapshot = _cb;
        ctxSnapshot = _cbCtx;

        // Re-open the gates: it's now safe to publish & let API callers launch a new
        // request (the _active flag & _mutex protect us until we leave this function)
        if (finalize) respClosing(false);

        if (_pResponse) *_pResponse = response;
        if (_tracker) *_tracker = SUCCESS;
        if (finalize) resetUnsafe();
        notifySyncWaiterUnsafe();
    }

    // Important - always call callback outside of mutex
    if (cbSnapshot) cbSnapshot(SUCCESS, &response, ctxSnapshot);
}

/* @brief Notify the synchronous waiting task
 * @note This method MUST be called while holding _mutex to ensure atomicity
 */
void Client::PendingRequest::notifySyncWaiterUnsafe() {
    // This method should be called while holding the mutex
    if (_waiterEventGroup) {
        xEventGroupSetBits(_waiterEventGroup, SYNC_COMPLETION_BIT);
        _waiterEventGroup = nullptr;
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

/* @brief Destructor for PendingRequest - cleanup timer
 */
Client::PendingRequest::~PendingRequest() {
    if (_timer) {
        xTimerDelete(_timer, 0);
        _timer = nullptr;
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
    return (_interface.isReady() 
            && !_pendingRequest.isActive()
            && !_pendingRequest.closingInProgress());
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

    // In sync mode, spawn the waiter event group before initializing the request
    EventGroupHandle_t syncEvtGrp = nullptr;
    if (!userTracker) {
        syncEvtGrp = xEventGroupCreateStatic(&_waiterEventGroupBuf);
        if (!syncEvtGrp) {
            return Error(ERR_BUSY, "cannot create waiter event group");
        }
    }

    // Basic checks passed, we try to initiate the pending request
    if (!_pendingRequest.set(request, &response, tracker, _requestTimeoutMs, syncEvtGrp)) {
        if (syncEvtGrp) vEventGroupDelete(syncEvtGrp); // Cleanup on failure
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
    // Capture pending request metadata
    PendingRequest::PendingSnapshot snapshot;
    if (!_pendingRequest.snapshotIfActive(snapshot)) {
        return Error(ERR_INVALID_RESPONSE, "no request in progress");
    }
    Modbus::FrameMeta& reqSnapshot = snapshot.reqMetadata;

    // Build the response

    // Reject any response to a broadcast request
    if (Modbus::isBroadcastId(reqSnapshot.slaveId)) {
        return Error(ERR_INVALID_RESPONSE, "response to broadcast");
    }

    // Check if the response is from the right slave (unless catch-all is enabled)
    if (!_interface.checkCatchAllSlaveIds() && response.slaveId != reqSnapshot.slaveId) {
        return Error(ERR_INVALID_RESPONSE, "response from wrong slave");
    }

    // Check if response matches the expected FC
    if (response.type != Modbus::RESPONSE || response.fc != reqSnapshot.fc) {
        return Error(ERR_INVALID_RESPONSE, "unexpected frame");
    }

    // Copy the response and re-inject the original metadata using snapshot
    _responseBuffer.clear();
    _responseBuffer = response;
    _responseBuffer.regAddress = reqSnapshot.regAddress;   // <- original address from snapshot
    _responseBuffer.regCount   = reqSnapshot.regCount;     // <- actual number of registers / coils from snapshot

    // Complete the response: setResponse() handles potential conflicts with timer
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

    // Capture metadata using existing API (fast, thread-safe)
    PendingRequest::PendingSnapshot snapshot;
    if (!client->_pendingRequest.snapshotIfActive(snapshot)) {
        return; // No active request, nothing to do
    }
    Modbus::FrameMeta& reqSnapshot = snapshot.reqMetadata;

    // Check if this requires finalization (error or broadcast)
    if (result != ModbusInterface::IInterface::SUCCESS || Modbus::isBroadcastId(reqSnapshot.slaveId)) {
        // Finalize based on result
        if (result != ModbusInterface::IInterface::SUCCESS) {
            client->_pendingRequest.setResult(ERR_TX_FAILED, true);
        } else { // Successful broadcast
            client->_responseBuffer.clear();
            client->_responseBuffer.type = Modbus::RESPONSE;
            client->_responseBuffer.fc = reqSnapshot.fc;
            client->_responseBuffer.slaveId = reqSnapshot.slaveId;
            client->_responseBuffer.regAddress = reqSnapshot.regAddress;
            client->_responseBuffer.regCount = reqSnapshot.regCount;
            client->_responseBuffer.exceptionCode = Modbus::NULL_EXCEPTION;
            client->_responseBuffer.clearData(false);
            client->_pendingRequest.setResponse(client->_responseBuffer, true);
        }
        return;
    }
    // Pour un TX non-broadcast réussi, on ne fait rien, on attend la réponse.
}


} // namespace Modbus
