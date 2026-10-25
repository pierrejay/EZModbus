/**
 * @file ModbusClient.cpp
 * @brief Modbus client implementation
 */

#include "ModbusClient.h"

// Request timeouts are handled on the FreeRTOS timer task: with debug traces, it needs ~2.5 KB
#if defined(EZMODBUS_DEBUG) && defined(CONFIG_FREERTOS_TIMER_TASK_STACK_DEPTH) && CONFIG_FREERTOS_TIMER_TASK_STACK_DEPTH < 3072
    #warning "EZMODBUS_DEBUG: set CONFIG_FREERTOS_TIMER_TASK_STACK_DEPTH to 4096 (timer task stack too small for debug traces)"
#endif

namespace Modbus {

// ===================================================================================
// DATA STRUCTURES METHODS
// ===================================================================================

namespace {
// Request timeout in ticks, rounded up & at least 1 tick (the timer must not fire before it)
TickType_t timeoutToTicks(uint32_t timeoutMs) {
    uint64_t ticks = ((uint64_t)timeoutMs + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
    if (ticks == 0) ticks = 1;
    if (ticks >= portMAX_DELAY) ticks = portMAX_DELAY - 1;
    return (TickType_t)ticks;
}
} // namespace

/* @brief Start a request
 * @param request The request
 * @param response Where to store the response (sync or async w/ tracker), nullptr otherwise
 * @param tracker Result tracker (sync or async w/ tracker), nullptr otherwise
 * @param cb Completion callback (async w/ callback), nullptr otherwise
 * @param cbCtx Completion callback context
 * @param waiter Event group of a synchronous caller, nullptr otherwise
 * @return SUCCESS if started (the caller then sends it), ERR_BUSY or ERR_TIMER_FAILURE otherwise
 * @note The timer is re-armed without waiting under _mutex: if its command can't be queued,
 *       nothing was published, so no event can observe the failed attempt
 */
Client::Result Client::PendingRequest::start(const Modbus::Frame& request, Modbus::Frame* response,
                                             Result* tracker, ResponseCallback cb, void* cbCtx,
                                             EventGroupHandle_t waiter) {
    Lock guard(_mutex);
    if (_active || _txPending) return ERR_BUSY;

    if (!_timer) {
        _timer = xTimerCreateStatic("ModbusTimeout", _timeoutTicks, pdFALSE, this, timeoutCallback, &_timerBuf);
        if (!_timer) return ERR_TIMER_FAILURE;
    }
    _startTick = xTaskGetTickCount(); // Before re-arming: the timer can't expire before the deadline
    if (xTimerChangePeriod(_timer, _timeoutTicks, 0) != pdPASS) return ERR_TIMER_FAILURE;

    _meta = Modbus::FrameMeta(request);
    _pResponse = response;
    _tracker = tracker;
    _cb = cb;
    _cbCtx = cbCtx;
    _waiterEventGroup = waiter;
    if (_tracker) *_tracker = NODATA;
    _active = true;
    _txPending = true;
    return SUCCESS;
}

/* @brief TX result of the last request
 * @note Called once per sendFrame() call (IInterface contract): from the interface task, or from
 *       the calling task when sendFrame() fails at once
 */
void Client::PendingRequest::onTxResult(ModbusInterface::IInterface::Result result) {
    Completion completion;
    {
        Lock guard(_mutex);
        _txPending = false;
        if (!_active) return; // Already ended (e.g. timeout while the interface was reconnecting)
        if (result != ModbusInterface::IInterface::SUCCESS) {
            completion = finishLocked(ERR_TX_FAILED, nullptr);
        } else if (Modbus::isBroadcastId(_meta.slaveId)) {
            // No response expected: a broadcast completes once sent
            Modbus::Frame& resp = _client->_responseBuffer;
            resp.clear();
            resp.type = Modbus::RESPONSE;
            resp.fc = _meta.fc;
            resp.slaveId = _meta.slaveId;
            resp.regAddress = _meta.regAddress;
            resp.regCount = _meta.regCount;
            completion = finishLocked(SUCCESS, &resp);
        } else {
            return; // Sent: wait for the response
        }
    }
    completion.deliver();
}

/* @brief Response received from the interface
 * @return True if it matched the active request & completed it
 * @note Not before the TX result of the request was notified: until then, the frame answers an
 *       earlier one (e.g. a request started while that response is delivered to the interface's consumers)
 * @note Responses are matched by slave ID & FC (TCP responses are already paired by transaction ID).
 *       Over RTU, a response arriving after its request timed out can't be told apart from the
 *       answer to the next request: the timeout must exceed the slave's worst-case response time.
 *       The content isn't checked against the request (e.g. echoed address, number of values): many
 *       slaves aren't strictly compliant, and the spec wouldn't identify the request anyway.
 */
bool Client::PendingRequest::onResponse(const Modbus::Frame& response) {
    Completion completion;
    {
        Lock guard(_mutex);
        if (!_active || _txPending || response.type != Modbus::RESPONSE) return false;
        if (Modbus::isBroadcastId(_meta.slaveId)) return false; // No response to a broadcast
        if (!_client->_interface.checkCatchAllSlaveIds() && response.slaveId != _meta.slaveId) return false;
        if (response.fc != _meta.fc) return false;

        Modbus::Frame& resp = _client->_responseBuffer;
        resp = response;
        resp.regAddress = _meta.regAddress; // Read responses don't carry the address,
        resp.regCount = _meta.regCount;     // nor the exact number of coils
        completion = finishLocked(SUCCESS, &resp);
    }
    completion.deliver();
    return true;
}

/* @brief Timer callback: the active request didn't get its outcome in time
 * @note A late event (request ended meanwhile, or a newer one started) finds no request whose
 *       deadline is reached and does nothing: a request is never timed out early
 */
void Client::PendingRequest::timeoutCallback(TimerHandle_t timer) {
    auto* self = static_cast<PendingRequest*>(pvTimerGetTimerID(timer));
    Completion completion;
    {
        Lock guard(self->_mutex);
        if (!self->_active || !self->deadlineReachedLocked()) return;
        self->_client->_interface.abortCurrentTransaction(); // Before a newer request can start
        completion = self->finishLocked(ERR_TIMEOUT, nullptr);
    }
    completion.deliver();
    Modbus::Debug::LOG_MSG("Request timed out");
}

/* @brief The synchronous wait expired: fallback when the timer daemon is late
 * @param waiter The caller's event group
 * @note Ends the request only if it's still the caller's one: it may have completed meanwhile
 *       (its result is in the caller's tracker), and another request may have started since
 */
void Client::PendingRequest::finishSyncWait(EventGroupHandle_t waiter) {
    Lock guard(_mutex);
    if (!_active || _waiterEventGroup != waiter) return;
    _client->_interface.abortCurrentTransaction(); // Before a newer request can start
    (void)finishLocked(ERR_TIMEOUT, nullptr); // Nothing to deliver: no callback in sync mode
}

/* @brief End the active request (under _mutex)
 * @return The completion to deliver once _mutex is released
 */
Client::PendingRequest::Completion Client::PendingRequest::finishLocked(Result result, const Modbus::Frame* response) {
    if (response && _pResponse) *_pResponse = *response;
    if (_tracker) *_tracker = result; // After the response: whoever sees the result reads a complete response
    if (_waiterEventGroup) xEventGroupSetBits(_waiterEventGroup, SYNC_COMPLETION_BIT);

    Completion completion;
    completion.cb = _cb;
    completion.cbCtx = _cbCtx;
    completion.result = result;
    completion.response = response;

    _active = false;
    _pResponse = nullptr;
    _tracker = nullptr;
    _cb = nullptr;
    _cbCtx = nullptr;
    _waiterEventGroup = nullptr;
    return completion;
}

/* @brief Drop the active request without completing it (destruction)
 */
void Client::PendingRequest::detach() {
    Lock guard(_mutex);
    _active = false;
    _pResponse = nullptr;
    _tracker = nullptr;
    _cb = nullptr;
    _cbCtx = nullptr;
    _waiterEventGroup = nullptr;
}

bool Client::PendingRequest::isIdle() {
    Lock guard(_mutex);
    return !_active && !_txPending;
}

bool Client::PendingRequest::isActive() {
    Lock guard(_mutex);
    return _active;
}

bool Client::PendingRequest::reserveSyncWaiter() {
    Lock guard(_mutex);
    if (_syncWaiterInUse) return false;
    _syncWaiterInUse = true;
    return true;
}

void Client::PendingRequest::releaseSyncWaiter() {
    Lock guard(_mutex);
    _syncWaiterInUse = false;
}

/* @brief Delete the timer, waiting until its callback can't run anymore
 * @note A timeout callback already running completes first
 */
void Client::PendingRequest::deleteTimer() {
    if (!_timer) return;
    xTimerDelete(_timer, portMAX_DELAY);
    // Fence: runs in the timer daemon after the delete command, once no callback can run
    if (xTaskGetCurrentTaskHandle() != xTimerGetTimerDaemonTaskHandle()) {
        while (_timerDeleteFence.tryTake()) {} // Purge the semaphore
        xTimerPendFunctionCall([](void* ctx, uint32_t) {
            static_cast<PendingRequest*>(ctx)->_timerDeleteFence.giveForce();
        }, this, 0, portMAX_DELAY);
        _timerDeleteFence.take();
    }
    _timer = nullptr;
}

/* @brief Destructor for PendingRequest - cleanup timer
 */
Client::PendingRequest::~PendingRequest() {
    deleteTimer();
}

// ===================================================================================
// PUBLIC METHODS
// ===================================================================================

Client::Client(ModbusInterface::IInterface& interface, uint32_t timeoutMs) : 
    _interface(interface),
    _pendingRequest(this, timeoutToTicks(timeoutMs)),
    _isInitialized(false)
{}

Client::~Client() {
    // The interface & the timeout timer must not call this object anymore: a request still
    // held by the interface is aborted (best effort), and its TX result waited for
    _interface.removeRcvCallbacks(this);
    if (_pendingRequest.isActive()) _interface.abortCurrentTransaction();
    _interface.waitTxCallbackDone(this);
    _pendingRequest.deleteTimer();
    _pendingRequest.detach(); // A request still pending is dropped without completion
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
 * @return true if interface ready, no active request & no TX result awaited
 */
bool Client::isReady() {
    if (!_isInitialized) return false;
    return _interface.isReady() && _pendingRequest.isIdle();
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
    auto validResult = ModbusCodec::isValidFrame(request, _interface.checkExtendedSlaveIds());
    if (validResult != ModbusCodec::SUCCESS) {
        return Error(ERR_INVALID_FRAME, "invalid fields");
    }
    if (!isReady()) {
        return Error(ERR_BUSY, "interface busy or active pending request");
    }

    // Sync mode: local tracker & event group, created once its storage is reserved
    Result localResult = NODATA;
    Result* tracker = userTracker ? userTracker : &localResult;
    EventGroupHandle_t syncEvtGrp = nullptr;
    if (!userTracker) {
        if (!_pendingRequest.reserveSyncWaiter()) {
            return Error(ERR_BUSY, "synchronous request already in progress");
        }
        syncEvtGrp = xEventGroupCreateStatic(&_waiterEventGroupBuf);
        if (!syncEvtGrp) {
            _pendingRequest.releaseSyncWaiter();
            return Error(ERR_BUSY, "cannot create waiter event group");
        }
    }

    Result startRes = _pendingRequest.start(request, &response, tracker, nullptr, nullptr, syncEvtGrp);
    if (startRes != SUCCESS) {
        if (syncEvtGrp) {
            vEventGroupDelete(syncEvtGrp);
            _pendingRequest.releaseSyncWaiter();
        }
        return Error(startRes, "cannot start the request");
    }

    // A failure is reported through the TX callback (synchronously or not), which ends the request
    bool sent = _interface.sendFrame(request, staticHandleTxResult, this) == ModbusInterface::IInterface::SUCCESS;

    // ---------- Asynchronous mode (userTracker != nullptr) ----------
    if (userTracker) {
        return sent ? Success() : Error(ERR_TX_FAILED);
    }

    // ---------- Synchronous mode (userTracker == nullptr) ----------
    // Wait for the outcome (response, TX error or timeout), with a margin for a late timer daemon
    const TickType_t margin = pdMS_TO_TICKS(100);
    const TickType_t timeoutTicks = _pendingRequest.getTimeoutTicks();
    const TickType_t waitTicks = (timeoutTicks < portMAX_DELAY - margin) ? timeoutTicks + margin : portMAX_DELAY - 1;
    EventBits_t bits = xEventGroupWaitBits(syncEvtGrp, SYNC_COMPLETION_BIT, pdTRUE, pdFALSE, waitTicks);
    if ((bits & SYNC_COMPLETION_BIT) == 0) {
        _pendingRequest.finishSyncWait(syncEvtGrp);
    }
    // The request no longer refers to the event group nor to localResult
    vEventGroupDelete(syncEvtGrp);
    _pendingRequest.releaseSyncWaiter();

    if (localResult != SUCCESS) return Error(localResult);
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
    auto validResult = ModbusCodec::isValidFrame(request, _interface.checkExtendedSlaveIds());
    if (validResult != ModbusCodec::SUCCESS) {
        return Error(ERR_INVALID_FRAME, "invalid fields");
    }
    if (!isReady()) {
        return Error(ERR_BUSY, "interface busy or active pending request");
    }

    Result startRes = _pendingRequest.start(request, nullptr, nullptr, cb, userCtx, nullptr);
    if (startRes != SUCCESS) {
        return Error(startRes, "cannot start the request");
    }

    // A failure is reported through the TX callback (synchronously or not), which ends the request
    if (_interface.sendFrame(request, staticHandleTxResult, this) != ModbusInterface::IInterface::SUCCESS) {
        return Error(ERR_TX_FAILED);
    }
    return Success();
}

// ===================================================================================
// PRIVATE METHODS
// ===================================================================================

/* @brief Handle a response from the interface
 * @param response The response to handle
 */
Client::Result Client::handleResponse(const Modbus::Frame& response) {
    if (!_pendingRequest.onResponse(response)) {
        return Error(ERR_INVALID_RESPONSE, "no request in progress matching the response");
    }
    return Success();
}

/* @brief Static callback for TX result from interface layer
 * @param result The result of the TX operation
 * @param ctx Pointer to ModbusClient instance
 * @note Called once per sendFrame() call: from the interface task, or from the calling task when
 *       sendFrame() fails at once
 */
void Client::staticHandleTxResult(ModbusInterface::IInterface::Result result, void* pClient) {
    static_cast<Client*>(pClient)->_pendingRequest.onTxResult(result);
}


// ===================================================================================
// CONVENIENCE HELPER METHODS IMPLEMENTATION
// ===================================================================================

// Helper methods implementations are now in ModbusClient.h as templates



} // namespace Modbus
