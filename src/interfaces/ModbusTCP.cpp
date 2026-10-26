/**
 * @file ModbusTCP.cpp
 * @brief Implementation of the ModbusTCP class
 */

#include "ModbusTCP.h"

namespace ModbusInterface {

// ===================================================================================
// PUBLIC METHODS
// ===================================================================================

TCP::TCP(ModbusHAL::TCP& hal, Modbus::Role role)
    : _tcpHAL(hal),
      _rxBuffer(_rxBuf.data(), _rxBuf.size()),
      _txBuffer(_txBuf.data(), _txBuf.size()),
      _currentTransaction(),
      _rxTxTaskHandle(nullptr)
{
    _role = role;
    _extendedSlaveIds = true; // TCP Unit IDs: 1-255
    _tracksOwner = true;

    if (role == Modbus::CLIENT) {
        Modbus::Debug::LOG_MSG("ModbusInterface::TCP (Client) created – connect HAL before calling begin()");
    } else {
        _catchAllSlaveIds = true; // serveur : accepte tout UID
        Modbus::Debug::LOG_MSG("ModbusInterface::TCP (Server) created – make sure HAL is listening before calling begin()");
    }
}

TCP::~TCP() {
    if (_isInitialized) {
        _isInitialized = false;
        killRxTxTask();
    }

    // Wait for in-flight API calls posting into our queues: the next ones see !_isInitialized.
    // Nothing can post into them anymore, so that they can be detached from the set in one go.
    { Lock guard(_txMutex); }      // sendFrame()
    { Lock guard(_controlMutex); } // abortCurrentTransaction()

    // Delete the FreeRTOS objects created in begin()
    beginCleanup();
}

/* @brief Initialize the ModbusTCP interface.
 * @return SUCCESS if the interface has been initialized
 * @note Launches the rxTxTask in a dedicated thread
 */
TCP::Result TCP::begin() {
    if (_isInitialized) return Success();

    if (_tcpHAL.getMode() == ModbusHAL::TCP::CfgMode::UNINIT) {
        return Error(ERR_INIT_FAILED, "HAL not initialized");
    }

    if (_role == Modbus::CLIENT) {
        if (_tcpHAL.getMode() != ModbusHAL::TCP::CfgMode::CLIENT) {
            return Error(ERR_INVALID_ROLE, "wrong HAL type, needs a client");
        }
        if (!_tcpHAL.isClientConnected()) {
            Modbus::Debug::LOG_MSG("HAL not yet connected to remote server");
        } else {
            Modbus::Debug::LOG_MSG("HAL connected to remote server");
        }
    } else { // _role == Modbus::SERVER
        if (_tcpHAL.getMode() != ModbusHAL::TCP::CfgMode::SERVER) {
            return Error(ERR_INVALID_ROLE, "wrong HAL type, needs a server");
        }
        if (!_tcpHAL.isServerRunning()) {
            return Error(ERR_INIT_FAILED, "HAL server not running");
        } else {
            Modbus::Debug::LOG_MSG("HAL server running");
        }
    }

    // Get HAL RX queue
    _rxEventQueue = _tcpHAL.getRxQueueHandle();
    if (!_rxEventQueue) {
        return Error(ERR_INIT_FAILED, "HAL RX queue not available");
    }

    // Create TX request queue (size 1)
    _txRequestQueue = xQueueCreateStatic(1, sizeof(void*), _txRequestQueueStorage, &_txRequestQueueBuffer);
    if (!_txRequestQueue) {
        _rxEventQueue = nullptr;
        return Error(ERR_INIT_FAILED, "failed to create TX queue");
    }

    // Create Txn control queue (size 1)
    _txnControlQueue = xQueueCreateStatic(1, sizeof(void*), _txnControlQueueStorage, &_txnControlQueueBuffer);
    if (!_txnControlQueue) {
        _rxEventQueue = nullptr;
        return Error(ERR_INIT_FAILED, "failed to create Txn control queue");
    }

    // Create QueueSet
    size_t setSize = ModbusHAL::TCP::RX_QUEUE_SIZE + 1 + 1;
    _eventQueueSet = xQueueCreateSet(setSize);
    if (!_eventQueueSet) {
        beginCleanup();
        return Error(ERR_INIT_FAILED, "failed to create QueueSet");
    }

    // Add queues to QueueSet (the HAL adds its RX queue itself, as it owns its producer)
    if (!_tcpHAL.attachRxQueue(_eventQueueSet)) {
        beginCleanup();
        return Error(ERR_INIT_FAILED, "failed to add HAL RX queue to set");
    }
    _rxQueueInSet = true;
    if (xQueueAddToSet(_txRequestQueue, _eventQueueSet) != pdPASS) {
        beginCleanup();
        return Error(ERR_INIT_FAILED, "failed to add TX queue to set");
    }
    _txQueueInSet = true;
    if (xQueueAddToSet(_txnControlQueue, _eventQueueSet) != pdPASS) {
        beginCleanup();
        return Error(ERR_INIT_FAILED, "failed to add Txn control queue to set");
    }
    _controlQueueInSet = true;

    // Clear buffers
    _rxBuffer.clear();
    _txBuffer.clear();

    _isInitialized = true; // Needed for the task not to terminate prematurely

    // Create the RX/TX task
    _rxTxTaskParked = 0;
    _rxTxTaskHandle = xTaskCreateStatic(
        /*pxTaskCode*/      rxTxTask,
        /*pcName*/          "ModbusTCP_RxTxTask",
        /*usStackDepth*/    RXTX_TASK_STACK_SIZE,
        /*pvParameters*/    this,
        /*uxPriority*/      tskIDLE_PRIORITY + 1,
        /*puxStackBuffer*/  _rxTxTaskStack,
        /*xTaskBuffer*/     &_rxTxTaskBuffer
    );

    // Cleanup resources if task creation failed
    if (!_rxTxTaskHandle) {
        _isInitialized = false;
        beginCleanup();
        return Error(ERR_INIT_FAILED, "failed to create poll task");
    }
    
    Modbus::Debug::LOG_MSGF("ModbusTCP Interface ready. Role: %s", (_role == Modbus::CLIENT ? "CLIENT" : "SERVER"));
    
    return Success();
}

/* @brief Send a Modbus frame to the TCP interface.
 * @param frame The frame to send.
 * @param txCallback The callback to call with the result.
 * @param ctx Context pointer passed to callback.
 * @return SUCCESS if the frame has been accepted & sent to the HAL
 * @note This method does not wait for the frame to be sent, it just encodes it
 *       and puts it in the TX buffer. The caller must wait for the rxTxTask
 *       to send a notification with the result of the TX operation.
 */
TCP::Result TCP::sendFrame(const Modbus::Frame &frame, TxResultCallback txCallback, void* ctx) {
    if (!_isInitialized) {
        if (txCallback) txCallback(ERR_NOT_INITIALIZED, ctx);
        return Error(ERR_NOT_INITIALIZED, "TCP interface not initialized");
    }

    bool replacedByException = false;

    // Update TX buffer under critical section (mutex protection)
    {
        Lock guard(_txMutex, 0); // try-lock no wait
        if (!guard.isLocked() || _txBuffer.size() != 0) {
            if (txCallback) txCallback(ERR_BUSY, ctx);
            return Error(ERR_BUSY, "TX already in progress");
        }

        // Checked again under the mutex: fenced by the destructor
        if (!_isInitialized) {
            if (txCallback) txCallback(ERR_NOT_INITIALIZED, ctx);
            return Error(ERR_NOT_INITIALIZED, "TCP interface not initialized");
        }

        if (_role == Modbus::CLIENT && ownedByOther(ctx)) {
            if (txCallback) txCallback(ERR_BUSY, ctx);
            return Error(ERR_BUSY, "transaction owned by another requester");
        }

        // Client: one transaction at a time, whoever sends (it would be sent, then fail to begin)
        if (_role == Modbus::CLIENT && _currentTransaction.active) {
            if (txCallback) txCallback(ERR_BUSY, ctx);
            return Error(ERR_BUSY, "transaction in progress");
        }

        if (_role == Modbus::SERVER && (!_currentTransaction.active || _currentTransaction.socketNum == -1)) {
            if (txCallback) txCallback(ERR_SEND_FAILED, ctx);
            return Error(ERR_SEND_FAILED, "Server not in transaction");
        }

        uint16_t mbapTid = (_role == Modbus::CLIENT) ? getNextOutgoingTransactionId() : _currentTransaction.tid;

        if (ModbusCodec::TCP::encode(frame, _txBuffer, mbapTid) != ModbusCodec::SUCCESS) {
            _txBuffer.clear();
            // Server: a response that can't be encoded is replaced by SLAVE_DEVICE_FAILURE, so the
            // transaction is still answered (it would otherwise stay open until the safety timeout)
            if (_role != Modbus::SERVER || frame.type != Modbus::RESPONSE ||
                !ModbusCodec::TCP::buildException(mbapTid, frame.slaveId, frame.fc, Modbus::SLAVE_DEVICE_FAILURE, _txBuffer)) {
                _txBuffer.clear();
                if (txCallback) txCallback(ERR_INVALID_FRAME, ctx);
                return Error(ERR_INVALID_FRAME, "encoding failed");
            }
            replacedByException = true;
        }

        // Fill TX metadata (the caller is notified of the encoding failure below, not of the exception TX)
        _txCtx.set(mbapTid, 
                   (_role == Modbus::SERVER) ? _currentTransaction.socketNum : -1, 
                   Modbus::isBroadcastId(frame.slaveId), 
                   replacedByException ? nullptr : txCallback,
                   ctx);
        // Aborts from now on cancel this frame if not sent yet (best effort: an earlier abort,
        // e.g. a timeout firing before this call, isn't seen)
        _txCtx.abortCount = _abortCount;
        _txCtx.connId = (_role == Modbus::SERVER) ? _currentTransaction.connId : 0;

        // The requester owns the transaction from now on, until it releases it
        const bool claimed = (_role == Modbus::CLIENT) && claimTransaction(ctx);

        // Signal dans la TX queue (taille 1) pour réveiller rxTxTask
        void* signalPtr = this; // valeur bidon
        if(xQueueSend(_txRequestQueue, &signalPtr, 0) != pdTRUE) {
            _txBuffer.clear();
            _txCtx.clear();
            if (claimed) unclaimTransaction(ctx);
            if (txCallback) txCallback(ERR_BUSY, ctx);
            return Error(ERR_BUSY, "TX queue full");
        }
    } // Mutex released here (end of RAII scope)

    if (replacedByException) {
        if (txCallback) txCallback(ERR_INVALID_FRAME, ctx);
        return Error(ERR_INVALID_FRAME, "encoding failed, replying SLAVE_DEVICE_FAILURE");
    }
    return Success();
}

/* @brief Check if the interface is ready to accept a new outgoing frame.
 * @return True if the interface is ready, false otherwise.
 */
bool TCP::isReady() {
    if (!_isInitialized) return false;

    Lock guard(_txMutex, 0); // try-lock no wait
    if (!guard.isLocked() || _txBuffer.size() > 0) return false;

    // If the transaction is active or owned, the interface is busy ONLY for a client
    // (a server still accepts responses to pending client requests)
    if (_role == Modbus::CLIENT && (_currentTransaction.active || isOwned())) return false;

    return _tcpHAL.isReady();
}

/* @brief Abort current transaction (cleanup hint from client)
 * @note Called when client times out to allow immediate cleanup
 * @note Safe to be called from outside (signals to txnControlQueue, managed by rxTxTask)
 * @note A request not sent yet (e.g. waiting for a reconnection) is dropped if the interface
 *       hasn't started sending it (best effort)
 */
void TCP::abortCurrentTransaction() {
    Lock guard(_controlMutex); // Fenced by the destructor
    if (!_isInitialized || !_txnControlQueue) return;
    _abortCount++;
    // Send a dummy signal, rxTxTask will take care of aborting the txn
    void* dummy = this;
    (void)xQueueSend(_txnControlQueue, &dummy, 0);
}

/* @brief Wait until no TX result callback registered with a context is pending or running
 * @param ctx The context passed to sendFrame()
 * @note The TX slot holds the callback until it returned (see handleTxRequest())
 * @note No wait from the rxTxTask itself (called from a callback)
 */
void TCP::waitTxCallbackDone(void* ctx) {
    if (xTaskGetCurrentTaskHandle() == _rxTxTaskHandle) return;
    while (true) {
        {
            Lock guard(_txMutex);
            if (_txBuffer.size() == 0 || _txCtx.ctx != ctx) return;
        }
        vTaskDelay(1);
    }
}

/* @brief Get the handle of the poll task.
 * @return The handle of the poll task.
 */
TaskHandle_t TCP::getRxTxTaskHandle() {
    return _rxTxTaskHandle;
}

// ===================================================================================
// PRIVATE METHODS
// ===================================================================================


/* @brief Clean up resources initialized in begin()
 * @note Called when the interface is destroyed or begin() fails
 */
void TCP::beginCleanup() {
    // Clean up resources in reverse order of creation
    if (_eventQueueSet) {
        // Queues can only be removed from a set while empty. Nothing posts into ours anymore
        // (task joined, API calls fenced) and the HAL detaches its RX queue under its own mutex:
        // each queue is emptied & removed in one go.
        if (_controlQueueInSet) {
            xQueueReset(_txnControlQueue);
            _controlQueueInSet = (xQueueRemoveFromSet(_txnControlQueue, _eventQueueSet) != pdPASS);
        }
        if (_txQueueInSet) {
            xQueueReset(_txRequestQueue);
            _txQueueInSet = (xQueueRemoveFromSet(_txRequestQueue, _eventQueueSet) != pdPASS);
        }
        if (_rxQueueInSet) _rxQueueInSet = !_tcpHAL.detachRxQueue(_eventQueueSet);
        if (!_controlQueueInSet && !_txQueueInSet && !_rxQueueInSet) {
            vQueueDelete(_eventQueueSet);
        } else {
            // Not expected (a producer bypassed the fences): never delete a set still referenced
            Modbus::Debug::LOG_MSG("Failed to detach queues from QueueSet, leaking it");
        }
        _eventQueueSet = nullptr;
    }
    
    if (_txRequestQueue) {
        vQueueDelete(_txRequestQueue);
        _txRequestQueue = nullptr;
    }

    if (_txnControlQueue) {
        vQueueDelete(_txnControlQueue);
        _txnControlQueue = nullptr;
    }
    
    // Do not delete HAL RX queue as it belongs to _tcpHAL
}

/* @brief Kill the RX/TX task
 * @note Called when the interface is destroyed
 * @note Waits for the task to finish its current work (e.g. an RX assembly or a send),
 *       then deletes it. A static task must not self-delete: its TCB & stack (members
 *       of this object) stay referenced until the idle task cleans it up.
 */
void TCP::killRxTxTask() {
    if (!_rxTxTaskHandle) return;
    if (_txRequestQueue) {
        // Send a dummy signal to wake up the task so it can check _isInitialized
        void* dummy_ptr = this;
        xQueueSend(_txRequestQueue, &dummy_ptr, 0);
    }
    while (!_rxTxTaskParked || eTaskGetState(_rxTxTaskHandle) != eSuspended) {
        vTaskDelay(1);
    }
    vTaskDelete(_rxTxTaskHandle);
    _rxTxTaskHandle = nullptr;
}

/* @brief Process a raw frame received from the HAL
 * @param frameBytes The frame to process
 * @param socketNum The socket number
 * @return SUCCESS if the frame has been decoded & handled, an error otherwise
 * @note This method is called by the rxTxTask exclusively
 */
TCP::Result TCP::processReceivedFrame(const ByteBuffer& frameBytes, int socketNum, uint32_t connId) {
    if (frameBytes.empty()) {
        return Error(ERR_INVALID_FRAME, "empty frame");
    }

    Modbus::Frame decodedFrame;
    Modbus::MsgType msgType = (_role == Modbus::CLIENT) ? Modbus::RESPONSE : Modbus::REQUEST;

    auto decRes = ModbusCodec::TCP::decode(frameBytes, decodedFrame, msgType);
    if (decRes != ModbusCodec::SUCCESS) {
        // Server: a request we can't decode may still call for an exception (e.g. unsupported
        // function code): hand it to the application, which answers it
        // (MBAP validity & minimum size are guaranteed by fetchSocketData())
        if (_role != Modbus::SERVER ||
            !ModbusCodec::makeRejectedRequest(decRes, frameBytes[6], frameBytes[7], decodedFrame)) {
            return Error(ERR_INVALID_FRAME, ModbusCodec::toString(decRes));
        }
        Modbus::Debug::LOG_MSGF("Malformed request (%s), answering exception", ModbusCodec::toString(decRes));
    }

    // Transaction ID = first 2 bytes of MBAP
    uint16_t tid = (static_cast<uint16_t>(frameBytes[0]) << 8) | frameBytes[1];

    return handleDecodedFrame(decodedFrame, socketNum, connId, tid);
}

/* @brief Handle a decoded frame received from the HAL
 * @param frame The decoded frame
 * @param socketNum The socket number
 * @param transactionId The transaction ID
 * @return SUCCESS if the frame has been handled, an error otherwise
 * @note This method is called by processReceivedFrame() exclusively
 */
TCP::Result TCP::handleDecodedFrame(const Modbus::Frame& frame, int socketNum, uint32_t connId, uint16_t transactionId) {

    if (_role == Modbus::CLIENT) {
        // For a client, we expect an active transaction and we end it if everything matches
        if (_currentTransaction.active &&
            _currentTransaction.socketNum == socketNum &&
            _currentTransaction.connId == connId &&
            _currentTransaction.tid == transactionId) {

            // Ready again before notifying: a higher-priority caller woken by the response
            // may send its next request right away
            endTransaction();
            notifyCallbacks(frame);

            Modbus::Debug::LOG_MSGF("Client received response for TID: %d on socket %d", transactionId, socketNum);
            return Success();
        }
        
        return Error(ERR_RX_FAILED, "received unexpected/mismatched TID or socket");
    } 
    
    if (_role == Modbus::SERVER) {
        // Start a new transaction
        if (!beginTransaction(socketNum, connId, transactionId)) {
            // A transaction is already in progress : the server is busy.
            // If the request is not a broadcast, we reply with a "Slave Device Busy" exception.

            bool isBroadcast = Modbus::isBroadcastId(frame.slaveId);

            if (frame.type == Modbus::REQUEST && !isBroadcast) {
                // Ultra-lightweight (9 bytes) exception response using builders defined in the codec layer
                std::array<uint8_t, ModbusCodec::TCP::EXCEPTION_FRAME_SIZE> respArr;
                ByteBuffer respBuf(respArr.data(), respArr.size());
                if (ModbusCodec::TCP::buildException(transactionId,
                                                      frame.slaveId,
                                                      frame.fc,
                                                      Modbus::SLAVE_DEVICE_BUSY,
                                                      respBuf)) {
                    _tcpHAL.sendMsg(respBuf.data(), respBuf.size(), socketNum, nullptr, connId);
                }
                Modbus::Debug::LOG_MSG("Server busy – replied with Slave Device Busy exception");
            }

            // We only signal the busy state to the caller of this function
            return Error(ERR_BUSY, "server already in transaction");
        }

        Modbus::Debug::LOG_MSGF("Server received request with MBAP_TID: %d from socket %d", transactionId, socketNum);

        notifyCallbacks(frame);

        // Handle broadcast requests: if no response is expected, end the transaction
        if (Modbus::isBroadcastId(frame.slaveId) && frame.type == Modbus::REQUEST) {
            Modbus::Debug::LOG_MSG("Server received broadcast request, ending transaction");
            endTransaction();
        }

        return Success();
    }

    return Error(ERR_INVALID_ROLE);
}

/* @brief Fetch data from a socket and put it into a RX buffer
 * @param socketNum The socket number
 * @param rcvBuf The buffer to store the received data
 * @param rcvTid The transaction ID
 * @return SUCCESS if the data has been fetched, an error otherwise
 * @note This method is called by the rxTxTask exclusively
 * @note Reads exactly one frame: the MBAP header first, then the length it announces.
 *       Any following frame stays in the socket and will be signaled again by the HAL.
 * @note If the frame is not complete within RX_ASSEMBLY_TIMEOUT_MS, or the header is
 *       invalid, the connection is closed: the stream position would otherwise be lost.
 */
TCP::Result TCP::fetchSocketData(int socketNum, ByteBuffer& rcvBuf, uint16_t& rcvTid, uint32_t& connId) {
    constexpr size_t MBAP_SIZE = ModbusCodec::TCP::MBAP_SIZE;
    constexpr size_t MIN_FRAME_SIZE = ModbusCodec::TCP::MIN_FRAME_SIZE;
    constexpr size_t MAX_FRAME_SIZE = ModbusCodec::TCP::MAX_FRAME_SIZE;

    if (rcvBuf.capacity() < MAX_FRAME_SIZE) {
        return Error(ERR_RX_FAILED, "buffer too small");
    }

    rcvBuf.clear();

    uint32_t t0 = TIME_MS();
    size_t frameLen = MBAP_SIZE; // Header first, updated once the header is complete
    connId = 0;

    while (true) {
        size_t currentSize = rcvBuf.size();
        rcvBuf.resize(frameLen); // Pre-reserve exactly what is missing
        // The whole frame must come from the same connection (the socket number may be reused)
        uint32_t readConnId = 0;
        size_t n = _tcpHAL.readSocketData(socketNum, rcvBuf.begin() + currentSize, frameLen - currentSize,
                                          connId, &readConnId);
        if (n != SIZE_MAX) connId = readConnId;

        if (n == SIZE_MAX) {
            rcvBuf.clear();
            return Error(ERR_RX_FAILED, "socket error/closed, aborting read");
        }

        rcvBuf.trim(currentSize + n);

        if (n == 0) {
            // No data on the first read: the data for this event has already been consumed
            if (currentSize == 0) {
                return Error(NODATA, "event data already consumed");
            }
            if (TIME_MS() - t0 >= RX_ASSEMBLY_TIMEOUT_MS) {
                rcvBuf.clear();
                _tcpHAL.closeConnection(socketNum, connId);
                return Error(ERR_RX_FAILED, "incomplete frame, connection closed");
            }
            vTaskDelay(1); // Wait for the rest of the frame (1 tick: WAIT_MS(1) is 0 tick below 1 kHz)
            continue;
        }

        if (rcvBuf.size() < frameLen) continue;

        if (frameLen == MBAP_SIZE) {
            // Header complete: check it and read exactly the announced length
            uint16_t protocolId = (rcvBuf[2] << 8) | rcvBuf[3];
            uint16_t mbapLen = (rcvBuf[4] << 8) | rcvBuf[5];
            frameLen = 6 + mbapLen; // Expected total size (MBAP+PDU)
            if (protocolId != 0 || frameLen < MIN_FRAME_SIZE || frameLen > MAX_FRAME_SIZE) {
                rcvBuf.clear();
                _tcpHAL.closeConnection(socketNum, connId);
                return Error(ERR_RX_FAILED, "invalid MBAP header, connection closed");
            }
            continue;
        }

        rcvTid = (static_cast<uint16_t>(rcvBuf[0]) << 8) | rcvBuf[1];
        return Success();
    }
}

/* @brief Handle a TX request signalled via _txRequestQueue
 * Consumes _txBuffer + _txCtx and sends the payload through HAL.
 * Returns the Result that will be notified to the waiting task (SUCCESS, ERR_SEND_FAILED, ...)
 */
TCP::Result TCP::handleTxRequest() {
    TxResultCallback txCallback = nullptr;
    void* ctx = nullptr;

    int actualSock = -1;
    uint32_t actualConnId = 0;
    uint16_t tid = 0;
    bool isBroadcast = false;
    bool sendMsgRes = false;
    bool aborted = false;

    // The TX slot stays busy until the TX callback returned (called outside the mutex): otherwise
    // another request could be queued meanwhile, and this result be applied to it
    struct TxSlotRelease {
        TCP* tcp;
        ~TxSlotRelease() { Lock guard(tcp->_txMutex); tcp->_txBuffer.clear(); tcp->_txCtx.clear(); }
    } txSlotRelease{this};

    // Access TX buffer & metadata under critical section (mutex protection)
    { 
        Lock guard(_txMutex);

        // Build the message to send
        const uint8_t* payload = _txBuffer.data();
        size_t len = _txBuffer.size();
        int destSock = _txCtx.destSock;

        // Fetch metadata for processing TX result outside of this scope
        txCallback = _txCtx.txCallback;
        ctx = _txCtx.ctx;
        tid = _txCtx.tid;
        isBroadcast = _txCtx.isBroadcast;

        if (len == 0) {
            // No data to send – this should not happen
            _txBuffer.clear();
            _txCtx.clear();
            // Notify outside mutex
            if (txCallback) {
                txCallback(ERR_SEND_FAILED, ctx);
            }
            return Error(ERR_SEND_FAILED, "no data to send");
        }

        // Client: (re)connect first, as it may take long. If the request was aborted
        // meanwhile (e.g. Client timeout), it must not be sent late: the send is bound to
        // that connection, so that it can't reconnect again after this check.
        uint32_t connId = _txCtx.connId;
        bool connected = (_role != Modbus::CLIENT) || _tcpHAL.ensureClientConnected(&connId);
        aborted = (_role == Modbus::CLIENT) && (_abortCount != _txCtx.abortCount);

        // Send the payload via HAL
        if (connected && !aborted) {
            sendMsgRes = _tcpHAL.sendMsg(payload, len, destSock,
                                      (_role == Modbus::CLIENT) ? &actualSock : nullptr,
                                      connId, &actualConnId);
        }
    } // Mutex released here (end of RAII scope)

    if (!sendMsgRes) {
        // For server role, we must end the transaction even if send fails
        // to avoid getting stuck in "busy" state
        if (_role == Modbus::SERVER) {
            endTransaction();
        }
        if (txCallback) txCallback(ERR_SEND_FAILED, ctx);
        if (aborted) return Error(ERR_SEND_FAILED, "request aborted before sending");
        return Error(ERR_SEND_FAILED, "HAL returned failure to send message");
    }

    if (_role == Modbus::CLIENT && !isBroadcast) {
        // Check if HAL returned a valid socket
        if (actualSock == -1) {
            if (txCallback) txCallback(ERR_SEND_FAILED, ctx);
            return Error(ERR_SEND_FAILED, "invalid current socket returned by HAL");
        }
        // Start the transaction
        if (!beginTransaction(actualSock, actualConnId, tid)) {
            if (txCallback) txCallback(ERR_BUSY, ctx);
            return Error(ERR_BUSY, "cannot start transaction");
        }
    } else {
        // Server replies or broadcasts: transaction ends immediately after send
        endTransaction();
    }

    if (txCallback) txCallback(SUCCESS, ctx);
    return Success();
}


/* @brief Get the next outgoing transaction ID
 * @return The next outgoing transaction ID
 * @note Called under _txMutex. The counter lives outside _currentTransaction so that
 *       ending a transaction (success, error or abort) never rewinds it: a late
 *       response to a timed-out request can't match the TID of the next one.
 */
uint16_t TCP::getNextOutgoingTransactionId() {
    // For client, increment transaction ID. Wraps around, skipping 0.
    if (_lastOutgoingTid == 0xFFFF) _lastOutgoingTid = 0;
    return ++_lastOutgoingTid;
}

/* @brief Mark the _currentTransaction.active flag to true at the beginning of a Modbus transaction.
 * @param socketNum The socket number associated with this transaction.
 * @param connId The HAL connection ID associated with this transaction.
 * @param transactionId The transaction ID for this transaction.
 * @return True if the transaction was started, false otherwise (e.g., mutex lock failed, already in transaction for a client).
 * @note IMPORTANT: must ONLY be called from rxTxTask context!
 */
bool TCP::beginTransaction(int socketNum, uint32_t connId, uint16_t transactionId) {
    Lock guard(_txMutex); // sendFrame() reads the transaction under it
    if (_currentTransaction.active) {
        Modbus::Debug::LOG_MSG("Transaction already in progress");
        return false;
    }
    _currentTransaction.set(socketNum, connId, transactionId);
    Modbus::Debug::LOG_MSGF("Transaction started on socket: %d with TID: %d", socketNum, transactionId);
    return true;
}

/* @brief Mark the _currentTransaction.active flag to false at the end of a Modbus transaction.
 * @note IMPORTANT: must ONLY be called from rxTxTask context!
 */
void TCP::endTransaction() {
    Lock guard(_txMutex); // sendFrame() reads the transaction under it
    if (_currentTransaction.active) {
        Modbus::Debug::LOG_MSGF("Transaction ended on socket: %d with TID: %d", _currentTransaction.socketNum, _currentTransaction.tid);
        _currentTransaction.clear();
    }
}

/* @brief Main task for the RX/TX operations
 * @brief - Waits for the next event in the queues (TCP HAL event or TX request)
 * @brief - Calls handleTxRequest() or processReceivedFrame() accordingly
 * @brief - Kills itself if _isInitialized becomes false
 * @param tcp: The pointer to the TCP interface
 * @note This task is started in begin()
 */
void TCP::rxTxTask(void* tcp) {
    TCP* self = static_cast<TCP*>(tcp);
    bool cleanupTxnFlag = false; // For Txn control queue

    while (self->_isInitialized) {
        // Select one of the queues (TX or RX) or timeout 100 ms
        QueueSetMemberHandle_t member = xQueueSelectFromSet(self->_eventQueueSet, pdMS_TO_TICKS(RXTX_QUEUE_CHECK_TIMEOUT_MS));

        if (!self->_isInitialized) { break; }

        // On a select() timeout (member == nullptr)
        // we fall through to the transaction-timeout 
        // check (emergency cleanup)
        if (member != nullptr) {
            if (member == self->_txRequestQueue) {
                // ----------- TX path -----------
                void* dummy;
                xQueueReceive(self->_txRequestQueue, &dummy, 0); // clear the signal

                // Check again if still initialized (could be a cleanup signal)
                if (!self->_isInitialized) { break; }

                self->handleTxRequest(); // Result is internally traced and task notified
            } else if (member == self->_rxEventQueue) {
                // ----------- RX path -----------
                int sock;
                if (xQueueReceive(self->_rxEventQueue, &sock, 0) == pdTRUE) {
                    uint16_t rcvTid;
                    uint32_t connId;
                    Result fetchRes = self->fetchSocketData(sock, self->_rxBuffer, rcvTid, connId);
                    if (fetchRes == SUCCESS) {
                        ByteBuffer frameView(self->_rxBuffer.data(), self->_rxBuffer.size());
                        self->processReceivedFrame(frameView, sock, connId);
                        self->_rxBuffer.clear(); // reset for next iteration
                    }
                }
            } else if (member == self->_txnControlQueue) {
                void* dummy;
                if (xQueueReceive(self->_txnControlQueue, &dummy, 0) == pdTRUE) {
                    cleanupTxnFlag = true;
                    // Txn will be closed in the transaction-timeout block below
                    // if it exists, otherwise this request is ignored
                }
            }
        }

        // ----------- Transaction timeout -----------
        if (self->_currentTransaction.active) {
            uint32_t now = TIME_MS();
            if (now - self->_currentTransaction.startMs > TCP_TRANSACTION_SAFETY_TIMEOUT_MS || cleanupTxnFlag) {
                // A response waiting to be sent ends its transaction itself (sent or not): until
                // then, another request must not take its place. Checked & closed under the lock
                // sendFrame() queues it with.
                Lock guard(self->_txMutex);
                if (cleanupTxnFlag || self->_txBuffer.size() == 0) {
                    Modbus::Debug::LOG_MSGF("Transaction %s on socket %d TID: %d",
                        cleanupTxnFlag ? "aborted" : "timeout",
                        self->_currentTransaction.socketNum, self->_currentTransaction.tid);
                    self->_currentTransaction.clear();
                }
            }
        }
        cleanupTxnFlag = false; // Always reset flag here (avoids killing the next active Txn)

        if (!self->_isInitialized) { break; }
    }

    Modbus::Debug::LOG_MSGF("ModbusTCP RxTx Task stopping for %s", (self->_role == Modbus::CLIENT ? "CLIENT" : "SERVER"));
    self->_rxTxTaskParked = 1; // Last access to the object
    vTaskSuspend(nullptr);     // Deleted by killRxTxTask()
}

} // namespace ModbusInterface 