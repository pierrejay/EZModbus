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

    // Create QueueSet
    size_t setSize = 1 + ModbusHAL::TCP::RX_QUEUE_SIZE;
    _eventQueueSet = xQueueCreateSet(setSize);
    if (!_eventQueueSet) {
        beginCleanup();
        return Error(ERR_INIT_FAILED, "failed to create QueueSet");
    }

    // Add queues to QueueSet
    if (xQueueAddToSet(_txRequestQueue, _eventQueueSet) != pdPASS) {
        beginCleanup();
        return Error(ERR_INIT_FAILED, "failed to add TX queue to set");
    }
    if (xQueueAddToSet(_rxEventQueue, _eventQueueSet) != pdPASS) {
        beginCleanup();
        return Error(ERR_INIT_FAILED, "failed to add HAL RX queue to set");
    }

    // Clear buffers
    _rxBuffer.clear();
    _txBuffer.clear();

    _isInitialized = true; // Needed for the task not to terminate prematurely

    // Create the RX/TX task
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

    // Update TX buffer under critical section (mutex protection)
    {
        Lock guard(_txMutex, 0); // try-lock no wait
        if (!guard.isLocked() || _txBuffer.size() != 0) {
            if (txCallback) txCallback(ERR_BUSY, ctx);
            return Error(ERR_BUSY, "TX already in progress");
        }

        if (_role == Modbus::SERVER && (!_currentTransaction.active || _currentTransaction.socketNum == -1)) {
            if (txCallback) txCallback(ERR_SEND_FAILED, ctx);
            return Error(ERR_SEND_FAILED, "Server not in transaction");
        }

        uint16_t mbapTid = (_role == Modbus::CLIENT) ? getNextOutgoingTransactionId() : _currentTransaction.tid;

        if (ModbusCodec::TCP::encode(frame, _txBuffer, mbapTid) != ModbusCodec::SUCCESS) {
            if (txCallback) txCallback(ERR_INVALID_FRAME, ctx);
            _txBuffer.clear();
            return Error(ERR_INVALID_FRAME, "encoding failed");
        }

        // Fill TX metadata
        _txCtx.set(mbapTid, 
                   (_role == Modbus::SERVER) ? _currentTransaction.socketNum : -1, 
                   Modbus::isBroadcastId(frame.slaveId), 
                   txCallback, 
                   ctx);

        // Signal dans la TX queue (taille 1) pour réveiller rxTxTask
        void* signalPtr = this; // valeur bidon
        if(xQueueSend(_txRequestQueue, &signalPtr, 0) != pdTRUE) {
            _txBuffer.clear();
            _txCtx.clear();
            if (txCallback) txCallback(ERR_BUSY, ctx);
            return Error(ERR_BUSY, "TX queue full");
        }
    } // Mutex released here (end of RAII scope)

    return Success();
}

/* @brief Check if the interface is ready to accept a new outgoing frame.
 * @return True if the interface is ready, false otherwise.
 */
bool TCP::isReady() {
    if (!_isInitialized) return false;

    Lock guard(_txMutex, 0); // try-lock no wait
    if (!guard.isLocked() || _txBuffer.size() > 0) return false;

    // If the transaction is active, the interface is busy ONLY for a client
    // (a server still accepts responses to pending client requests)
    if (_role == Modbus::CLIENT && _currentTransaction.active) return false;

    return _tcpHAL.isReady();
}

/* @brief Abort current transaction (cleanup hint from client)
 * @note Called when client times out to allow immediate cleanup
 */
void TCP::abortCurrentTransaction() {
    Lock guard(_transactionMutex, 0); // Try-lock no wait
    if (guard.isLocked() && _currentTransaction.active) {
        Modbus::Debug::LOG_MSG("Client timeout - aborting TCP transaction");
        endTransaction();
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
        if (_rxEventQueue) xQueueRemoveFromSet(_rxEventQueue, _eventQueueSet);
        if (_txRequestQueue) xQueueRemoveFromSet(_txRequestQueue, _eventQueueSet);
        vQueueDelete(_eventQueueSet);
        _eventQueueSet = nullptr;
    }
    
    if (_txRequestQueue) {
        vQueueDelete(_txRequestQueue);
        _txRequestQueue = nullptr;
    }
    
    // Do not delete HAL RX queue as it belongs to _tcpHAL
}

/* @brief Kill the RX/TX task
 * @note Called when the interface is destroyed
 */
void TCP::killRxTxTask() {
    if (_rxTxTaskHandle && _txRequestQueue) {
        // Send a dummy signal to wake up the task so it can check _isInitialized
        void* dummy_ptr = this;
        xQueueSend(_txRequestQueue, &dummy_ptr, 0);
        vTaskDelay(pdMS_TO_TICKS(RXTX_QUEUE_CHECK_TIMEOUT_MS + 50)); // Wait a bit more than task queue check timeout
    }
}

/* @brief Process a raw frame received from the HAL
 * @param frameBytes The frame to process
 * @param socketNum The socket number
 * @return SUCCESS if the frame has been decoded & handled, an error otherwise
 * @note This method is called by the rxTxTask exclusively
 */
TCP::Result TCP::processReceivedFrame(const ByteBuffer& frameBytes, int socketNum) {
    if (frameBytes.empty()) {
        return Error(ERR_INVALID_FRAME, "empty frame");
    }

    Modbus::Frame decodedFrame;
    Modbus::MsgType msgType = (_role == Modbus::CLIENT) ? Modbus::RESPONSE : Modbus::REQUEST;

    auto decRes = ModbusCodec::TCP::decode(frameBytes, decodedFrame, msgType);
    if (decRes != ModbusCodec::SUCCESS) {
        return Error(ERR_INVALID_FRAME, ModbusCodec::toString(decRes));
    }

    // Transaction ID = first 2 bytes of MBAP
    uint16_t tid = (static_cast<uint16_t>(frameBytes[0]) << 8) | frameBytes[1];

    return handleDecodedFrame(decodedFrame, socketNum, tid);
}

/* @brief Handle a decoded frame received from the HAL
 * @param frame The decoded frame
 * @param socketNum The socket number
 * @param transactionId The transaction ID
 * @return SUCCESS if the frame has been handled, an error otherwise
 * @note This method is called by processReceivedFrame() exclusively
 */
TCP::Result TCP::handleDecodedFrame(const Modbus::Frame& frame, int socketNum, uint16_t transactionId) {

    if (_role == Modbus::CLIENT) {
        // For a client, we expect an active transaction and we end it if everything matches
        if (_currentTransaction.active &&
            _currentTransaction.socketNum == socketNum &&
            _currentTransaction.tid == transactionId) {

            notifyCallbacks(frame);
            endTransaction();

            Modbus::Debug::LOG_MSGF("Client received response for TID: %d on socket %d", transactionId, socketNum);
            return Success();
        }
        
        return Error(ERR_RX_FAILED, "received unexpected/mismatched TID or socket");
    } 
    
    if (_role == Modbus::SERVER) {
        // Start a new transaction
        if (!beginTransaction(socketNum, transactionId)) {
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
                    _tcpHAL.sendMsg(respBuf.data(), respBuf.size(), socketNum);
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
 * @note It will loop until a "looking-valid" frame is assembled, or a timeout occurs
 */
TCP::Result TCP::fetchSocketData(int socketNum, ByteBuffer& rcvBuf, uint16_t& rcvTid) {
    if (rcvBuf.capacity() < ModbusCodec::TCP::MAX_FRAME_SIZE) {
        return Error(ERR_RX_FAILED, "buffer too small");
    }

    rcvBuf.clear();

    uint32_t t0 = TIME_MS();
    bool firstRead = true; // First read of the event

    while (TIME_MS() - t0 < RX_ASSEMBLY_TIMEOUT_MS) {
        // Taille libre dans le buffer d'assemblage
        size_t freeSpace = rcvBuf.free_space();
        if (freeSpace == 0) {
            rcvBuf.clear();
            return Error(ERR_RX_FAILED, "buffer overflow, flushing RX buffer");
        }

        size_t toRead = freeSpace;
        uint8_t* writePtr = rcvBuf.end();
        size_t currentSize = rcvBuf.size();

        // Pre-reserve the space
        rcvBuf.resize(currentSize + toRead);

        size_t n = _tcpHAL.readSocketData(socketNum, writePtr, toRead);

        if (n == SIZE_MAX) {
            rcvBuf.trim(currentSize); // Revert to previous size
            return Error(ERR_RX_FAILED, "socket error/closed, aborting read");
        }

        if (n > 0) {
            rcvBuf.trim(currentSize + n);
        } else if (n == 0) {
            // No bytes received, revert to previous size to avoid growing indefinitely
            rcvBuf.trim(currentSize);
            
            // If we get no data on the first read, we have already consumed the data for this event
            if (firstRead) {
                return Error(NODATA, "event data already consumed");
            }
        }
        
        firstRead = false;

        // Try to decode if we have at least the MBAP header (7 bytes)
        if (rcvBuf.size() >= 7) {
            uint16_t mbapLen = (rcvBuf[4] << 8) | rcvBuf[5];
            size_t frameLen = 6 + mbapLen; // Expected total size (MBAP+PDU)

            // If we don't have the whole frame, break the loop to read again
            if (rcvBuf.size() < frameLen) break;

            // We have the full frame in rcvBuf, resize it exactly to frameLen
            rcvBuf.trim(frameLen);

            rcvTid = (static_cast<uint16_t>(rcvBuf[0]) << 8) | rcvBuf[1];

            return Success();
        }

        if (n == 0) {
            WAIT_MS(1); // Wait for more data until timeout or failure
        }
    }

    if (rcvBuf.size() > 0) {
        rcvBuf.clear();
        return Error(ERR_RX_FAILED, "assembly timeout, dropping data");
    }
    else {
        return Error(NODATA, "assembly timeout w/o data");
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
    uint16_t tid = 0;
    bool isBroadcast = false;
    bool sendMsgRes = false;

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

        // Send the payload via HAL
        sendMsgRes = _tcpHAL.sendMsg(payload, len, destSock,
                                  (_role == Modbus::CLIENT) ? &actualSock : nullptr);

        // Clear TX buffer and metadata whatever happens
        _txBuffer.clear();
        _txCtx.clear();
    } // Mutex released here (end of RAII scope)

    if (!sendMsgRes) {
        // For server role, we must end the transaction even if send fails
        // to avoid getting stuck in "busy" state
        if (_role == Modbus::SERVER) {
            endTransaction();
        }
        if (txCallback) txCallback(ERR_SEND_FAILED, ctx);
        return Error(ERR_SEND_FAILED, "HAL returned failure to send message");
    }

    if (_role == Modbus::CLIENT && !isBroadcast) {
        // Check if HAL returned a valid socket
        if (actualSock == -1) {
            if (txCallback) txCallback(ERR_SEND_FAILED, ctx);
            return Error(ERR_SEND_FAILED, "invalid current socket returned by HAL");
        }
        // Start the transaction
        if (!beginTransaction(actualSock, tid)) {
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
 */
uint16_t TCP::getNextOutgoingTransactionId() {
    // For client, increment transaction ID. Wraps around. Starts from tid = 0.
    if (_currentTransaction.tid == 0xFFFF) _currentTransaction.tid = 0;
    _currentTransaction.tid++; 
    return _currentTransaction.tid;
}

/* @brief Mark the _currentTransaction.active flag to true at the beginning of a Modbus transaction.
 * @param socketNum The socket number associated with this transaction.
 * @param transactionId The transaction ID for this transaction.
 * @return True if the transaction was started, false otherwise (e.g., mutex lock failed, already in transaction for a client).
 */
bool TCP::beginTransaction(int socketNum, uint16_t transactionId) {
    Lock guard(_transactionMutex, 0);  // Try-lock
    if (!guard.isLocked() || _currentTransaction.active) {
        Modbus::Debug::LOG_MSG("transaction already in progress");
        return false;
    }

    _currentTransaction.set(socketNum, transactionId);
    Modbus::Debug::LOG_MSGF("Transaction started on socket: %d with TID: %d", socketNum, transactionId);
    return true;
}

/* @brief Mark the _currentTransaction.active flag to false at the end of a Modbus transaction.
 */
void TCP::endTransaction() {
    Lock guard(_transactionMutex); // Wait for lock
    if (_currentTransaction.active) {
        Modbus::Debug::LOG_MSGF("Transaction ended on socket: %d with TID: %d", _currentTransaction.socketNum, _currentTransaction.tid);
        _currentTransaction.clear();
    } else {
        // Modbus::Debug::LOG_MSG("endTransaction called but not in transaction.");
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

    while (self->_isInitialized) {
        // Select one of the queues (TX or RX) or timeout 100 ms
        QueueSetMemberHandle_t member = xQueueSelectFromSet(self->_eventQueueSet, pdMS_TO_TICKS(RXTX_QUEUE_CHECK_TIMEOUT_MS));

        // Get out of the loop if the task is not initialized, otherwise wait for the next event
        if (member == nullptr) {
            if (!self->_isInitialized) { break; }
            continue;
        }

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
                Result fetchRes = self->fetchSocketData(sock, self->_rxBuffer, rcvTid);
                if (fetchRes == SUCCESS) {
                    ByteBuffer frameView(self->_rxBuffer.data(), self->_rxBuffer.size());
                    self->processReceivedFrame(frameView, sock);
                    self->_rxBuffer.clear(); // reset for next iteration
                }
            }
        }

        // ----------- Transaction timeout -----------
        if (self->_currentTransaction.active) {
            uint32_t now = TIME_MS();
            if (now - self->_currentTransaction.startMs > TCP_TRANSACTION_SAFETY_TIMEOUT_MS) {
                Modbus::Debug::LOG_MSGF("Transaction timeout on socket %d TID: %d", self->_currentTransaction.socketNum, self->_currentTransaction.tid);
                self->endTransaction();
            }
        }

        if (!self->_isInitialized) { break; }
    }

    Modbus::Debug::LOG_MSGF("ModbusTCP RxTx Task stopping for %s", (self->_role == Modbus::CLIENT ? "CLIENT" : "SERVER"));
    self->_rxTxTaskHandle = nullptr;
    vTaskDelete(nullptr);
}

} // namespace ModbusInterface 