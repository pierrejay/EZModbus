/* @file ModbusRTU.cpp
 * @brief Implementation of the ModbusRTU class
 */

#include "ModbusRTU.h"

namespace ModbusInterface {

// ===================================================================================
// PUBLIC METHODS
// ===================================================================================

RTU::RTU(ModbusHAL::UART& uart, Modbus::Role role)
    : _uartHAL(uart),
      _rxBuffer(_rxBuf.data(), _rxBuf.size()),
      _txBuffer(_txBuf.data(), _txBuf.size()),
      _txMutex(),
      _txResultCallback(nullptr),
      _txCallbackCtx(nullptr),
      _rxEventQueue(nullptr),
      _rxTxTaskHandle(nullptr)
{
    this->_role = role; 
}

RTU::~RTU() {
    if (_isInitialized) {
        _isInitialized = false; 
        killRxTxTask();
    }

    // Wait for an in-flight sendFrame() (the next ones see !_isInitialized)
    { Lock guard(_txMutex); }

    // Delete the FreeRTOS objects created in begin()
    beginCleanup();
}

/* @brief Initialize the RTU interface
 * @return SUCCESS if the interface has been initialized
 * @note Launches the rxTxTask in a dedicated thread
 */
RTU::Result RTU::begin() {
    if (_isInitialized) return Success();

    // Get UART event queue from HAL
    _rxEventQueue = _uartHAL.getRegisteredEventQueue();
    if (!_rxEventQueue) {
        return Error(ERR_INIT_FAILED, "UART HAL not ready or void UART event queue");
    }

    // Flush parasitic UART events that occurred before RTU initialization
    uart_event_t dump;
    while (xQueueReceive(_rxEventQueue, &dump, 0) == pdTRUE) { }

    // Flush the buffers prior to starting the task
    _rxBuffer.clear();
    _uartHAL.flush_input();

    _isInitialized = true; // Needed for the task not to terminate prematurely

    // Create the RX/TX task
    _rxTxTaskParked = 0;
    _rxTxTaskHandle = xTaskCreateStatic(
        /*pxTaskCode*/      rxTxTask,
        /*pcName*/          "ModbusRTU_RxTxTask", 
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
        return Error(ERR_INIT_FAILED, "failed to create rxTxTask");
    }
    
    // Configure UART idle detection (silence time)
    Result resIdleCfg = ERR_CONFIG_FAILED;
    if (_silenceTimeUs > 0) {
        // If the silence time was already set by the user, use it
        resIdleCfg = updateUartIdleDetection();
    } else {
        // Otherwise, set the silence time based on the baud rate
        resIdleCfg = setSilenceTimeBaud(); 
    }
    if (resIdleCfg != SUCCESS) {
        _isInitialized = false;
        killRxTxTask();
        beginCleanup();
        return Error(ERR_INIT_FAILED, "failed to configure UART idle detection");
    }

    Modbus::Debug::LOG_MSGF("Interface ready. UART Port: %d, silence time: %u us. Call setSilenceTimeMs to override", _uartHAL.getPort(), (uint32_t)_silenceTimeUs);
    
    return Success();
}

/* @brief Set an arbitrary silence time in milliseconds
 * @param silenceTimeMs: The silence time to set
 * @return SUCCESS if the silence time has been set
 * @note Minimum silenceTimeMs is 1 ms
 * @note If the interface isn't initialized, the silence time will be applied when begin() is called
 */
RTU::Result RTU::setSilenceTimeMs(uint32_t silenceTimeMs) {
    // Calculate the new silence time in microseconds
    if (silenceTimeMs < 1) silenceTimeMs = 1; 
    uint64_t newSilenceTimeUs = static_cast<uint64_t>(silenceTimeMs) * 1000;
    
    // Apply the new silence time only if it's different from the current one
    if (newSilenceTimeUs != _silenceTimeUs) {
        _silenceTimeUs = newSilenceTimeUs;
        // If the interface isn't yet initialized, the silence time will be applied when begin() is called
        if (_isInitialized) {
            Result resIdleCfg = updateUartIdleDetection();
            if (resIdleCfg != SUCCESS) {
                return Error(ERR_CONFIG_FAILED, "failed to apply silence time to UART HAL");
            }
        }
    }

    Modbus::Debug::LOG_MSGF("Silence time set to %d ms (%d us)", silenceTimeMs, _silenceTimeUs);
    return Success();
}

/* @brief Set the silence time based on a baud rate
 * @param baudRateFromUser: The baud rate to use for the calculation
 * @return SUCCESS if the silence time has been set
 * @note Uses 3.5T as specified in the Modbus standard + 200us margin
 * @note If the interface isn't initialized, the silence time will be applied when begin() is called
 */
RTU::Result RTU::setSilenceTimeBaud() {
    // Get the baud rate from the HAL
    uint32_t baudRate = _uartHAL.getBaudrate();
    if (baudRate == 0) {
        return Error(ERR_CONFIG_FAILED, "HAL returned an invalid baud rate"); 
    }
    
    // Calculate the silence time based on the baud rate
    // Modbus RTU specification: 3.5 character times for frame separation
    // For baud rates > 19200, use fixed 1.75 ms per specification
    uint64_t newSilenceTimeUs;
    if (baudRate > 19200) {
        newSilenceTimeUs = 1750; // 1.75 ms
    } else {
        constexpr uint32_t bitsPerChar = 11;      // 1 start + 8 data + parity + stop
        uint64_t charTimeUs = (static_cast<uint64_t>(bitsPerChar) * 1000000ULL) / baudRate;
        newSilenceTimeUs = (charTimeUs * 35ULL) / 10ULL; // 3.5 chars = 35/10
    }

    // Apply the new silence time only if it's different from the current one
    if (newSilenceTimeUs != _silenceTimeUs) {
        _silenceTimeUs = newSilenceTimeUs;
        // If the interface isn't yet initialized, the silence time will be applied when begin() is called
        if (_isInitialized) {
            Result resIdleCfg = updateUartIdleDetection();
            if (resIdleCfg != SUCCESS) {
                return Error(ERR_CONFIG_FAILED, "failed to apply silence time to UART HAL");
            }
        }
    }

    Modbus::Debug::LOG_MSGF("Silence time for baud rate %d bps set to %d us", baudRate, _silenceTimeUs);
    return Success();
}

/* @brief Send a Modbus frame to the UART interface
 * @param frame: The frame to send
 * @param txCallback: The callback to call with the result of the TX operation
 * @param ctx: Context pointer passed to callback
 * @return Success if the frame has been accepted & sent to the UART TX queue
 * @note This method does not wait for the frame to be sent, it just encodes it
 *       and puts it in the TX buffer. The caller must wait for the rxTxTask
 *       to send a notification with the result of the TX operation.
 */
RTU::Result RTU::sendFrame(const Modbus::Frame& frame, TxResultCallback txCallback, void* ctx) {
    if (!_isInitialized) {
        if (txCallback) txCallback(ERR_NOT_INITIALIZED, ctx);
        return Error(ERR_NOT_INITIALIZED, "RTU interface not initialized");
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
            return Error(ERR_NOT_INITIALIZED, "RTU interface not initialized");
        }
        
        _txResultCallback = txCallback; // Store the callback for rxTxTask
        _txCallbackCtx = ctx; // Store the context for rxTxTask
        if (_role == Modbus::MASTER && frame.type == Modbus::REQUEST) _rtt.store(); // Store the start time for master request

        // _txBuffer is considered free. Encode the frame.
        auto encodeResult = ModbusCodec::RTU::encode(frame, _txBuffer);
        if (encodeResult != ModbusCodec::SUCCESS) {
            _txBuffer.clear(); // Don't leave the buffer in an invalid state
            // Slave: a response that can't be encoded is replaced by SLAVE_DEVICE_FAILURE,
            // so that the request is still answered
            if (_role != Modbus::SLAVE || frame.type != Modbus::RESPONSE || Modbus::isBroadcastId(frame.slaveId) ||
                !ModbusCodec::RTU::buildException(frame.slaveId, frame.fc, Modbus::SLAVE_DEVICE_FAILURE, _txBuffer)) {
                _txBuffer.clear();
                _txResultCallback = nullptr;
                _txCallbackCtx = nullptr;
                if (txCallback) txCallback(ERR_INVALID_FRAME, ctx);
                return Error(ERR_INVALID_FRAME, "encoding failed");
            }
            replacedByException = true;
            _txResultCallback = nullptr; // The caller is notified of the encoding failure below, not of the exception TX
            _txCallbackCtx = nullptr;
        }

        Modbus::Debug::LOG_MSGF("Encoded TX frame (%u bytes) for port %d", (uint32_t)_txBuffer.size(), (int)_uartHAL.getPort());
        Modbus::Debug::LOG_HEXDUMP(_txBuffer);

        if (_role == Modbus::MASTER && frame.type == Modbus::REQUEST) {
            _rtt.start(&_rtt.storeUs); // Start RTT for master request (using stored start time)
        }

        // Signal to rxTxTask that a frame is ready in _txBuffer
        if (!postTaskEvent(TASK_TX_REQUEST)) {
            _txBuffer.clear();
            _txResultCallback = nullptr;
            _txCallbackCtx = nullptr;
            if (txCallback) txCallback(ERR_BUSY, ctx);
            return Error(ERR_BUSY, "UART event queue full, cannot signal the TX request");
        }
    } // Mutex released here (end of RAII scope)
    
    if (replacedByException) {
        if (txCallback) txCallback(ERR_INVALID_FRAME, ctx);
        return Error(ERR_INVALID_FRAME, "encoding failed, replying SLAVE_DEVICE_FAILURE");
    }
    return Success(); 
}

/* @brief Check if the interface is ready
 * @return True if the interface is ready to accept a new outgoing frame
 */
bool RTU::isReady() {
    if (!_isInitialized) return false;
    Lock guard(_txMutex, 0); // try-lock no wait
    if (!guard.isLocked() || _txBuffer.size() > 0) return false;
    return true;
}

/* @brief Get the handle of the RX/TX task
 * @return The handle of the RX/TX task
 */
TaskHandle_t RTU::getRxTxTaskHandle() {
    return _rxTxTaskHandle;
}

// ===================================================================================
// PRIVATE METHODS
// ===================================================================================

/* @brief Clean up resources initialized in begin()
 * @note Called when the interface is destroyed
 */
void RTU::beginCleanup() {
    // Nothing to delete: _rxEventQueue belongs to _uartHAL
    _rxEventQueue = nullptr;
}

/* @brief Kill the RX/TX task
 * @note Called when the interface is destroyed
 * @note Waits for the task to finish its current work (e.g. a transmission), then deletes
 *       it. A static task must not self-delete: its TCB & stack (members of this object)
 *       stay referenced until the idle task cleans it up.
 */
void RTU::killRxTxTask() {
    if (!_rxTxTaskHandle) return;
    if (_rxEventQueue) postTaskEvent(TASK_WAKE_UP);
    while (!_rxTxTaskParked || eTaskGetState(_rxTxTaskHandle) != eSuspended) {
        vTaskDelay(1);
    }
    vTaskDelete(_rxTxTaskHandle);
    _rxTxTaskHandle = nullptr;
}

/* @brief Post one of the rxTxTask's own events into the UART event queue
 * @return True if the event was queued (false if the queue is full)
 * @note Posted behind the pending UART events, which keeps their processing order
 */
bool RTU::postTaskEvent(TaskEvent taskEvent) {
    // Our events start at UART_EVENT_MAX (the driver's last enumerator): they can't collide with
    // the driver's types. They must also fit in the value range of uart_event_type_t (an enum
    // without fixed underlying type: the smallest bit-field holding its enumerators), as casting
    // an out-of-range value to it is undefined. Fails to compile if an SDK update breaks this.
    constexpr auto enumValueRangeMax = [](unsigned v) { unsigned m = 1; while (m <= v) m <<= 1; return m - 1; };
    static_assert(static_cast<unsigned>(TASK_TX_REQUEST) <= enumValueRangeMax(UART_EVENT_MAX),
                  "RTU task events don't fit in uart_event_type_t");

    uart_event_t event = {};
    event.type = static_cast<uart_event_type_t>(taskEvent);
    return xQueueSend(_rxEventQueue, &event, 0) == pdTRUE;
}

/* @brief Process a received Modbus frame
 * @param frameBytes: The frame to process
 * @return SUCCESS if the frame has been processed
 * @note This method is called by the rxTxTask exclusively
 */
RTU::Result RTU::processReceivedFrame(const ByteBuffer& frameBytes) {
    if (frameBytes.empty()) {
        return Error(ERR_INVALID_FRAME, "called with empty buffer, no frame to process");
    }

    Modbus::Debug::LOG_MSGF("Received raw data (%u bytes) from port %d", (uint32_t)frameBytes.size(), (int)_uartHAL.getPort());
    Modbus::Debug::LOG_HEXDUMP(frameBytes);

    Modbus::Frame receivedFrame;
    Modbus::MsgType msgType;

    switch (_role) {
        case Modbus::MASTER:
            msgType = Modbus::RESPONSE;
            break;
        case Modbus::SLAVE: {
            msgType = Modbus::REQUEST;
            _rtt.start(); // Start RTT for received slave request
            break;
        }
        default: 
            return Error(ERR_INVALID_ROLE, "invalid role set, cannot process incoming data");
    }

    auto decodeResult = ModbusCodec::RTU::decode(frameBytes, receivedFrame, msgType);
    if (decodeResult != ModbusCodec::SUCCESS) {
        // Slave: a request we can't decode (valid CRC) may still call for an exception (e.g.
        // unsupported function code): hand it to the application, which checks the slave ID
        // & answers it. Smallest frame with a CRC: slave ID + FC + CRC (4 bytes)
        if (_role != Modbus::SLAVE || frameBytes.size() < 4 ||
            !ModbusCodec::makeRejectedRequest(decodeResult, frameBytes[0], frameBytes[1], receivedFrame)) {
            return Error(ERR_INVALID_FRAME, "cannot decode received frame");
        }
        Modbus::Debug::LOG_MSG("Malformed request, handing it to the application to answer an exception");
    }

    Modbus::Debug::LOG_FRAME(receivedFrame, "Received frame successfully decoded");

    if (_role == Modbus::MASTER && receivedFrame.type == Modbus::RESPONSE) {
        _rtt.end(); // Terminate RTT for received master response
    }

    Modbus::Debug::LOG_MSG("Transmitting frame to the application layer...");
    notifyCallbacks(receivedFrame); 

    return Success();
}

/* @brief Update the UART idle detection delay from silenceTimeUs
 * @return SUCCESS if the idle detection has been updated
 * @note This method is called in begin() + when setSilenceTimeXXX() public methods are called, 
 *       if the interface is already initialized
 */
RTU::Result RTU::updateUartIdleDetection() {
    if (!_uartHAL.getRegisteredEventQueue()) {
        return Error(ERR_CONFIG_FAILED, "UART not ready, cannot update idle detection timeout");
    }

    esp_err_t err = _uartHAL.setTimeoutMicroseconds(_silenceTimeUs);
    if (err != ESP_OK) {
        return Error(ERR_CONFIG_FAILED, esp_err_to_name(err));
    }

    Modbus::Debug::LOG_MSGF("UART idle detection time set to: %u us (frame-gap backstop: %u us)",
                            (uint32_t)_silenceTimeUs, (uint32_t)_uartHAL.getRxFrameGapTimeoutUs());
    return Success();
}

/* @brief Notify TX result via callback
 * @param res: The result of the TX operation
 * @note This method calls the TX callback with stored context
 */
inline void RTU::notifyTxResult(Result res) {
    if (_txResultCallback) {
        _txResultCallback(res, _txCallbackCtx);
    }
}

/* @brief Handle an UART event
 * @param event: The UART event to handle
 * @return SUCCESS if no critical error occured (buffer filled or event skipped)
 * @note This method is called by the rxTxTask exclusively
 */
RTU::Result RTU::handleUartEvent(const uart_event_t& event) {
    switch (event.type) {
        case UART_DATA: {
            // Read data from UART driver if event.size > 0
            if (event.size > 0) {
                if (_rxBuffer.free_space() == 0) {
                    _rxBuffer.clear();
                    return Error(ERR_RX_FAILED, "RX buffer full, cannot read new bytes from UART, flushing RX buffer"); // We skip the event as we cannot process it
                } 
                
                // Resize buffer to fit incoming data & read data from UART driver
                // Normally, there should always be enough space in the buffer to fit
                // the incoming data, unless the Modbus peer is flooding us. In this case,
                // we will take whatever fits and the loop will deal with it (i.e. reach
                // free_space() == 0 & flush or let processReceivedFrame() fail)
                size_t toReadFromEvent = std::min(event.size, _rxBuffer.free_space());
                uint8_t* writePtr = _rxBuffer.end();
                size_t currentSize = _rxBuffer.size();
                _rxBuffer.resize(currentSize + toReadFromEvent);
                int bytesActuallyRead = _uartHAL.read(writePtr, toReadFromEvent, 0); // 0 ticks_to_wait as data is already in driver queue
                
                // If we read some bytes, adjust buffer size & print received data size
                // Normally, bytesActuallyRead should be equal to toReadFromEvent, but if
                // the case happens the loop will deal with it (i.e. reach free_space() == 0
                // & flush or let processReceivedFrame() fail).
                if (bytesActuallyRead > 0) {
                    _rxBuffer.trim(currentSize + bytesActuallyRead);
                    _lastRxByteTimeUs = TIME_US(); // Arm the software silence fallback (see rxTxTask)
                    Modbus::Debug::LOG_MSGF("Pulled %d bytes from UART into RX buffer, total %u", bytesActuallyRead, (uint32_t)_rxBuffer.size());
                }

                // If read returned <0, it indicates an UART error, we flush everything
                if (bytesActuallyRead < 0) {
                    _rxBuffer.clear();
                    _uartHAL.flush_input();
                    return Error(ERR_RX_FAILED, "UART RX error from HAL, flushing RX & UART buffers");
                }

                if (bytesActuallyRead == 0) {
                    // This case (expected data but read 0 bytes without error) should be rare with 
                    // event-driven reads. Just print a log and skip the event.
                    Modbus::Debug::LOG_MSGF("Warning: read no bytes from UART, expected %d", toReadFromEvent);
                }
            }

            // If RX timeout occurred (event.timeout_flag is true), process the accumulated data as a frame
            if (event.timeout_flag) {
                Modbus::Debug::LOG_MSGF("Silence time detected by UART driver, processing RX buffer (%u bytes)", (uint32_t)_rxBuffer.size());
                flushRxFrame();
            }
            break;
        }
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL: 
            // HW buffer full (Modbus peer is probably flooding us), we flush everything
            _rxBuffer.clear();
            _uartHAL.flush_input(); 
            return Error(ERR_RX_FAILED, "UART overflow/full, flushing RX & UART buffers");
        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
            // HW error, we skip the event
            return Error(ERR_RX_FAILED, "UART frame or parity error");
        default:
            // This case should never happen, we just print a log and skip the event
            Modbus::Debug::LOG_MSGF("Unhandled UART event detected (type %d, size %d)", event.type, event.size);
            return Error(ERR_RX_FAILED, "unhandled UART event");
    }
    return Success();
}

/* @brief Decode the accumulated RX buffer as one complete frame, then clear it
 * @note Called by the rxTxTask exclusively, from either the hardware idle-timeout event or the
 *       software silence fallback. Safe to call with an empty buffer (no-op).
 */
void RTU::flushRxFrame() {
    if (_rxBuffer.size() > 0) {
        processReceivedFrame(_rxBuffer);
    }
    _rxBuffer.clear();
}

/* @brief Handle a TX request
 * @return SUCCESS if the TX request has been handled
 * @note This method is called by the rxTxTask exclusively
 */
RTU::Result RTU::handleTxRequest() {
    Result send_res; // Sera initialisé dans la logique ci-dessous

    if (_txBuffer.size() > 0) { 
        Modbus::Debug::LOG_MSGF("rxTxTask sending raw frame (%u bytes) from port %d", (uint32_t)_txBuffer.size(), (int)_uartHAL.getPort());
        Modbus::Debug::LOG_HEXDUMP(_txBuffer);

        // Wait for the silence time to pass if needed.
        // The silence is a minimum gap, so over-waiting is harmless. Yield the millisecond-scale
        // portion to the scheduler (vTaskDelay) instead of busy-waiting it, then re-measure and
        // busy-wait only the sub-millisecond remainder for precision. Re-measuring guarantees the
        // minimum silence regardless of tick granularity / vTaskDelay alignment.
        uint64_t lastTxElapsedUs = TIME_US() - _lastTxTimeUs;
        if (lastTxElapsedUs < _silenceTimeUs) {
            uint64_t remainingUs = _silenceTimeUs - lastTxElapsedUs;
            if (remainingUs >= 1000) {
                WAIT_MS(static_cast<uint32_t>(remainingUs / 1000));
            }
            lastTxElapsedUs = TIME_US() - _lastTxTimeUs;
            if (lastTxElapsedUs < _silenceTimeUs) {
                WAIT_US(static_cast<uint32_t>(_silenceTimeUs - lastTxElapsedUs));
            }
        }

        // Send the frame & update the last TX time
        size_t sent_len = _uartHAL.write(_txBuffer.data(), _txBuffer.size());
        _lastTxTimeUs = TIME_US(); // Update the last TX time

        // Check if the frame was sent successfully
        if (sent_len < _txBuffer.size()) {
            send_res = Error(ERR_SEND_FAILED, "UART HAL write error or partial send");
        } else {
            send_res = Success();
            if (_role == Modbus::SLAVE) _rtt.end(); // End RTT for sent slave response
            Modbus::Debug::LOG_MSG("Frame sent successfully");
        }
    } else { 
        send_res = Error(ERR_SEND_FAILED, "TX request processed but TX buffer was empty");
    }

    notifyTxResult(send_res);
    _txResultCallback = nullptr; 
    _txCallbackCtx = nullptr;
    _txBuffer.clear(); // Safe because it's locked from sendFrame() call until we clear it here
    return send_res;
}

/* @brief Main task for the RX/TX operations
 * @brief - Waits for the next event in the queues (UART event or TX request)
 * @brief - Calls handleUartEvent() or handleTxRequest() accordingly
 * @brief - Kills itself if _isInitialized becomes false
 * @param rtu: The pointer to the RTU interface
 * @note This task is started in begin()
 */
void RTU::rxTxTask(void* rtu) { 
    RTU* self = static_cast<RTU*>(rtu);

    while (self->_isInitialized) {
        // Block on the UART event queue (no busy-wait: the task sleeps until an event or timeout).
        // While a frame is being assembled, poll fast (1 tick) so the software silence fallback stays
        // responsive; when idle, sleep on the long interval to yield CPU.
        TickType_t waitTicks = (self->_rxBuffer.size() > 0)
            ? pdMS_TO_TICKS(1)
            : pdMS_TO_TICKS(RXTX_QUEUE_CHECK_TIMEOUT_MS);
        if (waitTicks == 0) waitTicks = 1; // pdMS_TO_TICKS(1) rounds to 0 below a 1 kHz tick: keep it blocking
        uart_event_t event;
        bool gotEvent = (xQueueReceive(self->_rxEventQueue, &event, waitTicks) == pdTRUE);

        // Get out of the loop if the task is not initialized, otherwise wait for the next event
        if (!gotEvent) {
            if (!self->_isInitialized) { break; }
            // Frame-gap backstop: delimit the buffered frame when no HW idle-timeout event arrives -
            // Kept as a safety net, but the primary delimiter is uart_set_always_rx_timeout() (enabled in begin())
            uint64_t frameGapUs = self->_uartHAL.getRxFrameGapTimeoutUs();
            if (self->_rxBuffer.size() > 0 && frameGapUs > 0 &&
                (TIME_US() - self->_lastRxByteTimeUs) >= frameGapUs) {
                Modbus::Debug::LOG_MSGF("Frame-gap backstop: processing RX buffer (%u bytes)", (uint32_t)self->_rxBuffer.size());
                self->flushRxFrame();
            }
            continue;
        }

        switch (static_cast<int>(event.type)) {
            case TASK_TX_REQUEST:   // A frame is ready in the TX buffer: send it
                self->handleTxRequest();
                break;
            case TASK_WAKE_UP:      // Nothing to do: the loop checks _isInitialized
                break;
            default:                // UART driver event (data received...)
                self->handleUartEvent(event); 
                break;
        } 

        if (!self->_isInitialized) { break; }
    } 

    Modbus::Debug::LOG_MSGF("Modbus RxTx Task stopping for UART port %d", self->_uartHAL.getPort());
    self->_rxTxTaskParked = 1; // Last access to the object
    vTaskSuspend(nullptr);     // Deleted by killRxTxTask()
}

} // namespace ModbusInterface