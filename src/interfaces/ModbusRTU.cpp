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
      _txRequestQueue(nullptr),
      _eventQueueSet(nullptr),
      _rxTxTaskHandle(nullptr)
{
    this->_role = role; 
}

RTU::~RTU() {
    if (_isInitialized) {
        _isInitialized = false; 
        killRxTxTask();
    }

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

    // Create TX request queue
    _txRequestQueue = xQueueCreateStatic(1, sizeof(void*), _txRequestQueueStorage, &_txRequestQueueBuffer);
    if (!_txRequestQueue) {
        _rxEventQueue = nullptr;
        return Error(ERR_INIT_FAILED, "failed to create TX request queue");
    }

    // Create event queue set (combines rxEventQueue + uartEventQueue)
    _eventQueueSet = xQueueCreateSet(_uartHAL.DRIVER_EVENT_QUEUE_SIZE + 1); // +1 for txRequestQueue (flag)
    if (!_eventQueueSet) {
        beginCleanup();
        return Error(ERR_INIT_FAILED, "failed to create event queue set");
    }

    // Flush rxEventQueue prior to adding it to the set
    // This is necessary so parasitic events that occur before RTU initialization do not prevent the QueueSet from being created
    ModbusHAL::UART::Event dump;
    while (xQueueReceive(_rxEventQueue, &dump, 0) == pdTRUE) { }

    // Add queues to QueueSet
    if (xQueueAddToSet(_rxEventQueue, _eventQueueSet) != pdPASS) {
        beginCleanup();
        return Error(ERR_INIT_FAILED, "failed to add RX event queue to set");
    }
    if (xQueueAddToSet(_txRequestQueue, _eventQueueSet) != pdPASS) {
        beginCleanup();
        return Error(ERR_INIT_FAILED, "failed to add TX request queue to set");
    }

    // Flush the buffers prior to starting the task
    _rxBuffer.clear();
    _uartHAL.flush_input();

    _isInitialized = true; // Needed for the task not to terminate prematurely

    // Create the RX/TX task
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

    // Update TX buffer under critical section (mutex protection)
    {
        Lock guard(_txMutex, 0); // try-lock no wait
        if (!guard.isLocked() || _txBuffer.size() != 0) {
            if (txCallback) txCallback(ERR_BUSY, ctx);
            return Error(ERR_BUSY, "TX already in progress");
        }
        
        _txResultCallback = txCallback; // Store the callback for rxTxTask
        _txCallbackCtx = ctx; // Store the context for rxTxTask
        if (_role == Modbus::MASTER && frame.type == Modbus::REQUEST) _rtt.store(); // Store the start time for master request

        // _txBuffer is considered free. Encode the frame.
        auto encodeResult = ModbusCodec::RTU::encode(frame, _txBuffer);
        if (encodeResult != ModbusCodec::SUCCESS) {
            if (txCallback) txCallback(ERR_INVALID_FRAME, ctx);
            _txBuffer.clear(); // Don't leave the buffer in an invalid state
            return Error(ERR_INVALID_FRAME, "encoding failed");
        }
    } // Mutex released here (end of RAII scope)

    Modbus::Debug::LOG_MSGF("Encoded TX frame (%u bytes) for port %d", (uint32_t)_txBuffer.size(), (int)_uartHAL.getPort());
    Modbus::Debug::LOG_HEXDUMP(_txBuffer);

    if (_role == Modbus::MASTER && frame.type == Modbus::REQUEST) {
        _rtt.start(&_rtt.storeUs); // Start RTT for master request (using stored start time)
    } 

    // Signal to rxTxTask that a frame is ready in _tx_buffer_internal
    void* signal_ptr = this; // Dummy value to signal the presence of a frame
    if (xQueueSend(_txRequestQueue, &signal_ptr, 0) != pdTRUE) {
        _txResultCallback = nullptr;
        _txCallbackCtx = nullptr;
        if (txCallback) txCallback(ERR_SEND_FAILED, ctx);
        return Error(ERR_SEND_FAILED, "failed to put encoded frame into TX queue");
    }
    
    Modbus::Debug::LOG_MSG("Frame queued for TX");
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
    
    // Do not delete _rxEventQueue as it belongs to _uartHAL
    _rxEventQueue = nullptr;
}

/* @brief Kill the RX/TX task
 * @note Called when the interface is destroyed
 */
void RTU::killRxTxTask() {
    if (_rxTxTaskHandle && _rxEventQueue) {
        // Send a dummy event to wake up the rxTxTask
        ModbusHAL::UART::Event dummy_evt = {
            .type = ModbusHAL::UART::EventType::UART_EVENT_MAX,
            .size = 0,
            .timeout_flag = false }; 
        xQueueSend(_rxEventQueue, &dummy_evt, 0); 
        vTaskDelay(pdMS_TO_TICKS(RXTX_QUEUE_CHECK_TIMEOUT_MS + 50)); // Wait a bit more than RxTxTask queue check timeout
    }
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
        return Error(ERR_INVALID_FRAME, "cannot decode received frame");
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

    auto err = _uartHAL.setTimeoutMicroseconds(_silenceTimeUs);
    if (err != ModbusHAL::UART::SUCCESS) {
        return Error(ERR_CONFIG_FAILED, ModbusHAL::UART::toString(err));
    }

    Modbus::Debug::LOG_MSGF("UART idle detection time set to: %u us", (uint32_t)_silenceTimeUs);
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
RTU::Result RTU::handleUartEvent(const ModbusHAL::UART::Event& event) {
#ifdef EZMODBUS_DEBUG
    Modbus::Debug::LOG_MSGF("UART event received - type: %s, size: %dB, timeout: %d", ModbusHAL::UART::toString(event.type), (int)event.size, event.timeout_flag);
#endif
    switch (event.type) {
        case ModbusHAL::UART::EventType::UART_DATA: {
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
                if (_rxBuffer.size() > 0) {
                    processReceivedFrame(_rxBuffer);
                }
                _rxBuffer.clear();
            }
            break;
        }
        case ModbusHAL::UART::EventType::UART_FIFO_OVF:
        case ModbusHAL::UART::EventType::UART_BUFFER_FULL: 
            // HW buffer full (Modbus peer is probably flooding us), we flush everything
            _rxBuffer.clear();
            _uartHAL.flush_input(); 
            return Error(ERR_RX_FAILED, "UART overflow/full, flushing RX & UART buffers");
        case ModbusHAL::UART::EventType::UART_FRAME_ERR:
        case ModbusHAL::UART::EventType::UART_PARITY_ERR:
            // HW error, we skip the event
            return Error(ERR_RX_FAILED, "UART frame or parity error");
        case ModbusHAL::UART::EventType::UART_EVENT_MAX: 
            // This case should never happen, we just print a log and skip the event
            // We use UART_EVENT_MAX to send a dummy event when we want to wake up the
            // loop to kill the rxTxTask.
            Modbus::Debug::LOG_MSG("UART_EVENT_MAX received, task will check _isInitialized to stop");
            break;
        default:
            // This case should never happen, we just print a log and skip the event
            Modbus::Debug::LOG_MSGF("Unhandled UART event detected (type %d, size %d)", event.type, event.size);
            return Error(ERR_RX_FAILED, "unhandled UART event");
    }
    return Success();
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

        // Wait for the silence time to pass if needed
        uint64_t lastTxElapsedUs = TIME_US() - _lastTxTimeUs;
        if (lastTxElapsedUs < _silenceTimeUs) {
            uint32_t delayUs = static_cast<uint32_t>(_silenceTimeUs - lastTxElapsedUs);
            WAIT_US(delayUs);
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
        QueueSetMemberHandle_t active_member = xQueueSelectFromSet(self->_eventQueueSet, pdMS_TO_TICKS(RXTX_QUEUE_CHECK_TIMEOUT_MS)); 

        // Get out of the loop if the task is not initialized, otherwise wait for the next event
        if (active_member == nullptr) { 
            if (!self->_isInitialized) { break; }
            continue; 
        }

        // If we received data from UART, push it to the rxBuffer
        if (active_member == self->_rxEventQueue) {
            // Light Translation Layer: safe conversion from native events to EZModbus events
            ModbusHAL::UART::NativeEvent nativeEvent;
            ModbusHAL::UART::Event event;
            if (xQueueReceive(self->_rxEventQueue, &nativeEvent, 0) == pdTRUE) {
                if (self->_uartHAL.translateEvent(nativeEvent, event)) {
                    self->handleUartEvent(event);
                }
                // Événements non traduits = ignorés silencieusement
            }
        } 
        // If we have a pending frame in the TX buffer, send it
        else if (active_member == self->_txRequestQueue) {
            void* signal_data; 
            if (xQueueReceive(self->_txRequestQueue, &signal_data, 0) == pdTRUE) {
                self->handleTxRequest(); 
            }
        } 

        if (!self->_isInitialized) { break; }
    } 

    Modbus::Debug::LOG_MSGF("Modbus RxTx Task stopping for UART port %d", self->_uartHAL.getPort());
    self->_rxTxTaskHandle = nullptr; 
    vTaskDelete(NULL); 
}

} // namespace ModbusInterface