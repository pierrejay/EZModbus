/**
 * @file ModbusHAL_UART_STM32.inl
 * @brief Event-driven HAL wrapper for UART using STM32 DMA + IDLE detection (implementation)
 * 
 * STM32CubeMX Configuration Required:
 * =====================================
 * 
 * 1. UART Configuration:
 *    - Mode: Asynchronous
 *    - Baud Rate: As needed (9600, 19200, etc.)
 *    - Word Length: 8 Bits (or as required)
 *    - Parity: None (or Even/Odd as required)
 *    - Stop Bits: 1 (or 2 as required)
 * 
 * 2. DMA Configuration (MANDATORY):
 *    - Add DMA Request for UARTx_RX
 *    - Mode: Normal (not Circular)
 *    - Direction: Peripheral to Memory
 *    - Data Width: Byte (both peripheral and memory)
 *    - Increment Address: Memory only
 * 
 *    - Add DMA Request for UARTx_TX  
 *    - Mode: Normal
 *    - Direction: Memory to Peripheral
 *    - Data Width: Byte (both peripheral and memory)
 *    - Increment Address: Memory only
 * 
 * 3. NVIC Configuration:
 *    - Enable UARTx global interrupt
 *    - Enable DMAx_Channely global interrupt (for RX)
 *    - Enable DMAx_Channelz global interrupt (for TX)
 * 
 * 4. GPIO Configuration (if using RS485):
 *    - Configure DE/RE pin as GPIO Output
 *    - Initial state: Low (receive mode)
 * 
 * Usage Example:
 * ==============
 * ```cpp
 * // In your main code:
 * ModbusHAL::UART uart(&huart1, 9600, ModbusHAL::UART::CONFIG_8N1, DE_Pin_Number);
 * 
 * QueueHandle_t uart_events;
 * uart.begin(&uart_events);
 * 
 * // Event loop (identical to ESP32):
 * ModbusHAL::UART::Event event;
 * while (xQueueReceive(uart_events, &event, portMAX_DELAY)) {
 *     if (event.type == ModbusHAL::UART::UART_DATA) {
 *         // Process received data (event.size bytes)
 *         // event.timeout_flag indicates end of frame (silence detected)
 *     }
 * }
 * ```
 */

#include "ModbusHAL_UART.h"

namespace ModbusHAL {

// ============================================================================
// Light Translation Layer - Moved to ModbusHAL_UART.h for visibility
// ============================================================================

// ============================================================================
// Event-driven architecture using DMA + IDLE detection
// Replicates ESP32 UART behavior with zero polling overhead
// No dynamic allocation - only stack-based RAII patterns

// Static lookup table for UART handle to instance mapping (no pUserData in STM32G0)
static UART* g_uart_instances[8] = {nullptr}; // Support up to 8 UART instances

static UART* getInstanceFromHandle(UART_HandleTypeDef* huart) {
    // Simple lookup based on UART instance address
    for (int i = 0; i < 8; i++) {
        if (g_uart_instances[i] && g_uart_instances[i]->getHandle() == huart) {
            return g_uart_instances[i];
        }
    }
    return nullptr;
}

static void registerInstance(UART* instance) {
    for (int i = 0; i < 8; i++) {
        if (g_uart_instances[i] == nullptr) {
            g_uart_instances[i] = instance;
            break;
        }
    }
}

static void unregisterInstance(UART* instance) {
    for (int i = 0; i < 8; i++) {
        if (g_uart_instances[i] == instance) {
            g_uart_instances[i] = nullptr;
            break;
        }
    }
}

UART::UART(UART_HandleTypeDef* huart, 
           uint32_t baud_rate,
           uint32_t config_flags,
           int dePin,
           GPIO_TypeDef* dePinPort)
    : _baud_rate(baud_rate),
      _config_flags(config_flags),
      _huart(huart),
      _pin_rts_de(dePin),
      _pin_rts_de_port(dePinPort),
      _is_driver_started(false),
      _silence_timer(nullptr),
      _silence_timeout_us(calculateModbusSilenceTimeout(baud_rate)), // Auto-calculate from baud rate
      _rx_bytes_pending(0) {
    
    // Register instance in static lookup table (STM32G0 has no pUserData)
    registerInstance(this);
    
    Modbus::Debug::LOG_MSGF("STM32 UART constructor for handle %p (DMA chunk: %u bytes, silence: %u us, FreeRTOS timer)", 
                           _huart, (unsigned int)DMA_CHUNK_SIZE, (unsigned int)_silence_timeout_us);
}

// Constructor from STM32Config struct
UART::UART(const STM32Config& config)
    : UART(config.huart,
           config.baud,
           config.config,
           config.dePin,
           config.dePinPort) {}

UART::~UART() {
    if (_is_driver_started) {
        end();
    }
    
    // Unregister instance from lookup table
    unregisterInstance(this);
    
    // Clean up FreeRTOS resources
    if (_silence_timer) {
        xTimerDelete(_silence_timer, portMAX_DELAY);
        _silence_timer = nullptr;
    }
    if (_byte_queue) {
        vQueueDelete(_byte_queue);
        _byte_queue = nullptr;
    }
    if (_event_queue) {
        vQueueDelete(_event_queue);
        _event_queue = nullptr;
    }
    if (_tx_semaphore) {
        vSemaphoreDelete(_tx_semaphore);
        _tx_semaphore = nullptr;
    }
    
    // No buffer cleanup needed (static allocation)
}

ModbusHAL::UART::Result UART::begin(QueueHandle_t* out_event_queue, int intr_alloc_flags) {
    (void)intr_alloc_flags; // Unused parameter in STM32 implementation
    if (_is_driver_started) {
        Modbus::Debug::LOG_MSG("Warning: STM32 UART already started. Call end() first.");
        return Error(ERR_INIT, "UART already started");
    }
    
    if (!_huart) {
        return Error(ERR_INIT, "invalid UART handle");
    }
    
    // DMA chunk size is now a compile-time constant - no validation needed
    
    // Create internal byte queue (producer/consumer pattern)
    _byte_queue = xQueueCreateStatic(BYTE_QUEUE_SIZE, sizeof(uint8_t), _byteQueueStorage, &_byteQueueBuffer);
    if (!_byte_queue) {
        // No buffer cleanup needed (static allocation)
        return Error(ERR_INIT, "failed to create byte queue");
    }
    
    // Create event queue (identical to ESP32 API)
    _event_queue = xQueueCreateStatic(DRIVER_EVENT_QUEUE_SIZE, sizeof(Event), _eventQueueStorage, &_eventQueueBuffer);
    if (!_event_queue) {
        vQueueDelete(_byte_queue);
        _byte_queue = nullptr;
        // No buffer cleanup needed (static allocation)
        return Error(ERR_INIT, "failed to create event queue");
    }
    
    // Create TX completion semaphore
    _tx_semaphore = xSemaphoreCreateBinaryStatic(&_txSemaphoreBuffer);
    if (!_tx_semaphore) {
        vQueueDelete(_byte_queue);
        vQueueDelete(_event_queue);
        _byte_queue = nullptr;
        _event_queue = nullptr;
        // No buffer cleanup needed (static allocation)
        return Error(ERR_INIT, "failed to create TX semaphore");
    }
    
    // Create FreeRTOS silence timer (one-shot timer)
    TickType_t timer_period = pdMS_TO_TICKS(_silence_timeout_us / 1000);
    if (timer_period == 0) timer_period = 1; // Minimum 1 tick for timeouts < 1000µs
    
    _silence_timer = xTimerCreateStatic("SilenceTimer", 
                                  timer_period,  // Ensure minimum 1 tick
                                  pdFALSE,       // One-shot timer
                                  this,          // Timer ID (instance pointer)
                                  silenceTimerCallback,
                                  &_silenceTimerBuffer);
    if (!_silence_timer) {
        vQueueDelete(_byte_queue);
        vQueueDelete(_event_queue);
        vSemaphoreDelete(_tx_semaphore);
        _byte_queue = nullptr;
        _event_queue = nullptr;
        _tx_semaphore = nullptr;
        return Error(ERR_INIT, "failed to create FreeRTOS silence timer");
    }
    
    // Configure RS485 mode if needed
    if (_pin_rts_de != -1) {
        Result rs485_result = setRS485Mode(true);
        if (rs485_result != SUCCESS) {
            vQueueDelete(_byte_queue);
            vQueueDelete(_event_queue);
            vSemaphoreDelete(_tx_semaphore);
            _byte_queue = nullptr;
            _event_queue = nullptr;
            _tx_semaphore = nullptr;
            // No buffer cleanup needed (static allocation)
            return rs485_result;
        }
    }
    
    // Start DMA reception with IDLE detection
    Result rx_result = startReceive();
    if (rx_result != SUCCESS) {
        vQueueDelete(_byte_queue);
        vQueueDelete(_event_queue);
        vSemaphoreDelete(_tx_semaphore);
        _byte_queue = nullptr;
        _event_queue = nullptr;
        _tx_semaphore = nullptr;
        // No buffer cleanup needed (static allocation)
        return rx_result;
    }
    
    _is_driver_started = true;
    _internal_event_queue_handle = _event_queue; // For compatibility with ESP32 API
    
    // Return event queue to user (identical to ESP32)
    if (out_event_queue) {
        *out_event_queue = _event_queue;
    }
    
    Modbus::Debug::LOG_MSGF("STM32 UART initialized (handle %p, DMA chunk: %u, byte queue: %u, silence: %u us, FreeRTOS timer: %p)", 
                           _huart, (unsigned int)DMA_CHUNK_SIZE, (unsigned int)BYTE_QUEUE_SIZE, 
                           (unsigned int)_silence_timeout_us, _silence_timer);
    return Success();
}

void UART::end() {
    if (!_is_driver_started) return;
    
    // Stop silence timer
    stopSilenceTimer();
    
    // Stop DMA operations
    HAL_UART_AbortReceive(_huart);
    HAL_UART_AbortTransmit(_huart);
    
    // Clean up resources
    if (_silence_timer) {
        xTimerDelete(_silence_timer, portMAX_DELAY);
        _silence_timer = nullptr;
    }
    if (_byte_queue) {
        vQueueDelete(_byte_queue);
        _byte_queue = nullptr;
    }
    if (_event_queue) {
        vQueueDelete(_event_queue);
        _event_queue = nullptr;
        _internal_event_queue_handle = nullptr;
    }
    if (_tx_semaphore) {
        vSemaphoreDelete(_tx_semaphore);
        _tx_semaphore = nullptr;
    }
    
    _is_driver_started = false;
    Modbus::Debug::LOG_MSG("STM32 UART de-initialized (DMA stopped, queues cleaned, FreeRTOS timer deleted)");
}

// Read method - reads from internal FreeRTOS byte queue (thread-safe)
// Producer/Consumer pattern: DMA callback fills queue, read() empties it
int UART::read(uint8_t* buf_ptr, size_t max_len_to_read, TickType_t ticks_to_wait) {
    if (!_is_driver_started || !buf_ptr || max_len_to_read == 0) {
        return -1;
    }
    
    size_t bytes_read = 0;
    
    // Read bytes from internal queue (thread-safe)
    while (bytes_read < max_len_to_read) {
        uint8_t byte;
        TickType_t timeout = (bytes_read == 0) ? ticks_to_wait : 0; // Timeout only for first byte
        
        if (xQueueReceive(_byte_queue, &byte, timeout) == pdTRUE) {
            buf_ptr[bytes_read] = byte;
            bytes_read++;
        } else {
            break; // Timeout or no more data available
        }
    }
    
    return (int)bytes_read;
}

size_t UART::write(const uint8_t* buf, size_t size) {
    if (!_is_driver_started || !buf || size == 0) {
        return 0;
    }
    
    // Set RS485 DE pin if configured (transmit mode)
    if (_pin_rts_de != -1) {
        setRS485TxMode(true);
    }
    
    // Start DMA transmission
    Modbus::Debug::LOG_MSGF("Starting DMA TX: %u bytes", (unsigned int)size);
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(_huart, (uint8_t*)buf, size);
    if (status != HAL_OK) {
        Modbus::Debug::LOG_MSGF("DMA TX failed with status: %d", status);
        if (_pin_rts_de != -1) {
            setRS485TxMode(false); // Back to receive mode
        }
        return 0;
    }
    Modbus::Debug::LOG_MSG("DMA TX started successfully, waiting for completion...");
    
    // Wait for transmission to complete
    if (xSemaphoreTake(_tx_semaphore, pdMS_TO_TICKS(WRITE_TIMEOUT_MS)) != pdTRUE) {
        // Timeout - abort transmission
        HAL_UART_AbortTransmit(_huart);
        Modbus::Debug::LOG_MSGF("DMA TX timeout after %d ms", WRITE_TIMEOUT_MS);
        if (_pin_rts_de != -1) {
            setRS485TxMode(false);
        }
        return 0;
    }
    
    // Clear RS485 DE pin (back to receive mode)
    if (_pin_rts_de != -1) {
        setRS485TxMode(false);
    }
    
    Modbus::Debug::LOG_MSGF("DMA TX completed successfully: %u bytes sent", (unsigned int)size);
    return size;
}

int UART::available() const {
    if (!_is_driver_started || !_byte_queue) return 0;
    
    // Return number of bytes available in internal queue
    return (int)uxQueueMessagesWaiting(_byte_queue);
}

ModbusHAL::UART::Result UART::flush_input() {
    if (!_is_driver_started) return Error(ERR_NOT_INITIALIZED, "driver not started");
    
    // Empty the internal byte queue
    uint8_t dummy_byte;
    while (xQueueReceive(_byte_queue, &dummy_byte, 0) == pdTRUE) {
        // Empty queue
    }
    
    // Empty the event queue
    Event dummy_event;
    while (xQueueReceive(_event_queue, &dummy_event, 0) == pdTRUE) {
        // Empty queue
    }
    
    // Restart DMA reception to clear any partial data
    HAL_UART_AbortReceive(_huart);
    startReceive();
    
    return Success();
}

ModbusHAL::UART::Result UART::setBaudrate(uint32_t baud_rate) {
    return Error(ERR_CONFIG, "dynamic baud rate change not implemented for STM32");
}

// Internal methods
ModbusHAL::UART::Result UART::startReceive() {
    // Start DMA reception with IDLE detection
    // This will automatically call HAL_UARTEx_RxEventCallback when:
    // 1. Buffer is full (DMA_CHUNK_SIZE bytes received)
    // 2. IDLE line detected (silence = end of frame)
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(_huart, _rx_buffer, DMA_CHUNK_SIZE);
    if (status != HAL_OK) {
        return Error(ERR_INIT, "failed to start DMA receive with IDLE detection");
    }
    
    Modbus::Debug::LOG_MSGF("DMA reception started (chunk size: %u bytes)", (unsigned int)DMA_CHUNK_SIZE);
    return Success();
}

ModbusHAL::UART::Result UART::stopReceive() {
    HAL_UART_AbortReceive(_huart);
    return Success();
}

ModbusHAL::UART::Result UART::setRS485Mode(bool enable) {
    if (_pin_rts_de == -1) {
        return Error(ERR_CONFIG, "no DE pin configured for RS485");
    }
    
    // Initialize DE pin to receive mode (low)
    // Note: GPIO port should be configured by user in STM32CubeMX
    setRS485TxMode(false);
    
    Modbus::Debug::LOG_MSGF("RS485 mode %s (DE pin: %d)", enable ? "enabled" : "disabled", _pin_rts_de);
    return Success();
}

ModbusHAL::UART::Result UART::setTimeoutMicroseconds(uint64_t timeout_us) {
    _silence_timeout_us = (uint32_t)timeout_us;
    
    if (_silence_timer && _is_driver_started) {
        // Reconfigure FreeRTOS timer period
        TickType_t new_period = pdMS_TO_TICKS(_silence_timeout_us / 1000);
        if (new_period == 0) new_period = 1; // Minimum 1 tick for timeouts < 1000µs
        
        xTimerChangePeriod(_silence_timer, new_period, portMAX_DELAY);
    }
    
    TickType_t actual_period = pdMS_TO_TICKS(_silence_timeout_us / 1000);
    if (actual_period == 0) actual_period = 1;
    
    Modbus::Debug::LOG_MSGF("STM32 silence timeout set to %u us (FreeRTOS timer: %u ms)", 
                           (unsigned int)_silence_timeout_us, 
                           (unsigned int)actual_period);
    return Success();
}

// DMA RX Event callback - called automatically on buffer full or IDLE detected
// ULTRA FAST: dumps DMA buffer to FreeRTOS queue and manages silence timer
void UART::rxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    UART* instance = getInstanceFromHandle(huart);
    if (instance) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        
        // ⚡ ULTRA FAST: Dump DMA buffer to internal byte queue
        uint16_t bytes_queued = 0;
        for (uint16_t i = 0; i < Size; i++) {
            if (xQueueSendFromISR(instance->_byte_queue, 
                                  &instance->_rx_buffer[i], 
                                  &xHigherPriorityTaskWoken) == pdTRUE) {
                bytes_queued++;
            } else {
                // 🗑️ Queue full → DROP remaining bytes (no blocking!)
                // Modbus::Debug::LOG_MSGF("UART byte queue full, dropped %d bytes", Size - bytes_queued);
                break;
            }
        }
        
        // Update pending bytes counter
        instance->_rx_bytes_pending += bytes_queued;
        
        // FreeRTOS timer mode: Start/restart silence timer on every data reception
        // This gives us configurable timeout instead of fixed IDLE detection
        bool timeout_flag = false;
        
        if (Size < DMA_CHUNK_SIZE) {
            // IDLE detected - start silence timer
            instance->startSilenceTimer();
        } else {
            // Buffer full - restart timer (more data might come)
            instance->startSilenceTimer();
        }
        
        // Don't set timeout_flag yet - wait for FreeRTOS timer expiry
        // Event will be sent by silenceTimerCallback() when timeout occurs
        
        // Don't send UART_DATA event immediately - wait for silence timer
        // The silenceTimerCallback() will send the event when timeout expires
        // This ensures proper frame separation according to Modbus RTU timing
        
        // ⚡ Automatically restart DMA reception for next chunk
        HAL_UARTEx_ReceiveToIdle_DMA(huart, instance->_rx_buffer, DMA_CHUNK_SIZE);
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void UART::txCompleteCallback(UART_HandleTypeDef* huart) {
    UART* instance = getInstanceFromHandle(huart);
    if (instance) {
        // Modbus::Debug::LOG_MSG("DMA TX completed, releasing semaphore");
        // Signal DMA transmission complete
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(instance->_tx_semaphore, &xHigherPriorityTaskWoken);
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void UART::errorCallback(UART_HandleTypeDef* huart) {
    UART* instance = getInstanceFromHandle(huart);
    if (instance) {
        // Modbus::Debug::LOG_MSGF("UART Error callback - ErrorCode: 0x%x", (unsigned int)huart->ErrorCode);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        
        // Determine error type and create appropriate event
        EventType error_type = UART_FRAME_ERR;
        if (huart->ErrorCode & HAL_UART_ERROR_PE) {
            error_type = UART_PARITY_ERR;
        } else if (huart->ErrorCode & HAL_UART_ERROR_FE) {
            error_type = UART_FRAME_ERR;
        } else if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
            error_type = UART_FIFO_OVF;
        }
        
        // Modbus::Debug::LOG_MSGF("STM32 UART error on handle %p: 0x%lx", huart, huart->ErrorCode);
        
        // Push error event to queue
        Event evt = { error_type, 0, false };
        xQueueSendFromISR(instance->_event_queue, &evt, &xHigherPriorityTaskWoken);
        
        // Clear errors and restart DMA reception
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, instance->_rx_buffer, DMA_CHUNK_SIZE);
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Start/restart FreeRTOS silence timer (called from ISR)
void UART::startSilenceTimer() {
    if (!_silence_timer) return;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Reset and start timer (this automatically stops it if already running)
    xTimerResetFromISR(_silence_timer, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Stop FreeRTOS silence timer (always called from task context)
void UART::stopSilenceTimer() {
    if (!_silence_timer) return;
    
    // Always task context - no need for ISR detection
    xTimerStop(_silence_timer, portMAX_DELAY);
}

// FreeRTOS timer callback - called when silence timeout expires
void UART::silenceTimerCallback(TimerHandle_t xTimer) {
    // Get UART instance from timer ID
    UART* instance = static_cast<UART*>(pvTimerGetTimerID(xTimer));
    if (!instance) return;
    
    // Create timeout event if we have pending bytes
    if (instance->_rx_bytes_pending > 0) {
        Event evt = { 
            UART_DATA, 
            instance->_rx_bytes_pending,  // All pending bytes
            true                          // timeout_flag = true (end of frame)
        };
        
        // Send event to queue (FreeRTOS timer callback runs in timer service task context)
        if (xQueueSend(instance->_event_queue, &evt, 0) == pdTRUE) {
            Modbus::Debug::LOG_MSGF("Silence timeout: %u bytes processed (frame complete)", 
                                   (unsigned int)evt.size);
        } else {
            Modbus::Debug::LOG_MSG("Warning: Failed to send silence timeout event (queue full)");
        }
        
        // Reset pending bytes counter
        instance->_rx_bytes_pending = 0;
    }
}

// Helper methods
void UART::setRS485TxMode(bool transmit_mode) {
    if (_pin_rts_de == -1) return;
    
    // Use the configured GPIO port for DE/RE pin
    uint16_t gpio_pin = (1 << _pin_rts_de);
    
    HAL_GPIO_WritePin(_pin_rts_de_port, gpio_pin, transmit_mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Calculate default Modbus silence timeout based on baud rate
// Modbus RTU specification: 3.5 character times for frame separation
// Character time = (start + data + parity + stop bits) / baud_rate
uint32_t UART::calculateModbusSilenceTimeout(uint32_t baud_rate) {
    // Standard Modbus RTU frame format: 1 start + 8 data + 1 parity + 1 stop = 11 bits per character
    // For baud rates > 19200, use fixed 1750 μs (per Modbus spec)
    if (baud_rate > 19200) {
        return 1750; // Fixed 1.75ms for high baud rates
    }
    
    // Calculate 3.5 character times for low baud rates
    // Time per character = 11 bits / baud_rate (in seconds)
    // 3.5 characters = 3.5 * 11 * 1000000 / baud_rate (in microseconds)
    uint32_t char_time_us = (11 * 1000000) / baud_rate;
    uint32_t silence_timeout = (35 * char_time_us) / 10; // 3.5 characters
    
    // Ensure minimum timeout (prevent division issues)
    if (silence_timeout < 500) {
        silence_timeout = 500; // Minimum 0.5ms
    }
    
    return silence_timeout;
}

} // namespace ModbusHAL

// HAL callback functions - these will be called automatically by STM32 HAL
extern "C" {

// DMA + IDLE detection callback (main event source)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    ModbusHAL::UART::rxEventCallback(huart, Size);
}

// DMA TX complete callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    ModbusHAL::UART::txCompleteCallback(huart);
}

// Error callback
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    ModbusHAL::UART::errorCallback(huart);
}

// Legacy RX complete callback (not used in DMA mode, but provided for compatibility)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    (void)huart; // Unused parameter
    // In DMA mode, this shouldn't be called
    // HAL_UARTEx_RxEventCallback is used instead
}

}