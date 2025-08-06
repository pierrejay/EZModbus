/**
 * @file ModbusHAL_UART_Pico.inl
 * @brief Event-driven HAL wrapper for UART using UartDmaDriver library
 */

#include "ModbusHAL_UART.h"

namespace ModbusHAL {

// ============================================================================
// Constants
// ============================================================================

static constexpr uint32_t WRITE_TIMEOUT_MS = 1000; // Timeout for DMA send operations

// ============================================================================
// Light Translation Layer - Moved to ModbusHAL_UART.h for visibility
// ============================================================================

// ============================================================================
// Helper functions
// ============================================================================

// Decode config flags into RP2040 UART parameters
static void decode_rp2040_config(uint32_t config_flags, uint& data_bits, uint& stop_bits, uart_parity_t& parity) {
    switch (config_flags) {
        case UART::CONFIG_8N1:
            data_bits = 8; stop_bits = 1; parity = UART_PARITY_NONE;
            break;
        case UART::CONFIG_8N2:
            data_bits = 8; stop_bits = 2; parity = UART_PARITY_NONE;
            break;
        case UART::CONFIG_8E1:
            data_bits = 8; stop_bits = 1; parity = UART_PARITY_EVEN;
            break;
        case UART::CONFIG_8O1:
            data_bits = 8; stop_bits = 1; parity = UART_PARITY_ODD;
            break;
        case UART::CONFIG_7N1:
            data_bits = 7; stop_bits = 1; parity = UART_PARITY_NONE;
            break;
        case UART::CONFIG_7E1:
            data_bits = 7; stop_bits = 1; parity = UART_PARITY_EVEN;
            break;
        case UART::CONFIG_7O1:
            data_bits = 7; stop_bits = 1; parity = UART_PARITY_ODD;
            break;
        default:
            // Default to 8N1 for unknown configs
            data_bits = 8; stop_bits = 1; parity = UART_PARITY_NONE;
            break;
    }
}

// Static helper to construct UartDmaDriver with correct parameters
static UartDmaDriver makeDmaDriver(uart_inst_t* uart, int pin_tx, int pin_rx, uint32_t baud_rate, uint32_t config_flags) {
    uint data_bits, stop_bits;
    uart_parity_t parity;
    decode_rp2040_config(config_flags, data_bits, stop_bits, parity);
    return UartDmaDriver(uart, pin_tx, pin_rx, baud_rate, data_bits, stop_bits, parity);
}


// ============================================================================
// Constructor / Destructor
// ============================================================================

UART::UART(uart_inst_t* uart,
           uint32_t baud_rate,
           uint32_t config_flags,
           int pin_rx,
           int pin_tx,
           int pin_rts_de)
    : _uart(uart),
      _pin_rx(pin_rx),
      _pin_tx(pin_tx),
      _pin_rts_de(pin_rts_de),
      _is_driver_started(false),
      _baud_rate(baud_rate),
      _config_flags(config_flags),
      _dmaDriver(makeDmaDriver(uart, pin_tx, pin_rx, baud_rate, config_flags)) {}

// Constructor from PicoConfig struct
UART::UART(const PicoConfig& config)
    : UART(config.uart,
           config.baud,
           config.config,
           config.rxPin,
           config.txPin,
           config.dePin) {}

UART::~UART() {
    end();
}

// ============================================================================
// Public API
// ============================================================================

UART::Result UART::begin(QueueHandle_t* out_event_queue, int /*intr_alloc_flags*/) {
    if (_is_driver_started) return Success("already started");
    
    // 1. Decode configuration before UartDmaDriver init
    uint data_bits, stop_bits;
    uart_parity_t parity;
    decode_rp2040_config(_config_flags, data_bits, stop_bits, parity);
    
    // 2. Pre-configure Modbus-optimized watchdog timing BEFORE starting driver
    uint32_t tickUs;
    uint8_t silenceTicks;
    
    if (_baud_rate >= 115200) {
        // High speed: 450µs × 4 ticks = 1.8ms (> 1.75ms Modbus spec)
        tickUs = 450;
        silenceTicks = 4;
    } else if (_baud_rate >= 19200) {
        // Med speed: 500µs × 4 ticks = 2.0ms (> 1.82ms for 3.5T@19200)
        tickUs = 500;
        silenceTicks = 4;
    } else {
        // Low speed (9600 bps): 800µs × 5 ticks = 4.0ms (> 3.65ms for 3.5T@9600)
        tickUs = 800;
        silenceTicks = 5;
    }
    
    if (_dmaDriver.setWatchdogTiming(tickUs, silenceTicks) != UartDmaDriver::SUCCESS) {
        return Error(ERR_CONFIG, "Failed to set Modbus-optimized timing");
    }
    
    Modbus::Debug::LOG_MSGF("RP2040 pre-configured Modbus timing: %u bps → %u µs (%u×%u)", 
                           _baud_rate, tickUs * silenceTicks, silenceTicks, tickUs);
    
    // 3. Initialize UartDmaDriver (object already statically constructed)
    if (_dmaDriver.init() != UartDmaDriver::SUCCESS) {
        return Error(ERR_INIT, "UartDmaDriver init failed");
    }
    
    // 3. Configure RS485 if necessary
    if (_pin_rts_de != -1) {
        gpio_init(_pin_rts_de);
        gpio_set_dir(_pin_rts_de, GPIO_OUT);
        gpio_put(_pin_rts_de, false); // Default to RX mode
    }
    
    // 4. Start UartDmaDriver
    if (_dmaDriver.start() != UartDmaDriver::SUCCESS) {
        return Error(ERR_INIT, "UartDmaDriver start failed");
    }
    
    // 5. API compatibility - Expose driver's native queue
    _internal_event_queue_handle = _dmaDriver.getEventQueueHandle();
    if (out_event_queue) *out_event_queue = _internal_event_queue_handle;
    
    _is_driver_started = true;
    return Success("RP2040 UART started (DMA mode)");
}

void UART::end() {
    if (!_is_driver_started) return;
    
    _is_driver_started = false;
    
    // 1. Stop UartDmaDriver (automatically manages its own queue)
    _dmaDriver.stop();
    
    _internal_event_queue_handle = nullptr;
}




// ============================================================================
// Public I/O methods (identical behaviour)
// ============================================================================

int UART::read(uint8_t* buf_ptr, size_t max_len_to_read, TickType_t /*ticks_to_wait*/) {
    if (!_is_driver_started || !buf_ptr || max_len_to_read == 0) {
        return -1;
    }
    
    // Event-driven architecture: if read() is called, a UART_DATA event 
    // has been popped -> data is guaranteed to be available in DMA buffer
    return static_cast<int>(_dmaDriver.read(buf_ptr, max_len_to_read));
}

size_t UART::write(const uint8_t* buf, size_t size) {
    if (!_is_driver_started || !buf || size == 0) {
        return 0;
    }
    
    // RS485 DE control (manual as before)
    if (_pin_rts_de != -1) {
        gpio_put(_pin_rts_de, true);
    }
    
    // Send via DMA (blocking)
    UartDmaDriver::Result result = _dmaDriver.send(buf, size, pdMS_TO_TICKS(WRITE_TIMEOUT_MS));
    
    if (_pin_rts_de != -1) {
        gpio_put(_pin_rts_de, false);
    }
    
    return (result == UartDmaDriver::SUCCESS) ? size : 0;
}

int UART::available() const {
    if (!_is_driver_started) return 0;
    return static_cast<int>(_dmaDriver.available());
}

UART::Result UART::flush_input() {
    if (!_is_driver_started) {
        return Error(ERR_NOT_INITIALIZED);
    }
    
    // Empty DMA buffer by reading everything
    uint8_t dummy[256];
    while (_dmaDriver.available() > 0) {
        _dmaDriver.read(dummy, sizeof(dummy));
    }
    
    return Success();
}

UART::Result UART::setBaudrate(uint32_t baud_rate) {
    // Use proper UartDmaDriver API for safe hot baud rate change
    auto result = _dmaDriver.setBaudrate(baud_rate);
    if (result != UartDmaDriver::SUCCESS) {
        return Error(ERR_CONFIG, "UartDmaDriver baud rate change failed");
    }
    
    // Update our stored baud rate
    _baud_rate = baud_rate;
    
    // Note: Watchdog timing reconfiguration while running requires driver restart
    // This will be handled by the application layer if needed
    
    return Success("Baudrate changed safely via UartDmaDriver");
}

UART::Result UART::setTimeoutMicroseconds(uint64_t timeout_us) {
    // Modbus-optimized timing is automatically configured in begin() based on baud rate
    // This method is kept for API compatibility but does not override the optimized settings
    
    Modbus::Debug::LOG_MSGF("RP2040 uses auto-configured Modbus timing (requested: %llu µs)", timeout_us);
    
    return Success("Using auto-configured Modbus-optimized timing");
}


void UART::setRS485TxMode(bool transmit_mode) {
    if (_pin_rts_de == -1) return;
    gpio_put(_pin_rts_de, transmit_mode);
}

UART::Result UART::setRS485Mode(bool /*enable*/) {
    // DE pin manually toggled in write(); nothing else to configure
    return Success();
}

} // namespace ModbusHAL