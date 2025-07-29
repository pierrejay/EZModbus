/**
 * @file ModbusHAL_UART_ESP32.inl
 * @brief Event-driven HAL wrapper for UART using ESP32 UART API (implementation)
 */

#include "ModbusHAL_UART.h"

namespace ModbusHAL {

// ============================================================================
// Constructors
// ============================================================================

// Vanilla constructor: always available
UART::UART(uart_port_t uart_num, 
                 uint32_t baud_rate, 
                 uint32_t config_flags, 
                 int pin_rx, 
                 int pin_tx, 
                 int pin_rts_de)
    : _baud_rate(baud_rate),
      _config_flags(config_flags),
      _uart_num(uart_num),
      _pin_rx(pin_rx),
      _pin_tx(pin_tx),
      _pin_rts_de((pin_rts_de == -1) ? GPIO_NUM_NC : static_cast<gpio_num_t>(pin_rts_de)), 
      _is_driver_installed(false) {
        // Initialisation de _current_hw_config avec les paramètres fournis
        decode_config_flags(_config_flags, _current_hw_config.data_bits, _current_hw_config.parity, _current_hw_config.stop_bits);
        _current_hw_config.baud_rate = _baud_rate;
        _current_hw_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
            _current_hw_config.source_clk = UART_SCLK_DEFAULT;
        #else
            _current_hw_config.source_clk = UART_SCLK_APB;
        #endif
        Modbus::Debug::LOG_MSGF("Constructor for port %d", _uart_num);
}

// Constructor from IDFConfig struct (always available)
UART::UART(const IDFConfig& config)
    : UART(config.uartNum,
           config.baud,
           config.config,
           config.rxPin,
           config.txPin,
           config.dePin) {}

#if defined(ARDUINO_ARCH_ESP32)
// Arduino constructor: only available for Arduino framework
UART::UART(HardwareSerial& serial_dev,
             uint32_t baud_rate,
             uint32_t arduino_config,
             int pin_rx,
             int pin_tx,
             int pin_rts_de) 
             : UART(serialArduinoToUartPort(&serial_dev), // Determine uart_port_t from HardwareSerial*
                    baud_rate,
                    convertArduinoConfig(arduino_config),
                    pin_rx,
                    pin_tx,
                    pin_rts_de) // Pass int directly, conversion handled by the delegated constructor
{
    // The actual initialization is done by the delegated constructor.
    Modbus::Debug::LOG_MSGF("Arduino constructor for UART port %d", getPort());
}

// Constructor from ArduinoConfig struct for Arduino
UART::UART(const ArduinoConfig& config)
    : UART(config.serial,
           config.baud,
           config.config,
           config.rxPin,
           config.txPin,
           config.dePin) {}
#endif // ARDUINO_ARCH_ESP32

UART::~UART() {
    if (_is_driver_installed) {
        end();
    }
}

/* @brief Initialize the UART driver
 * @param out_event_queue: The event queue to use for the driver
 * @param intr_alloc_flags: The interrupt allocation flags
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is already installed
 */
ModbusHAL::UART::Result UART::begin(QueueHandle_t* out_event_queue, int intr_alloc_flags) {
    if (_is_driver_installed) {
        Modbus::Debug::LOG_MSGF("Warning: Port %d already initialized. Call end() first.", _uart_num);
        return Error(ERR_INIT, "port already initialized");
    }

    esp_err_t err = uart_param_config(_uart_num, &_current_hw_config);
    if (err != ESP_OK) {
        Modbus::Debug::LOG_MSGF("Error: uart_param_config failed for port %d: %s", _uart_num, esp_err_to_name(err));
        return Error(ERR_INIT, esp_err_to_name(err));
    }

    int rts_pin_to_set = (_pin_rts_de == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : _pin_rts_de;
    err = uart_set_pin(_uart_num, _pin_tx, _pin_rx, rts_pin_to_set, UART_PIN_NO_CHANGE /*CTS*/);
    if (err != ESP_OK) {
        Modbus::Debug::LOG_MSGF("Error: uart_set_pin FAILED for port %d with TX:%d RX:%d RTS:%d Error: %s", _uart_num, _pin_tx, _pin_rx, rts_pin_to_set, esp_err_to_name(err));
        return Error(ERR_INIT, esp_err_to_name(err));
    }

    err = uart_driver_install(_uart_num, DRIVER_RX_BUFFER_SIZE, DRIVER_TX_BUFFER_SIZE, 
                              DRIVER_EVENT_QUEUE_SIZE, 
                              &_internal_event_queue_handle, 
                              intr_alloc_flags);
    if (err != ESP_OK) {
        Modbus::Debug::LOG_MSGF("Error: uart_driver_install failed for port %d: %s", _uart_num, esp_err_to_name(err));
        _internal_event_queue_handle = nullptr; 
        return Error(ERR_INIT, esp_err_to_name(err));
    }
    
    if (!_internal_event_queue_handle && DRIVER_EVENT_QUEUE_SIZE > 0) { 
         return Error(ERR_INIT, "uart_driver_install succeeded but queue handle is null");
    }

    // Configure RS485 mode if RTS/DE pin is specified
    if (_pin_rts_de != GPIO_NUM_NC) {
        err = setRS485Mode(true);
        if (err != ESP_OK) {
            Modbus::Debug::LOG_MSGF("Error: Failed to set RS485 mode for port %d: %s", _uart_num, esp_err_to_name(err));
            uart_driver_delete(_uart_num);
            _internal_event_queue_handle = nullptr;
            _is_driver_installed = false;
            return Error(ERR_INIT, esp_err_to_name(err));
        }
        Modbus::Debug::LOG_MSGF("Port %d configured for RS485 Half-Duplex with DE on pin %d", _uart_num, (int)_pin_rts_de);
    }
    
    _is_driver_installed = true;
    Modbus::Debug::LOG_MSGF("Port %d initialized. Baud: %d, Config: 0x%X, TX:%d, RX:%d, DE:%d", _uart_num, (int)_baud_rate, (unsigned int)_config_flags, _pin_tx, _pin_rx, (int)_pin_rts_de);
    return Success();
}

/* @brief Deinitialize the UART driver
 */
void UART::end() {
    if (_is_driver_installed) {
        esp_err_t err = uart_driver_delete(_uart_num);
        if (err != ESP_OK) {
            Modbus::Debug::LOG_MSGF("Error: uart_driver_delete failed for port %d: %s", _uart_num, esp_err_to_name(err));
        }
        _is_driver_installed = false;
        _internal_event_queue_handle = nullptr; 
        Modbus::Debug::LOG_MSGF("Port %d de-initialized.", _uart_num);
    }
}

/* @brief Read data from the UART driver
 * @param buf_ptr: The buffer to read the data into
 * @param max_len_to_read: The maximum number of bytes to read
 * @param ticks_to_wait: The number of ticks to wait for data
 * @return The number of bytes read, or -1 if there was an error
 */
int UART::read(uint8_t* buf_ptr, size_t max_len_to_read, TickType_t ticks_to_wait) {
    if (!_is_driver_installed || !buf_ptr || max_len_to_read == 0) {
        return -1; // Error convention: -1 for invalid parameters or driver not ready
    }
    int bytes_read = uart_read_bytes(_uart_num, buf_ptr, max_len_to_read, ticks_to_wait);
    return bytes_read;
}

/* @brief Write data to the UART driver
 * @param buf: The buffer to write the data from
 * @param size: The number of bytes to write
 * @return The number of bytes written, or 0 if there was an error
 * @note Blocks until the physical transmission is complete
 */
size_t UART::write(const uint8_t* buf, size_t size) {
    if (!_is_driver_installed || size == 0) return 0;
    
    int sent_by_driver = uart_write_bytes(_uart_num, (const char*)buf, size);
    
    if (sent_by_driver < 0) { 
        Modbus::Debug::LOG_MSGF("Error: uart_write_bytes error on port %d: %d", _uart_num, sent_by_driver);
        return 0; 
    }
    if ((size_t)sent_by_driver < size) {
        Modbus::Debug::LOG_MSGF("Warning: uart_write_bytes partial write on port %d. Requested %d, sent to buffer %d", _uart_num, size, sent_by_driver);
        return (size_t)sent_by_driver; // Return the number of bytes actually written
    }

    // Wait for the physical transmission to complete
    esp_err_t tx_done_err = uart_wait_tx_done(_uart_num, pdMS_TO_TICKS(WRITE_TIMEOUT_MS)); 
    if (tx_done_err == ESP_ERR_TIMEOUT) {
        Modbus::Debug::LOG_MSGF("Warning: uart_wait_tx_done timed out on port %d after %d ms for %d bytes", _uart_num, WRITE_TIMEOUT_MS, sent_by_driver);
        return 0; // Physical transmission confirmation failed
    } else if (tx_done_err != ESP_OK) {
        Modbus::Debug::LOG_MSGF("Error: uart_wait_tx_done failed on port %d: %s", _uart_num, esp_err_to_name(tx_done_err));
        return 0; // Physical transmission confirmation failed
    }
    
    return (size_t)sent_by_driver; // Success, all requested bytes have been accepted and physically transmitted
}

/* @brief Get the number of bytes available in the UART driver
 * @return The number of bytes available
 */
int UART::available() const {
    if (!_is_driver_installed) return 0;
    size_t len = 0;
    uart_get_buffered_data_len(_uart_num, &len);
    return (int)len;
}

/* @brief Flush the input buffer of the UART driver
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not installed
 */
ModbusHAL::UART::Result UART::flush_input(){
    if (!_is_driver_installed) return Error(ERR_NOT_INITIALIZED, "driver not installed");
    esp_err_t err = uart_flush_input(_uart_num);
    if (err != ESP_OK) return Error(ERROR, esp_err_to_name(err));
    return Success();
}

/* @brief Set the baud rate of the UART driver
 * @param baud_rate: The baud rate to set
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not installed
 */
ModbusHAL::UART::Result UART::setBaudrate(uint32_t baud_rate) {
    _baud_rate = baud_rate; 
    _current_hw_config.baud_rate = baud_rate; 

    if (!_is_driver_installed) {
        Modbus::Debug::LOG_MSGF("Info: Port %d baudrate set to %d (will be applied at begin())", _uart_num, (int)_baud_rate);
        return Success();
    }
    esp_err_t err = uart_param_config(_uart_num, &_current_hw_config);
    if (err != ESP_OK) {
        Modbus::Debug::LOG_MSGF("Error: Port %d failed to reconfigure baudrate to %d: %s", _uart_num, (int)baud_rate, esp_err_to_name(err));
        return Error(ERR_CONFIG, esp_err_to_name(err));
    }
    Modbus::Debug::LOG_MSGF("Info: Port %d baudrate reconfigured to %d", _uart_num, (int)_baud_rate);
    return Success();
}

/* @brief Wait for the physical transmission to complete
 * @param timeout_ticks: The number of ticks to wait for the transmission to complete
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not installed
 */
ModbusHAL::UART::Result UART::waitTxComplete(TickType_t timeout_ticks) const {
    if (!_is_driver_installed) return Error(ERR_NOT_INITIALIZED, "driver not installed");
    esp_err_t err = uart_wait_tx_done(_uart_num, timeout_ticks);
    if (err != ESP_OK) return Error(ERROR, esp_err_to_name(err));
    return Success();
}

/* @brief Set the RS485 mode of the UART driver
 * @param enable: True to enable RS485 mode, false to disable
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not installed
 * @note If the DE/RTS pin is not available, the driver will fall back to UART mode
 */
ModbusHAL::UART::Result UART::setRS485Mode(bool enable) {
    if (!_is_driver_installed) return Error(ERR_NOT_INITIALIZED, "driver not installed");
    if (_pin_rts_de == GPIO_NUM_NC && enable) {
        return Error(ERR_CONFIG, "impossible to enable RS485 mode without DE/RTS pin");
    }
    // Fall back to UART mode if DE/RTS pin is not available (not a critical error)
    uart_mode_t mode = enable ? UART_MODE_RS485_HALF_DUPLEX : UART_MODE_UART;
    esp_err_t err = uart_set_mode(_uart_num, mode);
    if (err != ESP_OK) return Error(ERR_CONFIG, esp_err_to_name(err));
    return Success();
}

/* @brief Get the number of bytes available in the UART driver
 * @param size: The number of bytes available
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not installed
 */
ModbusHAL::UART::Result UART::getBufferedDataLen(size_t* size) const {
    if (!_is_driver_installed) return Error(ERR_NOT_INITIALIZED, "driver not installed");
    if (!size) return Error(ERROR, "invalid size");
    esp_err_t err = uart_get_buffered_data_len(_uart_num, size);
    if (err != ESP_OK) return Error(ERROR, esp_err_to_name(err));
    return Success();
}

/* @brief Set the timeout threshold for the UART driver
 * @param timeout_threshold: The timeout threshold to set
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not installed
 */
ModbusHAL::UART::Result UART::setTimeoutThreshold(uint8_t timeout_threshold) {
    if (!_is_driver_installed) return Error(ERR_NOT_INITIALIZED, "driver not installed");
    if (timeout_threshold >= MAX_TOUT_THRESH) {
        timeout_threshold = MAX_TOUT_THRESH - 1;
    }
    esp_err_t err = uart_set_rx_timeout(_uart_num, timeout_threshold);
    if (err != ESP_OK) return Error(ERR_CONFIG, esp_err_to_name(err));
    return Success();
}

/* @brief Enable pattern detection for the UART driver
 * @param pattern_char: The character to detect
 * @param pattern_length: The length of the pattern to detect
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not installed
 */
ModbusHAL::UART::Result UART::enablePatternDetection(char pattern_char, uint8_t pattern_length) {
    if (!_is_driver_installed) return Error(ERR_NOT_INITIALIZED, "driver not installed");
    esp_err_t err = uart_enable_pattern_det_baud_intr(_uart_num, pattern_char, pattern_length, 1, 0, 0);
    if (err != ESP_OK) return Error(ERR_CONFIG, esp_err_to_name(err));
    return Success();
}

/* @brief Disable pattern detection for the UART driver
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not installed
 */
ModbusHAL::UART::Result UART::disablePatternDetection() {
    if (!_is_driver_installed) return Error(ERR_NOT_INITIALIZED, "driver not installed");
    esp_err_t err = uart_disable_pattern_det_intr(_uart_num);
    if (err != ESP_OK) return Error(ERR_CONFIG, esp_err_to_name(err));
    return Success();
}

/* @brief Flush the TX FIFO of the UART driver
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not installed
 */
ModbusHAL::UART::Result UART::flushTxFifo() {
    if (!_is_driver_installed) return Error(ERR_NOT_INITIALIZED, "driver not installed");
    esp_err_t err = uart_flush(_uart_num);
    if (err != ESP_OK) return Error(ERROR, esp_err_to_name(err));
    return Success();
}

/* @brief Set the timeout threshold for the UART driver in microseconds
 * @param timeout_us: The timeout threshold to set
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the driver is not installed
 */
ModbusHAL::UART::Result UART::setTimeoutMicroseconds(uint64_t timeout_us) {
    if (!_is_driver_installed) return Error(ERR_NOT_INITIALIZED, "driver not installed");
    if (_baud_rate == 0) return Error(ERR_CONFIG, "invalid baud rate");

    // Calculate the number of UART symbols for the timeout
    constexpr uint32_t bitsPerUartSymbol = 11;  // 1 start + 8 data + 1 parity + 1 stop
    uint64_t usPerUartSymbol = (static_cast<uint64_t>(bitsPerUartSymbol) * 1000000ULL) / _baud_rate;
    if (usPerUartSymbol == 0) return Error(ERR_CONFIG, "invalid baud rate");

    // Calculate the threshold in number of UART symbols
    uint8_t threshold = static_cast<uint8_t>((timeout_us + usPerUartSymbol - 1) / usPerUartSymbol);
    
    // Apply limits
    if (threshold == 0) threshold = 1;
    if (threshold >= MAX_TOUT_THRESH) threshold = MAX_TOUT_THRESH - 1;

    // Configure the timeout
    auto res = setTimeoutThreshold(threshold);
    if (res != SUCCESS) {
        Modbus::Debug::LOG_MSGF("Failed to set UART timeout: %s");
        return Error(ERR_CONFIG, "failed to set UART timeout");
    }
    Modbus::Debug::LOG_MSGF("UART timeout set to: %u us -> %d UART symbols threshold", (uint32_t)timeout_us, threshold);
    return Success();
}


/* @brief Decode the config flags into data bits, parity, and stop bits 
 * @param flags: The config flags
 * @param data_bits: The data bits
 * @param parity: The parity
 * @param stop_bits: The stop bits
 */
void UART::decode_config_flags(uint32_t flags, uart_word_length_t& data_bits, uart_parity_t& parity, uart_stop_bits_t& stop_bits) {
    data_bits = (uart_word_length_t)(flags & 0xFF);
    parity = (uart_parity_t)((flags >> 8) & 0xFF);
    stop_bits = (uart_stop_bits_t)((flags >> 16) & 0xFF);
}


#if defined(ARDUINO_ARCH_ESP32)
/* @brief Convert Arduino config to ESP-IDF config
 * @param arduino_config: The Arduino config (SerialConfig type: e.g. SERIAL_8N1)
 * @return The "Vanilla" config (see CONFIG_xxx constants)
 */
uint32_t UART::convertArduinoConfig(uint32_t arduino_config) {
    switch (arduino_config) {
        // 8-bit data
        case SERIAL_8N1: return CONFIG_8N1;
        case SERIAL_8N2: return CONFIG_8N2;
        case SERIAL_8E1: return CONFIG_8E1;
        case SERIAL_8E2: return CONFIG_8E2;
        case SERIAL_8O1: return CONFIG_8O1;
        case SERIAL_8O2: return CONFIG_8O2;
        // 7-bit data
        case SERIAL_7N1: return CONFIG_7N1;
        case SERIAL_7N2: return CONFIG_7N2;
        case SERIAL_7E1: return CONFIG_7E1;
        case SERIAL_7E2: return CONFIG_7E2;
        case SERIAL_7O1: return CONFIG_7O1;
        case SERIAL_7O2: return CONFIG_7O2;
        // 6-bit data
        case SERIAL_6N1: return CONFIG_6N1;
        case SERIAL_6N2: return CONFIG_6N2;
        case SERIAL_6E1: return CONFIG_6E1;
        case SERIAL_6E2: return CONFIG_6E2;
        case SERIAL_6O1: return CONFIG_6O1;
        case SERIAL_6O2: return CONFIG_6O2;
        // 5-bit data
        case SERIAL_5N1: return CONFIG_5N1;
        case SERIAL_5N2: return CONFIG_5N2;
        case SERIAL_5E1: return CONFIG_5E1;
        case SERIAL_5E2: return CONFIG_5E2;
        case SERIAL_5O1: return CONFIG_5O1;
        case SERIAL_5O2: return CONFIG_5O2;
        default:
            Modbus::Debug::LOG_MSGF("Unknown Arduino UART config: 0x%X. Defaulting to 8N1.", arduino_config);
            return CONFIG_8N1;
    }
}

/* @brief Map a HardwareSerial instance to its corresponding uart_port_t
 * @param serial_ptr: pointer to the HardwareSerial (Serial0, Serial1, Serial2)
 * @return uart_port_t corresponding to the hardware instance (defaults to UART_NUM_0)
 */
uart_port_t UART::serialArduinoToUartPort(const HardwareSerial* serial_ptr) {
    // Serial0 always exists and is a HardwareSerial instance.
    if (serial_ptr == &Serial0) return UART_NUM_0;
    #if SOC_UART_NUM > 1
        if (serial_ptr == &Serial1) return UART_NUM_1;
    #endif
    #if SOC_UART_NUM > 2
        if (serial_ptr == &Serial2) return UART_NUM_2;
    #endif
    // Fallback or error if it's not one of the global objects.
    // This situation should ideally not happen if users pass Serial0, Serial1, or Serial2.
    // If they create a HardwareSerial instance manually, e.g. HardwareSerial mySerial(X); they should
    // use the other UART constructor directly with the port number.
    Modbus::Debug::LOG_MSG("Could not identify Serial UART port, defaulting to UART_NUM_0.");
    return UART_NUM_0;
}

#endif // ARDUINO_ARCH_ESP32

} // namespace ModbusHAL