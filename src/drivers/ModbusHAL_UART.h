/**
 * @file ModbusHAL_UART.h
 * @brief Event-driven HAL wrapper for UART using ESP UART API (header)
 */

#pragma once

#include "core/ModbusTypes.hpp"
#include "utils/ModbusDebug.hpp"

// ===================================================================================
// EXTERNAL DEPENDENCIES
// ===================================================================================

#if defined(PICO_SDK)
    #include "../external/pico-freertos-UartDmaDriver/include/UartDmaDriver.hpp"
#endif


// ===================================================================================
// COMMON HEADERS
// ===================================================================================

namespace ModbusHAL {

class UART {
public:

    // Internal constants for the UART driver
    static constexpr int MAX_TOUT_THRESH = 102;
    static constexpr int DRIVER_RX_BUFFER_SIZE = 512; 
    static constexpr int DRIVER_TX_BUFFER_SIZE = 256; // Set to 0 for a blocking TX without driver buffer
    static constexpr int DRIVER_EVENT_QUEUE_SIZE = 20; // Event queue size (identical to ESP32)
    static constexpr int WRITE_TIMEOUT_MS = 1000;
    static constexpr int READ_TIMEOUT_MS = 10; 

    enum EventType {
        UART_DATA, 
        UART_BREAK,
        UART_BUFFER_FULL,
        UART_FIFO_OVF,
        UART_FRAME_ERR,
        UART_PARITY_ERR,
        UART_DATA_BREAK,
        UART_PATTERN_DET,
    #if SOC_UART_SUPPORT_WAKEUP_INT
        UART_WAKEUP,
    #endif
        UART_EVENT_MAX
    };
    // Only for logs, because the event type is not dispatched elsewhere
    static constexpr const char* toString(EventType event) {
        switch (event) {
            case UART_DATA: return "Data received";
            case UART_BREAK: return "Break received";
            case UART_BUFFER_FULL: return "Buffer full";
            case UART_FIFO_OVF: return "FIFO overflow";
            case UART_FRAME_ERR: return "Frame error";
            case UART_PARITY_ERR: return "Parity error";
            case UART_DATA_BREAK: return "Data and break sent";
            case UART_PATTERN_DET: return "Pattern detected";
        #if SOC_UART_SUPPORT_WAKEUP_INT
            case UART_WAKEUP: return "Wakeup event";
        #endif
            default: return "Unknown event";
        }
    }

    // Generic UART event structure
    struct Event {
        EventType type;     // UART event type
        size_t size;        // UART data size for UART_DATA event
        bool timeout_flag;  // UART data read timeout flag for UART_DATA event (no new data received during configured RX TOUT)
                            // If the event is caused by FIFO-full interrupt, then there will be no event with the timeout flag before the next byte coming.
    };

    enum Result {
        SUCCESS,
        ERR_INIT,
        ERR_NOT_INITIALIZED,
        ERR_CONFIG,
        ERR_SEND,
        ERROR // Generic error code for all other cases
    };
    static const char* toString(const Result res) {
        switch (res) {
            case SUCCESS: return "Success";
            case ERR_INIT: return "UART init failed";
            case ERR_NOT_INITIALIZED: return "UART not initialized";
            case ERR_CONFIG: return "UART config failed";
            case ERR_SEND: return "UART send failed";
            case ERROR: return "UART error";
            default: return "Unknown error";
        }
    }

    // Include Error() and Success() definitions
    // (helpers to cast a Result)
    #include "core/ModbusResultHelpers.inl"

    ~UART();

    Result begin(QueueHandle_t* out_event_queue = nullptr, int intr_alloc_flags = 0);
    void end();

    // Read/write methods
    int read(uint8_t* buf_ptr, size_t max_len_to_read, TickType_t ticks_to_wait = pdMS_TO_TICKS(READ_TIMEOUT_MS));
    size_t write(const uint8_t* buf, size_t size); 
    int available() const;
    Result flush_input();

    // UART config methods
    uint32_t getBaudrate() const { return _baud_rate; }
    Result setBaudrate(uint32_t baud_rate); 

    QueueHandle_t getRegisteredEventQueue() const { return _internal_event_queue_handle; } // Renommé pour clarté
    Result setTimeoutMicroseconds(uint64_t timeout_us);


private:
    uint32_t _baud_rate;
    uint32_t _config_flags;
    QueueHandle_t _internal_event_queue_handle = nullptr;



// ===================================================================================
// ESP32-SPECIFIC HEADERS
// ===================================================================================

#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)

public:
    // Configuration flags (ESP32 ONLY)
    static constexpr uint32_t CONFIG_5N1 = UART_DATA_5_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_5N2 = UART_DATA_5_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_5E1 = UART_DATA_5_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_5E2 = UART_DATA_5_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_5O1 = UART_DATA_5_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_5O2 = UART_DATA_5_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_6N1 = UART_DATA_6_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_6N2 = UART_DATA_6_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_6E1 = UART_DATA_6_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_6E2 = UART_DATA_6_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_6O1 = UART_DATA_6_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_6O2 = UART_DATA_6_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_7N1 = UART_DATA_7_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_7N2 = UART_DATA_7_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_7E1 = UART_DATA_7_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_7E2 = UART_DATA_7_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_7O1 = UART_DATA_7_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_7O2 = UART_DATA_7_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_8N1 = UART_DATA_8_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_8N2 = UART_DATA_8_BITS | (UART_PARITY_DISABLE << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_8E1 = UART_DATA_8_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_8E2 = UART_DATA_8_BITS | (UART_PARITY_EVEN    << 8) | (UART_STOP_BITS_2 << 16);
    static constexpr uint32_t CONFIG_8O1 = UART_DATA_8_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_1 << 16);
    static constexpr uint32_t CONFIG_8O2 = UART_DATA_8_BITS | (UART_PARITY_ODD     << 8) | (UART_STOP_BITS_2 << 16);
    
    // Config structures
    #if defined(ARDUINO_ARCH_ESP32)
    struct ArduinoConfig {
        HardwareSerial& serial      = Serial0;   // Port (Serial0, Serial1, ...)
        uint32_t        baud        = 115200;
        uint32_t        config      = SERIAL_8N1; // SERIAL_xx from Arduino
        int             rxPin       = UART_PIN_NO_CHANGE;
        int             txPin       = UART_PIN_NO_CHANGE;
        int             dePin       = -1;          // -1 disables RS485 helper
    };
    #endif

    // Declare IDFConfig even in Arduino build so the user may opt-in.
    struct IDFConfig {
        uart_port_t uartNum   = UART_NUM_0;
        uint32_t    baud      = 115200;
        uint32_t    config    = CONFIG_8N1;
        int         rxPin     = UART_PIN_NO_CHANGE;
        int         txPin     = UART_PIN_NO_CHANGE;
        int         dePin     = -1;
    };

    #if defined(ARDUINO_ARCH_ESP32)
    using Config = ArduinoConfig; // Default alias for this platform
    #else
    using Config = IDFConfig;
    #endif
    
    // ESP-IDF constructor
    UART(uart_port_t uart_num, 
            uint32_t baud_rate, 
            uint32_t config_flags = CONFIG_8N1, 
            int pin_rx = UART_PIN_NO_CHANGE, 
            int pin_tx = UART_PIN_NO_CHANGE, 
            int pin_rts_de = -1); // Accept int, will convert to gpio_num_t internally
            
    explicit UART(const IDFConfig& cfg);

    #if defined(ARDUINO_ARCH_ESP32)
    // Arduino constructor
    UART(HardwareSerial& serial_dev,
         uint32_t baud_rate,
         uint32_t arduino_config, // e.g. SERIAL_8N1 from Arduino's HardwareSerial.h
         int pin_rx,
         int pin_tx,
         int pin_rts_de = -1); // Accept int, will convert to gpio_num_t internally

    explicit UART(const ArduinoConfig& cfg);
    #endif 

    // Public helpers
    uint32_t getPort() const { return _uart_num; }

    // Light Translation Layer - Public pour ModbusRTU.cpp
    using NativeEvent = uart_event_t;
    inline bool translateEvent(const NativeEvent& src, Event& dst) const {
        switch (src.type) {
            case UART_DATA:
                dst = { UART_DATA, src.size, src.timeout_flag };
                return true;
                
            case UART_BREAK:
                dst = { UART_BREAK, 0, false };
                return true;
                
            case UART_BUFFER_FULL:
                dst = { UART_BUFFER_FULL, 0, false };
                return true;
                
            case UART_FIFO_OVF:
                dst = { UART_FIFO_OVF, 0, false };
                return true;
                
            case UART_FRAME_ERR:
                dst = { UART_FRAME_ERR, 0, false };
                return true;
                
            case UART_PARITY_ERR:
                dst = { UART_PARITY_ERR, 0, false };
                return true;
                
            case UART_DATA_BREAK:
                dst = { UART_DATA_BREAK, 0, false };
                return true;
                
            case UART_PATTERN_DET:
                dst = { UART_PATTERN_DET, 0, false };
                return true;
                
            #if SOC_UART_SUPPORT_WAKEUP_INT
            case UART_WAKEUP:
                dst = { UART_WAKEUP, 0, false };
                return true;
            #endif
                
            default:
                return false; // Événement ignoré ou inconnu
        }
    }

private:
    // UART configuration variables
    uart_port_t _uart_num;
    int _pin_rx;
    int _pin_tx;
    gpio_num_t _pin_rts_de;
    bool _is_driver_installed = false;
    uart_config_t _current_hw_config;

    // Internal methods to handle UART operations & configuration
    Result waitTxComplete(TickType_t timeout_ticks = pdMS_TO_TICKS(WRITE_TIMEOUT_MS)) const;
    Result setRS485Mode(bool enable);
    Result getBufferedDataLen(size_t* size) const;
    Result setTimeoutThreshold(uint8_t timeout_threshold);
    Result enablePatternDetection(char pattern_char, uint8_t pattern_length);
    Result disablePatternDetection();
    Result flushTxFifo();

    static void decode_config_flags(uint32_t flags, uart_word_length_t& data_bits, uart_parity_t& parity, uart_stop_bits_t& stop_bits);

    #if defined(ARDUINO_ARCH_ESP32)
    // Arduino-specific methods
    static uint32_t convertArduinoConfig(uint32_t arduino_config);
    static uart_port_t serialArduinoToUartPort(const HardwareSerial* serial_ptr);
    #endif

#endif // ESP_PLATFORM || ARDUINO_ARCH_ESP32


// ===================================================================================
// STM32-SPECIFIC HEADERS
// ===================================================================================

#if defined(STM32_HAL)

#include "main.h" // or appropriate STM32 HAL header

public:
    // Configuration flags (STM32: informational only - actual config in STM32CubeMX)
    static constexpr uint32_t CONFIG_8N1 = 0x00; // Placeholder - real config in STM32CubeMX
    static constexpr uint32_t CONFIG_8E1 = 0x01; // Placeholder - real config in STM32CubeMX
    static constexpr uint32_t CONFIG_8O1 = 0x02; // Placeholder - real config in STM32CubeMX

    // Config structure for STM32
    struct STM32Config {
        UART_HandleTypeDef* huart         = nullptr;
        uint32_t            baud          = 115200;
        uint32_t            config        = CONFIG_8N1;  // Informational only
        int                 dePin         = -1;          // DE/RE pin for RS485
    };
    using Config = STM32Config;

    // STM32 HAL constructor  
    UART(UART_HandleTypeDef* huart, 
         uint32_t baud_rate,
         uint32_t config_flags = CONFIG_8N1,
         int pin_rts_de = -1);                            // Simple constructor, FreeRTOS timer used internally

    explicit UART(const STM32Config& cfg);

    // Public helpers
    UART_HandleTypeDef* getHandle() const { return _huart; }
    
    // Port identifier for logging (STM32: return handle address as ID)
    uint32_t getPort() const { 
        return reinterpret_cast<uint32_t>(_huart->Instance); 
    }

    // Light Translation Layer - Public pour ModbusRTU.cpp
    using NativeEvent = Event;  // For ModbusRTU.cpp compatibility
    inline bool translateEvent(const NativeEvent& src, Event& dst) const {
        // STM32 implementation already uses EZModbus Event format directly
        // This is a passthrough for API consistency across platforms
        dst = src;
        return true;
    }

private:
    // STM32 HAL configuration variables
    UART_HandleTypeDef* _huart;
    int _pin_rts_de;
    bool _is_driver_started = false;
    
    // DMA buffer configuration (static allocation)
    static constexpr size_t DMA_CHUNK_SIZE = 64;          // Fixed chunk size optimized for Modbus
    static constexpr size_t MAX_DMA_CHUNK_SIZE = 256;     // Maximum chunk size supported
    static constexpr size_t BYTE_QUEUE_SIZE = 512;        // Internal byte queue (2 Modbus frames)
    uint8_t _rx_buffer[DMA_CHUNK_SIZE];                   // Static buffer with fixed size
    
    // Silence timeout management (FreeRTOS timer)
    TimerHandle_t _silence_timer;          // FreeRTOS software timer for silence detection
    uint32_t _silence_timeout_us;          // Configurable silence timeout in microseconds
    volatile uint16_t _rx_bytes_pending;   // Bytes received but not yet processed
    
    // FreeRTOS queues for thread-safe communication
    QueueHandle_t _byte_queue = nullptr;   // Internal byte queue (producer/consumer)
    QueueHandle_t _event_queue = nullptr;  // Event queue (identical to ESP32)
    
    // TX management
    SemaphoreHandle_t _tx_semaphore = nullptr;

    // Static allocation buffers for FreeRTOS objects
    uint8_t _byteQueueStorage[BYTE_QUEUE_SIZE];
    StaticQueue_t _byteQueueBuffer;
    uint8_t _eventQueueStorage[DRIVER_EVENT_QUEUE_SIZE * sizeof(Event)];
    StaticQueue_t _eventQueueBuffer;
    StaticSemaphore_t _txSemaphoreBuffer;
    StaticTimer_t _silenceTimerBuffer;
    
    // Internal methods
    Result startReceive();
    Result stopReceive();
    Result setRS485Mode(bool enable);
    void setRS485TxMode(bool transmit_mode);
    void startSilenceTimer();              // Start/restart FreeRTOS silence timer
    void stopSilenceTimer();               // Stop FreeRTOS silence timer
    
    // Helper method to calculate default Modbus silence timeout
    static uint32_t calculateModbusSilenceTimeout(uint32_t baud_rate);
    
public:
    // Static callback methods (called by HAL) - must be public for extern "C" access
    static void rxEventCallback(UART_HandleTypeDef* huart, uint16_t Size);
    static void txCompleteCallback(UART_HandleTypeDef* huart);
    static void errorCallback(UART_HandleTypeDef* huart);
    
    // FreeRTOS timer callback (called when silence timeout expires)
    static void silenceTimerCallback(TimerHandle_t xTimer);

private:

#endif // STM32_HAL

// ===================================================================================
// RP2040-SPECIFIC HEADERS
// ===================================================================================

#if defined(PICO_SDK)

public:
    // Configuration flags (RP2040)
    static constexpr uint32_t CONFIG_8N1 = 0x00; // 8 data bits, no parity, 1 stop bit
    static constexpr uint32_t CONFIG_8N2 = 0x01; // 8 data bits, no parity, 2 stop bits
    static constexpr uint32_t CONFIG_8E1 = 0x02; // 8 data bits, even parity, 1 stop bit
    static constexpr uint32_t CONFIG_8O1 = 0x03; // 8 data bits, odd parity, 1 stop bit
    static constexpr uint32_t CONFIG_7N1 = 0x04; // 7 data bits, no parity, 1 stop bit
    static constexpr uint32_t CONFIG_7E1 = 0x05; // 7 data bits, even parity, 1 stop bit
    static constexpr uint32_t CONFIG_7O1 = 0x06; // 7 data bits, odd parity, 1 stop bit

    // Config structure for RP2040
    struct PicoConfig {
        uart_inst_t*    uart        = uart0;
        uint32_t        baud        = 115200;
        uint32_t        config      = CONFIG_8N1;
        int             rxPin       = -1;
        int             txPin       = -1;
        int             dePin       = -1;          // DE/RE pin for RS485
    };
    using Config = PicoConfig;

    // RP2040 constructor
    UART(uart_inst_t* uart, 
         uint32_t baud_rate,
         uint32_t config_flags = CONFIG_8N1,
         int pin_rx = -1,
         int pin_tx = -1,
         int pin_rts_de = -1);

    explicit UART(const PicoConfig& cfg);

    // Public helpers
    uart_inst_t* getUartInstance() const { return _uart; }
    
    // Port identifier for logging (RP2040: return UART instance address as ID)
    uint32_t getPort() const { 
        return reinterpret_cast<uint32_t>(_uart); 
    }

    // Helper methods pour compatibility
    void setRS485TxMode(bool transmit_mode);
    Result setRS485Mode(bool enable);

    // Light Translation Layer - Public pour ModbusRTU.cpp
    using NativeEvent = UartDmaDriver::Event;  // For ModbusRTU.cpp compatibility
    inline bool translateEvent(const NativeEvent& src, Event& dst) const {
        switch (src.type) {
            case UartDmaDriver::EVT_DATA:
                dst = { UART_DATA, src.size, src.silenceFlag };
                return true;
                
            case UartDmaDriver::EVT_OVERFLOW:
                dst = { UART_FIFO_OVF, 0, false };
                return true;
                
            default:
                return false; // Événement ignoré
        }
    }

private:
    // RP2040 configuration variables (unchanged)
    uart_inst_t* _uart;
    int _pin_rx, _pin_tx, _pin_rts_de;
    bool _is_driver_started = false;
    
    // UartDmaDriver instance DIRECTE (allocation statique compile-time)
    UartDmaDriver _dmaDriver;

#endif // PICO_SDK

}; // class UART

} // namespace ModbusHAL