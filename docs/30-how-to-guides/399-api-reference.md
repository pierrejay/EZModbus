# API Reference

Summary of the main EZModbus types & overview of APIs used in application code, organized by component with clear signatures and usage patterns.

## Core Types & Enums

```cpp
namespace Modbus {
    // Register types
    enum RegisterType {
        NULL_RT = 0,            // Invalid/null register type
        COIL,                   // Read/write binary outputs
        DISCRETE_INPUT,         // Read-only binary inputs
        HOLDING_REGISTER,       // Read/write 16-bit registers  
        INPUT_REGISTER          // Read-only 16-bit registers
    };

    // Function codes
    enum FunctionCode {
        READ_COILS                 = 0x01,
        READ_DISCRETE_INPUTS       = 0x02,
        READ_HOLDING_REGISTERS     = 0x03,
        READ_INPUT_REGISTERS       = 0x04,
        WRITE_COIL                 = 0x05,
        WRITE_REGISTER             = 0x06,
        WRITE_MULTIPLE_COILS       = 0x0F,
        WRITE_MULTIPLE_REGISTERS   = 0x10
    };

    // Exception codes  
    enum ExceptionCode {
        NULL_EXCEPTION         = 0x00,
        ILLEGAL_FUNCTION       = 0x01,
        ILLEGAL_DATA_ADDRESS   = 0x02,
        ILLEGAL_DATA_VALUE     = 0x03,
        SLAVE_DEVICE_FAILURE   = 0x04,
        SLAVE_DEVICE_BUSY      = 0x06
        // ... others
    };

    // Device roles
    enum Role {
        CLIENT = 1,              // Master device (initiates requests)
        SERVER = 0               // Slave device (responds to requests)
    };
}
```

## HAL Layer (Hardware Abstraction)

### UART/RS485 Driver

```cpp
namespace ModbusHAL {
    class UART {
    public:
        // Configuration structures
        struct ArduinoConfig {           // Arduino API
            HardwareSerial& serial;      // Serial1, Serial2, etc.
            uint32_t baud;               // Baud rate (9600, 19200...)
            uint32_t config;             // SERIAL_8N1, SERIAL_8E1...
            int rxPin, txPin;            // GPIO pins
            int dePin;                   // DE/RE pin (-1 = disabled)
        };
        
        struct IDFConfig {               // ESP-IDF API  
            uart_port_t uartNum;         // UART_NUM_0, UART_NUM_1...
            uint32_t baud;               // Baud rate
            uint32_t config;             // CONFIG_8N1, CONFIG_8E1...
            gpio_num_t rxPin, txPin;     // GPIO pins
            gpio_num_t dePin;            // DE/RE pin (GPIO_NUM_NC = disabled)
        };
        
        using Config = ArduinoConfig;    // Default (Arduino builds)
        using Config = IDFConfig;        // Default (ESP-IDF builds)

        // Constructors
        UART(const Config& config);
        UART(HardwareSerial& serial, uint32_t baud, uint32_t config,
             int rxPin, int txPin, int dePin = -1);

        // Methods
        esp_err_t begin();               // Initialize UART
        void stop();                     // Stop UART
    };
}
```

### TCP Socket Driver

Initialized from application code, hands management to the Interface layer.

```cpp
namespace ModbusHAL {
    class TCP {
    public:

        TCP();                           // Default (call begin() manually)
        TCP(uint16_t serverPort);        // Server mode
        TCP(const char* serverIP,        // Client mode
            uint16_t port);

        // Methods  
        bool begin();    // Initialize based on constructor mode
        void stop();     // Close connections
        
        // Status
        bool isServerRunning() const;
        bool isClientConnected();
        size_t getActiveSocketCount();
    };
}
```


## Interface Layer

### ModbusInterface::RTU

```cpp
namespace ModbusInterface {
    class RTU : public IInterface {
    public:
        // Constructor
        RTU(ModbusHAL::UART& uart,       // HAL UART driver
            Modbus::Role role);          // CLIENT/MASTER or SERVER/SLAVE
        
        // Methods
        Result begin();                  // Initialize interface
        Result setSilenceTimeMs(uint32_t ms);  // Set custom silence time
        Result setSilenceTimeBaud();     // Reset to default (3.5 char times)
    };
}
```

### ModbusInterface::TCP

```cpp
namespace ModbusInterface {
    class TCP : public IInterface {
    public:
        // Constructor
        TCP(ModbusHAL::TCP& hal,         // HAL TCP driver
            Modbus::Role role);          // CLIENT/MASTER or SERVER/SLAVE
        
        // Methods
        Result begin();                  // Initialize interface
    };
}
```


## Application Layer

### Modbus Client (Master)

```cpp
namespace Modbus {
    class Client {
    public:
        // Result codes
        enum Result {
            SUCCESS,                    // Operation completed successfully
            NODATA,                     // Waiting for response...
            ERR_INVALID_FRAME,          // Invalid frame
            ERR_BUSY,                   // Client busy with another request
            ERR_TX_FAILED,              // Transmission failed
            ERR_TIMEOUT,                // Request timeout
            ERR_INVALID_RESPONSE,       // Invalid response
            ERR_NOT_INITIALIZED,        // Client not initialized
            ERR_INIT_FAILED             // Init failed
        };

        // Constructor
        Client(ModbusInterface::IInterface& interface,
               uint32_t timeoutMs = 1000);

        // Initialization
        Result begin();
        bool isReady();

        // =================================================================
        // SIMPLE HELPER API (recommended for basic sync use cases)
        // =================================================================

        // Read helper - automatic Frame construction and parsing
        template<typename T>
        Result read(uint8_t slaveId,
                   RegisterType regType,
                   uint16_t startAddr,
                   uint16_t qty,
                   T* dst,
                   ExceptionCode* rspExcep = nullptr);

        // Write helper - automatic Frame construction
        template<typename T>
        Result write(uint8_t slaveId,
                    RegisterType regType,
                    uint16_t startAddr,
                    uint16_t qty,
                    const T* src,
                    ExceptionCode* rspExcep = nullptr);

        // =================================================================
        // FRAME-BASED API (for advanced control)
        // =================================================================

        // Sync (tracker = nullptr) / async (tracker defined)
        Result sendRequest(const Modbus::Frame& request,
                          Modbus::Frame& response,
                          Result* tracker = nullptr);

        // Async with callback
        using ResponseCallback = void (*)(Result result,
                                         const Modbus::Frame* response,
                                         void* userCtx);
        Result sendRequest(const Modbus::Frame& request,
                          ResponseCallback callback,
                          void* userCtx = nullptr);
    };
}
```

### Modbus Server (Slave)

```cpp
namespace Modbus {
    class Server {
    public:
        // Result codes
        enum Result {
            SUCCESS,
            NODATA,
            ERR_WORD_BUSY,
            ERR_WORD_OVERFLOW,
            ERR_WORD_INVALID,
            ERR_WORD_DIRECT_PTR,
            ERR_WORD_HANDLER,
            ERR_WORD_OVERLAP,
            ERR_RCV_UNKNOWN_WORD,
            ERR_RCV_BUSY,
            ERR_RCV_INVALID_TYPE,
            ERR_RCV_WRONG_SLAVE_ID,
            ERR_RCV_ILLEGAL_FUNCTION,
            ERR_RCV_ILLEGAL_DATA_ADDRESS,
            ERR_RCV_ILLEGAL_DATA_VALUE,
            ERR_RCV_SLAVE_DEVICE_FAILURE,
            ERR_RSP_TX_FAILED,
            ERR_NOT_INITIALIZED,
            ERR_INIT_FAILED
        };

        // Constructors
        // Single interface
        Server(ModbusInterface::IInterface& interface,
               IWordStore& store,                           // Register storage
               uint8_t slaveId = 1,                         // Device slave ID
               bool rejectUndefined = true);                // Reject undefined registers

        // Multi-interface
        Server(std::initializer_list<ModbusInterface::IInterface*> interfaces,
               IWordStore& store,                            // Register storage
               uint8_t slaveId = 1,                          // Device slave ID
               bool rejectUndefined = true,                  // Reject undefined registers
               uint32_t requestMutexTimeoutMs = UINT32_MAX); // Mutex timeout

        // Initialization  
        Result begin();

        // =================================================================
        // WORD/REGISTER API
        // =================================================================
        
        // Single word
        Result addWord(const Word& word);
        
        // Multiple words  
        Result addWords(const std::vector<Word>& words);
        Result addWords(const Word* words, size_t count);
        
        // Management
        Result clearAllWords();
        Word getWord(RegisterType type, uint16_t startAddr);

        // =================================================================
        // WORD STORE TYPES
        // =================================================================
        
        // Stack-based storage (compile-time size)
        template<size_t N>
        class StaticWordStore : public IWordStore { /* ... */ };
        
        // Heap-based storage (runtime size)  
        class DynamicWordStore : public IWordStore { /* ... */ };
    };

    // Handler function types for custom read/write
    using ReadWordHandler = ExceptionCode (*)(const Word& word, 
                                             uint16_t* outVals, 
                                             void* userCtx);
    using WriteWordHandler = ExceptionCode (*)(const uint16_t* writeVals, 
                                              const Word& word, 
                                              void* userCtx);

    // Word definition
    struct Word {
        RegisterType type = NULL_RT;    // COIL, HOLDING_REGISTER, etc.
        uint16_t startAddr = 0;         // Starting address
        uint16_t nbRegs = 0;            // Number of registers
        
        // Data access (choose one approach)
        volatile uint16_t* value = nullptr;      // Direct pointer (single register only)
        ReadWordHandler readHandler = nullptr;   // Custom read function
        WriteWordHandler writeHandler = nullptr; // Custom write function
        void* userCtx = nullptr;                 // User context for handlers
        
        // Validation
        operator bool() const { return isValid(type) && nbRegs > 0; }
    };
}
```

**Usage Patterns:**

```cpp
// Direct pointer access (simple - single register only)
volatile uint16_t temperature = 250;  // 25.0°C
Modbus::Word tempWord = {
    .type = Modbus::HOLDING_REGISTER,
    .startAddr = 100,
    .nbRegs = 1,
    .value = &temperature
};
server.addWord(tempWord);

// Callback-based access (advanced - single or multiple registers)
Modbus::Word configWord = {
    .type = Modbus::HOLDING_REGISTER,
    .startAddr = 200,
    .nbRegs = 3,  // Multi-register word
    .readHandler = readConfig,
    .writeHandler = writeConfig,
    .userCtx = &myContext
};
server.addWord(configWord);
```

### Modbus Bridge (Proxy)

```cpp
namespace Modbus {
    class Bridge {
    public:
        // Result codes
        enum Result {
            SUCCESS,
            ERR_INIT_FAILED              // Interfaces have same role
        };

        // Constructor
        Bridge(ModbusInterface::IInterface& interface1,
               ModbusInterface::IInterface& interface2);

        // Methods
        Result begin();                  // Start bidirectional forwarding
    };
}
```

**Requirements:**

- Interfaces must have **different roles** (one CLIENT, one SERVER)
- Bridge automatically forwards requests/responses between interfaces
- Supports any combination: RTU↔TCP, RTU↔RTU, TCP↔TCP

## Common Type Aliases & Patterns

The examples commonly use type aliases for cleaner code:

```cpp
// HAL layer aliases
using UART = ModbusHAL::UART;
using UARTConfig = ModbusHAL::UART::Config;
using TCP = ModbusHAL::TCP;

// Interface layer aliases
using ModbusRTU = ModbusInterface::RTU;
using ModbusTCP = ModbusInterface::TCP;

// Application layer aliases
using ModbusClient = Modbus::Client;
using ModbusServer = Modbus::Server;
using ModbusBridge = Modbus::Bridge;
using ModbusFrame = Modbus::Frame;
using ModbusWord = Modbus::Word;
```

## Frame Structure

```cpp
namespace Modbus {
    struct Frame {
        MsgType type;                    // REQUEST or RESPONSE
        uint8_t slaveId;                // Target/source slave ID
        FunctionCode fc;                // Function code (0x01-0x10)
        uint16_t regAddress;            // Starting register address
        uint16_t regCount;              // Number of registers/coils
        std::array<uint16_t, FRAME_DATASIZE> data; // Response data
        ExceptionCode exceptionCode;     // Exception if any
        
        // Utility methods
        bool isValid() const;
        void clear();
    };
}
```