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
        struct ArduinoConfig {           // ESP32 Arduino API
            HardwareSerial& serial;      // Serial1, Serial2, etc.
            uint32_t baud;               // Baud rate (9600, 19200...)
            uint32_t config;             // SERIAL_8N1, SERIAL_8E1...
            int rxPin, txPin;            // GPIO pins
            int dePin;                   // DE/RE pin (-1 = disabled)
        };
        
        struct IDFConfig {               // ESP32 ESP-IDF API  
            uart_port_t uartNum;         // UART_NUM_0, UART_NUM_1...
            uint32_t baud;               // Baud rate
            uint32_t config;             // CONFIG_8N1, CONFIG_8E1...
            gpio_num_t rxPin, txPin;     // GPIO pins
            gpio_num_t dePin;            // DE/RE pin (GPIO_NUM_NC = disabled)
        };
        
        struct PicoConfig {              // Raspberry Pi Pico
            uart_inst_t* uart;           // uart0, uart1
            uint32_t baud;               // Baud rate
            uint32_t config;             // CONFIG_8N1, CONFIG_8E1...
            uint8_t rxPin, txPin;        // GPIO pins
            int8_t dePin;                // DE/RE pin (-1 = disabled)
        };
        
        struct STM32Config {             // STM32
            UART_HandleTypeDef* huart;   // UART handle from ST HAL
            uint32_t baud;               // Baud rate (informational)
            uint32_t config;             // CONFIG_8N1 (informational)
            int dePin;                   // DE/RE pin (-1 = disabled)
            GPIO_TypeDef* dePinPort;     // DE/RE GPIO port (defaults to GPIOA)
        };
        
        // Platform-specific type alias
        using Config = ArduinoConfig;    // ESP32 Arduino builds
        using Config = IDFConfig;        // ESP32 ESP-IDF builds
        using Config = PicoConfig;       // Pico SDK builds
        using Config = STM32Config;      // STM32 builds

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

        // =================================================================
        // ESP32 CONSTRUCTORS
        // =================================================================

        TCP();                           // Default (call begin() manually)
        TCP(uint16_t serverPort);        // Server mode
        TCP(const char* serverIP,        // Client mode
            uint16_t port);

        // =================================================================
        // Pico + CH9120 CONSTRUCTORS
        // =================================================================
        
        // Hardware config (CH9120 Ethernet module)
        struct HardwareConfig {
            uart_inst_t* uart = uart1;   // UART instance (uart0/uart1)
            uint32_t baudrate = 115200;  // UART baud rate (typically 115200)
            uint8_t txPin = 20;          // TX GPIO pin
            uint8_t rxPin = 21;          // RX GPIO pin  
            uint8_t cfgPin = 18;         // Config GPIO pin
            uint8_t resPin = 19;         // Reset GPIO pin
            uint8_t statusPin = 17;      // Status GPIO pin
        };
        
        // Network config  
        struct NetworkConfig {
            uint8_t localIp[4] = {192, 168, 1, 200};    // Module IP
            uint8_t gateway[4] = {192, 168, 1, 1};      // Gateway IP
            uint8_t subnetMask[4] = {255, 255, 255, 0}; // Subnet mask
            bool useDhcp = true;                        // Enable DHCP
            uint8_t targetIp[4] = {192, 168, 1, 100};   // Target IP (client mode)
            uint16_t localPort = 502;                   // Local port
            uint16_t targetPort = 502;                  // Target port (client mode)
            CH9120Driver::Mode mode;                    // TCP_SERVER or TCP_CLIENT
        };
        
        using CH9120Config = HardwareConfig;  // Alias for compatibility

        // Unified constructor (auto-detects server/client from NetworkConfig.mode)
        TCP(const HardwareConfig& hwConfig, 
            const NetworkConfig& netConfig);
        
        // =================================================================
        // COMMON API
        // =================================================================

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
            ERR_EXCEPTION_RESPONSE,     // Slave returned Modbus exception
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
        // TRANSPORT-LEVEL API (Frame-based)
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

        // =================================================================
        // APPLICATION-LEVEL API (Helper methods)
        // =================================================================
        
        // Read operations (registers: uint16_t*, coils: bool*)
        Result read(uint8_t slaveId, RegisterType regType,
                   uint16_t startAddr, uint16_t qty,
                   uint16_t* buffer,                 // For registers
                   ExceptionCode* exception = nullptr);
                   
        Result read(uint8_t slaveId, RegisterType regType,
                   uint16_t startAddr, uint16_t qty,
                   bool* buffer,                     // For coils/discrete inputs
                   ExceptionCode* exception = nullptr);

        // Write operations
        Result write(uint8_t slaveId, RegisterType regType,
                    uint16_t startAddr, uint16_t qty,
                    const uint16_t* buffer,          // For registers
                    ExceptionCode* exception = nullptr);
                    
        Result write(uint8_t slaveId, RegisterType regType,
                    uint16_t startAddr, uint16_t qty,
                    const bool* buffer,              // For coils
                    ExceptionCode* exception = nullptr);
    };
}
```

#### Return Value Contract:
- **`SUCCESS`**: Valid response received, data available in buffer
- **`ERR_EXCEPTION_RESPONSE`**: Slave returned exception (check `exception` parameter)
- **Other errors**: Transmission/protocol failures, no response data

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

        // Constructor
        Server(ModbusInterface::IInterface& interface,
               IWordStore& store,            // Register storage
               uint8_t slaveId = 1,         // Device slave ID
               bool rejectUndefined = true); // Reject undefined registers

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

## Frame Structure (Advanced Usage)

For low-level frame manipulation with `sendRequest()`:

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

## Usage Examples

### Basic Client (Synchronous)

```cpp
// Setup HAL + Interface + Application layers
ModbusHAL::UART uart(uartConfig);
ModbusInterface::RTU rtu(uart, Modbus::CLIENT);  // Interface wraps HAL
Modbus::Client client(rtu);                      // Client uses interface

uart.begin();  // Initialize HAL
client.begin(); // Initialize interface + client

// Read holding register
uint16_t value;
Modbus::ExceptionCode exception;
auto result = client.read(1, Modbus::HOLDING_REGISTER, 100, 1, &value, &exception);

if (result == Modbus::Client::SUCCESS) {
    // Success: use 'value'
} else if (result == Modbus::Client::ERR_EXCEPTION_RESPONSE) {
    // Exception: check 'exception' code
} else {
    // Other error: transmission failed
}
```

### Basic Server (ESP32)

```cpp
// Setup HAL + Interface + Application layers
ModbusHAL::TCP tcpHal(502);                      // HAL: Server on port 502
ModbusInterface::TCP tcp(tcpHal, Modbus::SERVER); // Interface wraps HAL
Modbus::StaticWordStore<10> store;               // Word storage
Modbus::Server server(tcp, store, 1);            // Server uses interface, slave ID = 1

// Expose variables
volatile uint16_t temperature = 250;
volatile uint16_t humidity = 600;

server.addWords({
    {.type = Modbus::HOLDING_REGISTER, .startAddr = 100, .nbRegs = 1, .value = &temperature},
    {.type = Modbus::HOLDING_REGISTER, .startAddr = 101, .nbRegs = 1, .value = &humidity}
});

tcpHal.begin();  // Auto-detects server mode from constructor
server.begin();
// Server runs automatically in background
```

### Basic Server (Pico + CH9120)

```cpp
// Hardware config (Waveshare CH9120 pinout)
const CH9120Driver::HardwareConfig hwConfig = {
    .uart = uart1,
    .baudrate = 115200,      // Typical CH9120 baud rate
    .txPin = 20,             // UART1 TX
    .rxPin = 21,             // UART1 RX
    .cfgPin = 18,            // Config pin
    .resPin = 19,            // Reset pin
    .statusPin = 17          // Status pin
};

// Network config (server mode)
const CH9120Driver::NetworkConfig netConfig = {
    .localIp = {192, 168, 0, 124},           // Module IP
    .gateway = {192, 168, 0, 1},             // Gateway
    .subnetMask = {255, 255, 255, 0},        // Subnet mask
    .useDhcp = true,                         // Use DHCP
    .targetIp = {192, 168, 0, 1},            // Dummy (required)
    .localPort = 502,                        // Modbus TCP port
    .targetPort = 80,                        // Dummy (required)
    .mode = CH9120Driver::Mode::TCP_SERVER   // Server mode
};

// Setup with type aliases
using TCP = ModbusHAL::TCP;
using ModbusTCP = ModbusInterface::TCP;
using ModbusServer = Modbus::Server;

TCP tcpHal(hwConfig, netConfig);
ModbusTCP tcpInterface(tcpHal, Modbus::SERVER);
Modbus::StaticWordStore<10> store;  // 10 words max
ModbusServer server(tcpInterface, store, 1);  // Slave ID = 1

// Register bank with initial values
volatile uint16_t registers[10] = {1000, 1001, 1002, /* ... */};

// Add words using direct pointers
for (size_t i = 0; i < 10; ++i) {
    server.addWord({
        .type = Modbus::HOLDING_REGISTER,
        .startAddr = static_cast<uint16_t>(100 + i),
        .nbRegs = 1,
        .value = &registers[i]
    });
}

tcpHal.begin();
server.begin();
```

### Bridge (RTU ↔ TCP)

```cpp
// RTU side (client to field devices)
ModbusHAL::UART uart(uartConfig);
ModbusInterface::RTU rtu(uart, Modbus::CLIENT);

// TCP side (server for SCADA/HMI) - ESP32 example
ModbusHAL::TCP tcpHal(502);  // Server on port 502
ModbusInterface::TCP tcp(tcpHal, Modbus::SERVER);

// Bridge them
Modbus::Bridge bridge(rtu, tcp);

uart.begin();
tcpHal.begin();  // Auto-detects server mode
bridge.begin();
// Bridge forwards automatically
```