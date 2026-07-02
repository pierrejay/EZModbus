# API Reference

Summary of the main EZModbus types and APIs used in application code, organized by component.

This page focuses on the public, user-facing API. Low-level hooks used internally by the built-in interfaces are intentionally omitted unless they are useful for normal application integration.

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
        NULL_FC = 0x00,
        READ_COILS                 = 0x01,
        READ_DISCRETE_INPUTS       = 0x02,
        READ_HOLDING_REGISTERS     = 0x03,
        READ_INPUT_REGISTERS       = 0x04,
        WRITE_COIL                 = 0x05,
        WRITE_REGISTER             = 0x06,
        WRITE_MULTIPLE_COILS       = 0x0F,
        WRITE_MULTIPLE_REGISTERS   = 0x10,
        NB_FC
    };

    // Exception codes
    enum ExceptionCode {
        NULL_EXCEPTION         = 0x00,
        ILLEGAL_FUNCTION       = 0x01,
        ILLEGAL_DATA_ADDRESS   = 0x02,
        ILLEGAL_DATA_VALUE     = 0x03,
        SLAVE_DEVICE_FAILURE   = 0x04,
        ACKNOWLEDGE            = 0x05,
        SLAVE_DEVICE_BUSY      = 0x06,
        NEGATIVE_ACKNOWLEDGE   = 0x07,
        MEMORY_PARITY_ERROR    = 0x08,
        NB_EC
    };

    // Message type
    enum MsgType {
        NULL_MSG,
        REQUEST,
        RESPONSE
    };

    // Device roles
    enum Role {
        SLAVE = 0,
        SERVER = SLAVE,
        MASTER = 1,
        CLIENT = MASTER
    };

    // Helpers
    const char* toString(RegisterType type);
    const char* toString(FunctionCode fc);
    const char* toString(ExceptionCode ec);
    const char* toString(MsgType type);
    bool isValid(RegisterType type);
    bool isValid(FunctionCode fc);
    bool isValid(ExceptionCode ec);
    bool isValid(MsgType type);
    bool isValid(Role role);
    RegisterType toRegisterType(FunctionCode fc);
    bool isBroadcastId(uint8_t slaveId);
}
```

## HAL Layer

### UART/RS485 Driver

```cpp
namespace ModbusHAL {
    class UART {
    public:
        // Serial config constants (data bits / parity / stop bits)
        // Format: CONFIG_<data><parity><stop>
        // Available for 5, 6, 7 and 8 data bits, N/E/O parity, 1/2 stop bits.
        static constexpr uint32_t CONFIG_8N1;
        static constexpr uint32_t CONFIG_8E1;
        static constexpr uint32_t CONFIG_8O1;
        static constexpr uint32_t CONFIG_8N2;
        static constexpr uint32_t CONFIG_8E2;
        static constexpr uint32_t CONFIG_8O2;

        #if defined(ARDUINO_ARCH_ESP32)
        struct ArduinoConfig {
            HardwareSerial& serial = Serial0;
            uint32_t baud = 115200;
            uint32_t config = SERIAL_8N1;
            int rxPin = UART_PIN_NO_CHANGE;
            int txPin = UART_PIN_NO_CHANGE;
            int dePin = -1;              // -1 disables RS485 direction control
        };
        #endif

        struct IDFConfig {
            uart_port_t uartNum = UART_NUM_0;
            uint32_t baud = 115200;
            uint32_t config = CONFIG_8N1;
            int rxPin = UART_PIN_NO_CHANGE;
            int txPin = UART_PIN_NO_CHANGE;
            int dePin = -1;              // -1 disables RS485 direction control
        };

        #if defined(ARDUINO_ARCH_ESP32)
        using Config = ArduinoConfig;
        #else
        using Config = IDFConfig;
        #endif

        // Constructors
        UART(uart_port_t uartNum,
             uint32_t baud,
             uint32_t config = CONFIG_8N1,
             int rxPin = UART_PIN_NO_CHANGE,
             int txPin = UART_PIN_NO_CHANGE,
             int dePin = -1);
        explicit UART(const IDFConfig& cfg);

        #if defined(ARDUINO_ARCH_ESP32)
        UART(HardwareSerial& serial,
             uint32_t baud,
             uint32_t config,
             int rxPin,
             int txPin,
             int dePin = -1);
        explicit UART(const ArduinoConfig& cfg);
        #endif

        ~UART();

        // Lifecycle
        esp_err_t begin();
        void end();

        // Runtime configuration
        uint32_t getBaudrate() const;
        esp_err_t setBaudrate(uint32_t baud);
        uint32_t getConfig() const;
        esp_err_t setConfig(uint32_t config);
        esp_err_t setParity(uart_parity_t parity);
        esp_err_t setStopBits(uart_stop_bits_t stopBits);
        esp_err_t setDataBits(uart_word_length_t dataBits);
        static constexpr uint32_t makeConfig(
            uart_word_length_t data,
            uart_parity_t parity,
            uart_stop_bits_t stop);

        // Port and RS485 helpers
        uart_port_t getPort() const;
        esp_err_t setRS485Mode(bool enable);
        bool isSoftDE() const;
    };
}
```

### TCP Socket Driver

Initialized from application code, then handed to the Interface layer.

```cpp
namespace ModbusHAL {
    class TCP {
    public:
        enum CfgMode { UNINIT, SERVER, CLIENT };

        TCP();                            // Default, call beginServer/beginClient
        explicit TCP(uint16_t serverPort);
        TCP(const char* serverIP, uint16_t port);
        ~TCP();

        TCP(const TCP&) = delete;
        TCP& operator=(const TCP&) = delete;

        // Lifecycle
        bool begin();                     // Initialize from constructor mode
        bool beginServer(uint16_t port, uint32_t ip = INADDR_ANY);
        bool beginClient(const char* serverIP, uint16_t port);
        void stop();

        // Status
        size_t getActiveSocketCount();
        bool isServerRunning() const;
        bool isClientConnected();
        bool isReady();
        CfgMode getMode() const;
    };
}
```

## Interface Layer

### Common Interface API

```cpp
namespace ModbusInterface {
    using RcvCallbackFn = void (*)(const Modbus::Frame& frame, void* ctx);

    class IInterface {
    public:
        enum Result {
            SUCCESS,
            NODATA,
            ERR_INIT_FAILED,
            ERR_INVALID_FRAME,
            ERR_BUSY,
            ERR_RX_FAILED,
            ERR_SEND_FAILED,
            ERR_INVALID_MSG_TYPE,
            ERR_INVALID_TRANSACTION_ID,
            ERR_TIMEOUT,
            ERR_INVALID_ROLE,
            ERR_ADD_CALLBACK_BUSY,
            ERR_TOO_MANY_CALLBACKS,
            ERR_NO_CALLBACKS,
            ERR_NOT_INITIALIZED,
            ERR_CONNECTION_FAILED,
            ERR_CONFIG_FAILED
        };

        using TxResultCallback = void (*)(Result result, void* ctx);

        virtual ~IInterface() = default;

        virtual Result begin() = 0;
        virtual Result sendFrame(const Modbus::Frame& frame,
                                 TxResultCallback txCallback = nullptr,
                                 void* ctx = nullptr) = 0;
        virtual bool isReady() = 0;
        Modbus::Role getRole() const;
        Result setRcvCallback(RcvCallbackFn fn, void* ctx = nullptr);
    };
}
```

### ModbusInterface::RTU

```cpp
namespace ModbusInterface {
    class RTU : public IInterface {
    public:
        explicit RTU(ModbusHAL::UART& uart,
                     Modbus::Role role = Modbus::MASTER);
        ~RTU() override;

        Result begin() override;
        Result setSilenceTimeMs(uint32_t silenceTimeMs);
        Result setSilenceTimeBaud();
        bool isReady() override;
    };
}
```

### ModbusInterface::TCP

```cpp
namespace ModbusInterface {
    class TCP : public IInterface {
    public:
        explicit TCP(ModbusHAL::TCP& hal, Modbus::Role role);
        ~TCP() override;

        Result begin() override;
        bool isReady() override;
    };
}
```

## Application Layer

### Modbus Client (Master)

```cpp
namespace Modbus {
    class Client {
    public:
        enum Result {
            SUCCESS,
            NODATA,
            ERR_INVALID_FRAME,
            ERR_BUSY,
            ERR_TX_FAILED,
            ERR_TIMEOUT,
            ERR_INVALID_RESPONSE,
            ERR_NOT_INITIALIZED,
            ERR_INIT_FAILED,
            ERR_TIMER_FAILURE
        };

        using ResponseCallback = void (*)(Result result,
                                          const Modbus::Frame* response,
                                          void* userCtx);

        Client(ModbusInterface::IInterface& interface,
               uint32_t timeoutMs = DEFAULT_REQUEST_TIMEOUT_MS);
        ~Client();

        Result begin();
        bool isReady();

        // Sync request. If userTracker is provided, it is updated with
        // the final request status.
        Result sendRequest(const Modbus::Frame& request,
                           Modbus::Frame& response,
                           Result* userTracker = nullptr);

        // Async request.
        Result sendRequest(const Modbus::Frame& request,
                           ResponseCallback cb,
                           void* userCtx = nullptr);

        // Convenience helpers.
        template<typename T>
        Result read(uint8_t slaveId,
                    RegisterType regType,
                    uint16_t startAddr,
                    uint16_t qty,
                    T* dst,
                    ExceptionCode* rspExcep = nullptr);

        template<typename T>
        Result write(uint8_t slaveId,
                     RegisterType regType,
                     uint16_t startAddr,
                     uint16_t qty,
                     const T* src,
                     ExceptionCode* rspExcep = nullptr);
    };
}
```

### Modbus Server (Slave)

```cpp
namespace Modbus {
    class Server {
    public:
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

        Server(ModbusInterface::IInterface& interface,
               IWordStore& store,
               uint8_t slaveId = 1,
               bool rejectUndefined = true,
               uint32_t reqMutexTimeoutMs = DEFAULT_REQ_MUTEX_TIMEOUT_MS);

        Server(std::initializer_list<ModbusInterface::IInterface*> interfaces,
               IWordStore& store,
               uint8_t slaveId = 1,
               bool rejectUndefined = true,
               uint32_t reqMutexTimeoutMs = DEFAULT_REQ_MUTEX_TIMEOUT_MS);

        ~Server();

        Result begin();

        Result addWord(const Word& word);
        Result addWords(const std::vector<Word>& words);
        Result addWords(const Word* words, size_t count);
        Result clearAllWords();
        Word getWord(RegisterType type, uint16_t startAddr);

        bool isBusy();
        uint8_t getSlaveId() const;
        void setSlaveId(uint8_t id);
    };
}
```

### Word Store API

```cpp
namespace Modbus {
    using ReadWordHandler = ExceptionCode (*)(const Word& word,
                                             uint16_t* outVals,
                                             void* userCtx);
    using WriteWordHandler = ExceptionCode (*)(const uint16_t* writeVals,
                                              const Word& word,
                                              void* userCtx);

    struct Word {
        RegisterType type = NULL_RT;
        uint16_t startAddr = 0;
        uint16_t nbRegs = 0;

        // Direct pointer access is allowed only for single-register words.
        volatile uint16_t* value = nullptr;

        // Handler access is required for multi-register words.
        ReadWordHandler readHandler = nullptr;
        WriteWordHandler writeHandler = nullptr;
        void* userCtx = nullptr;

        operator bool() const;
    };

    class IWordStore {
    public:
        virtual ~IWordStore() = default;
    };

    template<size_t N>
    class StaticWordStore : public IWordStore {
    public:
        StaticWordStore();
    };

    class DynamicWordStore : public IWordStore {
    public:
        explicit DynamicWordStore(size_t totalCapacity = 100);
    };
}
```

**Usage Patterns:**

```cpp
// Direct pointer access (single register only)
volatile uint16_t temperature = 250;  // 25.0 deg C
Modbus::Word tempWord = {
    .type = Modbus::HOLDING_REGISTER,
    .startAddr = 100,
    .nbRegs = 1,
    .value = &temperature
};
server.addWord(tempWord);

// Callback-based access (single or multiple registers)
Modbus::Word configWord = {
    .type = Modbus::HOLDING_REGISTER,
    .startAddr = 200,
    .nbRegs = 3,
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
        enum Result {
            SUCCESS,
            ERR_INIT_FAILED
        };

        Bridge(ModbusInterface::IInterface& interface1,
               ModbusInterface::IInterface& interface2);

        Result begin();
    };
}
```

**Requirements:**

- Interfaces must have different roles, one `CLIENT`/`MASTER` and one `SERVER`/`SLAVE`.
- Bridge forwards requests/responses between interfaces.
- Supported combinations include RTU<->TCP, RTU<->RTU and TCP<->TCP.

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
    enum class ByteOrder {
        AB,       // 16-bit big endian
        BA,       // 16-bit little endian
        ABCD,     // 32-bit big endian
        CDAB,     // 32-bit word swap
        BADC,     // 32-bit byte + word swap
        DCBA      // 32-bit little endian
    };

    struct Frame {
        MsgType type = NULL_MSG;
        FunctionCode fc = NULL_FC;
        uint8_t slaveId = 0;
        uint16_t regAddress = 0;
        uint16_t regCount = 0;
        std::array<uint16_t, FRAME_DATASIZE> data = {};
        ExceptionCode exceptionCode = NULL_EXCEPTION;

        void clear();
        void clearData(bool resetRegCount = true);

        uint16_t getRegister(size_t index) const;
        std::vector<uint16_t> getRegisters() const;
        size_t getRegisters(uint16_t* dst, size_t dstSize) const;
        bool getCoil(size_t index) const;
        std::vector<bool> getCoils() const;
        size_t getCoils(bool* dst, size_t dstSize) const;

        bool setRegisters(const std::vector<uint16_t>& src);
        bool setRegisters(const std::initializer_list<uint16_t>& src);
        bool setRegisters(const uint16_t* src, size_t len);
        bool setRegisters(const uint16_t* src, size_t len, size_t startRegIndex);

        bool setCoils(const std::vector<bool>& src);
        bool setCoils(const std::vector<uint16_t>& src);
        bool setCoils(const std::initializer_list<bool>& src);
        bool setCoils(const std::initializer_list<uint16_t>& src);
        bool setCoils(const bool* src, size_t len);
        bool setCoils(const uint16_t* src, size_t len);
        bool setCoils(const std::vector<bool>& src, size_t startCoilIndex);
        bool setCoils(const bool* src, size_t len, size_t startCoilIndex);

        size_t setFloat(float value, size_t regIndex,
                        ByteOrder order = ByteOrder::ABCD);
        size_t setUint32(uint32_t value, size_t regIndex,
                         ByteOrder order = ByteOrder::ABCD);
        size_t setInt32(int32_t value, size_t regIndex,
                        ByteOrder order = ByteOrder::ABCD);
        size_t setUint16(uint16_t value, size_t regIndex,
                         ByteOrder order = ByteOrder::AB);
        size_t setInt16(int16_t value, size_t regIndex,
                        ByteOrder order = ByteOrder::AB);

        bool getFloat(float& target, size_t regIndex,
                      ByteOrder order = ByteOrder::ABCD) const;
        bool getUint32(uint32_t& target, size_t regIndex,
                       ByteOrder order = ByteOrder::ABCD) const;
        bool getInt32(int32_t& target, size_t regIndex,
                      ByteOrder order = ByteOrder::ABCD) const;
        bool getUint16(uint16_t& target, size_t regIndex,
                       ByteOrder order = ByteOrder::AB) const;
        bool getInt16(int16_t& target, size_t regIndex,
                      ByteOrder order = ByteOrder::AB) const;
    };
}
```
