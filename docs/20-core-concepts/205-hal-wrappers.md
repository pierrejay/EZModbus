# HAL wrappers

EZModbus introduces HAL (Hardware Abstraction Layer) wrappers that act as UART & TCP drivers separating physical peripheral management from Modbus protocol logic. This design provides framework independence, event-driven responsiveness, and user control over hardware interfaces.

The EZModbus drivers replace manual peripheral initialization such as `Serial.begin()` or `uart_driver_install()` and are then passed to the Interface objects in order to setup the Modbus link when instanciating EZModbus objects.

This separation ensures you maintain control over hardware while benefiting from optimized, event-driven peripheral management designed specifically for Modbus communication requirements.

!!! note
    The drivers must be properly initialized with `begin()` **before** the other Modbus components! Otherwise you will get an `ERR_INIT_FAILED` error when trying to initialize the Modbus application layer.

## Design philosophy

**Framework independence**

* Works natively with ESP-IDF without Arduino dependencies
* Full backward compatibility with Arduino Serial objects (UART)
* Extensible architecture for future platform support

**Event-driven**

* Zero-polling approach using FreeRTOS primitives
* Asynchronous timeout management and frame detection
* Maximum responsiveness with minimal CPU overhead when idle

**User control**

* Complete control over physical interfaces
* No hidden peripheral initialization
* Transparent configuration and error handling

## ModbusHAL::UART - UART/RS485 driver

### Features

* Encapsulates UART driver configuration and RS485 direction control
* Automatic 3.5T silence detection and DE/RE timing
* Platform-specific implementations:
    - **ESP32**: Compatible with both Arduino `HardwareSerial` objects and IDF UART handles, using native event-based ESP-IDF UART API
    - **Pico**: DMA-based transfers with custom UART driver supporting automatic silence detection (`UartDmaDriver` library)
    - **STM32**: DMA with IDLE line detection (requires configuration, usually done in STM32CubeMX) + FreeRTOS timer for silence detection

### Usage pattern

The UART driver allows initialization with a `Config` struct to keep your code tidy. Each platform has its own config structure:

* **ESP32**:
    - `IDFConfig`: Always available, aliased to `Config` for ESP-IDF builds
    - `ArduinoConfig`: Available only on Arduino, aliased to `Config` on Arduino builds
    - Arduino builds can still use `IDFConfig` by explicitly declaring it
* **Pico**:
    - `PicoConfig`: Aliased to `Config` for Pico builds
* **STM32**:
    - `STM32Config`: Aliased to `Config` for STM32 builds

#### *Legacy usage: UART/RS485 traditional approach (Arduino style)*

*This example is here just to illustrate how Serial UART initialization is generally done in Arduino projects. The following sections show how it's done with EZModbus.*

```cpp
// Serial parameters
uint32_t RS485_BAUD_RATE = 9600;
uint32_t RS485_CONFIG = SERIAL_8N1;
uint8_t RS485_RX_PIN = 16;
uint8_t RS485_TX_PIN = 17;
uint8_t RS485_DE_PIN = 2;

// Serial configuration
Serial2.begin(RS485_BAUD_RATE, RS485_CONFIG, RS485_RX_PIN, RS485_TX_PIN);

// Additional RE/DE pin configuration
pinMode(RS485_DE_PIN, OUTPUT);
```

#### ESP32 UART/RS485 - EZModbus init (Arduino API)

```cpp
// Raw constructor
ModbusHAL::UART UART(HardwareSerial& serial, uint32_t baudRate, uint32_t config, 
                     int rxPin, int txPin, int dePin);

// Recommended: use UART configuration structure
ModbusHAL::UART::Config uartConfig = {
    .serial = Serial1,        // SerialX instance
    .baud = 9600,             // Baud rate (bps)
    .config = SERIAL_8N1,     // Serial config: data bits, parity, stop bits
    .rxPin = 7,               // RX pin        
    .txPin = 8,               // TX pin
    .dePin = -1               // RE/DE pin (optional, -1 = disabled)
}

ModbusHAL::UART uart(uartConfig); // Create UART instance
uart.begin(); // Handles all UART and RS-485 configuration (returns an esp_err_t)
```

#### ESP32 UART/RS485 - EZModbus init (ESP-IDF API)

```cpp
// Raw constructor
ModbusHAL::UART UART(uart_port_t uartNum, uint32_t baudRate, uint32_t config, 
                     int rxPin, int txPin, int dePin);

// Recommended: use UART configuration structure
ModbusHAL::UART::Config uartConfig = {
    .uartNum = UART_NUM_2,                  // UART port num
    .baud = 9600,                           // Baud rate (bps)
    .config = ModbusHAL::UART::CONFIG_8N1,  // Serial config: data bits, parity, stop bits
    .rxPin = GPIO_NUM_7,                    // RX pin        
    .txPin = GPIO_NUM_8,                    // TX pin
    .dePin = GPIO_NUM_NC                    // RE/DE pin (optional, -1 = disabled)
}

ModbusHAL::UART uart(uartConfig); // Create UART instance
auto result = uart.begin(); // Handles all UART and RS-485 configuration (returns a Result)
```

#### Pico UART/RS485 - EZModbus init

```cpp
// Raw constructor
ModbusHAL::UART UART(uart_inst_t* uart, uint32_t baudRate, uint32_t config,
                     uint8_t rxPin, uint8_t txPin, int8_t dePin);

// Type aliases for convenience
using UART = ModbusHAL::UART;
using UARTConfig = ModbusHAL::UART::Config;  // Alias to PicoConfig

// Recommended: use UART configuration structure
UARTConfig uartConfig = {
    .uart    = uart0,             // Hardware UART instance (uart0/uart1)
    .baud    = 921600,            // Baud rate
    .config  = UART::CONFIG_8N1,  // Data format
    .rxPin   = 16,                // RX GPIO pin
    .txPin   = 17,                // TX GPIO pin
    .dePin   = -1                 // DE/RE pin (-1 = disabled)
};

// Create and initialize
UART uart(uartConfig);
auto result = uart.begin();
```

#### STM32 UART/RS485 - EZModbus init

!!! note
    UART must be configured in STM32CubeMX with DMA for RX/TX prior to usage in code. See the [Installation](../10-getting-started/101-installation.md#stm32-installation) section for detailed CubeMX setup.

```cpp
// Raw constructor
ModbusHAL::UART UART(UART_HandleTypeDef* huart, uint32_t baudRate, uint32_t config,
                     int dePin, GPIO_TypeDef* dePinPort = GPIOA);

// Recommended: use UART configuration structure
ModbusHAL::UART::Config uartConfig = {
    .huart     = &huart1,         // UART handle from ST HAL
    .baud      = 9600,            // Informational only (set in CubeMX)
    .config    = UART::CONFIG_8N1,// Informational only (set in CubeMX)
    .dePin     = 8,               // DE/RE pin number (or -1 if not used)
    .dePinPort = GPIOB            // DE/RE GPIO port (defaults to GPIOA if not specified)
};

// Create and initialize
ModbusHAL::UART uart(uartConfig);
auto result = uart.begin();
```

The `ModbusHAL::UART` object is then passed to the Modbus interface:

```cpp
ModbusInterface::RTU rtu(uart, Modbus::CLIENT);
```



## ModbusHAL::TCP - TCP socket driver

### Features

* Event-driven TCP/IP communication for Modbus TCP
* Automatic connection management and reconnection logic
* Support for both client and server modes
* Platform-specific implementations:
    - **ESP32**: Native socket API with `netif` integration
    - **Pico SDK**: CH9120 Ethernet module via UART interface with custom DMA-based `CH9120Driver` library (LwIP & W5500 coming soon)
    - **STM32**: TCP unsupported for now (LwIP & W5500 coming soon)

### Usage pattern

#### *Legacy usage: TCP traditional approach (Arduino style)*

```cpp
// Parameters
uint16_t TCP_SERVER_PORT = 502;                         // Local server port
IPAddress TCP_CLIENT_IP = IPAddress(192, 168, 1, 100);  // Remote server IP address
uint16_t TCP_CLIENT_PORT = 502;                         // Remote server port

// TCP server
EthernetServer tcpServer(TCP_SERVER_PORT);
tcpServer.begin();

// TCP client
EthernetClient tcpClient;
tcpClient.connect(TCP_CLIENT_IP, TCP_CLIENT_PORT);
```

#### ESP32 TCP - EZModbus init (Arduino & ESP-IDF, same API)

```cpp
// Parameters
uint16_t TCP_SERVER_PORT = 502;               // Local server port
const char* TCP_CLIENT_IP = "192.168.1.100";  // Remote server IP address
uint16_t TCP_CLIENT_PORT = 502;               // Remote server port

// TCP server
ModbusHAL::TCP tcpServer(TCP_SERVER_PORT);
tcpServer.begin(); // Handles all TCP configuration

// TCP client
ModbusHAL::TCP tcpClient(TCP_CLIENT_IP, TCP_CLIENT_PORT);
tcpClient.begin(); // Handles all TCP configuration & connection to remote server (returns a bool)
```

#### Pico TCP - EZModbus init (CH9120)

The Pico SDK implementation uses the CH9120 Ethernet module, which requires two configuration items:

* **`HardwareConfig`**: Defines the physical connection between the Pico and CH9120 (UART settings, pinout)
* **`NetworkConfig`**: Sets up the network parameters and operating mode (client/server, IP addresses, ports)

This separation allows the same hardware setup to be used in different network configurations. The TCP HAL constructor automatically detects the operating mode from the `NetworkConfig` struct.

In doubt, check the [CH9120Driver documentation](https://github.com/pierrejay/pico-freertos-CH9120Driver) for more details.

```cpp
// Raw constructor
ModbusHAL::TCP TCP(const HardwareConfig& hwConfig, const NetworkConfig& netConfig);

// Type alias
using TCP = ModbusHAL::TCP;

// Hardware configuration for CH9120 module
const CH9120Driver::HardwareConfig hwConfig = {
    .uart      = uart1,      // UART connected to CH9120
    .baudrate  = 115200,     // CH9120 UART baudrate
    .txPin     = 20,         // UART TX to CH9120 RX
    .rxPin     = 21,         // UART RX to CH9120 TX
    .cfgPin    = 18,         // CH9120 config pin
    .resPin    = 19,         // CH9120 reset pin
    .statusPin = 17          // CH9120 status pin
};

// Network configuration - TCP Client mode
const CH9120Driver::NetworkConfig clientConfig = {
    .localIp    = {192, 168, 0, 101},     // Device IP
    .gateway    = {192, 168, 0, 1},       // Gateway
    .subnetMask = {255, 255, 255, 0},     // Subnet
    .useDhcp    = true,                   // Use DHCP
    .targetIp   = {192, 168, 0, 100},     // Server IP
    .localPort  = 8000,                   // Local port
    .targetPort = 502,                    // Server port
    .mode       = CH9120Driver::Mode::TCP_CLIENT
};

// Network configuration - TCP Server mode
const CH9120Driver::NetworkConfig serverConfig = {
    .localIp    = {192, 168, 0, 101},     // Device IP
    .gateway    = {192, 168, 0, 1},       // Gateway
    .subnetMask = {255, 255, 255, 0},     // Subnet
    .useDhcp    = false,                  // Static IP
    .localPort  = 502,                    // Listen port
    .mode       = CH9120Driver::Mode::TCP_SERVER
};

// Create TCP HAL (auto-detects client/server from config)
TCP tcpHal(hwConfig, clientConfig);  // or serverConfig
tcpHal.begin();
```

The HAL TCP objects are then passed to Modbus interfaces:

```cpp
ModbusInterface::TCP tcp(tcpServer, Modbus::SERVER);
// or
ModbusInterface::TCP tcp(tcpClient, Modbus::CLIENT);
```
