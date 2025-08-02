# HAL wrappers

EZModbus introduces HAL (Hardware Abstraction Layer) wrappers that act as UART & TCP drivers separating physical peripheral management from Modbus protocol logic. This design provides framework independence, event-driven responsiveness, and user control over hardware interfaces.

The EZModbus drivers replace manual peripheral initialization such as `Serial.begin()` or `uart_driver_install()` and are then passed to the Interface objects in order to setup the Modbus link when instanciating EZModbus objects.

This separation ensures you maintain control over hardware while benefiting from optimized, event-driven peripheral management designed specifically for Modbus communication requirements.

{% hint style="info" %}
**Note:** The drivers must be properly initialized with `begin()` **before** the other Modbus components! Otherwise you will get an `ERR_INIT_FAILED` error when trying to initialize the Modbus application layer.
{% endhint %}

## **Design philosophy**

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

## **ModbusHAL::UART** - UART/RS485 driver

### Features

* Encapsulates UART driver configuration and RS485 direction control
* Compatible with both Arduino `HardwareSerial` objects and native ESP-IDF
* Automatic 3.5T silence detection and DE/RE timing with built-in ESP UART API

### **Usage pattern**

The UART driver allows initialization with a `Config` struct to keep your code tidy. There are two versions of this struct :

* `IDFConfig`: always available, aliased to `Config` for ESP-IDF builds
* `ArduinoConfig`: available only on Arduino, aliased to `Config` on Arduino builds

Arduino builds can still use the IDF-style config struct, but they need to explicitly declare it as an `IDFConfig` .

#### **Legacy usage: UART/RS485 - Traditional approach (Arduino style)**

This example is here just to illustrate how Serial UART initialization is generally done in Arduino projects. The following sections show how it's done with EZModbus.

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

#### **UART/RS485 - EZModbus approach (Arduino API)**

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

#### **UART/RS485 - EZModbus approach (ESP-IDF API)**

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
uart.begin(); // Handles all UART and RS-485 configuration (returns an esp_err_t)
```

The HAL RTU object is then passed to the Modbus interface:

```
ModbusInterface::RTU rtu(uart, Modbus::CLIENT);
```



## **ModbusHAL::TCP** - TCP socket driver

### Features

* Native ESP-IDF socket handling with event-driven architecture
* Automatic connection management and reconnection logic
* Support for both client and server modes

### Usage pattern

#### **Legacy usage: TCP - Traditional approach (Arduino style)**

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

#### **TCP - EZModbus approach (Arduino & ESP-IDF, same API)**

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

The HAL TCP objects are then passed to Modbus interfaces:

```cpp
ModbusInterface::TCP tcp(tcpServer, Modbus::SERVER);
// or
ModbusInterface::TCP tcp(tcpClient, Modbus::CLIENT);
```
