# Quick Start Guide

This quick start guide presents the basic usage of EZModbus for `Client`, `Server` & `Bridge` applications, with the Arduino API.

ESP-IDF only differs in not including `Arduino.h` & using the IDF-style struct for UART configuration (see [Core concepts > HAL wrappers](../core-concepts/hal-wrappers.md#uart-rs485-ezmodbus-approach-esp-idf-api) for more information)

## Modbus Client

This example shows how to read a single holding register over Modbus RTU. Here, we use a simple "blocking" read (synchronous). Detailed use of asynchronous requests is introduced in the [How To Guides > Modbus Client (Master)](../how-to-guides/modbus-client-master.md#asynchronous-mode-with-result-tracker-non-blocking) section of this guide.

```cpp
#include <Arduino.h>
#include <EZModbus.h>

// Target device and register
#define SLAVE_ID       1
#define REG_ADDRESS    100  // Holding register 100

// Define UART config
ModbusHAL::UART::Config uartConfig = {
    .serial = Serial2,
    .baud = 9600,
    .config = SERIAL_8N1,
    .rxPin = 15,     
    .txPin = 14,
    .dePin = 5
}

// Instantiate UART, interface and client
ModbusHAL::UART         uart(uartConfig);
ModbusInterface::RTU    rtu(uart, Modbus::CLIENT);
Modbus::Client          client(rtu);

void setup() {
  Serial.begin(115200);  // Debug serial

  // Initialize UART + Modbus Client
  uart.begin();
  client.begin();

  Serial.println("✅ Modbus RTU client ready");
}

void loop() {
  // Build request frame for holding register
  Modbus::Frame request = {
    .type       = Modbus::REQUEST,
    .fc         = Modbus::READ_HOLDING_REGISTERS,
    .slaveId    = SLAVE_ID,
    .regAddress = REG_ADDRESS,
    .regCount   = 1, // Here we will read 1 register at REG_ADDRESS
    .data       = {} // No data in the request for reads
  };

  // Placeholder for response
  Modbus::Frame response;

  // Send synchronously (waits until response or timeout)
  if (client.sendRequest(request, response) == Modbus::Client::SUCCESS) {
    auto value = response.getRegister(0); // Get the first (and only) register value
    Serial.printf("Register %d value: %d\n", REG_ADDRESS, value);
  } else {
    Serial.println("⚠️ Read failed or timed out");
  }

  delay(1000);
}
```

That’s it! Your ESP32 now reads a register every second.

## Modbus Server

Here's how to implement a basic Modbus slave/server that exposes a few registers (Words). Here, we use a static Word store & basic pointer access to user variables. More advanced examples using custom callbacks are provided in the [How-To Guides > Modbus Server (Slave)](../how-to-guides/modbus-server-slave.md#handler-functions) section of this guide.

```cpp
#include <Arduino.h>
#include <EZModbus.h>

// Our server slave ID
#define SERVER_SLAVEID      1

// Define UART config
ModbusHAL::UART::Config uartConfig = {
    .serial = Serial2,
    .baud = 9600,
    .config = SERIAL_8N1,
    .rxPin = 15,     
    .txPin = 14,
    .dePin = 5
}

// Store to hold Modbus register map
Modbus::StaticWordStore<10> store;

// Create the RTU interface and server
ModbusHAL::UART      uart(uartConfig);
ModbusInterface::RTU rtu(uart, Modbus::SERVER);
Modbus::Server       server(rtu, store, SERVER_SLAVEID);

// Variables that will be exposed as Modbus registers
volatile uint16_t temperature = 230;  // 23.0°C (stored as fixed-point)
volatile uint16_t humidity = 450;     // 45.0% (stored as fixed-point)
volatile uint16_t fanState = 0;       // 0=off, 1=on

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Modbus Server...");
  
  // Register our variables as Modbus Words
  server.addWords({
    {Modbus::INPUT_REGISTER, 100, 1, &temperature},
    {Modbus::INPUT_REGISTER, 101, 1, &humidity},
    {Modbus::COIL,           1,   1, &fanState}
  });
  
  // Initialize UART + Modbus Server
  uart.begin();
  server.begin();
  
  Serial.println("Modbus server initialized!");
}

void loop() {
  // Update sensor values (in a real application, read from actual sensors)
  temperature = 230 + random(-10, 10);
  humidity = 450 + random(-20, 20);
  delay(1000); // Loop every second
}
```

No need to poll the server, it will process automatically incoming requests and fetch/update values in the background.

## Modbus Bridge

```cpp
#include <Arduino.h>
#include <EZModbus.h>
#include <WiFi.h>

// WiFi settings
const char* ssid = "YourNetworkName";
const char* password = "YourPassword";

// Define UART config
ModbusHAL::UART::Config uartConfig = {
    .serial = Serial2,
    .baud = 9600,
    .config = SERIAL_8N1,
    .rxPin = 15,     
    .txPin = 14,
    .dePin = 5
}

// Create TCP server on port 502 (standard Modbus TCP port) 
// & setup UART port with RS485 config
ModbusHAL::TCP  tcpServer(502);
ModbusHAL::UART uart(uartConfig);

// Create interfaces with complementary roles
ModbusInterface::RTU rtu(uart, Modbus::CLIENT);
ModbusInterface::TCP tcp(tcpServer, Modbus::SERVER);

// Create bridge to connect the interfaces
Modbus::Bridge bridge(rtu, tcp);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Modbus Bridge...");
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to WiFi. IP address: ");
  Serial.println(WiFi.localIP());
  
  // Initialize TCP & UART
  tcpServer.begin();
  uart.begin();
  
  // Start the bridge, it will automatically start the interfaces
  bridge.begin();
  
  Serial.println("Modbus bridge initialized!");
}

void loop() {
  // The bridge handles everything automatically in the background
  // If you just want to let it run, you can safely delete the main task
	vTaskDelete(NULL);
}
```

The bridge will automatically forward requests & responses in both directions.
