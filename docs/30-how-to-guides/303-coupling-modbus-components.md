# Coupling Modbus components

## Connecting multiple components to a single interface

EZModbus’s modular design allows you to connect multiple application components to the same physical interface, offering flexible configurations for complex scenarios.

### Server: multiple servers on one interface

**✅ Fully supported and recommended (RTU ONLY)**

You can easily create multiple server instances on a single physical RTU interface, each responding to different Slave IDs:

```cpp
// Create one UART & RTU interface using Serial2
ModbusHAL::UART uart(Serial2, 9600, SERIAL_8N1);
ModbusInterface::RTU rtuIface(uart, Modbus::SLAVE);

// Create one store for each server
Modbus::StaticWordStore<200> tempStore;
Modbus::StaticWordStore<200> humStore;
Modbus::StaticWordStore<200> ctrlStore;
// Create multiple servers with different Slave IDs
Modbus::Server temperatureServer(rtuIface, tempStore, 1);  // Responds to Slave ID 1
Modbus::Server humidityServer(rtuIface, humStore, 2);     // Responds to Slave ID 2
Modbus::Server controlServer(rtuIface, ctrlStore, 3);      // Responds to Slave ID 3

// Add your Words to each store here...

// Initialize everything
uart.begin();
temperatureServer.begin();
humidityServer.begin();
controlServer.begin();
```

This approach is ideal for:

* Organizing different device functions into separate logical units
* Mimicking multiple physical devices from a single ESP32
* Creating clear separation between different register sets

**❌ Not supported or needed (TCP)**

As stated before, the TCP server will totally ignore the `slaveId` field. It means ALL TCP servers sharing the same interface will be handed the requests received by the interface, and they will all try to respond it, which will create a conflict.

!!! note
    **Only one TCP server can be connected to a single interface.** If you need to run several Modbus TCP servers on the same device, the correct way is to use different TCP servers and use a different port for each server.


### Server: single server shared by multiple interfaces

**✅ Fully supported since v1.1.6**

A single server instance can accept multiple interfaces simultaneously, allowing the same register map to be accessible through different protocols:

```cpp
// Create ONE server with multiple interfaces (any mix of RTU/TCP)
Modbus::Server server({&tcpIface, &rtuIface}, store, 1);
server.begin(); // Initializes all provided interfaces automatically
```

By design, the server accepts any set of interfaces of any type (RTU/TCP mix) up to `MAX_INTERFACES` limit (2 by default). The server handles interface initialization and ensures thread-safe concurrent access through an internal mutex.

See [Modbus Server (Slave)](301-modbus-server-slave.md#multi-interface-setup) for detailed configuration and usage.

### Client: Multiple clients on one interface

**⚠️ Possible but not standard (RTU ONLY)**

While the Modbus specification doesn’t formally support multiple masters on the same bus, EZModbus doesn’t enforce this limitation at the software level:

```cpp
// Create one UART & RTU interface
ModbusHAL::UART uart(Serial2, 9600, SERIAL_8N1);
ModbusInterface::RTU rtuIface(uart, Modbus::MASTER);

// Create multiple clients
Modbus::Client temperatureClient(rtuIface);
Modbus::Client controlClient(rtuIface);

// Initialize everything
uart.begin();
temperatureClient.begin();
controlClient.begin();
```

!!! note
    **Important Note**: only one client can issue a request at a time. The others will get an `ERR_BUSY` error when calling `sendRequest()`.

**✅ Supported (TCP)**

The TCP HAL wrapper manages sockets independently, so you can have as many clients as you want on a single interface. However, only one TCP transaction can be active at a time, so you need to make sure to wait for the previous request to complete before sending a new one, or you will get an `ERR_BUSY` error when trying to send it.

### Bridge: multiple Bridges & combination of Bridge and Client/Server

**✅  Possible: combination of Bridge and Client/Server**

Each end of the bridge acting with a proper interface, you can totally mix bridge and client/server on the same interface, as long as the case you are trying to achieve is supported by the previous statements.

For example, you could have a unique TCP/UART connection to a bus formed of multiple slave devices, and access some of those devices from your code or act as another slave, and  expose the bus to external TCP clients through the bridge. However, the same limitations apply: only one request can be ongoing at a time.

## Using multiple physical interfaces

If you need to communicate with separate Modbus networks from the same MCU, simply create multiple interface instances for each physical connection:

```cpp
// Create multiple UART instances for two separate ports
ModbusHAL::UART uart1(Serial1, 9600, SERIAL_8N1);
ModbusHAL::UART uart2(Serial2, 115200, SERIAL_8E2);

// Create RTU interfaces
ModbusInterface::RTU rtuIface1(uart1, Modbus::CLIENT);
ModbusInterface::RTU rtuIface2(uart2, Modbus::SERVER); 

// Create Modbus Client & Server instances
Modbus::Client client(rtuIface1);
Modbus::StaticWordStore<500> store;
Modbus::Server server(rtuIface2, store, 1);

// Add Words to the store here...

// Initialize UARTs & Modbus client/server instances separately
uart1.begin();
uart2.begin();
client.begin();
server.begin();
```

This approach gives you complete flexibility to connect your ESP32 to multiple Modbus networks simultaneously, with each interface operating independently.
