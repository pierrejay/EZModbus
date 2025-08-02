# Modbus Bridge

The Modbus Bridge is a powerful component that transparently connects two different Modbus interfaces, allowing seamless communication between networks or protocols. This is ideal for integrating legacy RTU devices with modern TCP networks, or creating protocol converters for industrial environments.

## **Core functionality**

The Bridge acts as a bidirectional proxy between two interfaces:

* It forwards requests from a master interface to a slave interface
* It returns responses from the slave interface back to the master
* It preserves the original message content while converting the protocol format
* No programming logic is required beyond setup - forwarding happens automatically

## **Interface requirements**

The Bridge requires interfaces with complementary roles:

* One interface must be configured as `CLIENT`/`MASTER`
* The other interface must be configured as `SERVER`/`SLAVE`
* Both interface are initialized by calling the bridge’s `begin()` method
* The roles determine the direction of request and response forwarding

It relies on the same interface components as you would use for a client or server, so it works entirely in the background without polling or busy-waiting, efficiently utilizing system resources.

## Implementation example

```cpp
// Create instances of your physical interfaces with HAL
ModbusHAL::UART uart(RS485_SERIAL, RS485_BAUD, RS485_CONFIG);
ModbusHAL::TCP  tcpServer(PORT);

// Create interfaces with complementary roles
ModbusInterface::RTU rtuIface(uart, Modbus::CLIENT);  // RTU as master
ModbusInterface::TCP tcpIface(tcpServer, Modbus::SERVER);  // TCP as slave

// Initialize physical interfaces
uart.begin();
tcpServer.begin();

// Create and initialize bridge
Modbus::Bridge bridge(rtuIface, tcpIface);
bridge.begin();

// That's it! All communication now flows automatically
// Your loop() can focus on other application tasks
```

## **Bridge configuration**

When setting up a Modbus bridge to connect different networks, there are two critical concepts to understand:

* The bridge relies on the underlying interfaces, which means it follows the same sequential request pattern - **only one transaction can be active at a time**. This is a fundamental limitation of the Modbus protocol itself, not specific to EZModbus. If you have multiple TCP clients trying to communicate through the bridge simultaneously, the first arrived will be served, others will receive a `SLAVE_DEVICE_BUSY` exception. In RTU mode this should not happen since a multi-master topology is forbidden as per the Modbus specification (and not recommended anyway due to the RS485 half-duplex nature).
* When configuring the bridge, you must think from the perspective of your connection **relative to the host (microcontroller running EZModbus)**, not from the perspective of the device itself:
  * If you’re connecting to a Modbus slave device (like a sensor or actuator), that segment of your bridge must be configured as `MASTER`/`CLIENT`
  * If you’re connecting to a Modbus master device (like a PLC/SCADA system), that segment of your bridge must be configured as `SLAVE`/`SERVER`

This often creates confusion because you need to configure the interface with the **opposite role** of the device you’re connecting to:

```cpp
// CORRECT CONFIGURATION:
// Connecting to a SLAVE device over RTU, and exposing it to TCP masters
// 1. RTU as CLIENT to talk to a SLAVE
ModbusInterface::RTU rtuIface(uart, Modbus::CLIENT);  
// 2. TCP as SERVER to accept connection from CLIENTS
ModbusInterface::TCP tcpIface(tcpServer, Modbus::SERVER);  

// INCORRECT (wouldn't work):
// ModbusInterface::RTU rtuIface(uart, Modbus::SERVER);
```
