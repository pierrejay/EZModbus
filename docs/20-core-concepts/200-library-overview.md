# Library overview

## Basic flow

<figure><img src="../.gitbook/assets/Capture d’écran 2025-06-14 à 17.28.04.png" alt=""><figcaption></figcaption></figure>

Once initialized, the library handles all the low-level details:

* TX/RX tasks manage communication timing and framing
* Callbacks inform your application when data arrives
* No polling required - everything runs in background tasks

## Features

* **Dual-Mode Transport**
  * **Modbus RTU** over RS-485 via UART, optional DE/RE control
  * **Modbus TCP** over WiFi/Ethernet/netif
* **Flexible Application Layer**
  * **Client**: synchronous/blocking or asynchronous reads/writes with result trackers or callbacks
  * **Server**: direct‐pointer or callback register access, automatic validation and exception handling
  * **Bridge**: link any two interfaces (RTU↔︎TCP, RTU↔︎RTU, TCP↔︎TCP) for transparent proxying
* **Standards Compliance**
  * 3.5-character silent intervals for RTU, broadcast rules, exception code semantics
  * Full coverage of all common Modbus Function Codes (0x01–0x10)
* **Pragmatic Memory Approach**
  * Zero dynamic allocation by default
  * Dynamic allocations optional: Server can use stack or controlled (no reallocations at runtime) heap storage for Modbus registers.
* **Comprehensive Examples & Tests**
  * Unity-based native tests for codec, hardware loopback tests for interface & application layers

## Namespaces & components

* `Modbus` : general namespace for core & application components, compatible with all Modbus protocol fashions
  * `Client` : Client application class
  * `Server` : Server application class
  * `Bridge` : Bridge application class
  * `Frame` : abstraction for a Modbus frame (used in Client)
  * `Word` : abstraction for exposed Modbus registers (used in Server)
  * `IWordStore`, `Static/DynamicWordStore`: Word storage containers
* `ModbusCodec` : frame encoding/decoding & value conversion
* `ModbusInterface` : manages Modbus transport with a generic approach
  * `IInterface` : abstract class for all Transport layers
  * `RTU` : transport layer class for Modbus RTU
  * `TCP` : transport layer class for Modbus TCP
* `ModbusHAL` : HAL wrappers for physical input/output management in an event-driven fashion
  * `UART` : wrapper around ESP-IDF UART API to manage an RS485 link
  * `TCP` : wrapper around ESP-IDF Socket API to manage TCP transactions across any Network interface (Ethernet, WiFi, PPP...)
