# Introduction

## Why EZModbus?

If you’ve struggled with ArduinoModbus, ESP-Modbus or other libraries, you’ll appreciate how EZModbus addresses common pain points:

* **Developer-friendly**: Eliminates common frustrations and tedious boilerplate with a simple & intuitive API.
* **Asynchronous, event-driven design**: Built on FreeRTOS primitives (tasks, mutexes, queues, notifications) to deliver high responsiveness without continuous polling. Synchronous (blocking) & asynchronous Client API to integrate with all requirements. All public methods are thread-safe & no-lock.
* **Modular architecture**: Separates I/O, transport, codec, and application layers so you can mix and match RTU, TCP, client, server, and bridge components with minimal coupling. Support bridges between any two interfaces, several servers running on the same device sharing a single interface, as well as custom codecs & transports (e.g. Modbus JSON over HTTP) to extend its capabilities.
* **Native C++ design**: Built from scratch in C++ rather than wrapping older C libraries like libmodbus, bringing proper type-safety with a legible & maintainable codebase.

## What can you do with EZModbus?

* Build responsive Modbus clients that communicate with PLCs, sensors, and industrial equipment
* Create robust Modbus servers that expose your device’s functionality to industrial networks
* Bridge legacy RTU equipment to modern TCP/IP networks with minimal configuration
* Develop custom Modbus variants for specialized applications (Modbus over HTTP, Modbus to DMX…)

Whether you’re building a simple RTU thermostat client, a TCP-based greenhouse controller, or a bridge between legacy RTU devices and WiFi/Ethernet networks, EZModbus gives you the building blocks - and lets you focus on your application logic, not protocol plumbing.

## Concept in 60 seconds

EZModbus is built around a simple workflow:

1. **Set up a transport** - Create a physical (UART/TCP) & Modbus (RTU/TCP) interface with your hardware configuration
2. **Set up the application layer** - `Client` (master) to initiate requests, `Server` (slave) to respond to them, or `Bridge` to connect two interfaces
3. **Link them together** - Pass the interface to your application class
4. **Initialize the physical link** - You stay in control of I/O peripherals, no hidden logic or ownership in the library
5. **Begin & let it run** - Call `begin()` on EZModbus components and FreeRTOS will take over to manage the operations in the background
