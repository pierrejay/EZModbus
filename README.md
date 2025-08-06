## Introduction

EZModbus is a C++ Modbus RTU/TCP library based on FreeRTOS, initially designed for ESP32 projects on Arduino & ESP-IDF, and extended for RPi Pico & STM32 platforms.

It aims to offers a refreshing alternative to other Modbus implementations by prioritizing user experience, flexibility and efficiency, with a fully asynchronous & event-driven approach:

- Compatible with ESP32 (ESP-IDF & Arduino frameworks), RPi Pico (Pico SDK) & STM32 (STM32CubeMX)
- Support for Modbus RTU (UART/RS-485) & Modbus TCP (netif)
- Client, Server & Bridge components included, compatible with both protocols
- Easy coupling of components to suit any Modbus application needs
- Thread-safe & no-lock public API, easy to implement with "expert" use cases possible
- 100% static allocation by default
- Server allows safe advertising of data encoded on several registers (e.g. IEEE 754 floats)
- Comprehensive Unity test suite running on real hardware

## Current hardware support

|MCU|Framework|Version tested|RTU (UART)|TCP|
|--|--|--|--|--|
|ESP32|Arduino|3.0.0|✅|✅|
|ESP32|ESP-IDF|5.4.0|✅|✅ |
|STM32|STM32CubeMX|6.14.1|✅|⚙️ W5500|
|RP2040<br>RP2350|Pico SDK|2.1.1|✅|✅ CH9120<br>⚙️ W5500|

## Documentation

The docs are hosted on ReadTheDocs: [EZModbus Docs](https://ezmodbus.readthedocs.io/en/latest/).

## License

EZModbus is released under the MIT License. See the LICENSE file for details.

## Contributing

Contributions welcome!