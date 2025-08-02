# Welcome

## Welcome to the EZModbus documentation !

EZModbus is a **C++ Modbus RTU/TCP library based on FreeRTOS**, initially designed for ESP32 projects on Arduino & ESP-IDF, and extended for RPi Pico & STM32 platforms on their native SDKs.

It aims to offers a refreshing alternative to other Modbus implementations by prioritizing user experience, flexibility and efficiency, with a fully asynchronous & event-driven approach.

This documentation covers most aspects of the library from installation to advanced usage, with practical examples and API reference.

## Key features

- Compatible with ESP32 (ESP-IDF & Arduino frameworks), RPi Pico (Pico SDK) & STM32 (STM32CubeMX)
- Support for Modbus RTU (UART/RS-485) & Modbus TCP (netif, CH9120)
- Client, Server & Bridge components included, compatible with both protocols
- Easy coupling of components to suit any Modbus application needs
- Thread-safe & no-lock public API, easy to implement with "expert" use cases possible
- 100% static allocation by default
- Server allows safe advertising of data encoded on several registers (e.g. IEEE 754 floats)
- Comprehensive Unity test suite running on real hardware

## Repository & release version

* Repository URL : [https://github.com/pierrejay/EZModbus/](https://github.com/pierrejay/EZModbus/)
* Release version : `v1.1.3`

