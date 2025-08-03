## Introduction

`EZModbus` is a Modbus library based on FreeRTOS, designed specifically for ESP32 projects on Arduino & native ESP-IDF frameworks. Built from the ground up with C++, it prioritizes developer experience, flexibility and performance through a fully asynchronous & event-driven approach.

- Compatible with both native ESP-IDF & Arduino frameworks
- Support for Modbus RTU (UART/RS-485) & Modbus TCP (netif)
- Client, Server & Bridge components included, compatible with both protocols
- Easy coupling of components to suit any Modbus application needs
- Thread-safe & no-lock public API, easy to implement with "expert" use cases possible
- 100% static allocation by default
- Server allows safe advertising of data encoded on several registers (e.g. IEEE 754 floats)
- Comprehensive Unity test suite running on real hardware

## Documentation

Complete documentation available on [Read the Docs](https://ezmodbus.readthedocs.io/) & in the `/docs` directory.

## License

EZModbus is released under the MIT License. See the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit PRs or open issues. I'm always looking for ideas to improve EZModbus's robustness, performance, and memory usage & safety. Even without a PR, your advice and suggestions are greatly appreciated - don't hesitate to share your thoughts or expertise.