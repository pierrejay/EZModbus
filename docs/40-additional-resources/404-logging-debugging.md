# Logging/debugging

EZModbus includes a debug mode that can be enabled by defining the `EZMODBUS_DEBUG` flag in your project. When debug is enabled, the library will print detailed information about the Modbus protocol, including frame contents (human-readable output & raw hexdumps from the Codec layer), round-trip timing and errors (more on this earlier in the `Result` type section).

Example of log messages:
```
# Errors & informations
[ModbusCodec.h::isValidFrame:203] Error: Invalid function code
[ModbusClient.cpp::sendRequest:141] Error: Invalid frame (Invalid fields)
[ModbusRTU.cpp::setSilenceTimeMs:141] Silence time set to 2 ms (2000 us)

# Frame dump
[ModbusRTU.cpp::processReceivedFrame:317] Received raw data (8 bytes) from port 2
[ModbusRTU.cpp::processReceivedFrame:318] Hexdump: 01 03 27 0F 00 01 BE BD 
[ModbusRTU.cpp::processReceivedFrame:341] Received frame successfully decoded:
> Type           : REQUEST
> Function code  : 0x03 (Read holding registers)
> Slave ID       : 1
> Register Addr  : 9999
> Register Count : 1
> Data           : 0x0000 
```

Due to the multi-threaded nature of the library, EZModbus uses a custom thread-safe log sink (`Modbus::Logger`) internally, through log helpers defined in an utility file (`ModbusDebug.hpp`). When debug is disabled, the methods are all neutralized by the define flags, in order to completely remove any overhead (even evaluating strings in `LOG_X()` arguments).

## Enable debug logging

To enable debug, add the `EZMODBUS_DEBUG` compile flag. Refer to the section [Settings (compile flags)](../40-additional-resources/401-settings-compile-flags.md) for PlatformIO & IDF guidelines.

## Default outputs & settings

- **Arduino/PlatformIO:** logs go to `Serial` by default (i.e. USB CDC on most ESP32 boards). You can redirect to another `HardwareSerial` with `EZMODBUS_LOG_OUTPUT`.

- **ESP-IDF:** logs go to the IDF console (`printf` to stdout). The console destination is configured in menuconfig: `ESP System Settings → Channel for console output` (USB CDC/JTAG or UART). No direct UART driver handling is performed by EZModbus anymore since v1.1.5.

Other debug settings are explained in the [Settings (compile flags)](../40-additional-resources/401-settings-compile-flags.md) section.