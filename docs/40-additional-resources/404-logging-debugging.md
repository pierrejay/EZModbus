# Logging/debugging

EZModbus includes a debug mode that can be enabled by defining the `EZMODBUS_DEBUG` flag in your project. When debug is enabled, the library will print detailed information about the Modbus protocol, including frame contents (human-readable output & raw hexdumps from the Codec layer), round-trip timing and errors (more on this earlier in the `Result` type section).

To enable debug, you can set the flag in your project's `main.cpp` file before including the EZModbus header:

```cpp
#define EZMODBUS_DEBUG
```

Or on PlatformIO, in the `platformio.ini` file:

```ini
build_flags = -D EZMODBUS_DEBUG
```

Or add `EZMODBUS_DEBUG` to the `target_compile_definitions` of your main project's `CMakeLists.txt` file.

Due to the multi-threaded nature of the library, EZModbus uses a custom thread-safe log sink (`Modbus::Logger`) internally, through log helpers defined in an utility file (`ModbusDebug.h`). When debug is disabled, the methods are all neutralized by the define flags, in order to completely remove any overhead (even evaluating strings in `LOG_X()` arguments). This was chosen instead of the native ESP logging system due to its blocking nature, it is anyway not required in production as you can implement your own logs based on the error returns from the library.

By default, logs are printed to the default Serial port on Arduino (`Serial`, usually USB CDC on most ESP32 boards) or `UART_NUM_0` port for ESP-IDF. You can redirect them with the `EZMODBUS_LOG_OUTPUT` flag:

```cpp
// In your code:
#define EZMODBUS_DEBUG // Enables debug
#define EZMODBUS_LOG_OUTPUT Serial1 // Prints logs to Serial1
```

```ini
; In your platformio.ini:
build_flags = 
  -D EZMODBUS_DEBUG ; Enables debug
  -D EZMODBUS_LOG_OUTPUT=Serial1 ; Prints logs to Serial1
```
