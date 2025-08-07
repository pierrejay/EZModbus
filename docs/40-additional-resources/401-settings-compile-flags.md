# Settings (compile flags)

The following settings are editable via compile flags. Default settings are recommended for stability, use with caution.

## Usage

### PlatformIO (ESP32 Arduino)

In your `platformio.ini`, add compile flags to the `build_flags` section:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

build_flags =
    -DEZMODBUS_DEBUG                    ; Enable debug output
    -DEZMODBUS_EVENTBUS                 ; Enable EventBus system
    -DEZMODBUS_CLIENT_REQ_TIMEOUT=2000  ; Custom client timeout (2s)
    -DEZMODBUS_TCP_TXN_SAFETY_TIMEOUT=10000  ; Custom TCP safety timeout (10s)
```

### ESP-IDF (CMake)

Add to your project's main `CMakeLists.txt` or component file, **after** including EZModbus:

```cmake
# In your main CMakeLists.txt or component CMakeLists.txt
target_compile_definitions(EZModbus PUBLIC
    EZMODBUS_DEBUG                      # Enable debug output
    EZMODBUS_EVENTBUS                   # Enable EventBus system
    EZMODBUS_CLIENT_REQ_TIMEOUT=2000    # Custom client timeout (2s)
)
```

### Pico SDK (CMake)

In your project's `CMakeLists.txt`, **after** adding the EZModbus subdirectory but **before** linking:

```cmake
# Add EZModbus
add_subdirectory(path/to/EZModbus ezmodbus)

# Configure EZModbus compile definitions
target_compile_definitions(ezmodbus PUBLIC 
    EZMODBUS_DEBUG                      # Enable debug output
    EZMODBUS_EVENTBUS                   # Enable EventBus system
    EZMODBUS_CLIENT_REQ_TIMEOUT=2000    # Custom client timeout (2s)
)

# Link with your executable
target_link_libraries(your_executable
    pico_stdlib
    freertos
    ezmodbus
)
```

### STM32 (STM32CubeMX + CMake)

In your project's `CMakeLists.txt`, add definitions to the EZModbus target:

```cmake
# After including EZModbus as a subdirectory or component
target_compile_definitions(EZModbus PUBLIC
    EZMODBUS_DEBUG                      # Enable debug output  
    EZMODBUS_CLIENT_REQ_TIMEOUT=2000    # Custom client timeout (2s)
    EZMODBUS_HAL_UART_EVT_Q_SIZE=30     # Larger UART event queue
)
```

## Settings per file

### ModbusServer.h

* **`EZMODBUS_SERVER_MAX_WORD_SIZE`**
    * Max Word size (# Modbus registers per Word)
    * Default: 8

### ModbusClient.h

* **`EZMODBUS_CLIENT_REQ_TIMEOUT`**
    * Timeout before aborting a request if no response received from server (ms)
    * Default: 1000

### ModbusRTU.h

* **`EZMODBUS_RTU_TASK_STACK_SIZE`**
    * Stack size of RTU task (bytes)
    * Default: 2048, or 4096 if debug enabled
* **`EZMODBUS_RTU_TASK_PRIORITY`**
    * Priority of RTU task
    * Default: `tskIDLE_PRIORITY + 3`

### ModbusTCP.h

* **`EZMODBUS_TCP_TXN_SAFETY_TIMEOUT`**
    * Timeout before closing a transaction after 1st message if no response from server or client (ms)
    * Default: 5000
* **`EZMODBUS_TCP_TASK_STACK_SIZE`**
    * Stack size of TCP task (bytes)
    * Default: 4096, or 6144 if debug enabled
* **`EZMODBUS_TCP_TASK_PRIORITY`**
    * Priority of TCP task
    * Default: `tskIDLE_PRIORITY + 3`

### ModbusHAL_UART.h

* **`EZMODBUS_HAL_UART_EVT_Q_SIZE`**
    * Size of IDF UART event queue (# events)
    * Default: 20

### ModbusHAL_TCP.h

* **`EZMODBUS_HAL_TCP_RX_Q_SIZE`**
    * Size of TCP driver RX queue size (# frames)
    * Default: 16
* **`EZMODBUS_HAL_TCP_TASK_STACK_SIZE`**
    * Stack size of TCP driver task (bytes)
    * Default: 4096
* **`EZMODBUS_HAL_TCP_TASK_PRIORITY`**
    * Priority of TCP driver task
    * Default: `tskIDLE_PRIORITY + 4`

### ModbusDebug.hpp & ModbusLogSink.hpp

* **`EZMODBUS_DEBUG`**
    * Enables Debug logs
    * Default: undefined = no logs (no value, just a flag)
* **`EZMODBUS_LOG_Q_SIZE`**
    * Log message queue size (# messages)
    * Default: 16
* **`EZMODBUS_LOG_MAX_MSG_SIZE`**
    * Maximum length for a formatted message (char)
    * Default: 256
* **`EZMODBUS_LOG_TASK_PRIORITY`**
    * Priority of Log task
    * Default: `tskIDLE_PRIORITY + 1`
* **`EZMODBUS_LOG_TASK_STACK_SIZE`**
    * Stack size of Log task (bytes)
    * Default: 4096

### ModbusEventBus.hpp

* **`EZMODBUS_EVENTBUS`**
    * Enables EventBus system for production monitoring
    * Default: undefined = disabled (no value, just a flag)
* **`EZMODBUS_EVENTBUS_Q_SIZE`**
    * EventBus queue size (# events)
    * Default: 16
* **`EZMODBUS_EVENTBUS_INSTANCE_FILTER_SIZE`**
    * Number of instances that can be filtered out
    * Default: 8

