# Logging/debugging

EZModbus provides two complementary diagnostic systems:

- **Debug Console Logging** - Verbose console output with user-configurable output function
- **EventBus System** - Production-friendly event capture for monitoring errors and operations in real-time

## Debug Console Logging

EZModbus includes a flexible debug system that provides detailed information about Modbus protocol operations, frame contents, timing, and errors.

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

### Enabling Debug Mode

Debug mode is enabled by defining the `EZMODBUS_DEBUG` flag:

**PlatformIO**
```ini
; PlatformIO
build_flags = -D EZMODBUS_DEBUG
```

**CMake**
```cmake
# CMake
target_compile_definitions(EZModbus PUBLIC EZMODBUS_DEBUG)
```

When `EZMODBUS_DEBUG` is undefined, all debug calls are compiled out as no-op templates, ensuring zero overhead in production builds.

!!! warning
    You **MUST define a debug print function** in your code when `EZMODBUS_DEBUG` is defined (see next section). Otherwise you will get a compilation error.

### Configuring Output

Unlike previous versions, EZModbus no longer includes platform-specific log output code. Instead, you must provide a print function in your own code that handles the actual output. Rationale : 

- Maximum flexibility to handle logs output in any possible way (FIFO, DMA, etc.)
- Less platform-specific code in the library for non-mission-critical aspects (better readability, less risk of bugs, easier to maintain)
- Enables users to implement their own synchronization logic if needed, to properly interleave EZModbus logs with other messages.

The print function must implement the following contract:

```cpp
/**
 * @brief User-provided print function for EZModbus logs
 * @param msg Message to print (null-terminated)
 * @param len Length of message (excluding null terminator)
 * @return int Status code:
 *   -1 : Error occurred, skip this message
 *    0 : Busy/would block, retry later
 *   >0 : Success, return number of characters printed
 */
int Modbus::Debug::printLog(const char* msg, size_t len);
```

Internal behavior:

- If less than `len` characters were printed, Debug will call the function again with the remaining portion until the entire message is sent (or timeout occurs)
- If the function returns `-1`, the message is skipped and not printed

### Platform Examples

#### ESP32 

##### Arduino - basic Serial output

```cpp
int ESP32_printLogSerial(const char* msg, size_t len) {
    size_t written = Serial.write(msg, len);
    Serial.flush();
    return (int)written;
}

int Modbus::Debug::printLog(const char* msg, size_t len) {
    return ESP32_printLogSerial(msg, len);
}
```

##### ESP-IDF - basic console output

```cpp
int ESP32_printLogConsole(const char* msg, size_t len) {
    int written = fwrite(msg, 1, len, stdout);
    fflush(stdout);
    return written;
}

int Modbus::Debug::printLog(const char* msg, size_t len) {
    return ESP32_printLogConsole(msg, len);
}
```

#### Raspberry Pi Pico

##### Basic UART
```cpp
#include "hardware/uart.h"

int Pico_printLogUART(const char* msg, size_t len) {
    int written = 0;
    for (size_t i = 0; i < len; i++) {
        if (uart_is_writable(uart1)) {
            uart_putc_raw(uart1, msg[i]);
            written++;
        } else {
            break;  // UART busy, stop here
        }
    }
    return written;
}

int Modbus::Debug::printLog(const char* msg, size_t len) {
    return Pico_printLogUART(msg, len);
}
```

##### USB Serial (stdio)
```cpp
int Pico_printLogUSB(const char* msg, size_t len) {
    return printf("%.*s", (int)len, msg);
}

int Modbus::Debug::printLog(const char* msg, size_t len) {
    return Pico_printLogUSB(msg, len);
}
```

#### STM32

##### Basic UART transmit (blocking)

```cpp
int STM32_printLogBasic(const char* msg, size_t len) {
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
    return (status == HAL_OK) ? len : -1;
}

int Modbus::Debug::printLog(const char* msg, size_t len) {
    return STM32_printLogBasic(msg, len);
}
```

### Configuration

The debug system can be configured via compile-time defines:

```cpp
#define EZMODBUS_LOG_Q_SIZE 32              // Queue size (messages)
#define EZMODBUS_LOG_MAX_MSG_SIZE 256       // Max message size (bytes)
#define EZMODBUS_LOG_TASK_PRIORITY 1        // Task priority
#define EZMODBUS_LOG_TASK_STACK_SIZE 4096   // Task stack size (bytes)
```

See [Settings (compile flags)](./401-settings-compile-flags.md#modbusdebughpp--modbuslogsinkhpp) for details.

## EventBus

The EventBus captures Modbus operations and errors in a lightweight queue system. It's designed for production use where you need to monitor both Client and Server operations with minimal performance impact.

**Features**

- Non-blocking static FreeRTOS queue (no heap allocation)
- Thread-safe (callable from ISR and tasks)
- Zero overhead when disabled (no-op templates)
- Minimal CPU overhead: ~100-400 cycles per transaction (~1-2µs on typical MCUs)
- Configurable queue size via `EZMODBUS_EVENTBUS_Q_SIZE` (default: 16 events)

**Event Types**

The EventBus logs two types of events:

1. **`EVT_RESULT`**: Logged whenever an error occurs anywhere in the library
    - Triggered by any `Error()` call in the codebase
    - Contains error code, description, and context (file/line)
    - Examples: codec errors, interface errors, timeout errors

2. **`EVT_REQUEST`**: Logged when a Modbus request transaction completes
    - **Client**: Logged after sending request and receiving response (or getting an error)
    - **Server**: Logged after receiving, decoding and processing a request
    - Contains request metadata (FC, address, count) and result code
    - Result indicates success or specific failure reason

!!! note
    - Failed transactions generate at least TWO events: `EVT_REQUEST` with API-side error code + one `EVT_RESULT` per error that occured along the internal call chain
    - Server requests "not addressed to us" are silently dropped - no events logged
    - This dual logging provides complete context: what was attempted (REQUEST) and why it failed (RESULT)

### Event Record Structure

Each event is an instance of a `Modbus::EventBus::Record` structure, which contains the following fields:

```cpp
struct Record {
// Event classification
    EventType eventType;            // EVT_RESULT or EVT_REQUEST
// Payload
    uint16_t result;                // Error/result code (enum casted)
    const char* resultStr;          // toString(enum) - static string
    Modbus::FrameMeta requestInfo;  // Request metadata (POD) - only for EVT_REQUEST
// Context / origin
    uintptr_t instance;             // Caller instance address
    uint64_t timestampUs;           // TIME_US()
    const char* fileName;           // Basename
    uint16_t lineNo;                // Line number
};
```

The `Modbus::FrameMeta` structure contains only metadata of a `Modbus::Frame` object (without payload):

```cpp
struct FrameMeta {
    Modbus::MsgType type;       // Message type (request/response)
    Modbus::FunctionCode fc;    // Function code
    uint8_t slaveId;            // Origin/target slave ID
    uint16_t regAddress;        // Base register address
    uint16_t regCount;          // Number of registers read/written
};
```

### Usage

#### 1. Enable EventBus

- Add the compile flag to your project (see [Settings (compile flags)](./401-settings-compile-flags.md))
- The EventBus will be automatically initialized when the first event is pushed, you don't need to call `begin()` manually.

#### 2. Consume Events

**Single Consumer API**

The EventBus provides one method for reading events:

```cpp
static bool pop(Record& rcd, uint32_t timeoutMs = 0);
```

- **Pending behavior**: When `timeoutMs > 0`, the method yields CPU control and lets other tasks execute during the wait period
- **Non-blocking**: When `timeoutMs = 0`, returns immediately if no events are available
- **Return value**: `true` if an event was retrieved, `false` on timeout or no events

**Usage Pattern Example**

Example boilerplate of a task that processes events from the queue and prints them to the console:

```cpp
void eventTask(void* param) {
    constexpr uint32_t EVT_POP_WAIT_MS = 100;

    while (1) {
        // Pop an event from the event bus
        Modbus::EventBus::Record evt;
        if (Modbus::EventBus::pop(evt, EVT_POP_WAIT_MS)) {
            // Format timestamp in seconds
            float timestamp = (float)(evt.timestampUs / 1000000.0f);
            // Log the event
            if (evt.eventType == Modbus::EventBus::EVT_REQUEST) {
                // Request event - show function code, address, count
                printf("[%3f] REQUEST: %s addr=%u count=%u -> %s\n", timestamp,
                Modbus::toString(evt.requestInfo.fc), evt.requestInfo.regAddress, 
                evt.requestInfo.regCount, evt.resultStr);
            } else {
                // Error event
                printf("[%3f] ERROR: %s\n", timestamp, evt.resultStr);
            }
        }
    }
}

// Create the task
xTaskCreate(eventTask, "EventTask", 2048, NULL, 1, NULL);
```

**Example Output - Normal Operations:**
```
[12.345] REQUEST: Read holding registers addr=100 count=5 -> Success
[12.367] REQUEST: Write single register addr=105 count=1 -> Success
[13.123] REQUEST: Read coils addr=0 count=8 -> Success
```

**Example Output - Error Scenarios:**
```
// Client timeout - generates both REQUEST and ERROR events
[14.567] REQUEST: Read input registers addr=200 count=10 -> Timeout
[14.567] ERROR: Timeout

// Server illegal address - generates both events
[15.234] REQUEST: Write multiple registers addr=999 count=5 -> Illegal function in received frame
[15.234] ERROR: Illegal function in received frame

// Interface error - only ERROR event (no request was made)
[16.789] ERROR: TX failed (UART buffer full)
```

### Advanced Features

#### Instance Filtering

Filter out events from specific instances to reduce noise:

```cpp
// Filter out events from a specific server instance
Modbus::Server myServer(interface, store, 1);
Modbus::EventBus::filterOut(&myServer);
```

#### Dropped Event Monitoring

Monitor queue overflow:

```cpp
uint32_t dropped = Modbus::EventBus::getDroppedCount();
if (dropped > 0) {
    printf("Warning: %u events were dropped\n", dropped);
}
```

#### Configuration & Performance

The EventBus uses static FreeRTOS queues with minimal overhead (~100-400 CPU cycles per event, < 0.01% transaction impact).

Configure queue sizes via compile flags - see [Settings (compile flags)](./401-settings-compile-flags.md#modbuseventbushpp) for details:

- `EZMODBUS_EVENTBUS_Q_SIZE` - Event queue size (default: 16)
- `EZMODBUS_EVENTBUS_INSTANCE_FILTER_SIZE` - Instance filter slots (default: 8)