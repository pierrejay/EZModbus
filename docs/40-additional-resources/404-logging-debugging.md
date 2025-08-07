# Logging/debugging

EZModbus provides two complementary diagnostic systems:

- **Debug Console Logging** - Verbose console output with user-configurable output function
- **EventBus System** - Production-friendly event capture for monitoring errors and operations in real-time

## Debug Console Logging

EZModbus includes a flexible debug system that provides detailed information about Modbus protocol operations, frame contents, timing, and errors.

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

### Configuring Output

Unlike previous versions, EZModbus no longer includes platform-specific output code. Instead, you must provide a print function that handles the actual output:

**1. Using `setPrintFunction()`**

```cpp
// Define your print function
int MyLogPrint(const char* msg, size_t len) {
    // Your output implementation here
    // Return: >0 = success (bytes written), 0 = busy/retry, -1 = error
    //
    // If multiplexing user console & logs on a shared UART, implement
    // synchronization (Mutex, Semaphore, etc.) to avoid mixing characters
}

// Configure EZModbus to use your function in setup(), main() or your task
void setup() {
    Modbus::Debug::setPrintFunction(MyLogPrint);
    // ... rest of initialization
}
```

**2. Using `PrintFunctionSetter`(RAII)**

With this approach, you can register the print function before runtime, in the global scope, before entering `setup()` or `main()`.

```cpp
// Define your print function
int MyLogPrint(const char* msg, size_t len) {
    // Your output implementation here
    return len;
}

// Register the print function before runtime
static Modbus::Debug::PrintFunctionSetter func(MyLogPrint);

void main() {
    // Print function is already configured!
}
```

This gives the maximum flexibility to handle logs output in any possible way (FIFO, DMA, etc.) without platform-specific code in the library (less risk of bugs, easier to maintain).

### Print Function Contract

The print function must implement the following contract:

```cpp
int MyPrintFunction(const char* msg, size_t len);
```

- `msg`: message to print (null-terminated)
- `len`: length of message (excluding null terminator)
- Return value:
    - `-1`: Error occurred, skip this message
    - `0`: Busy/would block, retry later
    - `>0`: Success, number of characters printed

Internal behavior:

- If less than `len` characters were printed, Debug will call the function again with the remaining portion until the entire message is sent (or timeout occurs)
- If the function returns `-1`, the message is skipped and not printed

### Platform Examples

#### ESP32 

##### Arduino - basic Serial output

```cpp
int ESP32_LogPrint_Serial(const char* msg, size_t len) {
    size_t written = Serial.write(msg, len);
    Serial.flush();
    return (int)written;
}

static Modbus::Debug::PrintFunctionSetter func(ESP32_LogPrint_Serial);
```

##### ESP-IDF - basic console output

```cpp
int ESP32_LogPrint_Console(const char* msg, size_t len) {
    int written = fwrite(msg, 1, len, stdout);
    fflush(stdout);
    return written;
}

static Modbus::Debug::PrintFunctionSetter func(ESP32_LogPrint_Console);
```

#### Raspberry Pi Pico

##### Basic UART
```cpp
#include "hardware/uart.h"

int Pico_LogPrint_UART(const char* msg, size_t len) {
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

static Modbus::Debug::PrintFunctionSetter func(Pico_LogPrint_UART);
```

##### USB Serial (stdio)
```cpp
int Pico_LogPrint_USB(const char* msg, size_t len) {
    return printf("%.*s", (int)len, msg);
}

static Modbus::Debug::PrintFunctionSetter func(Pico_LogPrint_USB);
```

#### STM32

##### Basic UART transmit (blocking)

```cpp
int STM32_LogPrint_Basic(const char* msg, size_t len) {
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
    return (status == HAL_OK) ? len : -1;
}

static Modbus::Debug::PrintFunctionSetter func(STM32_LogPrint_Basic);
```

### System Features

#### Thread Safety
The debug system uses a FreeRTOS queue and dedicated log task to ensure thread-safe operation:

- Log calls from any context are queued
- A dedicated task processes messages sequentially
- User print function is always called from the same task context

#### Timeout Protection
The system includes timeout protection in the internal log task:

- 500ms timeout per message (configurable via `LOG_PRINT_TIMEOUT_MS`)
- Timeout is **reset on each successful chunk** - slow but progressing transmissions won't be cut
- Different timing functions: `TIME_US()` before scheduler, `TIME_MS()` after scheduler

#### Message Formatting
All log messages are automatically formatted with:

- Consistent line endings (exactly one `\r\n` per message)
- Context information `[file::function:line]` for debug messages
- Automatic truncation if messages exceed buffer size

#### Performance

- **Caller side**: Only formatting, no I/O blocking
- **Task side**: Handles all I/O and retry logic
- **Zero overhead** when `EZMODBUS_DEBUG` is undefined

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
    - Failed transactions generate TWO events: `EVT_REQUEST` with error code + `EVT_RESULT` with detailed description
    - Server requests "not addressed to us" are silently dropped - no events logged
    - This dual logging provides complete context: what was attempted (REQUEST) and why it failed (ERROR)

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
        Modbus::EventBus::Record evt;
        if (Modbus::EventBus::pop(evt, EVT_POP_WAIT_MS)) {
            // Format timestamp
            float timestamp = evt.timestampUs / 1000000.0f;
            
            if (evt.eventType == Modbus::EVT_REQUEST) {
                // Request event - show result status
                const char* statusStr = (evt.result == 0) ? "OK" : evt.resultStr;
                printf("[%.3f] REQUEST: %s addr=%u count=%u | %s\n",
                       timestamp,
                       Modbus::toString(evt.requestInfo.fc),
                       evt.requestInfo.regAddress,
                       evt.requestInfo.regCount,
                       statusStr);
            } else {
                // Error event - show error details
                printf("[%.3f] ERROR: %s (%s)\n",
                       timestamp,
                       evt.resultStr,
                       evt.desc ? evt.desc : "");
            }
        }
    }
}

// Create the task
xTaskCreate(eventTask, "EventTask", 2048, NULL, 1, NULL);
```

**Example Output - Normal Operations:**
```
[12.345] REQUEST: Read holding registers addr=100 count=5 | OK
[12.367] REQUEST: Write single register addr=105 count=1 | OK
[13.123] REQUEST: Read coils addr=0 count=8 | OK
```

**Example Output - Error Scenarios:**
```
// Client timeout - generates both REQUEST and ERROR events
[14.567] REQUEST: Read input registers addr=200 count=10 | Timeout
[14.567] ERROR: Timeout (No response received within 1000ms)

// Server illegal address - generates both events
[15.234] REQUEST: Write multiple registers addr=999 count=5 | Illegal data address
[15.234] ERROR: Illegal data address (Register 999 not found in store)

// Interface error - only ERROR event (no request was made)
[16.789] ERROR: TX failed (UART buffer full)
```

### Event Record Structure

```cpp
struct Record {
    // Metadata
    EventType eventType;       // EVT_RESULT or EVT_REQUEST
    // Payload (common to both event types)
    uint16_t result;           // Error/result code
    const char* resultStr;     // Result string
    // Error description (only for EVT_RESULT)
    const char* desc;
    // Frame metadata (for EVT_REQUEST)
    Modbus::FrameMeta requestInfo;
    // Context
    uintptr_t instance;        // Source instance address
    uint64_t timestampUs;      // Microsecond timestamp
    const char* fileName;      // Source file name
    uint16_t lineNo;           // Source line number
};
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