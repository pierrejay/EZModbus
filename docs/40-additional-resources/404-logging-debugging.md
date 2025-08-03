# Logging/debugging

EZModbus provides two complementary diagnostic systems:

- **Debug Console Logging** - Verbose console output ideal for development and troubleshooting
- **EventBus System** - Production-friendly event capture for monitoring errors and operations in real-time

## Debug Console Logging

EZModbus includes a debug mode that can be enabled by defining the `EZMODBUS_DEBUG` flag in your project. When debug is enabled, the library will print detailed information about the Modbus protocol, including frame contents (human-readable output & raw hexdumps from the Codec layer), round-trip timing and errors (more on this earlier in the `Result` type section).

To enable debug, you can set the flag in your project's `main.cpp` file before including the EZModbus header:

```cpp
#define EZMODBUS_DEBUG
```

Or on PlatformIO, in the `platformio.ini` file:

```ini
build_flags = -D EZMODBUS_DEBUG
```

Or via CMake in your project's `CMakeLists.txt`:

```cmake
# Add to your CMakeLists.txt after including EZModbus
target_compile_definitions(EZModbus PUBLIC
    EZMODBUS_DEBUG                      # Enable debug output
    EZMODBUS_LOG_OUTPUT=UART_NUM_1      # Optional: redirect to UART1 (ESP-IDF)
)
```

Debug mode uses a custom thread-safe logging system that's completely disabled when `EZMODBUS_DEBUG` is undefined, ensuring zero overhead in production builds.

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

1. **`EVENT_ERROR`**: Logged whenever an error occurs anywhere in the library
    - Triggered by any `Error()` call in the codebase
    - Contains error code, description, and context (file/line)
    - Examples: codec errors, interface errors, timeout errors

2. **`EVENT_REQUEST`**: Logged when a Modbus request transaction completes
    - **Client**: Logged after sending request and receiving response (or getting an error)
    - **Server**: Logged after receiving, decoding and processing a request
    - Contains request metadata (FC, address, count) and result code
    - Result indicates success or specific failure reason

!!! note
    - Failed transactions generate TWO events: `EVENT_REQUEST` with error code + `EVENT_ERROR` with detailed description
    - Server requests "not addressed to us" are silently dropped - no events logged
    - This dual logging provides complete context: what was attempted (REQUEST) and why it failed (ERROR)

### Usage

#### 1. Enable EventBus

Add the compile flag to your project (see [Settings (compile flags)](./401-settings-compile-flags.md))

#### 2. Initialize the EventBus

The EventBus must be initialized after the scheduler has started, otherwise FreeRTOS won't be able to create the queue.

```cpp
Modbus::EventBus::begin();
```

#### 3. Consume Events

Example boilerplate of a task that processes events from the queue and prints them to the console:

```cpp
void eventTask(void* param) {
    constexpr uint32_t EVT_POP_WAIT_MS = 100;
    
    while (1) {
        Modbus::EventBus::Record evt;
        if (Modbus::EventBus::pop(evt, EVT_POP_WAIT_MS)) {
            // Format timestamp
            float timestamp = evt.timestampUs / 1000000.0f;
            
            if (evt.eventType == Modbus::EVENT_REQUEST) {
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
    EventType eventType;       // EVENT_ERROR or EVENT_REQUEST
    // Payload (common to both event types)
    uint16_t result;           // Error/result code
    const char* resultStr;     // Result string
    // Error description (only for EVENT_ERROR)
    const char* desc;
    // Frame metadata (for EVENT_REQUEST)
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
