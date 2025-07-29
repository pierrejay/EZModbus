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

The EventBus captures Modbus operations and errors in a lightweight queue system. It's designed for production use where you need to monitor Server operations that don't return direct success/failure status.

**Features:**

- Non-blocking FreeRTOS queue implementation
- Thread-safe (callable from ISR and tasks)
- Zero overhead when disabled (no-op templates)
- No dynamic allocation or string formatting

**Event Types:**

- `EVENT_ERROR`: Error/exception events with result codes
- `EVENT_REQUEST`: Successful request events with frame metadata

### Usage

#### 1. Enable EventBus

Add the compile flag to your project:

```cpp
#define EZMODBUS_EVENTBUS
```

Or in `platformio.ini`:
```ini
build_flags = -DEZMODBUS_EVENTBUS
```

#### 2. Initialize the EventBus

```cpp
#include "EZModbus.h"

void setup() {
    // Initialize EventBus (called automatically on first use)
    Modbus::EventBus::begin();
}
```

#### 3. Consume Events

Create a task to process events from the queue:

```cpp
void eventTask(void* param) {
    constexpr uint32_t EVT_POP_WAIT_MS = 100;
    
    while (1) {
        Modbus::EventBus::Record evt;
        if (Modbus::EventBus::pop(evt, EVT_POP_WAIT_MS)) {
            // Format timestamp
            float timestamp = evt.timestampUs / 1000000.0f;
            
            if (evt.eventType == Modbus::EVENT_REQUEST) {
                // Successful request
                printf("[%.3f] REQUEST: %s addr=%u count=%u\n",
                       timestamp,
                       Modbus::toString(evt.requestInfo.fc),
                       evt.requestInfo.regAddress,
                       evt.requestInfo.regCount);
            } else {
                // Error event
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

**Example Output:**
```
[12.345] REQUEST: Read holding registers addr=100 count=5
[12.367] REQUEST: Write single register addr=105 count=1
[12.890] ERROR: Illegal data address (Register 999 not found)
[13.123] REQUEST: Read coils addr=0 count=8
```

### Event Record Structure

```cpp
struct Record {
    // Metadata
    EventType eventType;        // EVENT_ERROR or EVENT_REQUEST
    // Payload (for EVENT_ERROR)
    uint16_t result;           // Error/result code
    const char* resultStr;     // Result string (static)
    const char* desc;          // Additional description (static)
    // Payload (for EVENT_REQUEST)
    Modbus::FrameMeta requestInfo; // Frame metadata
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
