# Logging/debugging

EZModbus provides two complementary diagnostic systems:

- **Debug Console Logging** - Verbose console output with user-configurable output function
- **EventBus System** - Production-friendly event capture for monitoring errors and operations in real-time

## Debug Console Logging

EZModbus includes a flexible debug system that provides detailed information about Modbus protocol operations, frame contents, timing, and errors. The system is designed to be platform-agnostic and allows complete user control over log output.

### Architecture

The debug system provides a unified API through the `Modbus::Debug` namespace:

- **Public API**: `Modbus::Debug::setPrintFunction()` to set a print callback function
- **Internal Implementation**: Message formatting, queuing, threading, and output handled transparently

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

It gives the maximum flexibility to handle logs output in any possible way (FIFO, DMA, etc.) without platform-specific code in the library (less risk of bugs, lower maintenance cost).

#### RAII Configuration Helper

For convenience, EZModbus provides a RAII helper class that automatically registers the print function during global initialization (before entering `setup()`, `main()` = before runtime):



This approach ensures the logging system is configured early, even before `main()` execution, which is useful for debugging initialization issues or global constructor logging.

### Print Function Contract

The print function must implement the following contract:

```cpp
/**
 * User-provided print function for EZModbus logs
 * @param msg Message to print (null-terminated)
 * @param len Length of message (excluding null terminator)
 * @return int Status code:
 *   -1 : Error occurred, skip this message
 *    0 : Busy/would block, retry later
 *   >0 : Success, number of characters printed
 *        If less than 'len' characters were printed, Debug
 *        will call the function again with the remaining portion
 *        until the entire message is sent.
 */
int MyPrintFunction(const char* msg, size_t len);
```

### Platform Examples

#### STM32 HAL

##### Basic UART Transmit
```cpp
int STM32_LogPrint_Basic(const char* msg, size_t len) {
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
    return (status == HAL_OK) ? len : -1;
}

// Configure EZModbus to use this function
void setup() {
    Modbus::Debug::setPrintFunction(STM32_LogPrint_Basic);
    // ... rest of initialization
}
```

##### UART with Interrupt (IT)
```cpp
static volatile bool uart_tx_complete = true;
static ModbusTypeDef::Mutex uart_tx_mutex;

int STM32_LogPrint_IT(const char* msg, size_t len) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        ModbusTypeDef::Lock lock(uart_tx_mutex, 0);
        if (!lock.isLocked()) return 0; // Busy
        
        if (!uart_tx_complete) return 0; // Previous transmission not complete
        
        uart_tx_complete = false;
        HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, len);
        
        if (status != HAL_OK) {
            uart_tx_complete = true;
            return -1;
        }
        
        // Wait for completion with timeout
        uint32_t start = HAL_GetTick();
        while (!uart_tx_complete && (HAL_GetTick() - start) < 100) {
            if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
                vTaskDelay(1);
            }
        }
        
        return uart_tx_complete ? len : 0; // Return success or busy
    } else {
        // Before scheduler: use blocking mode
        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
        return (status == HAL_OK) ? len : -1;
    }
}

// In UART callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        uart_tx_complete = true;
    }
}
```

##### UART with DMA
```cpp
static volatile bool uart_dma_complete = true;
static ModbusTypeDef::Mutex uart_dma_mutex;

int STM32_LogPrint_DMA(const char* msg, size_t len) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        ModbusTypeDef::Lock lock(uart_dma_mutex, 0);
        if (!lock.isLocked()) return 0; // Busy
        
        if (!uart_dma_complete) return 0; // DMA busy
        
        uart_dma_complete = false;
        HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, len);
        
        if (status != HAL_OK) {
            uart_dma_complete = true;
            return -1;
        }
        
        // Return immediately - completion handled in callback
        return len;
    } else {
        // Before scheduler: use blocking mode
        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);
        return (status == HAL_OK) ? len : -1;
    }
}

// In DMA callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        uart_dma_complete = true;
    }
}

// Configure EZModbus to use this function
void setup() {
    Modbus::Debug::setPrintFunction(STM32_LogPrint_DMA);
    // ... rest of initialization
}
```

#### ESP32 (ESP-IDF)

##### Classic UART with FIFO
```cpp
#include "driver/uart.h"

int ESP32_LogPrint_FIFO(const char* msg, size_t len) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        // Non-blocking write attempt
        int written = uart_write_bytes(UART_NUM_1, msg, len);
        if (written < 0) return -1; // Error
        if (written == 0) return 0; // FIFO full, retry later
        return written; // Partial or complete write
    } else {
        // Before scheduler: blocking write
        int written = uart_write_bytes(UART_NUM_1, msg, len);
        if (written > 0) {
            uart_wait_tx_done(UART_NUM_1, portMAX_DELAY);
        }
        return (written > 0) ? written : -1;
    }
}
```

##### Console Output (Basic)
```cpp
int ESP32_LogPrint_Console(const char* msg, size_t len) {
    // ESP32 console is thread-safe
    int written = fwrite(msg, 1, len, stdout);
    fflush(stdout);
    return (written > 0) ? written : -1;
}

// Configure EZModbus to use this function
void setup() {
    Modbus::Debug::setPrintFunction(ESP32_LogPrint_Console);
    // ... rest of initialization
}
```


#### Raspberry Pi Pico (Pico SDK)

##### Basic UART
```cpp
#include "hardware/uart.h"

int Pico_LogPrint_UART(const char* msg, size_t len) {
    // Pico UART functions are not thread-safe, use mutex if needed
    for (size_t i = 0; i < len; i++) {
        uart_putc_raw(uart1, msg[i]);
    }
    return len;
}
```

##### UART with Simple DMA (TX-only)
```cpp
#include "hardware/dma.h"
#include "hardware/uart.h"

static int dma_channel = -1;
static volatile bool dma_complete = true;
static char dma_buffer[256]; // Static buffer for DMA

int Pico_LogPrint_DMA(const char* msg, size_t len) {
    if (dma_channel < 0) {
        // Initialize DMA channel once
        dma_channel = dma_claim_unused_channel(true);
        dma_channel_config cfg = dma_channel_get_default_config(dma_channel);
        channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
        channel_config_set_read_increment(&cfg, true);
        channel_config_set_write_increment(&cfg, false);
        channel_config_set_dreq(&cfg, uart_get_dreq(uart1, true));
        dma_channel_configure(dma_channel, &cfg, &uart_get_hw(uart1)->dr, NULL, 0, false);
        
        // Setup interrupt
        dma_channel_set_irq0_enabled(dma_channel, true);
        irq_set_exclusive_handler(DMA_IRQ_0, dma_complete_handler);
        irq_set_enabled(DMA_IRQ_0, true);
    }
    
    if (!dma_complete) return 0; // DMA busy
    
    // Copy to DMA buffer (required for DMA safety)
    size_t copy_len = (len < sizeof(dma_buffer)) ? len : sizeof(dma_buffer);
    memcpy(dma_buffer, msg, copy_len);
    
    dma_complete = false;
    dma_channel_set_read_addr(dma_channel, dma_buffer, false);
    dma_channel_set_trans_count(dma_channel, copy_len, true); // Start DMA
    
    return copy_len;
}

void dma_complete_handler() {
    if (dma_irq_get_channel_status(0) & (1 << dma_channel)) {
        dma_irq_acknowledge_channel(0, dma_channel);
        dma_complete = true;
    }
}
```

##### USB Serial (stdio)
```cpp
int Pico_LogPrint_USB(const char* msg, size_t len) {
    // USB stdio via printf
    return printf("%.*s", (int)len, msg);
}

// Configure EZModbus to use this function
void setup() {
    Modbus::Debug::setPrintFunction(Pico_LogPrint_USB);
    // ... rest of initialization
}
```

### Mixing User Logs with EZModbus Logs

When sharing the same output device (UART, console, etc.) between EZModbus debug logs and your application logs, **synchronization is crucial** to prevent character mixing. Here's how to properly coordinate both:

#### Simple Unified Logging

```cpp
#include "EZModbus.h"

// Import Modbus synchronization types  
using Mutex = ModbusTypeDef::Mutex;
using Lock = ModbusTypeDef::Lock;

// Shared FreeRTOS mutex for all logs output
static Mutex logMutex;

// Unified logging function for both user and
// EZModbus using Mutex protection
void logFmt(const char* fmt, ...) {
    Lock guard(logMutex); // Waits for the mutex.
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
} // Mutex released here

// Trampoline callback for EZModbus (forwards to logFmt)
int printLog(const char* msg, size_t len) {
    Lock guard(logMutex);
    return printf("%.*s", (int)len, msg);
} // Mutex released here

// Configure EZModbus to use the callback
static Modbus::Debug::PrintFunctionSetter func(printLog);

void setup() {
    // User and EZModbus logs are now synchronized
    logFmt("App started - temp: %.1f°C\n", 23.5);
    
    // ... initialize EZModbus ...
    
    logFmt("Modbus server ready\n");
    // EZModbus debug logs will also use the same mutex via the callback
}
```

#### Output Example (Properly Synchronized)

```
App started - temp: 23.5°C
[ModbusServer.cpp::begin:63] Server ready
Modbus server ready  
[ModbusTCP.cpp::handleDecodedFrame:335] Server received request with MBAP_TID: 1
Sensor reading: 24.1°C
[ModbusServer.cpp::handleRequest:184] Request processed successfully
```

#### Without Synchronization (BAD Example)

```
App started - tem[ModbusServer.cpp::begin:63] Sp: 23.5°C
erver ready
Modbus server reSensor reading: 2ady4.1°C
[ModbusTCP.cpp::[ModbusServer.cpp::handleRequest:184] handlRequest processedeDecodedFrame:335] Server received request with MBAP_TID: d successfully1
```

### System Features

#### Thread Safety
The debug system uses a FreeRTOS queue and dedicated task to ensure thread-safe operation:

- Log calls from any context (task, ISR, main) are queued
- A dedicated task processes messages sequentially
- User print function is always called from the same task context

#### Timeout Protection
The system includes bulletproof timeout protection:

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

### Migration from Previous Versions

If migrating from older EZModbus versions:

1. **Update configuration call**: `Modbus::LogSink::setPrintFunction()` → `Modbus::Debug::setPrintFunction()`
2. **Remove old macros**: `EZMODBUS_LOG_OUTPUT`, `EZMODBUS_LOG_CHUNK_SIZE`, etc.
3. **Implement print function** using examples above
4. **Configure at startup**: `Modbus::Debug::setPrintFunction(MyPrintFunction);`

The debug API (`LOG_MSG`, `LOG_HEXDUMP`, `LOG_FRAME`) remains unchanged.

**Note**: `Modbus::LogSink` is now an internal implementation detail and should not be used directly.

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