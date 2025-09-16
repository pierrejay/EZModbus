# Modbus Client (Master)

The Modbus Client implements the master device role, initiating requests to slave devices and processing their responses. It offers both synchronous ("blocking") and asynchronous (non-blocking) operation modes to suit different application needs.

## Synchronous mode ("blocking")

* Simpler to use - request and response in one function call
* Waits until the response arrives or a timeout occurs
* Perfect for sequential operations or simple applications

### Usage

The sync mode works exactly as in the former examples:

```cpp
Modbus::Frame request = { /* request details */ };
Modbus::Frame response;

auto result = client.sendRequest(request, response); // Only 2 args: sync
// sendRequest will wait until completion if no third argument is provided

if (result == Modbus::Client::SUCCESS) {
  // Process response here...
}
```

`sendRequest()` does not _really_ block execution as it uses a FreeRTOS event notification internally. It's actually pretty efficient when called from a dedicated task: instead of wasting CPU clocks by continuously polling for the request status, it will totally yield while waiting ; if you have other tasks running in parallel, they will run uninterrupted.

!!! note "Note on memory footprint"
    The two request/response frames have a total memory overhead of 536 bytes. If you create them inside a FreeRTOS task, make sure to have enough stack, or declare them `static`. When doing successive requests, it is possible to reuse the same request & response Frames between different calls to `sendRequest()`. 
    
    A "clever" trick to reduce memory usage, is to use the same `Frame` object for both request and response: internally, the Frame metadata & payload are stored before being sent on the bus so there is no race condition between reading your request and writing back the response.

## Asynchronous mode with Result tracker (non-blocking)

* Returns immediately after sending the request
* Allows your application to continue other tasks while waiting
* Uses a Result tracker to monitor request progress

### Usage

The async mode works by passing a pointer to a `Result` variable holding the transfer status while the transaction is ongoing:

```cpp
Modbus::Frame request = { /* request details */ };
Modbus::Frame response;
Modbus::Client::Result tracker; // Stores the request outcome

client.sendRequest(request, response, &tracker); // 3 args: async
// sendRequest returns immediately with `Modbus::Client::SUCCESS` 
// if the request is valid & queued for TX 

// Continue other operations while request is processed in background...
if (tracker == Modbus::Client::NODATA) {
 // Request is still being processed...
} else if (tracker == Modbus::Client::SUCCESS) {
 // Process response here...
} else {
 // Handle error or timeout...
}
```

This methods allows you to do other stuff while waiting for the response without blocking the calling thread. If you are just placing the status checks inside a busy-waiting loop in the same thread, it will be more efficient and less verbose to use the synchronous approach (see comment on blocking aspect).

### Understanding the request Result tracker

The request result tracker is a key concept that makes asynchronous operations manageable:

* It’s a simple integer variable (`Modbus::Client::Result` enum) you provide to the client
* It is automatically set to `NODATA` after the request is accepted
* The client updates its value automatically as the request progresses through its lifecycle
* You can check it at any time to determine the current state without callbacks
* The variable is updated from an internal task, so it’s always current

In asynchronous mode, when `sendRequest()` returns `SUCCESS`, it just indicates it was accepted & queued for transmission. The actual outcome of the operation is notified via the callback mechanism after the response is received (or in case of timeout or other transmission errors) and will update the tracker accordingly.

## Asynchronous mode with response callback (non-blocking):

* Returns immediately after sending the request
* Allows your application to continue other tasks while waiting
* Calls your callback function when an outcome (failure or success) happens

### Usage

This alternative path allows you to handle the response in a dedicated callback instead of waiting for the tracker to be updated. It offers true event-driven programming without the need for polling or periodic status checks, in a "fire & forget" fashion, mostly suited if you are looking for pure responsiveness & performance.

```cpp
Modbus::Frame request = { /* request details */ };
Modbus::Client::ResponseCallback cb = [](Modbus::Client::Result res, const Modbus::Frame* resp, void* ctx) {
    // Handle response here...
};

client.sendRequest(request, cb); // request + cb = async with callback (you can also pass a custom context)
// sendRequest returns immediately with `Modbus::Client::SUCCESS` if the request 
// is valid & queued for TX 
```

Basically, there is nothing to do after calling `sendRequest()` : the handler will take care of everything when the response arrives or if an error occurs! This methods allows you to totally decouple the request from the response handling, and even transmit a "transmission ID" (through `userCtx`) so that the thread handling the response can identify it.

!!! note "Note on memory usage"
    Callbacks are the most memory-efficient method because you don't need to allocate room for the response `Frame`. Inside the callback, you are directly reading into the server's internal response buffer, so a local copy isn't necessary.

### Understanding the callback system

As stated before, the callback allows you to define a custom function that will be called when the Modbus transaction succeeds (success, timeout or failure). In order to limit as much as possible the overhead and avoid dynamic allocation two design decisions were made for the Client application class:

* A callback isn't an `std::function` but a traditional function pointer (N.B.: in C++ a non-capturing lambda decays to a function pointer, so you can still use the lambda syntax when declaring your callback).
* The response is accessed through a pointer ; when no response is expected (transmission failure or broadcast request), EZModbus doesn't return an empty `Modbus::Frame` but a `nullptr` (saving \~260B of RAM and a few CPU clocks).

#### Callback function signature

The callback function follows a specific signature defined by the `ResponseCallback` type:

```cpp
using ResponseCallback = void (*)(Result result,
                                 const Modbus::Frame* response, // nullptr if no response
                                 void* userCtx);
```

**Parameters explained:**

* `Result result`: The outcome of the Modbus transaction (similar to Result Tracker)
* `const Modbus::Frame* response`: Pointer to the response frame
    * **Valid pointer**: When `result == SUCCESS`, contains the actual response data or exception code
    * **`nullptr`**: When the request failed (timeout, TX error) or for broadcast requests
* `void* userCtx`: User-defined context pointer passed-through unchanged
    * Allows sharing callbacks between multiple requests while maintaining context
    * Can be `nullptr` if no context is needed

The response pointer will be `nullptr` in three specific scenarios:

* Broadcast Requests: Modbus broadcast messages (slaveId = 0) don't expect responses
* Transmission Failures: When the request couldn't be sent over the physical interface
* Timeout Scenarios: When no response is received within the configured timeout

!!! warning
    Always check the validity of `response` before trying to dereference it!

#### Using user context

Since we cannot use a capturing lambda, the `userCtx` parameter allows you (but is not mandatory) to pass a custom context when calling `sendRequest`, that will be handed back to you when the callback is called:

```cpp
// Context definition
struct RequestContext {
    int requestId;
    uint32_t timestamp;
    const char* description;
};

// Callback definition
void myCallback(Result result, const Frame* response, void* userCtx) {
    // Cast back to your specific context type
    RequestContext* ctx = static_cast<RequestContext*>(userCtx);
    
    // Define the processing logic
    // 1. Error path
    if (result != SUCCESS) {
        logModbusResult("Request %d failed: %s\n", 
                 ctx->requestId, 
                 Modbus::Client::toString(result));
        return;
    }
    // 2. Success path without response (e.g. Broadcast)
    if (!response) {
        logModbusResult("Request %d (%s): OK with no response (RTT: %d ms)\n", 
                 ctx->requestId, 
                 ctx->description,
                 TIME_MS() - ctx->timestamp);
        return;
    }
    // 3. Success path with response
    Modbus::ExceptionCode ec = response.exceptionCode;
     // Exception response
    if (ec != Modbus::NULL_EXCEPTION) {
        logModbusResult("Request %d (%s): Exception = %s (RTT: %d ms)\n", 
                 ctx->requestId, 
                 ctx->description,
                 Modbus::ExceptionCode::toString(ec), 
                 TIME_MS() - ctx->timestamp);
        return;
      // Response with data
    } else { 
        uint16_t value = response->getRegister(0);
        logModbusResult("Request %d (%s): Value = %d (RTT: %d ms)\n", 
                 ctx->requestId, 
                 ctx->description,
                 value, 
                 TIME_MS() - ctx->timestamp);
        return;
    } 
}

// ...usage in code

Modbus::Frame request = { /* request details */ };

RequestContext ctx = {
    .requestId = 1001,
    .timestamp = TIME_MS(),
    .description = "Temperature sensor read"
};

client.sendRequest(request, myCallback, &ctx);
```

Here, we showcased an example of a simple context with a "catch-all" callback that just displays information about the original request & possible response, but it could be used to access any variable you want, or even a class instance if you need to update values or trigger actions on another component of your application.

#### Lambda functions vs function pointers

✅ Supported - Function pointers:

```cpp
// Global/static function
void myResponseHandler(Result result, const Frame* response, void* ctx) {
    // Handle response
}
client.sendRequest(request, myResponseHandler);

// Non-capturing lambda (decays to function pointer)
client.sendRequest(request, [](Result result, const Frame* response, void* ctx) {
    // Handle response
});
```

❌ Not supported - Capturing lambdas:

```cpp
int localVar = 42;
// This won't compile - capturing lambdas cannot convert to function pointers
client.sendRequest(request, [localVar](Result result, const Frame* response, void* ctx) {
    // Cannot capture localVar
});
```

✅ Alternative - Use context instead:

```cpp
int localVar = 42;
// Pass local data through context when calling sendRequest
client.sendRequest(request, 
                  [](Result result, const Frame* response, void* ctx) {
                      int* value = static_cast<int*>(ctx);
                      // Use *value instead of captured variable
                  }, 
                  &localVar);
```

#### Callback execution context & considerations

* Callbacks are executed in the context of the Modbus interface's internal task, not your main application thread, so they should be kept fast and non-blocking to avoid delaying other Modbus operations. If your callback accesses shared data, ensure proper synchronization between threads. Additionally, avoid calling blocking FreeRTOS functions within callbacks as this could interfere with the lib's internal operations.
* Non-`static` variables declared in your callbacks (even if the callbacks are `static` themselves) rely on the Modbus interface's internal task stack. Be careful with heavy internal variables & complex call paths inside the handlers, or resize the stack size in `ModbusRTU.h` / `ModbusTCP.h` (see [Settings](../40-additional-resources/401-settings-compile-flags.md#modbusrtuh) page)

**Good callback practices:**

```cpp
void quickCallback(Result result, const Frame* response, void* ctx) {
    if (result == SUCCESS && response) {
        // ✅ Fast operations: variable updates, simple calculations
        volatile uint32_t* targetVar = static_cast<volatile uint32_t*>(ctx);
        *targetVar = response->getRegister(0);
        
        // ✅ Non-blocking notifications
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(processingTask, 1, eSetBits, &higherPriorityTaskWoken);
    }
}
```

**Operations to avoid in callbacks:**

```cpp
void problematicCallback(Result result, const Frame* response, void* ctx) {
    // ❌ Avoid blocking operations
    vTaskDelay(100);  // Don't block the Modbus task
    
    // ❌ Avoid heavy processing
    performComplexCalculation();  // Offload to another task
    
    // ❌ Avoid synchronous I/O
    Serial.println("Response received");  // May block if buffer full
}
```

## Important note about variable lifetime

!!! warning
    * In asynchronous mode using a result tracker, it is the user’s responsibility to **ensure both the response placeholder & the tracker are valid throughout the whole request lifecycle**! Otherwise, a crash may occur as the library could try to access dangling memory locations. Once the tracker has updated to any other value than `NODATA`, it is safe to release the objects.
    * In asynchronous mode using a callback, it is the user's responsibility to **ensure the callback & the context (if used) remain valid throughout the whole request lifecycle**! Otherwise, a crash may occur as the library could try to access dangling memory locations. You are free to release the objects after the callback returns.

## Managing multiple devices

The client is designed to work with multiple slave devices on the same bus:

* Simply change the `slaveId` field in your request frame for each device
* Wait for each request to complete before sending the next one (Modbus is sequential) for example by testing the `Client` status with the `isReady()` method (or retrying if `sendRequest()` returns `ERR_BUSY`)
* For broadcast messages (slaveId 0), no response is expected and the status proceeds directly to `SUCCESS` after the TX fully completes.

## Error handling and diagnostics

The client provides several tools for effective error handling:

1. **Result Enum** - From synchronous calls, stored in the result tracker or returned by the callback:

   * `SUCCESS` - Operation completed successfully
   * `ERR_INVALID_FRAME` - Request was malformed or invalid
   * `ERR_BUSY` - Another transaction is in progress
   * `ERR_TX_FAILED` - Failed to transmit request
   * `ERR_TIMEOUT` - No response received within timeout
   * `ERR_INVALID_RESPONSE` - Received response was invalid

2.  **Exception Detection** - Modbus protocol exceptions from the slave can be easily decoded with the `toString` method:

    ```cpp
    if (response.exceptionCode != Modbus::NULL_EXCEPTION) {  
    	Serial.printf("Slave reported exception: %s\n",
                     Modbus::toString(response.exceptionCode));
    }
    ```

3. **Debug Mode** - When `EZMODBUS_DEBUG` is defined, detailed logs show frame contents and round-trip timing.

## Handling broadcast requests

Modbus allows broadcast messages that are received by all slaves but not acknowledged:

* Set `slaveId` to 0
* Only write operations are valid for broadcast (per Modbus specification)
* The client will not wait for a response and will complete immediately
* Use for operations like resetting multiple devices or synchronizing actions

Broadcast operations can significantly reduce bus traffic when updating multiple devices, but come with no guarantee that all devices received the message correctly.

## Setting custom request timeout

The client enforces a timeout: if a response isn’t received within this timeframe, the request will be closed (marked as `ERR_TIMEOUT`), and the client layer will be ready to accept a new request. The default round-trip timeout is 1 second (`DEFAULT_REQUEST_TIMEOUT_MS` constant). For convenience, you can select another timeout when instanciating the client :

```cpp
Modbus::Client client(rtu, 5000); // Use a 5 second timeout
```
