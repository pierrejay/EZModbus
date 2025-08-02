# Error handling

EZModbus provides comprehensive error handling through a structured `Result` enumeration system. Each component in the library has its own dedicated `Result` type, serving as a clear indicator of operation status. Errors are not only detected but also clearly communicated throughout the entire stack (most public methods in EZModbus use this return type).

## **Consistent and clear error reporting**

The `Result` enumeration follows a consistent pattern across all components, see this example with `Modbus::Client::Result`:

```cpp
enum Result {
    SUCCESS,              // Success is always 0
    NODATA,               // Only for Client (request in progress)
    ERR_INVALID_FRAME,    // Error codes...
    ERR_BUSY,
    ERR_TX_FAILED,
    ERR_TIMEOUT,
    ERR_INIT_FAILED,
    ...
};
```

Every component includes a `toString()` method that converts these numeric codes into human-readable messages:

```cpp
static constexpr const char* toString(const Result result) {
    switch (result) {
        case SUCCESS: return "success";
        case NODATA: return "no data (ongoing transaction)";
        case ERR_INVALID_FRAME: return "invalid frame";
        case ERR_BUSY: return "busy";
        case ERR_TX_FAILED: return "tx failed";
        case ERR_TIMEOUT: return "timeout";
        case ERR_INIT_FAILED: return "init failed";
        ...
        default: return "unknown error";
    }
}
```

This approach eliminates ambiguity in error reporting and simplifies debugging by providing immediate clarity on what went wrong.

## **Error propagation through layers**

EZModbus propagates errors through its layered architecture. When a low-level error occurs (such as in the transport layer), it doesn't get lost or obscured - it propagates up through the stack until it reaches your application code.

For example, if a CRC check fails in the RTU interface, this error is passed to the client, which then returns it to your code. This clear propagation path means you always know exactly where and why an operation failed.

Error that are never catched (such as invalid request to the Modbus server which is never polled) also trigger a log trace. Indeed, EZModbus includes utility functions that enhance error reporting when debug mode is enabled (defined by the `EZMODBUS_DEBUG` flag). When debug is enabled, these helpers provide a lightweight "stack trace" that shows exactly where an error occurred. For instance, you might see log messages like:

```
[ModbusCodec.h::isValidFrame:203] Error: invalid function code
[ModbusClient.cpp::sendRequest:141] Error: invalid frame (invalid fields)
```

This tells you not only that the frame submitted is invalid, but exactly which function encountered it and why. This capability makes troubleshooting substantially easier. More detail about this in the `Logging/debugging` section below.

## **Simplified error checking**

The `Result` system is designed for straightforward error handling:

* All success codes are `0` (`SUCCESS`)
* For the Client, there's a special `NODATA` code (`1`) indicating an ongoing transaction (i.e. response data not yet received)
* All other codes represent errors

This simplifies conditional checks in your code:

```cpp
auto result = client.sendRequest(request, response);
if (result != Modbus::Client::SUCCESS) {
    // Handle error
    Serial.printf("Error: %s\n", Modbus::Client::toString(result));
    return;
}
// If you reach here, the call to sendRequest has been successful
```

## **Always check & handle return values!**

{% hint style="info" %}
It's **strongly recommended** to check the return value of every function call, including `begin()` initialization functions. Some errors only become apparent at runtime, and failing to check for these can lead to subtle bugs.

```cpp
// Error check & error message printing at initialization
auto clientInitRes = client.begin();
if (clientInitRes != Modbus::Client::SUCCESS) {
    Serial.printf("Failed to initialize Modbus client: %s\n", Modbus::Client::toString(clientInitRes));
    while (1) { delay(1000) }; // Halt
}
```

This approach significantly simplifies troubleshooting when issues arise without burdening the code with unnecessary complexity or overhead.
{% endhint %}
