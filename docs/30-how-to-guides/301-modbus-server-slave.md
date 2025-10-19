# Modbus Server (Slave)

The Modbus Server implements the **Slave** role, responding to client requests by maintaining an internal list of `Word` blocks. It handles request validation, concurrency, and error reporting automatically.

Read the [Core concepts > Word](../20-core-concepts/203-modbus-word.md) section of this guide before what follows.

## Basic workflow

The workflow of the server is simple:

* You define your data model by adding a set of Words to the Server's store
* When the server receives a request, it will look for registered Words that match the target registers contained in the request, and if the request is valid, individually read/write each Word through the pointer or callbacks (handlers) defined in the Words metadata
* It sends back to the client a response frame, containing either result data or a Modbus exception if any operation failed (invalid Words in request, partial Word read/write, exception returned in handlers...).

### Single-interface setup

For basic single-interface setups (RTU or TCP), the workflow is straightforward:

```cpp
// 1. Create Word store
// - On the stack (hides an std::array)
Modbus::StaticWordStore<200> store;
// - Or on the heap (hides an std::vector)
Modbus::DynamicWordStore store(10000);

// 2. Create Server instance
Modbus::Server server(rtu,              // ModbusInterface instance (RTU/TCP)
                      store,            // WordStore reference
                      SERVER_SLAVEID);  // Server Slave ID (optional for TCP)

// 3. Add words (see below sections) - simplified here: no return value check
server.addWord({Modbus::COIL, 10, 1, &coilPtr1});
server.addWord({Modbus::COIL, 20, 1, &coilPtr2});
server.addWord({Modbus::INPUT_REGISTER, 100, 1, &regPtr1});
server.addWord({Modbus::INPUT_REGISTER, 110, 1, &regPtr2});
// ...

// 4. Finally, call begin()
server.begin(); // -> Returns ERR_WORD_xxx upon failure

// The server is now ready to operate
```

### Multi-interface setup

Since `v1.1.6` servers can accept multiple interfaces simultaneously, allowing a single server instance to handle requests coming from several Modbus interfaces of any type (TCP/RTU).

This approach is particularly useful for applications that need both low-level RS485/RTU access by legacy equipment and high-level TCP connectivity for remote monitoring or configuration tools. All interfaces share the same Word store (register map), ensuring data consistency regardless of the access method.

```cpp
// 1. Create interfaces
ModbusInterface::TCP tcpInterface(tcpHal, Modbus::SERVER);
ModbusInterface::RTU rtuInterface(rtuHal, Modbus::SERVER);

// 2. Create Word store
Modbus::StaticWordStore<200> store;

// 3. Create Server with multiple interfaces
Modbus::Server server({&tcpInterface, &rtuInterface},  // Initializer list of interface pointers
                      store,                           // WordStore reference
                      SERVER_SLAVEID,                  // Server Slave ID
                      true,                            // Reject undefined registers - optional
                      UINT32_MAX);                     // Request mutex timeout (ms) - optional

// 4. Add words and initialize
server.addWord({Modbus::HOLDING_REGISTER, 100, 1, &tempValue});
server.begin();

// Server now responds to both TCP and RTU requests
```

**Thread safety**: Multi-interface servers use an internal mutex to ensure thread-safe access to the Word store: register values and read/write handlers are protected from concurrent access. The "Request Mutex timeout" controls behavior:

- `UINT32_MAX` (default): Blocking wait until mutex is available - requests for all interfaces will be serialized (mapped to `portMAX_DELAY`)
- `0`: Try-lock mode - returns `SLAVE_DEVICE_BUSY` exception if mutex is already held
- In-between: waits for a max amount of time and returns if cannot lock the mutex

This parameter can be set in 2 different ways:

- At compile time, by setting the build flag `EZMODBUS_SERVER_REQ_MUTEX_TIMEOUT_MS`
- At runtime, by providing the `Server` ctor's last argument (`reqMutexTimeoutMs`)

The default behaviour ensures 100% request hit rate but may be prone to deadlocks, as a bug in user-defined handlers might cause a stuck interface to block other threads. A safer compromise may be to use a fixed timeout of 50~100 ms, long enough to guarantee success while allowing to detect logic bugs in handlers in case of repeated timeouts.

**Interface limits**: The server enforces a maximum number of interfaces defined at compile time (2 by default). This limit can be adjusted using the `EZMODBUS_SERVER_MAX_INTERFACES` build flag if more interfaces are needed.

### Words storage & memory management

Words are stored on the server inside a `Modbus::WordStore` container which can store an arbitrary number of `Word` objects regardless of their underlying `RegisterType`. This container is created in user code space (to keep in control of memory allocation) but directly managed by the Server.

The optimal process is the following :

1. Create the `WordStore` object on the stack or on the heap (see above example)
2. Pass the `WordStore` to the `Server` constructor
3. Register your Words on the server with the `Server::addWord()` method
4. Finally, call `Server::begin()` after adding all your Words

To guarantee performance with a low RAM footprint & simple API, the Server's `WordStore` must stay sorted at all times (enables binary search), as it relies on a raw buffer/`std::vector` instead of more engineered STL containers such as `std::map`. Adding a Word to the store also implies collision checks to detect a conflict (address overlap with an existing Word) which are highly time-consuming if not done on contiguous Words.

The following process is observed to optimize computation overhead:

* `addWord()` always does basic unit checks at each addition: validity of function code, register count, address range & pointer/handler definition.
* The user can add Words to the register before calling `Server::begin()` , which on the 1st time will sort the whole store & check for conflicted Words (address overlap). `begin()` returns an `ERR_WORD_OVERLAP` error upon the first overlap detected. All of this is efficient since it's done in a single operation.
* Further Word additions after `Server::begin()` **individually** check for overlap & do a sorted insertion to ensure the Word store stays properly stored, which could require a significant amount of time for instance if you're adding 1000s of Words back-to-back.

!!! note
    TLDR:

    * Ideally do all `Word` additions to the store **before calling** `Server::begin()` (especially on servers exposing a large number of Words).
    * **Always monitor** the return value of `Server::addWord` especially if you add Words while the `Server` is running to be sure to catch any detect conflicts.

### Capacity limit

The capacity defined in `WordStore` instanciation will be enforced by the server: you won't be able to add more Words after the capacity is reached, so size it carefully.

### Notes on `DynamicWordStore`

Each `Word` has an ~18 bytes RAM footprint, which becomes significant for thousands of Words registered on a server: using dynamic allocation may become necessary if the Word store grows too much and the stack cannot handle it.

The `DynamicWordStore` is quite efficient because it allocates a fixed chunk of memory at the start of the program, so it won't be re-allocated at runtime, avoiding fragmentation & allocation errors.

On ESP32, the `DynamicWordStore` will be automatically added to PSRAM (if enabled) if its size exceeds the free memory on the initial 320 kB DRAM, which is quite interesting for large servers with > 10k registers.

### Adding multiple Words at once

To ease the initialization process, the `addWords(Word* words, size_t count)` method will add several words from a buffer to the server.

!!! note
    This method is atomic, which means all Words will be added or none. If there's a validation issue with **any** of the Words you submit, it will return an error (`ERR_WORD_...`) and no Word will be stored consequently. The log trace will display which invalid Word returned the error first:

    ```
    [ModbusServer.cpp::addWords:110] Error: Malformed handlers (holding register 23-24)
    ```

Out of convenience, the method also has an overload to take an `std::vector<Word>` as argument.

## Word value access methods

The server supports two complementary approaches for making `Word` value accessible from clients, designed for different use cases and performance requirements.

### Direct Value Pointers

* Simply provide a pointer to an existing variable (`volatile uint16_t`) in your code
* The server will read from and write to this variable automatically
* Best for straightforward data with maximum performance
* Very efficient with minimal overhead
* **Only valid for single-register Words (**`nbRegs = 1`**)**

#### Word declaration example using a direct value pointer

```cpp
volatile uint16_t temperature = 0; // The variable exposed on the Word

Modbus::Word tempWord = {
  .type = Modbus::INPUT_REGISTER,
  .startAddr = 100,
  .nbRegs = 1,
  .value = &temperature
};
server.addWord(tempWord);
 
// Later in your code:
temperature = getSensorReading() * 100;  // Update the value any time
```

Instead of the classical "struct w/ designated initializers" notation, you can also use a more straightforward syntax using a simple initializer list :

```cpp
Modbus::Word tempWord = {Modbus::INPUT_REGISTER, 100, 1, &temperature};
```

!!! note "Thread safety note"
    Using `volatile uint16_t` ensures atomic access for single registers & prevents concurrent access on most 32-bit architectures. However, if you're updating related Words or need strict timing guarantees, consider using handlers with proper synchronization instead.

### Handler functions

* Provide custom read and/or write functions for the entire Word
* Automatically invoked by the server after checking request validity
* Allows for dynamic values, validation, transformations, or side effects
* Required for multi-register Words (`nbRegs > 1`)
* Handlers can return any Modbus exception code depending on request outcome

#### Handlers signature

The handlers are stored as static function pointers that must follow a precise signature. We will describe each of the parameters below.

```cpp
// Read handler signature
Modbus::ExceptionCode (*)(const Word& reqWord, // Called Word context
                          uint16_t* outVals,   // Output values array
                          void* userCtx);      // Optional user context

// Write handler signature
Modbus::ExceptionCode (*)(const uint16_t* writeVals, // Input values array
                          const Word& reqWord,       // Called Word context
                          void* userCtx);            // Optional user context

```

**Return type**

* Your read handlers must fill the output array `outVals` and return a `Modbus::ExceptionCode` that will be sent back to the client in case of read failure (`NULL_EXCEPTION` to signal success & send back response data to the client).
* Your write handlers read the requested write value array `writeVals`, process data & must also return an exception code that indicates whether the write operation  succeeded.

It is then possible to manage validation of client input data by returning a non-null Modbus exception in the handler (on the contrary, the direct pointer won't allow any validation step as it accesses the raw variable).

**Requested word** `reqWord`

Handlers provide the full requested Word context as a parameter. It is passed by the server when a valid frame is received & processed.  The `Word&` parameter contains all metadata about the Word being accessed. This includes its `type`, `startAddr` & `nbRegs` .

You can use this context for:

* **Determining how many raw registers should be read/written in the input/output values array**
* Sharing handlers between multiple Words
* Logging which Word triggered the handler
* Dynamic behavior based on Word properties
* Mapping between Word addresses and application logic

!!! note
    For Words that contain an unique register (`nbRegs == 1`) or Word registered with their own handler you don't need to use the context since you already know how many registers should be read or written. Still it's good practice to always check `reqWord.nbRegs` to avoid any buffer overflow, and not rely on the hard value defined when declaring your Word.

**Input/output values**

* For a read request, you should write the result to the `outVals` buffer passed as argument when the handler is called. The number of `uint16_t` register values to write in the buffer can be determined by accessing `reqWord.nbRegs`.
* For a write request, you should read the requested value in the `writeVals` buffer passed as argument. The number of `uint16_t` requested register values to read from the buffer can be determined by accessing `reqWord.nbRegs`.

**User context**

Handlers are static function pointers that cannot capture any external variable like a lambda would allow. The user context allows you to attach to a `Word`, a pointer that will be passed to the handler as argument when it's called by the Server. It is very useful if you need to access class instances or non-global/non-static variables inside the handlers.

The use of the user context and the implications in terms of syntax is very similar to what is done in the Client for asynchronous requests using callbacks. Refer to the [How-To Guides > Modbus Client (Master)](300-modbus-client-master.md)  section for more information.

!!! note
    Since `userCtx` is part of the handlers' signature, you must use it when declaring a handler even if you are not using it at all inside your callback.

#### Word declaration examples with handlers

To keep the exemples clear, we assume that :

* The handler definition is done globally (outside of your main program scope a.k.a. `main()`, `app_main()` , `setup()`/`loop()` or any FreeRTOS task).
* The `addWord()` method is called in the main program scope

You will need to adapt those examples if you wish to do differently:

* Handler declaration after the entry point: use stateless lambda syntax
* Hander declaration in a class (see last example): use the `static` qualifier

**Simple handlers example**

```cpp
// Read handler function
Modbus::ExceptionCode spReadCb(const Modbus::Word& reqWord,
                              uint16_t* outVals,
                              void* userCtx) {
   outVals[0] = getSetpoint(); // Only one register here
   return Modbus::NULL_EXCEPTION; // OK
}

// Write handler function
Modbus::ExceptionCode spWriteCb(const uint16_t* writeVals,
                               const Modbus::Word& reqWord,
                               void* userCtx) {
   uint16_t newSetpoint = writeVals[0]; // Only one register here
   
   // Validation: return an exception if out of bounds
   if (newSetpoint < MIN_SETPOINT || newSetpoint > MAX_SETPOINT) {
       return Modbus::ILLEGAL_DATA_VALUE;
   }
   
   // Apply the change
   bool success = setSetpoint(newSetpoint);
   return success ? Modbus::NULL_EXCEPTION    // OK
                  : Modbus::SLAVE_DEVICE_FAILURE;
}

// Add Word in main loop scope
server.addWord({
   .type = Modbus::HOLDING_REGISTER,
   .startAddr = 200,
   .nbRegs = 1,
   .readHandler = spReadCb,
   .writeHandler = spWriteCb
});
```

**Shared handler example (using Word context)**

In this example, we use a unique handler shared between all Words/Modbus registers mapped to the 512 DMX channels (1st DMX channel mapped to register 0, and so on).

The read & update value behaviour are the same across all DMX channels, we just need to figure out which DMX channel was requested by reading the startAddr value of the requested Word (i.e. address of the register requested by the client)

```cpp
// Single read handler for multiple DMX channels
Modbus::ExceptionCode dmxReadCb(const Modbus::Word& reqWord,
                               uint16_t* outVals,
                               void* userCtx) {
   // Check if requested DMX channel is valid
   if (rcvWord.startAddr >= DMX_CHANNELS) return Modbus::ILLEGAL_DATA_ADDRESS;
   
   // Read DMX output value for this channel
   outVals[0] = (uint16_t)dmx.get(rcvWord.startAddr);
   
   return Modbus::NULL_EXCEPTION; // OK
}

// Single write handler for multiple DMX channels
Modbus::ExceptionCode dmxWriteCb(const uint16_t* writeVals,
                                const Modbus::Word& reqWord,
                                void* userCtx) {
   // Check if requested DMX channel is valid
   if (rcvWord.startAddr >= DMX_CHANNELS) return Modbus::ILLEGAL_DATA_ADDRESS;
   
   // Check if requested output value is valid
   if (writeVals[0] > 255) return Modbus::ILLEGAL_DATA_VALUE;
   
   // Write DMX output value for this channel (exception returned upon failure)
   if (!dmx.set(rcvWord.startAddr, (uint8_t)writeVals[0])) return Modbus::SLAVE_DEVICE_FAILURE;
   
   return Modbus::NULL_EXCEPTION; // OK
}

// Add Words in main loop scope:
// Register all DMX channels with the same handlers
for (int channel = 0; channel < DMX_CHANNELS; channel++) {
   server.addWord({
       .type = Modbus::HOLDING_REGISTER,
       .startAddr = channel,
       .nbRegs = 1,
       .readHandler = dmxReadCb,
       .writeHandler = dmxWriteCb
   });
}
```

**Multi-register handler example (32-bit float)**

The `ModbusCodec` namespace provides helper functions to encode a `float` value to IEEE 754 format (two 16-bit raw Modbus registers) and vice-versa.

```cpp
// Float read handler
Modbus::ExceptionCode floatReadCb(const Modbus::Word& reqWord,
                                  uint16_t* outVals,
                                  void* userCtx) {
   // Get value from user application
   float currentValue = app.getFloatValue();
   
   // Convert to float & write to output array
   ModbusCodec::floatToRegisters(currentValue, outVals);
   
   return Modbus::NULL_EXCEPTION; // OK
}

// Float write handler
Modbus::ExceptionCode floatWriteCb(const uint16_t* writeVals,
                                   const Modbus::Word& reqWord,
                                   void* userCtx) {
   // Convert requested write value to float
   float newValue = ModbusCodec::registersToFloat(writeVals);
   
   // Validate requested write value
   if (newValue < 0.0f || newValue > 100.0f) {
       return Modbus::ILLEGAL_DATA_VALUE;
   }
   
   // Apply value to user application
   app.setFloatValue(newValue);
   return Modbus::NULL_EXCEPTION; // OK
}

// Add Word in main loop scope: unique Word
// with a span of 2 Modbus registers
server.addWord({
   .type = Modbus::HOLDING_REGISTER,
   .startAddr = 300,
   .nbRegs = 2,
   .readHandler = floatReadCb,
   .writeHandler = floatWriteCb
});
```

**Handler using** `userCtx` **example**

Let's say you have a `TempController` class, and you want to expose a method that will register a handler used to fetch the `temperature` value. Using `userCtx` is necessary in this case if you want to make the handler generic.

```cpp
class TempController;
TempController myTempCtrl();

// Generic handler for all TempController instances
Modbus::ExceptionCode readTemperature(const Modbus::Word& reqWord, 
                                      uint16_t* outVals, 
                                      void* userCtx) {
    // Cast userCtx to the class instance
    TempController* self = static_cast<TempController*>(userCtx);
    
    // Check controller status (class API)
    if (!self->isHealthy()) return Modbus::SLAVE_DEVICE_FAILURE;
    
    // Fetch & return temperature value
    outVals[0] = static_cast<uint16_t>(self->getTemperature() * 10);
    return Modbus::NULL_EXCEPTION; // OK
}

// Add Word in main loop scope
server.addWord({
    .type = Modbus::INPUT_REGISTER,
    .startAddr = 100,
    .nbRegs = 1,
    .readHandler = readTemperature,
    .userCtx = &myTempCtrl // Pointer to your TempController instance
});
```

* If you want to put handler definition inside the class, use the `static` qualifier.
* If you want to put Word declaration inside the class, use `this` instead of `&myTempCtrl`

**Important notes on handlers usage**

!!! note
    * The handlers are called in the context of the Modbus interface task, so they must be fast and not block. Design them like you would do for an ISR; if there's intensive processing to do, consider offloading it to a background task.
    * Non-`static` variables declared in your callbacks (even if the callbacks are `static` themselves) rely on the Modbus interface's internal task stack. Be careful with heavy internal variables & complex call paths inside the handlers, or resize the stack size (see [Settings](../40-additional-resources/401-settings-compile-flags.md) page).

    See notes related to Callback usage in [How-To Guides > Modbus Client (Master)](300-modbus-client-master.md), they apply to Modbus Server handlers as well.

    * For read-only register types, the `writeHandler` **must not** be provided, otherwise this will trigger an error when calling `addWord()`.
    * If you specify both a value pointer AND handlers, the handlers take priority, and the value pointer will be ignored.
    * If you try to add a Word that overlaps with existing Words, it will be rejected with an `ERR_WORD_OVERLAP` error
    * If used, the object designated by `userCtx` must be accessible during the whole Server lifetime! Otherwise, incoming requests could trigger a crash as the Server would access a dangling memory location

## Error codes

```cpp
enum Result {
    // Success
    SUCCESS,                        // Success
    // addWord errors
    ERR_WORD_BUSY,                  // Busy word store
    ERR_WORD_OVERFLOW,              // Stored too many words
    ERR_WORD_INVALID,               // Invalid word reg type
    ERR_WORD_DIRECT_PTR,            // Forbidden direct pointer
    ERR_WORD_HANDLER,               // Malformed handlers
    ERR_WORD_OVERLAP,               // Word overlapping an existing one
    // Request processing errors
    ERR_RCV_UNKNOWN_WORD,           // Word not found
    ERR_RCV_BUSY,                   // Incoming request: busy
    ERR_RCV_INVALID_TYPE,           // Received invalid request type
    ERR_RCV_WRONG_SLAVE_ID,         // Wrong slave ID in rcv'd frame
    ERR_RCV_ILLEGAL_FUNCTION,       // Illegal function in rcv'd frame
    ERR_RCV_ILLEGAL_DATA_ADDRESS,   // Illegal data address in rcv'd frame
    ERR_RCV_ILLEGAL_DATA_VALUE,     // Illegal data value in rcv'd frame
    ERR_RCV_SLAVE_DEVICE_FAILURE,   // Slave device failure on rcv'd frame
    ERR_RSP_TX_FAILED,              // Transmit response failed
    // Misc errors
    ERR_NOT_INITIALIZED,            // Server not initialized
    ERR_INIT_FAILED                 // Init failed
};
```

## Rejecting undefined Words

In the Modbus specification, access to undefined registers should result in an `ILLEGAL_DATA_ADDRESS` exception. However, if you have holes in your register table and both devices clearly know the correct set of registers to use, it might be more efficient to just ignore those requests (i.e. return `0` values) and proceed with the rest of the transaction, so that the client can use a single multiple-register-read/write request instead of many single-register-read/write requests.

By default, the EZModbus Server _does reject_ calls to undefined registers with an exception, but you can disable this behavior by instantiating the server object with an additional `rejectUndefined` argument (which is set to `true` by default):

```cpp
Modbus::Server server(iface, store, slaveId, false); // false = no exception on undefined registers
```

A simple example to illustrate this: let's say you have Words registered covering the following Input Registers :

```
[100, 102] : Temperature value
[110, 112] : Humidity value
[120, 121, 122, 123] : IP address
```

If the client tries to send a read request for the whole range `[100...123]`  to fetch the 3 values at once, the server will:

* With `rejectUndefined == true` (default): return a Modbus exception to the client
* With `rejectUndefined == false` (forced): return values for registers defined above, and fill the gaps with `0`'s

This work even if you have leading or trailing undefined registers, and does not override the atomicity checks for Words registers: **if the requested range contains Words defined on several registers, the client still MUST read or write all of them**, or the request will fail, regardless if `rejectUndefined` is `true` or `false`!

## Exception handling

#### Single read/write

For single read & write requests, an exception will be returned to the client/master :

* If the requested word is not registered,
* Or in case of a partial read of a multi-register word (before even calling the handler),
* Or if the read/write handler itself returns a non-null `Modbus::ExceptionCode`

#### Multiple read/write

The way for the Server to handle Modbus exceptions for read/write handlers depends on the case:

* For both read & write requests, a first pass is done checking the existence of words and the validity of requested ranges : if any of the requested words is not registered, OR in case of any partial read of a multi-register word, an exception is immediately returned to the client/master without even processing the handler.
* After checking word validity:
    * For multiple read requests, any failing read handler execution (returning a non-null `Modbus::ExceptionCode`) will trigger the exception being returned to the client
    * For multiple write requests, the Modbus specification requires **atomic operations**. In this case, **all handlers will be called one after the other**, and if any (or several) of those handlers fail with a non-null `Modbus::ExceptionCode`, the client/master will get no value but the first triggered exception. It is the responsability of the client/master to read back registers, if required, to check which of the writes were successful or not.

## Unit ID for Modbus TCP

Modbus TCP devices are normally not addressed using the traditional `slaveId` field, but by their port and IP address.\
A `Unit ID` field exists in the Modbus TCP protocol, and is used to relay Modbus requests to other RTU devices that use the `slaveId` field (reserved to Modbus Gateway applications such as EZModbus's `Bridge` component).

To make it work correctly, TCP manages the `slaveId` field as follows:

* The TCP codec does not check the `slaveId` field in either direction, so it can be set to any value (even up to 255 i.e. above the Modbus RTU limit of 247)
* When used with a TCP interface, the server will accept any Unit ID value (with RTU, it will discard requests that do not match its own `slaveId` except for broadcast requests)
* Broadcast requests (`slaveId` 0) are still handled correctly (response is dropped, request is completed right after handling the frame to the application layer)

In short, you don't have to worry about the `slaveId` field when starting a Modbus TCP server:

* The server will accept any Unit ID in received requests, and will echo it back in the response
* The client will accept to send requests to hosts with any Unit ID value even above the Modbus RTU limit of 247

When connecting a TCP interface to an EZModbus Bridge, the Unit ID received will become the `slaveId` on the RTU side, and vice-versa.
