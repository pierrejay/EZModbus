# Memory & system resources

## Memory management

### Zero dynamic allocation by default

EZModbus uses **zero dynamic allocation** in all its component by default. All FreeRTOS objects are created statically (using `xQueueCreateStatic`, `xTaskCreateStatic`, etc.) except for the UART event queue for ESP32 that is created by ESP-IDF (we only use the handle of this queue).

### Strategic memory management in Server

The `Server` component uses a transparent `WordStore` approach for memory management:

* **No hidden allocations**: The server never performs large memory allocations behind the scenes - you explicitly choose your storage strategy
* **Stack or heap flexibility**: Use `StaticWordStore<N>` for stack allocation or `DynamicWordStore` for heap allocation based on your needs
* **Transparent capacity**: Memory requirements are explicit at construction - no surprises about RAM usage during runtime
* **Controlled allocation**: WordStore capacity is user-defined and enforced. Large Word collections (1000s of registers) can be explicitly allocated on heap when needed, with zero reallocation guaranteed

This transparent approach ensures you maintain full control over memory allocation patterns, with no hidden costs or surprise allocations during server operation.

## FreeRTOS resources

### HAL wrappers

* The UART/RS485 HAL wrapper uses the native ESP UART implementation and does not rely on additional tasks.
* The TCP HAL wrapper spins up a task to handle sockets. Its stack size is set to 4096 bytes by default (sufficient for handling 4 sockets at a time i.e. the default max supported), with a priority of `tskIDLE_PRIORITY + 1` & no core affinity. Since the task uses a `select()` loop with a default timeout of 1 second, it spends most of its time doing nothing (idle), and only consumes CPU when awoken by the scheduler to process sockets as reactively as possible.

### Modbus interfaces

Both interfaces rely on a FreeRTOS task to manage communication:

* Modbus RTU interface: 2048 bytes stack size (or 4096 when logs enabled), `tskIDLE_PRIORITY + 1` priority & no core affinity
* Modbus TCP interface: 4096 bytes stack size (or 6144 when logs enabled), `tskIDLE_PRIORITY + 1` priority & no core affinity

Those tasks use an event-driven logic (fed by the driver & application layers event/data queues using `xQueueSet`) and do not consume useless CPU clocks when idle (e.g. continuous polling with `vTaskDelay(1)`).

### Application components

Application classes do not rely on additional tasks to reduce overhead as much as possible:

* The `Bridge` is just an overlay for the underlying transport layers, it only forwards messages between the two.
* The `Client` is purely asynchronous: it uses internal callbacks & event groups to handle outcome of TX requests asynchronously, without spinning up a task, and relies on a FreeRTOS timer to cleanup timeouted requests.
* The `Server` is also purely asynchronous, it is fed by the requests forwarded by the transport layer and does not use any FreeRTOS task internally either.

The main execution context that handles Modbus transactions is the task running in the transport layers.

## Flash & RAM footprint

Stats from IDF ELF size analysis, per full Modbus stack (Client/Server app + RTU/TCP interface + driver, all declared globally)

* ROM size (`.text` + `.rodata`):
    * ~10KB of Flash for both Modbus RTU & TCP
* RAM usage (`.bss`):
    * ~4KB for Modbus RTU
    * ~10KB for Modbus TCP

This includes all tasks' stacks as they are allocated statically, but does not include :

* IDF dependencies (ESP drivers, TCP/IP stack, Wi-Fi stack, etc.)
* `Modbus::Frame` buffers allocated in user code (268B RAM each)
* `Modbus::Word` registered on the server (18B RAM each)
* Code for user callbacks/handlers in `Client` & `Server` components
* The overhead of the logging system, when enabled with `EZMODBUS_DEBUG` flag
