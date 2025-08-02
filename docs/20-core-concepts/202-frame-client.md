# Modbus::Frame

At the heart of EZModbus is the `Modbus::Frame` structure - a representation of a Modbus message that serves as the common language between all components of the library. It is used internally by all the application components, but you will have to manipulate it directly to be able to send and receive Modbus messages using the `Client`.

```cpp
// The core data structure you'll work with
struct Frame {
    Modbus::MsgType type;           // REQUEST or RESPONSE
    Modbus::FunctionCode fc;        // Function code (READ_COILS, WRITE_REGISTER, etc.)
    uint8_t slaveId;                // Device ID (1-247, or 0 for broadcast)
    uint16_t regAddress;            // Starting register address
    uint16_t regCount;              // Number of registers/coils to read/write
    std::array<uint16_t, 125> data; // Register values or coil states (packed)
    Modbus::ExceptionCode exceptionCode; // Error code (if any) from the slave device
};
```

The memory footprint of a `Frame` is fixed: 268 bytes. It is the size required to store the data of the largest request payload (on 125 registers or 2000 coils).

## Definition

The `Frame` structure is an enhanced representation of a raw Modbus message. It contains all the information needed to:

1. **Create requests**: Fill the fields to specify what you want to read or write
2. **Send messages**: Pass to `Client::sendRequest()` to transmit over the network
3. **Process responses**: Examine the returned frame to extract values or check for errors
4. **Handle exceptions**: Check for Modbus protocol errors reported by slave devices

Think of a `Frame` as a bidirectional envelope for your Modbus communication. For requests, you populate it with your command details. For responses, the library fills it with the returned data, ready for your application to use.

Two aspects will make your life easier when manipulating `Frame` in Modbus client mode:

* The library automatically “enriches” the response frame so it keeps all information from the initial request. That means `regAddress`, `regCount` & `data` will match your original request even if the Modbus spec doesn’t state so (depending on the case, some of these fields aren’t part of the raw Modbus message)
* A response which is an exception will keep the original function code intact in the `fc` field. The `exceptionCode` field will hold the error code (illegal data address, slave device failure…). No need to flip bits to check if a response is an exception! Just check the value of the `exceptionCode` field: if it’s a regular response, it will be `NULL_EXCEPTION (= 0)` .

The `Frame` structure eliminates the need to understand low-level Modbus protocol details like PDUs, ADUs, coil bit packing or byte ordering - EZModbus handles all of that for you behind the scenes, and hands you a ready-to-use object filled with proper data once the response is received.

## Reading/writing register data in Frame

In order to optimize memory usage without dynamic allocation, the `data` field in `Modbus::Frame` stores registers & coils in a unique array: coils are packed to occupy as little space as possible.

Register data is thus typically not accessed directly by reading or writing the `data` array, but by using a simple API that will take care of storing & retrieving data in the correct format:

### Writing frame data

Those methods will set the `data` field of the `Frame` structure from a value or set of values :

#### During `Frame` initialization

* `packRegisters`: set frame data from a unique register value or list of registers values
    * From a vector: `packRegisters(std::vector<uint16_t> regs)`
    * From a buffer: `packRegisters(uint16_t* buf, size_t len)`
    * In place : `packRegisters(std::initializer_list<uint16_t>)`
* `packCoils`: set frame data from a unique coil state or list of coils states
    * From a vector: `packCoils(std::vector<bool> regs)`
    * From a buffer: `packCoils(bool* buf, size_t len)`
    * In place: `packCoils(std::initializer_list<bool>)`

Out of convenience, `packCoils` also include overloads for `uint16_t` in addition to `bool`, where any non-zero value will be considered as a `true` state.

#### After `Frame` initialization

* `Frame::setRegisters`: set frame data & register count from a unique or list of registers
* `Frame::setCoils`: set frame data & register count from a unique or list of coils

The types used for arguments are the same as the `packXXX()` methods.

#### Examples

```cpp
// From initializer list, in place
Modbus::Frame request = {
    .type = Modbus::REQUEST,
    .fc = Modbus::WRITE_REGISTER,
    .slaveId = 1,
    .regAddress = 100,
    .regCount = 4,
    .data = Modbus::packRegisters({100, 83, 94, 211})
};

// From a vector, in place
std::vector<bool> coils = {true, false, true, true, false};
Modbus::Frame request = {
    .type = Modbus::REQUEST,
    .fc = Modbus::WRITE_COILS,
    .slaveId = 1,
    .regAddress = 20,
    .regCount = coils.size(),
    .data = Modbus::packCoils(coils)
};

// From a buffer, after init
uint16_t values[10] = {100, 101, 102, 103, 104, 105, 106, 107, 108, 109};
Modbus::Frame request;
request.setRegisters(values, (sizeof(values) / sizeof(values[0])));
...
```

### Reading frame data

Those methods will recover the data from a struct, either for a specific register or all of them:

#### Reading a specific register

* `Frame::getRegister(size_t idx)`: returns a `uint16_t` with the register value at index `idx`
* `Frame::getCoil(size_t idx)`: returns a `bool` with the coil state at index `idx`

#### Fetching all registers

* `Frame::getRegisters()`: returns all registers values
    * To a vector: `Frame::getRegisters()` returns a `std::vector<uint16_t>`
    * To a buffer: `Frame::getRegisters(uint16_t* buf, size_t len)` writes registers values into the provided buffer & returns a `size_t` with the number of registers actually fetched
* `Frame::getCoils()`: returns all coil values
    * To a vector: `Frame::getCoils()` returns a `std::vector<bool>`
    * To a buffer: `Frame::getCoils(bool* buf, size_t len)` writes coil states into the provided buffer & returns a `size_t` with the number of coils actually fetched

#### Examples

```cpp
// REGISTERS EXAMPLES

Modbus::Frame registerResponse;

// Get register value at index 4
// (returns 0 if the index is out of bounds -> make sure the index is valid!)
uint16_t regValue = registerResponse.getRegister(4);

// Copy data into a vector
// (returns an empty vector if regCount is 0)
std::vector<uint16_t> regValues = registerResponse.getRegisters();

// Copy data into a buffer 
// (returns the number of elements actually copied)
uint16_t buffer[10];
size_t copied = registerResponse.getRegisters(buffer, (sizeof(buffer) / sizeof(buffer[0])));


// COILS EXAMPLES

Modbus::Frame coilResponse;

// Get coil value at index 2 
// (returns false if the index is out of bounds -> make sure the index is valid!)
bool coilValue = coilResponse.getCoil(2);

// Copy data into a vector
// (returns an empty vector if regCount is 0)
std::vector<bool> coilValues = coilResponse.getCoils();

// Copy data into a buffer
// (returns the number of elements actually copied)
bool buffer[10];
size_t copied = coilResponse.getCoils(buffer, (sizeof(buffer) / sizeof(buffer[0])));
```

!!! note
    Technically, for registers it is possible to read/write the `data` field directly since they are not packed, but using those methods for both registers & coils will guarantee 100% validity of the data stored in the `Frame`, so they are recommended in all cases.

### Unit conversion

EZModbus offers in the `ModbusCodec` namespace, two conversion functions to convert a `float` from/to a pair of registers encoded in IEEE 754 format.  It follows the "big-endian word, little-endian byte" (3-2-1-0) convention.

```cpp
Modbus::Frame frame; // The request/response frame
float floatVal;      // Your source/target float value

// Encode a float to Modbus registers & set request frame data
uint16_t buffer[2];
ModbusCodec::floatToRegisters(floatVal, buffer);
frame.setCoils(buffer, 2);

// Extract values from response frame & parse float value 
uint16_t buffer[2];
size_t copied = frame.getCoils(buffer, 2);
if (copied == 2) floatVal = ModbusCodec::registersToFloat(buffer);
```
