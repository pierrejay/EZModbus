# Modbus::Frame

At the heart of EZModbus is the `Modbus::Frame` structure - a representation of a Modbus message that serves as the common language between all components of the library. It is used internally by all the application components, but you may have to manipulate it directly to be able to send and receive Modbus messages using the `Client`.

!!! note
    If you're just getting started or need to perform basic read/write operations, **you don't need to work with Frame directly**. The `ModbusClient` provides simple helper methods (`read()` and `write()`) that handle Frame construction and parsing automatically. See the [Client guide](../30-how-to-guides/300-modbus-client-master.md#simple-readwrite-helpers-recommended-for-simple-use-cases) for details.
    Use the Frame-based API when you need more control over the Modbus protocol or want to work with asynchronous operations.

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

## Frame data conversion (since v1.1.5)

In addition to the raw register manipulation methods described above, EZModbus now provides high-level data conversion utilities that handle endianness and multi-word data types transparently. These are the **recommended** approach for working with complex data types.

### Overview

The conversion API allows you to work directly with common data types (`float`, `uint32_t`, `int32_t`, `uint16_t`, `int16_t`) without manually handling byte ordering or register splitting. This is especially useful when communicating with devices that store data in different endianness formats.

```cpp
// Before: Manual approach with raw registers
uint16_t buffer[2];
ModbusCodec::floatToRegisters(123.45f, buffer);  // Legacy approach
frame.setRegisters(buffer, 2);

// After: Direct conversion with endianness control
frame.clearData();  // Always clear before building a new frame
frame.setFloat(123.45f, 0, Modbus::ByteOrder::CDAB);  // New approach since v1.1.5
```

### Supported data types

| Type | Size | Description |
|------|------|-------------|
| `float` | 32-bit (2 registers) | IEEE 754 floating point |
| `uint32_t` | 32-bit (2 registers) | Unsigned integer |
| `int32_t` | 32-bit (2 registers) | Signed integer |
| `uint16_t` | 16-bit (1 register) | Unsigned short with endianness |
| `int16_t` | 16-bit (1 register) | Signed short with endianness |

### Byte ordering (Endianness)

Different Modbus devices store multi-byte data in different orders within their register tables. EZModbus supports all common formats through the `ByteOrder` enum.

**Understanding the notation:** Letters A, B, C, D represent bytes from the original value in order of significance, and the `ByteOrder` describes **how these bytes are arranged in the Modbus register table**.

For example, the 32-bit value `0x12345678` contains these bytes:

- A = `0x12` (most significant byte)  
- B = `0x34`
- C = `0x56` 
- D = `0x78` (least significant byte)

```cpp
namespace Modbus {
    enum class ByteOrder {
        // 16-bit (1 register) - 2 bytes stored in Modbus table
        AB,          // Modbus register = [A][B] - big endian storage (default)
        BA,          // Modbus register = [B][A] - little endian storage
        
        // 32-bit (2 registers) - 4 bytes stored across 2 Modbus registers
        ABCD,        // Modbus: Reg0=[A][B] Reg1=[C][D] - big endian storage (default)
        CDAB,        // Modbus: Reg0=[C][D] Reg1=[A][B] - word swap (very common)
        BADC,        // Modbus: Reg0=[B][A] Reg1=[D][C] - byte + word swap
        DCBA         // Modbus: Reg0=[D][C] Reg1=[B][A] - little endian storage
    };
}
```

**Example:** The float value `123.45f` has hex representation `0x42F6E666` (A=0x42, B=0xF6, C=0xE6, D=0x66).
This value would be **stored in the Modbus register table** as:

- `ABCD`: Register 0 = `0x42F6`, Register 1 = `0xE666` (natural order)
- `CDAB`: Register 0 = `0xE666`, Register 1 = `0x42F6` ← Very common
- `BADC`: Register 0 = `0xF642`, Register 1 = `0x66E6` 
- `DCBA`: Register 0 = `0x66E6`, Register 1 = `0xF642` (fully reversed)

### Writing data with conversion

#### Setter methods signatures

```cpp
// 32-bit setters (2 registers) - return number of registers written
size_t Frame::setFloat(float value, size_t regIndex, ByteOrder order = ByteOrder::ABCD);
size_t Frame::setUint32(uint32_t value, size_t regIndex, ByteOrder order = ByteOrder::ABCD);  
size_t Frame::setInt32(int32_t value, size_t regIndex, ByteOrder order = ByteOrder::ABCD);

// 16-bit setters (1 register with endianness) - return number of registers written  
size_t Frame::setUint16(uint16_t value, size_t regIndex, ByteOrder order = ByteOrder::AB);
size_t Frame::setInt16(int16_t value, size_t regIndex, ByteOrder order = ByteOrder::AB);
```

**Return value:** All setters return the number of registers successfully written:

- `2` for 32-bit types (float, uint32_t, int32_t)
- `1` for 16-bit types (uint16_t, int16_t) 
- `0` if an error occurred (out of bounds, invalid parameters)

#### Usage example

```cpp
Modbus::Frame request;
request.clearData();  // ⚠️ Good practice: ALWAYS clear before setting data

// Set different data types with appropriate byte ordering
size_t regsUsed = 0;
regsUsed += request.setFloat(123.45f, 0, Modbus::ByteOrder::CDAB);    // 2 registers
regsUsed += request.setUint32(67890, 2, Modbus::ByteOrder::CDAB);     // 2 registers  
regsUsed += request.setInt16(999, 4, Modbus::ByteOrder::AB);          // 1 register
// Total: 5 registers used, regCount automatically set to 5

if (regsUsed != 5) {
    // Handle error - frame construction failed
}
```

#### Important notes for setters

- Good practice: **always call `clearData()` before setting data**
- `regCount` is automatically updated to accommodate the data: `regCount = max(regCount, regIndex + nbRegisters)`
- **Risk of overwriting**: If you call setters on overlapping register ranges, data may be overwritten silently

### Reading data with conversion

#### Getter methods signatures

```cpp
// 32-bit getters (2 registers) - return bool success, value by reference
// order arg defaults to ByteOrder::ABCD
bool Frame::getFloat(float& target, size_t regIndex, ByteOrder order) const;
bool Frame::getUint32(uint32_t& target, size_t regIndex, ByteOrder order) const;
bool Frame::getInt32(int32_t& target, size_t regIndex, ByteOrder order) const;

// 16-bit getters (1 register with endianness) - return bool success, value by reference
// order arg defaults to ByteOrder::AB
bool Frame::getUint16(uint16_t& target, size_t regIndex, ByteOrder order) const;
bool Frame::getInt16(int16_t& target, size_t regIndex, ByteOrder order) const;
```

**Return value:** All getters return a boolean indicating success:

- `true`: Value successfully extracted and written to the `result` parameter
- `false`: Error occurred (insufficient data, invalid register index, etc.)

**Parameters:**

- `target`: Reference to store the extracted value (only modified on success)
- `regIndex`: Starting register index (0-based)
- `order`: Byte ordering format (defaults to big endian)

#### Usage example

```cpp
Modbus::Frame response; // Received from server

float voltage;
uint32_t current;
int16_t status;

// Extract data with proper byte ordering
bool success = true;
success &= response.getFloat(voltage, 0, Modbus::ByteOrder::CDAB);
success &= response.getUint32(current, 2, Modbus::ByteOrder::CDAB);
success &= response.getInt16(status, 4, Modbus::ByteOrder::AB);

if (success) {
    // All values extracted successfully
    processData(voltage, current, status);
} else {
    // Parsing error - insufficient data or invalid register indices
}
```

### Relationship with raw data methods

The conversion API complements but doesn't replace the raw `Modbus::Frame` data read/write methods:

- **Use raw methods** (`setRegisters`, `getRegisters`, etc.) for raw register manipulation with default Modbus endianness (usually OK in most cases where you are dealing with individual registers)
- **Use conversion API** (`setFloat`, `getUint32`, etc.) when working with typed data and known endianness

It's best to avoid mixing both approaches, but if needed, start by using the raw methods first, then override the desired registers sets with the conversion API.
