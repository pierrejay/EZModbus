# EZModbus Examples for Pico SDK

This directory contains examples and test programs for EZModbus on RP2040/RP2350 with Modbus RTU over UART & Modbus TCP with CH9120 Ethernet driver.

## Organization

```
examples/pico-sdk/
├── README.md                  # This documentation
├── rtu_loopback_main.cpp      # RTU loopback test (internal UART)
├── tcp_server_main.cpp        # Modbus TCP server via CH9120
├── tcp_client_main.cpp        # Modbus TCP client via CH9120
├── modbus_echo_server.py      # Python test server for Pico client
└── modbus_client_tester.py    # Python automated tester for Pico server
```

## Network Configuration

- **Pico (RP2040/RP2350)**: `192.168.0.124`
- **PC (Python test tools)**: `192.168.0.234`
- **Test port**: `5020` (avoids sudo requirement)

## Available Examples

### 1. RTU Loopback Test (`rtu_loopback_main.cpp`)

**Purpose**: Test local Modbus RTU communication (UART loopback)

**Hardware**: None required (uses 2 UART ports chained together)

**Tested features**:
- Client ↔ Server RTU communication
- Register read/write operations
- Timeout handling

**Build this example**: Edit CMakeLists.txt and uncomment:
```cmake
add_executable(ezmodbus_examples rtu_loopback_main.cpp)
```

### 2. TCP Server Example (`tcp_server_main.cpp`)

**Purpose**: Modbus TCP server exposing registers via CH9120 Ethernet

**Hardware**: CH9120 connected to UART1 (pins GP20/21)

**Features**:
- TCP server on port 5020
- 10 holding registers (addresses 100-109)
- Initial values: 1000, 1001, 1002...
- EventBus logging for incoming requests

**Test with Python client**:
```bash
# Install dependencies
pip3 install pymodbus

# Run tester
python3 modbus_client_tester.py
```

**Build this example**: Edit CMakeLists.txt and uncomment:
```cmake
add_executable(ezmodbus_examples tcp_server_main.cpp)
```

### 3. TCP Client Example (`tcp_client_main.cpp`)

**Purpose**: Modbus TCP client sending requests via CH9120 Ethernet

**Hardware**: CH9120 connected to UART1 (pins GP20/21)

**Features**:
- TCP client connecting to 192.168.0.234:5020
- Single register read/write tests
- Multiple register read/write tests
- Write verification
- Automatic 5-second test cycles

**Test with Python server**:
```bash
# Terminal 1: Python echo server
pip3 install pymodbus  # If not already installed
python3 modbus_echo_server.py

# Terminal 2: Flash client on Pico
# Client connects automatically
```

**Build this example**: Edit CMakeLists.txt and uncomment:
```cmake
add_executable(ezmodbus_examples tcp_client_main.cpp)
```

## Python Test Tools

### Prerequisites

Install Python dependencies:
```bash
pip3 install pymodbus
```

### `modbus_echo_server.py`

Modbus TCP test server that echoes operations for client testing.

**Usage**:
```bash
python3 modbus_echo_server.py
```

**Features**:
- Listens on 192.168.0.234:5020
- 200 holding registers (0-199)
- Initial values: 1000, 1001, 1002...
- Detailed logging of all READ/WRITE operations

### `modbus_client_tester.py`

Automated test suite for Pico TCP server validation.

**Usage**:
```bash
# First, flash tcp_server_main.cpp on Pico
python3 modbus_client_tester.py
```

**Automated tests**:
- ✅ **TCP Connection** to 192.168.0.124:5020
- ✅ **Single register read** (register 100)
- ✅ **Multiple register read** (5 registers from 100)
- ✅ **Single register write** + verification
- ✅ **Multiple register write** + verification
- ✅ **Write-Read-Verify cycle**
- ✅ **Boundary address tests** (first/last register)
- ✅ **Invalid address test** (should fail gracefully)

**Sample output**:
```
✅ Single Register Read - Addr 100: 1000
✅ Multiple Register Read - Addr 100-104: [1000, 1001, 1002, 1003, 1004]
✅ Single Register Write - Addr 100: 1100
✅ Write-Read-Verify - Value 9999 verified
✅ Invalid Address Test - Correctly rejected address 999

TEST SUMMARY
============
Total tests: 8
Passed: 8
Failed: 0
```

## Complete Testing Process

### Building Examples

1. **Configure build**:
   ```bash
   cd examples/pico-sdk
   mkdir build && cd build
   cmake -G Ninja ..
   ```

2. **Choose example to build** (edit CMakeLists.txt):
   ```cmake
   # Uncomment ONE of:
   # add_executable(ezmodbus_examples rtu_loopback_main.cpp)
   # add_executable(ezmodbus_examples tcp_server_main.cpp)
   add_executable(ezmodbus_examples tcp_client_main.cpp)
   ```

3. **Build**:
   ```bash
   ninja
   ```

### Client → Server Test

1. **Start Python echo server**:
   ```bash
   python3 modbus_echo_server.py
   ```

2. **Build and flash client** (see Building Examples above)

3. **Observe logs** on both sides

### Server ← Client Test

1. **Build and flash server** (edit CMakeLists.txt, choose tcp_server_main.cpp)

2. **Run Python test suite**:
   ```bash
   python3 modbus_client_tester.py
   ```

3. **Verify all tests pass**

## CH9120 Pinout (Waveshare Board)

```
Pico Pin | CH9120 Pin | Function
---------|------------|----------
GP20     | RX         | UART1 TX
GP21     | TX         | UART1 RX
GP18     | CFG        | Config mode
GP19     | RES        | Reset
GP17     | STATUS     | Connection status
```

## Debugging

### Python server logs
```
2025-07-23 22:44:35,294 INFO [ModbusEchoServer] READ: Address=10, Count=1, Values=[1010]
2025-07-23 22:44:35,391 INFO [ModbusEchoServer] WRITE: Address=10, Values=[1110]
```

### Pico client logs
```
[MODBUS_TCP_CLIENT] Read single register: value = 1010 (0x03F2)
[MODBUS_TCP_CLIENT] Write single register: SUCCESS (Echo: 1110)
[MODBUS_TCP_CLIENT] Readback single register: value = 1110 (0x0456) - VERIFIED
```

### Common Issues

1. **Permission denied port 502**: Use port 5020 instead
2. **Connection refused**: Check IPs and ensure server is listening
3. **CH9120 not responding**: Verify wiring and power supply
4. **Compilation failed**: Check that `examples/pico-sdk/` paths are correct
5. **Client timeout too short**: Client request timeout is 1 second by default. For longer operations, create client with custom timeout: `Modbus::Client modbusClient(tcpInterface, 60000);` (60 seconds)

## Architecture Notes

### Why Examples Instead of Pure Tests?

Unlike ESP32 which supports pure loopback testing using internal TCP/IP stack, the RP2040/RP2350 platform requires external Ethernet hardware (CH9120) for TCP communication. This means:

- **RTU examples** can be pure loopback (internal UART)
- **TCP examples** require external hardware and network setup
- **Python tools** are needed to provide the "other end" of TCP communication

These examples serve as both **functional demonstrations** and **validation tests** for the EZModbus TCP implementation on Pico.

### Validated Architecture

The EZModbus + CH9120Driver integration is **100% functional** with:

- ✅ **Unified API** - Same interface across ESP32/RP2040 platforms
- ✅ **Platform conditionals** - Clean `.inl` file pattern
- ✅ **Clean abstraction** - Application code doesn't expose CH9120Driver
- ✅ **Real-time performance** - No communication issues
- ✅ **Robust error handling** - Complete exception management
- ✅ **Comprehensive testing** - Bidirectional validation

**Professional, KISS implementation ready for production! 🚀**