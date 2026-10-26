# Testing

EZModbus includes a comprehensive suite of `Unity`-based unit and integration tests that run on both native environments and real ESP32 hardware. The testing approach follows a progressive validation strategy:

1. **Codec Tests (native)** - Validate encoding/decoding & frame data conversion functions in a desktop environment
2. **Server Tests with Mock client (Arduino ESP32)** - Validate register storage and request handling
3. **Full Client/Server Tests (Arduino ESP32)** - Complete full round-trip tests with the actual EZModbus classes

For the two latter steps, we used an ESP32-S3 dev board connected in “loopback mode”:

* Server and Client tasks are allocated on two different cores for truly parallel operation
* For Modbus RTU, two physical UART interfaces are used and linked together (RX1 to TX2 and vice-versa)
* For Modbus TCP, we use the internal loopback from LwIP (127.0.0.1) so that both server and client running on the same board can communicate without needing to be connected to an actual LAN or WiFi AP

## Test coverage

Each test category ensures key functionality works as expected:

**Codec tests:**

* Encoding/decoding for all function codes + frame data conversion
* CRC, MBAP, reg count, error handling
* ~1500 frame tests

**RTU server tests:**

* Word storage
* Request & exception handling (malformed requests, unencodable responses)
* ~40 test cases

**RTU client tests:**

* Sync & async operations
* Timeout handling, error recovery, interface lifecycle
* ~60 test cases

**TCP tests (client & server):**

* Framing, connection identity (reused sockets), reconnection, keepalive
* Interface & HAL lifecycle
* ~60 test cases

**Bridge tests**

* Round-trip operation, gateway exceptions, lifecycle
* Client request engine concurrency (timeouts, TX results, synchronous fallback)
* Server corner cases on dummy interfaces (busy server, Word store), Logger first use
* ~20 test cases

For more details on the testing methodology, see the `test/` directory of the lib.

## Running tests with PlatformIO

The `platformio.ini` file defines host environments (`native`, `native-san`) and one environment
per hardware board (`s3` and `c6`). Each board environment runs all hardware suites;
use `-f` to select an individual suite.

It requires [pioarduino](https://github.com/pioarduino)'s `espressif32` platform to be installed before running the tests  (except `test_codec` which is compiled & run on the host machine), and the ESP32 board to be properly wired & connected via USB to the computer.

```bash
# Run codec tests on the host (plain, then under AddressSanitizer & UBSan)
pio test -e native
pio test -e native-san

# Run all hardware suites on a wired XIAO ESP32-S3 or ESP32-C6
pio test -e s3
pio test -e c6

# Run one hardware suite on a selected board
pio test -e s3 -f test_rtu_client_loopback
pio test -e c6 -f test_rtu_server_loopback
```

## ESP-IDF checks

The GitHub CI compiles the ESP-IDF examples and a smoke test app (normal & debug builds)
against the local library sources, for ESP-IDF v5.4, v5.5 & v6.1 on several chips, and runs
the codec tests (plain & under sanitizers).

The smoke test app wires clients, a server and a bridge together on one board (TCP over
127.0.0.1, RTU over two cross-wired UARTs) and checks itself. `ci/smoke.sh` builds, flashes
and runs it for a chosen ESP-IDF version in the official Docker image. See
[ci/README.md](https://github.com/pierrejay/EZModbus/blob/main/ci/README.md) for details.
