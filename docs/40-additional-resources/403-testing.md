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

**Server tests:**

* Word storage
* Request & exception handling
* ~40 test cases

**Client tests:**

* Sync & async operations
* Timeout handling, error recovery
* ~40 test cases

**Bridge tests**

* Still quite basic, but checks bridge initialization & round-trip operation

For more details on the testing methodology, see the `test/` directory of the lib.

## Running tests with PlatformIO

The `platformio.ini` file is configured with one environment for each test.

It requires [pioarduino](https://github.com/pioarduino)'s `espressif32` platform to be installed before running the tests  (except `test_codec` which is compiled & run on the host machine), and the ESP32 board to be properly wired & connected via USB to the computer.

```bash
# Run individual tests
pio test -e test_codec
pio test -e test_rtu_client_loopback
pio test -e test_rtu_server_loopback
pio test -e test_tcp_client_loopback
pio test -e test_bridge_loopback

# Run the full test suite
pio test
```
