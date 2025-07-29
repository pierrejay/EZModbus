# Examples

## Arduino/PlatformIO

5 basic examples are available for ESP32 Arduino core framework on PlatformIO:

* **`/client`**: Modbus RTU & TCP clients (2 examples)
* **`/server`**: Modbus RTU & TCP servers (2 examples)
* **`/bridge`**: Modbus RTU ↔ TCP bridge

To run them, replace the content of your `main.cpp` file by the provided ones. Normally, if you have already imported the library in your PlatformIO project repository (`/lib` directory), you don't need to modify your `platformio.ini` file.

## ESP-IDF

The same examples as PlatformIO are available for IDF, with specific folders for each example :&#x20;

* **`/rtu-client`**: Modbus RTU client
* **`/rtu-server`**: Modbus RTU server
* **`/tcp-client`**: Modbus TCP client
* **`/tcp-server`**: Modbus TCP server
* **`/bridge`**: Modbus RTU ↔ TCP bridge

To run them, create an IDF project using files provided in the example (`CMakeLists.txt`, `main.cpp` & `WiFiSTA.hpp` helper class for TCP examples). You need to have imported the EZModbus library in your project's `components` first (see the [Installation](../10-getting-started/101-installation.md) section of this guide).

An additional example is available in `/rtu-client-server-loopback`: it runs an RTU `Client` & an RTU `Server` that communicate together between 2 different UART ports. It will allow you to test the basic features of the lib & communication, without any external Modbus device/app. To run this example, connect the two serial ports together in "loopback" mode, default pins (ESP32-S3):

```
RX1 (Pin 44) ←→ TX2 (Pin 43)
TX1 (Pin 7)  ←→ RX2 (Pin 6)
```

!!! note
    TCP examples use a Wi-Fi connection (STA) to enable networking. If you want to run the examples with your own Ethernet connection instead, just remove the Wi-Fi code and initialize your Ethernet driver before the calls to EZModbus components.

    EZModbus relies on the ESP32's TCP/IP stack, it is agnostic regarding the link used (Wi-Fi, Ethernet, PPP...). TCP frames will be forwarded behind the scenes between your network interface & EZModbus through LwIP.

## Pico SDK

3 examples are available for RP2040/RP2350 with both RTU (UART) and TCP (CH9120 Ethernet module) support:

* **`rtu_loopback_main.cpp`**: RTU Client ↔ Server loopback test using 2 UART ports
* **`tcp_server_main.cpp`**: Modbus TCP server via CH9120 (with Python test client)
* **`tcp_client_main.cpp`**: Modbus TCP client via CH9120 (with Python test server)

### Hardware Requirements

**RTU loopback example** works on all Pico variants with these default UART pins:

**Pico**:
```
UART0 RX (Pin 16) ←→ UART1 TX (Pin 5)
UART0 TX (Pin 17) ←→ UART1 RX (Pin 4)
```

**Pico2**:
```
UART0 RX (Pin 0) ←→ UART1 TX (Pin 8)
UART0 TX (Pin 1) ←→ UART1 RX (Pin 9)
```

**Pico W / Pico2 W**:
```
UART0 RX (Pin 13) ←→ UART1 TX (Pin 8)
UART0 TX (Pin 12) ←→ UART1 RX (Pin 9)
```

**TCP examples** use the [Waveshare RP2350-ETH](https://www.waveshare.com/rp2350-eth.htm) dev board with default pinout:

```
Pico Pin | CH9120 Pin | Function
---------|------------|----------
GP20     | RX         | UART1 TX
GP21     | TX         | UART1 RX
GP18     | CFG        | Config mode
GP19     | RES        | Reset
GP17     | STATUS     | Connection status
```

### Building Examples

1. **Configure build**:
   ```bash
   cd examples/pico-sdk
   mkdir build && cd build
   cmake -G Ninja ..
   ```

2. **Choose example** (edit `CMakeLists.txt` and uncomment ONE):
   ```cmake
   # add_executable(ezmodbus_examples rtu_loopback_main.cpp)
   # add_executable(ezmodbus_examples tcp_server_main.cpp)
   add_executable(ezmodbus_examples tcp_client_main.cpp)
   ```

3. **Build**:
   ```bash
   ninja
   ```

The TCP examples include Python utilities for testing:

* **`modbus_echo_server.py`**: Test server for client example
* **`modbus_client_tester.py`**: Automated test suite for server example

**Procedure**:
```bash
# Install dependencies
pip3 install pymodbus

# Test TCP SERVER
# 1. Flash tcp_server_main.cpp on Pico
# 2. Run automated tests
python3 modbus_client_tester.py

# - or -

# Test TCP CLIENT
# 1. Start echo server
python3 modbus_echo_server.py
# 2. Flash tcp_client_main.cpp on Pico (connects automatically)
```

See the [complete README](https://github.com/pierrejay/EZModbus/tree/main/examples/pico-sdk) for details.

## STM32

*Examples coming soon...*
