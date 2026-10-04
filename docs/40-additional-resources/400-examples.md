# Examples

## Arduino/PlatformIO

5 basic examples are available for ESP32 Arduino core framework on PlatformIO:

* **`/client`**: Modbus RTU & TCP clients (2 examples)
* **`/server`**: Modbus RTU & TCP servers (2 examples)
* **`/bridge`**: Modbus RTU ↔ TCP bridge

To run them, replace the content of your `main.cpp` file by the provided ones. Normally, if you have already imported the library in your PlatformIO project repository (`/lib` directory), you don't need to modify your `platformio.ini` file.

## ESP-IDF

The same examples as PlatformIO are available for IDF, with specific folders for each example :

* **`/rtu-client`**: Modbus RTU client
* **`/rtu-server`**: Modbus RTU server
* **`/tcp-client`**: Modbus TCP client
* **`/tcp-server`**: Modbus TCP server
* **`/bridge`**: Modbus RTU ↔ TCP bridge

To run them, create an IDF project using files provided in the example (`CMakeLists.txt`, `main.cpp` & `WiFiSTA.hpp` helper class for TCP examples). You need to have imported the EZModbus library in your project's `components` first (see the [Installation](../10-getting-started/101-installation.md) section of this guide).

An additional example is available in `/rtu-client-server-loopback`: it runs an RTU `Client` & an RTU `Server` that communicate together between 2 different UART ports. It uses `UART_NUM_1` and `UART_NUM_2`, so the supplied example builds on ESP32, ESP32-S3 and ESP32-P4. ESP32-C6's LP-UART requires a different configuration and is not used by this example. It will allow you to test the basic features of the lib & communication, without any external Modbus device/app. To run this example, connect the two serial ports together in "loopback" mode, default pins (ESP32-S3 and ESP32-P4):

```
RX1 (Pin 44) ←→ TX2 (Pin 43)
TX1 (Pin 7)  ←→ RX2 (Pin 6)
```

On the classic ESP32, connect GPIO16 to GPIO19 and GPIO17 to GPIO18 instead.

!!! note
    TCP examples use a Wi-Fi connection (STA) to enable networking. If you want to run the examples with your own Ethernet connection instead, just remove the Wi-Fi code and initialize your Ethernet driver before the calls to EZModbus components.

    The supplied Wi-Fi examples (`tcp-client`, `tcp-server`, `bridge`) are skipped by the compile checks on targets without native Wi-Fi, including ESP32-P4.

    EZModbus relies on the ESP32's TCP/IP stack, it is agnostic regarding the link used (Wi-Fi, Ethernet, PPP...). TCP frames will be forwarded behind the scenes between your network interface & EZModbus through LwIP.
