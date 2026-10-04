# Installation

## Dependencies

### Hardware

Works with all ESP32 series chip (classic, S2, S3, C3...), single or dual-core.

**Communication devices:**

* For Modbus RTU, you need an UART to RS485 transceiver such as MAX485, TD321D485, SN75176... (usually, RXD/TXD pins + optional DE pin for flow control connected to a GPIO)
* For Modbus TCP, as the library is based on the ESP32's TCP stack, it will work with any available network interface. Use either WiFi or any Ethernet chip/module such as W5500 (must be properly initialized prior to create the TCP instance)

### Environment

* ESP-IDF v5.4+ (CI-tested on v5.4, v5.5 and v6.1 - see [Testing](../40-additional-resources/403-testing.md))
* or ESP32 Arduino Core (tested on v3.0+) & PlatformIO - see [pioarduino](https://github.com/pioarduino/platform-espressif32/tree/55.03.39)
* Requires C++17 or up (normally already enabled in ESP-IDF & PlatformIO)

## Installation

### Component Manager installation

ESP-IDF 6.x is supported starting with EZModbus 1.1.9.

* Add the dependency to your project's `idf_component.yml`:

    ```yaml
    dependencies:
      pierrejay/ezmodbus: "^1.1.9"
    ```

* Build your project - the component will be downloaded automatically:

    ```bash
    idf.py build
    ```

* Include in your code:

    ```cpp
    #include "EZModbus.h"
    ```

### Manual installation (alternative)

* Clone the repo into the `components` folder in your project - **its name should be in lowercase to match the ESP-IDF component naming**:

    ```bash
    cd /path/to/your/project/components
    git clone https://github.com/pierrejay/EZModbus.git ezmodbus
    ```

* Declare the dependency in your application's component `CMakeLists.txt` (normally `main/CMakeLists.txt`):

    ```cmake
    idf_component_register(SRCS "main.cpp"
                           INCLUDE_DIRS "."
                           PRIV_REQUIRES ezmodbus)
    ```

* Include in your code:

    ```cpp
    #include "EZModbus.h"
    ```

### PlatformIO Installation

* Clone the repo into your `lib` folder
* In your main sketch, include:

```cpp
#include <EZModbus.h>
```
