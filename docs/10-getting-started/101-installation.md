---
hidden: true
---

# Installation (WIP)

## Prerequisites

### Target MCUs & SDKs

* **Espressif ESP32** series chips, single or dual-core : tested with ESP32-S3
    * ESP-IDF (tested on v5.4)
    * ESP32 Arduino Core (tested on v3.0+) & PlatformIO - see [pioarduino](https://github.com/pioarduino/platform-espressif32)

* **Raspberry Pi RP2040 & RP2350**: tested with RPi Pico & RPi Pico 2
    * Pico-SDK (tested on v2.1.1)
  
* **STMicroelectronics STM32** series chips: tested with STM32G0B1 & STM32H523
    * STM32CubeMX (tested on v6.14.1) - used to generate the CMake project

Normally, any IDE supporting CMake should work (except for Arduino). For the 3 chip families, building the library was tested on VSCode with the official vendor extension. Example projects are provided in the [Examples](../40-additional-resources/400-examples.md) section.

### Hardware peripherals

* For **Modbus RTU**, you need an UART to RS485 transceiver such as MAX485, TD321D485, SN75176... (usually, RXD/TXD pins + optional DE pin for flow control connected to a GPIO)
* For **Modbus TCP**: 
    * **ESP32**: as the library is based on the ESP32's native networking stack, it will work with any available network interface. You can use either WiFi, PPP, or supported Ethernet chips/modules such as W5500 (the Ethernet driver must be properly initialized prior to create the TCP instance)
    * **RP2040/RP2350**: currently only compatible with CH9120 UART-to-Ethernet module, like on the [Waveshare RP2350-ETH](https://www.waveshare.com/rp2350-eth.htm) dev board. Work is currently in progress to support LwIP TCP/IP stack as well as the WizNet W5500 Ethernet chip (through LwIP, or standalone in "offload mode").
    * **STM32**: the current version does not support Modbus TCP on ST chips out-of-the-box. Work is currently in progress to support LwIP TCP/IP stack as well as the WizNet W5500 Ethernet chip in "offload mode" (through LwIP, or standalone in "offload mode").

### Software dependencies

* **FreeRTOS**
    * Built-in for ESP32 & STM32
    * Instructions & drop-in config files provided for RPi chips (see below)
* Requires **C++17** or up: normally already default except for STM32 - must be set manually (see below)

## ESP32 - Arduino/PlatformIO installation

* Clone the repo into your `lib` folder
* In your main sketch, include:

```cpp
#include <EZModbus.h>
```

## ESP32 - ESP-IDF installation

### Component Manager installation

* Add the dependency to your project's `idf_component.yml`:

    ```yaml
    dependencies:
      pierrejay/EZModbus: "^1.2.0"
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

* Include the component in your project's root `CMakeLists.txt`:

    ```cmake
    idf_component_register(SRCS "main.cpp" PRIV_REQUIRES ezmodbus ... # <- add here INCLUDE_DIRS "")
    ```

* Include in your code:

    ```cpp
    #include "EZModbus.h"
    ```

## RP2040 & RP2350 - Pico SDK installation

### FreeRTOS installation

The official documentation for Pico SDK is not crystal clear about how to install FreeRTOS, so we're giving some guidance here. Follow this link for a guide that will explain how you can get running in under one minute:

[https://github.com/pierrejay/pico-freertos-setup](https://github.com/pierrejay/pico-freertos-setup)

The EZModbus example contained in `examples/pico-sdk` already includes the `freertos` local configuration directory (configuration header file, hooks & `CMakeLists.txt` file for compilation) that is mentioned in the guide, so basically if you have followed the link above, you can run the examples as-is - they are standalone Pico projects.

### EZModbus installation

* Clone the EZModbus directory in your project folder & initialize its submodules

!!! note
    Since the EZModbus RPi Pico port depends on two external libraries (`UartDmaDriver` & `CH9120Driver`) included as submodules, you need to clone the EZModbus repository with the `--recurse-submodules` flag, otherwise their content won't be fetched:
    
    ```bash
    git clone --recurse-submodules https://github.com/pierrejay/EZModbus.git
    ```
    or

    ```bash
    git clone https://github.com/pierrejay/EZModbus.git
    cd EZModbus
    git submodule update --init --recursive
    ```

* If you followed the guide above to install FreeRTOS, make sure the `freertos` configuration folder is located in your project's root (it should be at the same level than `main.cpp`)
* Adapt your main project's `CMakeLists.txt` file. Here, we show the default file from a blank Pico-SDK project so you can see exactly where the required sections should be added/edited.

```cmake
# ...existing CMakeLists.txt in project's root directory

# Initialise the Raspberry Pi Pico SDK (existing section)
pico_sdk_init()


##### ADD THIS SECTION HERE #####
add_subdirectory(freertos) # Add FreeRTOS
add_subdirectory(EZModbus ezmodbus) # Add EZModbus
target_link_libraries(ezmodbus PUBLIC freertos) # Link it with FreeRTOS

# Enable debug and/or EventBus (optional, see docs > Logging/debugging section)
# target_compile_definitions(ezmodbus PUBLIC EZMODBUS_EVENTBUS) # Enable EventBus
# target_compile_definitions(ezmodbus PUBLIC EZMODBUS_DEBUG)    # Enable debug output
######### END OF SECTION #########


# Add executable. Default name is the project name, version 0.1
add_executable(YourProjectName main.cpp)

pico_set_program_name(YourProjectName "YourProjectName")
pico_set_program_version(YourProjectName "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(YourProjectName 0)
pico_enable_stdio_usb(YourProjectName 1)


####### EDIT THIS SECTION #######
target_link_libraries(YourProjectName
    pico_stdlib
    # Other libraries...
    freertos        # <- Add FreeRTOS
    ezmodbus        # <- Add EZModbus
)
######### END OF SECTION #########

pico_add_extra_outputs(YourProjectName)
```

* Include in your `main.cpp`:

```cpp
#include <stdio.h>         // Existing includes
#include "pico/stdlib.h"   // Existing includes
#include "FreeRTOS.h"
#include "EZModbus.h"
```

## STM32 installation

In order to be as generic as possible, the library assumes you created the project through STM32CubeMX, where "middlewares" (FreeRTOS) & peripherals (UART/IRQ/DMA/SysTick) are configured. The compatibility of EZModbus with all chips is not guaranteed, but it should work with most of the mainstream chip lines: STM32G0B1 & H523 were tested, and the `CMakeLists.txt` automatically recognizes which chip should be used to properly configure the lib.

### STM32CubeMX configuration

!!! note
    The configuration slighly differs depending on your chip model. The guide above refers to STM32G0. For STM32H5, DMA settings are found in `System Core > GPDMAx` , and FreeRTOS requires the `CMSIS-RTOS2 API` to be enabled. In doubt, check the two example CubeMX files provided for G0 & H5.

* Enable USART peripherals on your RS485 pins (& logging console)
    * Mode: Asynchronous, Hardware Flow Control disabled
    * Tab `Parameter settings`: set your Modbus com settings (baud rate, parity, stop bits...)
    * Tab `DMA Settings`: enable DMA in both directions with the correct settings (Peripheral <> Memory depending on direction, Priority: low)
    * Tab `NVIC Settings`: enable global interrupt
* Enable GPIO Output on your RE/DE pin if used
* Enable FreeRTOS (Middleware section)
* Change your timebase source in `System Core > SYS` (use a free TIM instead of `SysTick` - this is strongly recommended when using an RTOS)
* Chose "CMake target IDE"
* Create the project

### Transform the project from C to C++

If you are starting from scratch, you may need to adapt the generated files for C++, as CubeMX creates a C project by default. Open the project in your preferred IDE and follow those steps:

* Rename `main.c` to `main.cpp`
* Edit `cmake/stm32cubemx/CMakeLists.txt` : replace `main.c` by `main.cpp`
* Edit the CMakeLists.txt at the root of your project folder:

```cmake
#...CMakeLists.txt generated by CubeMX

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

##### ADD THIS SECTION HERE #####
set(CMAKE_CXX_STANDARD 20) # <- For C++20. EZModbus requires C++17 or up
set(CMAKE_CXX_STANDARD_REQUIRED ON)
######### END OF SECTION #########

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME YourProjectName)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

###### MOVE THIS SECTION HERE AFTER project() #####
# Already exists, but this will suppress CMake warning
# Enable CMake support for ASM and C languages
enable_language(C ASM)
################# END OF SECTION ##################

# File continues...

```

* Depending on your chip type / default CubeMX code, there will be a few quirks to solve to suppress compiler error/warnings, for example:
    * On H5: set `extern "C" MX_FREERTOS_Init();`  (or comment out the call if you're doing everything in `main.cpp`)
    * Remove calls to default CMSIS primitives (`defaultTaskHandle` & `defaultTaskHandle_attributes` declarations, `osThreadNew()` call), they are useless and will cause warnings due to improper initialization order in the default generated code.
    * Basically, try and build the baseline `main.cpp` until you have solved all compiler errors, it shouldn't take too many steps.

### Check FreeRTOS configuration

In `Inc/FreeRTOSConfig.h`, check/set the following settings:

```c
// Default settings
#define configSUPPORT_STATIC_ALLOCATION    1               // <- should be 1
#define configTOTAL_HEAP_SIZE              ((size_t)8192)  // <- should be enough for your own tasks (here 8k words = 32KB)
#define configTIMER_TASK_STACK_DEPTH       512             // <- increase to >=512 words if enabling Debug mode

// Add in "USER CODE" section (if absent)
#define configUSE_QUEUE_SETS               1               // <- enable queue sets (required for EZModbus)
```

Reminder: EZModbus uses statically-allocated FreeRTOS objects, so you don't need to account for its tasks to calculate the required total heap size.

### Implement EZModbus

* Clone the `EZModbus` library folder to the root of your project
* Adapt your project's root `CMakeLists.txt` to link EZModbus

```cmake
#...existing CMakeLists.txt

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

##### ADD THIS SECTION HERE #####
add_subdirectory(EZModbus)
target_compile_definitions(EZModbus PUBLIC STM32H523xx) # <- use your chip ref
######## END OF SECTION #########

##### EDIT THIS SECTION #####
# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    #...other libraries
    EZModbus # <- Add EZModbus here
)
####### END OF SECTION #######
```

!!! note
    **Very important**: your chip model in compile definitions should be written exactly like in this example snippet, **including uppercase & lowercase characters,** for EZModbus to correctly recognize which ST HAL files to use! For instance :

    - `STM32H523xx`, `STM32G0B1xx` -> correct
    - `STM32H523`, `STM32H523XX`, `STM32H5xx`... -> won't work!

* Add EZModbus include to your `main.cpp`

<pre class="language-cpp"><code class="lang-cpp"><strong>#include "EZModbus.h"
</strong></code></pre>

That's it! Now implement your EZModbus instances & tasks and build the project. There are many resources about FreeRTOS on STM32 on Internet & YouTube so we won't go further here, but you can check the provided examples to get some inspiration.
