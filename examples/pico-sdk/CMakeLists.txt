# EZModbus Examples for Pico SDK

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ezmodbus_examples C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add FreeRTOS
add_subdirectory(freertos)

# Add EZModbus & link it with FreeRTOS
add_subdirectory(../../lib/EZModbus ezmodbus)
target_link_libraries(ezmodbus PUBLIC freertos)

# Enable debug and/or EventBus (optional)
target_compile_definitions(ezmodbus PUBLIC EZMODBUS_EVENTBUS) # Enable EventBus
# target_compile_definitions(ezmodbus PUBLIC EZMODBUS_DEBUG)    # Enable debug output

# Add executable. Choose which one to build by commenting/uncommenting:

# RTU Loopback Test (original)
add_executable(ezmodbus_examples rtu_loopback_main.cpp)

# TCP Server using CH9120
# add_executable(ezmodbus_examples tcp_server_main.cpp)

# TCP Client using CH9120
# add_executable(ezmodbus_examples tcp_client_main.cpp)

pico_set_program_name(ezmodbus_examples "ezmodbus_examples")
pico_set_program_version(ezmodbus_examples "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ezmodbus_examples 0)
pico_enable_stdio_usb(ezmodbus_examples 1)

# Add the standard library to the build
target_link_libraries(ezmodbus_examples
        pico_stdlib
        freertos    # link FreeRTOS
        ezmodbus)   # link EZModbus

# Add the standard include files to the build
target_include_directories(ezmodbus_examples PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(ezmodbus_examples)