# EZModbus ESP-IDF smoke test (see ci/README.md)
cmake_minimum_required(VERSION 3.16)

# idf.py -B build-debug -DEZMODBUS_DEBUG=1 build -> library debug traces on, with their
# own sdkconfig (bigger timer task stack) so the normal build keeps the IDF defaults
if(EZMODBUS_DEBUG)
    add_compile_definitions(EZMODBUS_DEBUG)
    set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.debug")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ezmodbus_smoke)
