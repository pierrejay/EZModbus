# ------------------------------------------------------------------
#  freertos/CMakeLists.txt  –  drop-in for RP2040 & RP2350
# ------------------------------------------------------------------

# 1. FreeRTOS port selection
if (PICO_BOARD MATCHES "pico2" OR PICO_BOARD MATCHES "rp2350")
    set(_freertos_port RP2350_ARM_NTZ)
else()
    set(_freertos_port RP2040)
endif()

# 2. Kernel import from SDK directory
set(FREERTOS_KERNEL_PATH ${PICO_SDK_PATH}/lib/FreeRTOS-Kernel CACHE PATH "")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/${_freertos_port}/FreeRTOS_Kernel_import.cmake)

# 3. Hooks + new/delete overload
add_library(freertos STATIC hooks.c cpp_mem.cpp)

target_include_directories(freertos PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})          # <-- contains FreeRTOSConfig.h

target_compile_features(freertos PUBLIC
        c_std_11
        cxx_std_17)

target_compile_definitions(freertos PUBLIC
        projCOVERAGE_TEST=0)

target_link_libraries(freertos PUBLIC
        FreeRTOS-Kernel
        FreeRTOS-Kernel-Heap4
        pico_stdlib)
