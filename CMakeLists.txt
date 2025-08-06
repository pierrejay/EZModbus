###############################################################################
# EZModbus – CMakeLists.txt
#  • ESP-IDF                    : auto-detection via IDF_TARGET
#  • Pico SDK (RP2040 & RP2350) : pico_stdlib present or PICO_BOARD defined
#  • STM32 (fallback)           : everything else
#       ──→ scan CubeMX, macro CPU from startup_*.s, FreeRTOS auto-include
###############################################################################
cmake_minimum_required(VERSION 3.15)

# Only call project() when NOT in ESP-IDF context
if(NOT DEFINED IDF_TARGET)
    project(EZModbus LANGUAGES C CXX)
endif()

# ---------------------------------------------------------------------------
# 1. Configuration variables
# ---------------------------------------------------------------------------
# Users can set these variables before calling add_subdirectory() to override defaults
# Default values are defined in the respective header files

# ---------------------------------------------------------------------------
# 2. Sources & includes communs
# ---------------------------------------------------------------------------
set(EZMODBUS_SRCS_BASE
    ${CMAKE_CURRENT_LIST_DIR}/src/apps/ModbusClient.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/apps/ModbusServer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/drivers/ModbusHAL_UART.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/interfaces/ModbusRTU.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/utils/ModbusDebug.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/utils/ModbusEventBus.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/utils/ModbusLogSink.cpp
)

set(EZMODBUS_SRCS_TCP
    ${CMAKE_CURRENT_LIST_DIR}/src/drivers/ModbusHAL_TCP.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/interfaces/ModbusTCP.cpp
)

# Default: include all sources
set(EZMODBUS_SRCS ${EZMODBUS_SRCS_BASE} ${EZMODBUS_SRCS_TCP})

set(EZMODBUS_INC
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/src/apps
    ${CMAKE_CURRENT_LIST_DIR}/src/core
    ${CMAKE_CURRENT_LIST_DIR}/src/drivers
    ${CMAKE_CURRENT_LIST_DIR}/src/interfaces
    ${CMAKE_CURRENT_LIST_DIR}/src/utils
)

# ---------------------------------------------------------------------------
# 3. Helper function to apply configuration
# ---------------------------------------------------------------------------
function(apply_ezmodbus_config TARGET_NAME)
    # Only apply settings that were explicitly set by the user
    # Default values are handled in the headers
    
    if(DEFINED EZMODBUS_SERVER_MAX_WORD_SIZE)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_SERVER_MAX_WORD_SIZE=${EZMODBUS_SERVER_MAX_WORD_SIZE})
    endif()
    if(DEFINED EZMODBUS_CLIENT_REQ_TIMEOUT)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_CLIENT_REQ_TIMEOUT=${EZMODBUS_CLIENT_REQ_TIMEOUT})
    endif()
    if(DEFINED EZMODBUS_RTU_TASK_STACK_SIZE)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_RTU_TASK_STACK_SIZE=${EZMODBUS_RTU_TASK_STACK_SIZE})
    endif()
    if(DEFINED EZMODBUS_RTU_TASK_PRIORITY)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_RTU_TASK_PRIORITY=${EZMODBUS_RTU_TASK_PRIORITY})
    endif()
    if(DEFINED EZMODBUS_TCP_TXN_SAFETY_TIMEOUT)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_TCP_TXN_SAFETY_TIMEOUT=${EZMODBUS_TCP_TXN_SAFETY_TIMEOUT})
    endif()
    if(DEFINED EZMODBUS_TCP_TASK_STACK_SIZE)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_TCP_TASK_STACK_SIZE=${EZMODBUS_TCP_TASK_STACK_SIZE})
    endif()
    if(DEFINED EZMODBUS_TCP_TASK_PRIORITY)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_TCP_TASK_PRIORITY=${EZMODBUS_TCP_TASK_PRIORITY})
    endif()
    if(DEFINED EZMODBUS_HAL_UART_EVT_Q_SIZE)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_HAL_UART_EVT_Q_SIZE=${EZMODBUS_HAL_UART_EVT_Q_SIZE})
    endif()
    if(DEFINED EZMODBUS_HAL_TCP_RX_Q_SIZE)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_HAL_TCP_RX_Q_SIZE=${EZMODBUS_HAL_TCP_RX_Q_SIZE})
    endif()
    if(DEFINED EZMODBUS_HAL_TCP_TASK_STACK_SIZE)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_HAL_TCP_TASK_STACK_SIZE=${EZMODBUS_HAL_TCP_TASK_STACK_SIZE})
    endif()
    if(DEFINED EZMODBUS_HAL_TCP_TASK_PRIORITY)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_HAL_TCP_TASK_PRIORITY=${EZMODBUS_HAL_TCP_TASK_PRIORITY})
    endif()
    if(DEFINED EZMODBUS_MAX_DEBUG_MSG_SIZE)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_MAX_DEBUG_MSG_SIZE=${EZMODBUS_MAX_DEBUG_MSG_SIZE})
    endif()
    if(DEFINED EZMODBUS_LOG_Q_SIZE)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_LOG_Q_SIZE=${EZMODBUS_LOG_Q_SIZE})
    endif()
    if(DEFINED EZMODBUS_LOG_MAX_MSG_SIZE)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_LOG_MAX_MSG_SIZE=${EZMODBUS_LOG_MAX_MSG_SIZE})
    endif()
    if(DEFINED EZMODBUS_LOG_TASK_PRIORITY)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_LOG_TASK_PRIORITY=${EZMODBUS_LOG_TASK_PRIORITY})
    endif()
    if(DEFINED EZMODBUS_LOG_TASK_STACK_SIZE)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_LOG_TASK_STACK_SIZE=${EZMODBUS_LOG_TASK_STACK_SIZE})
    endif()
    
    # Boolean flags
    if(DEFINED EZMODBUS_DEBUG)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_DEBUG)
    endif()
    if(DEFINED EZMODBUS_LOG_OUTPUT)
        target_compile_definitions(${TARGET_NAME} PRIVATE EZMODBUS_LOG_OUTPUT=${EZMODBUS_LOG_OUTPUT})
    endif()
endfunction()

# ---------------------------------------------------------------------------
# 4. Platform detection
# ---------------------------------------------------------------------------
if(DEFINED IDF_TARGET)                               # -------- ESP-IDF -----
    message(STATUS "EZModbus : ESP-IDF")

    idf_component_register(
        SRCS         "${EZMODBUS_SRCS}"
        INCLUDE_DIRS "src"
        REQUIRES     driver freertos lwip esp_timer esp_netif
    )
    
    # Apply configuration to the IDF component
    apply_ezmodbus_config(${COMPONENT_LIB})

elseif(TARGET pico_stdlib OR DEFINED PICO_BOARD)     # -------- Pico SDK -----
    message(STATUS "EZModbus : RPi Pico SDK")

    # Add UartDmaDriver as subdirectory 
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/external/pico-freertos-UartDmaDriver UartDmaDriver)
    
    # Add CH9120Driver as subdirectory  
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/external/pico-freertos-CH9120Driver CH9120Driver)

    add_library(ezmodbus STATIC ${EZMODBUS_SRCS})
    target_include_directories(ezmodbus PUBLIC ${EZMODBUS_INC})
    target_compile_definitions(ezmodbus PUBLIC PICO_SDK=1)

    target_link_libraries(ezmodbus PUBLIC
        pico_stdlib
        hardware_uart
        hardware_timer
        hardware_dma
        CH9120Driver
        UartDmaDriver  # Shared with CH9120Driver
    )

    # FreeRTOS (optional, if already defined by the project)
    if(TARGET FreeRTOS-Kernel)
        target_link_libraries(ezmodbus PUBLIC FreeRTOS-Kernel)
    endif()
    if(TARGET freertos_config)
        target_link_libraries(ezmodbus PUBLIC freertos_config)
    endif()
    
    # Apply configuration to the Pico target
    apply_ezmodbus_config(ezmodbus)

else()                                               # -------- STM32 -------
    message(STATUS "EZModbus : STM32")

    # 0) Target - STM32 uses only base sources (no TCP support)
    add_library(EZModbus STATIC ${EZMODBUS_SRCS_BASE})
    target_include_directories(EZModbus PUBLIC ${EZMODBUS_INC})

    # 1) CPU detection via startup_*.s
    file(GLOB _STARTUP_S
        "${CMAKE_SOURCE_DIR}/Core/Src/startup_*.s"
        "${CMAKE_SOURCE_DIR}/startup_*.s")
    if(_STARTUP_S)
        list(GET _STARTUP_S 0 _STARTUP_FILE)
        get_filename_component(_STARTUP_NAME "${_STARTUP_FILE}" NAME)
        string(REGEX REPLACE "startup_(.*)\\.s" "\\1" _CHIP "${_STARTUP_NAME}")
        # Ex. startup_stm32h523xx.s → STM32H523xx  (the "xx" remains in lowercase)
        string(TOUPPER "${_CHIP}" _TMP)
        string(REGEX REPLACE "XX$" "xx" _CHIP_MACRO "${_TMP}")
        message(STATUS "EZModbus : micro détecté = ${_CHIP_MACRO}")
        target_compile_definitions(EZModbus PUBLIC ${_CHIP_MACRO})
    else()
        message(WARNING
            "EZModbus : startup_*.s introuvable ; "
            "définissez -DSTM32xxxx si nécessaire.")
    endif()

    # 2) CubeMX includes : HAL / CMSIS / Core
    file(GLOB _HAL_DIR     "${CMAKE_SOURCE_DIR}/Drivers/*HAL_Driver/Inc")
    file(GLOB _DEVICE_DIR  "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/*/Include")
    set   (_CMSIS_DIR      "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include")
    set   (_CORE_DIR       "${CMAKE_SOURCE_DIR}/Core/Inc")

    target_include_directories(EZModbus PUBLIC
        ${_CORE_DIR}
        ${_HAL_DIR}
        ${_DEVICE_DIR}
        ${_CMSIS_DIR}
    )

    # 3) FreeRTOS : only headers (Cube already compiles the kernel)
    set(_FREERTOS_BASE "${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source")
    if(EXISTS "${_FREERTOS_BASE}/include")
        #--- Mapping ARM core → port directory --------------------------------
        if("${_CHIP_MACRO}" MATCHES "F0|G0|L0")                               # M0/M0+
            set(_PORT_DIR "${_FREERTOS_BASE}/portable/GCC/ARM_CM0")
        elseif("${_CHIP_MACRO}" MATCHES "F1|F2|L1")                           # M3
            set(_PORT_DIR "${_FREERTOS_BASE}/portable/GCC/ARM_CM3")
        elseif("${_CHIP_MACRO}" MATCHES "F3|F4|G4|L4|WL|WB")                  # M4/M4F
            set(_PORT_DIR "${_FREERTOS_BASE}/portable/GCC/ARM_CM4F")
        elseif("${_CHIP_MACRO}" MATCHES "F7|H7")                              # M7
            set(_PORT_DIR "${_FREERTOS_BASE}/portable/GCC/ARM_CM7/r0p1")
        elseif("${_CHIP_MACRO}" MATCHES "L5|H5|U5")                           # M33
            set(_PORT_DIR "${_FREERTOS_BASE}/portable/GCC/ARM_CM33_NTZ/non_secure")
        else()
            message(WARNING "FreeRTOS : unknown core → ARM_CM0 by default")
            set(_PORT_DIR "${_FREERTOS_BASE}/portable/GCC/ARM_CM0")
        endif()

        target_include_directories(EZModbus PUBLIC
            "${_FREERTOS_BASE}/include"
            "${_PORT_DIR}"
            "${_FREERTOS_BASE}/CMSIS_RTOS_V2"
        )
    endif()

    # 4) Generic STM32 definitions
    target_compile_definitions(EZModbus PUBLIC USE_HAL_DRIVER STM32_HAL)
    
    # Apply configuration to the STM32 target
    apply_ezmodbus_config(EZModbus)

endif()
