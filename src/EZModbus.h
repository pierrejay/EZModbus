/**
 * @file EZModbus.h
 * @brief Main include file for the EZModbus library
 */

#pragma once

// Core components
#include "core/ModbusCore.h"
#include "core/ModbusCodec.hpp"
#include "interfaces/ModbusInterface.hpp"

// Drivers
#include "drivers/ModbusHAL_UART.h"
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM) || defined(PICO_SDK)
    #include "drivers/ModbusHAL_TCP.h" // TCP available on ESP32 and Pico (with CH9120)
#endif

// Interfaces
#include "interfaces/ModbusRTU.h"
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM) || defined(PICO_SDK)
    #include "interfaces/ModbusTCP.h" // TCP available on ESP32 and Pico (with CH9120)
#endif

// Application components
#include "apps/ModbusClient.h"
#include "apps/ModbusServer.h"
#include "apps/ModbusBridge.hpp"